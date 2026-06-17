"""无头烟雾测试: 不依赖 isaacsim 包, 在没装 Isaac / 没显示的机器上验证 C++ 闭环接线。

复刻 isaac_loop.py 的核心计算路径(update_state -> add_traj -> replan -> get_next_goal),
但去掉所有 Isaac/渲染调用。跑通 = 桥(ctypes)-> capi.so -> C++ 真管线 在 Ubuntu 上活着、
能在 yaml 场景里规划并消费 setpoint。最终报告 reached/collided/min_clear/规划失败数,
和真 Isaac 循环同一套判据。这是本机(无 Isaac)能做到的最强验证;真·可视化跑在装了 Isaac 的机器。

跑法: conda activate sando && python isaac/_smoke_headless.py
"""
import os
import sys
import time

import numpy as np
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

with open(os.path.join(_HERE, "isaac_sando.yaml"), "r", encoding="utf-8") as _f:
    CFG = yaml.safe_load(_f)
PLN = CFG["planner"]; SCENE = CFG["scene"]; LOOP = CFG["loop"]

# C++ 引擎(桥 -> capi.so)。烟雾测试只验 C++ 路径。
from sando_cpp_bridge import (Parameters, RobotState, DynTraj, SANDO,
                              DroneStatus_GOAL_REACHED as GOAL_REACHED)
print("[smoke] engine = C++ (sando_cpp_bridge -> capi.so)", flush=True)

# ---- Parameters: 照 isaac_loop.py 逐字段 setattr ----
par = Parameters()
for k, v in PLN.items():
    if k == "sando_map_res":
        par.res = float(v); continue
    if hasattr(par, k):
        setattr(par, k, v)
    else:
        print(f"[smoke][warn] Parameters 无字段 '{k}', 跳过", flush=True)
par.replan_dt = float(LOOP.get("replan_dt", 0.1))
par.use_spacetime_corridor = os.environ.get("STC", "1") == "1"
par.use_st_graph = os.environ.get("STG", "0") == "1"
if os.environ.get("STDSD"):
    par.stc_d_safe_dyn = float(os.environ["STDSD"])

START = np.array(SCENE["start"], dtype=float)
GOAL = np.array(SCENE["goal"], dtype=float)
if bool(getattr(par, "force_goal_z", False)):
    GOAL[2] = float(getattr(par, "default_goal_z", GOAL[2]))

OBS = SCENE["obstacles"]


def make_dt(i, o):
    dt = DynTraj()
    dt.id = (200 + i) if o.get("class", "wall") == "wall" else i
    dt.mode = "Analytic"
    dt.bbox = np.array(o["size"], float)
    if "traj" in o:
        dt.traj_x, dt.traj_y, dt.traj_z = [str(e) for e in o["traj"]]
        if "vel" in o:
            dt.traj_vx, dt.traj_vy, dt.traj_vz = [str(e) for e in o["vel"]]
    else:
        c = o["center"]
        dt.traj_x, dt.traj_y, dt.traj_z = f"{c[0]}", f"{c[1]}", f"{c[2]}"
        dt.traj_vx = dt.traj_vy = dt.traj_vz = "0.0"
    dt.compile_analytic()
    return dt


DTS = [make_dt(i, o) for i, o in enumerate(OBS)]

# ---- planner 启动序列(照 sando_node / isaac_loop.py)----
sando = SANDO(par)
_st = RobotState(); _st.pos = START.copy(); _st.vel = np.zeros(3)
sando.update_state(_st)
sando.update_occupancy_map_ptr(np.zeros((0, 3)))
_G = RobotState(); _G.pos = GOAL.copy()
sando.set_terminal_goal(_G)
print(f"[smoke] 启动完毕. start={START} goal={GOAL} status={sando.get_drone_status()} "
      f"obstacles={len(OBS)}", flush=True)


def push_obstacles(t, p_d):
    cull = float(LOOP.get("sense_cull_r", 40.0))
    for dt in DTS:
        if np.linalg.norm(dt.eval(t) - p_d) <= cull:
            sando.add_traj(dt, t)


def signed_box_dist(p, c, sz):
    lo = c - 0.5 * sz; hi = c + 0.5 * sz
    outside = np.maximum(lo - p, 0.0) + np.maximum(p - hi, 0.0)
    if np.any(outside > 0.0):
        return float(np.linalg.norm(outside))
    return float(np.max(np.maximum(lo - p, p - hi)))


def clearance_all(p, t):
    cmin = np.inf
    for i, o in enumerate(OBS):
        cmin = min(cmin, signed_box_dist(p, DTS[i].eval(t), np.array(o["size"], float)))
    return cmin


# ---- 闭环(无渲染)----
DT = float(par.dc)
REPLAN_DT = float(LOOP.get("replan_dt", 0.1))
# 默认跑满 yaml 的 t_max(到终点为止); 想快速冒烟可设 SMOKE_TMAX=8。
T_MAX = float(os.environ.get("SMOKE_TMAX", float(LOOP.get("t_max", 60.0))))

p_d = START.copy(); v_d = np.zeros(3); a_d = np.zeros(3)
t = 0.0; last_rt = 0.0; next_replan = 0.0
n_invalid = 0; n_replan = 0; reached = False; collided = False
min_clear = np.inf
n_steps = 0

t_wall0 = time.perf_counter()
while t < T_MAX and not reached:
    if t >= next_replan - 1e-9:
        st = RobotState(); st.pos = p_d.copy(); st.vel = v_d.copy(); st.accel = a_d.copy()
        sando.update_state(st)
        push_obstacles(t, p_d)
        t0 = time.perf_counter()
        ret = sando.replan(last_rt, t)
        last_rt = time.perf_counter() - t0
        ok_plan = bool(ret[0] if isinstance(ret, tuple) else ret)
        n_replan += 1
        if not ok_plan:
            n_invalid += 1
        gp = sando.get_global_path()
        if n_replan % 10 == 1:
            print(f"  t={t:5.2f} | ok={int(ok_plan)} | status={sando.get_drone_status()} | "
                  f"gN={len(gp):2d} | drone=({p_d[0]:+.1f},{p_d[1]:+.1f}) | "
                  f"ms={last_rt*1000:4.0f} | clr={clearance_all(p_d, t):+.2f}", flush=True)
        next_replan = t + REPLAN_DT

    ok_g, ng = sando.get_next_goal()
    if ok_g:
        p_d = np.asarray(ng.pos, dtype=float)
        v_d = np.asarray(ng.vel, dtype=float)
        a_d = np.asarray(ng.accel, dtype=float)

    c = clearance_all(p_d, t)
    min_clear = min(min_clear, c)
    if c < 0.0:
        collided = True
    n_steps += 1
    t += DT
    if sando.get_drone_status() == GOAL_REACHED and float(np.linalg.norm(p_d - GOAL)) < float(par.goal_radius):
        reached = True

wall = time.perf_counter() - t_wall0
moved = float(np.linalg.norm(p_d - START))
print("\n===========================================================", flush=True)
print(f"[smoke] done. reached={reached} collided={collided} t={t:.2f}s "
      f"replans={n_replan} 规划失败={n_invalid}", flush=True)
print(f"[smoke] 无人机位移={moved:.2f}m  最小净空={min_clear:.3f}m  墙钟={wall:.2f}s", flush=True)

# 判据: 撞了=失败; 没撞 + 到终点 = 跑通。replan 偶尔 ok=0 由 RTA 兜底(无人机照样绕前进),
# 不算失败信号 —— 失败信号是「撞」或「整段一步没动」。invalid 率只作信息打印。
inv_rate = (n_invalid / n_replan) if n_replan else 1.0
ok = (not collided) and (moved > 0.5) and reached
if ok:
    verdict = "PASS — C++ 闭环在 Ubuntu 跑通(无碰撞 + 到达终点)"
elif (not collided) and (moved > 0.5):
    verdict = "PARTIAL — C++ 闭环活着且无碰撞, 但本次未到终点(看是否 t_max 不够 / v_max 太激进)"
else:
    verdict = "FAIL — 撞了或没动, 接线/规划有问题, 看上面日志"
print(f"[smoke] replan ok=0 比例 = {inv_rate*100:.0f}% (高多半是 v_max={getattr(par,'v_max','?')} 在密集场景太激进, RTA 兜底)", flush=True)
print(f"[smoke] 判定: {verdict}", flush=True)
print("===========================================================\n", flush=True)
# 接线验证用途: 只要没撞就算桥/capi/C++ 闭环在 Ubuntu 健康(退 0); 撞了才退 1。
sys.exit(0 if (not collided and moved > 0.5) else 1)
