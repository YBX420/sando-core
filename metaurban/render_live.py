"""LIVE-rendered MetaUrban × SANDO demo.

Same closed loop as run_demo.py, but each tick renders MetaUrban's bird's-eye view (BEV) of the LIVE
scene and overlays the SANDO-controlled drone (marker + trail), the 4-class gauntlet (static box /
pedestrian / vehicle / real-mesh animals dog·cat·cow·sheep) and a HUD, then shows it in a live
window (cv2.imshow on $DISPLAY). The native MetaUrban crowd (pedestrians/vehicles/static) is drawn
by MetaUrban itself in the BEV; the drone + scripted obstacles are virtual, overlaid via a fixed-
camera world→pixel affine (camera_position + screen_size==film_size → zero crop offset).

Run (metaurban env, from the metaurban repo root for PEDESTRIAN_ROOT):
  cd /media/boxuan/Data21/projects/metaurban
  ~/miniconda3/envs/metaurban/bin/python /media/boxuan/Data21/projects/sando_py/sando-core/metaurban/render_live.py --seed 3
Keys: ESC/q quits.  --mp4 also writes out/drone_live.mp4 ; --no_show skips the window (mp4 only).
"""
import os, sys, time, argparse
import numpy as np
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "isaac"))
from sando_cpp_bridge import (Parameters, RobotState, DynTraj, SANDO,
                              DroneStatus_GOAL_REACHED as GOAL_REACHED)
import cv2
from metaurban import SidewalkDynamicMetaUrbanEnv
from metaurban.component.agents.pedestrian.base_pedestrian import BasePedestrian
from metaurban.component.delivery_robot.base_deliveryrobot import BaseDeliveryRobot
from metaurban.component.robotdog.base_robotdog import BaseRobotDog
from metaurban.component.vehicle.base_vehicle import BaseVehicle

with open(os.path.join(_HERE, "metaurban_sando.yaml"), encoding="utf-8") as f:
    CFG = yaml.safe_load(f)
PLN, LOOP = CFG["planner"], CFG["loop"]

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=3)
ap.add_argument("--t_max", type=float, default=60.0)
ap.add_argument("--mp4", action="store_true")
ap.add_argument("--no_show", action="store_true")
ap.add_argument("--fps", type=float, default=20.0)
args = ap.parse_args()

CRUISE_Z = float(LOOP["cruise_z"]); SENSE_R = float(LOOP["sense_cull_r"])
REPLAN_DT = float(LOOP["replan_dt"]); MIN_GOAL = float(LOOP["min_goal_dist"])

CLASS_LABEL = {"pedestrian": [0], "vehicle": [1], "animal": [2], "static": []}
# BGR colors for cv2
COL = {"pedestrian": (60, 60, 220), "vehicle": (220, 140, 30),
       "animal": (200, 80, 150), "static": (130, 130, 130), "drone": (40, 230, 70)}


def classify(o):
    if isinstance(o, BasePedestrian): return "pedestrian"
    if isinstance(o, (BaseVehicle, BaseDeliveryRobot, BaseRobotDog)): return "vehicle"
    return "static"


def obj_size(o):
    if all(hasattr(o, a) for a in ("WIDTH", "LENGTH", "HEIGHT")):
        try: return np.array([float(o.WIDTH), float(o.LENGTH), float(o.HEIGHT)], float)
        except Exception: pass
    return np.array([float(getattr(o, "top_down_width", 0.6) or 0.6),
                     float(getattr(o, "top_down_length", 0.6) or 0.6), 1.6], float)


def p3(xy, z): return np.array([float(xy[0]), float(xy[1]), float(z)], float)

ANIMAL_SIZE = {"dog": [0.4, 1.0, 0.6], "cat": [0.25, 0.5, 0.35], "cow": [0.9, 2.6, 1.5], "sheep": [0.6, 1.3, 1.0]}


class Scripted:
    def __init__(self, oid, cls, p0, vel, size, animal=None):
        self.id = oid; self.cls = cls; self.p0 = np.asarray(p0, float)
        self.vel = np.asarray(vel, float); self.size = np.array(size, float); self.animal = animal
    def pos_xy(self, t): return self.p0 + self.vel * t


def make_gauntlet(start, goal):
    s, g = np.asarray(start[:2], float), np.asarray(goal[:2], float)
    d = g - s; L = np.linalg.norm(d); fwd = d / (L + 1e-9); left = np.array([-fwd[1], fwd[0]])
    st = lambda f: s + fwd * (L * f)
    obs = []; oid = 900
    obs.append(Scripted(oid, "static", st(0.18), [0, 0], [1.2, 1.2, 2.5])); oid += 1
    obs.append(Scripted(oid, "pedestrian", st(0.34) + left * 5.0, -left * 1.2, [0.6, 0.6, 1.7])); oid += 1
    obs.append(Scripted(oid, "vehicle", st(0.52) + left * 14.0, -left * 4.5, [2.0, 4.5, 1.6])); oid += 1
    for j, name in enumerate(["dog", "cow", "sheep", "cat"]):
        side = (-1) ** j
        obs.append(Scripted(oid, "animal", st(0.66 + 0.08 * j) + left * (6.0 * side),
                            -left * side * 1.0, ANIMAL_SIZE[name], animal=name)); oid += 1
    return obs


env_cfg = dict(
    crswalk_density=1, object_density=0.4, walk_on_all_regions=False,
    use_render=False, image_observation=False, manual_control=False, map='X',
    default_expert=False, drivable_area_extension=55, height_scale=1,
    show_mid_block_map=False, show_ego_navigation=False, debug=False, horizon=10000,
    on_continuous_line_done=False, out_of_route_done=False,
    vehicle_config=dict(show_lidar=False, show_navi_mark=False, show_line_to_navi_mark=False,
                        show_dest_mark=False, enable_reverse=True),
    show_sidewalk=True, show_crosswalk=True, random_spawn_lane_index=False,
    num_scenarios=20, accident_prob=0, relax_out_of_road_done=True, max_lateral_dist=1e3,
    crash_vehicle_done=False, crash_object_done=False, crash_human_done=False, traffic_density=0.25,
    spawn_human_num=18, spawn_wheelchairman_num=1, spawn_edog_num=2, spawn_erobot_num=1,
    spawn_drobot_num=1, max_actor_num=40)

print("[live] constructing env ...", flush=True)
env = SidewalkDynamicMetaUrbanEnv(env_cfg)
env.reset(seed=args.seed)
for _ in range(10): env.step([0.0, 0.0])
eng = env.engine; ego = env.agent


def native_objects():
    out = []
    for oid, o in eng.get_objects().items():
        if o is ego: continue
        try:
            pos = np.asarray(o.position, float); vel = np.asarray(o.velocity, float)
        except Exception: continue
        if pos.shape[0] < 2 or not np.all(np.isfinite(pos)): continue
        out.append((oid, classify(o), pos[:2], vel[:2] if vel.shape[0] >= 2 else np.zeros(2), obj_size(o)))
    return out


objs0 = native_objects()
peds = np.array([p for (_, c, p, _, _) in objs0 if c == "pedestrian"])
if len(peds) < 3:
    peds = np.array([p for (_, c, p, _, _) in objs0 if c in ("pedestrian", "vehicle")] or [np.asarray(ego.position[:2], float)])
ctr = peds.mean(axis=0)
axis = (np.linalg.svd(peds - ctr)[2][0] if len(peds) >= 3 else np.array([1.0, 0.0]))
axis = axis / (np.linalg.norm(axis) + 1e-9)
half = (MIN_GOAL + 6.0) / 2.0
START = p3(ctr - axis * half, CRUISE_Z); GOAL = p3(ctr + axis * half, CRUISE_Z)
GOAL[2] = float(PLN.get("default_goal_z", CRUISE_Z))
GAUNTLET = make_gauntlet(START, GOAL)
CORR_CTR = 0.5 * (START[:2] + GOAL[:2])
print(f"[live] corridor {np.round(START,1)}->{np.round(GOAL,1)} "
      f"dist={np.linalg.norm(GOAL[:2]-START[:2]):.1f}m", flush=True)

# ----- fixed-camera BEV world->pixel affine (camera_position + screen==film -> zero crop offset) -----
N = 950; S = 10.0                                  # 950 px, 10 px/m -> ~95 m field (covers 56 m + margin)
CAM = CORR_CTR


def w2p(x, y):
    return int((x - CAM[0]) * S + N / 2), int(N / 2 - (y - CAM[1]) * S)


def render_bev():
    img = env.render(mode="topdown", window=False, screen_size=(N, N), film_size=(N, N),
                     scaling=S, camera_position=(float(CAM[0]), float(CAM[1])),
                     target_vehicle_heading_up=False, draw_target_vehicle_trajectory=False)
    a = np.asarray(img)
    if a.dtype != np.uint8: a = a.astype(np.uint8)
    return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if a.shape[2] == 3 else a


# ----- planner -----
par = Parameters()
for k, v in PLN.items():
    if hasattr(par, k): setattr(par, k, v)
par.replan_dt = REPLAN_DT
sando = SANDO(par)
_st = RobotState(); _st.pos = START.copy(); _st.vel = np.zeros(3); sando.update_state(_st)
sando.update_occupancy_map_ptr(np.zeros((0, 3)))
_G = RobotState(); _G.pos = GOAL.copy(); sando.set_terminal_goal(_G)

_cache = {}


def feed(t_sim, p_drone):
    fed = []; cloud = []
    for oid, cls, pos, vel, size in native_objects():
        c3 = p3(pos, size[2] * 0.5)
        if np.linalg.norm(c3[:2] - p_drone[:2]) > SENSE_R: continue
        if cls == "static":
            for dx in (-0.3, 0, 0.3):
                for dy in (-0.3, 0, 0.3): cloud.append([pos[0] + dx, pos[1] + dy, CRUISE_Z])
            _dt(abs(hash(oid)) % 1000 + 200, size, pos, np.zeros(2), [], t_sim)
        else:
            _dt(abs(hash(oid)) % 1000, size, pos, vel, CLASS_LABEL[cls], t_sim)
        fed.append((cls, c3, size))
    for a in GAUNTLET:
        pos = a.pos_xy(t_sim); c3 = p3(pos, a.size[2] * 0.5)
        if np.linalg.norm(c3[:2] - p_drone[:2]) > SENSE_R: continue
        if a.cls == "static":
            for dx in (-0.4, 0, 0.4):
                for dy in (-0.4, 0, 0.4): cloud.append([pos[0] + dx, pos[1] + dy, CRUISE_Z])
            _dt(a.id + 200, a.size, pos, np.zeros(2), [], t_sim)
        else:
            _dt(a.id, a.size, pos, a.vel, CLASS_LABEL[a.cls], t_sim)
        fed.append((a.cls, c3, a.size, a.animal if a.cls == "animal" else None))
    sando.update_occupancy_map_ptr(np.asarray(cloud, float) if cloud else np.zeros((0, 3)))
    return fed


def _dt(tid, size, pos, vel, label, t_sim):
    d = _cache.get(tid)
    if d is None:
        d = DynTraj(); d.id = int(tid); d.mode = "Analytic"; _cache[tid] = d
    d.bbox = np.asarray(size, float)
    x0, y0, vx, vy = float(pos[0]), float(pos[1]), float(vel[0]), float(vel[1])
    d.traj_x = f"{x0}+({vx})*(t-({t_sim}))"; d.traj_y = f"{y0}+({vy})*(t-({t_sim}))"; d.traj_z = f"{CRUISE_Z}"
    d.traj_vx, d.traj_vy, d.traj_vz = f"{vx}", f"{vy}", "0.0"; d.label_set = list(label)
    d.compile_analytic(); sando.add_traj(d, t_sim)


def clearance(p, fed):
    r = float(par.drone_radius); gmin = np.inf; per = {}
    for item in fed:
        cls, c3, size = item[0], item[1], item[2]
        d = p - c3; half = 0.5 * np.asarray(size, float)
        outside = np.maximum(np.abs(d) - half, 0.0)
        sd = (np.linalg.norm(outside) if np.any(outside > 0) else -np.min(half - np.abs(d))) - r
        gmin = min(gmin, sd); per[cls] = min(per.get(cls, np.inf), sd)
    return gmin, per


def draw(frame, t, fed, trail, status, mclr, per):
    # corridor + start/goal
    cv2.line(frame, w2p(*START[:2]), w2p(*GOAL[:2]), (90, 90, 90), 1, cv2.LINE_AA)
    for P, c in ((START, (0, 215, 215)), (GOAL, (0, 215, 255))):
        cv2.drawMarker(frame, w2p(P[0], P[1]), c, cv2.MARKER_STAR, 22, 2)
    # scripted gauntlet (the 4 named classes)
    for item in fed:
        cls, c3, size = item[0], item[1], item[2]
        animal = item[3] if len(item) > 3 else None
        col = COL.get(cls, (150, 150, 150)); px, py = w2p(c3[0], c3[1])
        w = max(4, int(0.5 * size[0] * S)); h = max(4, int(0.5 * size[1] * S))
        cv2.rectangle(frame, (px - w, py - h), (px + w, py + h), col, 2)
        if animal: cv2.putText(frame, animal, (px - 10, py - h - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
    # drone trail + body
    for i in range(1, len(trail)):
        cv2.line(frame, trail[i - 1], trail[i], COL["drone"], 2, cv2.LINE_AA)
    if trail:
        px, py = trail[-1]
        cv2.circle(frame, (px, py), 7, COL["drone"], -1, cv2.LINE_AA)
        cv2.circle(frame, (px, py), int(float(par.drone_radius) * S) + 7, COL["drone"], 1, cv2.LINE_AA)
    # HUD
    bar = "COLLIDED" if mclr < 0 else "CLEAR"
    cv2.rectangle(frame, (0, 0), (N, 64), (0, 0, 0), -1)
    cv2.putText(frame, f"MetaUrban x SANDO  t={t:4.1f}s  status={status}  minClr={mclr:+.2f}m  {bar}",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    pc = "  ".join(f"{k}:{per.get(k, float('nan')):+.2f}" for k in ("static", "pedestrian", "vehicle", "animal"))
    cv2.putText(frame, "per-class clr  " + pc, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (210, 210, 210), 1, cv2.LINE_AA)
    return frame


# ----- loop -----
DT = float(par.dc); T_MAX = float(args.t_max)
p_d = START.copy(); v_d = np.zeros(3); a_d = np.zeros(3)
t = 0.0; last_rt = 0.0; reached = False; mclr = np.inf; per_all = {}
trail = []
writer = None
if args.mp4:
    import imageio
    writer = imageio.get_writer(os.path.join(_HERE, "out", "drone_live.mp4"), fps=int(args.fps))
win = "MetaUrban x SANDO (drone, 4-class avoidance)"
can_show = not args.no_show
frame_dt = 1.0 / max(1.0, args.fps)
print("[live] loop start (ESC/q to quit)", flush=True)
while t < T_MAX and not reached:
    st = RobotState(); st.pos = p_d.copy(); st.vel = v_d.copy(); st.accel = a_d.copy()
    sando.update_state(st)
    fed = feed(t, p_d)
    t0 = time.perf_counter(); sando.replan(last_rt, t); last_rt = time.perf_counter() - t0
    env.step([0.0, 0.0])
    # advance drone over one replan_dt by consuming setpoints
    for _ in range(int(round(REPLAN_DT / DT))):
        ok, ng = sando.get_next_goal()
        if ok:
            p_d = np.asarray(ng.pos, float); v_d = np.asarray(ng.vel, float); a_d = np.asarray(ng.accel, float)
        t += DT
    c, per = clearance(p_d, fed); mclr = min(mclr, c)
    for k, val in per.items(): per_all[k] = min(per_all.get(k, np.inf), val)
    trail.append(w2p(p_d[0], p_d[1])); trail = trail[-400:]
    frame = draw(render_bev(), t, fed, trail, sando.get_drone_status(), mclr, per)
    if writer is not None: writer.append_data(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    if can_show:
        try:
            cv2.imshow(win, frame)
            if (cv2.waitKey(max(1, int(frame_dt * 1000))) & 0xFF) in (27, ord('q')): break
        except Exception as e:
            print(f"[live] window unavailable ({e}); switching to mp4-only. Re-run with --mp4 / on a display.", flush=True)
            can_show = False
    if (sando.get_drone_status() == GOAL_REACHED and np.linalg.norm(p_d - GOAL) < float(par.goal_radius)):
        reached = True

collided = mclr < 0
print(f"\n[live] done. reached={reached} collided={collided} min_clr={mclr:.3f}m  "
      + "  ".join(f"{k}:{v:.2f}" for k, v in sorted(per_all.items())), flush=True)
if writer is not None:
    writer.close(); print(f"[live] mp4 -> {os.path.join(_HERE, 'out', 'drone_live.mp4')}", flush=True)
if can_show:
    try:                                       # hold the final frame so the window stays up to inspect
        verdict = ("REACHED, NO COLLISION" if (reached and not collided) else
                   ("COLLIDED" if collided else "stopped"))
        cv2.putText(frame, f"DONE: {verdict} — press any key / ESC to close",
                    (10, N - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow(win, frame); cv2.waitKey(0)
    except Exception:
        pass
try: cv2.destroyAllWindows()
except Exception: pass
env.close()
