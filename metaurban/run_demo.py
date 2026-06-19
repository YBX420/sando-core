"""MetaUrban <-> C++ SANDO closed loop: a low-cruise delivery drone weaves a >=50 m sidewalk
corridor through MetaUrban ground-truth obstacles, applying a DIFFERENT avoidance behaviour per
class (static / pedestrian / vehicle / animal), all decided by the golden-verified C++ core.

The C++ core (cpp/capi/sando_capi.so, driven via isaac/sando_cpp_bridge.py ctypes) is the brain:
global heat-A* over an occupancy map it BUILDS from the static GT (mapping), per-class MINCO with
the conformal label-set gate (human/vehicle/animal -> hard with class-specific d_safe; static ->
soft EGO field), plus RTA + freeze-yield recovery. This file is only MetaUrban's "steering wheel":
it reads GT, marshals it to the core, and applies the returned 3-D setpoint to the drone.

Classes -> conformal Mondrian label codes (DynTraj.label_set) -> C++ derived_class -> d_safe:
  pedestrian -> [0] human   (hard, d_safe 0.8)
  vehicle    -> [1] vehicle (hard, d_safe 0.5)
  animal     -> [2] animal  (hard, d_safe 0.7)
  static     -> DynTraj wall (soft EGO field) + fed into the occupancy map for global routing.

Run (metaurban conda env):
  cd /media/boxuan/Data21/projects/metaurban    # PEDESTRIAN_ROOT is a CWD-relative path
  ~/miniconda3/envs/metaurban/bin/python /media/boxuan/Data21/projects/sando_py/sando-core/metaurban/run_demo.py
"""
import os
import sys
import time
import argparse

import numpy as np
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))          # sando-core/metaurban
_CORE = os.path.dirname(_HERE)                              # sando-core
sys.path.insert(0, os.path.join(_CORE, "isaac"))           # reuse the proven ctypes bridge

from sando_cpp_bridge import (Parameters, RobotState, DynTraj, SANDO,
                              DroneStatus_GOAL_REACHED as GOAL_REACHED)

from metaurban import SidewalkDynamicMetaUrbanEnv
from metaurban.component.agents.pedestrian.base_pedestrian import BasePedestrian
from metaurban.component.delivery_robot.base_deliveryrobot import BaseDeliveryRobot
from metaurban.component.robotdog.base_robotdog import BaseRobotDog
from metaurban.component.vehicle.base_vehicle import BaseVehicle

# ----------------------------------------------------------------------------- config
with open(os.path.join(_HERE, "metaurban_sando.yaml"), encoding="utf-8") as f:
    CFG = yaml.safe_load(f)
PLN, LOOP = CFG["planner"], CFG["loop"]

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=3)
ap.add_argument("--t_max", type=float, default=float(LOOP["t_max"]))
ap.add_argument("--out", type=str, default=os.path.join(_HERE, "out"))
ap.add_argument("--no_animals", action="store_true")
args = ap.parse_args()
os.makedirs(args.out, exist_ok=True)

CRUISE_Z = float(LOOP["cruise_z"])
SENSE_R = float(LOOP["sense_cull_r"])
REPLAN_DT = float(LOOP["replan_dt"])
MIN_GOAL = float(LOOP["min_goal_dist"])

# ----------------------------------------------------------------------------- the 4 classes
# name -> (Mondrian label code or None for static, friendly colour for the plot)
CLASS_LABEL = {"pedestrian": [0], "vehicle": [1], "animal": [2], "static": []}
CLASS_COLOR = {"pedestrian": "#d62728", "vehicle": "#1f77b4",
               "animal": "#9467bd", "static": "#7f7f7f"}


def classify(o):
    if isinstance(o, BasePedestrian):
        return "pedestrian"
    if isinstance(o, (BaseVehicle, BaseDeliveryRobot, BaseRobotDog)):
        return "vehicle"        # any wheeled MetaUrban agent -> vehicle bucket
    return "static"


def obj_size(o):
    if all(hasattr(o, a) for a in ("WIDTH", "LENGTH", "HEIGHT")):
        try:
            return np.array([float(o.WIDTH), float(o.LENGTH), float(o.HEIGHT)], float)
        except Exception:
            pass
    w = float(getattr(o, "top_down_width", 0.6) or 0.6)
    l = float(getattr(o, "top_down_length", 0.6) or 0.6)
    return np.array([w, l, 1.6], float)


def p3(xy, z):
    return np.array([float(xy[0]), float(xy[1]), float(z)], float)


# ----------------------------------------------------------------------------- scripted gauntlet
# A controlled "gauntlet": one obstacle of EACH class placed on the corridor so the per-class
# avoidance is GUARANTEED to be exercised and cleanly measurable, on TOP of the full native crowd
# (which stays as realistic background complexity). Each scripted obstacle is fed to the C++ core
# through the exact same per-class DynTraj path as native GT -> identical treatment.
ANIMAL_MESH = {"dog": "dog_shiba_quaternius.glb", "cat": "cat_quaternius.glb",
               "cow": "cow_quaternius.glb", "sheep": "sheep_quaternius.glb"}
ANIMAL_SIZE = {"dog": [0.4, 1.0, 0.6], "cat": [0.25, 0.5, 0.35],
               "cow": [0.9, 2.6, 1.5], "sheep": [0.6, 1.3, 1.0]}


class ScriptedObstacle:
    """A scripted obstacle of a given class, moving at constant velocity (vel=0 -> static).
    Headless -> only kinematics matter; `mesh` (animals) is loaded only when a render pipeline exists."""
    def __init__(self, oid, cls, p0, vel, size, mesh=None, animal=None):
        self.id = oid
        self.cls = cls                         # pedestrian | vehicle | animal | static
        self.p0 = np.asarray(p0, float)
        self.vel = np.asarray(vel, float)
        self.size = np.array(size, float)
        self.mesh = mesh
        self.animal = animal

    def pos_xy(self, t):
        return self.p0 + self.vel * t


def make_gauntlet(start, goal):
    """Place a static / pedestrian / vehicle / 4 animals gauntlet across the corridor."""
    s, g = np.asarray(start[:2], float), np.asarray(goal[:2], float)
    d = g - s; L = np.linalg.norm(d); fwd = d / (L + 1e-9)
    left = np.array([-fwd[1], fwd[0]])
    obs = []
    oid = 900

    def station(frac):
        return s + fwd * (L * frac)

    # static obstacle sitting ON the line (drone must route AROUND it)
    obs.append(ScriptedObstacle(oid, "static", station(0.18), [0, 0], [1.2, 1.2, 2.5])); oid += 1
    # pedestrian crossing slowly (hard, wide berth + pass-behind)
    c = station(0.34); obs.append(ScriptedObstacle(oid, "pedestrian", c + left * 5.0, -left * 1.2, [0.6, 0.6, 1.7])); oid += 1
    # vehicle crossing fast (hard, tighter d_safe, fast prediction)
    c = station(0.52); obs.append(ScriptedObstacle(oid, "vehicle", c + left * 14.0, -left * 4.5, [2.0, 4.5, 1.6])); oid += 1
    # animals crossing (hard, animal d_safe) — real meshes dog/cat/cow/sheep
    for j, name in enumerate(["dog", "cow", "sheep", "cat"]):
        c = station(0.66 + 0.08 * j); side = (-1) ** j
        obs.append(ScriptedObstacle(oid, "animal", c + left * (6.0 * side), -left * side * 1.0,
                                    ANIMAL_SIZE[name], mesh=ANIMAL_MESH[name], animal=name)); oid += 1
    return obs


# ----------------------------------------------------------------------------- env
env_cfg = dict(
    crswalk_density=1, object_density=0.4, walk_on_all_regions=False,
    use_render=False, image_observation=False, manual_control=False, map='X',
    default_expert=False, drivable_area_extension=55, height_scale=1,
    show_mid_block_map=False, show_ego_navigation=False, debug=False, horizon=10000,
    on_continuous_line_done=False, out_of_route_done=False,
    vehicle_config=dict(show_lidar=False, show_navi_mark=False,
                        show_line_to_navi_mark=False, show_dest_mark=False, enable_reverse=True),
    show_sidewalk=True, show_crosswalk=True, random_spawn_lane_index=False,
    num_scenarios=20, accident_prob=0, relax_out_of_road_done=True, max_lateral_dist=1e3,
    crash_vehicle_done=False, crash_object_done=False, crash_human_done=False,
    traffic_density=0.25,
    spawn_human_num=18, spawn_wheelchairman_num=1, spawn_edog_num=2,
    spawn_erobot_num=1, spawn_drobot_num=1, max_actor_num=40,
)
print("[demo] constructing env ...", flush=True)
env = SidewalkDynamicMetaUrbanEnv(env_cfg)
obs, info = env.reset(seed=args.seed)
for _ in range(10):
    env.step([0.0, 0.0])                               # warm up so movers acquire velocity
eng = env.engine
ego = env.agent


def native_objects():
    out = []
    for oid, o in eng.get_objects().items():
        if o is ego:
            continue
        try:
            pos = np.asarray(o.position, float)
            vel = np.asarray(o.velocity, float)
        except Exception:
            continue
        if pos.shape[0] < 2 or not np.all(np.isfinite(pos)):
            continue
        out.append((oid, classify(o), pos[:2], vel[:2] if vel.shape[0] >= 2 else np.zeros(2), obj_size(o)))
    return out


# ----------------------------------------------------------------------------- pick a >=50 m corridor
# Center a fixed ~56 m corridor on the PEDESTRIAN cluster, oriented along its principal spread, so the
# drone flies straight through the densest crowd region (full native complexity stays on the path).
objs0 = native_objects()
peds = np.array([p for (_, c, p, _, _) in objs0 if c == "pedestrian"])
if len(peds) < 3:
    peds = np.array([p for (_, c, p, _, _) in objs0 if c in ("pedestrian", "vehicle")] or
                    [np.asarray(ego.position[:2], float)])
ctr = peds.mean(axis=0)
if len(peds) >= 3:
    _, _, vt = np.linalg.svd(peds - ctr)
    axis = vt[0] / (np.linalg.norm(vt[0]) + 1e-9)
else:
    axis = np.array([1.0, 0.0])
CORRIDOR_LEN = MIN_GOAL + 6.0                          # 56 m: comfortably >= 50 m
half = CORRIDOR_LEN / 2.0
START = p3(ctr - axis * half, CRUISE_Z)
GOAL = p3(ctr + axis * half, CRUISE_Z)
GOAL[2] = float(PLN.get("default_goal_z", CRUISE_Z))
GOAL_DIST = float(np.linalg.norm(GOAL[:2] - START[:2]))
print(f"[demo] corridor: start={np.round(START,1)} goal={np.round(GOAL,1)} dist={GOAL_DIST:.1f} m "
      f"(>=50 m: {GOAL_DIST >= MIN_GOAL})  | native objs={len(objs0)} peds={len(peds)}", flush=True)

GAUNTLET = [] if args.no_animals else make_gauntlet(START, GOAL)

# ----------------------------------------------------------------------------- planner
par = Parameters()
for k, v in PLN.items():
    if hasattr(par, k):
        setattr(par, k, v)
par.replan_dt = REPLAN_DT

sando = SANDO(par)
_st = RobotState(); _st.pos = START.copy(); _st.vel = np.zeros(3)
sando.update_state(_st)
sando.update_occupancy_map_ptr(np.zeros((0, 3)))
_G = RobotState(); _G.pos = GOAL.copy()
sando.set_terminal_goal(_G)
print(f"[demo] SANDO started. status={sando.get_drone_status()}", flush=True)

# ----------------------------------------------------------------------------- per-class DynTraj feed
_dyntraj_cache = {}      # id -> DynTraj (reused across ticks; add_traj dedups by id)


def feed_obstacles(t_sim, p_drone):
    """Push GT obstacles within SENSE_R to the core as per-class DynTraj, and the static footprints
    into the occupancy map (mapping). Returns the list of (class, pos3, size) actually fed (for metrics)."""
    fed = []
    static_cloud = []
    # --- native MetaUrban GT ---
    for oid, cls, pos, vel, size in native_objects():
        c3 = p3(pos, size[2] * 0.5)
        if np.linalg.norm(c3[:2] - p_drone[:2]) > SENSE_R:
            continue
        if cls == "static":
            # occupancy: sample the footprint column around cruise z (mapping input)
            for dx in (-0.3, 0.0, 0.3):
                for dy in (-0.3, 0.0, 0.3):
                    static_cloud.append([pos[0] + dx, pos[1] + dy, CRUISE_Z])
            _push_dyntraj(oid_hash(oid) % 1000 + 200, size, pos, np.zeros(2), [], t_sim)  # wall (soft)
            fed.append((cls, c3, size))
        else:
            code = CLASS_LABEL[cls]
            _push_dyntraj(oid_hash(oid) % 1000, size, pos, vel, code, t_sim)
            fed.append((cls, c3, size))
    # --- scripted gauntlet (one obstacle of each class on the corridor) ---
    for a in GAUNTLET:
        pos = a.pos_xy(t_sim)
        c3 = p3(pos, a.size[2] * 0.5)
        if np.linalg.norm(c3[:2] - p_drone[:2]) > SENSE_R:
            continue
        if a.cls == "static":
            for dx in (-0.4, 0.0, 0.4):
                for dy in (-0.4, 0.0, 0.4):
                    static_cloud.append([pos[0] + dx, pos[1] + dy, CRUISE_Z])
            _push_dyntraj(a.id + 200, a.size, pos, np.zeros(2), [], t_sim)
        else:
            _push_dyntraj(a.id, a.size, pos, a.vel, CLASS_LABEL[a.cls], t_sim)
        fed.append((a.cls, c3, a.size))
    sando.update_occupancy_map_ptr(np.asarray(static_cloud, float) if static_cloud else np.zeros((0, 3)))
    return fed


def oid_hash(oid):
    return abs(hash(oid))


def _push_dyntraj(tid, size, pos, vel, label, t_sim):
    dt = _dyntraj_cache.get(tid)
    if dt is None:
        dt = DynTraj(); dt.id = int(tid); dt.mode = "Analytic"; _dyntraj_cache[tid] = dt
    dt.bbox = np.asarray(size, float)
    # const-velocity prediction anchored at the CURRENT snapshot time t_sim:  x(t) = x0 + v*(t - t_sim)
    x0, y0 = float(pos[0]), float(pos[1])
    vx, vy = float(vel[0]), float(vel[1])
    dt.traj_x = f"{x0}+({vx})*(t-({t_sim}))"
    dt.traj_y = f"{y0}+({vy})*(t-({t_sim}))"
    dt.traj_z = f"{CRUISE_Z}"
    dt.traj_vx, dt.traj_vy, dt.traj_vz = f"{vx}", f"{vy}", "0.0"
    dt.label_set = list(label)
    dt.compile_analytic()
    sando.add_traj(dt, t_sim)


# ----------------------------------------------------------------------------- clearance metrics
def signed_clearance(p, fed):
    """min over fed obstacles of (surface distance - drone_radius), and the per-class min."""
    r = float(par.drone_radius)
    gmin = np.inf
    per = {}
    for cls, c3, size in fed:
        d = p - c3
        half = 0.5 * np.asarray(size, float)
        outside = np.maximum(np.abs(d) - half, 0.0)
        sd = np.linalg.norm(outside) if np.any(outside > 0) else -np.min(half - np.abs(d))
        sd -= r
        gmin = min(gmin, sd)
        per[cls] = min(per.get(cls, np.inf), sd)
    return gmin, per


# ----------------------------------------------------------------------------- closed loop
DT = float(par.dc)
T_MAX = float(args.t_max)
p_d = START.copy(); v_d = np.zeros(3); a_d = np.zeros(3)
t = 0.0; last_rt = 0.0; next_replan = 0.0
reached = False; n_invalid = 0
min_clear = np.inf
per_class_min = {}
log = []
last_fed = []
print("[demo] loop start. t | ok | status | gN | drone_xyz | min_clr | solve_ms", flush=True)

while t < T_MAX and not reached:
    if t >= next_replan - 1e-9:
        st = RobotState(); st.pos = p_d.copy(); st.vel = v_d.copy(); st.accel = a_d.copy()
        sando.update_state(st)
        last_fed = feed_obstacles(t, p_d)
        t0 = time.perf_counter()
        ret = sando.replan(last_rt, t)
        last_rt = time.perf_counter() - t0
        ok_plan = bool(ret[0] if isinstance(ret, tuple) else ret)
        if not ok_plan:
            n_invalid += 1
        gp = sando.get_global_path()
        env.step([0.0, 0.0])                            # advance the MetaUrban world one tick
        next_replan = t + REPLAN_DT
        if int(round(t / REPLAN_DT)) % 5 == 0:
            print(f"  t={t:5.2f} | ok={int(ok_plan)} | st={sando.get_drone_status()} | "
                  f"gN={len(gp):2d} | xyz=[{p_d[0]:6.1f},{p_d[1]:6.1f},{p_d[2]:4.1f}] | "
                  f"clr={min_clear if min_clear<1e8 else -1:+.2f} | ms={last_rt*1000:4.0f}", flush=True)

    ok_g, ng = sando.get_next_goal()
    if ok_g:
        p_d = np.asarray(ng.pos, float)
        v_d = np.asarray(ng.vel, float)
        a_d = np.asarray(ng.accel, float)

    c, per = signed_clearance(p_d, last_fed)            # clearance vs last sensed set (movers ~static over 0.1s)
    min_clear = min(min_clear, c)
    for k, val in per.items():
        per_class_min[k] = min(per_class_min.get(k, np.inf), val)
    log.append((t, float(p_d[0]), float(p_d[1]), float(p_d[2]), float(c), int(sando.get_drone_status())))

    t += DT
    if (sando.get_drone_status() == GOAL_REACHED and
            float(np.linalg.norm(p_d - GOAL)) < float(par.goal_radius)):
        reached = True

collided = min_clear < 0.0
travelled = float(np.linalg.norm(p_d[:2] - START[:2]))

# ----------------------------------------------------------------------------- outputs
csv_path = os.path.join(args.out, "drone_traj.csv")
with open(csv_path, "w") as f:
    f.write("t,x,y,z,clearance,status\n")
    for r in log:
        f.write(",".join(str(v) for v in r) + "\n")

summary = {
    "seed": args.seed, "reached": bool(reached), "collided": bool(collided),
    "t_end": round(t, 2), "travelled_m": round(travelled, 1), "goal_dist_m": round(GOAL_DIST, 1),
    "min_clearance_m": round(float(min_clear), 3),
    "per_class_min_clearance_m": {k: round(float(v), 3) for k, v in per_class_min.items()},
    "replan_failures": n_invalid, "n_native_objs": len(objs0),
    "gauntlet": {c: sum(1 for a in GAUNTLET if a.cls == c) for c in CLASS_COLOR},
    "start": [round(float(x), 1) for x in START], "goal": [round(float(x), 1) for x in GOAL],
}
import json
with open(os.path.join(args.out, "summary.json"), "w") as f:
    json.dump(summary, f, indent=2)

print("\n=============================================================", flush=True)
print(f"[demo] reached={reached} collided={collided} travelled={travelled:.1f}m / goal {GOAL_DIST:.1f}m", flush=True)
print(f"[demo] min clearance = {min_clear:.3f} m  | per-class: "
      + ", ".join(f"{k}:{v:.2f}" for k, v in sorted(per_class_min.items())), flush=True)
print(f"[demo] replan failures={n_invalid}  t={t:.1f}s  points={len(log)}", flush=True)
print("=============================================================\n", flush=True)

# top-down trajectory artifact (matplotlib, headless-safe)
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    arr = np.array([(r[1], r[2]) for r in log])
    fig, ax = plt.subplots(figsize=(13, 9))
    seen = set()
    for oid, cls, pos, vel, size in objs0:
        lab = cls if cls not in seen else None
        seen.add(cls)
        ax.add_patch(plt.Rectangle((pos[0] - size[0] / 2, pos[1] - size[1] / 2), size[0], size[1],
                                   color=CLASS_COLOR.get(cls, "gray"), alpha=0.45, label=lab))
    for a in GAUNTLET:
        col = CLASS_COLOR.get(a.cls, "gray")
        if np.linalg.norm(a.vel) < 1e-6:
            pxy = a.pos_xy(0.0)
            ax.add_patch(plt.Rectangle((pxy[0] - a.size[0] / 2, pxy[1] - a.size[1] / 2),
                                       a.size[0], a.size[1], color=col, alpha=0.6, hatch="xx"))
        else:
            for tt in np.linspace(0, t, 7):
                pxy = a.pos_xy(tt)
                ax.add_patch(plt.Circle((pxy[0], pxy[1]), max(a.size[0], a.size[1]) / 2,
                                        color=col, alpha=0.22))
        lab = f"{a.cls}*" if (a.cls + "*") not in seen else None
        seen.add(a.cls + "*")
        if lab:
            ax.scatter([], [], color=col, marker="s", label=lab)
    ax.plot(arr[:, 0], arr[:, 1], "-", color="green", lw=2.2, label="drone path")
    ax.scatter([START[0], GOAL[0]], [START[1], GOAL[1]], c=["black", "gold"],
               marker="*", s=240, zorder=5, label="start/goal")
    ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title(f"SANDO drone in MetaUrban — {travelled:.0f} m corridor  "
                 f"reached={reached} collided={collided} min_clr={min_clear:.2f}m")
    fig.savefig(os.path.join(args.out, "drone_topdown.png"), dpi=110, bbox_inches="tight")
    print(f"[demo] artifact -> {os.path.join(args.out, 'drone_topdown.png')}", flush=True)
except Exception as e:
    print(f"[demo] (plot skipped: {e})", flush=True)

env.close()
