"""3D-rendered MetaUrban × SANDO demo — the real Panda3D view (not the BEV top-down).

use_render=True opens MetaUrban's 3D window. The drone (real drone .glb) and four animals
(dog/cow/sheep/cat .glb) are spawned as actual scene objects and moved each tick — the drone by the
C++ SANDO setpoint, the animals along scripted crossings. The native MetaUrban crowd
(pedestrians / vehicles / static) renders natively in 3D and is fed to the C++ core as ground-truth
obstacles. The main camera tracks the drone so you watch it weave the >=50 m corridor in 3D.

Collision is geometric (drone body sphere r=drone_radius vs obstacle 3-D boxes, computed in the loop);
the planning is full 3-D (the C++ core plans x,y,z with z in [z_min,z_max]); cruise z≈1.5 m.

Run ON A MACHINE WITH A DISPLAY (this opens a real window):
  cd /media/boxuan/Data21/projects/metaurban
  DISPLAY=:1 ~/miniconda3/envs/metaurban/bin/python \
      /media/boxuan/Data21/projects/sando_py/sando-core/metaurban/render_3d.py --seed 3
"""
import os, sys, time, argparse
import numpy as np
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "isaac"))
sys.path.insert(0, _HERE)
from sando_cpp_bridge import (Parameters, RobotState, DynTraj, SANDO,
                              DroneStatus_GOAL_REACHED as GOAL_REACHED)
from custom_glb_object import make_glb_class
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
args = ap.parse_args()

CRUISE_Z = float(LOOP["cruise_z"]); SENSE_R = float(LOOP["sense_cull_r"])
REPLAN_DT = float(LOOP["replan_dt"]); MIN_GOAL = float(LOOP["min_goal_dist"])
ASSETS = "/media/boxuan/Data21/projects/metaurban/custom_assets/"
CLASS_LABEL = {"pedestrian": [0], "vehicle": [1], "animal": [2]}
ANIMAL = {"dog": ("dog_shiba_quaternius.glb", [0.4, 1.0, 0.6]),
          "cow": ("cow_quaternius.glb", [0.9, 2.6, 1.5]),
          "sheep": ("sheep_quaternius.glb", [0.6, 1.3, 1.0]),
          "cat": ("cat_quaternius.glb", [0.25, 0.5, 0.35])}


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

env_cfg = dict(
    crswalk_density=1, object_density=0.4, walk_on_all_regions=False,
    use_render=True, image_observation=False, manual_control=False, map='X',
    window_size=(1280, 800),
    default_expert=False, drivable_area_extension=55, height_scale=1,
    show_mid_block_map=False, show_ego_navigation=False, debug=False, horizon=100000,
    on_continuous_line_done=False, out_of_route_done=False,
    vehicle_config=dict(show_lidar=False, show_navi_mark=False, show_line_to_navi_mark=False,
                        show_dest_mark=False, enable_reverse=True),
    show_sidewalk=True, show_crosswalk=True, random_spawn_lane_index=False,
    num_scenarios=20, accident_prob=0, relax_out_of_road_done=True, max_lateral_dist=1e3,
    crash_vehicle_done=False, crash_object_done=False, crash_human_done=False, traffic_density=0.25,
    spawn_human_num=16, spawn_wheelchairman_num=1, spawn_edog_num=2, spawn_erobot_num=1,
    spawn_drobot_num=1, max_actor_num=40)

print("[3d] constructing env (use_render=True window) ...", flush=True)
env = SidewalkDynamicMetaUrbanEnv(env_cfg)
env.reset(seed=args.seed)
for _ in range(8): env.step([0.0, 0.0])
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
left = np.array([-axis[1], axis[0]])
print(f"[3d] corridor {np.round(START,1)}->{np.round(GOAL,1)} dist={np.linalg.norm(GOAL[:2]-START[:2]):.1f}m", flush=True)

# ---- spawn the drone + animal meshes ----
DroneCls = make_glb_class("Drone", ASSETS + "drone_core_polygoogle.glb", 0.6, 0.6, 0.25, up_fix=True)
drone = eng.spawn_object(DroneCls, position=[float(START[0]), float(START[1])], heading_theta=float(np.arctan2(axis[1], axis[0])))
drone.set_position([float(START[0]), float(START[1]), CRUISE_Z])


class Animal:
    def __init__(self, name, p0, vel):
        glb, size = ANIMAL[name]
        self.name = name; self.size = np.array(size, float)
        self.p0 = np.asarray(p0, float); self.vel = np.asarray(vel, float); self.id = 0
        cls = make_glb_class(name, ASSETS + glb, size[0], size[1], size[2], up_fix=True)
        self.obj = eng.spawn_object(cls, position=[float(p0[0]), float(p0[1])],
                                    heading_theta=float(np.arctan2(vel[1], vel[0]) if np.linalg.norm(vel) > 0 else 0.0))
    def update(self, t):
        pos = self.p0 + self.vel * t
        self.obj.set_position([float(pos[0]), float(pos[1]), self.size[2] * 0.5])
        return pos


animals = []
for i, name in enumerate(["dog", "cow", "sheep", "cat"]):
    frac = 0.40 + 0.13 * i
    c = START[:2] + axis * (np.linalg.norm(GOAL[:2] - START[:2]) * frac)
    side = (-1) ** i
    a = Animal(name, c + left * (6.0 * side), -left * side * 1.0)
    a.id = 900 + i
    animals.append(a)

# ---- camera follows the drone ----
try:
    eng.main_camera.track(drone)
    print("[3d] camera tracking drone", flush=True)
except Exception as e:
    print(f"[3d] camera track failed ({e}); default camera", flush=True)

# ---- planner ----
par = Parameters()
for k, v in PLN.items():
    if hasattr(par, k): setattr(par, k, v)
par.replan_dt = REPLAN_DT
sando = SANDO(par)
_st = RobotState(); _st.pos = START.copy(); _st.vel = np.zeros(3); sando.update_state(_st)
sando.update_occupancy_map_ptr(np.zeros((0, 3)))
_G = RobotState(); _G.pos = GOAL.copy(); sando.set_terminal_goal(_G)
_cache = {}


def _dt(tid, size, pos, vel, label, t_sim):
    d = _cache.get(tid)
    if d is None:
        d = DynTraj(); d.id = int(tid); d.mode = "Analytic"; _cache[tid] = d
    d.bbox = np.asarray(size, float)
    x0, y0, vx, vy = float(pos[0]), float(pos[1]), float(vel[0]), float(vel[1])
    d.traj_x = f"{x0}+({vx})*(t-({t_sim}))"; d.traj_y = f"{y0}+({vy})*(t-({t_sim}))"; d.traj_z = f"{CRUISE_Z}"
    d.traj_vx, d.traj_vy, d.traj_vz = f"{vx}", f"{vy}", "0.0"; d.label_set = list(label)
    d.compile_analytic(); sando.add_traj(d, t_sim)


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
    for a in animals:
        pos = a.p0 + a.vel * t_sim; c3 = p3(pos, a.size[2] * 0.5)
        if np.linalg.norm(c3[:2] - p_drone[:2]) <= SENSE_R:
            _dt(a.id, a.size, pos, a.vel, CLASS_LABEL["animal"], t_sim)
            fed.append(("animal", c3, a.size))
    sando.update_occupancy_map_ptr(np.asarray(cloud, float) if cloud else np.zeros((0, 3)))
    return fed


def clearance(p, fed):
    r = float(par.drone_radius); gmin = np.inf; per = {}
    for cls, c3, size in fed:
        d = p - c3; half = 0.5 * np.asarray(size, float)
        outside = np.maximum(np.abs(d) - half, 0.0)
        sd = (np.linalg.norm(outside) if np.any(outside > 0) else -np.min(half - np.abs(d))) - r
        gmin = min(gmin, sd); per[cls] = min(per.get(cls, np.inf), sd)
    return gmin, per


# ---- loop ----
DT = float(par.dc); T_MAX = float(args.t_max)
p_d = START.copy(); v_d = np.zeros(3); a_d = np.zeros(3)
t = 0.0; last_rt = 0.0; reached = False; mclr = np.inf; per_all = {}
print("[3d] loop start — watch the 3D window", flush=True)
while t < T_MAX and not reached:
    st = RobotState(); st.pos = p_d.copy(); st.vel = v_d.copy(); st.accel = a_d.copy()
    sando.update_state(st)
    fed = feed(t, p_d)
    t0 = time.perf_counter(); sando.replan(last_rt, t); last_rt = time.perf_counter() - t0
    for _ in range(int(round(REPLAN_DT / DT))):
        ok, ng = sando.get_next_goal()
        if ok:
            p_d = np.asarray(ng.pos, float); v_d = np.asarray(ng.vel, float); a_d = np.asarray(ng.accel, float)
        t += DT
    # drive the drone mesh + heading; animals; advance + render the world
    drone.set_position([float(p_d[0]), float(p_d[1]), float(p_d[2])])
    if np.linalg.norm(v_d[:2]) > 0.1:
        drone.set_heading_theta(float(np.arctan2(v_d[1], v_d[0])))
    for a in animals: a.update(t)
    env.step([0.0, 0.0])
    env.render(text={"SANDO": "drone 4-class avoidance", "t": f"{t:.1f}s",
                     "z(alt)": f"{p_d[2]:.2f}m", "minClr": f"{mclr if mclr<1e8 else 0:.2f}m"})
    c, per = clearance(p_d, fed); mclr = min(mclr, c)
    for k, val in per.items(): per_all[k] = min(per_all.get(k, np.inf), val)
    if (sando.get_drone_status() == GOAL_REACHED and np.linalg.norm(p_d - GOAL) < float(par.goal_radius)):
        reached = True

print(f"\n[3d] done. reached={reached} collided={mclr < 0} min_clr={mclr:.3f}m  "
      + "  ".join(f"{k}:{v:.2f}" for k, v in sorted(per_all.items())), flush=True)
print("[3d] holding 3D window 20s ...", flush=True)
for _ in range(400):
    env.step([0.0, 0.0]); env.render()
env.close()
