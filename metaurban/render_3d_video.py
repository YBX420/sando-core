"""Real-time 3D fly-through of the MetaUrban × SANDO drone demo — FPV + 3rd-person + planned path.

The onscreen 3D window is gray on this box (DISPLAY :1 is a software-GL/virtual display whose shader
terrain does not draw), but the OFFSCREEN RGB camera renders the real 3D scene on the GPU. So we mount
the RGB camera twice per frame (onboard FPV + behind/above chase-cam) and blit both, side-by-side, into
a live cv2 window (cv2 on :1 is a 2D blit, which works). We ALSO draw, in the real 3D world, the path the
drone just chose: the heat-A* global route (green), the immediate next set-point (magenta), and the
drone->next-goal look-ahead (yellow) — so both cameras see exactly what SANDO decided to do next.

The drone (real .glb) + four animals (dog/cow/sheep/cat .glb) are spawned as 3D objects; the drone is
driven by the C++ SANDO 3-D set-points, animals by scripted crossings; the native MetaUrban crowd is
fed to the C++ core as GT. Full-3D planning (x,y,z), cruise z≈1.5 m.

Run (metaurban env, from the metaurban repo root):
  cd /media/boxuan/Data21/projects/metaurban
  DISPLAY=:1 ~/miniconda3/envs/metaurban/bin/python \
      /media/boxuan/Data21/projects/sando_py/sando-core/metaurban/render_3d_video.py --seed 3 --live --loop_scene
  # one verification frame (no live window):  ... render_3d_video.py --seed 3 --frame_only
  # record an mp4:                            ... render_3d_video.py --seed 3 --mp4
"""
import os, sys, time, argparse, random
import cv2  # IMPORTANT: import cv2 BEFORE panda3d/metaurban — importing it after them segfaults (GL/X lib clash)
import numpy as np
import yaml
from panda3d.core import LineSegs, NodePath, Vec3

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "isaac"))
sys.path.insert(0, _HERE)
from sando_cpp_bridge import (Parameters, RobotState, DynTraj, SANDO,
                              DroneStatus_GOAL_REACHED as GOAL_REACHED)
from custom_glb_object import make_glb_class, make_drone_class
from quadrotor import Quadrotor
from px4_bridge import PX4Bridge   # real PX4 SITL flight stack (used when --px4)
from ego_bridge import EGOPlanner  # standalone EGO-Planner core (used when --ego), fed the depth-FOV cloud
# sando_native_bridge is imported lazily inside the --native branch (its .so links GUROBI; only load on demand)
from metaurban import SidewalkDynamicMetaUrbanEnv
from metaurban.component.sensors.rgb_camera import RGBCamera
from metaurban.obs.observation_base import DummyObservation   # skip per-step obs gather (no wasted readback)
from metaurban.component.agents.pedestrian.base_pedestrian import BasePedestrian
from metaurban.component.delivery_robot.base_deliveryrobot import BaseDeliveryRobot
from metaurban.component.robotdog.base_robotdog import BaseRobotDog
from metaurban.component.vehicle.base_vehicle import BaseVehicle
import imageio.v2 as imageio

with open(os.path.join(_HERE, "metaurban_sando.yaml"), encoding="utf-8") as f:
    CFG = yaml.safe_load(f)
PLN, LOOP = CFG["planner"], CFG["loop"]

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=3)
ap.add_argument("--t_max", type=float, default=40.0)
ap.add_argument("--fps", type=int, default=20)
ap.add_argument("--view", type=str, default="dual", choices=["dual", "fpv", "chase"],
                help="dual = FPV + 3rd-person side-by-side; or just one")
ap.add_argument("--w", type=int, default=640, help="per-view render width")
ap.add_argument("--h", type=int, default=400, help="per-view render height")
ap.add_argument("--frame_only", action="store_true",
                help="fly a few seconds then dump one composed frame to out/drone_3d_frame.png and exit")
ap.add_argument("--live", action="store_true", help="show a REAL-TIME window (offscreen GPU render -> cv2.imshow); on a software-GL display this is black, prefer --serve")
ap.add_argument("--serve", action="store_true",
                help="REAL-TIME view over HTTP MJPEG (open http://localhost:<port> in a browser) — robust on software-GL/headless boxes where cv2 windows are black")
ap.add_argument("--port", type=int, default=8089, help="port for --serve")
ap.add_argument("--px4", action="store_true", help="fly the REAL PX4 SITL flight stack via MAVSDK offboard (needs PX4 SITL running) instead of the lightweight quadrotor.py")
ap.add_argument("--ego", action="store_true", help="use the standalone EGO-Planner core (ESDF-free, depth-FOV point cloud) instead of the SANDO C++ core")
ap.add_argument("--native", action="store_true", help="use the NATIVE MIT-ACL SANDO baseline (de-ROS'd: heat-A* + DecompUtil SFC + GUROBI), fed the depth-FOV cloud, instead of our MINCO core")
ap.add_argument("--fov_range", type=float, default=8.0, help="depth-camera perception range (m) when --ego (D435i ~ a few m); only obstacles in this forward cone are seen")
ap.add_argument("--fov_deg", type=float, default=45.0, help="depth-camera half-FOV (deg) for the forward perception cone when --ego")
ap.add_argument("--seam", action="store_true", help="enable seam C2-from-exec-state (A4): re-anchor each MINCO solve at the drone's real execution state so 'what flies == what is certified' (our MINCO core only)")
ap.add_argument("--ego_safe", action="store_true", help="wrap EGO with MINCO's per-class certified MOVER safety: certify EGO's committed B-spline (S3 continuous-time deficit) against each detected mover with per-class d_safe (human0.8/vehicle0.6/animal0.7); uncertified commit -> RTA HOLD. Static stays EGO's own cloud avoidance. Needs --ego")
ap.add_argument("--maneuver", action="store_true", help="NO-HOLD CYLINDER maneuvering (M3): each step, run a fastest-safe candidate tournament (straight/around-L/R/over/climb sub-goals), certify each committed B-spline vs every mover with the CYLINDER disjunction (horizontal sqrt(dx^2+dy^2)>=r+d_safe OR vertical p_z>=z_clear), and FLY the certified candidate with the most goal-ward speed. Fly OVER a wall, AROUND a crosser, climb as the no-freeze escape. Needs --ego; replaces --ego_safe's HOLD")
ap.add_argument("--slip", action="store_true", help="SLIP (space-time speed-warp): plan ONE tight near-native EGO path, then fly it at the FASTEST scalar speed-warp s whose RE-TIMED flight the continuous-time cert proves clears every KF-predicted moving object (s>1 slip-AHEAD = faster than EGO, s<1 slip-BEHIND a crosser). KF relative velocity optimizes the acceleration; no re-route, no climb. Needs --ego")
ap.add_argument("--clear_spawn", action="store_true", help="re-roll the route until the drone's START is genuinely clear of static obstacles (full field incl. trees), so it never spawns inside foliage. Deterministic per seed, so A/B stays controlled")
ap.add_argument("--mp4", action="store_true", help="also write out/drone_3d.mp4")
ap.add_argument("--loop_scene", action="store_true", help="restart the fly-through forever for continuous live viewing")
args = ap.parse_args()
if args.maneuver:
    args.ego = True; args.ego_safe = False   # the no-HOLD tournament REPLACES the ego_safe HOLD wrapper
if args.slip:
    args.ego = True; args.ego_safe = False; args.maneuver = False   # SLIP replaces the brake/tournament with speed-warp

CRUISE_Z = float(LOOP["cruise_z"]); SENSE_R = float(LOOP["sense_cull_r"])
REPLAN_DT = float(LOOP["replan_dt"]); MIN_GOAL = float(LOOP["min_goal_dist"])
# disk was renamed Data21 -> Data2 (same drive); pick whichever custom_assets/ actually exists so the
# absolute path survives the rename (see memory dev-env-data2-data21).
ASSETS = next((p for p in ("/media/boxuan/Data2/projects/metaurban/custom_assets/",
                           "/media/boxuan/Data21/projects/metaurban/custom_assets/")
               if os.path.isdir(p)), "/media/boxuan/Data2/projects/metaurban/custom_assets/")
CLASS_LABEL = {"pedestrian": [0], "vehicle": [1], "animal": [2]}
# keep it simple: the cow is the only custom animal (glb, hpr_fix stands it up, grounded by make_glb_class)
ANIMAL = {"cow": ("cow_quaternius.glb", [0.9, 2.6, 1.6], (0.0, -90.0, 0.0))}  # stands up, nose -> +X (heading 0)
# MetaUrban object frame: heading 0 == +X (forward). A camera looks +X with H=-90 (default look is +Y).
# camera poses relative to drone.origin (forward=+X, +Z=up):
FPV_POS, FPV_HPR = (0.35, 0.0, 0.20), (-90.0, -12.0, 0.0)    # onboard nose-cam, looks forward (+X), slight down
CHASE_POS, CHASE_HPR = (-7.0, 0.0, 3.0), (-90.0, -18.0, 0.0) # behind (-X) + above, looks forward (+X), pitched down


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
    crswalk_density=1, object_density=0.9, walk_on_all_regions=False,   # DENSE scene -> the avoider has real work
    use_render=False, image_observation=True, sensors=dict(rgb_camera=(RGBCamera, args.w, args.h)),
    interface_panel=[], manual_control=False, map='X', daytime="12:00",
    default_expert=False, drivable_area_extension=55, height_scale=1,
    show_mid_block_map=False, show_ego_navigation=False, debug=False, horizon=100000,
    on_continuous_line_done=False, out_of_route_done=False,
    vehicle_config=dict(show_lidar=False, show_navi_mark=False, show_line_to_navi_mark=False,
                        show_dest_mark=False, enable_reverse=True,
                        lidar=dict(num_lasers=0, distance=50)),   # no ego lidar ray-casts
    # --- perf ---
    agent_observation=DummyObservation,   # don't gather/throw-away an obs (no wasted camera readback) each step
    multi_thread_render=False,            # single-thread render: one renderFrame/view is correct (no 2x taskMgr.step)
    decision_repeat=2, physics_world_step_size=0.05,   # 0.1s/step but ~60% fewer bullet substeps (crowd = ORCA, unaffected)
    show_sidewalk=True, show_crosswalk=True, random_spawn_lane_index=False,
    num_scenarios=20, accident_prob=0, relax_out_of_road_done=True, max_lateral_dist=1e3,
    crash_vehicle_done=False, crash_object_done=False, crash_human_done=False, traffic_density=0.5,
    spawn_human_num=85, spawn_wheelchairman_num=5, spawn_edog_num=6, spawn_erobot_num=3,
    spawn_drobot_num=3, max_actor_num=170)

print("[3dv] constructing offscreen env ...", flush=True)
env = SidewalkDynamicMetaUrbanEnv(env_cfg)
# MetaUrban only has num_scenarios scenes -> the SCENE index must wrap into [0, num_scenarios). The full
# args.seed still drives plan_route's RNG (seed*1000), so seeds >= num_scenarios reuse a scene but get a
# DIFFERENT route (more variety without crashing on env.reset's [0:N) assertion).
_NUM_SCENARIOS = 20
env.reset(seed=args.seed % _NUM_SCENARIOS)
for _ in range(8): env.step([0.0, 0.0])
eng = env.engine; agent_ego = env.agent
cam = eng.get_sensor("rgb_camera")


def native_objects():
    out = []
    for oid, o in eng.get_objects().items():
        if o is agent_ego: continue
        try:
            pos = np.asarray(o.position, float); vel = np.asarray(o.velocity, float)
        except Exception: continue
        if pos.shape[0] < 2 or not np.all(np.isfinite(pos)): continue
        out.append((oid, classify(o), pos[:2], vel[:2] if vel.shape[0] >= 2 else np.zeros(2), obj_size(o)))
    return out


ROUTE_LEN = max(MIN_GOAL + 25.0, 75.0)   # LONGER corridor than the old 56 m


def plan_route(lap_idx):
    """Per-lap LONG, randomised route that THREADS THROUGH PEOPLE: pick 2-3 random pedestrians as
    waypoints (movers first => crossing at crosswalks), order them along their principal axis, then
    extend a start/goal before & after for length. Guarantees the drone flies through the crowd, with
    a different (random) start & route each lap. Returns (route3 [start, wp..., goal], dir, left)."""
    rng = random.Random(args.seed * 1000 + lap_idx)
    objs = native_objects()
    peds = [p for (_, c, p, _, _) in objs if c == "pedestrian"]
    movers = [p for (_, c, p, v, _) in objs if c == "pedestrian" and np.linalg.norm(v) > 0.2]  # crossing => crosswalk
    pool = peds if len(peds) >= 3 else [p for (_, c, p, _, _) in objs if c in ("pedestrian", "vehicle")]
    if len(pool) < 2:
        e = np.asarray(ego.position[:2], float)
        return [p3(e, CRUISE_Z), p3(e + np.array([ROUTE_LEN, 0.0]), CRUISE_Z)], np.array([1.0, 0.0]), np.array([0.0, 1.0])
    anchor = np.asarray(rng.choice(movers if movers else pool), float)   # crossing ped => crosswalk
    cluster = sorted((np.asarray(q, float) for q in pool),
                     key=lambda q: float(np.linalg.norm(q - anchor)))[:6]  # a LOCAL crowd cluster, not the whole map
    C = np.array(cluster); ctr = C.mean(axis=0)
    d = (np.linalg.svd(C - ctr)[2][0] if len(C) >= 2 else np.array([1.0, 0.0]))
    if rng.random() < 0.5: d = -d                                # random direction each lap
    d = d / (np.linalg.norm(d) + 1e-9); left = np.array([-d[1], d[0]])
    # ALWAYS 2 points: a straight start->goal through the crowd-cluster centroid (SANDO weaves en route).
    # Random anchor/direction per lap => different start & route each time; no intermediate waypoints.
    statics = [(np.asarray(p, float), np.asarray(s, float)) for (_, c, p, _, s) in objs if c == "static"]

    def _clear(pt):                                          # keep start/goal OUT of static-object bboxes
        for sp, ssz in statics:
            if np.all(np.abs(pt[:2] - sp[:2]) < 0.5 * ssz[:2] + 0.8): return False
        return True

    start_xy = ctr - d * (ROUTE_LEN / 2.0); goal_xy = ctr + d * (ROUTE_LEN / 2.0)
    for _ in range(30):                                      # nudge inward (toward the crowd) until clear
        if _clear(start_xy): break
        start_xy = start_xy + d * 2.0
    for _ in range(30):
        if _clear(goal_xy): break
        goal_xy = goal_xy - d * 2.0
    route = [p3(start_xy, CRUISE_Z), p3(goal_xy, CRUISE_Z)]
    route[-1][2] = float(PLN.get("default_goal_z", CRUISE_Z))
    return route, d, left


_route0, axis, left = plan_route(0)
START = _route0[0].copy(); GOAL = _route0[-1].copy()
print(f"[3dv] route(lap0) {len(_route0)} pts, {np.round(START,1)}->{np.round(GOAL,1)} "
      f"len~{np.linalg.norm(GOAL[:2]-START[:2]):.0f}m", flush=True)

# procedural quadcopter (recognisable: body + X-frame arms + 4 rotors + red nose at +X = heading)
DroneCls = make_drone_class("Drone", span=1.1)
drone = eng.spawn_object(DroneCls, position=[float(START[0]), float(START[1])],
                         heading_theta=float(np.arctan2(axis[1], axis[0])))
drone.set_position([float(START[0]), float(START[1]), CRUISE_Z])
drone_model = getattr(drone, "_model", None)
# real quadrotor flight dynamics: SANDO set-points are TRACKED through this (tilt-to-accelerate, momentum)
quad = Quadrotor()
px4 = None
if args.px4:
    print("[3dv] --px4: connecting to PX4 SITL via MAVSDK offboard ...", flush=True)
    px4 = PX4Bridge(takeoff_alt=CRUISE_Z)
    if not px4.wait_ready(90):
        print("[3dv] PX4 offboard NOT ready (is PX4 SITL running?) — falling back to quadrotor.py", flush=True)
        px4 = None
    else:
        print("[3dv] PX4 OFFBOARD ready — flight stack = real PX4 + jMAVSim dynamics", flush=True)



class Animal:
    def __init__(self, name, p0, vel):
        glb, size, hpr = ANIMAL[name]
        self.name = name; self.size = np.array(size, float)
        self.p0 = np.asarray(p0, float); self.vel = np.asarray(vel, float); self.id = 0
        cls = make_glb_class(name, ASSETS + glb, size[0], size[1], size[2], hpr_fix=hpr)  # grounded by default
        self.obj = eng.spawn_object(cls, position=[float(p0[0]), float(p0[1])],
                                    heading_theta=float(np.arctan2(vel[1], vel[0]) if np.linalg.norm(vel) > 0 else 0.0))
    def place(self, p0, vel):
        self.p0 = np.asarray(p0, float); self.vel = np.asarray(vel, float)
    def update(self, t):
        pos = self.p0 + self.vel * t
        self.obj.set_position([float(pos[0]), float(pos[1]), 0.0])   # feet on the ground (mesh auto-grounded)


animals = []
_cow_frac = 0.5                                      # the single cow crosses the corridor at mid-span
c = START[:2] + axis * (np.linalg.norm(GOAL[:2] - START[:2]) * _cow_frac)
_cow = Animal("cow", c + left * 6.0, -left * 1.0); _cow.id = 900
animals.append(_cow)

# ---- 3D path overlay (lives in the real world, so BOTH cameras render it) -----------------------
path_root = NodePath("sando_path"); path_root.reparentTo(eng.render)
path_root.setLightOff(1)        # ignore scene lighting -> vivid colours, not washed grey
path_root.setShaderOff(1)       # scene auto-shader must not tint the overlay
path_root.setDepthTest(False)   # ALWAYS-ON-TOP: the planned path is a HUD-in-3D, never occluded
path_root.setDepthWrite(False)
path_root.setBin("fixed", 60)
pred_root = NodePath("kf_pred"); pred_root.reparentTo(eng.render)   # LIVE Kalman-prediction overlay (orange)
for _f in (lambda n: n.setLightOff(1), lambda n: n.setShaderOff(1), lambda n: n.setDepthTest(False),
           lambda n: n.setDepthWrite(False), lambda n: n.setBin("fixed", 61)):
    _f(pred_root)
GROUND_Z = 0.06                 # height of the ground-projected "racing line" of the chosen route


def _polyline(pts, color, thick):
    ls = LineSegs(); ls.setThickness(thick); ls.setColor(*color)
    for i, p in enumerate(pts):
        (ls.moveTo if i == 0 else ls.drawTo)(float(p[0]), float(p[1]), float(p[2]))
    return ls.create()


def _marker(c, color, r=0.5, thick=4.0):
    ls = LineSegs(); ls.setThickness(thick); ls.setColor(*color)
    x, y, z = float(c[0]), float(c[1]), float(c[2])
    for d in ((r, 0, 0), (0, r, 0), (0, 0, r)):
        ls.moveTo(x - d[0], y - d[1], z - d[2]); ls.drawTo(x + d[0], y + d[1], z + d[2])
    return ls.create()


def draw_path(global_path, next_goal, drone_pos):
    for ch in path_root.getChildren(): ch.removeNode()
    if global_path is not None and len(global_path) >= 2:
        path_root.attachNewNode(_polyline(global_path, (0.10, 1.0, 0.25, 1.0), 8.0))   # green = chosen route (3D)
        ground = [(p[0], p[1], GROUND_Z) for p in global_path]                         # cyan ground "racing line"
        path_root.attachNewNode(_polyline(ground, (0.0, 0.95, 1.0, 1.0), 7.0))         #   -> visible from FPV on the pavement
        for p in global_path[::2]:   # vertical posts ground->route at every other waypoint = readable from FPV too
            path_root.attachNewNode(_polyline([(p[0], p[1], GROUND_Z), (p[0], p[1], float(p[2]))],
                                              (0.2, 1.0, 0.5, 1.0), 3.0))
    if next_goal is not None:
        path_root.attachNewNode(_polyline([drone_pos, next_goal], (1.0, 0.9, 0.0, 1.0), 6.0))  # yellow look-ahead
        path_root.attachNewNode(_polyline([(next_goal[0], next_goal[1], GROUND_Z), tuple(next_goal)],
                                          (1.0, 0.1, 0.85, 1.0), 4.0))                  # magenta post under set-point
        path_root.attachNewNode(_marker(next_goal, (1.0, 0.1, 0.85, 1.0), r=0.7))      # magenta = next set-point


def draw_predictions():
    """VISIBLE PROOF the live Kalman filter is running and driving the avoidance — for EVERY moving object
    (pedestrian / vehicle / animal), not just people. Per mover: white dot = the NOISY detection the filter
    sees; orange ring = the KF-smoothed 'now'; orange arrow+ring ahead = the KF-PREDICTED position over the
    0.75 s trust horizon = where the drone routes AROUND. No KF -> only the white jitter, no orange forecast."""
    for ch in pred_root.getChildren(): ch.removeNode()
    ORG, ORGd = (1.0, 0.55, 0.0, 1.0), (1.0, 0.4, 0.0, 1.0)
    for (det_xy, now_xy, pred, hz) in _KF_PRED:
        top = max(1.8, float(hz) if hz else 1.8)
        trail = [(float(x), float(y), GROUND_Z + 0.03) for (x, y) in pred]
        if len(trail) >= 2:
            pred_root.attachNewNode(_polyline(trail, ORG, 9.0))                          # orange = KF-predicted future PATH
        ex, ey = pred[-1]
        # a tall orange PILLAR at the predicted future position = "the mover WILL be HERE" (visible in both cameras)
        pred_root.attachNewNode(_polyline([(ex, ey, GROUND_Z), (ex, ey, top)], ORGd, 7.0))
        pred_root.attachNewNode(_marker((ex, ey, top), ORGd, r=0.6, thick=7.0))          # ghost head at the forecast
        pred_root.attachNewNode(_marker((now_xy[0], now_xy[1], GROUND_Z + 0.03), (1.0, 0.8, 0.15, 1.0), r=0.35, thick=5.0))  # KF 'now'
        pred_root.attachNewNode(_marker((det_xy[0], det_xy[1], GROUND_Z + 0.06), (1.0, 1.0, 1.0, 1.0), r=0.2, thick=3.0))    # noisy detection


par = Parameters()
for k, v in PLN.items():
    if hasattr(par, k): setattr(par, k, v)
par.replan_dt = REPLAN_DT
if args.seam:
    par.seam_c2_from_state = True
    print("[3dv] --seam: seam C2-from-exec-state ON (MINCO re-anchored at the real execution state)", flush=True)

ego = None
EGO_HOR = 12.0          # EGO receding-horizon look-ahead (m): local goal clip + ground patch radius
MIN_FLY_Z = 0.7         # hard floor on the commanded altitude — the quad is never driven into the ground
# per-class certified-safety config for --ego_safe (mirrors MINCO's avoid_config default_config d_safe).
EGO_PERCLASS_DSAFE = {"static": 0.4, "wall": 0.4, "pedestrian": 0.8, "human": 0.8, "vehicle": 0.6, "animal": 0.7}
EGO_TAU_TRUST = 0.75    # s: certify the committed B-spline over [0, tau_trust] (commit-ahead trust window)
EGO_BRAKE_BAND = 0.6    # m: anticipatory brake band above d_safe -> start slowing this far before the hard stop
EGO_APPROACH_EPS = 0.2  # m/s: closing-speed threshold. A mover that is separating / co-moving (closing below
                        # this) is NOT a threat -> skip its anticipatory brake band (kills same-direction
                        # jitter); the hard d_safe stop still applies to everyone.
EGO_G_RELEASE = 0.15    # per-replan cap on how fast the speed scale RISES (brake instantly, release slowly)
# ---- SLIP space-time speed-warp knobs ----
EGO_S_MIN = 0.30        # slowest warp before we give up and hover (no certified slip-behind)
EGO_S_MAX = 1.60        # fastest warp (slip-AHEAD): fly up to 1.6x EGO's planned speed when space-time is clear
EGO_S_STEP = 0.1        # warp scan granularity
EGO_VEFF_SLIP = float(os.environ.get("EGO_VEFF", 0.25))   # KF-residual tube (covers prediction error; calibrate)
EGO_SLIP_DSAFE = float(os.environ.get("EGO_SLIPDSAFE", 0.2))   # SLIP standoff: tight like native (cert guarantees
                                                              # it, so never collides). User-tunable; only no-collision matters.
if args.ego:
    ego = EGOPlanner(map_origin=(-200, -200, -1), map_size=(400, 400, 8), res=0.2, inflation=0.3)
    ego.set_params(max_vel=float(PLN.get("v_max", 6.0)), max_acc=float(PLN.get("a_max", 10.0)),
                   ctrl_pt_dist=0.5, horizon=EGO_HOR,
                   l_collision=0.8, dist0=max(0.4, float(par.drone_radius) + 0.2))
    print(f"[3dv] --ego: EGO-Planner core active (depth-FOV: range {args.fov_range}m, +-{args.fov_deg}deg cone)", flush=True)
    if args.ego_safe:
        print("[3dv] --ego_safe: per-class certified mover safety wrapping EGO "
              f"(S3 cert, d_safe human{EGO_PERCLASS_DSAFE['human']}/vehicle{EGO_PERCLASS_DSAFE['vehicle']}/"
              f"animal{EGO_PERCLASS_DSAFE['animal']}, tau_trust={EGO_TAU_TRUST}s; uncertified commit -> RTA HOLD; "
              "static = EGO's own cloud avoidance)", flush=True)

native = None
if args.native:
    from sando_native_bridge import SandoNative   # lazy: only load the GUROBI-linked .so when actually used
    native = SandoNative(overrides={
        "v_max": float(PLN.get("v_max", 6.0)), "a_max": float(PLN.get("a_max", 12.0)),
        "x_min": -250.0, "x_max": 250.0, "y_min": -250.0, "y_max": 250.0,
        "z_min": 0.0, "z_max": float(PLN.get("z_max", 6.0)), "goal_radius": float(par.goal_radius)})
    print(f"[3dv] --native: NATIVE MIT-ACL SANDO baseline active (heat-A* + DecompUtil SFC + GUROBI; "
          f"depth-FOV range {args.fov_range}m, +-{args.fov_deg}deg)", flush=True)

PLANNER_NAME = ("EGO + no-HOLD cylinder maneuver" if (args.ego and args.maneuver) else
               "EGO + per-class safety" if (args.ego and args.ego_safe) else
               "EGO-Planner" if args.ego else "SANDO (native)" if args.native else "MINCO (ours)")
# NOTE: the default core (sando_capi.so) is OUR per-class MINCO planner, NOT the native MIT-ACL SANDO baseline
# (github.com/mit-acl/sando). "SANDO" in this repo's filenames is historical; the algorithm here is ours.


def _dt_into(sando, _cache, tid, size, pos, vel, label, t_sim, z=None):
    d = _cache.get(tid)
    if d is None:
        d = DynTraj(); d.id = int(tid); d.mode = "Analytic"; _cache[tid] = d
    zc = CRUISE_Z if z is None else float(z)
    x0, y0, vx, vy = float(pos[0]), float(pos[1]), float(vel[0]), float(vel[1])
    tx = f"{x0}+({vx})*(t-({t_sim}))"; ty = f"{y0}+({vy})*(t-({t_sim}))"; tz = f"{zc}"
    sig = (tx, ty, tz, tuple(label), float(size[0]), float(size[1]), float(size[2]))
    if getattr(d, "_sig", None) != sig:        # recompile only when the traj actually changed (statics: once)
        d.bbox = np.asarray(size, float)
        d.traj_x, d.traj_y, d.traj_z = tx, ty, tz
        d.traj_vx, d.traj_vy, d.traj_vz = f"{vx}", f"{vy}", "0.0"; d.label_set = list(label)
        d.compile_analytic(); d._sig = sig
    sando.add_traj(d, t_sim)


Z_CEIL = float(PLN.get("z_max", 6.0)) + 0.5    # only voxelise static structure up to where the drone can fly


def build_static_field():
    """Precompute (once; statics don't move) the FULL-3D occupancy of every static object — its real
    footprint W×L over its real HEIGHT (capped at the operating ceiling), so tall structure (trees, poles)
    is SOLID to the global heat-A* map (no more flying through the canopy). Returns (cloud Nx3, xy Nx2,
    objs[(tid,pos,size)], fed[(cls,c3,size)]). Uses real WLH (available headless; getTightBounds is not)."""
    pts = []; objs = []; fed = []
    for oid, cls, pos, vel, size in native_objects():
        if cls != "static": continue
        w, l, h = float(size[0]), float(size[1]), float(size[2])
        tid = abs(hash(oid)) % 1000 + 200
        objs.append((tid, np.asarray(pos, float), np.asarray(size, float)))
        fed.append(("static", p3(pos, h * 0.5), np.asarray(size, float)))   # full-height bbox for clearance
        ztop = min(h, Z_CEIL)
        # GAP-FREE voxel spacing: must be <= EGO's inflation coverage (~0.4m) so the inflated voxels overlap into
        # a SOLID wall — otherwise wide objects (tree canopies) get sparse voxels with holes the drone slips
        # through, then gets surrounded by point-blank depth and stuck. Spacing tied to object size, not a hard cap.
        VS = 0.5
        nx = max(1, min(20, int(np.ceil(w / VS)))); ny = max(1, min(20, int(np.ceil(l / VS))))
        nz = max(1, min(20, int(np.ceil(ztop / VS))))
        for ix in range(nx + 1):
            fx = pos[0] + (ix / nx - 0.5) * w
            for iy in range(ny + 1):
                fy = pos[1] + (iy / ny - 0.5) * l
                for iz in range(nz + 1):
                    pts.append((fx, fy, 0.1 + (iz / nz) * ztop))
    cloud = np.asarray(pts, float) if pts else np.zeros((0, 3))
    xy = cloud[:, :2].copy() if len(cloud) else np.zeros((0, 2))
    return cloud, xy, objs, fed


def feed(sando, _cache, t_sim, p_drone):
    fed = []
    # --- static: precomputed 3D occupancy (full height) culled by range; soft-wall DynTraj kept (height-aware) ---
    if len(STATIC_XY):
        dxy = STATIC_XY - p_drone[:2]
        cloud = STATIC_CLOUD[np.einsum('ij,ij->i', dxy, dxy) <= SENSE_R * SENSE_R]
    else:
        cloud = np.zeros((0, 3))
    for tid, pos, size in STATIC_OBJS:
        if np.linalg.norm(pos[:2] - p_drone[:2]) <= SENSE_R:
            if sando is not None:
                _dt_into(sando, _cache, tid, size, pos, np.zeros(2), [], t_sim, z=min(float(size[2]), Z_CEIL) * 0.5)
    fed.extend((c, c3, sz) for (c, c3, sz) in STATIC_FED if np.linalg.norm(c3[:2] - p_drone[:2]) <= SENSE_R)
    # --- dynamic movers (per frame) ---
    for oid, cls, pos, vel, size in native_objects():
        if cls == "static": continue
        c3 = p3(pos, size[2] * 0.5)
        if np.linalg.norm(c3[:2] - p_drone[:2]) > SENSE_R: continue
        if sando is not None:
            _dt_into(sando, _cache, abs(hash(oid)) % 1000, size, pos, vel, CLASS_LABEL[cls], t_sim)
        fed.append((cls, c3, size))
    for a in animals:
        pos = a.p0 + a.vel * t_sim; c3 = p3(pos, a.size[2] * 0.5)
        if np.linalg.norm(c3[:2] - p_drone[:2]) <= SENSE_R:
            if sando is not None:
                _dt_into(sando, _cache, a.id, a.size, pos, a.vel, CLASS_LABEL["animal"], t_sim)
            fed.append(("animal", c3, a.size))
    if sando is not None:
        sando.update_occupancy_map_ptr(cloud)
    return fed


def ego_safety_obstacles(p_d, t_sim):
    """The MOVERS near the drone as (centre3, vel3, r_obs, d_safe) for the --ego_safe per-class certify gate.
    pedestrian/vehicle/animal -> their per-class d_safe (mirrors feed()'s mover sources). This is exactly the
    per-class differentiation MINCO applies; STATIC ('wall') is NOT certified here — it stays EGO's own cloud
    avoidance job, because sphere-certifying every GT tree at canopy radius is grossly over-conservative in
    dense foliage (permanent HOLD). 'per-class' = the mover-class margins, which is what this gate adds to EGO."""
    out = []
    for oid, cls, pos, vel, size in native_objects():
        if cls == "static":
            continue
        c3 = p3(pos, size[2] * 0.5)
        if np.linalg.norm(c3[:2] - p_d[:2]) > SENSE_R:
            continue
        d = EGO_PERCLASS_DSAFE.get(cls)
        if d is None:
            continue
        out.append((c3, (float(vel[0]), float(vel[1]), 0.0), 0.5 * float(max(size[0], size[1])), d))
    for a in animals:
        pos = a.p0 + a.vel * t_sim
        c3 = p3(pos, a.size[2] * 0.5)
        if np.linalg.norm(c3[:2] - p_d[:2]) <= SENSE_R:
            out.append((c3, (float(a.vel[0]), float(a.vel[1]), 0.0),
                        0.5 * float(max(a.size[0], a.size[1])), EGO_PERCLASS_DSAFE["animal"]))
    return out


def kf_movers(p_d, t_sim):
    """Like ego_safety_obstacles, but each mover's centre + velocity come from a LIVE per-mover CA-Kalman filter
    fed NOISY detections of the GT position (this is what proves the KF is in the loop, not GT omniscience). Also
    stashes each filter's PREDICTED future trajectory into _KF_PRED for draw_predictions. Returns
    [(oid, c3_kf, vel_kf, r_obs, d_safe)]."""
    global _KF_PRED
    _KF_PRED = []
    raw = []
    for oid, cls, pos, vel, size in native_objects():
        if cls == "static":
            continue
        c3 = p3(pos, size[2] * 0.5)
        if np.linalg.norm(c3[:2] - p_d[:2]) > SENSE_R:
            continue
        d = EGO_PERCLASS_DSAFE.get(cls)
        if d is not None:
            raw.append((oid, c3, 0.5 * float(max(size[0], size[1])), d))
    for a in animals:
        pos = a.p0 + a.vel * t_sim
        c3 = p3(pos, a.size[2] * 0.5)
        if np.linalg.norm(c3[:2] - p_d[:2]) <= SENSE_R:
            raw.append((getattr(a, "id", id(a)), c3, 0.5 * float(max(a.size[0], a.size[1])), EGO_PERCLASS_DSAFE["animal"]))
    out = []
    for (oid, c3, r, d) in raw:
        det = np.asarray(c3, float) + _KF_RNG.normal(0, KF_MEAS_NOISE, 3)   # NOISY detection -> the filter's input
        trk = _KF.get(oid)
        if trk is None:
            trk = _KF[oid] = MoverTracker(dt=REPLAN_DT, meas_noise=KF_MEAS_NOISE)
        trk.update(det)
        if trk.ready:
            kc0, kv, _ka = trk.state()                                      # KF-smoothed centre + velocity
            pred = trk.predict(np.linspace(0.0, EGO_TAU_TRUST, 6))          # KF-PREDICTED future trajectory
        else:
            kc0, kv = np.asarray(c3, float), np.zeros(3)
            pred = np.asarray([kc0, kc0])
        out.append((oid, kc0, (float(kv[0]), float(kv[1]), 0.0), r, d))
        _KF_PRED.append((det[:2].copy(), kc0[:2].copy(), [(float(p[0]), float(p[1])) for p in pred], float(c3[2])))
    return out


EGO_BRAKE_LEVELS = (1.0, 0.66, 0.33, 0.0)   # anticipatory slow-down band fractions above d_safe (descending)


def ego_certify_commit(p_d, v_d, t_sim):
    """ANTICIPATORY graded per-class certify over EGO's committed B-spline. Certify each mover at several
    inflated radii (d_safe + EGO_BRAKE_BAND*frac) and grade a speed scale g in [0,1] from how deep into the
    brake band the trajectory sits: comfortably clear -> g=1; entering the band -> g ramps down (brake EARLY);
    uncertified at d_safe -> g=0 (stop). g feeds a time-warp slow-down (smooth decel along the SAME path).
    Direction-aware: a mover that is SEPARATING / co-moving (closing speed below EGO_APPROACH_EPS) gets NO
    anticipatory brake (kills same-direction jitter); only an APPROACHING mover triggers the band. The hard
    d_safe stop (g=0) still applies to everyone, approaching or not. Returns (g, worst_class)."""
    body = float(par.drone_radius)
    g = 1.0
    worst = None
    nlev = len(EGO_BRAKE_LEVELS)
    for (c0, vel, r_obs, d_safe) in ego_safety_obstacles(p_d, t_sim):
        base = r_obs + body + d_safe
        g_mover = 0.0                                   # fails even at d_safe -> stop, unless a level certifies
        for idx, frac in enumerate(EGO_BRAKE_LEVELS):   # try LARGEST inflation first
            cert, _m = ego.certify(obs_c0=c0, R=base + EGO_BRAKE_BAND * frac, obs_vel=vel, t_hi=EGO_TAU_TRUST)
            if cert:
                g_mover = (nlev - idx) / nlev           # idx levels above failed -> this deep into the band
                break
        # direction gate: closing speed = (v_drone - v_mover) . unit(mover - drone). >0 = approaching.
        rel = np.asarray(c0, float)[:2] - np.asarray(p_d, float)[:2]
        dist = float(np.linalg.norm(rel))
        closing = float(np.dot(np.asarray(v_d, float)[:2] - np.asarray(vel, float)[:2], rel / dist)) if dist > 1e-6 else 1.0
        if closing <= EGO_APPROACH_EPS and g_mover > 0.0:
            g_mover = 1.0                               # separating / co-moving and not breaching -> don't brake
        if g_mover < g:
            g = g_mover
            if g_mover < 1.0:
                worst = next((k for k, v in EGO_PERCLASS_DSAFE.items() if abs(v - d_safe) < 1e-9), "obstacle")
    return g, worst


def _slip_mover_cloud(p_d, t_sim):
    """SLIP occupancy for the movers: each CURRENT KF mover as a vertical cylinder inflated to the cert margin
    (r_obs + drone_radius + d_safe) so EGO's own route already clears what the cert demands -> the cert PASSES
    the route at full speed (no permanent hover) instead of rejecting every warp because EGO routed 0.1 m too
    tight. Current position only (no swath): the speed-warp + per-tick re-cert handle the timing; this just keeps
    EGO from planning THROUGH a mover. Capped at head height (overhead free)."""
    pts = []
    body = float(par.drone_radius)
    for (oid, c0, vel, r_obs, d_safe) in kf_movers(p_d, t_sim):
        c0 = np.asarray(c0, float)
        if np.linalg.norm(c0[:2] - p_d[:2]) > SENSE_R:
            continue
        R = r_obs + body + EGO_SLIP_DSAFE; head = 2.0 * c0[2]
        for th in np.linspace(0, 2 * np.pi, 12, endpoint=False):
            for z in np.linspace(0.3, head, 3):
                pts.append([c0[0] + R * np.cos(th), c0[1] + R * np.sin(th), z])
    return np.asarray(pts, float) if pts else np.zeros((0, 3))


def ego_speed_search(p_d, v_d, t_sim):
    """SLIP core. EGO has just committed ONE tight B-spline X(u). Pick the FASTEST scalar speed-warp s such that
    flying X at rate s is certified clear of every KF-predicted moving object, via the SOUND re-timing
    substitution: flying X(u) at rate s -> real time tau=u/s -> mover c(tau)=c0+v tau+0.5 a tau^2 becomes, in the
    trajectory's own param u, c0 + (v/s)u + 0.5(a/s^2)u^2; real horizon [0,TAU] -> param [0, s*TAU]; tube in real
    time -> v_eff/s, delta*s. So certify_horizontal(c0, R, obs_vel=v/s, obs_acc=a/s^2, t_hi=s*TAU, v_eff=VEFF/s,
    delta=REPLAN_DT*s) proves ||X(s*tau)-c(tau)||>=rho for ALL real tau in [0,TAU] — the ACTUAL flown space-time
    path. s>1 = slip-AHEAD (faster than EGO's plan); s<1 = slip-BEHIND a crosser. Returns (s_best, worst_class).
    s_best=0 -> hover (no certified warp). The flown (post-LPF) scale MUST be re-certified by the caller."""
    dur = ego.duration()
    if dur <= 1e-3:
        return 0.0, "stale"
    # kinodynamic ceiling: one eval sweep -> peak speed/accel -> how much we can warp UP before exceeding limits
    vp = ap = 1e-6
    for u in np.linspace(0.0, dur, 16):
        r = ego.eval(u)
        if r is None:
            continue
        vp = max(vp, float(np.linalg.norm(r[1]))); ap = max(ap, float(np.linalg.norm(r[2])))
    vmax = float(PLN.get("v_max", 6.0)); amax = float(PLN.get("a_max", 10.0))
    s_kino = min(vmax / vp, float(np.sqrt(amax / ap)))
    s_max = min(EGO_S_MAX, s_kino)
    body = float(par.drone_radius)
    # threatening movers only (direction gate): a separating / co-moving mover not currently breaching is skipped
    threat = []
    for (oid, c0, vel, r_obs, d_safe) in kf_movers(p_d, t_sim):
        rel = np.asarray(c0, float)[:2] - p_d[:2]; dist = float(np.linalg.norm(rel))
        closing = float(np.dot(np.asarray(v_d, float)[:2] - np.asarray(vel, float)[:2], rel / dist)) if dist > 1e-6 else 1.0
        if closing <= EGO_APPROACH_EPS and dist > r_obs + body + EGO_SLIP_DSAFE:
            continue
        threat.append((np.asarray(c0, float), np.asarray(vel, float), r_obs, d_safe))
    if not threat:
        return s_max, None                                   # nothing to yield to -> fly as fast as kinodynamics allow
    s = s_max
    while s >= EGO_S_MIN - 1e-9:
        ok = True
        for (c0, vel, r_obs, d_safe) in threat:
            R = r_obs + body + EGO_SLIP_DSAFE                # tight standoff; the v_eff tube covers KF residual
            # certify the PREDICTED moving obstacle (re-timed). NO frozen-mover variant: it certifies the mover's
            # CURRENT position, which is exactly where slip-behind passes through -> it would forbid every slip.
            # KF deviation is covered by the v_eff tube + the 0.1s per-tick re-cert (a mover can't jump in one DT).
            hp, _ = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=tuple(vel / s), obs_acc=(0, 0, 0),
                                           t_hi=s * EGO_TAU_TRUST, v_eff=EGO_VEFF_SLIP / s, delta=REPLAN_DT * s)
            if not hp:
                ok = False; break
        if ok:
            return float(s), None
        s -= EGO_S_STEP
    return 0.0, "blocked"                                    # no certified warp -> hover on the same path


def ego_slip_feasible(p_d, v_d, t_sim, s):
    """Re-certify ONE specific speed-warp s against the current KF movers (used to certify the post-LPF FLOWN
    scale: the LPF may release to a slower s that is NOT certified — for a slip-ahead mover, slowing is unsafe)."""
    if s <= 1e-3:
        return False
    body = float(par.drone_radius)
    for (oid, c0, vel, r_obs, d_safe) in kf_movers(p_d, t_sim):
        c0 = np.asarray(c0, float); vel = np.asarray(vel, float)
        rel = c0[:2] - p_d[:2]; dist = float(np.linalg.norm(rel))
        closing = float(np.dot(np.asarray(v_d, float)[:2] - vel[:2], rel / dist)) if dist > 1e-6 else 1.0
        if closing <= EGO_APPROACH_EPS and dist > r_obs + body + EGO_SLIP_DSAFE:
            continue
        R = r_obs + body + EGO_SLIP_DSAFE
        hp, _ = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=tuple(vel / s), obs_acc=(0, 0, 0),
                                       t_hi=s * EGO_TAU_TRUST, v_eff=EGO_VEFF_SLIP / s, delta=REPLAN_DT * s)
        if not hp:
            return False
    return True


MAN_REACH_PAD = 0.3   # posture/arm reach added to a mover's head-top for the fly-OVER vertical clearance
MAN_DSAFE_V = 0.5     # vertical standoff above the head
MAN_QCONF = float(os.environ.get("EGO_QCONF", 0.2))   # q_conformal keep-out covering prediction residual
# horizontal standoff ours holds from a mover (overrides the per-class 0.8). LOWER = ours flies tighter/faster
# to race the real EGO (which flies at ~0.3 and grazes); the cert still guarantees this clearance so ours never
# collides where EGO does. Tune via EGO_MANDSAFE.
MAN_DSAFE = float(os.environ.get("EGO_MANDSAFE", 0.45))
MAN_PLANHI = float(os.environ.get("EGO_PLANHI", 0.7))   # how far ahead the KF-predicted SWEPT footprint is fed to
                                                        # EGO so it weaves around the FUTURE smoothly (one trajectory,
                                                        # no late braking) instead of reacting to the present
MAN_VCRUISE = float(os.environ.get("EGO_VCRUISE", 3.5))  # drone nominal speed, for the closest-approach conflict test

# ---- LIVE KALMAN FILTER (proves the KF is actually driving the avoidance, not GT omniscience) ----
# Each tracked mover gets a CA-Kalman filter fed NOISY detections of its GT position; the filter outputs the
# smoothed centre + velocity + the predicted future trajectory c(t)=c0+v t+1/2 a t^2 that the planner routes
# AROUND and that draw_predictions() renders. Without the KF the drone would only know where people ARE, not
# where they WILL be.
from kf_tracker import MoverTracker
_KF = {}                                                    # mover id -> MoverTracker (persists across ticks)
_KF_RNG = np.random.default_rng(int(args.seed) * 7 + 1)
KF_MEAS_NOISE = float(os.environ.get("EGO_MEASNOISE", 0.10))   # detection noise (m) the filter must see through
_KF_PRED = []                                               # latest [(now_xy, [predicted xy over horizon])] for drawing
MAN_PHI = np.radians(25.0)
MAN_DEADBAND = 0.5    # hysteresis: keep the CURRENT maneuver unless another certified one beats its goal-ward
                      # speed by >this (m/s). Stops the around-L/R/over flicker that brakes-and-reaccelerates
                      # (the "hesitation") every time a mover twitches; straight resumes the moment it re-certifies.
_MAN_STATE = {"kind": None}


def _man_cloud(p_d, heading, t_sim, movers):
    """Occupancy for the maneuver planner: the FOV static voxels + ground, PLUS each mover's PREDICTED footprint
    rendered as a CYLINDER inflated to r_obs+d_safe and CAPPED at head height (so the overhead column stays free
    for fly-OVER). The d_safe inflation makes EGO's own 2-D route already clear the certificate margin, so the
    cert passes ground routes (fly fast) instead of rejecting them and forcing a constant climb."""
    pts = [ground_patch(p_d, radius=EGO_HOR + 4.0)]
    # static voxels only (drop the raw mover boxes; we re-add movers inflated + predicted below)
    stat = np.asarray(fov_cloud(p_d, heading, t_sim), float)
    if len(stat):
        pts.append(stat)
    v_nom = np.array([np.cos(heading), np.sin(heading)]) * MAN_VCRUISE   # drone's nominal motion
    for (_oid, c3, vel, r_obs, d_safe) in movers:
        # RELATIVE-MOTION timing (this is where KF prediction buys SPEED & smooth accel): block each mover at where
        # it WILL BE at the closest-approach time t_cpa of the relative motion (drone - mover), not where it is now.
        # A mover that will have swept past the corridor has its t_cpa footprint OFF the drone's path -> the drone
        # flies STRAIGHT through the gap behind it (no reactive braking) instead of detouring like native EGO. Each
        # mover is always fed exactly ONCE (at current + t_cpa) so the occupancy never toggles -> EGO stays smooth.
        dp = np.array([c3[0] - p_d[0], c3[1] - p_d[1]])
        dv = np.array([vel[0], vel[1]]) - v_nom
        dvn = float(dv @ dv)
        tcpa = float(np.clip(-(dp @ dv) / dvn, 0.0, MAN_PLANHI)) if dvn > 1e-6 else 0.0
        R = r_obs + MAN_DSAFE; head = 2.0 * c3[2]
        for lead in (0.0, tcpa):                                  # current + closest-approach predicted footprint
            cx, cy = c3[0] + vel[0] * lead, c3[1] + vel[1] * lead
            ring = [[cx + R * np.cos(a), cy + R * np.sin(a), z]
                    for a in np.linspace(0, 2 * np.pi, 10, endpoint=False) for z in np.linspace(0.3, head, 3)]
            pts.append(np.asarray(ring, float))
    return np.concatenate([p for p in pts if len(p)], axis=0)


def ego_maneuver_replan(p_d, v_d, a_d, cur_wp, t_sim):
    """WIN-EGO maneuvering: fly the GROUND route to goal when the continuous-time cylinder certificate clears it
    (= native EGO, fast), trying straight then biased around-L/R sub-goals; only when NO ground route certifies
    (a wall) fly OVER (certified); if boxed, climb straight up. The d_safe-inflated predicted mover occupancy
    (_man_cloud) keeps EGO's own routes certifiable, so it weaves on the ground like EGO instead of climbing
    everything. Returns (kind, traj_pts)."""
    # depth-FOV is aimed along the GOAL direction (where the drone is going), not the lagging body yaw, so the
    # forward obstacles EGO must route around are actually in view.
    fdir = np.asarray(cur_wp, float)[:2] - p_d[:2]
    heading = float(np.arctan2(fdir[1], fdir[0])) if np.linalg.norm(fdir) > 1e-3 else 0.0
    movers = kf_movers(p_d, t_sim)                         # (oid, KF-centre3, KF-vel3, r_obs, d_safe) — LIVE Kalman
    ego.update_cloud(_man_cloud(p_d, heading, t_sim, movers), p_d)
    gxy = np.asarray(cur_wp, float)[:2] - p_d[:2]; dist = float(np.linalg.norm(gxy))
    gdir = gxy / dist if dist > 1e-6 else np.array([1.0, 0.0])
    L = min(EGO_HOR, max(dist, 1.0))
    zc = [2.0 * c3[2] + MAN_REACH_PAD + MAN_DSAFE_V for (_oid, c3, _, _, _) in movers]
    z_top = min(Z_CEIL, (max(zc) if zc else CRUISE_Z + 1.0) + 0.2)

    def rot(v, ang):
        c, s = np.cos(ang), np.sin(ang); return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])

    def cert_clear():                                      # CURRENTLY-held EGO B-spline vs every mover
        for (_oid, c3, vel, r_obs, d_safe) in movers:
            R = r_obs + MAN_DSAFE + MAN_QCONF
            hp, _ = ego.certify_horizontal(obs_c0=c3, R=R, obs_vel=vel, t_hi=EGO_TAU_TRUST, v_eff=0.2, delta=REPLAN_DT)
            hc, _ = ego.certify_horizontal(obs_c0=c3, R=R, obs_vel=(0, 0, 0), t_hi=EGO_TAU_TRUST, v_eff=0.2, delta=REPLAN_DT)
            vo, _ = ego.certify_above(z_clear=2.0 * c3[2] + MAN_REACH_PAD + MAN_DSAFE_V + MAN_QCONF,
                                      t_hi=EGO_TAU_TRUST, delta=REPLAN_DT)
            if not ((hp and hc) or vo):
                return False
        return True

    # ONE smooth EGO trajectory straight to the goal at cruise, routing around the KF-PREDICTED future occupancy
    # (the swept footprint already in the grid). Because EGO weaves around where movers WILL be — not where they
    # ARE — the single B-spline stays smooth and never brakes late, so the acceleration is graceful and the drone
    # is FASTER than reactive native. No candidate switching = no chopped-up jerky path. The certificate is a guard.
    chosen = "straight"
    ego.replan(p_d, v_d, a_d, np.array([cur_wp[0], cur_wp[1], CRUISE_Z]))
    if ego.duration() <= 1e-3 or not cert_clear():
        if ego.replan(p_d, v_d, a_d, np.array([cur_wp[0], cur_wp[1], z_top])) and ego.duration() > 1e-3 and cert_clear():
            chosen = "over"                                   # ground blocked -> fly OVER (certified)
        else:
            ego.replan(p_d, v_d, a_d, np.array([p_d[0], p_d[1], z_top])); chosen = "climb"   # boxed -> climb up
    _MAN_STATE["kind"] = chosen
    dur = ego.duration()
    pts = [ego.eval(s)[0] for s in np.linspace(0, dur, 24)] if dur > 1e-3 else None
    return chosen, pts


def _voxel_box(pos, size, t_sim=0.0, vel=(0, 0)):
    """coarse point grid over a mover's bbox (centered at pos, height size[2]) — its 'depth' surface."""
    p = np.asarray(pos, float)[:2] + np.asarray(vel, float)[:2] * t_sim
    w, l, h = float(size[0]), float(size[1]), float(size[2])
    out = []
    for ix in (-0.5, 0, 0.5):
        for iy in (-0.5, 0, 0.5):
            for iz in (0.2, 0.5, 0.8):
                out.append((p[0] + ix * w, p[1] + iy * l, iz * h))
    return out


def fov_cloud(p_drone, heading, t_sim):
    """The drone's DEPTH-camera FOV point cloud (NOT GT omniscience): static voxels + mover boxes within
    the forward cone (+-fov_deg) and range fov_range of the heading. This is what --ego perceives."""
    R = float(args.fov_range); cmax = float(np.cos(np.radians(args.fov_deg)))
    fwd = np.array([np.cos(heading), np.sin(heading)])
    chunks = []
    if len(STATIC_CLOUD):
        d = STATIC_CLOUD[:, :2] - p_drone[:2]; dist = np.linalg.norm(d, axis=1)
        with np.errstate(invalid="ignore", divide="ignore"):
            cang = (d @ fwd) / np.maximum(dist, 1e-6)
        chunks.append(STATIC_CLOUD[(dist <= R) & (cang >= cmax)])

    def _seen(c):
        dd = c[:2] - p_drone[:2]; r = float(np.linalg.norm(dd))
        return r <= R and (float(dd @ fwd) / max(r, 1e-6)) >= cmax
    for oid, cls, pos, vel, size in native_objects():
        if cls == "static": continue
        if _seen(p3(pos, size[2] * 0.5)): chunks.append(np.asarray(_voxel_box(pos, size), float))
    for a in animals:
        pos = a.p0 + a.vel * t_sim
        if _seen(p3(pos, a.size[2] * 0.5)): chunks.append(np.asarray(_voxel_box(pos, a.size), float))
    chunks = [c for c in chunks if len(c)]
    return np.concatenate(chunks, axis=0) if chunks else np.zeros((0, 3))


_GP_CACHE = {}
def ground_patch(p_drone, radius=16.0, step=1.0, z=0.0):
    """A ground-plane patch (z=0) under/around the drone so EGO never routes BELOW the floor.
    Ground knowledge is NOT FOV-limited (a drone always senses the floor via its down-facing sensor/altimeter),
    so this is added on top of the depth-FOV cloud, not gated by the forward cone."""
    key = (round(radius, 1), round(step, 2))
    base = _GP_CACHE.get(key)
    if base is None:
        xs = np.arange(-radius, radius + 1e-6, step)
        gx, gy = np.meshgrid(xs, xs)
        m = (gx * gx + gy * gy) <= radius * radius
        base = np.column_stack([gx[m], gy[m]]); _GP_CACHE[key] = base
    out = np.empty((base.shape[0], 3))
    out[:, 0] = base[:, 0] + p_drone[0]; out[:, 1] = base[:, 1] + p_drone[1]; out[:, 2] = z
    return out


def _add_native_traj(tid, size, pos, vel, cls, t_sim):
    """feed one dynamic obstacle to native SANDO as an analytic DynTraj (same exprtk strings as MINCO).
    native expects bbox = obstacle_half_extent + drone_half (node convention: msg.bbox/2 + drone_bbox/2)."""
    x0, y0 = float(pos[0]), float(pos[1]); vx, vy = float(vel[0]), float(vel[1])
    tx = f"{x0}+({vx})*(t-({t_sim}))"; ty = f"{y0}+({vy})*(t-({t_sim}))"; tz = f"{CRUISE_Z}"
    bb = (size[0] * 0.5 + 0.1, size[1] * 0.5 + 0.1, size[2] * 0.5 + 0.1)
    native.add_traj(tid, bb, tx, ty, tz, f"{vx}", f"{vy}", "0.0", is_agent=(cls == "pedestrian"), t=t_sim)


def feed_native(t_sim, p_drone):
    """Feed native SANDO this frame: static = depth-FOV cloud + ground (FOV-limited, NOT GT); dynamic
    movers/animals = analytic DynTraj. Returns the `fed` list for the clearance metric (same shape as feed())."""
    heading = float(quad.yaw)
    cloud = np.concatenate([fov_cloud(p_drone, heading, t_sim), ground_patch(p_drone, radius=EGO_HOR + 4.0)], axis=0)
    native.update_occupancy(cloud, t_sim)
    fed = []
    for oid, cls, pos, vel, size in native_objects():
        if cls == "static": continue
        c3 = p3(pos, size[2] * 0.5)
        if np.linalg.norm(c3[:2] - p_drone[:2]) > SENSE_R: continue
        _add_native_traj(abs(hash(oid)) % 100000, size, pos, vel, cls, t_sim)
        fed.append((cls, c3, size))
    for a in animals:
        pos = a.p0 + a.vel * t_sim; c3 = p3(pos, a.size[2] * 0.5)
        if np.linalg.norm(c3[:2] - p_drone[:2]) <= SENSE_R:
            _add_native_traj(100000 + a.id, a.size, pos, a.vel, "animal", t_sim)
            fed.append(("animal", c3, a.size))
    native.clean_old_trajs(t_sim)
    return fed


def clearance(p, fed):
    r = float(par.drone_radius); gmin = np.inf; per = {}
    for cls, c3, size in fed:
        d = p - c3; halfb = 0.5 * np.asarray(size, float)
        outside = np.maximum(np.abs(d) - halfb, 0.0)
        sd = (np.linalg.norm(outside) if np.any(outside > 0) else -np.min(halfb - np.abs(d))) - r
        gmin = min(gmin, sd); per[cls] = min(per.get(cls, np.inf), sd)
    return gmin, per


FONT = cv2.FONT_HERSHEY_SIMPLEX


def grab(pos, hpr):
    """offscreen GPU frame at (pos,hpr) rel. to the drone (BGR uint8). ONE renderFrame per view
    (multi_thread_render=False makes a single pass correct) instead of perceive()'s 2x taskMgr.step."""
    cam.cam.reparentTo(drone.origin)
    cam.cam.setPos(Vec3(*pos)); cam.cam.setHpr(Vec3(*hpr))
    eng.graphicsEngine.renderFrame()
    a = np.asarray(cam.get_rgb_array_cpu())
    if a.dtype != np.uint8:
        a = (a * 255).astype(np.uint8) if a.max() <= 1.01 else a.astype(np.uint8)
    return a


def grab_views():
    """Return {name: BGR frame}. FPV hides the drone body so it doesn't fill the lens."""
    out = {}
    if args.view in ("fpv", "dual"):
        if drone_model is not None: drone_model.hide()
        out["fpv"] = grab(FPV_POS, FPV_HPR)
        if drone_model is not None: drone_model.show()
    if args.view in ("chase", "dual"):
        out["chase"] = grab(CHASE_POS, CHASE_HPR)
    return out


def compose(views, t, z, mclr, per, status, seam=None, ego_info=None):
    panels = []
    for key, label in (("fpv", "FPV  onboard"), ("chase", "3rd PERSON  chase")):
        if key in views:
            f = np.ascontiguousarray(views[key])
            cv2.rectangle(f, (0, 0), (f.shape[1], 26), (0, 0, 0), -1)
            cv2.putText(f, label, (8, 19), FONT, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
            panels.append(f)
    canvas = np.hstack(panels) if len(panels) > 1 else panels[0]
    W = canvas.shape[1]
    bar = np.zeros((58 if (seam is None and ego_info is None) else 80, W, 3), np.uint8)   # +1 line for seam/ego readout
    cv2.putText(bar, f"{PLANNER_NAME} 3D real-time   t={t:4.1f}s   alt z={z:.2f}m   status={status}   "
                     f"minClr={(mclr if mclr < 1e8 else 0):+.2f}m   {'COLLIDED' if mclr < 0 else 'CLEAR'}",
                (10, 22), FONT, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    pc = "  ".join(f"{k}:{per.get(k, float('nan')):+.2f}" for k in ("static", "pedestrian", "vehicle", "animal"))
    legend = "[green=route  ORANGE=Kalman forecast (where movers WILL be)  white=noisy detection]" if args.maneuver \
        else "[green=route  yellow=look-ahead  magenta=next set-point]"
    cv2.putText(bar, "per-class clr  " + pc + "    " + legend,
                (10, 46), FONT, 0.48, (210, 210, 210), 1, cv2.LINE_AA)
    if seam is not None:                                            # seam C2-from-exec-state readout (own line)
        on = seam.get("on", False)
        txt = (f"seam C2: ON   bias={seam['bias']:.2f}m   (re-anchored at real exec state -> flown == certified)"
               if on else "seam C2: OFF   (certified line may be ~0.1-0.2m optimistic vs the flown path)")
        cv2.putText(bar, txt, (10, 70), FONT, 0.48,
                    (0, 230, 0) if on else (150, 150, 150), 1, cv2.LINE_AA)
    if ego_info is not None:                                        # --ego_safe per-class graded-brake readout
        g = ego_info.get("g", 1.0); cls = ego_info.get("cls", "obstacle")
        tot = ego_info['n_cert'] + ego_info['n_slow'] + ego_info['n_hold']
        if ego_info.get("hold", False):
            txt = f"EgoSafe per-class: HOLD  (uncertified vs {cls}; smooth stop -> climb-over)"
            col = (0, 120, 255)                                     # red-amber = full stop
        elif g < 0.999:
            txt = f"EgoSafe per-class: BRAKING  speed x{g:.2f}  (anticipating {cls}, smooth slow-down)"
            col = (0, 200, 255)                                     # amber = anticipatory braking
        else:
            txt = "EgoSafe per-class: CERTIFIED  P(hit mover)<=eps  (human0.8/veh0.6/animal0.7)"
            col = (0, 230, 0)
        cv2.putText(bar, txt + f"   [cert {ego_info['n_cert']} brake {ego_info['n_slow']} hold {ego_info['n_hold']}]",
                    (10, 70), FONT, 0.46, col, 1, cv2.LINE_AA)
    return np.vstack([canvas, bar])


def make_sando(start, goal):
    s = SANDO(par)
    r = RobotState(); r.pos = np.asarray(start, float).copy(); r.vel = np.zeros(3); s.update_state(r)
    s.update_occupancy_map_ptr(np.zeros((0, 3)))
    g = RobotState(); g.pos = np.asarray(goal, float).copy(); s.set_terminal_goal(g)
    return s


def set_goal(s, goal):
    g = RobotState(); g.pos = np.asarray(goal, float).copy(); s.set_terminal_goal(g)


def step_env():
    try: env.step([0.0, 0.0])
    except Exception: pass   # ego may "arrive_dest"; we don't care, keep rendering the scene


DT = float(par.dc); T_MAX = float(args.t_max)
os.makedirs(os.path.join(_HERE, "out"), exist_ok=True)
writer = imageio.get_writer(os.path.join(_HERE, "out", "drone_3d.mp4"), fps=args.fps) if args.mp4 else None
WIN = "MetaUrban x SANDO — REAL-TIME 3D (FPV + 3rd person + planned path)"

# ---- real-time MJPEG-over-HTTP view (robust where cv2 windows are black on software GL) ----------
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

_FRAME = {"jpg": None}
_FRAME_LOCK = threading.Lock()
_PAGE = (b"<!doctype html><html><head><title>SANDO 3D real-time</title></head>"
         b"<body style='margin:0;background:#101014;text-align:center'>"
         b"<img src='/stream' style='max-width:100vw;max-height:100vh;image-rendering:auto'></body></html>")


class _MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200); self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(_PAGE))); self.end_headers()
            self.wfile.write(_PAGE); return
        if self.path == "/snapshot.jpg":
            with _FRAME_LOCK: jpg = _FRAME["jpg"]
            if jpg is None: self.send_response(503); self.end_headers(); return
            self.send_response(200); self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg))); self.end_headers()
            self.wfile.write(jpg); return
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache"); self.end_headers()
            try:
                while True:
                    with _FRAME_LOCK: jpg = _FRAME["jpg"]
                    if jpg is not None:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    time.sleep(0.04)
            except (BrokenPipeError, ConnectionResetError, OSError): return
        self.send_response(404); self.end_headers()


def publish(frame_bgr):
    ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if ok:
        with _FRAME_LOCK: _FRAME["jpg"] = buf.tobytes()


_httpd = None
if args.serve:
    _httpd = ThreadingHTTPServer(("0.0.0.0", args.port), _MJPEGHandler)
    threading.Thread(target=_httpd.serve_forever, daemon=True).start()
    print(f"[3dv] >>> REAL-TIME view ready: open  http://localhost:{args.port}/  in a browser "
          f"(MJPEG stream; snapshot at /snapshot.jpg) <<<", flush=True)

print(f"[3dv] {'SERVE http://localhost:%d' % args.port if args.serve else ('LIVE window' if args.live else ('frame_only' if args.frame_only else 'recording'))}"
      f"  view={args.view}  {'(ESC/q to quit)' if args.live else ''}", flush=True)

STATIC_CLOUD, STATIC_XY, STATIC_OBJS, STATIC_FED = build_static_field()
print(f"[3dv] static field: {len(STATIC_CLOUD)} voxels / {len(STATIC_OBJS)} objects (full-3D up to {Z_CEIL:.1f}m)", flush=True)

lap_idx = 0
quit_now = False
while not quit_now:
    route, axis, left = plan_route(lap_idx)
    # clear-spawn guard: plan_route's own _clear only checks native_objects' "static", NOT the full STATIC
    # field (trees/posts) -> the drone can spawn inside foliage (e.g. seed 11). Re-roll the route with a
    # bumped sub-seed until the START is genuinely clear by the REAL clearance() (which sees the whole field
    # incl. trees). Keep the best attempt if none reaches the target. Same seed+flag -> deterministic, so an
    # A/B (raw vs safe) still gets the identical route.
    if args.clear_spawn:
        SPAWN_CLR_MIN = 1.5
        best = (-1e9, route, axis, left)
        for att in range(16):
            s0 = np.asarray(route[0], float)
            c0, _ = clearance(s0, feed(None, {}, 0.0, s0))
            if c0 > best[0]: best = (c0, route, axis, left)
            if c0 >= SPAWN_CLR_MIN: break
            route, axis, left = plan_route(lap_idx + (att + 1) * 7919)   # different start/route, same seed family
        c_best, route, axis, left = best
        print(f"[3dv] clear_spawn: START spawn clearance {c_best:+.2f}m (target >= {SPAWN_CLR_MIN}m)", flush=True)
    lap_idx += 1
    START = route[0]; GOAL = route[-1]
    wp = [np.asarray(q, float) for q in route[1:]]   # ordered goals: intermediate waypoints + final goal
    wp_i = 0
    hdg0 = float(np.arctan2(axis[1], axis[0]))
    if px4 is not None:
        if lap_idx == 1: px4.set_world_origin(START)      # anchor world<->NED once; drone flies continuously
        lap_start = px4.get_pose_world()[0] if lap_idx > 1 else START
    else:
        drone.set_position([float(START[0]), float(START[1]), CRUISE_Z]); drone.set_heading_theta(hdg0)
        lap_start = START
    _mid = 0.5 * (START[:2] + GOAL[:2])
    _cow.place(_mid + left * 6.0, -left * 1.0)        # cow crosses near mid-route
    sando = make_sando(lap_start, wp[0]) if (ego is None and native is None) else None; _cache = {}
    if native is not None: native.set_terminal_goal(wp[0])
    quad.reset(np.asarray(lap_start, float), yaw=hdg0)
    if px4 is not None:
        p_d, _yw0, v_d, _ = px4.get_pose_world(); p_d = np.asarray(p_d, float); v_d = np.asarray(v_d, float)
    else:
        p_d = quad.p.copy(); v_d = np.zeros(3)
    a_d = np.zeros(3)
    t = 0.0; last_rt = 0.0; reached = False; mclr = np.inf; per_all = {}; iters = 0; seam_bias_max = 0.0
    ego_n_cert = 0; ego_n_hold = 0; ego_n_slow = 0; ego_cert_hold = False; ego_hold_class = None
    ego_speed_g = 1.0; ego_g_prev = 1.0   # --ego_safe graded anticipatory-brake speed scale [0,1] (+ release LPF)
    next_goal_pos = None; _last_wall = time.perf_counter()
    print(f"[3dv] lap {lap_idx-1}: {len(route)}-pt route len~{np.linalg.norm(GOAL[:2]-START[:2]):.0f}m "
          f"start {np.round(START[:2],1)}", flush=True)
    ego_dur = 0.0; t_ego = 0.0; ego_stuck = 0; ego_traj_pts = None; man_kind = None
    man_switches = 0; man_counts = {}; _prev_mk = None; _MAN_STATE["kind"] = None   # reset hysteresis per lap
    while t < T_MAX and not reached:
        cur_wp = wp[wp_i]
        if ego is not None:
            # EGO-Planner core: perceive ONLY the depth-camera FOV cloud (not GT omniscience) PLUS the ground
            # plane (floor knowledge isn't FOV-limited), aim at the waypoint clipped to the receding horizon.
            heading = float(quad.yaw)
            cloud = np.concatenate([fov_cloud(p_d, heading, t), ground_patch(p_d, radius=EGO_HOR + 4.0)], axis=0)
            if args.slip:                                              # route EGO around movers at the cert margin
                cloud = np.concatenate([cloud, _slip_mover_cloud(p_d, t)], axis=0)
            ego.update_cloud(cloud, p_d)
            man_kind = None
            if args.maneuver:
                # NO-HOLD cylinder fastest-safe tournament (fly over / around / climb); leaves EGO holding the winner
                t0 = time.perf_counter(); man_kind, ego_traj_pts2 = ego_maneuver_replan(p_d, v_d, a_d, cur_wp, t)
                last_rt = time.perf_counter() - t0
                ego_dur = ego.duration(); ego_ok = ego_dur > 1e-3
            else:
                to_wp = cur_wp[:2] - p_d[:2]; dwp = float(np.linalg.norm(to_wp))
                lg2 = (p_d[:2] + to_wp / max(dwp, 1e-6) * EGO_HOR) if dwp > EGO_HOR else cur_wp[:2]
                local_goal = np.array([lg2[0], lg2[1], CRUISE_Z], float)
                t0 = time.perf_counter(); ego_ok = ego.replan(p_d, v_d, a_d, local_goal)
                last_rt = time.perf_counter() - t0
                ego_dur = ego.duration()                   # EGO retains the last good traj even when replan fails
            if ego_ok and ego_dur > 1e-3:
                ego_stuck = 0; t_ego = 0.0                 # fresh plan -> restart from its head
                ego_traj_pts = [ego.eval(s)[0] for s in np.linspace(0, ego_dur, 24)]   # the REAL EGO B-spline
            else:
                ego_stuck += 1                             # keep executing the last good plan; only recover if stuck
            fed = feed(None, _cache, t, p_d)
            # per-class certified RTA gate: certify the committed B-spline vs every obstacle with its
            # per-class d_safe; uncertified -> HOLD (force the executor's hover/climb-over branch). This
            # gives EGO the per-class certified safety MINCO has (EGO alone has no per-class margin).
            ego_cert_hold = False; ego_hold_class = None; ego_speed_g = 1.0
            if args.ego_safe and ego_dur > 1e-3:
                g_raw, wc = ego_certify_commit(p_d, v_d, t)                  # direction-aware graded brake (S3 cert)
                # anti-chatter: brake immediately (g drops free), release slowly (rate-limited rise)
                ego_speed_g = g_raw if g_raw < ego_g_prev else min(g_raw, ego_g_prev + EGO_G_RELEASE)
                ego_g_prev = ego_speed_g
                # NOTE: static obstacles are deliberately NOT gated here. A current-FOV clearance gate is both
                # over-conservative (holds on forward obstacles it would safely pass) and under-protective
                # (a tree beside/behind the drone leaves the +-45deg cone -> unseen), and a tight corridor is
                # simply infeasible at a hard 0.4m margin (MINCO gets stuck there too). Static stays EGO's own
                # cloud avoidance; a proper static-hard needs accumulated occupancy MEMORY, not a per-frame gate.
                if ego_speed_g >= 0.999:
                    ego_n_cert += 1                          # comfortably certified -> full speed
                elif ego_speed_g <= 1e-3:
                    ego_cert_hold = True; ego_hold_class = wc; ego_n_hold += 1; ego_stuck += 1   # hard stop
                else:
                    ego_hold_class = wc; ego_n_slow += 1     # anticipatory brake: slowing, not stopped
            elif args.slip and ego_dur > 1e-3:
                # SLIP: fastest space-time speed-warp the cert allows (>1 = slip-AHEAD, faster than EGO's plan).
                s_raw, wc = ego_speed_search(p_d, v_d, t)
                g = s_raw if s_raw < ego_g_prev else min(s_raw, ego_g_prev + EGO_G_RELEASE)   # brake free, release slow
                while g >= EGO_S_MIN - 1e-9 and not ego_slip_feasible(p_d, v_d, t, g):         # RE-CERT THE FLOWN SCALE
                    g -= EGO_S_STEP
                ego_speed_g = g if g >= EGO_S_MIN - 1e-9 else 0.0
                ego_g_prev = ego_speed_g
                if ego_speed_g <= 1e-3:
                    # blocked by a mover -> HOVER IN PLACE and wait for it to pass; do NOT increment ego_stuck
                    # (that would trigger the static-entrapment climb recovery, which strands the drone at altitude
                    # where the horizontal cert still rejects). The mover layer waits + resumes; it never climbs.
                    ego_cert_hold = True; ego_hold_class = "slip"; ego_n_hold += 1
                elif ego_speed_g >= 0.999:
                    ego_n_cert += 1                          # slip-ahead / clear -> at or above EGO's planned speed
                else:
                    ego_hold_class = "yield"; ego_n_slow += 1   # slip-behind a crosser
        elif native is not None:
            # NATIVE MIT-ACL SANDO: feed state + depth-FOV occupancy + analytic DynTraj, then replan
            # (heat-A* global -> DecompUtil safe-flight-corridor -> GUROBI local).
            native.update_state(p_d, v_d, a_d, float(quad.yaw))
            fed = feed_native(t, p_d)
            t0 = time.perf_counter(); native.replan(last_rt, t); last_rt = time.perf_counter() - t0
        else:
            st = RobotState(); st.pos = p_d.copy(); st.vel = v_d.copy(); st.accel = a_d.copy()
            sando.update_state(st)
            fed = feed(sando, _cache, t, p_d)
            t0 = time.perf_counter(); sando.replan(last_rt, t); last_rt = time.perf_counter() - t0
        if px4 is not None:
            # REAL PX4: stream the committed set-point to offboard, pace to wall-clock (PX4 is real-time),
            # read PX4's fused pose back to drive the render. PX4 = inner control + jMAVSim dynamics.
            yaw_ref = float(np.arctan2(cur_wp[1] - p_d[1], cur_wp[0] - p_d[0]))
            ok, ng = sando.get_next_goal()
            if ok: next_goal_pos = np.asarray(ng.pos, float).copy()
            px4.set_setpoint(next_goal_pos if next_goal_pos is not None else cur_wp, yaw_ref)
            dtw = time.perf_counter() - _last_wall
            if dtw < REPLAN_DT: time.sleep(REPLAN_DT - dtw)
            _last_wall = time.perf_counter(); t += max(dtw, REPLAN_DT)
            pw, yw, vw, _ok = px4.get_pose_world()
            p_d = np.asarray(pw, float); v_d = np.asarray(vw, float); a_d = np.zeros(3)
            drone.set_position([float(p_d[0]), float(p_d[1]), float(p_d[2])])
            drone.set_heading_theta(float(yw))
        elif ego is not None:
            for _ in range(int(round(REPLAN_DT / DT))):
                yaw_ref = float(np.arctan2(cur_wp[1] - quad.p[1], cur_wp[0] - quad.p[0]))
                # --ego_safe ANTICIPATORY BRAKE: advance along the SAME B-spline path at the graded rate
                # ego_speed_g (time-warp). g<1 -> slower (smooth deceleration as a mover gets close);
                # g==0 -> ego_cert_hold -> s=None -> hover (already near-stopped, so no jerk), climb if stuck.
                s = (ego.eval(min(t_ego + ego_speed_g * DT, max(ego_dur - 1e-3, 0.0)))
                     if (ego_dur > 1e-3 and not ego_cert_hold) else None)
                if s is not None:
                    t_ego += ego_speed_g * DT
                    sp_pos, sp_vel, sp_acc = s; sp_pos = np.asarray(sp_pos, float).copy()
                    sp_vel = np.asarray(sp_vel, float) * ego_speed_g            # feed-forward vel matches the warp
                    sp_acc = np.asarray(sp_acc, float) * (ego_speed_g ** 2)
                    if args.slip:                                              # SLIP g>1 (slip-ahead): hard-cap to limits
                        _vmx = float(PLN.get("v_max", 6.0)); _amx = float(PLN.get("a_max", 10.0))
                        _nv = float(np.linalg.norm(sp_vel)); _na = float(np.linalg.norm(sp_acc))
                        if _nv > _vmx: sp_vel = sp_vel * (_vmx / _nv)
                        if _na > _amx: sp_acc = sp_acc * (_amx / _na)
                    if sp_pos[2] < MIN_FLY_Z:                       # never command the quad into the ground
                        sp_pos[2] = MIN_FLY_Z
                        if sp_vel[2] < 0: sp_vel = sp_vel.copy(); sp_vel[2] = 0.0
                    next_goal_pos = sp_pos.copy()
                    quad.step(sp_pos, sp_vel, sp_acc, DT, yaw_ref=yaw_ref)   # quad tracks the EGO B-spline (slowed)
                elif ego_stuck < 3:
                    quad.step(quad.p, np.zeros(3), np.zeros(3), DT, yaw_ref=yaw_ref)   # transient miss -> hover
                else:
                    # SUSTAINED stuck (surrounded / inside a tree): RECOVERY = climb to clear the canopy and ease
                    # toward the goal instead of freezing. Space above the voxel ceiling is free. Gentle climb.
                    climb = min(Z_CEIL + 1.0, quad.p[2] + 0.6)
                    nd = cur_wp[:2] - quad.p[:2]; ndn = float(np.linalg.norm(nd))
                    rec = np.array([quad.p[0] + (nd[0] / ndn * 0.3 if ndn > 1e-3 else 0.0),
                                    quad.p[1] + (nd[1] / ndn * 0.3 if ndn > 1e-3 else 0.0), climb], float)
                    next_goal_pos = rec.copy()
                    quad.step(rec, np.zeros(3), np.zeros(3), DT, yaw_ref=yaw_ref)
                t += DT
            p_d = quad.p.copy(); v_d = quad.v.copy(); a_d = quad.a.copy()
            drone.set_position([float(p_d[0]), float(p_d[1]), float(p_d[2])])
            drone.set_heading_theta(float(quad.yaw))
            if drone_model is not None:
                _pitch, _roll = quad.tilt_deg(); drone_model.setHpr(0.0, _pitch, _roll)
        elif native is not None:
            for _ in range(int(round(REPLAN_DT / DT))):
                yaw_ref = float(np.arctan2(cur_wp[1] - quad.p[1], cur_wp[0] - quad.p[0]))
                ok, ng = native.get_next_goal()   # (pos, vel, accel) from native SANDO's committed traj
                if ok:
                    sp_pos = np.asarray(ng[0], float).copy()
                    if sp_pos[2] < MIN_FLY_Z: sp_pos[2] = MIN_FLY_Z
                    next_goal_pos = sp_pos.copy()
                    quad.step(sp_pos, ng[1], ng[2], DT, yaw_ref=yaw_ref)   # quad tracks native set-point
                else:
                    quad.step(quad.p, np.zeros(3), np.zeros(3), DT, yaw_ref=yaw_ref)
                t += DT
            p_d = quad.p.copy(); v_d = quad.v.copy(); a_d = quad.a.copy()
            drone.set_position([float(p_d[0]), float(p_d[1]), float(p_d[2])])
            drone.set_heading_theta(float(quad.yaw))
            if drone_model is not None:
                _pitch, _roll = quad.tilt_deg(); drone_model.setHpr(0.0, _pitch, _roll)
        else:
            for _ in range(int(round(REPLAN_DT / DT))):
                yaw_ref = float(np.arctan2(cur_wp[1] - quad.p[1], cur_wp[0] - quad.p[0]))   # face current waypoint
                ok, ng = sando.get_next_goal()
                if ok:
                    next_goal_pos = np.asarray(ng.pos, float).copy()
                    quad.step(ng.pos, ng.vel, ng.accel, DT, yaw_ref=yaw_ref)   # track set-point thru quad dynamics
                else:
                    quad.step(quad.p, np.zeros(3), np.zeros(3), DT, yaw_ref=yaw_ref)
                t += DT
            p_d = quad.p.copy(); v_d = quad.v.copy(); a_d = quad.a.copy()   # TRUE quad state -> fed back to SANDO
            drone.set_position([float(p_d[0]), float(p_d[1]), float(p_d[2])])
            drone.set_heading_theta(float(quad.yaw))
            if drone_model is not None:
                _pitch, _roll = quad.tilt_deg(); drone_model.setHpr(0.0, _pitch, _roll)   # visible quadrotor tilt
        if args.maneuver and man_kind is not None:
            man_counts[man_kind] = man_counts.get(man_kind, 0) + 1
            if _prev_mk is not None and man_kind != _prev_mk:
                man_switches += 1                           # maneuver-kind change = a brake/re-accel (the "flicker")
            _prev_mk = man_kind
        for a in animals: a.update(t)
        # advance through the random waypoints (don't stop at intermediate ones)
        if wp_i < len(wp) - 1 and np.linalg.norm(p_d[:2] - wp[wp_i][:2]) < 3.5:
            wp_i += 1
            if sando is not None: set_goal(sando, wp[wp_i])
            elif native is not None: native.set_terminal_goal(wp[wp_i])
        if sando is not None:
            gpath = sando.get_global_path()
        elif native is not None:
            sp = native.get_setpoints()                # native's committed local trajectory (the chosen path)
            gpath = sp if len(sp) >= 2 else native.get_global_path()
        elif ego_traj_pts is not None:
            gpath = ego_traj_pts                       # <- the ACTUAL EGO B-spline trajectory (sampled)
        else:
            gpath = [p_d] + [np.array([w[0], w[1], CRUISE_Z], float) for w in wp[wp_i:]]
        draw_path(gpath, next_goal_pos, p_d)   # <- the path the drone just chose
        if args.maneuver or args.slip:
            draw_predictions()                 # <- the LIVE Kalman forecast every mover is routed around
        step_env()
        c, per = clearance(p_d, fed); mclr = min(mclr, c)
        for k, val in per.items(): per_all[k] = min(per_all.get(k, np.inf), val)
        views = grab_views()
        status = (sando.get_drone_status() if sando is not None
                  else native.get_drone_status() if native is not None
                  else ("REACHED" if reached else
                        (man_kind.upper() if (args.maneuver and man_kind) else "EGO")))
        seam_hud = ({"on": bool(args.seam), "bias": float(np.linalg.norm(sando.get_seam_bias()))}
                    if sando is not None else None)
        if seam_hud is not None: seam_bias_max = max(seam_bias_max, seam_hud["bias"])
        ego_hud = ({"hold": ego_cert_hold, "g": ego_speed_g, "cls": ego_hold_class,
                    "n_cert": ego_n_cert, "n_slow": ego_n_slow, "n_hold": ego_n_hold}
                   if (ego is not None and args.ego_safe) else None)
        frame = compose(views, t, p_d[2], mclr, per, status, seam=seam_hud, ego_info=ego_hud)
        if writer is not None: writer.append_data(frame[..., ::-1])   # BGR->RGB for imageio
        if args.serve: publish(frame)                                  # push to the live MJPEG stream
        if args.frame_only:
            iters += 1
            if iters >= 16:   # fly a few seconds so a real path + motion is on screen, then dump
                gp = sando.get_global_path() if sando is not None else gpath
                hdg = float(np.arctan2(v_d[1], v_d[0]))
                print(f"[dbg] drone={np.round(p_d,2)} vel={np.round(v_d,2)} hdg={np.degrees(hdg):.0f}deg "
                      f"origin_H={drone.origin.getH():.0f}deg", flush=True)
                print(f"[dbg] GOAL={np.round(GOAL,2)} gp[0]={np.round(gp[0],2)} gp[-1]={np.round(gp[-1],2)} "
                      f"n={len(gp)}", flush=True)
                if len(gp) >= 2:
                    d0 = np.asarray(gp[0])[:2] - p_d[:2]; dN = np.asarray(gp[-1])[:2] - p_d[:2]
                    print(f"[dbg] dir_to_gp0={np.round(d0,2)} dir_to_gpEnd={np.round(dN,2)} "
                          f"fwd=({np.cos(hdg):.2f},{np.sin(hdg):.2f})", flush=True)
                imageio.imwrite(os.path.join(_HERE, "out", "drone_3d_frame.png"), frame[..., ::-1])
                print(f"[3dv] wrote out/drone_3d_frame.png (global_path pts={len(gp)})", flush=True)
                if writer is not None: writer.close()
                env.close(); sys.exit(0)
        if args.live:
            cv2.imshow(WIN, frame)
            if (cv2.waitKey(1) & 0xFF) in (27, ord('q')): quit_now = True; break
        final_close = (wp_i == len(wp) - 1 and np.linalg.norm(p_d - GOAL) < float(par.goal_radius))
        if final_close and (sando is None or sando.get_drone_status() == GOAL_REACHED):
            reached = True
    t_goal = t if reached else float("inf")
    print(f"[3dv] lap done. reached={reached} t_goal={t_goal:.1f}s collided={mclr < 0} min_clr={mclr:.3f}m  "
          + "  ".join(f"{k}:{v:.2f}" for k, v in sorted(per_all.items()))
          + (f"  seam_bias_max={seam_bias_max:.3f}m" if args.seam else "")
          + (f"  maneuver[switches={man_switches} " + " ".join(f"{k}:{v}" for k, v in sorted(man_counts.items())) + "]"
             if args.maneuver else "")
          + (f"  egosafe[cert={ego_n_cert} brake={ego_n_slow} hold={ego_n_hold}]" if (args.ego and args.ego_safe) else ""), flush=True)
    # --serve and live+loop_scene keep flying laps forever; everything else stops after one lap
    if not (args.serve or (args.live and args.loop_scene)):
        break
if writer is not None:
    writer.close(); print(f"[3dv] mp4 -> {os.path.join(_HERE, 'out', 'drone_3d.mp4')}", flush=True)
try: cv2.destroyAllWindows()
except Exception: pass
env.close()
