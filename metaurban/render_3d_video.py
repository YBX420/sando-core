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
import os, sys, time, argparse, random, copy
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
from metaurban.component.sensors.depth_camera import DepthCamera   # D435i depth perception input (--d435i)
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
ap.add_argument("--d435i", action="store_true", help="use a REAL Intel RealSense D435i depth camera as the perception INPUT: mount a D435i-spec DepthCamera (FOV 87x58, range ~0.3-8m, axial noise) on the drone nose, render the depth image, deproject to a world point cloud, and feed THAT to EGO instead of the GT-omniscient fov_cloud. Occlusion + range + noise = sim2real-faithful perception. Same point-cloud interface accepts a real D435i via pyrealsense2. Needs --ego")
ap.add_argument("--seam", action="store_true", help="enable seam C2-from-exec-state (A4): re-anchor each MINCO solve at the drone's real execution state so 'what flies == what is certified' (our MINCO core only)")
ap.add_argument("--ego_safe", action="store_true", help="wrap EGO with MINCO's per-class certified MOVER safety: certify EGO's committed B-spline (S3 continuous-time deficit) against each detected mover with per-class d_safe (human0.8/vehicle0.6/animal0.7); uncertified commit -> RTA HOLD. Static stays EGO's own cloud avoidance. Needs --ego")
ap.add_argument("--maneuver", action="store_true", help="NO-HOLD CYLINDER maneuvering (M3): each step, run a fastest-safe candidate tournament (straight/around-L/R/over/climb sub-goals), certify each committed B-spline vs every mover with the CYLINDER disjunction (horizontal sqrt(dx^2+dy^2)>=r+d_safe OR vertical p_z>=z_clear), and FLY the certified candidate with the most goal-ward speed. Fly OVER a wall, AROUND a crosser, climb as the no-freeze escape. Needs --ego; replaces --ego_safe's HOLD")
ap.add_argument("--slip", action="store_true", help="SLIP (space-time speed-warp): plan ONE tight near-native EGO path, then fly it at the FASTEST scalar speed-warp s whose RE-TIMED flight the continuous-time cert proves clears every KF-predicted moving object (s>1 slip-AHEAD = faster than EGO, s<1 slip-BEHIND a crosser). KF relative velocity optimizes the acceleration; no re-route, no climb. Needs --ego")
ap.add_argument("--clear_spawn", action="store_true", help="re-roll the route until the drone's START is genuinely clear of static obstacles (full field incl. trees), so it never spawns inside foliage. Deterministic per seed, so A/B stays controlled")
ap.add_argument("--mp4", action="store_true", help="also write out/drone_3d.mp4")
ap.add_argument("--headless", action="store_true", help="run the SAME MetaUrban sim + planner + safety layer + real-quad dynamics but SKIP all 3-D rendering (no grab_views/compose/path overlays) -> fast headless-on-MetaUrban; the lap-done reach/collision/clearance numbers are byte-identical to the rendered run (same scenario, just no pixels). Incompatible with --d435i (which needs the depth camera), --mp4/--live/--serve.")
ap.add_argument("--pointmass", action="store_true", help="DIAGNOSTIC: fly the commanded set-point exactly (teleport) instead of through the real quadrotor dynamics -> flown==planned, isolates how much of the collision gap is plan->flown TRACKING error vs planning/perception.")
ap.add_argument("--loop_scene", action="store_true", help="restart the fly-through forever for continuous live viewing")
ap.add_argument("--scenario", type=str, default=None,
                help="sando-scenario-v1 JSON: SCRIPTED movers (compiled tracks) replace/augment the native "
                     "crowd, and the drone flies the scenario's start->goal corridor. THE bridge from the "
                     "scenario workbench into this full-sensor renderer (D435i/cone/KF views/AB).")
ap.add_argument("--no_crowd", action="store_true",
                help="minimal native crowd (deterministic benchmark: scripted movers only; ORCA needs >=1 human)")
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

_SCN = None
if args.scenario:
    import scenario_lib as _SLB
    _SCN = _SLB.load(args.scenario)

env_cfg = dict(
    crswalk_density=1, object_density=0.9, walk_on_all_regions=False,   # DENSE scene -> the avoider has real work
    use_render=False, image_observation=(not args.headless),   # --headless: no camera -> no GL, faster startup
    sensors=(dict() if args.headless
             else dict(rgb_camera=(RGBCamera, args.w, args.h), d435i_depth=(DepthCamera, 160, 106)) if args.d435i
             else dict(rgb_camera=(RGBCamera, args.w, args.h))),
    interface_panel=[], manual_control=False, map=os.environ.get("MU_MAP", "X"), daytime="12:00",
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
    spawn_human_num=(1 if args.no_crowd else 85), spawn_wheelchairman_num=(0 if args.no_crowd else 5),
    spawn_edog_num=(0 if args.no_crowd else 6), spawn_erobot_num=(0 if args.no_crowd else 3),
    spawn_drobot_num=(0 if args.no_crowd else 3), max_actor_num=(2 if args.no_crowd else 170))

print("[3dv] constructing offscreen env ...", flush=True)
env = SidewalkDynamicMetaUrbanEnv(env_cfg)
# MetaUrban only has num_scenarios scenes -> the SCENE index must wrap into [0, num_scenarios). The full
# args.seed still drives plan_route's RNG (seed*1000), so seeds >= num_scenarios reuse a scene but get a
# DIFFERENT route (more variety without crashing on env.reset's [0:N) assertion).
_NUM_SCENARIOS = 20
env.reset(seed=args.seed % _NUM_SCENARIOS)
for _ in range(8): env.step([0.0, 0.0])
eng = env.engine; agent_ego = env.agent
cam = None if args.headless else eng.get_sensor("rgb_camera")


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
    if _SCN is not None:                                     # scenario corridor overrides the random route
        st = np.asarray(_SCN["drone"]["start"], float); gl = np.asarray(_SCN["drone"]["goal"], float)
        d = gl[:2] - st[:2]; L = max(1e-6, float(np.linalg.norm(d))); d = d / L
        return [p3(st[:2], CRUISE_Z), p3(gl[:2], CRUISE_Z)], d, np.array([-d[1], d[0]])
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
    # MAP-EDGE CLAMP (2026-07-08): a crowd cluster near the scene rim used to shoot the route
    # endpoints onto the map edge (start y~-60 = flying the void). Pull any endpoint that strays
    # beyond MAP_R of the walkable anchor (the agent spawn) back along the route axis.
    _anchor = np.asarray(ctr, float)[:2]   # crowd-cluster centre: it LIVES on the sidewalk network
    _map_r = float(os.environ.get("MAP_R", 28.0))
    for _pt, _sgn in ((start_xy, +1.0), (goal_xy, -1.0)):
        for _ in range(40):
            if float(np.linalg.norm(_pt - _anchor)) <= _map_r: break
            _pt += d * _sgn * 2.0
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

# --- D435i depth camera as perception INPUT (--d435i): real rendered depth -> deprojected world cloud -> EGO ---
d435 = None
if args.d435i:
    from d435i_sensor import D435iDepth
    d435 = D435iDepth(eng, width=160, height=106, zmax=float(args.fov_range), noise=True)
    print(f"[3dv] --d435i: D435i depth camera perception (FOV 87x58, range {args.fov_range}m, axial noise) "
          f"replaces the GT fov_cloud -> EGO sees occlusion-limited surfaces only", flush=True)

def d435i_cloud(p_d, heading):
    """Real D435i depth -> world point cloud, AIMED ALONG `heading` (the planned travel/goal direction), not the
    lagging body yaw. A real go-around yaws its perception toward where it is flying; bolting the depth cam to the
    body yaw makes it stare BACKWARD during a climb/turn (frac_ahead -> 0), and EGO then plans against a cloud that
    is behind the drone -> optimise fails -> climbs more -> yaw swings further: a divergent loop. Mounting on the
    world origin at the nose offset rotated by `heading` keeps the depth FOV pointed where the planner is going,
    exactly like fov_cloud(p_d, heading, t) does for the GT cone. Clipped to the [0.2, 3.0] m obstacle band."""
    hd = float(heading)
    ch, sh = np.cos(hd), np.sin(hd)
    wpos = (float(p_d[0]) + FPV_POS[0] * ch, float(p_d[1]) + FPV_POS[0] * sh, float(p_d[2]) + FPV_POS[2])
    whpr = (np.degrees(hd) + FPV_HPR[0], FPV_HPR[1], FPV_HPR[2])   # world heading - 90 -> cam +Y looks along heading
    c = d435.cloud_world(eng.origin, position=wpos, hpr=whpr, subsample=2, z_floor=0.2, z_ceil=3.0)
    # VOXEL-DOWNSAMPLE to ~one point per D435I_VOX m cell. The raw depth cloud is ~4000 dense surface points;
    # EGO's A* front-end has a hard 0.2 s search budget and TIMES OUT on that density (-> "a star error" -> every
    # straight/over/climb candidate fails to plan -> erratic altitude + no progress). GT's fov_cloud is a few
    # hundred coarse voxels and A* is instant. Downsampling to GT-like density keeps the obstacle geometry while
    # making A* tractable; the certificate still runs on the committed B-spline, so safety is unaffected.
    if len(c):
        vox = float(os.environ.get("D435I_VOX", 0.25))
        keys = np.floor(c / vox).astype(np.int64)
        _, keep = np.unique(keys, axis=0, return_index=True)
        c = c[keep]
    if os.environ.get("D435I_DEBUG") == "1" and len(c):
        dxy = np.linalg.norm(c[:, :2] - np.asarray(p_d, float)[:2], axis=1)
        fwd = np.array([ch, sh])
        ahead = ((c[:, :2] - np.asarray(p_d, float)[:2]) @ fwd) / np.maximum(dxy, 1e-6)
        print(f"[d435dbg] n={len(c)} dist[min/med/max]={dxy.min():.2f}/{np.median(dxy):.2f}/{dxy.max():.2f} "
              f"z[{c[:,2].min():.2f},{c[:,2].max():.2f}] frac_ahead={np.mean(ahead>0):.2f} "
              f"frac_within1m={np.mean(dxy<1.0):.2f}", flush=True)
    return c if len(c) else np.zeros((0, 3))


# ---- D435i OCCUPANCY MEMORY: remember the static structure already driven past (closes the occlusion gap vs GT) --
# The depth camera only sees FRONT surfaces inside an 87deg/8m frustum. Without memory the drone forgets a static
# obstacle the instant it leaves the FOV and clips its unseen side/back (static collisions GT avoids because GT
# knows the whole STATIC_CLOUD). Accumulating each frame's cloud in the WORLD frame gives the planner the same
# persistent static map. Mover surfaces are dropped at accumulation time (the KF cylinder layer handles movers;
# remembering them would smear phantom walls along their trails). Voxel-deduped + bounded to a radius -> A* stays
# fast. Reset per lap.
_OCC_MEM = np.zeros((0, 3))
_OCC_VOX = float(os.environ.get("OCC_VOX", 0.25))
_OCC_R = float(os.environ.get("OCC_R", 22.0))     # keep memory within this radius of the drone


def occ_reset():
    global _OCC_MEM
    _OCC_MEM = np.zeros((0, 3))


_OCC_CAP = int(os.environ.get("OCC_CAP", 20000))       # v2: HARD density cap (v1 over-densified and
_OCC_TTL = float(os.environ.get("OCC_TTL", 30.0))       #     walled seed 5 in); unconditional TTL (s)
_OCC_T = [0.0]


def occ_remember(new_pts, p_d, movers, t_sim=None):
    """v2 online static memory (real-deployment mapping): accumulate d435i clouds minus mover
    surfaces, voxel-dedup, HARD point cap (v1's unbounded densification walled the drone in on
    seed 5) and an UNCONDITIONAL per-point TTL (a stale wall eventually re-earns its existence by
    being re-observed; killing only-in-cone points left phantom walls behind the drone forever)."""
    global _OCC_MEM, _OCC_AGE
    t_now = float(t_sim if t_sim is not None else _OCC_T[0]); _OCC_T[0] = t_now
    new_pts = np.asarray(new_pts, float).reshape(-1, 3)
    if len(new_pts) and movers:
        keep = np.ones(len(new_pts), bool)
        for (_oid, c3, _vel, r_obs, _d) in movers:
            keep &= np.linalg.norm(new_pts[:, :2] - np.asarray(c3, float)[:2], axis=1) > (float(r_obs) + 0.5)
        new_pts = new_pts[keep]
    if "_OCC_AGE" not in globals() or len(_OCC_AGE) != len(_OCC_MEM):
        _OCC_AGE = np.full(len(_OCC_MEM), t_now)
    allpts = np.vstack([_OCC_MEM, new_pts]) if (len(_OCC_MEM) and len(new_pts)) else (
        new_pts if len(new_pts) else _OCC_MEM)
    ages = np.concatenate([_OCC_AGE, np.full(len(new_pts), t_now)]) if len(new_pts) else _OCC_AGE
    if not len(allpts):
        _OCC_MEM = allpts; _OCC_AGE = ages; return allpts
    d = np.linalg.norm(allpts[:, :2] - np.asarray(p_d, float)[:2], axis=1)
    m = (d <= _OCC_R) & (t_now - ages <= _OCC_TTL)          # bound footprint + unconditional TTL
    allpts, ages = allpts[m], ages[m]
    keys = np.floor(allpts / _OCC_VOX).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    allpts, ages = allpts[idx], ages[idx]
    if len(allpts) > _OCC_CAP:                              # hard density cap: keep the FRESHEST
        order = np.argsort(-ages)[:_OCC_CAP]
        allpts, ages = allpts[order], ages[order]
    _OCC_MEM = allpts; _OCC_AGE = ages
    return _OCC_MEM


# real quadrotor flight dynamics: SANDO set-points are TRACKED through this (tilt-to-accelerate, momentum)

_YAW_ST = [None]


def _yaw_smooth(raw):
    """Yaw-reference rate limiter (YAW_SLEW deg/tick; yaw never enters the certificate -- the quad
    is rotationally symmetric -- so this is free comfort: kills the near-waypoint heading flapping
    that read as high-frequency wobble even at 3 m/s, revs=49)."""
    lim = float(os.environ.get("YAW_SLEW", "0"))
    if lim <= 0:
        return raw
    prev = _YAW_ST[0]
    if prev is None:
        _YAW_ST[0] = raw
        return raw
    d = (raw - prev + np.pi) % (2 * np.pi) - np.pi
    step = np.clip(d, -np.radians(lim), np.radians(lim))
    _YAW_ST[0] = prev + step
    return float(_YAW_ST[0])



def _telem_row(t, tp, tr, quad_, t_sim):
    """Extended telemetry row: (t, pitch, roll, |v|, x, y, z, cpd, dxn, dyn, dvxn, dvyn) --
    cpd = centre distance to nearest non-static mover; dn*/dvn* = that mover's relative position
    and velocity (raw material for PSC/CPD/TTC standard comfort metrics; radii convention doc'd
    in docs/comfort-metrics-adoption.md)."""
    best = (1e9, 0.0, 0.0, 0.0, 0.0)
    for _oid, _c, _p, _v, _s in native_objects():
        if _c == "static":
            continue
        dx, dy = float(_p[0] - quad_.p[0]), float(_p[1] - quad_.p[1])
        dd = float(np.hypot(dx, dy))
        if dd < best[0]:
            best = (dd, dx, dy, float(_v[0] - quad_.v[0]), float(_v[1] - quad_.v[1]))
    return (float(t), tp, tr, float(np.linalg.norm(quad_.v)),
            float(quad_.p[0]), float(quad_.p[1]), float(quad_.p[2]),
            round(best[0], 3), round(best[1], 3), round(best[2], 3),
            round(best[3], 3), round(best[4], 3))

quad = Quadrotor()
_TELEM = [] if os.environ.get("TELEM_OUT") else None   # (t, pitch, roll, speed) per tick
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


class ScriptedMover:
    """Scenario-JSON mover riding the EXISTING `animals` duck-type: every consumer evaluates
    `a.p0 + a.vel * t_sim`, so update(t) re-linearizes the compiled track at the current tick
    (p0 = pos(t) - vel*t) and ALL six consumption sites (cert, occupancy, native feed, fov cloud,
    clearance) work unchanged. cls_name carries the true class for per-class d_safe."""
    _next_id = 5000

    def __init__(self, spec, track):
        import scenario_lib as _SLB
        self.cls_name = spec.get("cls", "animal")
        self.track = track
        r = float(spec.get("r") or _SLB.CLS_R.get(self.cls_name, 0.4))
        h = float(spec.get("h") or _SLB.CLS_H.get(self.cls_name, 1.6))
        self.size = np.array([2 * r, 2 * r, h], float)
        self.id = ScriptedMover._next_id; ScriptedMover._next_id += 1
        x0, y0 = float(track["xy"][0][0]), float(track["xy"][0][1])
        self.p0 = np.array([x0, y0]); self.vel = np.zeros(2)
        glb, sz, hpr = _SCRIPTED_VISUAL.get(self.cls_name, _SCRIPTED_VISUAL["animal"])
        cls_obj = make_glb_class(f"scn_{self.cls_name}_{self.id}", glb,
                                 self.size[0], self.size[1], self.size[2], hpr_fix=hpr)
        self.obj = eng.spawn_object(cls_obj, position=[x0, y0], heading_theta=0.0)

    def update(self, t):
        tr = self.track
        if t > float(tr["t"][-1]) + 1e-6:                    # despawned: park far away, out of SENSE_R
            self.p0 = np.array([9e3, 9e3]); self.vel = np.zeros(2)
            self.obj.set_position([9e3, 9e3, 0.0])
            return
        x = float(np.interp(t, tr["t"], tr["xy"][:, 0])); y = float(np.interp(t, tr["t"], tr["xy"][:, 1]))
        t2 = min(t + 0.2, float(tr["t"][-1]))
        x2 = float(np.interp(t2, tr["t"], tr["xy"][:, 0])); y2 = float(np.interp(t2, tr["t"], tr["xy"][:, 1]))
        v = (np.array([x2, y2]) - np.array([x, y])) / max(1e-6, t2 - t) if t2 > t else np.zeros(2)
        self.vel = v
        self.p0 = np.array([x, y]) - v * t                   # linearization: p0 + vel*t == pos(t)
        self.obj.set_position([x, y, 0.0])
        if float(np.hypot(*v)) > 0.15:
            self.obj.set_heading_theta(float(np.arctan2(v[1], v[0])))


_MU_TEST = "/media/boxuan/Data2/projects/metaurban/metaurban/assets/models/test/"
_MU_PED = "/media/boxuan/Data2/projects/metaurban/metaurban/assets/models/pedestrian/scene.gltf"
_SCRIPTED_VISUAL = {   # cls -> (glb path, default size, hpr_fix); sizes overridden per-spec above
    "pedestrian": (_MU_PED, [0.5, 0.5, 1.75], (0.0, 0.0, 0.0)),
    "vehicle": (_MU_TEST + "car-334ebc41e59747898f612b38cef4aa7a.glb", [1.8, 4.4, 1.5], (0.0, 0.0, 0.0)),
    "animal": (ASSETS + "cow_quaternius.glb", [0.9, 2.6, 1.6], (0.0, -90.0, 0.0)),
    "static": (_MU_TEST + "Bollard-0b87b812751d4545943ef436c5f997a8.glb", [0.8, 0.8, 3.0], (0.0, 0.0, 0.0)),
}

if _SCN is not None:
    _raw = _SLB.to_movers_raw(_SCN)
    _specs = list(_SCN.get("movers", [])) + [dict(cls="static", r=st.get("r"), h=st.get("h"))
                                             for st in _SCN.get("statics", [])]
    for _spec, _tr in zip(_specs, _raw):
        animals.append(ScriptedMover(_spec, _tr))
    print(f"[3dv] scenario '{_SCN['name']}': {len(_raw)} scripted movers grafted onto the animals path",
          flush=True)
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
    for _kp in _KF_PRED:
        (det_xy, now_xy, pred, hz) = _kp[:4]
        ring = _kp[4] if len(_kp) > 4 else None            # v5 ELLIPSE arm: keep-out footprint outline
        if ring:
            pred_root.attachNewNode(_polyline([(x, y, GROUND_Z + 0.05) for (x, y) in ring],
                                              (0.15, 0.9, 0.9, 1.0), 5.0))   # cyan ellipse = certified keep-out
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
    _zsz = float(os.environ.get("OVER_Z", 0)) + 1.0 if os.environ.get("OVER_Z") else 8.0
    ego = EGOPlanner(map_origin=(-200, -200, -1), map_size=(400, 400, max(8.0, _zsz)), res=0.2, inflation=0.3)
    # EGO_VMAX env override: lower v_max so the drone does not OUTRUN its forward cone (8m cone / 8 m/s = ~1s lookahead
    # -> fast-flight-into-late-detected-obstacle collisions). A reaction-feasible cap is the stable global half of the
    # speed-FOV coupling (the per-tick _path_free_dist warp is the dynamic half).
    _vmax_req = float(os.environ.get("EGO_VMAX", PLN.get("v_max", 6.0)))
    if os.environ.get("V_CAP", "0") == "1":
        import safety_layer as _SLc
        _vmax_req = min(_vmax_req, _SLc.v_cap(args.fov_range, PLN.get("a_max", 10.0), REPLAN_DT))
    ego.set_params(max_vel=_vmax_req,
                   max_acc=float(PLN.get("a_max", 10.0)),
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


Z_CEIL = max(float(PLN.get("z_max", 6.0)), float(os.environ.get("OVER_Z", 0.0))) + 0.5   # OVER_Z lifts
#   the flight ceiling (user 2026-07-08: unlimited transit z, mission-controlled GOAL z -> crowds
#   escaped over the top instead of held)


def build_static_field():
    """Precompute (once; statics don't move) the FULL-3D occupancy of every static object — its real
    footprint W×L over its real HEIGHT (capped at the operating ceiling), so tall structure (trees, poles)
    is SOLID to the global heat-A* map (no more flying through the canopy). Returns (cloud Nx3, xy Nx2,
    objs[(tid,pos,size)], fed[(cls,c3,size)]). Uses real WLH (available headless; getTightBounds is not)."""
    pts = []; objs = []; fed = []
    for oid, cls, pos, vel, size in native_objects():
        if cls != "static": continue
        w, l, h = float(size[0]), float(size[1]), float(size[2])
        # SQUARE footprint at max(W,L): the declared WLH under-counts the W direction (e.g. 0.90 vs the real mesh
        # 1.17 from getTightBounds) and is rectangular while trees/poles are ROUND. Using a max(W,L) square
        # conservatively covers the true round canopy from ANY approach angle, in BOTH the clearance metric and the
        # voxelised safety field (getTightBounds is render-only, so this is the headless-safe accurate model).
        s = max(w, l); w = l = s
        size = np.array([s, s, h], float)
        tid = abs(hash(oid)) % 1000 + 200
        objs.append((tid, np.asarray(pos, float), size.copy()))
        fed.append(("static", p3(pos, h * 0.5), size.copy()))   # full-height square footprint for clearance
        ztop = min(h, Z_CEIL)
        # GAP-FREE voxel spacing: must be <= EGO's inflation coverage (~0.4m) so the inflated voxels overlap into
        # a SOLID wall — otherwise wide objects (tree canopies) get sparse voxels with holes the drone slips
        # through, then gets surrounded by point-blank depth and stuck. Spacing tied to object size, not a hard cap.
        VS = 0.5
        nx = max(1, min(20, int(np.ceil(w / VS)))); ny = max(1, min(20, int(np.ceil(l / VS))))
        nz = max(1, min(20, int(np.ceil(ztop / VS))))
        R2 = (0.5 * s) ** 2                                # voxelise a CYLINDER (drop the box corners) -> matches the
        for ix in range(nx + 1):                          # round mesh; the drone may safely use the corner gaps
            fx = pos[0] + (ix / nx - 0.5) * w
            for iy in range(ny + 1):
                fy = pos[1] + (iy / ny - 0.5) * l
                if (fx - pos[0]) ** 2 + (fy - pos[1]) ** 2 > R2:
                    continue
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
                _dt_into(sando, _cache, a.id, a.size, pos, a.vel,
                         CLASS_LABEL.get(getattr(a, "cls_name", "animal"), CLASS_LABEL["animal"]), t_sim)
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
                        0.5 * float(max(a.size[0], a.size[1])),
                        EGO_PERCLASS_DSAFE.get(getattr(a, "cls_name", "animal"), 0.7)))
    return out


# --- SAME forward-cone sensor model native's fov_cloud uses (range args.fov_range, +-args.fov_deg about a heading) ---
# Ours and native MUST perceive through the identical cone so the only A/B variable is the safety layer, not the FOV.
def _cone_mask(pts_xy, p_d, heading):
    R = float(args.fov_range); cmax = float(np.cos(np.radians(args.fov_deg)))
    fwd = np.array([np.cos(heading), np.sin(heading)])
    d = np.asarray(pts_xy, float) - p_d[:2]; dist = np.linalg.norm(d, axis=1)
    with np.errstate(invalid="ignore", divide="ignore"):
        cang = (d @ fwd) / np.maximum(dist, 1e-6)
    return (dist <= R) & (cang >= cmax)


def _cone_static(p_d, heading):
    """Static points inside the forward depth cone (NOT the 360-degree omniscient map)."""
    return STATIC_CLOUD[_cone_mask(STATIC_CLOUD[:, :2], p_d, heading)] if len(STATIC_CLOUD) else np.zeros((0, 3))


def _local_static(p_d):
    """Static within EGO_HOR+4 radius (360-degree local KNOWN map). A real drone in a known environment HAS the static
    map (SLAM/prior/floorplan) and does NOT forget walls it flew past -- forward-cone-only static makes side-blind
    grazes physically unavoidable. Movers stay forward-cone (the dynamic unknown)."""
    if not len(STATIC_CLOUD):
        return np.zeros((0, 3))
    dxy = STATIC_CLOUD[:, :2] - p_d[:2]
    return STATIC_CLOUD[np.einsum('ij,ij->i', dxy, dxy) <= (EGO_HOR + 4.0) ** 2]


# EGO_STATIC_MAP=1: BOTH ours and native perceive static as the local KNOWN map (fair + realistic); movers stay
# forward-cone for both. =0: forward-cone static (the strict single-depth-cam setup). The only A/B variable stays the
# safety layer either way.
STATIC_MAP = os.environ.get("EGO_STATIC_MAP", "0") == "1"   # REAL-DEPLOYMENT DEFAULT (2026-07-03):
# no prior map -- the map is BUILT (cone observation + occ_remember v2 memory). EGO_STATIC_MAP=1
# restores the old 360-degree known-map convenience as an explicit CONTROL condition.   # default ON: known static map (both ours+native, fair+real)


def _percept_static(p_d, heading):
    return _local_static(p_d) if STATIC_MAP else _cone_static(p_d, heading)


def _in_cone(c3, p_d, heading):
    R = float(args.fov_range); cmax = float(np.cos(np.radians(args.fov_deg)))
    fwd = np.array([np.cos(heading), np.sin(heading)])
    dd = np.asarray(c3, float)[:2] - p_d[:2]; r = float(np.linalg.norm(dd))
    return r <= R and (float(dd @ fwd) / max(r, 1e-6)) >= cmax


# --- SPEED-FOV COUPLING ("don't outrun the sensor"): cap flown speed by the free distance the forward cone sees ---
FOV_ABRAKE = float(os.environ.get("EGO_FOV_ABRAKE", 4.0))    # m/s^2 brake decel assumed for the stopping-distance cap
FOV_DMARGIN = float(os.environ.get("EGO_FOV_DMARGIN", 1.2))  # m: must stop this far short of a perceived obstacle surface
FOV_GMIN = float(os.environ.get("EGO_FOV_GMIN", 0.15))       # floor on the speed-warp (keep crawling, never dead-stop here)


def _path_free_dist(p_d, heading, t_sim):
    """Arc-length along the committed B-spline before it first comes within FOV_DMARGIN of any perceived (forward-cone)
    obstacle surface = the distance the drone may fly before it MUST already have avoided. inf if the committed path
    stays clear through the lookahead (then no slow-down). This is the 'don't outrun the sensor' distance: cap the
    flown speed so the braking distance fits inside it. Uses the SAME forward cone (fov_cloud) the planner perceives."""
    dur = ego.duration()
    if dur <= 1e-3:
        return np.inf
    fc = fov_cloud(p_d, heading, t_sim)
    if not len(fc):
        return np.inf
    obs = fc[:, :2]
    acc = 0.0; prev = np.asarray(ego.eval(0.0)[0], float)[:2]
    for s in np.linspace(0.0, min(dur, EGO_TAU_TRUST + 0.5), 12)[1:]:
        p = np.asarray(ego.eval(s)[0], float)[:2]
        acc += float(np.linalg.norm(p - prev)); prev = p
        if float(np.min(np.linalg.norm(obs - p, axis=1))) < FOV_DMARGIN:
            return acc
    return np.inf


# A/B ablation knob: EGO_PREDICT=0 makes ours REACTIVE (no KF forecast) -- each mover is certified as STATIC at its
# current KF-smoothed centre, isolating the contribution of PREDICTION itself (same cone, noise, planner, everything else).
EGO_PREDICT = os.environ.get("EGO_PREDICT", "1") == "1"


_PFE_MEMO = {"t": None, "out": None, "pred": None}
_GT_VELFD = {}   # GT oid -> (last_pos2, last_t): finite-diff true velocity (engine o.velocity is 0 for humanoids)
_ORA_MAP = {}    # track id -> GT oid matched while detected (oracle's omniscient coast during miss ticks)


def _kf_movers_realistic(p_d, t_sim, cam_heading):
    """PERCEPT=realistic twin of kf_movers on the cone path: same output contract
    [(oid, c3_kf, vel3, r_eff, d_safe)] + _KF_PRED, but detections come from the shared perception
    front-end (occlusion + miss + noise + NN association -- track ids, NOT GT object ids). Coasting
    READY tracks inflate their keep-out by MAN_MEM_K * pos_sigma exactly like the GT path's track
    memory; a coasting NOT-ready track (single glimpse, huge two-point prior sigma) is NOT emitted,
    mirroring the GT memory loop's `if not trk.ready: continue` -- otherwise one noisy glimpse casts
    a ~+8 m phantom keep-out the very next tick. MEMOIZED per t_sim: debug re-queries in the same
    tick (MAN_COLLDBG, slip) must not advance the stateful front-end twice.
    v1 honesty note: occluders are the MOVER cylinders only; static buildings do not occlude yet."""
    global _KF_PRED
    if _PFE_MEMO["t"] == t_sim:
        _KF_PRED = _PFE_MEMO["pred"]
        return _PFE_MEMO["out"]
    _KF_PRED = []
    cyls = []
    gtl = []                                                # GT (pos2, vel2) rows aligned with cyls (oracle/debug only)
    for _oid, cls, pos, vel, size in native_objects():
        if cls == "static" or EGO_PERCLASS_DSAFE.get(cls) is None:
            continue
        p2 = np.asarray(pos[:2], float)
        cyls.append((p2, 0.5 * float(max(size[0], size[1])), float(size[2]), cls))
        # engine o.velocity is 0 for animation-driven humanoids -> true velocity = GT position finite-diff
        prev = _GT_VELFD.get(_oid)
        v_fd = (p2 - prev[0]) / (t_sim - prev[1]) if (prev is not None and t_sim > prev[1]) else np.zeros(2)
        _GT_VELFD[_oid] = (p2.copy(), t_sim)
        gtl.append((_oid, p2, v_fd))
    for a in animals:
        pos = a.p0 + a.vel * t_sim
        cyls.append((np.asarray(pos[:2], float), 0.5 * float(max(a.size[0], a.size[1])), float(a.size[2]),
                     getattr(a, "cls_name", "animal")))
        gtl.append((getattr(a, "id", id(a)), np.asarray(pos[:2], float), np.asarray(a.vel[:2], float)))
    if GT_ORACLE:                                           # full-field omniscience: bypass the sensor front-end
        out = []                                            # entirely (no cone/range/occlusion/miss, no KF, no
        for (xy, r, h, cls), (oid, p2, v2) in zip(cyls, gtl):   # track-maturity gates) -- every mover in SENSE_R
            if float(np.hypot(*(p2 - p_d[:2]))) > SENSE_R:      # is tracked exactly from tick 0
                continue
            zc = 0.5 * float(h)
            kc0 = np.array([p2[0], p2[1], zc])
            kv = np.array([v2[0], v2[1], 0.0])
            if not EGO_PREDICT:
                kv = np.zeros(3)
            pred = kc0[None, :] + np.linspace(0.0, EGO_TAU_TRUST, 6)[:, None] * kv[None, :]
            _ELL_TRK[oid] = (99, False, str(cls))           # mature, never coasting -> shape always eligible
            out.append((oid, kc0, (float(kv[0]), float(kv[1]), 0.0), float(r),
                        EGO_PERCLASS_DSAFE.get(cls, 0.8)))
            _KF_PRED.append((p2.copy(), p2.copy(), [(float(p[0]), float(p[1])) for p in pred], zc)
                            + ((_ell_ring(oid, kc0, kv, r),) if MAN_ELLIPSE else
                               ((_cap_ring(oid, kc0, kv, r),) if MAN_CAPSULE else ())))
            if KFDBG and cls == "pedestrian":
                print(f"[KFDBG] t={t_sim:6.2f} {oid} ORACLE gt=({p2[0]:7.2f},{p2[1]:7.2f}) "
                      f"gtv=({v2[0]:6.2f},{v2[1]:6.2f})", flush=True)
        _PFE_MEMO["t"], _PFE_MEMO["out"], _PFE_MEMO["pred"] = t_sim, out, _KF_PRED
        return out
    tracks = _PFE.step(p_d[:2], (np.cos(cam_heading), np.sin(cam_heading)), cyls)
    out = []
    for tr in tracks:
        if tr.miss > 0 and (not MAN_MEM or not tr.trk.ready):
            continue    # memory OFF -> no coasting ghosts (same EGO_MEM dial as gt); single-glimpse coaster:
            #             no usable state yet either way
        d_safe = EGO_PERCLASS_DSAFE.get(tr.cls, 0.8)
        zc = 0.5 * tr.h
        if tr.trk.ready:
            kc0, kv, _ka = tr.trk.state()
            kc0 = np.array([kc0[0], kc0[1], zc])
            pred = tr.trk.predict(np.linspace(0.0, EGO_TAU_TRUST, 6))
        else:
            kc0, kv = p3(tr.xy, zc), np.zeros(3)
            pred = np.asarray([kc0, kc0])
        if not EGO_PREDICT:                                 # A/B ablation: reactive, no forecast
            kv = np.zeros(3); pred = np.asarray([kc0, kc0])
        if GT_ORACLE or KFDBG:                              # nearest-GT association (track ids are NOT GT ids)
            gt = min(gtl, key=lambda g: float(np.hypot(*(g[1] - kc0[:2])))) if gtl else None
            gt_d = float(np.hypot(*(gt[1] - kc0[:2]))) if gt is not None else 1e9
            if tr.miss == 0 and gt is not None and gt_d <= 2.0:
                _ORA_MAP[tr.id] = gt[0]                     # remember the GT identity while detected...
            elif tr.miss > 0 and tr.id in _ORA_MAP:         # ...so coasting is omniscient too (no KF drift in the
                by_oid = {g[0]: g for g in gtl}             #    upper bound; the CA coast integrates garbage young-
                gt = by_oid.get(_ORA_MAP[tr.id], gt)        #    track accel and runs away quadratically)
                gt_d = float(np.hypot(*(gt[1] - kc0[:2]))) if gt is not None else 1e9
            if GT_ORACLE and gt is not None and (tr.miss == 0 and gt_d <= 2.0 or
                                                 tr.miss > 0 and _ORA_MAP.get(tr.id) == gt[0]):
                kc0 = np.array([gt[1][0], gt[1][1], zc])    # exact sim pos/vel; detection timing unchanged
                kv = np.array([gt[2][0], gt[2][1], 0.0])
                pred = kc0[None, :] + np.linspace(0.0, EGO_TAU_TRUST, 6)[:, None] * kv[None, :]
            if KFDBG and tr.cls == "pedestrian":
                gs = (f"gt=({gt[1][0]:7.2f},{gt[1][1]:7.2f}) gtv=({gt[2][0]:6.2f},{gt[2][1]:6.2f}) d={gt_d:5.2f}"
                      if gt is not None else "gt=NONE")
                print(f"[KFDBG] t={t_sim:6.2f} trk{tr.id} n={tr.trk.n:3d} miss={tr.miss} {gs} "
                      f"kf=({kc0[0]:7.2f},{kc0[1]:7.2f}) kfv=({kv[0]:6.2f},{kv[1]:6.2f})", flush=True)
        r_eff = tr.r + (MAN_MEM_K * tr.trk.pos_sigma if tr.miss > 0 else 0.0)
        _ELL_TRK[f"trk{tr.id}"] = (int(tr.trk.n), tr.miss > 0, str(tr.cls))   # v5 ellipse eligibility
        out.append((f"trk{tr.id}", kc0, (float(kv[0]), float(kv[1]), 0.0), float(r_eff), d_safe))
        _KF_PRED.append((np.asarray(tr.xy[:2], float).copy(), np.asarray(kc0[:2], float).copy(),
                         [(float(p[0]), float(p[1])) for p in pred], float(zc))
                        + ((_ell_ring(f"trk{tr.id}", kc0, kv, tr.r),) if MAN_ELLIPSE else
                           ((_cap_ring(f"trk{tr.id}", kc0, kv, tr.r),) if MAN_CAPSULE else ())))
    _PFE_MEMO["t"], _PFE_MEMO["out"], _PFE_MEMO["pred"] = t_sim, out, _KF_PRED
    return out


def kf_movers(p_d, t_sim, cam_heading=None):
    """Like ego_safety_obstacles, but each mover's centre + velocity come from a LIVE per-mover CA-Kalman filter
    fed NOISY detections of the GT position (this is what proves the KF is in the loop, not GT omniscience). Also
    stashes each filter's PREDICTED future trajectory into _KF_PRED for draw_predictions. When cam_heading is given,
    a mover is only DETECTED if it falls in the forward depth cone (same sensor as native's fov_cloud) -> ours and
    native track the SAME movers; the KF prediction on top is the safety layer. Returns [(oid,c3,vel,r_obs,d_safe)].
    PERCEPT=realistic reroutes the cone path through the shared perception front-end (see _kf_movers_realistic)."""
    if _PERCEPT_REAL and cam_heading is not None:
        return _kf_movers_realistic(p_d, t_sim, cam_heading)
    global _KF_PRED
    _KF_PRED = []
    raw = []
    seen = (lambda c3: _in_cone(c3, p_d, cam_heading)) if cam_heading is not None \
        else (lambda c3: np.linalg.norm(c3[:2] - p_d[:2]) <= SENSE_R)
    for oid, cls, pos, vel, size in native_objects():
        if cls == "static":
            continue
        c3 = p3(pos, size[2] * 0.5)
        if not seen(c3):
            continue
        d = EGO_PERCLASS_DSAFE.get(cls)
        if d is not None:
            raw.append((oid, c3, 0.5 * float(max(size[0], size[1])), d, cls, np.asarray(vel[:2], float)))
    for a in animals:
        pos = a.p0 + a.vel * t_sim
        c3 = p3(pos, a.size[2] * 0.5)
        if seen(c3):
            raw.append((getattr(a, "id", id(a)), c3, 0.5 * float(max(a.size[0], a.size[1])),
                        EGO_PERCLASS_DSAFE.get(getattr(a, "cls_name", "animal"), EGO_PERCLASS_DSAFE["animal"]),
                        getattr(a, "cls_name", "animal"), np.asarray(a.vel[:2], float)))
    out = []
    fresh = set()                                                          # oids DETECTED in-cone this tick
    for (oid, c3, r, d, _cls, gt_vel) in raw:
        fresh.add(oid)
        det = np.asarray(c3, float) + _KF_RNG.normal(0, KF_MEAS_NOISE, 3)   # NOISY detection -> the filter's input
        trk = _KF.get(oid)
        if trk is None or trk.miss > MAN_MEM_TICKS:                         # new, or re-acquired after memory expired:
            trk = _KF[oid] = MoverTracker(dt=REPLAN_DT, meas_noise=KF_MEAS_NOISE)   # re-init fresh (no stale long-gap state)
        trk.update(det)
        trk.r_obs, trk.d_safe = r, d                                       # stash so out-of-cone memory needs no GT read
        if trk.ready:
            kc0, kv, _ka = trk.state()                                      # KF-smoothed centre + velocity
            pred = trk.predict(np.linspace(0.0, EGO_TAU_TRUST, 6))          # KF-PREDICTED future trajectory
        else:
            kc0, kv = np.asarray(c3, float), np.zeros(3)
            pred = np.asarray([kc0, kc0])
        if not EGO_PREDICT:                                                # A/B ablation: reactive, no forecast
            kv = np.zeros(3); pred = np.asarray([kc0, kc0])                # certify the mover as STATIC at its current KF centre
        if GT_ORACLE:                                                      # diagnostic: exact sim pos/vel, no noise/lag
            kc0 = np.asarray(c3, float)
            kv = np.array([gt_vel[0], gt_vel[1], 0.0])
            pred = kc0[None, :] + np.linspace(0.0, EGO_TAU_TRUST, 6)[:, None] * kv[None, :]
        if KFDBG and _cls == "pedestrian":
            print(f"[KFDBG] t={t_sim:6.2f} {oid} n={trk.n:3d} "
                  f"gt=({c3[0]:7.2f},{c3[1]:7.2f}) gtv=({gt_vel[0]:6.2f},{gt_vel[1]:6.2f}) "
                  f"det=({det[0]:7.2f},{det[1]:7.2f}) kf=({kc0[0]:7.2f},{kc0[1]:7.2f}) "
                  f"kfv=({kv[0]:6.2f},{kv[1]:6.2f})", flush=True)
        _ELL_TRK[oid] = (int(trk.n), False, str(_cls))                     # v5 ellipse eligibility
        out.append((oid, kc0, (float(kv[0]), float(kv[1]), 0.0), r, d))
        _KF_PRED.append((det[:2].copy(), kc0[:2].copy(), [(float(p[0]), float(p[1])) for p in pred], float(c3[2]))
                        + ((_ell_ring(oid, kc0, kv, r),) if MAN_ELLIPSE else
                           ((_cap_ring(oid, kc0, kv, r),) if MAN_CAPSULE else ())))
    # ---- TRACK MEMORY: movers that just LEFT the cone are COASTED (extrapolated + covariance grown) and kept in the
    # cert/occupancy set for MAN_MEM_TICKS so 'straight' cannot instantly re-certify behind a mover still on a collision
    # course (forget-after-pass). Extrapolate-only: the centre/vel come from the coasted KF, never a fresh GT read.
    # Gated to the real cone path (cam_heading set) so the omniscient re-query (COLLDBG) and slip paths are unchanged.
    if MAN_MEM and cam_heading is not None:
        for oid, trk in list(_KF.items()):
            if oid in fresh:
                continue                                                   # detected this tick -> already emitted above
            trk.coast()                                                    # advance one dt, GROW covariance, miss += 1
            if trk.miss > MAN_MEM_TICKS:
                del _KF[oid]; continue                                     # forgotten -> drop (re-entry re-inits fresh)
            if not trk.ready:
                continue
            kc0, kv, _ka = trk.state()
            r_mem = trk.r_obs + MAN_MEM_K * trk.pos_sigma                  # covariance growth -> bigger keep-out tube
            if oid in _ELL_TRK:                                            # coasting -> ellipse ineligible
                _ELL_TRK[oid] = (_ELL_TRK[oid][0], True, _ELL_TRK[oid][2])
            out.append((oid, kc0, (float(kv[0]), float(kv[1]), 0.0), r_mem, trk.d_safe))
            pred = trk.predict(np.linspace(0.0, EGO_TAU_TRUST, 6))
            _KF_PRED.append((kc0[:2].copy(), kc0[:2].copy(), [(float(p[0]), float(p[1])) for p in pred], float(kc0[2])))
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


def ego_speed_search(p_d, v_d, t_sim, u0=0.0):
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
            # STALE-SPLINE WINDOW FIX (audit 6/27): the drone is u0 deep into the committed spline
            # (u0~0 right after a replan; grows when replans FAIL). Certifying [0, s*TAU] from the
            # spline HEAD proves the already-flown past, not the upcoming window. Back-date the mover
            # to the spline start (c0 - v*u0/s) and certify [0, u0 + s*TAU]: at param u the mover sits
            # at its true CV position for real time (u-u0)/s, so the REAL upcoming [0,TAU] is covered.
            c0b = np.asarray(c0, float) - np.asarray(vel, float) * (u0 / s)
            hp, _ = ego.certify_horizontal(obs_c0=c0b, R=R, obs_vel=tuple(vel / s), obs_acc=(0, 0, 0),
                                           t_hi=u0 + s * EGO_TAU_TRUST, v_eff=EGO_VEFF_SLIP / s, delta=REPLAN_DT * s)
            if not hp:
                ok = False; break
        if ok:
            return float(s), None
        s -= EGO_S_STEP
    return 0.0, "blocked"                                    # no certified warp -> hover on the same path


def ego_slip_feasible(p_d, v_d, t_sim, s, u0=0.0):
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
        c0b = c0 - vel * (u0 / s)                    # stale-spline window fix (see ego_speed_search)
        hp, _ = ego.certify_horizontal(obs_c0=c0b, R=R, obs_vel=tuple(vel / s), obs_acc=(0, 0, 0),
                                       t_hi=u0 + s * EGO_TAU_TRUST, v_eff=EGO_VEFF_SLIP / s, delta=REPLAN_DT * s)
        if not hp:
            return False
    return True


MAN_REACH_PAD = 0.3   # posture/arm reach added to a mover's head-top for the fly-OVER vertical clearance
MAN_DSAFE_V = 0.5     # vertical standoff above the head
MAN_QCONF = float(os.environ.get("EGO_QCONF", 0.125))  # fallback q_conformal (pedestrian, CV, eps=0.05) if no per-class
MAN_VEFF = float(os.environ.get("EGO_MAN_VEFF", 0.61)) # fallback tube growth; OWN env (was aliased to EGO_VEFF, which the
#                                                        SLIP path also reads -> setting one silently changed the other)
MAN_EPS = float(os.environ.get("EGO_EPS", 0.05))       # conformal risk level for the per-class mover keep-out


def _load_perclass_conf(eps):
    """Per-class (q_conformal, v_eff) from out/conformal/calib.json, keyed by the mover's per-class d_safe so cert_clear
    can use the VEHICLE / ANIMAL tube instead of the pedestrian scalar (code-review 2026-06-27 HIGH: ped tube was applied
    to all classes -> vehicles/animals under-/mis-covered). pedestrian/vehicle are measured; animal -> the pooled 'all'."""
    for cand in (os.path.join(os.path.dirname(_HERE), "out", "conformal", "calib.json"),
                 os.path.join(_HERE, "out", "conformal", "calib.json")):
        try:
            cj = json.load(open(cand)); g = cj["groups"]; k = str(eps)
            lv = lambda grp: (float(g[grp]["levels"][k]["q_conformal"]), float(g[grp]["levels"][k]["v_eff"]))
            return {0.8: lv("pedestrian"), 0.6: lv("vehicle"), 0.7: lv("all")}   # keyed by EGO_PERCLASS_DSAFE values
        except Exception:
            continue
    return {}


PERCLASS_CONF = _load_perclass_conf(MAN_EPS)   # {d_safe: (q_conformal, v_eff)}; empty -> fall back to MAN_QCONF/MAN_VEFF
MAN_TRACK = float(os.environ.get("EGO_TRACK", 0.473))  # HCT-D plan->flown TRACKING margin: 0.473 = the LEGAL
#   eps=0.01 per-flight quantile over all 100 episodes (B-bucket 2026-07-03; old 0.45 was hand-picked BELOW the
#   observed max 0.473 -- an unbacked number the 6/27 audit flagged).
#   on the PER-FLIGHT (episode) unit -- the correct exchangeable unit for a per-flight collision-freedom guarantee
#   (windows within one flight are autocorrelated, NOT exchangeable -- the old per-window 0.29 m gave only marginal
#   coverage and ~30% of FLIGHTS breached it; code-review 2026-06-27 critical fix). delta_track = (1-eps) quantile of
#   per-EPISODE-max ||p_flown - p_planned|| over 100 harvested real-quad episodes: eps0.05 -> 0.264 m, max observed
#   0.473 m; 0.45 covers ~all observed flights. This was only ~0.4 m (not the 2.9 m the broken thrash implied) BECAUSE
#   the maneuver-switch THRASH that caused the heavy tail is fixed (commit hysteresis): per-episode-max delta p95
#   collapsed 1.72 -> 0.27 m. delta ~ ||a||/curvature, NOT speed. track_calib.json / track_conformal.py. EGO_TRACK=0 off.
# horizontal standoff ours holds from a mover (overrides the per-class 0.8). LOWER = ours flies tighter/faster
# to race the real EGO (which flies at ~0.3 and grazes); the cert still guarantees this clearance so ours never
# collides where EGO does. Tune via EGO_MANDSAFE.
MAN_DSAFE = float(os.environ.get("EGO_MANDSAFE", 0.45))
PHI_MAN = np.radians(25.0)   # ground around-L/R deflection angle for the maneuver tournament
# CCF/CRET/FOVCAP speed patches DELETED 2026-07-07 (stage-2): absorbed by SL.maneuver_decide_v2
# (CCF itself was empirically retired 2026-06-30: froze 2/5 natural seeds; v2's carrot-release +
# evade mapping + dwell probes address exactly that failure mode -- verified per-seed before the
# default flip, see commit message).
EGO_DECIDE = os.environ.get("EGO_DECIDE", "v2")   # unified tournament DEFAULT (stage-2 2026-07-07);
#   "v1" = frozen legacy tournament kept ONLY for A/B reproduction of pre-unification renders
#   (one shared implementation with the headless benchmark; absorbs CCF/CRET/FOVCAP speed patches)
import safety_layer as _SL
_MAN_V2 = {}
# ---- v5 MOTION-FRAME ELLIPSE on the render face (ELLIPSE=1, default OFF -> byte-identical) ----
# Mature (age>=4), non-coasting, moving (|v|>=v_min_dir) ped/veh tracks swap their mover law to the
# calib_v5 ellipse: along-axis A(t)=R+v_eff*(t+d) on the KF velocity, cross A(t)/kappa. Carried as the
# optional 7th cyl field into the SHARED _SL tournament (cert_clear/_warp dispatch the aniso cert).
# Everything else (young/static/coasting/slow, vertical zc law) keeps today's production numbers.
MAN_ELLIPSE = os.environ.get("ELLIPSE", "0") == "1"
MAN_CAPSULE = os.environ.get("CAPSULE", "0") == "1"   # v6: keep-out = [mover's back, KF tip] ⊕ q̃,
#   pearl-string certified through the SHARED _SL tournament (the "cap" 7th cyl field)
GT_ORACLE = os.environ.get("GT_ORACLE", "0") == "1"   # diagnostic: skip noisy-detection+KF, feed exact
#   sim pos/vel as kc0/kv (upper bound on prediction quality -- separates KF lag/noise from shape/cert issues)
KFDBG = os.environ.get("KFDBG", "0") == "1"           # diagnostic: per-tick GT-vs-KF dump for pedestrian tracks
EGO_TDYN = os.environ.get("EGO_TDYN", "0") == "1"     # TIME-AWARE movers: solver-level time-aligned penalty
#   (EGO-Swarm/MIGHTY-style) replaces the mover occupancy rings -- EGO may plan THROUGH space a mover will
#   have vacated; the pearl-chain cert still gates every commit (s=0 pearl keeps "he might stop" honest)
EGO_TDYN_W = float(os.environ.get("EGO_TDYN_W", "10.0"))
EGO_TDYN_PAD = float(os.environ.get("EGO_TDYN_PAD", "0.6"))   # hinge pad past r+d_safe (cert-scale reach)
assert not (MAN_ELLIPSE and MAN_CAPSULE), "ELLIPSE=1 and CAPSULE=1 are mutually exclusive"
_ELL_V5 = {}; _ELL_VMIN = 0.5; _ELL_TRK = {}      # _ELL_TRK: oid -> (kf_age, coasting, cls)
_CAP_V6 = {}; _CAP_K = 4
if MAN_CAPSULE:
    _v6 = _SL.load_calib_v6(eps=MAN_EPS)
    for _c6 in ("pedestrian", "vehicle"):
        _e6 = _v6.get(_c6) or {}
        if _e6.get("capsule") and "mature" in _e6:
            _CAP_V6[_c6] = (float(_e6["mature"][0]), float(_e6["mature"][1]), _e6.get("rear"))
            _CAP_K = int(_e6.get("n_pearls", 4))
    print(f"[3dv] CAPSULE=1: v6.1 segment keep-out (mature tracks) { _CAP_V6 } K={_CAP_K}", flush=True)
if MAN_ELLIPSE:
    _v5 = _SL.load_calib_v5(eps=MAN_EPS)
    for _c5 in ("pedestrian", "vehicle"):
        _e5 = _v5.get(_c5) or {}
        if "mature" in _e5 and _e5.get("status", "ok") != "UNCALIBRATED":
            _ELL_V5[_c5] = (float(_e5["mature"][0]), float(_e5["mature"][1]), float(_e5.get("kappa", 1.0)))
            _ELL_VMIN = float(_e5.get("v_min_dir", 0.5))
    print(f"[3dv] ELLIPSE=1: v5 motion-frame ellipse (mature moving tracks) "
          f"{ {c: v for c, v in _ELL_V5.items()} } v_min={_ELL_VMIN}", flush=True)


def _ell_of_mover(oid, vel, r_obs):
    """(kappa, ux, uy, R_warp, q5, veff5) for an ellipse-eligible mover, else None. The ONE render-face
    eligibility predicate (mirrors build_cylinders + the v5 calibration): mature KF, not coasting,
    class calibrated, speed >= v_min_dir."""
    if not MAN_ELLIPSE:
        return None
    a = _ELL_TRK.get(oid)
    if not a or a[0] < 4 or a[1]:
        return None
    ent = _ELL_V5.get(a[2])
    if ent is None:
        return None
    q5, veff5, kap = ent
    sp = float(np.hypot(vel[0], vel[1]))
    if sp < _ELL_VMIN:
        return None
    # kap == 1 (e.g. the v5-iso ablation twin via CALIB_FILE_V5): the mover still SWAPS to the v5
    # numbers -- same-generation circle -- but carries no ellipse field. Keeps the render A/B a
    # single-variable (shape-only) comparison instead of stale-gen-1 circle vs modern-width ellipse.
    rw = kap * (float(r_obs) + MAN_DSAFE + MAN_TRACK) + q5
    return (kap, float(vel[0]) / sp, float(vel[1]) / sp, rw, q5, veff5)


def _cap_of_mover(oid, vel, r_obs):
    """(q6, veff6_with_pearl_gap, K) for a capsule-eligible mover, else None. Mirrors
    build_cylinders: mature KF, not coasting, class calibrated -- NO speed gate (a slow mover's
    segment degenerates continuously to the point law; the calibration scored it the same way)."""
    if not MAN_CAPSULE:
        return None
    a = _ELL_TRK.get(oid)
    if not a or a[0] < 4 or a[1]:
        return None
    ent = _CAP_V6.get(a[2])
    if ent is None:
        return None
    q6, veff6, rear = ent
    sp = float(np.hypot(vel[0], vel[1]))
    return (q6, veff6 + sp / (2.0 * max(_CAP_K - 1, 1)), _CAP_K, rear)


def _cap_ring(oid, kc0, vel, r_obs):
    """Ground outline of the deployed capsule at the trust-window tip: stadium from the mover's
    BACK cap (around c0) to the KF apex cap (around c0 + v*tau). Drawn on the CAPSULE arm."""
    c = _cap_of_mover(oid, vel, r_obs)
    if c is None:
        return None
    q6, veff6, _K, rear = c
    rad = float(r_obs) + MAN_DSAFE + q6 + veff6 * (EGO_TAU_TRUST + REPLAN_DT)
    tip = np.asarray(kc0[:2], float) + np.asarray(vel[:2], float) * EGO_TAU_TRUST
    c0 = np.asarray(kc0[:2], float)
    sp = float(np.hypot(*(tip - c0)))
    u = (tip - c0) / sp if sp > 1e-6 else np.array([1.0, 0.0])
    a0 = float(np.arctan2(u[1], u[0]))
    if rear is not None and sp > 1e-6:
        # v6.1: FLAT rear -- the wake boundary sits at the (constant) rear-overrun quantile +
        # body/standoff behind the mover, not at the full growing q̃ cap
        back = float(rear[0]) + float(rear[1]) * (EGO_TAU_TRUST + REPLAN_DT) + float(r_obs) + MAN_DSAFE
        bl = c0 - u * back
        n = np.array([-u[1], u[0]])
        pts = [(float(bl[0] + n[0] * rad), float(bl[1] + n[1] * rad)),
               (float(bl[0] - n[0] * rad), float(bl[1] - n[1] * rad))]   # flat rear chord
        pts = [pts[0]] + [(float(tip[0] + rad * np.cos(a0 + th)), float(tip[1] + rad * np.sin(a0 + th)))
                          for th in np.linspace(np.pi / 2, -np.pi / 2, 13)] + [pts[1]]
        return pts + [pts[0]]
    pts = [(float(c0[0] + rad * np.cos(a0 + th)), float(c0[1] + rad * np.sin(a0 + th)))
           for th in np.linspace(np.pi / 2, 3 * np.pi / 2, 13)]          # back cap around the mover
    pts += [(float(tip[0] + rad * np.cos(a0 + th)), float(tip[1] + rad * np.sin(a0 + th)))
            for th in np.linspace(-np.pi / 2, np.pi / 2, 13)]            # apex cap at the KF tip
    return pts + [pts[0]]


def _ell_ring(oid, kc0, vel, r_obs):
    """Ground outline of the deployed keep-out ellipse at t=0 (drawn by draw_predictions on the
    ELLIPSE arm as the visible proof of the motion-frame tube). None when the mover is a circle."""
    e = _ell_of_mover(oid, vel, r_obs)
    if e is None:
        return None
    kap, ux, uy, rw, _q5, veff5 = e
    a = rw + veff5 * REPLAN_DT                # along semi-axis incl. the staleness charge
    b = a / kap                               # cross-track: the thin side the drone may now use
    return [(float(kc0[0] + a * np.cos(th) * ux - b * np.sin(th) * uy),
             float(kc0[1] + a * np.cos(th) * uy + b * np.sin(th) * ux))
            for th in np.linspace(0.0, 2.0 * np.pi, 33)]
# HCT-D tracking-tube HARVEST: TRACK_HARVEST=1 logs per sub-step (window, delta=||flown-planned||, hodograph
# features ||v||,||a||,lateral-accel) so a split-conformal tracking tube kappa*g can be calibrated off-line.
_TRACKH = os.environ.get("TRACK_HARVEST") == "1"
_track_rows = []
_min_static = [np.inf, None, None]   # [clearance, box_size, drone_pos] of the closest static approach (TREE_DBG)
MAN_STATIC_MARGIN = float(os.environ.get("EGO_STATICM", 0.70))  # reject a candidate whose PREDICTED-FLOWN path
#   comes within this of KNOWN static. Because the gate now forward-sims the real quad (overshoot included), this is
#   just the drone BODY radius + a small buffer -- the tracking tube is in the flown path, not the margin.
MAN_STATIC_BUF = float(os.environ.get("EGO_STATIC_BUF", 0.45))  # static gate keep-out = drone_radius + this buffer
#   (the gate now uses the cylinder-SDF, same model as GT clearance(); rmarg ~= 0.70 m matches the swept margin).
MAN_STATIC_HZ = float(os.environ.get("EGO_STATIC_HZ", 1.20))   # forward-sim LOOKAHEAD (s) for the static gate. Longer =
#   the drone starts avoiding static sooner so its inertia doesn't carry it into a building it only reacts to late
#   (the 2026-06-27 diagnosis: most ours collisions are MODERATE-speed static grazes the short-horizon gate missed).
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
# ---- PERCEPT=realistic: swap the GT-see-through mover perception (guaranteed in-cone detection, GT-keyed
# identity) for the SHARED front-end (perception.py: cone + hard occlusion + distance miss/noise + NN-associated
# tracks, no GT identity). Applies only to the real cone path (cam_heading given); the omniscient re-query /
# debug paths keep GT. Default stays "gt" until the B-bucket recalibration (headline semantics must not move).
_PERCEPT_REAL = os.environ.get("PERCEPT", "realistic") == "realistic"   # FROZEN 2026-07-03: realistic default
_PFE = None
if _PERCEPT_REAL:
    from perception import PerceptionFrontEnd, PerceptCfg
    _pcfg = PerceptCfg.from_env(dt=REPLAN_DT)
    if "PERCEPT_FOV_DEG" not in os.environ:                 # default the cone to THIS harness's sensor args
        _pcfg.fov_deg = float(args.fov_deg)
    if "PERCEPT_RANGE" not in os.environ:
        _pcfg.fov_range = float(args.fov_range)
    _PFE = PerceptionFrontEnd(_pcfg, seed=int(args.seed) * 13 + 5)
    print(f"[percept] REALISTIC front-end on: cone {_pcfg.fov_deg:.0f}deg/{_pcfg.fov_range:.0f}m, "
          f"occlusion={_pcfg.occlusion}, p_miss0={_pcfg.p_miss0}", flush=True)
# ---- OUT-OF-CONE MOVER TRACK MEMORY (extrapolate-only; NO GT read for unseen movers, so still fair vs native) ----
# A mover that leaves the +-fov_deg/fov_range cone used to vanish from the cert + occupancy the SAME tick (forget-
# after-pass) while its KF froze (no predict, no covariance growth). Instead we KEEP a recently-seen mover for a few
# ticks, COAST its filter forward (growing covariance), and inflate its keep-out by the covariance growth. The state
# comes purely from the last in-cone KF estimate -- we never re-read native_objects() for an out-of-cone mover.
MAN_MEM = os.environ.get("EGO_MEM", "1") == "1"             # master switch for out-of-cone mover track memory
MAN_MEM_TICKS = int(os.environ.get("EGO_MEM_TICKS", 8))    # remember an out-of-cone mover this many replan ticks (~0.8s @10Hz)
MAN_MEM_K = float(os.environ.get("EGO_MEM_K", 2.0))        # keep-out inflation = this many KF position-sigmas (covariance growth)
if _PFE is not None and "PERCEPT_TTL" not in os.environ:
    _PFE.cfg.ttl_ticks = MAN_MEM_TICKS                     # realistic memory horizon follows the SAME dial as gt
    # (a gt-vs-realistic A/B must not silently compare different memory policies)
MAN_PHI = np.radians(25.0)
MAN_DEADBAND = 0.5    # hysteresis: keep the CURRENT maneuver unless another certified one beats its goal-ward
                      # speed by >this (m/s). Stops the around-L/R/over flicker that brakes-and-reaccelerates
                      # (the "hesitation") every time a mover twitches; straight resumes the moment it re-certifies.
_MAN_STATE = {"kind": None}
_V3_ST = {}          # CPL-v3 (EGO_DECIDE=v3, stage-3 render wiring): warm-start incumbent primitive
_V3_PLAN = [None]    # the committed composite plan for the executor to fly via local_lattice.plan_eval
_SPAWNDBG = [0]   # MAN_SPAWNDBG=1: dump the spawn occupancy (360 static vs native forward cone) for the first calls


_VF_STATE = {}


def _man_cloud(p_d, heading, t_sim, movers):
    """Occupancy for the maneuver planner: the FOV static voxels + ground, PLUS each mover's PREDICTED footprint
    rendered as a CYLINDER inflated to r_obs+d_safe and CAPPED at head height (so the overhead column stays free
    for fly-OVER). The d_safe inflation makes EGO's own 2-D route already clear the certificate margin, so the
    cert passes ground routes (fly fast) instead of rejecting them and forcing a constant climb."""
    pts = [ground_patch(p_d, radius=EGO_HOR + 4.0)]
    # static perception: real D435i depth (occlusion+range limited) when --d435i, else the GT fov_cloud.
    # (movers are re-added below as KF-predicted inflated cylinders, so static-only here.)
    if args.d435i:
        _raw = d435i_cloud(p_d, heading)
        # OCC_MEM=1 opts into persistent static occupancy memory. It SHOULD close the occlusion gap vs GT, but the
        # naive version (mover surfaces removed only at the current frame) leaves phantom walls along mover trails
        # and over-densifies -> on seed 5 it walled the drone in (froze, climb x31). Default OFF = the validated
        # single-frame path. Fixing memory (decay + mover-track removal) is future work; see occ_remember().
        stat = np.asarray(_raw if os.environ.get("OCC_MEM", "1") == "0" else
                          occ_remember(_raw, p_d, movers, t_sim), float)
        # ^ REAL-DEPLOYMENT DEFAULT (2026-07-03): the map is BUILT (v2 memory: cap+TTL). OCC_MEM=0
        #   restores single-frame cone-only static as an explicit ablation.
    elif len(STATIC_CLOUD):
        # FAIR-COMPARISON perception: static comes through the SAME forward depth cone native's fov_cloud uses (heading
        # == cam yaw), NOT a 360-degree omniscient map. Both planners therefore see the identical static, so the only
        # A/B variable is the safety layer. (Movers are re-added below as KF cylinders, which IS the safety layer.)
        stat = _percept_static(p_d, heading)
    else:
        stat = np.zeros((0, 3))
    stat = np.asarray(stat, float)
    if len(stat):
        pts.append(stat)
    v_nom = np.array([np.cos(heading), np.sin(heading)]) * MAN_VCRUISE   # drone's nominal motion
    _vf_a = float(os.environ.get("VF_EMA", "0"))
    for (_oid, c3, vel, r_obs, d_safe) in movers:
        if EGO_TDYN:
            continue    # time-aware solver term owns the movers -- no occupancy rings (they would re-freeze
            #             the swept corridor spatially and defeat the time dimension); statics stay in the map
        if _vf_a > 0:
            # PLANNER-FEED velocity EMA (cert cylinders keep the raw KF; calibration matches raw).
            # Measurement noise -> per-tick velocity wiggle -> t_cpa ring wiggle -> reference
            # jitter even at 3 m/s. Smooth the FEED only.
            _pv = _VF_STATE.get(_oid)
            vel = tuple(vel) if _pv is None else tuple((1 - _vf_a) * np.asarray(_pv) + _vf_a * np.asarray(vel))
            _VF_STATE[_oid] = vel
        # RELATIVE-MOTION timing (this is where KF prediction buys SPEED & smooth accel): block each mover at where
        # it WILL BE at the closest-approach time t_cpa of the relative motion (drone - mover), not where it is now.
        # A mover that will have swept past the corridor has its t_cpa footprint OFF the drone's path -> the drone
        # flies STRAIGHT through the gap behind it (no reactive braking) instead of detouring like native EGO. Each
        # mover is always fed exactly ONCE (at current + t_cpa) so the occupancy never toggles -> EGO stays smooth.
        dp = np.array([c3[0] - p_d[0], c3[1] - p_d[1]])
        dv = np.array([vel[0], vel[1]]) - v_nom
        dvn = float(dv @ dv)
        tcpa = (float(np.clip(-(dp @ dv) / dvn, 0.0, MAN_PLANHI)) if dvn > 1e-6 else 0.0) \
            if os.environ.get("CPA_OFF", "0") != "1" else 0.0   # ablation arm: dodge-the-past
        R = r_obs + MAN_DSAFE; head = 2.0 * c3[2]
        # GT fov_cloud only voxelises a coarse mover box, so it needs BOTH the current and the predicted keep-out
        # ring. The D435i depth ALREADY paints the mover's current front surface densely; stacking a 0.8 m ring on
        # top over-inflates the current position, and in a dense crowd the doubled blobs seal the corridor -> EGO
        # optimise fails -> stall. With d435i, feed ONLY the PREDICTED (t_cpa) ring (where the mover WILL be, which
        # depth can't see yet); the current-position d_safe is still enforced by cert_clear()'s static-mover check.
        leads = (tcpa,) if args.d435i else (0.0, tcpa)
        _eta = os.environ.get("ETA_FEED", "0") == "1"
        _qv = PERCLASS_CONF.get(d_safe, (MAN_QCONF, MAN_VEFF))
        for lead in leads:                                        # current + closest-approach predicted footprint
            cx, cy = c3[0] + vel[0] * lead, c3[1] + vel[1] * lead
            R_l = R
            if _eta and lead > 0.0:
                # GapWeave S1 (time-matched feed radius): the planner must avoid the ring the
                # CERTIFICATE will demand at arrival time -- q + v_eff*(t_view+delta), capped, plus
                # the band offset so the unconstrained optimum sits at floor+0.25 (plan once,
                # certify once; the plan-small/cert-big mismatch was the recurring kill->hold chain).
                R_l = min(R + _qv[0] + _qv[1] * (min(lead, REPLAN_DT) + REPLAN_DT), R + 2.6) + 0.25
            _n_th = max(10, int(np.ceil(2 * np.pi * R_l / 0.5)))   # gap-free ring at any radius
            ring = [[cx + R_l * np.cos(a), cy + R_l * np.sin(a), z]
                    for a in np.linspace(0, 2 * np.pi, _n_th, endpoint=False)
                    for z in np.linspace(0.3, head, 3)]
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
    # PERCEPTION heading = body-mounted depth cam (quad.yaw), IDENTICAL to native's fov_cloud(p_d, quad.yaw): ours and
    # native must see through the same forward cone (static AND mover detection) so the only A/B variable is the cert.
    cam_heading = float(quad.yaw)
    movers = kf_movers(p_d, t_sim, cam_heading)            # cone-DETECTED movers + their KF prediction (the safety layer)
    ego.update_cloud(_man_cloud(p_d, cam_heading, t_sim, movers), p_d)
    if EGO_TDYN:                                           # feed mover polys to the solver's time-aligned term
        # hinge radius must reach CERT scale (r + per-class d_safe + pad), not the ring's r+0.45: the
        # tournament keeps the drone ~1.5m off movers, so a smaller hinge never activates (verified:
        # 728/728 replans byte-identical with the small radius)
        _rows = [[c3[0], c3[1], c3[2], vel[0], vel[1], vel[2], r_obs + _ds + EGO_TDYN_PAD, 2.0 * c3[2]]
                 for (_o, c3, vel, r_obs, _ds) in movers]
        ego.set_moving_obstacles(np.asarray(_rows, float).reshape(-1, 8), EGO_TDYN_W if _rows else 0.0)
        if KFDBG:
            print(f"[TDYN] t={t_sim:6.2f} fed {len(_rows)} movers w={EGO_TDYN_W}", flush=True)
    if os.environ.get("MAN_SPAWNDBG") == "1" and _SPAWNDBG[0] < 4:
        _SPAWNDBG[0] += 1
        n360 = int(len(STATIC_CLOUD)) if len(STATIC_CLOUD) else 0
        _os = _cone_static(p_d, cam_heading)                       # what OURS now feeds (cone)
        n1 = int(len(_os)); s_near = float(np.linalg.norm(_os[:, :2] - p_d[:2], axis=1).min()) if len(_os) else 9.9
        _fc = fov_cloud(p_d, cam_heading, t_sim)                   # what NATIVE feeds (same cone+heading)
        if len(_fc):
            _fdd = np.linalg.norm(_fc[:, :2] - p_d[:2], axis=1); ncone = len(_fc); c_near = float(_fdd.min())
        else:
            ncone = 0; c_near = 9.9
        mv_near = min((float(np.hypot(*(np.asarray(c3)[:2] - p_d[:2]))) - r for (_o, c3, _v, r, _d) in movers), default=9.9)
        print(f"[SPAWNDBG] call{_SPAWNDBG[0]} p_d={np.round(p_d,2)} goal={np.round(np.asarray(cur_wp,float)[:2],1)}  "
              f"ours-cone-static: n={n1} nearest={s_near:.2f}m (full map={n360}) | "
              f"native-cone: n={ncone} nearest={c_near:.2f}m | mover_nearest_edge={mv_near:.2f}m", flush=True)
    gxy = np.asarray(cur_wp, float)[:2] - p_d[:2]; dist = float(np.linalg.norm(gxy))
    gdir = gxy / dist if dist > 1e-6 else np.array([1.0, 0.0])
    L = min(EGO_HOR, max(dist, 1.0))
    zc = [2.0 * c3[2] + MAN_REACH_PAD + MAN_DSAFE_V + MAN_QCONF + MAN_TRACK for (_oid, c3, _, _, _) in movers]
    z_top = min(Z_CEIL, (max(zc) if zc else CRUISE_Z + 1.0) + 0.2)

    def rot(v, ang):
        c, s = np.cos(ang), np.sin(ang); return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])

    # static clearance gate sees the SAME forward-cone static the planner does (no 360-degree omniscience) -> the gate
    # cannot reject against buildings the drone could not perceive, keeping ours' perception identical to native's.
    loc_static = _percept_static(p_d, cam_heading)

    def cert_clear():                                      # CURRENTLY-held EGO B-spline vs every mover
        # NOTE: MAN_DSAFE (0.45) is the body+standoff (> drone_radius 0.25), so it already keeps the BODY out; adding
        # drone_radius again double-counts and over-inflates -> reverted (A+B 2026-06-27 made aggregate worse: more
        # mover-conservatism -> path churn -> MORE static grazes 9->13, reach 88->84). Per-class tube also reverted to
        # the pedestrian scalar (documented limitation); the dominant problem is the STATIC gate, not the mover cert.
        for (_oid, c3, vel, r_obs, d_safe) in movers:
            # PER-CLASS conformal tube (was the audit's dead code: PERCLASS_CONF loaded but never read;
            # every class got the pedestrian scalar). Falls back to the scalars when calib lacks the class.
            q_c, veff_c = PERCLASS_CONF.get(d_safe, (MAN_QCONF, MAN_VEFF))
            R = r_obs + MAN_DSAFE + q_c + MAN_TRACK         # +tracking margin so the cert covers the FLOWN path
            hp, _ = ego.certify_horizontal(obs_c0=c3, R=R, obs_vel=vel, t_hi=EGO_TAU_TRUST, v_eff=veff_c, delta=REPLAN_DT)
            hc, _ = ego.certify_horizontal(obs_c0=c3, R=R, obs_vel=(0, 0, 0), t_hi=EGO_TAU_TRUST, v_eff=veff_c, delta=REPLAN_DT)
            vo, _ = ego.certify_above(z_clear=2.0 * c3[2] + MAN_REACH_PAD + MAN_DSAFE_V + q_c + MAN_TRACK,
                                      t_hi=EGO_TAU_TRUST, delta=REPLAN_DT)
            if not ((hp and hc) or vo):
                return False
        return True

    def flown_samples():
        """The path the REAL drone will actually FLY: forward-simulate a COPY of the quad tracking the committed
        B-spline over [0, TAU]. The planned point-path hides the tracking OVERSHOOT (inertia / tilt-to-accelerate);
        gating this predicted-flown tube is how we account for the real drone's dynamics, not just its radius."""
        d = ego.duration()
        hz = min(d, MAN_STATIC_HZ)                         # static-gate forward-sim lookahead (>= the flown-per-tick dist)
        if args.pointmass:                                # diagnostic: flown == planned
            return np.array([ego.eval(s)[0] for s in np.linspace(0, hz, 16)])
        q = copy.deepcopy(quad)                           # current REAL state + params
        n = max(1, int(hz / DT)); pts = [q.p.copy()]
        for k in range(1, n + 1):
            r = ego.eval(min(k * DT, d - 1e-3))
            if r is None:
                break
            sp, sv, sa = (np.asarray(x, float) for x in r)
            yr = float(np.arctan2(sv[1], sv[0])) if np.linalg.norm(sv[:2]) > 1e-3 else q.yaw
            q.step(sp, sv, sa, DT, yaw_ref=yr)
            pts.append(q.p.copy())
        return np.array(pts)

    # PRECISE static gate: gate the PREDICTED-FLOWN path against the static obstacle CYLINDERS (same SDF as the GT
    # clearance() metric) instead of sparse cloud points -> catches grazes of large buildings the cloud-point gate
    # missed (a 40m building hugged at its curved face). Reject = drone BODY within (drone_r + buffer) of any cylinder
    # surface; the maneuver then detours or HOLDs rather than grazing known static. loc_obs = local static cylinders.
    # filter by SURFACE distance (center_dist - radius), NOT center distance: a huge building's centre can be 20 m away
    # while its wall is right beside the drone -- a center-distance filter would wrongly drop it (seed 55 graze bug).
    loc_obs = [(np.asarray(c3, float), 0.5 * float(sz[0]), 0.5 * float(sz[2]))
               for (_c, c3, sz) in STATIC_FED
               if float(np.hypot(*(np.asarray(c3)[:2] - p_d[:2]))) - 0.5 * float(sz[0]) <= EGO_HOR + 6.0]
    _ST_RMARG = float(par.drone_radius) + MAN_STATIC_BUF   # body radius + buffer, matches GT clearance() drone radius

    def static_clear():
        if ego.duration() <= 1e-3 or not loc_obs:
            return True
        for qp in flown_samples():
            for (c3, R, hh) in loc_obs:
                d = qp - c3
                dr_out = max(float(np.hypot(d[0], d[1])) - R, 0.0); dz_out = max(abs(float(d[2])) - hh, 0.0)
                sd = np.hypot(dr_out, dz_out) if (dr_out > 0 or dz_out > 0) else -min(R - float(np.hypot(d[0], d[1])), hh - abs(float(d[2])))
                if sd < _ST_RMARG:                          # flown body within margin of this cylinder surface
                    return False
        return True

    def mover_clear_flown():
        """Forward-sim FLOWN-path gate vs the KF-PREDICTED mover cylinders (the analytic cert_clear has NO forward-sim,
        so a fast climb-over / tracking overshoot can graze a tall mover's roof -- e.g. the seed-56 van -- while the
        planned path certified). Cylinder disjunction per mover: clear iff horizontally outside r_obs+keep-out OR the
        flown body is above the mover top. Predicted mover pos = KF centre + vel*t at each flown sample time."""
        if ego.duration() <= 1e-3 or not movers:
            return True
        fs = flown_samples()
        for k, qp in enumerate(fs):
            tk = k * DT
            for (_oid, c3, vel, r_obs, d_safe) in movers:
                mc = np.asarray(c3, float) + np.array([float(vel[0]), float(vel[1]), 0.0]) * tk
                dh = float(np.hypot(qp[0] - mc[0], qp[1] - mc[1]))
                R = r_obs + MAN_DSAFE + MAN_QCONF           # tracking already in the flown path; q covers KF pred error
                ztop = 2.0 * float(c3[2]) + MAN_REACH_PAD + MAN_DSAFE_V + MAN_QCONF
                if dh < R and float(qp[2]) < ztop:          # inside the cylinder horizontally AND not above its top
                    return False
        return True

    # ---- CCF: Certified Commitment Function (anti-thrash) --------------------------------------------------------
    # The stateless tournament re-picks a winner every tick (straight first), so a crosser that briefly clears/blocks
    # flips straight<->around_l<->around_r each tick (the measured switches~100 thrash). CCF COMMITS to a detour homotopy
    # and holds it -- restoring certified margin by the SLIP speed-warp (YIELD behind the crosser) before EVER switching
    # sides -- and re-opens the tournament only when the committed side is INFEASIBLE at every speed. It returns to
    # goal-direct only after 'straight' has certified for EGO_MAN_RELEASE consecutive ticks (debounce kills the
    # straight<->around flip). Switch trigger = the hard certificate's feasibility, NOT a goal-ward speed deadband.
    # (_committed_warp DELETED 2026-07-07: orphaned after CCF/CRET removal; the retime
    #  identity lives on in SL.cert_clear_warp + maneuver_decide_v2's speed grid.)
    def _straight_clear():
        return (ego.replan(p_d, v_d, a_d, np.array([p_d[0] + gdir[0] * L, p_d[1] + gdir[1] * L, CRUISE_Z]))
                and ego.duration() > 1e-3 and cert_clear() and static_clear() and mover_clear_flown())

    if EGO_DECIDE == "v3":
        # CPL-v3 (stage-3): plan a certified LOCAL composite INSIDE the certified set (no discrete
        # tournament). Same cyl keep-out as v2; EGO supplies the global guide; the executor flies the
        # composite via plan_eval (not ego.eval). None -> certified brake -> hold (never blind-flee).
        import local_lattice as _LL
        _v3dt = float(os.environ.get("V3_DT", "0.3"))
        if _V3_PLAN[0] is not None and _V3_ST.get("t", 1e9) < _v3dt - 1e-6:
            return "cpl", None, 1.0          # COMMIT-AND-FLY: keep flying the committed segment. The
            #   composite's recursive feasibility makes the whole [0,_v3dt] sound, so re-plan only when it
            #   is flown out -- flying just REPLAN_DT of a from-rest quintic never builds speed (crept in place).
        _cyl3 = []
        for (_oid, c3, vel, r_obs, d_safe) in movers:
            q_c, veff_c = PERCLASS_CONF.get(d_safe, (MAN_QCONF, MAN_VEFF))
            _cyl3.append((np.asarray(c3, float), np.array([float(vel[0]), float(vel[1]), 0.0]),
                          np.zeros(3), r_obs + MAN_DSAFE + q_c + MAN_TRACK,
                          2.0 * float(c3[2]) + MAN_REACH_PAD + MAN_DSAFE_V + q_c + MAN_TRACK, veff_c))
        _g3 = np.array([cur_wp[0], cur_wp[1], CRUISE_Z], float)
        _vmx = float(os.environ.get("V3_VMAX", "3.0")); _amx = float(os.environ.get("V3_AMAX", "6.0"))
        _v3dt = float(os.environ.get("V3_DT", "0.3"))   # committed-segment horizon: must be long enough to
        #   BUILD speed from near-zero (0.1s commits never accelerate -> the drone crept in place). Re-planned
        #   every REPLAN_DT with warm-start, so only the segment head is flown before the next solve.
        _guide = None
        if ego.replan(p_d, v_d, a_d, _g3) and ego.duration() > 1e-3:
            _ge = ego.eval(min(_LL.T_P, ego.duration() - 1e-3))
            if _ge is not None:
                _guide = _LL.quintic3(p_d, v_d, a_d, np.asarray(_ge[0], float),
                                      np.asarray(_ge[1], float), np.zeros(3), _LL.T_P)
        _plan, _prim, _tag, _diag = _LL.plan_local(p_d, v_d, a_d, _g3, z_top, _cyl3,
                                                   v_max=_vmx, a_max=_amx, dt=_v3dt, delta=REPLAN_DT,
                                                   incumbent=_V3_ST.get("prim"), guide=_guide)
        if _plan is None:
            _V3_ST.pop("prim", None)
            _bs, _bd, _bt = _LL.make_composite(
                _LL.quintic3(p_d, v_d, a_d, p_d, v_d * 0.0, np.zeros(3), _LL.T_P), _amx, _v3dt)
            if _LL.certify_composite(_bs, _bd, _cyl3, _bt, REPLAN_DT)[0]:
                _V3_PLAN[0] = (_bs, _bd, _bt); _V3_ST["t"] = 0.0; return "brake", None, 1.0
            if os.environ.get("V3_ESC", "0") == "1":
                # escape tree: L1.5 fresh dodge-to-rest grid; L2 keep flying the incumbent's
                # PRE-CERTIFIED branch (certified last commit, absolute window covers now) --
                # a certified maneuver replaces the frozen uncertified hold
                _ep = _LL.escape_fallback(p_d, v_d, a_d, _cyl3, _amx, _v3dt, REPLAN_DT, v_max=_vmx)
                if _ep is not None:
                    _V3_PLAN[0] = _ep; _V3_ST["t"] = 0.0; return "brake", None, 1.0
                _old = _V3_PLAN[0]
                if _old is not None and _V3_ST.get("t", 1e9) + REPLAN_DT <= _old[2]:
                    return "brake", None, 1.0
            _V3_PLAN[0] = None; _V3_ST["t"] = 0.0; return "hold", None, 1.0
        _V3_ST["prim"] = _prim; _V3_PLAN[0] = _plan; _V3_ST["t"] = 0.0
        _pts3 = [_LL.plan_eval(_plan, u)[0] for u in np.linspace(0, _plan[2], 24)]
        return ("cpl" if _tag not in ("hover",) else "hold"), _pts3, 1.0

    if EGO_DECIDE == "v2":
        _cyl = []
        for (_oid, c3, vel, r_obs, d_safe) in movers:
            q_c, veff_c = PERCLASS_CONF.get(d_safe, (MAN_QCONF, MAN_VEFF))
            _e7 = _ell_of_mover(_oid, vel, r_obs)
            _c7 = _cap_of_mover(_oid, vel, r_obs)
            tag7 = ()
            if _c7 is not None:                 # v6 arm: capsule law (q̃ + pearl-gap veff) + cap tag
                q_c, veff_c, _K6, _rr6 = _c7
                _sp6 = float(np.hypot(vel[0], vel[1]))
                if _rr6 is not None and _sp6 > 1e-6:   # v6.1 rear-plane disjunct parameters
                    _th6 = float(_rr6[0]) + float(r_obs) + MAN_DSAFE + MAN_TRACK
                    tag7 = (("cap", _K6, float(vel[0]) / _sp6, float(vel[1]) / _sp6, _th6, float(_rr6[1])),)
                else:
                    tag7 = (("cap", _K6),)
            elif _e7 is not None:               # v5 arm: this mover's law = calib_v5 mature numbers
                _k5, _ux5, _uy5, _rw5, q_c, veff_c = _e7
                if _k5 > 1.0 + 1e-9:
                    tag7 = ((_k5, _ux5, _uy5, _rw5),)
            _cyl.append((np.asarray(c3, float), np.array([float(vel[0]), float(vel[1]), 0.0]),
                         np.zeros(3), r_obs + MAN_DSAFE + q_c + MAN_TRACK,
                         2.0 * float(c3[2]) + MAN_REACH_PAD + MAN_DSAFE_V + q_c + MAN_TRACK, veff_c)
                        + tag7)
        kind, s_v2 = _SL.maneuver_decide_v2(
            ego, p_d, v_d, a_d, np.asarray(cur_wp, float), z_top, _cyl, _MAN_V2,
            cruise_z=CRUISE_Z, horizon=L, straight_clip=EGO_HOR,
            tau=EGO_TAU_TRUST, delta=REPLAN_DT, carrot="angle",
            speeds=(1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3),   # CCF-granularity warp grid (cert is 7us)
            extra_gate=lambda: static_clear() and mover_clear_flown())
        if kind in ("around_l2", "around_r2"):
            kind = kind[:-1]
        if kind == "evade":
            kind = "hold"                       # renderer semantics: never blind-flee into buildings
        dur = ego.duration()
        pts = [ego.eval(u)[0] for u in np.linspace(0, dur, 24)] if (kind != "hold" and dur > 1e-3) else None
        return kind, pts, (float(s_v2) if kind != "hold" else 1.0)

    chosen = None; man_g = 1.0
    # (CCF commitment block DELETED 2026-07-07: superseded by SL.maneuver_decide_v2's built-in
    #  incumbent retry + angle carrot + speed grid. Reproduce old behaviour with EGO_DECIDE=v1 --
    #  which now runs the PLAIN tournament below, i.e. the pre-CCF default.)
    _csub = _MAN_STATE.get("sub")                       # v1 tournament still prefers the committed SIDE
    if chosen is None:
        # Tournament: straight -> SAME-side detour -> mirror -> wider; then fly OVER -> climb -> HOLD. EACH candidate must
        # clear the mover cert AND the static gate. (blind EVADE not used: dense static -> fleeing drives into buildings.)
        cands = [("straight", 0.0), ("around_l", PHI_MAN), ("around_r", -PHI_MAN),
                 ("around_l", 2.0 * PHI_MAN), ("around_r", -2.0 * PHI_MAN)]
        _pri = lambda c: 0 if c[0] == "straight" else (1 if (_csub is not None and c[1] * _csub > 0) else 2)
        cands.sort(key=_pri)                                                 # stick to the previously-committed SIDE
        _ang = 0.0
        for gk, ang in cands:
            gsub = np.array([*(p_d[:2] + L * rot(gdir, ang)), CRUISE_Z])
            if (ego.replan(p_d, v_d, a_d, gsub) and ego.duration() > 1e-3
                    and cert_clear() and static_clear() and mover_clear_flown()):
                chosen = gk; _ang = ang; break
        if chosen is None:
            # ground blocked -> fly OVER if certified; else a CERTIFIED vertical climb-escape; else HOLD (brake/hover; NEVER
            # fly uncertified -- that was the seed-56 soundness hole where a labelled-certified lap could collide).
            if (ego.replan(p_d, v_d, a_d, np.array([gdir[0] * L + p_d[0], gdir[1] * L + p_d[1], z_top]))
                    and ego.duration() > 1e-3 and cert_clear() and static_clear() and mover_clear_flown()):
                chosen = "over"
            elif (ego.replan(p_d, v_d, a_d, np.array([p_d[0], p_d[1], z_top]))
                    and ego.duration() > 1e-3 and cert_clear() and static_clear() and mover_clear_flown()):
                chosen = "climb"                                  # certified vertical escape
            else:
                chosen = "hold"                                   # uncertifiable -> brake/hover, do NOT fly uncertified
        _MAN_STATE["sub"] = _ang if chosen in ("around_l", "around_r") else None
        _MAN_STATE["streak"] = 0
    _MAN_STATE["kind"] = chosen
    dur = ego.duration()
    pts = [ego.eval(s)[0] for s in np.linspace(0, dur, 24)] if dur > 1e-3 else None
    return chosen, pts, man_g


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
    # STATIC: local KNOWN map when EGO_STATIC_MAP=1 (native gets the same realistic static both ours does), else cone.
    _st = _percept_static(p_drone, heading)
    if len(_st):
        chunks.append(_st)

    def _seen(c):
        dd = c[:2] - p_drone[:2]; r = float(np.linalg.norm(dd))
        return r <= R and (float(dd @ fwd) / max(r, 1e-6)) >= cmax
    _eta_on = os.environ.get("ETA_CLOUD", "0") == "1"

    def _eta_pos(pos, vel):
        # ANTICIPATORY OCCUPANCY (ETA projection): place the mover's planning-obstacle where the
        # mover WILL BE when the drone ARRIVES there -- "use the agents' running logic to fly
        # faster/smoother" (the planner threads predicted gaps instead of dodging the past; the
        # CERTIFICATE still judges the true predicted tubes, so safety semantics are untouched).
        v_eta = max(0.6 * float(PLN.get("v_max", 6.0)), 1.0)
        q = np.asarray(pos, float).copy(); vv = np.asarray(vel, float)
        for _ in range(2):                                  # fixed-point ETA iteration (2 rounds)
            eta = min(float(np.linalg.norm(q[:2] - p_drone[:2])) / v_eta, 2.5)
            q = np.asarray(pos, float) + vv * eta
        return q

    for oid, cls, pos, vel, size in native_objects():
        if os.environ.get("MU_ANOM", "0") == "1":
            # statics INCLUDED: the sky-bicycle glitch is a PROP spawned at a bad z, class=static --
            # the first monitor version skipped statics and was blind to it (2026-07-08)
            _z_a = float(pos[2]) if len(np.atleast_1d(pos)) > 2 else 0.0
            if _z_a > 2.0 and cls == "static":
                print(f"[MU-ANOM] t={t_sim:.1f} AIRBORNE-PROP {cls} oid={oid} z={_z_a:.1f}", flush=True)
        if cls == "static": continue
        if os.environ.get("MU_ANOM", "0") == "1":
            _sp_a = float(np.hypot(vel[0], vel[1]))
            if _sp_a > 12.0 or abs(float(pos[2]) if len(np.atleast_1d(pos)) > 2 else 0.0) > 2.5:
                print(f"[MU-ANOM] t={t_sim:.1f} {cls} oid={oid} |v|={_sp_a:.1f} z={float(pos[2]) if len(np.atleast_1d(pos))>2 else 0:.1f}", flush=True)
        _p = _eta_pos(pos, vel) if _eta_on else pos
        if _seen(p3(_p, size[2] * 0.5)) or (_eta_on and _seen(p3(pos, size[2] * 0.5))):
            chunks.append(np.asarray(_voxel_box(_p, size), float))
    for a in animals:
        pos = a.p0 + a.vel * t_sim
        _p = _eta_pos(pos, a.vel) if _eta_on else pos
        if _seen(p3(_p, a.size[2] * 0.5)) or (_eta_on and _seen(p3(pos, a.size[2] * 0.5))):
            chunks.append(np.asarray(_voxel_box(_p, a.size), float))
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
            _cn = getattr(a, "cls_name", "animal")
            _add_native_traj(100000 + a.id, a.size, pos, a.vel, _cn, t_sim)
            fed.append((_cn, c3, a.size))
    native.clean_old_trajs(t_sim)
    return fed


def clearance(p, fed):
    r = float(par.drone_radius); gmin = np.inf; per = {}
    for cls, c3, size in fed:
        d = p - c3; sz = np.asarray(size, float)
        if cls == "static":   # round trees/poles -> VERTICAL CYLINDER (radius = half the square footprint), not a box:
            R = 0.5 * sz[0]; hh = 0.5 * sz[2]              # matches the real round mesh; no over-conservative corners
            dh = float(np.hypot(d[0], d[1])); dz = abs(d[2])
            dr_out = max(dh - R, 0.0); dz_out = max(dz - hh, 0.0)
            sd = ((np.hypot(dr_out, dz_out)) if (dr_out > 0 or dz_out > 0) else -min(R - dh, hh - dz)) - r
        else:
            halfb = 0.5 * sz
            outside = np.maximum(np.abs(d) - halfb, 0.0)
            sd = (np.linalg.norm(outside) if np.any(outside > 0) else -np.min(halfb - np.abs(d))) - r
        gmin = min(gmin, sd); per[cls] = min(per.get(cls, np.inf), sd)
        if cls == "static" and sd < _min_static[0]:
            _min_static[0] = sd; _min_static[1] = np.asarray(size, float).copy(); _min_static[2] = np.asarray(p, float).copy()
    return gmin, per


FONT = cv2.FONT_HERSHEY_SIMPLEX


_CAM_EMA = [None]


def grab(pos, hpr, gimbal=False):
    """offscreen GPU frame at (pos,hpr) rel. to the drone (BGR uint8). ONE renderFrame per view
    (multi_thread_render=False makes a single pass correct) instead of perceive()'s 2x taskMgr.step.
    gimbal=True + CAM_SMOOTH>0: chase cam rides a DAMPED virtual gimbal (EMA position+yaw in the
    WORLD frame) instead of being bolted to the airframe -- every micro-tilt of the body otherwise
    shakes the whole frame, which reads as high-frequency judder even when the flight is smooth."""
    a = float(os.environ.get("CAM_SMOOTH", "0"))
    if gimbal and a > 0:
        p_now = np.asarray(drone.origin.getPos(eng.render), float)
        yaw_now = float(np.radians(drone.origin.getH(eng.render)))
        st = _CAM_EMA[0]
        if st is None:
            st = [p_now, yaw_now]
        else:
            st[0] = (1 - a) * st[0] + a * p_now
            dy = (yaw_now - st[1] + np.pi) % (2 * np.pi) - np.pi
            st[1] = st[1] + a * dy
        _CAM_EMA[0] = st
        ca, sa = np.cos(st[1]), np.sin(st[1])
        off = np.array([pos[0] * ca - pos[1] * sa, pos[0] * sa + pos[1] * ca, pos[2]])
        cam.cam.reparentTo(eng.render)
        cam.cam.setPos(Vec3(*(st[0] + off)))
        cam.cam.setHpr(Vec3(np.degrees(st[1]) + hpr[0], hpr[1], hpr[2]))
        eng.graphicsEngine.renderFrame()
        a2 = np.asarray(cam.get_rgb_array_cpu())
        if a2.dtype != np.uint8:
            a2 = (a2 * 255).astype(np.uint8) if a2.max() <= 1.01 else a2.astype(np.uint8)
        return a2
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
        out["chase"] = grab(CHASE_POS, CHASE_HPR, gimbal=True)
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
_MP4_OUT = os.environ.get("OUT_MP4") or os.path.join(_HERE, "out", "drone_3d.mp4")  # per-job override -> safe parallel renders
writer = imageio.get_writer(_MP4_OUT, fps=args.fps) if args.mp4 else None
WIN = "MetaUrban x SANDO — REAL-TIME 3D (FPV + 3rd person + planned path)"
_LIVE_PG = {}                      # --live pygame window state (lazy init at first composed frame)

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
if os.environ.get("TREE_DBG") == "1":   # diagnostic: does WLH match the real mesh extent (getTightBounds)?
    from collections import Counter
    cnt = Counter(); szs = {}; tight = {}
    for oid, o in eng.get_objects().items():
        if classify(o) == "static":
            tn = type(o).__name__; cnt[tn] += 1
            w, l, h = obj_size(o)
            szs.setdefault(tn, []).append((w, l, h))
            for getter in (lambda: o.origin.getTightBounds(), lambda: o.origin.node().getBounds()):
                try:
                    bb = getter()
                    if bb is not None and hasattr(bb, "__getitem__"):
                        lo, hi = bb; ext = (hi[0]-lo[0], hi[1]-lo[1], hi[2]-lo[2])
                        if ext[0] > 0: tight.setdefault(tn, []).append(ext); break
                except Exception:
                    pass
    print("[treedbg] static types: WLH collision box vs real mesh getTightBounds:", flush=True)
    for tn, n in cnt.most_common():
        a = np.asarray(szs[tn]); m = np.median(a, axis=0)
        tt = (f"  tight W={np.median([t[0] for t in tight[tn]]):.2f} L={np.median([t[1] for t in tight[tn]]):.2f} "
              f"H={np.median([t[2] for t in tight[tn]]):.2f}") if tn in tight else "  (no tight bounds)"
        print(f"[treedbg]   {tn}: n={n}  WLH W={m[0]:.2f} L={m[1]:.2f} H={m[2]:.2f}{tt}", flush=True)

_STALE_CNT = [0, 0]                    # [stale ticks, flown ticks] -- composition-theorem branch 2 freq
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
        SPAWN_CLR_MIN = 1.5; GOAL_PED_MIN = 3.0
        def _goal_idle_ped_gap(g_xy):   # dist to nearest STATIONARY ped (|v|<0.2): a frozen person on the
            gaps = [float(np.linalg.norm(p - g_xy)) for (_, c, p, v, _) in native_objects()   # goal approach
                    if c == "pedestrian" and float(np.linalg.norm(v)) < 0.2]                  # strands the drone
            return min(gaps) if gaps else 1e9                                                 # (climb, can't
        best = (-1e9, route, axis, left)                                                      #  descend; seed23)
        for att in range(16):
            s0 = np.asarray(route[0], float); g0 = np.asarray(route[-1], float)
            c0, _ = clearance(s0, feed(None, {}, 0.0, s0))
            cg, _ = clearance(g0, feed(None, {}, 0.0, g0))   # GOAL must be OPEN too: a cluttered goal makes
            gped = _goal_idle_ped_gap(g0[:2])                #   the drone climb near it and fail to descend
            score = min(min(c0, cg) - SPAWN_CLR_MIN, gped - GOAL_PED_MIN)  # BOTH must clear their targets
            if score > best[0]: best = (score, route, axis, left)
            if score >= 0.0: break
            route, axis, left = plan_route(lap_idx + (att + 1) * 7919)   # different start/route, same seed family
        _sc, route, axis, left = best
        print(f"[3dv] clear_spawn: START/GOAL clr + goal-idle-ped gap >= {GOAL_PED_MIN}m "
              f"(margin {_sc:+.2f}m over targets)", flush=True)
    if _STALE_CNT[1]:
        print(f"[theorem] stale-spline branch: {_STALE_CNT[0]}/{_STALE_CNT[1]} flown ticks "
              f"({100.0*_STALE_CNT[0]/_STALE_CNT[1]:.1f}%)", flush=True)
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
    if os.environ.get("EGO_ADV_CROSSER") == "1":
        # ADVERSARIAL side-crosser: a fast mover timed to reach the drone's path midpoint EXACTLY as the drone does,
        # entering from ~90 deg (OUTSIDE the +-fov_deg forward cone) so it is detected only when already close -- the
        # "see-too-late" stress that gives the FOV failure a non-zero collision denominator (default scene = 0 collisions).
        # Deterministic: speed + lateral offset are derived from the drone's expected arrival time at _mid (no random/Date).
        _vcru = float(os.environ.get("EGO_ADV_CROSS_VCRU", 3.0))                  # REALISTIC avg drone speed (not v_max:
        _t_mid = float(np.linalg.norm(_mid - START[:2])) / max(_vcru, 0.5)        # the cert weaves+brakes ~2.7 m/s) -> ETA at _mid
        _vx = float(os.environ.get("EGO_ADV_CROSS_SPEED", 2.5))                   # crosser speed (m/s; brisk run)
        _side = 1.0 if os.environ.get("EGO_ADV_CROSS_SIDE", "L") == "L" else -1.0
        _cow.place(_mid + left * _side * (_vx * _t_mid), -left * _side * _vx)     # reaches _mid at t=_t_mid, perpendicular
    else:
        _cow.place(_mid + left * 6.0, -left * 1.0)        # cow crosses near mid-route
    sando = make_sando(lap_start, wp[0]) if (ego is None and native is None) else None; _cache = {}
    if native is not None: native.set_terminal_goal(wp[0])
    quad.reset(np.asarray(lap_start, float), yaw=hdg0)
    occ_reset()                                          # fresh D435i occupancy memory per lap
    if px4 is not None:
        p_d, _yw0, v_d, _ = px4.get_pose_world(); p_d = np.asarray(p_d, float); v_d = np.asarray(v_d, float)
    else:
        p_d = quad.p.copy(); v_d = np.zeros(3)
    a_d = np.zeros(3)
    t = 0.0; last_rt = 0.0; reached = False; crashed = False; mclr = np.inf; per_all = {}; iters = 0; seam_bias_max = 0.0
    ego_n_cert = 0; ego_n_hold = 0; ego_n_slow = 0; ego_cert_hold = False; ego_hold_class = None
    ego_speed_g = 1.0; ego_g_prev = 1.0   # --ego_safe graded anticipatory-brake speed scale [0,1] (+ release LPF)
    next_goal_pos = None; _last_wall = time.perf_counter()
    print(f"[3dv] lap {lap_idx-1}: {len(route)}-pt route len~{np.linalg.norm(GOAL[:2]-START[:2]):.0f}m "
          f"start {np.round(START[:2],1)}", flush=True)
    ego_dur = 0.0; t_ego = 0.0; ego_stuck = 0; ego_traj_pts = None; man_kind = None
    man_switches = 0; man_counts = {}; _prev_mk = None; _MAN_STATE["kind"] = None   # reset hysteresis per lap
    _sw_lr = 0; _sw_sa = 0; _sw_oth = 0; _sp_hist = []   # thrash instrumentation: classify switches + speed-history cost
    _KF.clear()                                          # fresh mover trackers per lap (no stale cross-lap KF state)
    if _PFE is not None:
        _PFE.tracks.clear()                              # realistic front-end: no stale cross-lap tracks either
        _PFE_MEMO["t"] = None                            # and no stale same-t memo across laps
    while t < T_MAX and not reached:
        cur_wp = wp[wp_i]
        if ego is not None:
            # EGO-Planner core: perceive ONLY the depth-camera FOV cloud (not GT omniscience) PLUS the ground
            # plane (floor knowledge isn't FOV-limited), aim at the waypoint clipped to the receding horizon.
            heading = float(quad.yaw)
            # perception source: real D435i depth (occlusion-limited surfaces) when --d435i, else the GT fov_cloud
            _percept = d435i_cloud(p_d, heading) if args.d435i else fov_cloud(p_d, heading, t)
            cloud = np.concatenate([_percept, ground_patch(p_d, radius=EGO_HOR + 4.0)], axis=0)
            if args.slip:                                              # route EGO around movers at the cert margin
                cloud = np.concatenate([cloud, _slip_mover_cloud(p_d, t)], axis=0)
            ego.update_cloud(cloud, p_d)
            man_kind = None; man_g = 1.0
            if args.maneuver:
                # NO-HOLD cylinder fastest-safe tournament (fly over / around / climb); leaves EGO holding the winner
                t0 = time.perf_counter(); man_kind, ego_traj_pts2, man_g = ego_maneuver_replan(p_d, v_d, a_d, cur_wp, t)
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
                s_raw, wc = ego_speed_search(p_d, v_d, t, u0=t_ego)
                g = s_raw if s_raw < ego_g_prev else min(s_raw, ego_g_prev + EGO_G_RELEASE)   # brake free, release slow
                while g >= EGO_S_MIN - 1e-9 and not ego_slip_feasible(p_d, v_d, t, g, u0=t_ego):  # RE-CERT THE FLOWN SCALE
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
            elif args.maneuver and man_kind == "hold":
                # NOTHING certified (boxed) -> safe RTA backstop: hover/brake in place (executor s=None holds position).
                # NO uncertified flight -> restores 'flown == certified' (was the climb-fallback soundness hole, e.g. seed 56).
                ego_cert_hold = True; ego_hold_class = "boxed"; ego_n_hold += 1
            # (EGO_FOVCAP speed-FOV warp DELETED 2026-07-07: its comment promised "future anti-thrash
            #  work (commit hysteresis)" -- that future is maneuver_decide_v2's speed grid.)
            elif args.maneuver and ego_dur > 1e-3:
                # CCF yield-behind warp: ego_maneuver_replan returns the fastest certified speed on the COMMITTED side
                # (1.0 unless it is slowing behind a crosser to HOLD the side instead of switching). Brake fast, release slow.
                g_raw = float(man_g)
                ego_speed_g = g_raw if g_raw < ego_g_prev else min(g_raw, ego_g_prev + EGO_G_RELEASE)
                ego_g_prev = ego_speed_g
                if ego_speed_g >= 0.999:
                    ego_n_cert += 1
                else:
                    ego_hold_class = "yield"; ego_n_slow += 1
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
            yaw_ref = _yaw_smooth(float(np.arctan2(cur_wp[1] - p_d[1], cur_wp[0] - p_d[0])))
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
        elif ego is not None and EGO_DECIDE == "v3":
            # CPL-v3 executor: fly the committed composite plan (plan_eval), NOT the EGO B-spline.
            import local_lattice as _LL3
            _pl = _V3_PLAN[0]
            for _k in range(int(round(REPLAN_DT / DT))):
                yaw_ref = _yaw_smooth(float(np.arctan2(cur_wp[1] - quad.p[1], cur_wp[0] - quad.p[0])))
                _tt = _V3_ST.get("t", 0.0)                 # ACCUMULATED flown time into the committed plan
                if _pl is not None:
                    _rr = _LL3.plan_eval(_pl, min(_tt + DT, _pl[2]))   # track one step ahead -> builds speed
                    _sp = np.asarray(_rr[0], float).copy()
                    if _sp[2] < MIN_FLY_Z:
                        _sp[2] = MIN_FLY_Z
                    next_goal_pos = _sp.copy()
                    quad.step(_sp, np.asarray(_rr[1], float), np.asarray(_rr[2], float), DT, yaw_ref=yaw_ref)
                else:
                    quad.step(quad.p, np.zeros(3), np.zeros(3), DT, yaw_ref=yaw_ref)   # hold in place
                _V3_ST["t"] = _tt + DT
                t += DT
            p_d = quad.p.copy(); v_d = quad.v.copy(); a_d = quad.a.copy()   # write flown state back
            drone.set_position([float(p_d[0]), float(p_d[1]), float(p_d[2])])
            drone.set_heading_theta(float(quad.yaw))
            if drone_model is not None:
                _pitch, _roll = quad.tilt_deg(); drone_model.setHpr(0.0, _pitch, _roll)
            if _TELEM is not None:
                _tp3, _tr3 = quad.tilt_deg(); _TELEM.append(_telem_row(t, _tp3, _tr3, quad, t))
        elif ego is not None:
            for _ in range(int(round(REPLAN_DT / DT))):
                yaw_ref = _yaw_smooth(float(np.arctan2(cur_wp[1] - quad.p[1], cur_wp[0] - quad.p[0])))
                # --ego_safe ANTICIPATORY BRAKE: advance along the SAME B-spline path at the graded rate
                # ego_speed_g (time-warp). g<1 -> slower (smooth deceleration as a mover gets close);
                # g==0 -> ego_cert_hold -> s=None -> hover (already near-stopped, so no jerk), climb if stuck.
                s = (ego.eval(min(t_ego + ego_speed_g * DT, max(ego_dur - 1e-3, 0.0)))
                     if (ego_dur > 1e-3 and not ego_cert_hold) else None)
                if s is not None:
                    _STALE_CNT[1] += 1
                    if t_ego > REPLAN_DT * 1.5:
                        _STALE_CNT[0] += 1        # flying a spline older than one replan period =
                        #                            the theorem's STALE branch; freq printed per lap
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
                    if args.pointmass:                                       # DIAGNOSTIC: flown == planned (teleport)
                        quad.p = np.asarray(sp_pos, float).copy(); quad.v = np.asarray(sp_vel, float).copy()
                        quad.a = np.asarray(sp_acc, float).copy(); quad.yaw = float(yaw_ref)
                    elif os.environ.get("SMOOTH_EXEC", "0") == "1":
                        # CONTINUOUS-REFERENCE executor: one spline sample held for a whole tick is a
                        # stair-step target -- the quad lunges/overshoots/brakes every tick (revs=49
                        # accel<->decel reversals = the residual high-frequency judder even at 3 m/s).
                        # Sample the SAME certified spline at 3 sub-instants instead: flown path hugs
                        # the plan tighter (uses LESS delta_track budget -- strictly sound).
                        _tbase = t_ego - ego_speed_g * DT     # t_ego already advanced by g*DT above
                        for _k in (1, 2, 3):
                            _rr = ego.eval(min(_tbase + ego_speed_g * DT * (_k / 3.0),
                                               max(ego_dur - 1e-3, 0.0)))
                            _sp, _sv, _sa = _rr
                            _sp = np.asarray(_sp, float).copy()
                            if _sp[2] < MIN_FLY_Z:
                                _sp[2] = MIN_FLY_Z
                            quad.step(_sp, np.asarray(_sv, float) * ego_speed_g,
                                      np.asarray(_sa, float) * (ego_speed_g ** 2), DT / 3.0,
                                      yaw_ref=yaw_ref)
                    else:
                        quad.step(sp_pos, sp_vel, sp_acc, DT, yaw_ref=yaw_ref)   # quad tracks the EGO B-spline (slowed)
                    if _TRACKH:   # HCT-D harvest: tracking error + hodograph covariates of THIS planned set-point
                        nv = float(np.linalg.norm(sp_vel)); na = float(np.linalg.norm(sp_acc))
                        lat = float(np.linalg.norm(np.cross(sp_vel, sp_acc))) / max(nv, 1e-3)   # centripetal = kappa*v^2
                        _track_rows.append((int(iters), float(np.linalg.norm(quad.p - sp_pos)), nv, na, lat))
                elif ego_stuck < 3:
                    _vq = float(np.linalg.norm(quad.v[:2]))
                    if os.environ.get("HOLD_DECEL", "0") == "1" and _vq > 0.4:
                        # BRAKING REFERENCE FIELD (graceful stop): the physical stop path is momentum-
                        # dominated either way; freezing the reference at quad.p just adds a reference
                        # step the controller answers with a violent pitch-up. Command a decaying
                        # velocity-aligned reference instead -- same stop, no attitude spike.
                        _beta = 0.5
                        quad.step(quad.p + quad.v * DT * _beta, quad.v * _beta, np.zeros(3), DT,
                                  yaw_ref=yaw_ref)
                    else:
                        quad.step(quad.p, np.zeros(3), np.zeros(3), DT, yaw_ref=yaw_ref)   # hover
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
            if _TELEM is not None:
                _tp, _tr = quad.tilt_deg()
                _TELEM.append(_telem_row(t, _tp, _tr, quad, t))
        elif native is not None:
            for _ in range(int(round(REPLAN_DT / DT))):
                yaw_ref = _yaw_smooth(float(np.arctan2(cur_wp[1] - quad.p[1], cur_wp[0] - quad.p[0])))
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
            if _TELEM is not None:
                _tp2, _tr2 = quad.tilt_deg()
                _TELEM.append(_telem_row(t, _tp2, _tr2, quad, t))
        else:
            for _ in range(int(round(REPLAN_DT / DT))):
                yaw_ref = _yaw_smooth(float(np.arctan2(cur_wp[1] - quad.p[1], cur_wp[0] - quad.p[0])))   # face current waypoint
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
                _ar = ("around_l", "around_r")
                if _prev_mk in _ar and man_kind in _ar:
                    _sw_lr += 1                              # L<->R flip = the WASTEFUL thrash CCF targets
                elif ("straight" in (_prev_mk, man_kind)) and (_prev_mk in _ar or man_kind in _ar):
                    _sw_sa += 1                              # straight<->around = FUNCTIONAL (return-to-goal progress)
                else:
                    _sw_oth += 1                             # involves over/climb/hold
            _prev_mk = man_kind
        _sp_hist.append(float(np.hypot(quad.v[0], quad.v[1])))   # flown horizontal speed (thrash cost: jerk + reversals)
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
        if not args.headless:
            draw_path(gpath, next_goal_pos, p_d)   # <- the path the drone just chose
            if args.maneuver or args.slip:
                draw_predictions()                 # <- the LIVE Kalman forecast every mover is routed around
        step_env()
        c, per = clearance(p_d, fed); mclr = min(mclr, c)
        for k, val in per.items(): per_all[k] = min(per_all.get(k, np.inf), val)
        if c < -1e-6 and os.environ.get("MAN_COLLDBG") == "1":
            # which fed entry is the offender, and what did the cert see for it?
            hit_cls = min(per, key=per.get)
            off = min(((cl, c3, sz) for (cl, c3, sz) in fed if cl == hit_cls),
                      key=lambda e: (np.linalg.norm((p_d - e[1])[:2]) - 0.5 * max(e[2][0], e[2][1])))
            ocls, oc3, osz = off
            # attribute against the SAME forward cone the planner/cert actually used (cam_heading=quad.yaw), NOT the
            # omniscient 30 m SENSE_R gate -- otherwise an out-of-cone offender still shows up here and gets mislabeled
            # as a benign small-KF-error instead of the true "never seen / outran the cone" cause (colldbg-masks-cone-blindness).
            mv = next(((oid, mc3, mvel, r_obs, ds) for (oid, mc3, mvel, r_obs, ds) in kf_movers(p_d, t, cam_heading=float(quad.yaw))
                       if np.linalg.norm((np.asarray(mc3)[:2] - np.asarray(oc3)[:2])) < 1.5), None)
            print(f"\n[COLLDBG] t={t:.2f}s class={hit_cls} clr={per[hit_cls]:+.3f}m", flush=True)
            print(f"[COLLDBG] drone p={np.round(p_d,2)} v=({v_d[0]:.2f},{v_d[1]:.2f},{v_d[2]:.2f}) |v|={np.linalg.norm(v_d):.2f}", flush=True)
            print(f"[COLLDBG] GT obstacle pos={np.round(oc3,2)} size={np.round(osz,2)} "
                  f"box_halfR=({0.5*osz[0]:.2f},{0.5*osz[1]:.2f}) cyl_r={0.5*max(osz[0],osz[1]):.2f}", flush=True)
            if mv is not None:
                oid, mc3, mvel, r_obs, ds = mv
                print(f"[COLLDBG] KF est pos={np.round(mc3,2)} vel=({mvel[0]:.2f},{mvel[1]:.2f}) |vel|={np.hypot(mvel[0],mvel[1]):.2f} "
                      f"r_obs={r_obs:.2f} d_safe={ds:.2f}  cert_R={r_obs+MAN_DSAFE+MAN_QCONF+MAN_TRACK:.2f} "
                      f"(=r_obs+DSAFE{MAN_DSAFE}+QCONF{MAN_QCONF}+TRACK{MAN_TRACK})", flush=True)
                gt_v = np.asarray(oc3) - np.asarray(mc3)
                print(f"[COLLDBG] KF-vs-GT pos err={np.linalg.norm(gt_v[:2]):.2f}m  "
                      f"flown-vs-plan tracking: see static gate (mover gate has NO forward-sim)", flush=True)
            else:
                print(f"[COLLDBG] (mover not in kf_movers list -> NOT tracked/fed to cert!)", flush=True)
        if c < -1e-6 and os.environ.get("NO_STOP_ON_CRASH") != "1":
            crashed = True; break       # a COLLISION is a crash: stop here, NOT a reach (realistic; same for both)
        frame = None
        if not args.headless:   # --headless skips ALL 3-D rendering: same sim/planner/safety, no pixels (fast)
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
            # pygame window, NOT cv2.imshow: cv2's Qt backend deadlocks/black-screens next to Panda3D
            # in the same process (the historical "--live is black" pit) -- pygame coexists fine.
            import pygame as _pg
            if _LIVE_PG.get("s") is None:
                _pg.display.init()
                _LIVE_PG["s"] = _pg.display.set_mode((frame.shape[1], frame.shape[0]))
                _pg.display.set_caption(WIN)
            _surf = _pg.image.frombuffer(np.ascontiguousarray(frame[..., ::-1]).tobytes(),
                                         (frame.shape[1], frame.shape[0]), "RGB")
            _LIVE_PG["s"].blit(_surf, (0, 0)); _pg.display.flip()
            for _e in _pg.event.get():
                if _e.type == _pg.QUIT or (_e.type == _pg.KEYDOWN and _e.key in (_pg.K_ESCAPE, _pg.K_q)):
                    quit_now = True
            if quit_now: break
        if os.environ.get("MAN_TRACE") == "1" and (iters % 20 == 0):
            print(f"[trace] t={t:5.1f}s p=({p_d[0]:6.1f},{p_d[1]:6.1f},{p_d[2]:4.1f}) "
                  f"dgoal={np.linalg.norm(p_d[:2]-GOAL[:2]):5.1f}m wp_i={wp_i} kind={man_kind} "
                  f"vd={np.linalg.norm(v_d[:2]):.2f}", flush=True)
        iters += 1
        final_close = (wp_i == len(wp) - 1 and np.linalg.norm(p_d[:2] - GOAL[:2]) < float(par.goal_radius))  # HORIZONTAL reach: goal is an (x,y) location; requiring exact z stranded the drone hovering above it (seed23)
        if final_close and (sando is None or sando.get_drone_status() == GOAL_REACHED):
            reached = True
    t_goal = t if reached else float("inf")
    _sp = np.asarray(_sp_hist, float)                                                        # thrash COST metrics:
    _rms_jerk = float(np.sqrt(np.mean(np.square(np.diff(_sp, 2))))) / (REPLAN_DT ** 2) if len(_sp) > 2 else 0.0
    _dv = np.diff(_sp) if len(_sp) > 1 else np.zeros(1)
    _revs = int(np.count_nonzero(np.diff(np.sign(_dv)))) if len(_dv) > 1 else 0             # decel<->accel reversals
    _spvar = float(np.sum(np.abs(_dv)))                                                      # total speed churn (energy proxy)
    if _TELEM is not None:
        import json as _json
        _json.dump(_TELEM, open(os.environ["TELEM_OUT"], "w"))
        print(f"[3dv] telemetry -> {os.environ['TELEM_OUT']} ({len(_TELEM)} ticks)", flush=True)
    print(f"[3dv] lap done. reached={reached} t_goal={t_goal:.1f}s collided={mclr < 0} min_clr={mclr:.3f}m  "
          + "  ".join(f"{k}:{v:.2f}" for k, v in sorted(per_all.items()))
          + (f"  seam_bias_max={seam_bias_max:.3f}m" if args.seam else "")
          + (f"  maneuver[switches={man_switches}(lr{_sw_lr}/sa{_sw_sa}/oth{_sw_oth}) jerk={_rms_jerk:.2f} revs={_revs} "
             f"spchurn={_spvar:.1f} " + " ".join(f"{k}:{v}" for k, v in sorted(man_counts.items())) + "]"
             if args.maneuver else "")
          + (f"  egosafe[cert={ego_n_cert} brake={ego_n_slow} hold={ego_n_hold}]" if (args.ego and args.ego_safe) else ""), flush=True)
    if os.environ.get("TREE_DBG") == "1" and _min_static[1] is not None:
        print(f"[treedbg] closest static approach: clearance={_min_static[0]:.3f}m to a box "
              f"W={_min_static[1][0]:.2f} L={_min_static[1][1]:.2f} H={_min_static[1][2]:.2f} "
              f"at drone z={_min_static[2][2]:.2f}m", flush=True)
    if _TRACKH and _track_rows:   # HCT-D harvest: dump (window, delta, ||v||, ||a||, lateral_accel) for calibration
        _td = os.path.join(_HERE, "out", "conformal", "track"); os.makedirs(_td, exist_ok=True)
        np.save(os.path.join(_td, f"track_s{args.seed}.npy"), np.asarray(_track_rows, float))
        print(f"[3dv] track-harvest: {len(_track_rows)} rows -> out/conformal/track/track_s{args.seed}.npy", flush=True)
    # --serve and live+loop_scene keep flying laps forever; everything else stops after one lap
    if not (args.serve or (args.live and args.loop_scene)):
        break
if writer is not None:
    writer.close(); print(f"[3dv] mp4 -> {_MP4_OUT}", flush=True)
try: cv2.destroyAllWindows()
except Exception: pass
env.close()
