"""Headless batch evaluation of the SANDO core in MetaUrban — success / collision / STUCK rate.

Runs N episodes (no rendering, fast). Each episode: a random 2-point route through a local pedestrian
crowd, flown with IDEAL tracking (drone := SANDO's get_next_goal set-point) so we exercise the C++ CORE
directly (no quadrotor-tracking confound). Records, per episode:
  reached    — got within goal_radius of the goal
  collided   — min signed clearance < 0 at any step (a real hit)
  stuck      — never reached, never collided, but progress-to-goal stalled (recovery deadlock / no plan)
  timeout    — ran out of t_max while still making progress (just slow / very long)
and the worst per-class clearance + max single replan time (to catch hangs).

Usage (metaurban env, from the metaurban repo root):
  python eval_batch.py --n 100 --seed0 1000 --reset_every 1
  python eval_batch.py --n 30  --out out/eval.json
"""
import os, sys, time, json, argparse
import numpy as np
import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "isaac"))
sys.path.insert(0, _HERE)
from sando_cpp_bridge import (Parameters, RobotState, DynTraj, SANDO,
                              DroneStatus_GOAL_REACHED as GOAL_REACHED)
from metaurban import SidewalkDynamicMetaUrbanEnv
from metaurban.obs.observation_base import DummyObservation   # skip the 240-ray lidar observe() each step
from metaurban.component.agents.pedestrian.base_pedestrian import BasePedestrian
from metaurban.component.delivery_robot.base_deliveryrobot import BaseDeliveryRobot
from metaurban.component.robotdog.base_robotdog import BaseRobotDog
from metaurban.component.vehicle.base_vehicle import BaseVehicle

with open(os.path.join(_HERE, "metaurban_sando.yaml"), encoding="utf-8") as f:
    CFG = yaml.safe_load(f)
PLN, LOOP = CFG["planner"], CFG["loop"]

ap = argparse.ArgumentParser()
ap.add_argument("--n", type=int, default=100, help="number of episodes")
ap.add_argument("--seed0", type=int, default=1000)
ap.add_argument("--t_max", type=float, default=30.0)
ap.add_argument("--reset_every", type=int, default=1, help="re-spawn the scene every k episodes (1 = fresh each)")
ap.add_argument("--route_len", type=float, default=0.0, help="0 = random per-episode in [min_goal_dist, 120]m")
ap.add_argument("--map", type=str, default="X", help="map topology: 'X' intersection, or an int N for a random N-block road sequence")
ap.add_argument("--density", type=str, default="dense", choices=["sparse", "med", "dense"])
ap.add_argument("--animals", type=int, default=2, help="synthetic crossing animals (label [2]) injected per episode")
ap.add_argument("--out", type=str, default="out/eval.json")
ap.add_argument("--diag", action="store_true", help="Family-A diagnosis: per ep print route len, start/goal static clearance, global-path length, motion in first 5 replans (no full flight)")
args = ap.parse_args()

# density tiers (the Mondrian benchmark axis) — pick via --density
DENSITY = {
    "sparse": dict(object_density=0.3, spawn_human_num=20, spawn_wheelchairman_num=1, spawn_edog_num=2,
                   spawn_erobot_num=1, spawn_drobot_num=1, traffic_density=0.2, max_actor_num=45),
    "med":    dict(object_density=0.6, spawn_human_num=50, spawn_wheelchairman_num=3, spawn_edog_num=4,
                   spawn_erobot_num=2, spawn_drobot_num=2, traffic_density=0.4, max_actor_num=100),
    "dense":  dict(object_density=0.9, spawn_human_num=85, spawn_wheelchairman_num=5, spawn_edog_num=6,
                   spawn_erobot_num=3, spawn_drobot_num=3, traffic_density=0.5, max_actor_num=170),
}[args.density]
try: MAP = int(args.map)            # int => random N-block PG road sequence (topology variety)
except ValueError: MAP = args.map   # 'X' etc.

CRUISE_Z = float(LOOP["cruise_z"]); SENSE_R = float(LOOP["sense_cull_r"])
REPLAN_DT = float(LOOP["replan_dt"])
CLASS_LABEL = {"pedestrian": [0], "vehicle": [1], "animal": [2]}


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
    crswalk_density=1, walk_on_all_regions=False,
    use_render=False, image_observation=False,            # headless, no camera -> fast
    interface_panel=[], manual_control=False, map=MAP, daytime="12:00",
    default_expert=False, drivable_area_extension=55, height_scale=1,
    show_mid_block_map=False, show_ego_navigation=False, debug=False, horizon=100000,
    on_continuous_line_done=False, out_of_route_done=False,
    vehicle_config=dict(show_lidar=False, show_navi_mark=False, show_line_to_navi_mark=False,
                        show_dest_mark=False, enable_reverse=True,
                        lidar=dict(num_lasers=0, distance=50)),
    agent_observation=DummyObservation,   # no per-step obs gather -> kills the 240-ray lidar loop (dominant eval cost)
    # NOTE: keep default physics substeps in the BENCHMARK for fidelity (decision_repeat change is render-only)
    show_sidewalk=True, show_crosswalk=True, random_spawn_lane_index=False,
    num_scenarios=200, accident_prob=0, relax_out_of_road_done=True, max_lateral_dist=1e3,
    crash_vehicle_done=False, crash_object_done=False, crash_human_done=False,
    **DENSITY)                                            # object_density / spawn_* / traffic_density / max_actor_num

print(f"[eval] building env (headless) ...", flush=True)
env = SidewalkDynamicMetaUrbanEnv(env_cfg)
eng = None; ego = None


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


def plan_route(ep):
    """random 2-point route through a local pedestrian cluster (mirrors render_3d_video.plan_route)."""
    import random
    rng = random.Random(args.seed0 * 1000 + ep)
    objs = native_objects()
    peds = [p for (_, c, p, _, _) in objs if c == "pedestrian"]
    movers = [p for (_, c, p, v, _) in objs if c == "pedestrian" and np.linalg.norm(v) > 0.2]
    pool = peds if len(peds) >= 3 else [p for (_, c, p, _, _) in objs if c in ("pedestrian", "vehicle")]
    if len(pool) < 2:
        e = np.asarray(ego.position[:2], float)
        return p3(e, CRUISE_Z), p3(e + np.array([max(args.route_len, 75.0), 0.0]), CRUISE_Z)
    anchor = np.asarray(rng.choice(movers if movers else pool), float)
    cluster = sorted((np.asarray(q, float) for q in pool), key=lambda q: float(np.linalg.norm(q - anchor)))[:6]
    C = np.array(cluster); ctr = C.mean(axis=0)
    d = (np.linalg.svd(C - ctr)[2][0] if len(C) >= 2 else np.array([1.0, 0.0]))
    if rng.random() < 0.5: d = -d
    d = d / (np.linalg.norm(d) + 1e-9)
    statics = [(np.asarray(p, float), np.asarray(s, float)) for (_, c, p, _, s) in objs if c == "static"]

    def _clear(pt):
        for sp, ssz in statics:
            if np.all(np.abs(pt[:2] - sp[:2]) < 0.5 * ssz[:2] + 0.8): return False
        return True

    L = args.route_len if args.route_len > 0 else rng.uniform(50.0, 120.0)   # per-episode route-length variety
    sxy = ctr - d * (L / 2); gxy = ctr + d * (L / 2)
    for _ in range(30):
        if _clear(sxy): break
        sxy = sxy + d * 2.0
    for _ in range(30):
        if _clear(gxy): break
        gxy = gxy - d * 2.0
    start = p3(sxy, CRUISE_Z)
    goal = p3(gxy, CRUISE_Z); goal[2] = float(PLN.get("default_goal_z", CRUISE_Z))
    return start, goal


par = Parameters()
for k, v in PLN.items():
    if hasattr(par, k): setattr(par, k, v)
par.replan_dt = REPLAN_DT
DT = float(par.dc)
GOAL_R = float(par.goal_radius)
_cache = {}


Z_CEIL = float(PLN.get("z_max", 6.0)) + 0.5
STATIC = dict(cloud=np.zeros((0, 3)), xy=np.zeros((0, 2)), objs=[], fed=[])   # rebuilt per scene reset


def _dt(sando, tid, size, pos, vel, label, t_sim, z=None):
    dd = _cache.get(tid)
    if dd is None:
        dd = DynTraj(); dd.id = int(tid); dd.mode = "Analytic"; _cache[tid] = dd
    zc = CRUISE_Z if z is None else float(z)
    x0, y0, vx, vy = float(pos[0]), float(pos[1]), float(vel[0]), float(vel[1])
    tx = f"{x0}+({vx})*(t-({t_sim}))"; ty = f"{y0}+({vy})*(t-({t_sim}))"; tz = f"{zc}"
    sig = (tx, ty, tz, tuple(label), float(size[0]), float(size[1]), float(size[2]))
    if getattr(dd, "_sig", None) != sig:        # recompile only on change (statics: once/episode)
        dd.bbox = np.asarray(size, float)
        dd.traj_x, dd.traj_y, dd.traj_z = tx, ty, tz
        dd.traj_vx, dd.traj_vy, dd.traj_vz = f"{vx}", f"{vy}", "0.0"; dd.label_set = list(label)
        dd.compile_analytic(); dd._sig = sig
    sando.add_traj(dd, t_sim)


def build_static_field():
    """Full-3D occupancy of every static object (footprint W×L over real HEIGHT, capped at Z_CEIL), so tall
    structure (trees/poles) is solid to the global planner — same fix as render_3d_video. Rebuilt per reset."""
    pts = []; objs = []; fed = []
    for oid, cls, pos, vel, size in native_objects():
        if cls != "static": continue
        w, l, h = float(size[0]), float(size[1]), float(size[2])
        objs.append((abs(hash(oid)) % 1000 + 200, np.asarray(pos, float), np.asarray(size, float)))
        fed.append(("static", p3(pos, h * 0.5), np.asarray(size, float)))
        ztop = min(h, Z_CEIL)
        nx = max(1, min(7, int(round(w / 0.45)))); ny = max(1, min(7, int(round(l / 0.45))))
        nz = max(1, min(16, int(round(ztop / 0.5))))
        for ix in range(nx + 1):
            fx = pos[0] + (ix / nx - 0.5) * w
            for iy in range(ny + 1):
                fy = pos[1] + (iy / ny - 0.5) * l
                for iz in range(nz + 1):
                    pts.append((fx, fy, 0.1 + (iz / nz) * ztop))
    cloud = np.asarray(pts, float) if pts else np.zeros((0, 3))
    STATIC["cloud"] = cloud; STATIC["xy"] = cloud[:, :2].copy() if len(cloud) else np.zeros((0, 2))
    STATIC["objs"] = objs; STATIC["fed"] = fed


def feed(sando, t_sim, p_drone):
    fed = []
    xy = STATIC["xy"]
    if len(xy):
        dxy = xy - p_drone[:2]; cloud = STATIC["cloud"][np.einsum('ij,ij->i', dxy, dxy) <= SENSE_R * SENSE_R]
    else:
        cloud = np.zeros((0, 3))
    for tid, pos, size in STATIC["objs"]:
        if np.linalg.norm(pos[:2] - p_drone[:2]) <= SENSE_R:
            _dt(sando, tid, size, pos, np.zeros(2), [], t_sim, z=min(float(size[2]), Z_CEIL) * 0.5)
    fed.extend((c, c3, sz) for (c, c3, sz) in STATIC["fed"] if np.linalg.norm(c3[:2] - p_drone[:2]) <= SENSE_R)
    for oid, cls, pos, vel, size in native_objects():
        if cls == "static": continue
        c3 = p3(pos, size[2] * 0.5)
        if np.linalg.norm(c3[:2] - p_drone[:2]) > SENSE_R: continue
        _dt(sando, abs(hash(oid)) % 1000, size, pos, vel, CLASS_LABEL[cls], t_sim)
        fed.append((cls, c3, size))
    sando.update_occupancy_map_ptr(cloud)
    return fed


def clearance(p, fed):
    r = float(par.drone_radius); gmin = np.inf; per = {}
    for cls, c3, size in fed:
        dd = p - c3; halfb = 0.5 * np.asarray(size, float)
        outside = np.maximum(np.abs(dd) - halfb, 0.0)
        sd = (np.linalg.norm(outside) if np.any(outside > 0) else -np.min(halfb - np.abs(dd))) - r
        gmin = min(gmin, sd); per[cls] = min(per.get(cls, np.inf), sd)
    return gmin, per


def run_episode(ep):
    global _cache
    _cache = {}
    start, goal = plan_route(ep)
    sando = SANDO(par)
    rs = RobotState(); rs.pos = start.copy(); rs.vel = np.zeros(3); sando.update_state(rs)
    sando.update_occupancy_map_ptr(np.zeros((0, 3)))
    g = RobotState(); g.pos = goal.copy(); sando.set_terminal_goal(g)
    # inject synthetic ANIMAL movers (label [2]) crossing the route, so the C++ animal-avoidance class
    # is actually exercised/scored (it never was in the headless benchmark). bbox only, no mesh needed.
    seg = goal[:2] - start[:2]; perp = np.array([-seg[1], seg[0]]); perp = perp / (np.linalg.norm(perp) + 1e-9)
    animals = []
    for i in range(int(args.animals)):
        frac = (i + 1) / (args.animals + 1)
        c = start[:2] + seg * frac; side = (-1) ** i
        animals.append(dict(p0=c + perp * 8.0 * side, vel=-perp * side * 1.2,
                            size=np.array([0.9, 2.6, 1.6]), id=900 + i))
    p_d = start.copy(); v_d = np.zeros(3); a_d = np.zeros(3)
    t = 0.0; last_rt = 0.0; mclr = np.inf; per_all = {}; max_rt = 0.0
    d0 = float(np.linalg.norm(goal[:2] - start[:2]))
    best_remaining = d0; stall_t = 0.0
    reached = collided = stuck = False
    collide_class = None; collide_t = None; n_collide = 0   # MODEL-collision record (which obstacle model, when)
    while t < args.t_max:
        st = RobotState(); st.pos = p_d.copy(); st.vel = v_d.copy(); st.accel = a_d.copy()
        sando.update_state(st)
        fed = feed(sando, t, p_d)
        for a in animals:                                   # feed crossing animals (label [2]) + score them
            apos = a["p0"] + a["vel"] * t; c3 = p3(apos, a["size"][2] * 0.5)
            if np.linalg.norm(c3[:2] - p_d[:2]) <= SENSE_R:
                _dt(sando, a["id"], a["size"], apos, a["vel"], [2], t)
                fed.append(("animal", c3, a["size"]))
        t0 = time.perf_counter(); sando.replan(last_rt, t); last_rt = time.perf_counter() - t0
        max_rt = max(max_rt, last_rt)
        for _ in range(int(round(REPLAN_DT / DT))):
            ok, ng = sando.get_next_goal()
            if ok:
                p_d = np.asarray(ng.pos, float); v_d = np.asarray(ng.vel, float); a_d = np.asarray(ng.accel, float)
            t += DT
        try: env.step([0.0, 0.0])
        except Exception: pass
        c, per = clearance(p_d, fed); mclr = min(mclr, c)
        for k, val in per.items(): per_all[k] = min(per_all.get(k, np.inf), val)
        if c < 0:                                          # drone body penetrated an obstacle MODEL bbox
            collided = True; n_collide += 1
            hit = min(per.items(), key=lambda kv: kv[1])[0]   # which class/model we hit
            if collide_t is None: collide_t = round(t, 2); collide_class = hit
        remaining = float(np.linalg.norm(goal[:2] - p_d[:2]))
        if remaining < best_remaining - 0.3:
            best_remaining = remaining; stall_t = 0.0
        else:
            stall_t += REPLAN_DT
        if sando.get_drone_status() == GOAL_REACHED and np.linalg.norm(p_d - goal) < GOAL_R:
            reached = True; break
        if stall_t > 5.0 and remaining > GOAL_R + 1.0:    # no progress for 5s and not at goal => stuck
            stuck = True; break
    timeout = (not reached) and (not collided) and (not stuck)
    return dict(ep=ep, reached=bool(reached), collided=bool(collided), stuck=bool(stuck), timeout=bool(timeout),
                t=round(t, 1), min_clr=round(float(mclr), 3), remaining=round(best_remaining, 1),
                max_replan_ms=round(max_rt * 1000, 1),
                collide_class=collide_class, collide_t=collide_t, n_collide=int(n_collide),
                per={k: round(float(v), 2) for k, v in per_all.items()})


def _near_static(pt):
    """signed clearance (m) from pt to the NEAREST solid static footprint (neg = inside a box)."""
    best = np.inf
    for _tid, sp, ssz in STATIC["objs"]:
        gap = np.abs(pt[:2] - sp[:2]) - 0.5 * ssz[:2]
        d = float(np.linalg.norm(np.maximum(gap, 0.0))) if np.any(gap > 0) else -float(np.max(-gap))
        best = min(best, d)
    return best if np.isfinite(best) else 99.9


def diag_episode(ep):
    """Why does the drone never launch? Print the global heat-A* path length + actual motion."""
    start, goal = plan_route(ep)
    sando = SANDO(par)
    rs = RobotState(); rs.pos = start.copy(); rs.vel = np.zeros(3); sando.update_state(rs)
    sando.update_occupancy_map_ptr(np.zeros((0, 3)))
    g = RobotState(); g.pos = goal.copy(); sando.set_terminal_goal(g)
    p_d = start.copy(); v_d = np.zeros(3); a_d = np.zeros(3); gp_len = 0; status = -1
    for it in range(5):
        st = RobotState(); st.pos = p_d.copy(); st.vel = v_d.copy(); st.accel = a_d.copy(); sando.update_state(st)
        feed(sando, it * REPLAN_DT, p_d)
        sando.replan(0.0, it * REPLAN_DT)
        try: gp_len = len(sando.get_global_path())
        except Exception: gp_len = -1
        for _ in range(int(round(REPLAN_DT / DT))):
            ok, ng = sando.get_next_goal()
            if ok: p_d = np.asarray(ng.pos, float); v_d = np.asarray(ng.vel, float); a_d = np.asarray(ng.accel, float)
        status = sando.get_drone_status()
    moved = float(np.linalg.norm(p_d[:2] - start[:2]))
    route = float(np.linalg.norm(goal[:2] - start[:2]))
    flag = "NOLAUNCH" if moved < 0.5 else "ok"
    print(f"[diag] ep{ep:2d} {flag:8s} route={route:6.1f}m start_clr={_near_static(start):5.1f} "
          f"goal_clr={_near_static(goal):5.1f} gp_len={gp_len:4d} moved5={moved:5.2f}m status={status}", flush=True)


results = []
t_start = time.perf_counter()
for ep in range(args.n):
    if ep % args.reset_every == 0:
        env.reset(seed=(args.seed0 + ep) % int(env_cfg["num_scenarios"]))   # seed must be < num_scenarios
        eng = env.engine; ego = env.agent
        for _ in range(8): env.step([0.0, 0.0])
        build_static_field()                       # precompute full-3D static occupancy for this scene
    if args.diag:
        diag_episode(ep); continue
    r = run_episode(ep)
    results.append(r)
    tag = ("REACHED" if r["reached"] else "COLLIDED" if r["collided"] else "STUCK" if r["stuck"] else "TIMEOUT")
    print(f"[eval] ep {ep:3d}  {tag:8s} t={r['t']:4.1f} minClr={r['min_clr']:+.2f} "
          f"rem={r['remaining']:5.1f} replan_max={r['max_replan_ms']:.0f}ms per={r['per']}", flush=True)

n = len(results)
def rate(key): return sum(1 for r in results if r[key]) / max(n, 1)
collide_by_class = {}
for r in results:
    if r["collided"] and r["collide_class"]:
        collide_by_class[r["collide_class"]] = collide_by_class.get(r["collide_class"], 0) + 1
summary = dict(n=n, reached=rate("reached"), collided=rate("collided"), stuck=rate("stuck"), timeout=rate("timeout"),
               wall_s=round(time.perf_counter() - t_start, 1),
               collide_by_class=collide_by_class,
               stuck_eps=[r["ep"] for r in results if r["stuck"]],
               collided_eps=[{"ep": r["ep"], "hit": r["collide_class"], "t": r["collide_t"], "min_clr": r["min_clr"]}
                             for r in results if r["collided"]],
               timeout_eps=[r["ep"] for r in results if r["timeout"]])
os.makedirs(os.path.join(_HERE, "out"), exist_ok=True)
outp = args.out if os.path.isabs(args.out) else os.path.join(_HERE, args.out)
with open(outp, "w") as f: json.dump(dict(summary=summary, results=results), f, indent=2)
print("\n[eval] ===== SUMMARY =====", flush=True)
print(f"[eval] n={n}  reached={summary['reached']*100:.0f}%  collided={summary['collided']*100:.0f}%  "
      f"stuck={summary['stuck']*100:.0f}%  timeout={summary['timeout']*100:.0f}%  wall={summary['wall_s']}s", flush=True)
print(f"[eval] collisions by model class: {summary['collide_by_class']}", flush=True)
print(f"[eval] stuck eps: {summary['stuck_eps']}", flush=True)
print(f"[eval] collided eps: {summary['collided_eps']}", flush=True)
print(f"[eval] wrote {outp}", flush=True)
env.close()
