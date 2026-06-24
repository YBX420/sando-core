"""conformal_harvest — log REAL MetaUrban mover trajectories (headless) for conformal calibration.

WHY: the synthetic scenario.py movers walk at CONSTANT velocity, so a CA-Kalman filter predicts them
almost perfectly -> the calibrated keep-out would be artificially tiny and indefensible. Real MetaUrban
pedestrians/vehicles run an ORCA crowd policy: they turn, brake, swerve around each other -> the KF has
GENUINE prediction error. We harvest those real trajectories once (heavy sim) and reuse them offline for
(a) conformal calibration of (q_conformal, v_eff), (b) the ours-vs-native A/B replay, (c) the
continuous-time-vs-discrete-sampling ablation. One asset, three consumers, fully reproducible.

Output per seed: out/conformal/traj_seed{N}.npz
  movers: object array of dict(cls, r, h, t[T], xy[T,2])   # t in seconds, world-frame xy, DT_LOG cadence
  meta:   dict(seed, dt_log, n_steps)

Run (metaurban conda env):
  PYTHONPATH=/media/boxuan/Data2/projects/metaurban python metaurban/conformal_harvest.py --seeds 0-19 --steps 600
"""
import os, sys, argparse, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUTDIR = os.path.join(os.path.dirname(HERE), "out", "conformal")

DT_LOG = 0.1   # env.step cadence: decision_repeat=2 * physics_world_step_size=0.05 = 0.1 s


def _classify(o, Ped, Veh, Robo, Dog):
    if isinstance(o, Ped):
        return "pedestrian"
    if isinstance(o, (Veh, Robo, Dog)):
        return "vehicle"
    return "static"


def _size(o):
    if all(hasattr(o, a) for a in ("WIDTH", "LENGTH", "HEIGHT")):
        try:
            return float(o.WIDTH), float(o.LENGTH), float(o.HEIGHT)
        except Exception:
            pass
    w = float(getattr(o, "top_down_width", 0.6) or 0.6)
    l = float(getattr(o, "top_down_length", 0.6) or 0.6)
    return w, l, 1.6


def make_env():
    from metaurban import SidewalkDynamicMetaUrbanEnv
    cfg = dict(
        crswalk_density=1, object_density=0.9, walk_on_all_regions=False,
        use_render=False, image_observation=False,          # <- no camera -> truly headless, no GL readback
        interface_panel=[], manual_control=False, map='X', daytime="12:00",
        default_expert=False, drivable_area_extension=55, height_scale=1,
        show_mid_block_map=False, show_ego_navigation=False, debug=False, horizon=100000,
        on_continuous_line_done=False, out_of_route_done=False,
        vehicle_config=dict(show_lidar=False, show_navi_mark=False, show_line_to_navi_mark=False,
                            show_dest_mark=False, enable_reverse=True, lidar=dict(num_lasers=0, distance=50)),
        multi_thread_render=False,
        decision_repeat=2, physics_world_step_size=0.05,
        show_sidewalk=True, show_crosswalk=True, random_spawn_lane_index=False,
        num_scenarios=20, accident_prob=0, relax_out_of_road_done=True, max_lateral_dist=1e3,
        crash_vehicle_done=False, crash_object_done=False, crash_human_done=False, traffic_density=0.5,
        spawn_human_num=85, spawn_wheelchairman_num=5, spawn_edog_num=6, spawn_erobot_num=3,
        spawn_drobot_num=3, max_actor_num=170)
    return SidewalkDynamicMetaUrbanEnv(cfg)


def harvest_seed(env, classes, seed, n_steps):
    from metaurban import SidewalkDynamicMetaUrbanEnv  # noqa (env already built)
    Ped, Veh, Robo, Dog = classes
    env.reset(seed=seed % 20)
    for _ in range(8):
        env.step([0.0, 0.0])
    eng = env.engine
    ego = env.agent
    # per-oid running track: list of (step, x, y), plus static meta (cls, r, h)
    tracks = {}
    meta = {}
    for step in range(n_steps):
        env.step([0.0, 0.0])
        for oid, o in eng.get_objects().items():
            if o is ego:
                continue
            try:
                pos = np.asarray(o.position, float)
            except Exception:
                continue
            if pos.shape[0] < 2 or not np.all(np.isfinite(pos[:2])):
                continue
            cls = _classify(o, Ped, Veh, Robo, Dog)
            if cls == "static":
                continue
            if oid not in meta:
                w, l, h = _size(o)
                meta[oid] = (cls, 0.5 * float(np.hypot(w, l)) * 0.5 + 0.3 * (cls == "pedestrian"), h)
            tracks.setdefault(oid, []).append((step, float(pos[0]), float(pos[1])))
    # assemble movers that (a) live long enough and (b) actually move at some point
    movers = []
    for oid, seq in tracks.items():
        if len(seq) < 12:                       # need >= ~1.2 s of track to predict/score
            continue
        arr = np.asarray(seq, float)
        t = arr[:, 0] * DT_LOG
        xy = arr[:, 1:3]
        speed = np.linalg.norm(np.diff(xy, axis=0), axis=1) / DT_LOG
        if float(np.max(speed)) < 0.3:          # skip permanently-stationary furniture-like agents
            continue
        cls, r, h = meta[oid]
        movers.append(dict(cls=cls, r=float(r), h=float(h), t=t, xy=xy))
    return movers


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="0-9", help="e.g. 0-19 or 3,5,7")
    ap.add_argument("--steps", type=int, default=600, help="env steps per seed (0.1s each -> 60s @600)")
    args = ap.parse_args()
    if "-" in args.seeds and "," not in args.seeds:
        a, b = args.seeds.split("-"); seeds = list(range(int(a), int(b) + 1))
    else:
        seeds = [int(s) for s in args.seeds.split(",")]
    os.makedirs(OUTDIR, exist_ok=True)

    print(f"[harvest] building headless env ...", flush=True)
    t0 = time.time()
    env = make_env()
    from metaurban.component.agents.pedestrian.base_pedestrian import BasePedestrian as Ped
    from metaurban.component.delivery_robot.base_deliveryrobot import BaseDeliveryRobot as Robo
    from metaurban.component.robotdog.base_robotdog import BaseRobotDog as Dog
    from metaurban.component.vehicle.base_vehicle import BaseVehicle as Veh
    classes = (Ped, Veh, Robo, Dog)
    print(f"[harvest] env up in {time.time()-t0:.1f}s; harvesting seeds {seeds} x {args.steps} steps", flush=True)

    for sd in seeds:
        t1 = time.time()
        movers = harvest_seed(env, classes, sd, args.steps)
        nmov = len(movers)
        tot = sum(len(m["t"]) for m in movers)
        path = os.path.join(OUTDIR, f"traj_seed{sd}.npz")
        np.savez_compressed(path, movers=np.array(movers, dtype=object),
                            meta=dict(seed=sd, dt_log=DT_LOG, n_steps=args.steps))
        print(f"[harvest] seed {sd}: {nmov} movers, {tot} samples -> {path}  ({time.time()-t1:.1f}s)", flush=True)
    env.close()
    print("[harvest] done.", flush=True)


if __name__ == "__main__":
    main()
