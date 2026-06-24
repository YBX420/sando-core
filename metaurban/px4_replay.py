"""px4_replay — spot-check the replay against a REAL PX4 SITL flight stack (the gold-standard dynamics).

replay_core's --dynamics flies set-points through quadrotor.py, the documented LOCAL STAND-IN for PX4. This
harness instead streams the SAME set-points to a live PX4 SITL via MAVSDK offboard (px4_bridge) and measures
clearance on PX4's fused flown pose -- so we can confirm the stand-in isn't optimistic vs the real controller.

Real-time + lockstep, so it only does a few episodes (a spot-check, not the 120-ep A/B). Runs ours@speedup.

Prereq: PX4 SITL up (px4 binary + jmavsim, MAVSDK on udp 14540). Run in the metaurban env WITH
  LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6   (so ego_capi.so and mavsdk coexist).

  LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6 \
  python metaurban/px4_replay.py --seed 1 --n_ep 3 --speedup 1.33
"""
import os, sys, math, time, json, glob, argparse
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import replay_core as R
from px4_bridge import PX4Bridge


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--n_ep", type=int, default=3)
    ap.add_argument("--speedup", type=float, default=1.33)
    ap.add_argument("--eps", type=float, default=0.05)
    args = ap.parse_args()

    path = os.path.join(R.OUTDIR, f"traj_seed{args.seed}.npz")
    dat = np.load(path, allow_pickle=True)
    movers = R.Movers(list(dat["movers"]))
    episodes = R.build_episodes(movers, args.seed, n_ep=args.n_ep)
    calib = R.load_calib(args.eps)
    print(f"[px4r] seed {args.seed}: {len(episodes)} episodes; bringing up PX4 offboard ...", flush=True)

    px4 = PX4Bridge(takeoff_alt=R.CRUISE_Z)
    if not px4.wait_ready(90):
        print("[px4r] PX4 offboard NOT ready (is SITL up on udp 14540?)"); return 1
    print("[px4r] PX4 offboard ready.", flush=True)

    rows = []
    for k, ep in enumerate(episodes):
        # local frame origin = corridor midpoint (same as run_replay); anchor PX4 there at the episode start
        org = 0.5 * (ep["start"][:2] + ep["goal"][:2])
        start_local = ep["start"].copy(); start_local[:2] -= org
        px4.set_world_origin(start_local)
        # let PX4 settle at the start set-point before launching the run
        for _ in range(8):
            px4.set_setpoint(start_local, 0.0); time.sleep(0.1)

        def flier(p_ref, v_ref, a_ref, dt):
            yaw = math.atan2(v_ref[1], v_ref[0]) if float(np.linalg.norm(v_ref[:2])) > 0.3 else 0.0
            px4.set_setpoint(np.asarray(p_ref, float), yaw)
            time.sleep(dt)                                  # real-time; PX4 lockstep advances the dynamics
            w, _yaw, wv, ok = px4.get_pose_world()
            return np.asarray(w, float), np.asarray(wv, float)

        ro_px4 = R.run_replay(movers, ep, "ours", calib, max_vel=3.0 * args.speedup, flier=flier)
        ro_dyn = R.run_replay(movers, ep, "ours", calib, max_vel=3.0 * args.speedup, dynamics=True)
        ro_pt = R.run_replay(movers, ep, "ours", calib, max_vel=3.0 * args.speedup)
        rows.append(dict(seed=args.seed, ep=k,
                         px4_clr=ro_px4["min_clr"], px4_t=ro_px4["time_s"], px4_coll=ro_px4["collided"],
                         px4_trackerr=ro_px4["track_err_med"],
                         dyn_clr=ro_dyn["min_clr"], dyn_t=ro_dyn["time_s"], dyn_coll=ro_dyn["collided"],
                         pt_clr=ro_pt["min_clr"], pt_t=ro_pt["time_s"]))
        r = rows[-1]
        print(f"[px4r] ep{k}: PX4 clr={_f(r['px4_clr'])} t={r['px4_t']:.1f} coll={int(r['px4_coll'])} "
              f"trackerr={r['px4_trackerr']:.2f} | quad-stand-in clr={_f(r['dyn_clr'])} t={r['dyn_t']:.1f} | "
              f"point-mass clr={_f(r['pt_clr'])} t={r['pt_t']:.1f}", flush=True)

    # summary
    pc = [r["px4_clr"] for r in rows if r["px4_clr"] is not None]
    dc = [r["dyn_clr"] for r in rows if r["dyn_clr"] is not None]
    ptc = [r["pt_clr"] for r in rows if r["pt_clr"] is not None]
    print("\n=== PX4 spot-check summary ===")
    print(f"  min clearance:  PX4={min(pc):.3f}  quad-stand-in={min(dc):.3f}  point-mass={min(ptc):.3f}")
    print(f"  ours collisions on PX4: {sum(r['px4_coll'] for r in rows)}/{len(rows)}")
    print(f"  median plan->flown tracking error (PX4): {np.median([r['px4_trackerr'] for r in rows]):.3f} m")
    with open(os.path.join(R.OUTDIR, "px4_spotcheck.json"), "w") as f:
        json.dump(rows, f, indent=2)
    print(f"[px4r] wrote {os.path.join(R.OUTDIR, 'px4_spotcheck.json')}")
    return 0


def _f(x):
    return "None" if x is None else f"{x:.2f}"


if __name__ == "__main__":
    sys.exit(main())
