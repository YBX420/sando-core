"""px4_diag — per-tick trace of ONE episode flown through real PX4, to see WHY ours under-performs on PX4.

Prints, each control tick: dgoal (distance to goal), |v| (flown speed), the maneuver kind, clearance, and the
plan->flown gap |flown - p_ref|. Compares the PX4 run to the quadrotor stand-in on the SAME episode so the
divergence (where PX4 lags / stalls / creeps) is visible. This is the diagnostic before any PX4 fix.

  LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6 python metaurban/px4_diag.py --seed 1 --ep 0
"""
import os, sys, math, time, argparse, contextlib
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import replay_core as R
from px4_bridge import PX4Bridge


@contextlib.contextmanager
def _quiet():
    fd = os.dup(1); dn = os.open(os.devnull, os.O_WRONLY); os.dup2(dn, 1); os.close(dn)
    try: yield
    finally: os.dup2(fd, 1); os.close(fd)


def trace(hist, label):
    print(f"\n--- {label}: {len(hist)} ticks ---")
    print(f"{'tick':>4}{'dgoal':>7}{'|v|':>6}{'kind':>10}{'clr':>7}{'plan-flown':>11}")
    for h in hist:
        gap = math.hypot(h["p"][0] - h["pref"][0], h["p"][1] - h["pref"][1])
        clr = h["clr"] if h["clr"] is not None else float("nan")
        print(f"{h['tick']:>4}{h['dgoal']:>7.2f}{h['v']:>6.2f}{str(h['kind']):>10}{clr:>7.2f}{gap:>11.2f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--ep", type=int, default=0)
    ap.add_argument("--speedup", type=float, default=1.33)
    args = ap.parse_args()
    movers = R.Movers(list(np.load(os.path.join(R.OUTDIR, f"traj_seed{args.seed}.npz"), allow_pickle=True)["movers"]))
    ep = R.build_episodes(movers, args.seed, n_ep=args.ep + 1)[args.ep]
    calib = R.load_calib(0.05)
    mv = 3.0 * args.speedup

    print(f"[diag] seed{args.seed} ep{args.ep}: start={np.round(ep['start'][:2],1)} goal={np.round(ep['goal'][:2],1)} "
          f"members={len(ep['members'])} max_vel={mv:.2f}", flush=True)

    # 1) quadrotor stand-in (fast, the reference)
    with _quiet():
        ro_dyn = R.run_replay(movers, ep, "ours", calib, max_vel=mv, dynamics=True, record=True)
    trace(ro_dyn["history"], f"QUADROTOR stand-in  reached={ro_dyn['reached']} t={ro_dyn['time_s']:.1f} clr={ro_dyn['min_clr']}")

    # 2) real PX4
    print("\n[diag] bringing up PX4 ...", flush=True)
    px4 = PX4Bridge(takeoff_alt=R.CRUISE_Z)
    if not px4.wait_ready(90):
        print("[diag] PX4 not ready"); return 1
    org = 0.5 * (ep["start"][:2] + ep["goal"][:2])
    start_local = ep["start"].copy(); start_local[:2] -= org
    px4.set_world_origin(start_local)
    for _ in range(10):
        px4.set_setpoint(start_local, 0.0); time.sleep(0.1)
    w0, _, _, _ = px4.get_pose_world()
    print(f"[diag] PX4 settled at {np.round(w0,2)} (target start_local {np.round(start_local,2)})", flush=True)

    # PRE-FLIGHT MOTION CHECK: command +3 m forward, verify the drone actually MOVES there (offboard truly engaged
    # + armed). If it doesn't move, the harness/arm/offboard is broken and any "stall" result would be a fake.
    probe = start_local.copy(); probe[0] += 3.0
    for _ in range(30):
        px4.set_setpoint(probe, 0.0); time.sleep(0.1)
    w1, _, _, _ = px4.get_pose_world()
    moved = float(np.linalg.norm(w1[:2] - w0[:2]))
    print(f"[diag] motion check: commanded +3m, drone moved {moved:.2f} m -> {np.round(w1,2)}  "
          f"{'OK (flying)' if moved > 1.5 else 'FAIL (NOT flying -> arm/offboard broken, result invalid)'}", flush=True)
    # fly back to start before the episode
    for _ in range(15):
        px4.set_setpoint(start_local, 0.0); time.sleep(0.1)

    lat = {"t": time.perf_counter()}
    def flier(p_ref, v_ref, a_ref, dt):
        yaw = math.atan2(v_ref[1], v_ref[0]) if float(np.linalg.norm(v_ref[:2])) > 0.3 else 0.0
        px4.set_setpoint(np.asarray(p_ref, float), yaw)
        time.sleep(dt)
        w, _yaw, wv, ok = px4.get_pose_world()
        return np.asarray(w, float), np.asarray(wv, float)

    with _quiet():
        ro_px4 = R.run_replay(movers, ep, "ours", calib, max_vel=mv, flier=flier, record=True)
    trace(ro_px4["history"], f"REAL PX4  reached={ro_px4['reached']} t={ro_px4['time_s']:.1f} clr={ro_px4['min_clr']} "
                             f"trackerr_med={ro_px4['track_err_med']:.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
