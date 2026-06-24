"""px4_batch — BATCH validation through real PX4 SITL: ours / native EGO / native SANDO, many episodes.

PX4 is real-time (~tens of s/episode) so this is built to run UNATTENDED + RESILIENT:
  * ONE bridge is reused across episodes (connect/arm once, not per episode).
  * before each episode the drone is flown to the episode start; the world origin is re-anchored there.
  * if PX4 dies (pose stops updating) we restart it via px4_sitl.sh, rebuild the bridge, and RETRY the episode.
  * every episode result is appended to out/conformal/px4_batch.jsonl immediately, so a crash never loses data.
Aggregates per mode at the end. Same harvested trajectories + run_replay loop as the headless A/B, so the only
difference is the flight stack (real PX4 vs point-mass/quadrotor) -> an apples-to-apples sim2real comparison.

Run (metaurban env, with both preloads):
  bash metaurban/px4_sitl.sh restart && sleep 55
  LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6 \
  LD_LIBRARY_PATH=/home/boxuan/gurobi1103/linux64/lib \
  python metaurban/px4_batch.py --modes ours,native,sando --seeds 0-4 --n_ep 3 --speedup 1.33
"""
import os, sys, math, time, json, glob, argparse, subprocess
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import replay_core as R
from px4_bridge import PX4Bridge

SITL = os.path.join(HERE, "px4_sitl.sh")
JSONL = os.path.join(R.OUTDIR, "px4_batch.jsonl")


def px4_alive():
    try:
        out = subprocess.run(["pgrep", "-ax", "px4"], capture_output=True, text=True).stdout
        return any("bin/px4" in ln for ln in out.splitlines())
    except Exception:
        return False


def restart_px4():
    print("[batch] restarting PX4 SITL ...", flush=True)
    subprocess.run(["bash", SITL, "restart"], timeout=180)
    # EKF settle
    for _ in range(55):
        if not px4_alive():
            time.sleep(1); continue
        time.sleep(1)
    print(f"[batch] PX4 restarted; alive={px4_alive()}", flush=True)


def make_bridge():
    px4 = PX4Bridge(takeoff_alt=R.CRUISE_Z)
    if not px4.wait_ready(120):
        return None
    return px4


def parse_seeds(s):
    if "-" in s and "," not in s:
        a, b = s.split("-"); return list(range(int(a), int(b) + 1))
    return [int(x) for x in s.split(",")]


def fly_to(px4, target_local, hold=12):
    """Stream a position set-point until the drone settles near it (or hold ticks elapse)."""
    for _ in range(hold):
        px4.set_setpoint(target_local, 0.0); time.sleep(0.15)
        w, _y, _v, ok = px4.get_pose_world()
        if ok and np.linalg.norm(np.asarray(w)[:2] - target_local[:2]) < 0.6:
            break


def run_one(px4, movers, ep, mode, calib, speedup):
    """Fly one episode through PX4; returns a result dict (or None if the drone never actually moved)."""
    org = 0.5 * (ep["start"][:2] + ep["goal"][:2])
    start_local = ep["start"].copy(); start_local[:2] -= org
    px4.set_world_origin(start_local)
    fly_to(px4, start_local)
    w0, _, _, _ = px4.get_pose_world()
    # motion sanity: command +2.5 m, confirm the drone moves (else this run is a fake stall)
    probe = start_local.copy(); probe[0] += 2.5
    for _ in range(20):
        px4.set_setpoint(probe, 0.0); time.sleep(0.12)
    w1, _, _, _ = px4.get_pose_world()
    moved = float(np.linalg.norm(np.asarray(w1)[:2] - np.asarray(w0)[:2]))
    fly_to(px4, start_local)
    if moved < 1.0:
        return None    # not flying -> invalid; caller restarts PX4

    def flier(p_ref, v_ref, a_ref, dt):
        yaw = math.atan2(v_ref[1], v_ref[0]) if float(np.linalg.norm(v_ref[:2])) > 0.3 else 0.0
        px4.set_setpoint(np.asarray(p_ref, float), yaw)
        time.sleep(dt)
        w, _y, wv, _ok = px4.get_pose_world()
        return np.asarray(w, float), np.asarray(wv, float)

    mv = 3.0 * (speedup if mode == "ours" else 1.0)
    r = R.run_replay(movers, ep, mode, calib, max_vel=mv, flier=flier)
    return dict(reached=r["reached"], time_s=r["time_s"], min_clr=r["min_clr"],
                collided=r["collided"], trackerr=r["track_err_med"], moved_probe=round(moved, 2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--modes", default="ours,native,sando")
    ap.add_argument("--seeds", default="0-4")
    ap.add_argument("--n_ep", type=int, default=3)
    ap.add_argument("--speedup", type=float, default=1.33)
    ap.add_argument("--eps", type=float, default=0.05)
    args = ap.parse_args()
    modes = args.modes.split(","); seeds = parse_seeds(args.seeds)
    calib = R.load_calib(args.eps)

    if not px4_alive():
        restart_px4()
    px4 = make_bridge()
    if px4 is None:
        print("[batch] PX4 bridge not ready after restart; aborting"); return 1
    print(f"[batch] PX4 bridge ready. modes={modes} seeds={seeds} n_ep={args.n_ep}", flush=True)

    rows = []
    fout = open(JSONL, "a")
    for mode in modes:
        for sd in seeds:
            path = os.path.join(R.OUTDIR, f"traj_seed{sd}.npz")
            if not os.path.exists(path):
                continue
            movers = R.Movers(list(np.load(path, allow_pickle=True)["movers"]))
            for k, ep in enumerate(R.build_episodes(movers, sd, n_ep=args.n_ep)):
                res = None
                for attempt in range(2):
                    if not px4_alive():
                        restart_px4(); px4 = make_bridge()
                        if px4 is None:
                            break
                    try:
                        res = run_one(px4, movers, ep, mode, calib, args.speedup)
                    except Exception as e:
                        print(f"[batch] {mode} seed{sd} ep{k} error: {e}", flush=True); res = None
                    if res is not None:
                        break
                    print(f"[batch] {mode} seed{sd} ep{k}: not flying / failed -> restart + retry", flush=True)
                    restart_px4(); px4 = make_bridge()
                    if px4 is None:
                        break
                row = dict(mode=mode, seed=sd, ep=k, **(res or dict(reached=False, invalid=True)))
                rows.append(row); fout.write(json.dumps(row) + "\n"); fout.flush()
                print(f"[batch] {mode} seed{sd} ep{k}: reached={row.get('reached')} "
                      f"t={row.get('time_s')} clr={row.get('min_clr')} coll={row.get('collided')}", flush=True)
    fout.close()

    print("\n=== PX4 BATCH SUMMARY ===")
    for mode in modes:
        mr = [r for r in rows if r["mode"] == mode and not r.get("invalid")]
        if not mr:
            print(f"  {mode}: no valid runs"); continue
        reached = [r for r in mr if r["reached"]]
        coll = sum(1 for r in mr if r.get("collided"))
        clrs = [r["min_clr"] for r in mr if r.get("min_clr") is not None]
        ts = [r["time_s"] for r in reached]
        print(f"  {mode:>7}: valid={len(mr)} reached={len(reached)} collisions={coll} "
              f"min_clr={min(clrs):.2f} median_clr={np.median(clrs):.2f} "
              f"median_t={np.median(ts):.1f}s" if clrs else f"  {mode}: no clearance data")
    json.dump(rows, open(os.path.join(R.OUTDIR, "px4_batch_summary.json"), "w"), indent=2)
    print(f"[batch] wrote {JSONL} + px4_batch_summary.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
