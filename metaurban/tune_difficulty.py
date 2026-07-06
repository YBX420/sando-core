"""tune_difficulty — closed-loop difficulty calibration: make encounters actually HAPPEN.

Open-loop retiming (populate's contested) guesses the drone reaches a crossing at frac*L/v_nom;
the real flight deviates by seconds, and at 1.3 m/s a 2 s miss = 2.6 m of clearance -- which is why
even 42-mover packed scenes showed identical fat clearances across arms. This closes the loop:

  1. fly the scenario once (ours, PERCEPT=gt, 1 seed, record=True);
  2. for every mover whose path crosses the corridor: find the tick the REAL drone is nearest to
     the crossing point; shift that mover's spawn_t by the measured timing error;
  3. verify spacing floors after every shift (revert offenders);
  4. iterate until min_clr(ours,gt) <= target or budget exhausted.

The tuned scenario stays fully legal (spacing/citys rules untouched) -- only WHEN people set off
changes, which is exactly the dial a benchmark designer owns.

Run:  python3 tune_difficulty.py scenarios/bench_hard/street_packed_s0.json --target 1.0
"""
import argparse
import json

import numpy as np

import replay_core as RC
import scenario_lib as SLB
from populate import min_pair_dist, _seg_cross


def fly_once(scn, mode="native"):
    movers = SLB.apply_rh_overrides(RC.Movers(SLB.to_movers_raw(scn)), scn)
    ep = SLB.to_episode(scn)
    import os
    os.environ["PERCEPT"] = "gt"
    res = RC.run_replay(movers, ep, mode=mode, record=True,
                        max_vel=float(scn["drone"].get("max_vel", 3.0)))
    os.environ.pop("PERCEPT", None)
    org = 0.5 * (np.asarray(ep["start"][:2]) + np.asarray(ep["goal"][:2]))
    traj = [(h["t"], np.asarray(h["p"][:2]) + org) for h in (res.get("history") or [])]
    return res, traj


def retime(scn, traj):
    st = np.asarray(scn["drone"]["start"][:2], float)
    gl = np.asarray(scn["drone"]["goal"][:2], float)
    shifts = 0
    for m in scn["movers"]:
        path = np.asarray(m["path"], float)
        if len(path) < 2:
            continue
        hit = None
        for k in range(len(path) - 1):
            tt = _seg_cross(st, gl, path[k], path[k + 1])
            if tt is not None:
                hit = (st + tt * (gl - st), k)
                break
        if hit is None:
            continue
        cross_pt, k = hit
        # measured: when is the real drone nearest to this crossing point?
        t_drone = min(traj, key=lambda p: np.linalg.norm(p[1] - cross_pt))[0]
        # mover's arrival at the crossing under its own kinematics
        seg = np.linalg.norm(np.diff(path[:k + 1], axis=0), axis=1).sum() if k > 0 else 0.0
        dist = seg + float(np.linalg.norm(cross_pt - path[k]))
        sp = m["speed"]
        spd = float(sp.get("v1", 5.0)) if isinstance(sp, dict) else float(sp)
        if spd < 0.2:
            continue
        t_mover = float(m.get("spawn_t", 0.0)) + dist / spd
        err = t_drone - t_mover
        if abs(err) < 0.3:
            continue
        old = m["spawn_t"]
        m["spawn_t"] = round(max(0.0, float(m.get("spawn_t", 0.0)) + err), 1)
        w = min_pair_dist(scn)
        if w is not None and w[2] < w[3]:
            m["spawn_t"] = old
        else:
            shifts += 1
    return shifts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario")
    ap.add_argument("--target", type=float, default=1.0, help="stop once min_clr(ours,gt) <= this")
    ap.add_argument("--iters", type=int, default=4)
    ap.add_argument("--arm", default="native", choices=["native", "ours"],
                    help="tune against the WEAKEST defender (native): it defines when encounters bind")
    args = ap.parse_args()
    scn = json.load(open(args.scenario))
    for it in range(args.iters):
        resolved = SLB.load(scn)
        res, traj = fly_once(resolved, mode=args.arm)
        clr = res["min_clr"]
        print(f"[tune] iter{it}: min_clr={clr:.2f} collided={res['collided']}")
        if clr <= args.target or not traj:
            break
        n = retime(scn, traj)
        print(f"[tune]   retimed {n} movers to the MEASURED drone schedule")
        if n == 0:
            break
    w = min_pair_dist(SLB.load(scn))
    assert w is None or w[2] >= w[3] - 1e-6, "spacing breached after tuning?!"
    json.dump(scn, open(args.scenario, "w"), indent=1)
    print(f"[tune] saved {args.scenario} (spacing gate still holds)")


if __name__ == "__main__":
    main()
