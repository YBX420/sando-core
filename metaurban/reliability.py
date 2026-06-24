"""reliability — large-scale headless no-collision rate for ours. Can we approach 99.999% collision-free?

Runs ours over MANY distinct corridors (all seeds x many episodes each) at a chosen conformal eps, counts
collisions + reaches. Reports the empirical collision rate and a distribution-free upper bound (rule of three:
0 collisions in N -> 95%-CI collision rate <= 3/N). 99.999% (1e-5) needs ~3e5 runs to verify DIRECTLY; this
reports the trend + the conformal guarantee backing it. Tighter eps -> wider keep-out -> lower collision rate.

Run:  python metaurban/reliability.py --eps 0.01 --seeds 0-19 --n_ep 60
"""
import os, sys, glob, json, argparse, contextlib
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import replay_core as R


@contextlib.contextmanager
def _quiet():
    fd = os.dup(1); dn = os.open(os.devnull, os.O_WRONLY); os.dup2(dn, 1); os.close(dn)
    try:
        yield
    finally:
        os.dup2(fd, 1); os.close(fd)


def parse_seeds(s):
    if "-" in s and "," not in s:
        a, b = s.split("-"); return list(range(int(a), int(b) + 1))
    return [int(x) for x in s.split(",")]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--eps", type=float, default=0.01)
    ap.add_argument("--seeds", default="0-19")
    ap.add_argument("--n_ep", type=int, default=60)
    ap.add_argument("--speedup", type=float, default=1.33)
    ap.add_argument("--max_vel", type=float, default=3.0)
    args = ap.parse_args()
    calib = R.load_calib(args.eps)

    n = coll = reach = 0
    min_clr = 1e18
    fail = []
    for sd in parse_seeds(args.seeds):
        path = os.path.join(R.OUTDIR, f"traj_seed{sd}.npz")
        if not os.path.exists(path):
            continue
        movers = R.Movers(list(np.load(path, allow_pickle=True)["movers"]))
        for k, ep in enumerate(R.build_episodes(movers, sd, n_ep=args.n_ep)):
            with _quiet():
                r = R.run_replay(movers, ep, "ours", calib, max_vel=args.max_vel * args.speedup)
            n += 1
            reach += int(r["reached"])
            if r["collided"]:
                coll += 1; fail.append((sd, k, r["min_clr"]))
            if r["min_clr"] is not None:
                min_clr = min(min_clr, r["min_clr"])
            if n % 50 == 0:
                print(f"[rel] {n} episodes: collisions={coll} reach={reach} min_clr={min_clr:.3f}", flush=True)

    rate = coll / n if n else float("nan")
    ub = 3.0 / n if (coll == 0 and n) else None   # rule of three (95% CI) when 0 collisions
    succ = 1 - rate
    print(f"\n=== RELIABILITY (ours, eps={args.eps}, {n} episodes) ===")
    print(f"  collisions: {coll}/{n}  -> collision rate = {rate:.6f}  (success {succ*100:.4f}%)")
    print(f"  reached: {reach}/{n} ({reach/n*100:.1f}%)   min clearance over all = {min_clr:.3f} m")
    if ub is not None:
        print(f"  0 collisions in {n} -> distribution-free 95%-CI collision rate <= {ub:.6f} (success >= {(1-ub)*100:.4f}%)")
        print(f"  to DIRECTLY verify 99.999% (<=1e-5) need ~3e5 episodes; conformal P(collision)<=eps={args.eps} backs the trend")
    if fail:
        print(f"  collision episodes: {fail[:10]}")
    json.dump(dict(eps=args.eps, n=n, collisions=coll, reach=reach, collision_rate=rate,
                   success_rate=succ, ci95_upper=ub, min_clr=float(min_clr), failures=fail),
              open(os.path.join(R.OUTDIR, f"reliability_eps{args.eps}.json"), "w"), indent=2)
    print(f"[rel] wrote reliability_eps{args.eps}.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
