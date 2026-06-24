"""ab_replay — the acceptance A/B: ours (conformal-certified maneuvering) vs native EGO, on REAL harvested
MetaUrban mover trajectories, across many contested corridors.

ACCEPTANCE (strict, set by the user 2026-06-24): over all episodes where both reach,
  (1) ours has ZERO collisions, and
  (2) ours_time <= native_time + 2.0 s   (faster is better).
native is the reckless real EGO (grid inflation 0.3, flies ~0.3 m from people); it is allowed to collide --
the contrast is the point. We report time deltas, min clearances, collision/reach counts, maneuver mix.

Run:  python metaurban/ab_replay.py --seeds 0-19 --n_ep 6 --eps 0.05
"""
import os, sys, json, glob, argparse, contextlib
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import replay_core as R

OUTDIR = R.OUTDIR


@contextlib.contextmanager
def _quiet():
    """Silence the EGO C++ stdout spam (fd1) so the JSON/table stays readable."""
    fd = os.dup(1); devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 1); os.close(devnull)
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
    ap.add_argument("--seeds", default="0-19")
    ap.add_argument("--n_ep", type=int, default=6)
    ap.add_argument("--eps", type=float, default=0.05)
    ap.add_argument("--max_vel", type=float, default=3.0)
    ap.add_argument("--ours_speedup", type=float, default=1.0, help="ours flies at max_vel*speedup; the cert gates "
                    "every faster commit so it stays safe (native can't: no cert -> crashes). This is the "
                    "'I KNOW how they move, so I dare to fly FASTER but safe' speed budget the prediction earns.")
    ap.add_argument("--budget", type=float, default=3.0, help="time budget: ours must reach within native+budget s")
    ap.add_argument("--tag", default="")
    args = ap.parse_args()
    calib = R.load_calib(args.eps)
    print(f"[ab] conformal eps={args.eps}  per-class keep-out (q_conformal, v_eff):", flush=True)
    for c in ("pedestrian", "vehicle"):
        print(f"[ab]   {c}: q={calib[c][0]:.3f} m, v_eff={calib[c][1]:.3f} m/s", flush=True)

    rows = []
    for sd in parse_seeds(args.seeds):
        path = os.path.join(OUTDIR, f"traj_seed{sd}.npz")
        if not os.path.exists(path):
            continue
        dat = np.load(path, allow_pickle=True)
        movers = R.Movers(list(dat["movers"]))
        episodes = R.build_episodes(movers, sd, n_ep=args.n_ep)
        for k, ep in enumerate(episodes):
            with _quiet():
                ro = R.run_replay(movers, ep, "ours", calib, max_vel=args.max_vel * args.ours_speedup)
                rn = R.run_replay(movers, ep, "native", calib, max_vel=args.max_vel)
            rows.append(dict(seed=sd, ep=k, n_members=len(ep["members"]),
                             ours_t=ro["time_s"], ours_clr=ro["min_clr"], ours_reach=ro["reached"],
                             ours_coll=ro["collided"], ours_maxz=round(ro["max_z"], 2), counts=ro["counts"],
                             nat_t=rn["time_s"], nat_clr=rn["min_clr"], nat_reach=rn["reached"],
                             nat_coll=rn["collided"]))
            o, n = rows[-1], rows[-1]
            print(f"[ab] seed{sd} ep{k} (m={o['n_members']}): "
                  f"ours t={o['ours_t']:.1f} clr={_f(o['ours_clr'])} reach={int(o['ours_reach'])} coll={int(o['ours_coll'])} "
                  f"| nat t={o['nat_t']:.1f} clr={_f(o['nat_clr'])} reach={int(o['nat_reach'])} coll={int(o['nat_coll'])} "
                  f"| dt={o['ours_t']-o['nat_t']:+.1f}", flush=True)

    summarize(rows, args)
    return 0


def _f(x):
    return "None" if x is None else f"{x:.2f}"


def summarize(rows, args):
    both = [r for r in rows if r["ours_reach"] and r["nat_reach"]]
    ours_coll = sum(r["ours_coll"] for r in rows)
    nat_coll = sum(r["nat_coll"] for r in rows)
    ours_reach = sum(r["ours_reach"] for r in rows)
    nat_reach = sum(r["nat_reach"] for r in rows)
    dts = [r["ours_t"] - r["nat_t"] for r in both]
    ours_clrs = [r["ours_clr"] for r in rows if r["ours_clr"] is not None]
    nat_clrs = [r["nat_clr"] for r in rows if r["nat_clr"] is not None]
    budget = getattr(args, "budget", 3.0)
    within2 = sum(1 for d in dts if d <= budget + 1e-9)
    faster = sum(1 for d in dts if d < 0)

    print("\n" + "=" * 78)
    print(f"  A/B SUMMARY  (eps={args.eps}, {len(rows)} episodes, {len(both)} where both reach)")
    print("=" * 78)
    print(f"  collisions:   ours={ours_coll}/{len(rows)}    native={nat_coll}/{len(rows)}")
    print(f"  reached goal: ours={ours_reach}/{len(rows)}    native={nat_reach}/{len(rows)}")
    if ours_clrs:
        print(f"  min clearance (m): ours min={min(ours_clrs):.3f} median={np.median(ours_clrs):.3f}   "
              f"native min={min(nat_clrs):.3f} median={np.median(nat_clrs):.3f}")
    if dts:
        print(f"  time delta ours-native (s): median={np.median(dts):+.2f}  mean={np.mean(dts):+.2f}  "
              f"min={min(dts):+.2f}  max={max(dts):+.2f}")
        print(f"  ours within native+{budget:g}s: {within2}/{len(dts)}    ours strictly faster: {faster}/{len(dts)}")
    print("-" * 78)
    acc_zero_coll = (ours_coll == 0)
    acc_within2 = dts and all(d <= budget + 1e-9 for d in dts)
    print(f"  ACCEPTANCE  zero-collision(ours): {'PASS' if acc_zero_coll else 'FAIL'}    "
          f"all within native+{budget:g}s: {'PASS' if acc_within2 else 'FAIL'}")
    print("=" * 78 + "\n")

    out = dict(eps=args.eps, max_vel=args.max_vel, n_episodes=len(rows), n_both_reach=len(both),
               ours_collisions=int(ours_coll), native_collisions=int(nat_coll),
               ours_reached=int(ours_reach), native_reached=int(nat_reach),
               time_delta_median=float(np.median(dts)) if dts else None,
               time_delta_max=float(max(dts)) if dts else None,
               within_2s=int(within2), strictly_faster=int(faster), n_both=len(both),
               ours_min_clr=float(min(ours_clrs)) if ours_clrs else None,
               native_min_clr=float(min(nat_clrs)) if nat_clrs else None,
               acceptance_zero_collision=bool(acc_zero_coll), acceptance_within_2s=bool(acc_within2),
               rows=rows)
    os.makedirs(OUTDIR, exist_ok=True)
    fn = os.path.join(OUTDIR, f"ab_summary{('_' + args.tag) if args.tag else ''}.json")
    with open(fn, "w") as f:
        json.dump(out, f, indent=2)
    print(f"[ab] wrote {fn}")


if __name__ == "__main__":
    sys.exit(main())
