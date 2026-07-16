"""compare3 — three-way headless comparison on the SAME real trajectories: ours vs native EGO vs native SANDO.

Two regimes (the user's experiments):
  --regime matched : every planner at the SAME max_vel (isolates planner quality at equal speed).
  --regime nolimit : ours flies at a HIGH speed cap gated only by the certificate (no artificial limit, only the
                     0-collision/reach constraint binds) vs the baselines at the same high cap -> shows how much
                     speed the certificate lets ours safely exploit (where the baselines collide / fail).

Writes a per-episode CSV (out/conformal/compare3_<tag>.csv) + a summary (reach rate, collision rate, median time,
median clearance per planner). Needs LD_LIBRARY_PATH=~/gurobi1103/linux64/lib for the SANDO (GUROBI) baseline.

Run:  python metaurban/compare3.py --regime matched --max_vel 3.0 --seeds 0-19 --n_ep 6
"""
import os, sys, csv, json, argparse, contextlib
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import replay_core as R

OUT = R.OUTDIR
MODES = [("ours", "ours"), ("ego", "native"), ("sando", "sando")]   # (label, run_replay mode)


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
    ap.add_argument("--speeds", default="3,4,5,6,7,8", help="MATCHED-speed sweep (m/s): every planner at each, all recorded")
    ap.add_argument("--seeds", default="0-19")
    ap.add_argument("--n_ep", type=int, default=6)
    ap.add_argument("--eps", type=float, default=0.05)
    ap.add_argument("--tag", default="matched")
    args = ap.parse_args()
    calib = R.load_calib(args.eps)
    tag = args.tag
    speeds = [float(x) for x in args.speeds.split(",")]

    rows = []
    for vmax in speeds:
        for sd in parse_seeds(args.seeds):
            path = os.path.join(OUT, f"traj_seed{sd}.npz")
            if not os.path.exists(path):
                continue
            movers = R.Movers(list(np.load(path, allow_pickle=True)["movers"]))
            for k, ep in enumerate(R.build_episodes(movers, sd, n_ep=args.n_ep)):
                row = dict(vmax=vmax, seed=sd, ep=k, n_movers=len(ep["members"]))
                for label, mode in MODES:
                    with _quiet():
                        r = R.run_replay(movers, ep, mode, calib, max_vel=vmax)
                    row[f"{label}_reach"] = int(r["reached"])
                    row[f"{label}_t"] = round(r["time_s"], 1)
                    row[f"{label}_clr"] = round(r["min_clr"], 3) if r["min_clr"] is not None else None
                    row[f"{label}_coll"] = int(r["collided"])
                rows.append(row)
        # per-speed summary line as we go (so a long sweep shows progress)
        sub = [r for r in rows if r["vmax"] == vmax]
        print(f"[c3] vmax={vmax}: " + " | ".join(
            f"{l} reach={sum(r[f'{l}_reach'] for r in sub)}/{len(sub)} coll={sum(r[f'{l}_coll'] for r in sub)}"
            for l, _ in MODES), flush=True)

    # CSV (every episode at every speed, reusable)
    csv_path = os.path.join(OUT, f"compare3_{tag}.csv")
    if rows:
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

    def _stats(sub, label):
        reach = [r for r in sub if r[f"{label}_reach"]]
        coll = sum(r[f"{label}_coll"] for r in sub)
        clrs = [r[f"{label}_clr"] for r in sub if r[f"{label}_clr"] is not None]
        ts = [r[f"{label}_t"] for r in reach]
        return dict(reach_rate=round(len(reach) / len(sub), 3) if sub else 0, collisions=coll, n=len(sub),
                    min_clr=round(min(clrs), 3) if clrs else None,
                    median_clr=round(float(np.median(clrs)), 3) if clrs else None,
                    median_t=round(float(np.median(ts)), 1) if ts else None)

    print(f"\n=== 3-WAY MATCHED-SPEED SWEEP ({len(rows)} episode-runs over speeds {speeds}) ===")
    summary = {}
    for vmax in speeds:
        sub = [r for r in rows if r["vmax"] == vmax]
        summary[str(vmax)] = {label: _stats(sub, label) for label, _ in MODES}
        print(f"  vmax={vmax} m/s ({len(sub)} eps):")
        for label, _ in MODES:
            s = summary[str(vmax)][label]
            print(f"    {label:>6}: reach={s['reach_rate']*100:.0f}%  collisions={s['collisions']}/{s['n']}  "
                  f"min_clr={s['min_clr']}  median_clr={s['median_clr']}  median_t={s['median_t']}s")
    json.dump(dict(speeds=speeds, summary=summary), open(os.path.join(OUT, f"compare3_{tag}.json"), "w"), indent=2)
    print(f"[c3] wrote {csv_path} + compare3_{tag}.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
