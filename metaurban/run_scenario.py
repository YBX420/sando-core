"""run_scenario — run a designed scenario JSON through the untouched replay_core evaluation.

  python run_scenario.py scenarios/vehicle_spawn_accel.json                 # single run
  python run_scenario.py scenarios/wall_and_crosser.json --variants 10      # expand params, run all
  python run_scenario.py x.json --mode native --dynamics                    # baseline / real quad dynamics

Writes out/scenario_runs/<name>_hist.json: {"scenario", "mode", "result", "history":[...]} with the drone
path mapped BACK to world frame (run_replay works in a corridor-local frame; org is re-added here), so the
designer overlays it directly:  python scenario_designer.py --load <scn.json> --replay <hist.json>
"""
import argparse
import json
import os

import numpy as np

import scenario_lib as SLB
from replay_core import Movers, run_replay

HERE = os.path.dirname(os.path.abspath(__file__))
OUTD = os.path.join(HERE, "out", "scenario_runs")


def run_one(scn, mode="ours", dynamics=False, record=True, tick_cb=None):
    if scn.get("drone", {}).get("waypoints"):
        print(f"[warn] {scn['name']}: drone.waypoints are NOT consumed yet -- flying start->goal direct")
    movers = SLB.apply_rh_overrides(Movers(SLB.to_movers_raw(scn)), scn)
    ep = SLB.to_episode(scn)
    d = scn["drone"]
    res = run_replay(movers, ep, mode=mode, record=record, dynamics=dynamics,
                     max_vel=float(d.get("max_vel", 3.0)), max_acc=float(d.get("max_acc", 6.0)),
                     tick_cb=tick_cb)
    org = 0.5 * (np.asarray(d["start"][:2]) + np.asarray(d["goal"][:2]))
    hist = res.pop("history", None) or []
    for h in hist:                                        # local corridor frame -> world frame
        h["p"] = [round(h["p"][0] + org[0], 2), round(h["p"][1] + org[1], 2)]
        if "pref" in h:
            h["pref"] = [round(h["pref"][0] + org[0], 2), round(h["pref"][1] + org[1], 2)]
    return res, hist


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario")
    ap.add_argument("--mode", default="ours", choices=["ours", "native", "sando"])
    ap.add_argument("--dynamics", action="store_true", help="fly through real quadrotor dynamics")
    ap.add_argument("--variants", type=int, default=0, help="expand params into N jittered variants, run all")
    ap.add_argument("--seed", type=int, default=0, help="variant jitter seed")
    ap.add_argument("--resample", type=int, default=0,
                    help="under PERCEPT=realistic, rerun each scenario N times with different sensor "
                         "seeds and report min_clr quantiles -- single-draw numbers are NOT citable")
    args = ap.parse_args()
    os.makedirs(OUTD, exist_ok=True)

    scns = (SLB.expand_variants(args.scenario, args.variants, args.seed) if args.variants
            else [SLB.load(args.scenario)])
    agg = []
    for scn in scns:
        if args.resample > 1:
            runs = []
            for k in range(args.resample):
                os.environ["PERCEPT_SEED"] = str(1234567 + k * 7919)
                r, _ = run_one(scn, mode=args.mode, dynamics=args.dynamics, record=False)
                runs.append(r)
            os.environ.pop("PERCEPT_SEED", None)
            clr = np.array([r["min_clr"] for r in runs])
            res = dict(runs[0], min_clr=float(np.median(clr)),
                       resample=dict(n=len(runs),
                                     reached=int(sum(r["reached"] for r in runs)),
                                     collided=int(sum(r["collided"] for r in runs)),
                                     clr_p05=float(np.quantile(clr, 0.05)),
                                     clr_med=float(np.median(clr)), clr_worst=float(clr.min())))
            hist = []
            rs = res["resample"]
            print(f"[{scn['name']}] x{rs['n']} sensor draws: reached {rs['reached']}/{rs['n']} "
                  f"collided {rs['collided']}  min_clr med={rs['clr_med']:.3f} "
                  f"p05={rs['clr_p05']:.3f} worst={rs['clr_worst']:.3f}")
        else:
            res, hist = run_one(scn, mode=args.mode, dynamics=args.dynamics)
            print(f"[{scn['name']}] reached={res['reached']} t={res['time_s']}s min_clr={res['min_clr']:.3f} "
                  f"collided={res['collided']} counts={res['counts']}")
        out = os.path.join(OUTD, f"{scn['name']}_{args.mode}_hist.json")
        json.dump(dict(scenario=scn["name"], mode=args.mode, result=res, history=hist),
                  open(out, "w"), indent=None, default=float)
        agg.append(res)
    if len(agg) > 1:
        print(f"\n[agg] {len(agg)} variants: reached {sum(r['reached'] for r in agg)}/{len(agg)}, "
              f"collided {sum(r['collided'] for r in agg)}, "
              f"min_clr med={np.median([r['min_clr'] for r in agg]):.3f} "
              f"worst={min(r['min_clr'] for r in agg):.3f}")


if __name__ == "__main__":
    main()
