"""bench_shard — one (config, shard) slice of the EGO-line pool benchmark (v2-era retest).
Configs: ours_v2 (CALIB_V2 tubes) | ours_legacy | native. Env is set BEFORE importing replay_core."""
import argparse
import json
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--config", choices=["ours_v2", "ours_legacy", "native"], required=True)
ap.add_argument("--shard", type=int, required=True)
ap.add_argument("--nshard", type=int, default=4)
ap.add_argument("--out", required=True)
args = ap.parse_args()

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); os.chdir(HERE)
if args.config == "ours_v2":
    os.environ["CALIB_V2"] = "1"; os.environ["CALIB_EPS"] = "0.10"
os.environ["PERCEPT"] = "realistic"

import replay_core as RC
import scenario_lib as SLB

CFG = json.load(open("out/conformal/calib_v2_config.json"))
POOL = sorted(CFG["scenario_pool"])[args.shard::args.nshard]
SEEDS = (11, 42, 77, 123, 999)
mode = "native" if args.config == "native" else "ours"
out = open(args.out, "a")
for scn_name in POOL:
    f = (f"scenarios/{scn_name}.json" if os.path.exists(f"scenarios/{scn_name}.json")
         else f"scenarios/bench/{scn_name}.json")
    scn = SLB.load(f)
    movers = SLB.apply_rh_overrides(RC.Movers(SLB.to_movers_raw(scn)), scn)
    ep = SLB.to_episode(scn)
    for sd in SEEDS:
        os.environ["PERCEPT_SEED"] = str(sd)
        try:
            r = RC.run_replay(movers, ep, mode=mode, record=False,
                              max_vel=float(scn["drone"].get("max_vel", 3.0)))
            row = dict(scn=scn_name, seed=sd, cfg=args.config, reached=bool(r["reached"]),
                       collided=bool(r["collided"]), min_clr=round(float(r["min_clr"]), 3),
                       t=round(float(r["time_s"]), 1))
        except Exception as e:
            row = dict(scn=scn_name, seed=sd, cfg=args.config, error=type(e).__name__)
        out.write(json.dumps(row) + "\n"); out.flush()
out.close()
print(f"[shard {args.config}/{args.shard}] done")
