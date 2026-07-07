"""bench_shard — one (config, shard) slice of the EGO-line pool benchmark (v2-era retest).
Configs: ours_v2 (CALIB_V2 tubes) | ours_legacy | native. Env is set BEFORE importing replay_core."""
import argparse
import json
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--config", choices=["ours_v2", "ours_legacy", "native", "gt_thin", "cone_oracle"],
                required=True)
ap.add_argument("--shard", type=int, required=True)
ap.add_argument("--nshard", type=int, default=4)
ap.add_argument("--out", required=True)
args = ap.parse_args()

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); os.chdir(HERE)
if args.config == "ours_v2":
    os.environ["CALIB_V2"] = "1"; os.environ["CALIB_EPS"] = "0.10"
os.environ["PERCEPT"] = "gt" if args.config == "gt_thin" else "realistic"
if args.config == "cone_oracle":
    # premise ceiling: IDEAL cone sensor -- keep the 45deg cone + occlusion (the premise), zero out
    # noise/miss/clutter/class-error, near-zero tubes. What clean-arrival can ANY algorithm reach
    # under cone-limited information? (paper envelope: the info-theoretic cost of cone sensing)
    for k, v in (("PERCEPT_SIGMA0", "0.0"), ("PERCEPT_SIGMA_K", "0.0"), ("PERCEPT_PMISS0", "0.0"),
                 ("PERCEPT_PMISS_K", "0.0"), ("PERCEPT_FP_RATE", "0.0"), ("PERCEPT_CLS_ERR", "0.0"),
                 ("PERCEPT_SIZE_ERR", "0.0")):
        os.environ[k] = v

import replay_core as RC
import scenario_lib as SLB

CFG = json.load(open("out/conformal/calib_v2_config.json"))
POOL = sorted(CFG["scenario_pool"])[args.shard::args.nshard]
SEEDS = (11, 42, 77, 123, 999)
mode = "native" if args.config == "native" else "ours"
# gt_thin = oracle counterfactual: omniscient perception + near-zero tube. Residual evades under
# near-perfect knowledge are PHYSICALLY NECESSARY; the gap vs ours_v2 = tube-width artifacts
# (the provably-superfluous share of emergency interventions).
CAL_OVR = ({c: (0.05, 0.1) for c in ("pedestrian", "vehicle", "animal", "static")} | {"_all": (0.05, 0.1), "static": (0.05, 0.0)})     if args.config == "gt_thin" else None
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
            r = RC.run_replay(movers, ep, mode=mode, record=False, calib=CAL_OVR,
                              max_vel=float(scn["drone"].get("max_vel", 3.0)))
            cnt = r.get("counts", {})
            emerg = int(cnt.get("evade", 0)) + int(cnt.get("cret", 0))
            row = dict(scn=scn_name, seed=sd, cfg=args.config, reached=bool(r["reached"]),
                       collided=bool(r["collided"]), min_clr=round(float(r["min_clr"]), 3),
                       t=round(float(r["time_s"]), 1), emerg=emerg,
                       clean=bool(r["reached"] and not r["collided"] and emerg == 0))
        except Exception as e:
            row = dict(scn=scn_name, seed=sd, cfg=args.config, error=type(e).__name__)
        out.write(json.dumps(row) + "\n"); out.flush()
out.close()
print(f"[shard {args.config}/{args.shard}] done")
