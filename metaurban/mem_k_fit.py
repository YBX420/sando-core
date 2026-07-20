"""mem_k_fit — EGO_MEM_K determination on the retired-design face (07-20 final sequence, step 4).

EGO_MEM_K is an ALGORITHM hyperparameter (the out-of-cone memory keep-out inflation, applied as
K * pos_sigma while a track coasts). Per the 07-20 ruling it must be fixed on the RETIRED design
face and frozen BEFORE any new-scenario cal/test data exists -- K changes flight paths, so it can
never be chosen by looking at the calibration folds.

Fit: over COAST rows (the population the memory keep-out exists for), the ratio e / psig is the
multiplier that would have covered THIS row's true prediction error with the filter's own
uncertainty. K = the design-face shape quantile (q90, the house shape convention) of that ratio,
reported per class and per coast-age bucket, with q95/q99 for the tail picture.
"""
import argparse
import json

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--npy", required=True, help="merged design69 v3 npy")
ap.add_argument("--out", default=None, help="optional json report path")
args = ap.parse_args()

D = np.load(args.npy)
m = (D["coast"] == 1) & (D["psig"] > 1e-6)
C = D[m]
print(f"[mem_k] rows={len(D)} coast rows={len(C)} "
      f"({100.0 * len(C) / max(1, len(D)):.1f}%) scenarios={len(set(D['scn'].tolist()))}")
if len(C) < 50:
    raise SystemExit("[mem_k] too few coast rows to fit K -- inspect the harvest")

ratio = C["e"] / C["psig"]
rep = dict(n_coast=int(len(C)),
           q90=round(float(np.quantile(ratio, 0.90)), 3),
           q95=round(float(np.quantile(ratio, 0.95)), 3),
           q99=round(float(np.quantile(ratio, 0.99)), 3),
           per_class={}, per_coast_age={})
print(f"[mem_k] e/psig quantiles: q90={rep['q90']} q95={rep['q95']} q99={rep['q99']}  "
      f"(current default K=2.0)")
for cls in sorted(set(C["cls"].tolist())):
    mc = C["cls"] == cls
    if mc.sum() < 30:
        rep["per_class"][cls] = dict(n=int(mc.sum()), note="too few rows")
        continue
    rep["per_class"][cls] = dict(n=int(mc.sum()),
                                 q90=round(float(np.quantile(ratio[mc], 0.90)), 3),
                                 q95=round(float(np.quantile(ratio[mc], 0.95)), 3))
    print(f"[mem_k]   {cls:12s} n={mc.sum():6d}  q90={rep['per_class'][cls]['q90']:6.3f}  "
          f"q95={rep['per_class'][cls]['q95']:6.3f}")
for lo, hi in ((0.0, 0.3), (0.3, 0.8), (0.8, 1.6)):
    mb = (C["coast_s"] >= lo) & (C["coast_s"] < hi)
    if mb.sum() < 30:
        continue
    q90 = round(float(np.quantile(ratio[mb], 0.90)), 3)
    rep["per_coast_age"][f"{lo}-{hi}s"] = dict(n=int(mb.sum()), q90=q90)
    print(f"[mem_k]   coast {lo}-{hi}s  n={mb.sum():6d}  q90={q90:6.3f}")
rep["recommendation"] = dict(K=rep["q90"],
                             rule="design-face shape quantile q90 over coast rows (house shape "
                                  "convention); frozen BEFORE any cal/test data exists")
print(f"[mem_k] RECOMMENDED EGO_MEM_K = {rep['q90']} (frozen on the retired design face)")
if args.out:
    json.dump(rep, open(args.out, "w"), indent=1)
    print(f"[mem_k] report -> {args.out}")
