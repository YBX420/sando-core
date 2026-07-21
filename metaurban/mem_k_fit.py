"""mem_k_fit — coast anchor-buffer evidence on the retired-design face (07-21 corrected).

07-21 ruling: the 07-20 pooled fit priced e(d)/psig(0) over ALL horizons, but the runtime K only
inflates the CURRENT coast anchor (anchor radius += K * current pos_sigma) -- two different
quantities, so the pooled q90 is INVALID as a K. This tool now reports the STRATIFIED truth:

  - the ANCHOR law (d == 0 rows only): the only stratum whose quantity matches what K buys;
  - per-horizon strata (how fast the pooled number inflates with d -- the part the final
    conformal tube prices via q + v_eff*t; charging it into K would double-bill);
  - per-maturity strata (young frozen-predictor rows dominate the fat tails).

Caveats stamped into the report: this face is the 0.30 s replay cadence; a renderer K refit must
run at 0.10 s with the renderer's predictor/young policy, and any class K must be wired into BOTH
the realistic and GT-keyed coast paths. Current ruling: uniform K=2.0 stays (engineering anchor
buffer), no class K, no age-taper this cycle.
"""
import argparse
import hashlib
import json

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--npy", required=True, help="merged design v3 npy")
ap.add_argument("--out", default=None, help="optional json report path")
args = ap.parse_args()

D = np.load(args.npy)
m = (D["coast"] == 1) & (D["psig"] > 1e-6)
C = D[m]
ratio = C["e"] / C["psig"]
print(f"[mem_k] rows={len(D)} coast rows={len(C)} scenarios={len(set(D['scn'].tolist()))} "
      f"(cadence: REPLAY 0.30 s -- renderer application needs its own 0.10 s refit)")


def q(x, p):
    return round(float(np.quantile(x, p)), 3) if len(x) else None


rep = dict(input_sha=hashlib.sha256(open(args.npy, "rb").read()).hexdigest()[:16],
           n_coast=int(len(C)), cadence_s=0.30,
           pooled_all_horizons=dict(q90=q(ratio, 0.90), q95=q(ratio, 0.95),
                                    note="INVALID as K: mixes future-horizon error into an "
                                         "anchor-buffer quantity (07-21 ruling)"),
           anchor_d0={}, per_horizon={}, per_maturity={})

d0 = np.abs(C["d"]) < 1e-6
print(f"[mem_k] ANCHOR law (d=0 only, the quantity K actually buys): n={int(d0.sum())} "
      f"q90={q(ratio[d0], 0.90)} q95={q(ratio[d0], 0.95)}")
rep["anchor_d0"]["all"] = dict(n=int(d0.sum()), q90=q(ratio[d0], 0.90), q95=q(ratio[d0], 0.95))
for cls in sorted(set(C["cls"].tolist())):
    mc = d0 & (C["cls"] == cls)
    if mc.sum() < 30:
        continue
    rep["anchor_d0"][cls] = dict(n=int(mc.sum()), q90=q(ratio[mc], 0.90), q95=q(ratio[mc], 0.95))
    print(f"[mem_k]   anchor {cls:12s} n={int(mc.sum()):6d} q90={rep['anchor_d0'][cls]['q90']}")

for dv in sorted(set(np.round(C["d"], 2).tolist())):
    md = np.abs(C["d"] - dv) < 1e-6
    if md.sum() < 30:
        continue
    rep["per_horizon"][str(dv)] = dict(n=int(md.sum()), q90=q(ratio[md], 0.90))
    print(f"[mem_k]   horizon d={dv:5.2f} n={int(md.sum()):6d} q90={rep['per_horizon'][str(dv)]['q90']}"
          f"   <- priced by the conformal tube (q + v_eff*t), NOT by K")

for tag, mm in (("mature(age>=4)", C["age"] >= 4), ("young(age<4)", C["age"] < 4)):
    if mm.sum() < 30:
        continue
    rep["per_maturity"][tag] = dict(n=int(mm.sum()), q90=q(ratio[mm], 0.90))
    print(f"[mem_k]   {tag:16s} n={int(mm.sum()):6d} q90={rep['per_maturity'][tag]['q90']}")

rep["ruling"] = ("07-21: uniform K=2.0 retained (engineering anchor buffer); vehicle K=13.5 "
                 "RETRACTED (quantity mismatch); future-horizon error belongs to the final "
                 "conformal calibration; refit preconditions: renderer cadence + same predictor/"
                 "young policy + d=0 law + both coast paths")
if args.out:
    json.dump(rep, open(args.out, "w"), indent=1)
    print(f"[mem_k] report -> {args.out}")
