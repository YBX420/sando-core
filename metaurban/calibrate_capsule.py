"""calibrate_capsule — v6 CAPSULE (segment) conformal: keep-out = [mover's back, KF predicted tip] ⊕ q̃.

The user-drawn shape (2026-07-13): the keep-out must NOT be centred on the mover -- its along-axis
endpoints are the mover's BACK (current position) and the KF prediction's TIP. Score = distance from
the true future position to the SEGMENT [KF anchor c0, predicted c(Δ)]: a mover that stops or slows
scores ~0 (it sits by the segment start) instead of paying the full along-track miss, so the residual
quantile collapses ~2x for pedestrians (q90@0.9s: 1.19 point -> 0.56 segment on designCR).

Machinery = calibrate_v3 verbatim on the esg column. Young/static rows harvest a DEGENERATE segment
(frozen centre -> esg == e), so their laws are unchanged automatically -- one column swap covers the
whole population, no eligibility gate needed at calibration time. The cross-pool stability gate still
runs on the isotropic e slopes (pool sanity is norm-independent).

Deployment (safety_layer CAPSULE=1): mature non-zombie ped/veh are certified as a PEARL-STRING cover
of the capsule -- AND over s in {0..1} of the existing circle cert with obs_vel = s*v and the pearl
gap |v|*t/(2K) folded into v_eff. Zero C++ changes; s=0 is the old frozen-current conjunct, s=1 the
old predicted tube, both at the collapsed radius. calib_v6.json; folds E (shared with v5, noted).
"""
import hashlib
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); os.chdir(HERE)
import calibrate_v3 as V3

N_PEARLS = 4        # deployment pearl count (registered here so cert + calibration stay one object)


def to_segment(D):
    """Copy of the harvest with the score column e replaced by the dist-to-segment residual."""
    D6 = D.copy()
    D6["e"] = D["esg"]
    return D6


if __name__ == "__main__":
    D = V3.load("designCR")
    if "esg" not in D.dtype.names:
        raise SystemExit("designCR harvest lacks the capsule column; re-harvest with the v6 replay_core")
    sh_iso = V3.fit_shapes(D)                     # sets GRID; isotropic twin for the cross-pool gate
    V3.stability_gate(D, sh_iso)
    D6 = to_segment(D)
    sh = V3.fit_shapes(D6)
    print("[shapes:v6]", {f"{k[0][:4]}-{k[1]}": v for k, v in sh.items()})
    blob = json.dumps({f"{k[0]}|{k[1]}": v for k, v in sh.items()}, sort_keys=True).encode()
    sh_hash = hashlib.sha256(blob).hexdigest()[:16]
    B, T = to_segment(V3.load("foldE")), to_segment(V3.load("testE"))
    SB, ST = V3.flight_sups(B, sh), V3.flight_sups(T, sh)
    n = len(SB)
    out = dict(provenance=dict(spec="v6 capsule (segment conformal, pearl-string cert)", shape_hash=sh_hash,
                               n_flights=n, fold="E (shared with v5 -- both ranked, selection by behaviour AB)",
                               date="2026-07-13"),
               capsule=True, n_pearls=N_PEARLS, groups={}, young={}, flags=[])
    vals = np.sort(np.array(list(SB.values())))
    for eps in (0.05, 0.10):
        k = int(np.ceil((n + 1) * (1 - eps)))
        if k > n:
            out["flags"].append(f"eps={eps}: UNDER_CALIBRATED n={n}"); continue
        lam = float(vals[k - 1])
        cov = float(np.mean([s <= lam for s in ST.values()]))
        print(f"[rank] eps={eps}: n={n} k={k} lambda={lam:.3f}  testE coverage={cov:.3f} (n={len(ST)})"
              + ("  MAX_RANK_WARNING" if k == n else ""))
        eps_s = {0.05: "0.05", 0.10: "0.1"}[eps]
        for cls in ("pedestrian", "vehicle", "animal", "static"):
            g = out["groups"].setdefault(cls, {"levels": {}})
            if cls == "animal":
                g["levels"][eps_s] = dict(status="UNCALIBRATED"); continue
            b, v = sh[(cls, "M")] if cls != "static" else sh[("static", "ALL")]
            g["levels"][eps_s] = dict(q_conformal=round(lam * b, 4), v_eff=round(lam * v, 4),
                                      status=("ok_zero_slack" if k == n else "ok"))
            if cls != "static":
                by, vc = sh[(cls, "Y")]
                out["young"].setdefault(cls, {})[eps_s] = dict(q0y=round(lam * by, 3), growth=vc)
        out["young"].setdefault("static", {})[eps_s] = dict(
            q0y=round(lam * sh[("static", "ALL")][0], 3), growth=0.0)
        if eps == 0.10:
            pedm = lam * (sh[("pedestrian", "M")][0] + sh[("pedestrian", "M")][1] * 0.3)
            print(f"[gates] ped-M capsule q̃@0.3={pedm:.2f} (point-law gate 1.5 for reference)")
    att = sorted(((v, kk) for kk, v in SB.items()), reverse=True)[:10]
    json.dump([dict(sup=round(v, 3), scn=kk[0], ep=kk[1]) for v, kk in att],
              open("out/conformal/v6_sup_attribution.json", "w"), indent=1)
    json.dump(out, open(os.path.join("..", "out", "conformal", "calib_v6.json"), "w"), indent=1)
    print("[calib] wrote ../out/conformal/calib_v6.json + v6 sup attribution table")
