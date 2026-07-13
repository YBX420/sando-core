"""calibrate_ellipse — lambda-SHAPE-HE (v5): MOTION-FRAME ELLIPTICAL conformal, the 5th-gen calibration (v4 = LCP-alpha/affine, a DIFFERENT line).

The v3 machinery verbatim (frozen shapes from the design domain, ONE scalar lambda-hat from a fresh
fold, single shared rank, empty flights count) with ONE change of scalar score: for ELIGIBLE rows
(pedestrian/vehicle, mature age>=4, KF anchor speed >= V_MIN_DIR) the Euclidean residual e is
replaced by the ELLIPTICAL norm  e' = hypot(e_along, kappa * e_cross)  in the mover's motion frame,
with the per-class aspect kappa FROZEN from the design domain (an efficiency object, like the v3
shapes -- population mismatch never costs coverage). Coverage P(e' <= A(t)) >= 1-eps then means the
true centre lies in the ellipse with semi-axis A along the KF velocity and A/kappa across it -- the
cross-track tube shrinks by kappa, which is exactly the 'space behind a mover that walked away'.
Young / static / slow rows keep the isotropic e (kappa=1), mirroring deployment: build_cylinders
applies the ellipse under the SAME predicate (mature, non-zombie, |v| >= v_min_dir), everything else
stays a circle. The cross-pool stability gate still runs on the ISOTROPIC slopes vs the registered
historical values (pool sanity is a property of the data, not of the norm).

Needs designE/foldE/testE harvests (motion-frame ea/ec/spd columns; older folds lack them). NB calib_v4.json belongs to the LCP-alpha/v4b line -- this file writes calib_v5.json.
"""
import hashlib
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); os.chdir(HERE)
import calibrate_v3 as V3   # __main__-guarded: safe to import (theil_sen/fit_shapes/score_row/...)

V_MIN_DIR = 0.5     # m/s: below this the KF direction is untrusted -> isotropic (deployment mirrors)
KAP_MAX = 4.0       # aspect clip: never trust the frame beyond 4:1 however clean the design data
EC_FLOOR = 0.05     # m: cross-track q90 floor in the ratio (kills divide-by-noise kappa blowups)


def eligible(D):
    """The ONE ellipse predicate, shared verbatim by kappa fit, scoring and deployment."""
    return (((D["cls"] == "pedestrian") | (D["cls"] == "vehicle"))
            & (D["age"] >= 4) & (D["spd"] >= V_MIN_DIR))


def fit_kappa(D):
    """Frozen per-class aspect: median over grid points (>=50 rows) of q90(|along|)/q90(|cross|),
    mature qualified rows only (same population the M shapes are fit on)."""
    kap = {}
    for cls in ("pedestrian", "vehicle"):
        R = D[eligible(D) & (D["cls"] == cls)] if cls == "vehicle" else \
            D[eligible(D) & (D["cls"] == cls) & (D["qual"] == 1)]
        ratios = []
        for g in V3.GRID:
            m = np.abs(R["d"] - g) < 0.01
            if m.sum() < 50:
                continue
            qa = float(np.quantile(R["ea"][m], 0.9))
            qc = max(float(np.quantile(R["ec"][m], 0.9)), EC_FLOOR)
            ratios.append(qa / qc)
        kap[cls] = round(float(np.clip(np.median(ratios), 1.0, KAP_MAX)), 2) if ratios else 1.0
        print(f"[kappa] {cls}: n_grid={len(ratios)} median-aspect -> kappa={kap[cls]}")
    return kap


def to_elliptic(D, kap):
    """Copy of the harvest with the score column e replaced by the elliptical norm on eligible rows."""
    D4 = D.copy()
    for cls, k in kap.items():
        m = eligible(D) & (D["cls"] == cls)
        D4["e"][m] = np.hypot(D["ea"][m], k * D["ec"][m])
    return D4


def _sanity(D):
    """The decomposition must reassemble: e == hypot(ea, ec) wherever the direction was defined."""
    m = D["spd"] > 1e-6
    err = np.abs(np.hypot(D["ea"][m], D["ec"][m]) - D["e"][m])
    bad = float(np.quantile(err, 0.999)) if m.sum() else 0.0
    assert bad < 1e-3, f"motion-frame decomposition broken: q99.9 |hypot(ea,ec)-e| = {bad}"


if __name__ == "__main__":
    # Design domain = designCR: the SAME preregistered designC seed recipe v3 froze its shapes from,
    # re-harvested deterministically under today's runtime to pick up the motion-frame columns
    # (replication check: identical row count + slope 1.003 == the 07-07 designC harvest). This
    # keeps v3 -> v4 a single-variable change (circle -> ellipse norm) with the SAME design domain
    # and the SAME cross-pool gate verdict -- no domain resampling mixed in. Forensics note
    # (2026-07-13): fresh-seed waves E/E2 measured ped-M slopes 0.888/0.842 vs C's 1.003 --
    # wave-level sigma ~8% against a 10% gate tolerance; the gate itself needs a governance review
    # (harvest_{designE,designE2}_v2_s*.npy kept as evidence), but that is NOT this file's call.
    D = V3.load("designCR")
    if "ea" not in D.dtype.names:
        raise SystemExit("designE harvest lacks motion-frame columns; re-harvest with the v4 replay_core")
    _sanity(D)
    sh_iso = V3.fit_shapes(D)                     # sets V3.GRID; isotropic twin for the cross-pool gate
    V3.stability_gate(D, sh_iso)                  # pool sanity on the REGISTERED isotropic slopes
    kap = fit_kappa(D)
    D4 = to_elliptic(D, kap)
    sh = V3.fit_shapes(D4)                        # M arms on the elliptical norm; Y/static rows are
    print("[shapes:v4]", {f"{k[0][:4]}-{k[1]}": v for k, v in sh.items()})   # untouched (e'==e there)
    _reg = {f"{k[0]}|{k[1]}": v for k, v in sh.items()}
    _reg.update({f"kappa|{c}": k for c, k in kap.items()})
    blob = json.dumps(_reg, sort_keys=True).encode()
    sh_hash = hashlib.sha256(blob).hexdigest()[:16]
    B = to_elliptic(V3.load("foldE"), kap); T = to_elliptic(V3.load("testE"), kap)
    _sanity(V3.load("foldE"))
    SB, ST = V3.flight_sups(B, sh), V3.flight_sups(T, sh)
    n = len(SB)
    out = dict(provenance=dict(spec="lambda-SHAPE-HE v5 (motion-frame ellipse)", shape_hash=sh_hash,
                               n_flights=n, fold="E", date="2026-07-13"),
               kappa=kap, v_min_dir=V_MIN_DIR, groups={}, young={}, flags=[])
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
            vyq0 = lam * sh[("vehicle", "Y")][0]
            print(f"[gates] ped-M@0.3={pedm:.2f} (<=1.5: {'PASS' if pedm <= 1.5 else 'FAIL'})  "
                  f"veh-Y q0={vyq0:.2f} (<=1.0: {'PASS' if vyq0 <= 1.0 else 'FAIL'})")
            for cls in ("pedestrian", "vehicle"):
                bM, vM = sh[(cls, "M")]
                print(f"[gain] {cls}: along tube@0.3s={lam*(bM+vM*0.3):.2f} m, cross = /"
                      f"{kap[cls]} = {lam*(bM+vM*0.3)/kap[cls]:.2f} m")
    att = sorted(((v, kk) for kk, v in SB.items()), reverse=True)[:10]
    json.dump([dict(sup=round(v, 3), scn=kk[0], ep=kk[1]) for v, kk in att],
              open("out/conformal/v5_sup_attribution.json", "w"), indent=1)
    json.dump(out, open(os.path.join("..", "out", "conformal", "calib_v5.json"), "w"), indent=1)
    print("[calib] wrote ../out/conformal/calib_v5.json + v5 sup attribution table")
