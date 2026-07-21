"""calibrate_v3 — lambda-SHAPE-H (Frozen-Shape Hybrid Conformal), the 3rd-gen calibration.

No sigma anywhere. Frozen shapes from the design domain carry all structure; ONE scalar lambda-hat
from a fresh fold carries all estimation risk -- and only into tube WIDTH, never into coverage
(validity/efficiency decoupled). Scores: mature/static MULTIPLICATIVE s=e/(b+v*Delta); young
ADDITIVE s=(e_fc - VCAP*Delta)/b_y with the slope PINNED at the physical cap (never multiplied by
lambda-hat -- kills theta5 hijack and super-physical growth). Young harvest rows already score vs
the frozen centre (replay_core HARV_V2), so e IS e_fc for age<4. Single shared rank; empty flights
count in n with S=-inf. Design spec: sigma-gen3 workflow wf_18b3fdce-93a (2026-07-08).
"""
import glob
import hashlib
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); os.chdir(HERE)
GRID = None
VCAP = {"pedestrian": 2.2, "vehicle": 11.0, "static": 0.0}
B_MIN = 0.05
GATES = {"ped_M_at_0.3_eps0.1": 1.5, "veh_Y_q0_eps0.1": 1.0}


def load(name):
    fs = sorted(glob.glob(f"out/conformal/harvest_{name}_v2_s*.npy")) or \
        [f"out/conformal/harvest_{name}_v2.npy"]
    return np.concatenate([np.load(f) for f in fs])


def theil_sen(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    return float(np.median([(y[j] - y[i]) / (x[j] - x[i])
                            for i in range(len(x)) for j in range(i + 1, len(x)) if x[j] > x[i]]))


def fit_shapes(D):
    global GRID
    GRID = sorted({round(float(g), 2) for g in np.unique(np.round(D["d"], 2))})
    sh = {}
    for cls in ("pedestrian", "vehicle"):
        # veh-M: designC qualified mature veh = 0 rows -> fit on qual 0∪1 (preregistered; shape is
        # an efficiency object only, population mismatch never costs coverage)
        R = D[(D["cls"] == cls) & (D["age"] >= 4)] if cls == "vehicle" else \
            D[(D["cls"] == cls) & (D["age"] >= 4) & (D["qual"] == 1)]
        q90 = [float(np.quantile(R["e"][np.abs(R["d"] - g) < 0.01], 0.9))
               for g in GRID if (np.abs(R["d"] - g) < 0.01).sum() >= 50]
        gs = [g for g in GRID if (np.abs(R["d"] - g) < 0.01).sum() >= 50]
        v = theil_sen(gs, q90)
        b = max(max(q - v * g for q, g in zip(q90, gs)), B_MIN)
        sh[(cls, "M")] = (round(b, 3), round(v, 3))
        Ry = D[(D["cls"] == cls) & (D["age"] >= 2) & (D["age"] <= 3) & (D["qual"] == 1)]
        if len(Ry) < 500:      # sparse qualified arm (veh): fit on qual 0∪1, preregistered same as veh-M
            Ry = D[(D["cls"] == cls) & (D["age"] >= 2) & (D["age"] <= 3)]
            print(f"[shapes] {cls}-Y: qualified rows sparse -> shape fit on qual0∪1 (n={len(Ry)})")
        by = max(float(np.quantile(Ry["e"][np.abs(Ry["d"] - g) < 0.01], 0.95)) - VCAP[cls] * g
                 for g in GRID if (np.abs(Ry["d"] - g) < 0.01).sum() >= 30)
        sh[(cls, "Y")] = (round(max(by, 0.30), 3), VCAP[cls])
    Rs = D[(D["cls"] == "static") & (D["qual"] == 1)]
    if len(Rs) < 500:
        Rs = D[D["cls"] == "static"]
    bs = max(float(np.quantile(Rs["e"][np.abs(Rs["d"] - g) < 0.01], 0.9))
             for g in GRID if (np.abs(Rs["d"] - g) < 0.01).sum() >= 50)
    sh[("static", "ALL")] = (round(max(bs, 0.27), 3), 0.0)   # register conservative side per spec
    return sh


HIST_SLOPES = {"pedestrian": 1.02, "vehicle": 0.95}   # registered from the retired-era pipeline


def stability_gate(D, sh):
    """theta3-immunity gate, REVISED after bootstrap audit (2026-07-08): random half-pool splits
    have median slope drift 29% / q95 40% from SAMPLING ALONE (60 bootstraps; the workflow's 10%
    split-gate passes 0% of null splits = miscalibrated). The real cross-POOL check is vs the
    independently-registered historical slopes; half-pool dispersion is width-risk, reported only."""
    for cls in ("pedestrian", "vehicle"):
        v = sh[(cls, "M")][1]
        drift = abs(v - HIST_SLOPES[cls]) / HIST_SLOPES[cls]
        print(f"[gate] {cls} slope designC={v:.3f} vs historical={HIST_SLOPES[cls]} drift={drift:.3f}")
        if drift > 0.10:
            raise SystemExit(f"CROSS-POOL GATE FAIL {cls}: fail-closed, no calib emitted")


def score_row(e, d, age, cls, sh):
    if cls == "static":
        b, _ = sh[("static", "ALL")]
        return e / max(b, B_MIN)
    if age <= 3:
        by, vc = sh[(cls, "Y")]
        return (e - vc * d) / by
    if (cls, "M") not in sh:
        return None
    b, v = sh[(cls, "M")]
    return e / max(b + v * d, B_MIN)


def flight_sups(D, sh, universe=None):
    """Per-flight sup scores. universe: optional {scn: [ep, ...]} registration (harvest_v3
    universe.json) -- every REGISTERED episode enters the ranking, zero-row episodes at -inf.
    (07-21 ruling: 'visible in the manifest' is not enough -- without this, ~20 of the 69 retired
    scenarios silently vanished from the ranking universe because this function only enumerated
    episodes that happened to have residual rows.)"""
    out = {}
    keys = {(str(s), int(e)) for s, e in zip(D["scn"], D["ep"])}
    if universe is not None:
        keys |= {(str(s), int(e)) for s, eps in universe.items() for e in eps}
    for key in keys:
        m = (D["scn"] == key[0]) & (D["ep"] == key[1]) & (D["qual"] == 1) & (D["age"] >= 2)
        best = -np.inf
        for e, d, age, cls in zip(D["e"][m], D["d"][m], D["age"][m], D["cls"][m]):
            s = score_row(float(e), float(d), int(age), str(cls), sh)
            if s is not None:
                best = max(best, s)
        out[key] = best                                   # empty flights: -inf, still counted in n
    return out


if __name__ == "__main__":
    D = load("designC")
    sh = fit_shapes(D)
    print("[shapes]", {f"{k[0][:4]}-{k[1]}": v for k, v in sh.items()})
    stability_gate(D, sh)
    blob = json.dumps({f"{k[0]}|{k[1]}": v for k, v in sh.items()}, sort_keys=True).encode()
    sh_hash = hashlib.sha256(blob).hexdigest()[:16]
    B = load("foldB7"); T = load("test7")
    SB, ST = flight_sups(B, sh), flight_sups(T, sh)
    n = len(SB)
    out = dict(provenance=dict(spec="lambda-SHAPE-H v3 (wf_18b3fdce-93a)", shape_hash=sh_hash,
                               n_flights=n, fold="B7/T7", date="2026-07-08"),
               groups={}, young={}, flags=[])
    vals = np.sort(np.array(list(SB.values())))
    for eps in (0.05, 0.10):
        k = int(np.ceil((n + 1) * (1 - eps)))
        if k > n:
            out["flags"].append(f"eps={eps}: UNDER_CALIBRATED n={n}"); continue
        lam = float(vals[k - 1])
        cov = float(np.mean([s <= lam for s in ST.values()]))
        print(f"[rank] eps={eps}: n={n} k={k} lambda={lam:.3f}  TEST7 coverage={cov:.3f} (n={len(ST)})"
              + ("  MAX_RANK_WARNING" if k == n else ""))
        eps_s = str(eps).rstrip("0").rstrip(".") if eps != 0.05 else "0.05"
        eps_s = {"0.05": "0.05", "0.1": "0.1"}[eps_s]
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
    # sup attribution table (abort-only discipline)
    att = sorted(((v, k) for k, v in SB.items()), reverse=True)[:10]
    json.dump([dict(sup=round(v, 3), scn=k[0], ep=k[1]) for v, k in att],
              open("out/conformal/v3_sup_attribution.json", "w"), indent=1)
    json.dump(out, open(os.path.join("..", "out", "conformal", "calib_v3.json"), "w"), indent=1)
    print("[calib] wrote ../out/conformal/calib_v3.json + sup attribution table")
