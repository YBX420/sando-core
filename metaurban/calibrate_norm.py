"""calibrate_norm — sigma_kf-NORMALIZED conformal calibration (memo #7, the principled family).

Score r = e / sigma_kf(age, Delta): the analytic KF covariance propagation carries ALL the age- and
horizon-structure, so there is NO fitted shape (no b, no Theil-Sen v, no per-episode-sup sigma --
the estimator that regressed theta3 three times is simply gone). Guarantee: single conformal rank
over fold-B5 per-flight sups of r; deployment tube for a track of age a is q_hat*sigma_kf(a, t+delta),
shipped as per-age AFFINE plates (q0_a = q_hat*sigma(a,delta), v_a = q_hat*max-secant -- a sound
linear majorant of the sigma curve on the sample grid), consumed by the EXISTING plate machinery.
"""
import hashlib
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); os.chdir(HERE)
from b_bucket_recalibrate import sigma_kf
from conformal_calibrate import conformal_quantile

CFG = json.load(open("out/conformal/calib_v2_config.json"))
DELTA = 0.30
import glob as _g


def load_sharded(name):
    fs = sorted(_g.glob(f"out/conformal/harvest_{name}_v2_s*.npy")) or \
        [f"out/conformal/harvest_{name}_v2.npy"]
    return np.concatenate([np.load(f) for f in fs])


B = load_sharded("foldB5"); T = load_sharded("test5")
SIG = {}


def sig(age, d):
    k = (int(age), round(float(d), 2))
    if k not in SIG:
        SIG[k] = max(sigma_kf(int(age), float(d)), 0.02)
    return SIG[k]


def flight_sups(D):
    out = {}
    for key in {(str(s), int(e)) for s, e in zip(D["scn"], D["ep"])}:
        scn, epi = key
        m = (D["scn"] == scn) & (D["ep"] == epi) & (D["qual"] == 1) & (D["age"] >= 2)
        if not m.sum():
            continue
        r = np.array([e / sig(a, d) for e, a, d in zip(D["e"][m], D["age"][m], D["d"][m])])
        out[key] = float(np.max(r))
    return out


SB = flight_sups(B); ST = flight_sups(T)
n = len(SB)
res = dict(provenance=dict(spec="sigma_kf-normalized (memo#7)", config_sha=CFG["config_sha256"],
                           n_flights=n, date="2026-07-07"), groups={}, young={}, flags=[])
grid = np.arange(0.0, 1.06, 0.05)
for eps in (0.05, 0.10):
    k = int(np.ceil((n + 1) * (1 - eps)))
    if k > n:
        res["flags"].append(f"eps={eps}: UNDER_CALIBRATED n={n}")
        continue
    qhat = float(np.sort(list(SB.values()))[k - 1])
    cov = float(np.mean([s <= qhat for s in ST.values()])) if ST else float("nan")
    print(f"[rank] eps={eps}: n={n} k={k} qhat={qhat:.3f}  TEST5 flight coverage={cov:.3f} (n={len(ST)})")
    for cls in ("pedestrian", "vehicle", "animal", "static"):
        g = res["groups"].setdefault(cls, {"levels": {}})
        if cls == "animal":
            g["levels"][str(eps)] = dict(status="UNCALIBRATED")
            continue
        plates = {}
        for a in range(2, 31):
            if cls == "static":
                q0 = qhat * sig(a, 0.0); v = 0.0
            else:
                s0 = sig(a, DELTA)
                secants = [(sig(a, t + DELTA) - s0) / t for t in grid[1:]]
                q0 = qhat * s0; v = qhat * max(max(secants), 0.0)
            plates[f"a{a}"] = dict(q_conformal=round(q0, 4), v_eff=round(v, 4),
                                   age_lo=a, age_hi=(a if a < 30 else 999), coast=None)
        lv = plates["a8"]                                   # flat fallback = mid-age plate
        g["levels"][str(eps)] = dict(q_conformal=lv["q_conformal"], v_eff=lv["v_eff"],
                                     status="ok", plates=plates)
        res["young"].setdefault(cls, {})[str(eps)] = dict(
            q0y=plates["a2"]["q_conformal"], growth=plates["a2"]["v_eff"])
    if eps == 0.10:
        p = res["groups"]["pedestrian"]["levels"]["0.1"]["plates"]
        print("  ped 盘样例:", {k: (p[k]["q_conformal"], p[k]["v_eff"]) for k in ("a2", "a4", "a8", "a13", "a20")})
json.dump(res, open(os.path.join("..", "out", "conformal", "calib_norm.json"), "w"), indent=1)
print("[calib] wrote ../out/conformal/calib_norm.json")
