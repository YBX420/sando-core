"""calibrate_v4 — LCP-alpha calibration (theta6). Spec: docs/calib-spec-v4-LCP.md.

lambda-SHAPE-H generalization: the per-flight sup score is max_t alpha_t*e_t; v3 used an affine-
envelope-fitted alpha (2 params/class); v4 uses a 22-point alpha profile per mature class-arm,
initialized as per-Delta quantile normalization (alpha0_t = 1/q_t^{0.9}, fit on A_shape) and
refined on the DISJOINT A_opt split by coordinate descent on deployed area. Young/static arms are
unchanged from v3 (additive VCAP / flat). Rank: single shared lambda-hat on the fresh one-shot B8
fold; validation on T8; deployment radii r_t = lambda_hat/alpha_t emitted BOTH as the raw curve
(future two-segment cert) and as the affine upper envelope (today's C++ interface).
"""
import glob
import hashlib
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); os.chdir(HERE)
VCAP = {"pedestrian": 2.2, "vehicle": 11.0, "static": 0.0}
YNG = {"pedestrian": (0.35, 2.2, 0.16), "vehicle": (0.37, 11.0, 0.15), "static": (0.30, 0.0, 0.05)}
B_MIN = 0.05


def load(name):
    fs = sorted(glob.glob(f"out/conformal/harvest_{name}_v2_s*.npy")) or \
        [f"out/conformal/harvest_{name}_v2.npy"]
    return np.concatenate([np.load(f) for f in fs])


D = load("designC")
GRID = sorted({round(float(g), 2) for g in np.unique(np.round(D["d"], 2))})
scens = sorted(set(D["scn"].tolist()))
A_shape = np.isin(D["scn"], scens[0::2])
A_opt = np.isin(D["scn"], scens[1::2])          # scenario-disjoint (red line: opt never sees rank data)


def q_profile(mask, cls, q=0.90):
    """per-Delta quantile curve on masked design rows (mature arm)."""
    sel = mask & (D["cls"] == cls) & (D["age"] >= 4)
    if cls == "pedestrian":
        sel &= D["qual"] == 1
    out = {}
    for g in GRID:
        m = sel & (np.abs(D["d"] - g) < 0.01)
        if m.sum() >= 40:
            out[g] = max(float(np.quantile(D["e"][m], q)), B_MIN)
    return out


def flight_sups(DD, alphas):
    out = {}
    for key in {(str(s), int(e)) for s, e in zip(DD["scn"], DD["ep"])}:
        m0 = (DD["scn"] == key[0]) & (DD["ep"] == key[1]) & (DD["qual"] == 1) & (DD["age"] >= 2)
        best = -np.inf
        for e, d, age, cls in zip(DD["e"][m0], DD["d"][m0], DD["age"][m0], DD["cls"][m0]):
            cls = str(cls); d = round(float(d), 2)
            if cls == "static":
                by, _, _ = YNG["static"]
                s = float(e) / 0.27
            elif int(age) <= 3:
                by, vc, _ = YNG[cls]
                s = (float(e) - vc * d) / by
            else:
                a = alphas.get(cls, {}).get(d)
                if a is None:
                    continue
                s = a * float(e)
            best = max(best, s)
        out[key] = best
    return out


# ---- Stage 0: alpha profiles ------------------------------------------------------------------
alphas = {}
for cls in ("pedestrian", "vehicle"):
    prof = q_profile(A_shape, cls)
    alphas[cls] = {g: 1.0 / v for g, v in prof.items()}
print(f"[alpha] ped pts={len(alphas['pedestrian'])} veh pts={len(alphas['vehicle'])} "
      f"(init = per-Delta q90 normalization on A_shape)")

# coordinate-descent refinement on A_opt: minimize deployed area sum_t(lam(alpha)/alpha_t)
def deployed_area(al):
    sups = flight_sups(D[A_opt], al)
    n = len(sups)
    k = int(np.ceil((n + 1) * 0.90))
    if k > n:
        return None, 1e18
    lam = float(np.sort(list(sups.values()))[k - 1])
    area = sum(lam / a for cls in al for a in al[cls].values())
    return lam, area


lam0, area0 = deployed_area(alphas)
best_area = area0
for sweep in range(2):
    for cls in ("pedestrian", "vehicle"):
        for g in list(alphas[cls]):
            a0 = alphas[cls][g]
            for f in (0.8, 1.25):
                alphas[cls][g] = a0 * f
                _, ar = deployed_area(alphas)
                if ar < best_area - 1e-9:
                    best_area = ar
                    a0 = alphas[cls][g]
            alphas[cls][g] = a0
print(f"[opt] deployed-area {area0:.1f} -> {best_area:.1f} ({100*(1-best_area/area0):.1f}% tighter on A_opt)")

# ---- Stage 1: rank on B8, validate on T8 ------------------------------------------------------
B, T = load("foldB8"), load("test8")
SB, ST = flight_sups(B, alphas), flight_sups(T, alphas)
n = len(SB)
vals = np.sort(np.array(list(SB.values())))
out = dict(provenance=dict(spec="v4 LCP-alpha (theta6)", n_flights=n, fold="B8/T8",
                           alpha_hash=hashlib.sha256(json.dumps(
                               {c: {str(k): round(v, 6) for k, v in a.items()}
                                for c, a in alphas.items()}, sort_keys=True).encode()).hexdigest()[:16],
                           date="2026-07-08"),
           groups={}, young={}, alpha_curves={c: {str(k): round(v, 5) for k, v in a.items()}
                                              for c, a in alphas.items()})
for eps in (0.05, 0.10):
    k = int(np.ceil((n + 1) * (1 - eps)))
    if k > n:
        print(f"[rank] eps={eps}: UNDER n={n}"); continue
    lam = float(vals[k - 1])
    cov = float(np.mean([s <= lam for s in ST.values()]))
    print(f"[rank] eps={eps}: n={n} k={k} lambda={lam:.3f}  TEST8 coverage={cov:.3f} (n={len(ST)})"
          + ("  zero-slack" if k == n else ""))
    eps_s = "0.05" if eps == 0.05 else "0.1"
    for cls in ("pedestrian", "vehicle", "animal", "static"):
        g = out["groups"].setdefault(cls, {"levels": {}})
        if cls == "animal":
            g["levels"][eps_s] = dict(status="UNCALIBRATED"); continue
        if cls == "static":
            g["levels"][eps_s] = dict(q_conformal=round(lam * 0.27, 4), v_eff=0.0, status="ok")
        else:
            # affine upper envelope of the radius curve r_t = lam/alpha_t
            gs = sorted(alphas[cls])
            rs = [lam / alphas[cls][x] for x in gs]
            v = max(0.0, max((rs[i] - rs[0]) / (gs[i] - gs[0]) for i in range(1, len(gs))))
            q0 = max(r - v * x for r, x in zip(rs, gs))
            g["levels"][eps_s] = dict(q_conformal=round(q0, 4), v_eff=round(v, 4),
                                      status="ok", r_curve={str(x): round(r, 3) for x, r in zip(gs, rs)})
        by, gr, _ = YNG[cls if cls != "static" else "static"]
        out["young"].setdefault(cls, {})[eps_s] = dict(q0y=round(lam * 0.16 + by, 3) if cls != "static"
                                                       else round(lam * 0.05 + by, 3), growth=gr)
    if eps == 0.10:
        pl = out["groups"]["pedestrian"]["levels"]["0.1"]
        vl = out["groups"]["vehicle"]["levels"]["0.1"]
        print(f"[deploy] ped envelope q0={pl['q_conformal']} v={pl['v_eff']} | veh q0={vl['q_conformal']} v={vl['v_eff']}"
              f"  (v3: ped 0.50/1.63, veh 0.63/1.62)")
json.dump(out, open(os.path.join("..", "out", "conformal", "calib_v4.json"), "w"), indent=1)
print("[calib] wrote ../out/conformal/calib_v4.json")
