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


# ---- Stage 0: DIRECT AFFINE-SHAPE optimization (v4b) --------------------------------------------
# Envelope-aware coordinate descent over 22 free alphas stalls (envelope is piecewise-constant in
# single-point moves). Optimize the DEPLOYED representation directly instead: per mature class an
# affine shape (b, v), score e/(b+v*Delta), (b, v) chosen on the DISJOINT A_opt to minimize the
# joint deployed-envelope area. Same LCP spirit (parameterized score optimized held-out), zero
# representation mismatch.
shapes = {"pedestrian": (0.50, 1.63), "vehicle": (0.63, 1.62)}   # v3 envelopes as init


def alphas_of(sh):
    return {c: {g: 1.0 / max(b + v * g, B_MIN) for g in GRID} for c, (b, v) in sh.items()}


def area_of(sh):
    sups = flight_sups(D[A_opt], alphas_of(sh))
    n = len(sups)
    k = int(np.ceil((n + 1) * 0.90))
    if k > n:
        return 1e18, None
    lam = float(np.sort(list(sups.values()))[k - 1])
    T = GRID[-1]
    area = sum(lam * (b * T + 0.5 * v * T * T) for (b, v) in sh.values())
    return area, lam


base_area, _ = area_of(shapes)
for rnd in range(2):
    for cls in ("pedestrian", "vehicle"):
        b0, v0 = shapes[cls]
        best = (base_area, b0, v0)
        for b in np.linspace(max(0.1, b0 - 0.25), b0 + 0.25, 5):
            for v in np.linspace(max(0.3, v0 - 0.5), v0 + 0.5, 5):
                shapes[cls] = (float(b), float(v))
                ar, _ = area_of(shapes)
                if ar < best[0] - 1e-9:
                    best = (ar, float(b), float(v))
        base_area, shapes[cls] = best[0], (best[1], best[2])
        print(f"[opt] {cls}: (b,v)=({best[1]:.3f},{best[2]:.3f}) area={best[0]:.2f}")
alphas = alphas_of(shapes)
print(f"[opt] final shapes {shapes}")

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
