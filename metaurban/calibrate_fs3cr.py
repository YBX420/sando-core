"""calibrate_fs3cr — FS3C-R theta2 calibration (spec: docs/calib-spec-FS3C-R.md).

Fold-A (design domain, shapes only): harvest_vehA_v2 + retired harvest_foldB/test_v2 (+ static
frozen-law rows live there). Fold-B2 (quantile, one flight/scenario, fresh pre-registered seeds):
single shared rank over per-flight sup of class-normalized excess on FUTURE-REACH-qualified rows.
TEST2: per-flight coverage validation + Mondrian per-class diagnostic + deployability report.
Fail-loud: UNDER_CALIBRATED / UNCALIBRATED explicit, no +inf, no silent zeros.
"""
import hashlib
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); os.chdir(HERE)
CFG = json.load(open("out/conformal/calib_v2_config.json"))
_chk = dict(CFG); _sha = _chk.pop("config_sha256")
assert hashlib.sha256(json.dumps(_chk, sort_keys=True).encode()).hexdigest()[:16] == _sha, "config tampered"
GRID = np.array(CFG["GRID"]); H = CFG["H"]; AGE_MIN = CFG["AGE_MIN"]
CLASSES = CFG["CLASSES"]; VCAP = CFG["VCAP_TRUE"]; SIG_FLOOR = CFG["SIG_FLOOR"]

def load(name, fields8=False):
    d = np.load(f"out/conformal/harvest_{name}_v2.npy")
    if "qual" not in d.dtype.names:                     # pre-theta2 design harvests lack the flag;
        out = np.empty(len(d), dtype=d.dtype.descr + [("qual", "i4")])
        for f in d.dtype.names:                         # shapes never read qual -> pad with 1
            out[f] = d[f]
        out["qual"] = 1
        d = out
    return d

# design domain = vehA + ALL retired folds (spec: anything not the live quantile/test folds)
_design = ["vehA", "foldB", "test"]
if os.path.exists("out/conformal/harvest_foldB4_v2.npy"):
    _live_B, _live_T = "foldB4", "test4"
    _design += ["foldB2", "test2", "foldB3", "test3"]
elif os.path.exists("out/conformal/harvest_foldB3_v2.npy"):
    _live_B, _live_T = "foldB3", "test3"
    _design += ["foldB2", "test2"]
else:
    _live_B, _live_T = "foldB2", "test2"
A = [load(n) for n in _design]
B2 = load(_live_B); T2 = load(_live_T)
print(f"[folds] live quantile={_live_B} test={_live_T}  design={_design}")
assert "qual" in B2.dtype.names and "qual" in T2.dtype.names

def rows_of(ds, cls, mature=None):
    out = []
    for d in ds:
        m = d["cls"] == cls
        if mature is True:
            m &= d["age"] >= AGE_MIN
        elif mature is False:
            m &= (d["age"] >= 2) & (d["age"] < AGE_MIN)
        out.append(d[m])
    return np.concatenate(out) if out else np.array([])

def theil_sen(xs, ys):
    sl = [(ys[j] - ys[i]) / (xs[j] - xs[i]) for i in range(len(xs)) for j in range(i + 1, len(xs))
          if xs[j] > xs[i]]
    return float(np.median(sl)) if sl else 0.0

def shape(cls, mature=True, force_slope=None):
    """(b, v, sigma, n_rows) from design-domain rows; q90(Delta) envelope + Theil-Sen slope."""
    r = rows_of(A, cls, mature)
    if len(r) < 200:
        return None
    ds = sorted(set(np.round(r["d"], 2).tolist()))
    q90 = []
    for x in ds:
        m = np.abs(r["d"] - x) < 1e-6
        if m.sum() < 20:
            continue
        q90.append((x, float(np.quantile(r["e"][m], CFG["Q_LEVEL"]))))
    if len(q90) < 2:
        return None
    xs, ys = zip(*q90)
    v = force_slope if force_slope is not None else float(np.clip(theil_sen(list(xs), list(ys)), 0.0, CFG["V_SLOPE_CAP"]))
    b = float(max(y - v * x for x, y in q90))            # envelope intercept dominates the grid
    # sigma: per-episode sup-excess spread (q90-q50) on design rows
    ex = r["e"] - v * r["d"] - b
    per_ep = [float(np.max(ex[r["ep"] == e])) for e in np.unique(r["ep"])]
    sig = float(max(SIG_FLOOR, np.quantile(per_ep, 0.9) - np.quantile(per_ep, 0.5)))
    return dict(b=b, v=v, sigma=sig, n=len(r))

shapes, young = {}, {}
for c in CLASSES:
    if c == "animal":
        continue
    s = shape(c, mature=True, force_slope=(0.0 if c == "static" else None))
    if s: shapes[c] = s
    y = shape(c, mature=False, force_slope=0.0)          # young arm: frozen residual, deterministic growth
    if y: young[c] = y
print("[shapes]", {k: {kk: round(vv, 3) for kk, vv in v.items()} for k, v in shapes.items()})
print("[young ]", {k: {kk: round(vv, 3) for kk, vv in v.items()} for k, v in young.items()})

def flight_scores(D):
    """per-FLIGHT score S = sup over QUALIFIED rows of normalized excess (both arms).
    Key = (scn, ep): fold-B2 has one flight per scenario; TEST2 has two -- merging them (old bug)
    changed the exchangeable unit."""
    out = {}
    for key in {(str(s), int(e)) for s, e in zip(D["scn"], D["ep"])}:
        scn, epi = key
        m0 = (D["scn"] == scn) & (D["ep"] == epi)
        best, any_row = -1e18, False
        for c in set(D["cls"][m0].tolist()):
            if c not in shapes:
                continue
            for arm, tab, growth in (("M", shapes, None), ("Y", young, None)):
                sh = tab.get(c)
                if sh is None:
                    continue
                m = m0 & (D["cls"] == c) & (D["qual"] == 1)
                m &= (D["age"] >= AGE_MIN) if arm == "M" else ((D["age"] >= 2) & (D["age"] < AGE_MIN))
                if not m.sum():
                    continue
                gr = sh["v"] if arm == "M" else VCAP.get(c, 0.0)
                rsc = (D["e"][m] - gr * D["d"][m] - sh["b"]) / sh["sigma"]
                best = max(best, float(np.max(rsc))); any_row = True
        if any_row:
            out[key] = best
    return out

SB = flight_scores(B2)
n = len(SB)
res = dict(provenance=dict(spec="FS3C-R theta2", config_sha=_sha, n_foldB2_flights=n,
                           date="2026-07-07"), groups={}, young={}, flags=[])
print(f"[foldB2] scoring flights n={n} (scenarios with qualified rows)")
for eps in (0.05, 0.10):
    k = int(np.ceil((n + 1) * (1 - eps)))
    if k > n:
        qhat, flag = float(np.max(list(SB.values()))), "UNDER_CALIBRATED"
        res["flags"].append(f"eps={eps}: n={n} rank {k}>n -> qhat=max, achieved eps={1/(n+1):.3f}")
    else:
        qhat = float(np.sort(list(SB.values()))[k - 1])
        flag = "ok_zero_slack" if k == n else "ok"       # k==n is a VALID rank (guarantee holds);
        #   zero slack just means qhat rides the sample max -- fragile to one new tail flight
    for c in CLASSES:
        g = res["groups"].setdefault(c, {"levels": {}})
        if c == "animal" or c not in shapes:
            g["levels"][str(eps)] = dict(status="UNCALIBRATED")
            continue
        sh = shapes[c]
        eta = (VCAP.get(c, 0.0) + CFG["VCAP_PRED"].get(c, 0.0) + sh["v"]) * H / 2
        g["levels"][str(eps)] = dict(q_conformal=round(sh["b"] + qhat * sh["sigma"] + eta, 4),
                                     v_eff=round(sh["v"], 4), status=flag)
        if c in young:
            yy = young[c]
            res["young"].setdefault(c, {})[str(eps)] = dict(
                q0y=round(yy["b"] + qhat * yy["sigma"] + (2 * VCAP.get(c, 0.0)) * H / 2, 4),
                growth=VCAP.get(c, 0.0))
    print(f"[rank] eps={eps}: n={n} k={k} qhat={qhat:.3f} ({flag})")

# ---- TEST2 validation ----
ST = flight_scores(T2)
for eps in (0.05, 0.10):
    k = int(np.ceil((n + 1) * (1 - eps)))
    qh = float(np.max(list(SB.values()))) if k > n else float(np.sort(list(SB.values()))[k - 1])
    cov = float(np.mean([s <= qh for s in ST.values()])) if ST else float("nan")
    # Mondrian diagnostic: flights containing each class
    mond = {}
    for c in ("pedestrian", "vehicle", "static"):
        f_c = [s for (scn, epi), s in ST.items()
               if c in set(T2["cls"][(T2["scn"] == scn) & (T2["ep"] == epi)].tolist())]
        mond[c] = (round(float(np.mean([s <= qh for s in f_c])), 3), len(f_c)) if f_c else (None, 0)
    print(f"[TEST2] eps={eps}: flight coverage={cov:.3f} (n={len(ST)})  Mondrian={mond}")
    res.setdefault("validation", {})[str(eps)] = dict(test2_flight_coverage=round(cov, 4),
                                                      n_test2=len(ST), mondrian=str(mond))

# ---- deployability ----
dep = {}
for c, sh in shapes.items():
    lv = res["groups"][c]["levels"]["0.05"]
    if "q_conformal" in lv:
        q0, v = lv["q_conformal"], lv["v_eff"]
        dep[c] = {f"tube@{t}": round(q0 + v * t, 2) for t in (0.3, 0.75, 1.05)}
print("[deployability]", dep)
res["deployability"] = dep

json.dump(res, open(os.path.join("..", "out", "conformal", "calib_v2.json"), "w"), indent=1)
print("[calib] wrote ../out/conformal/calib_v2.json (SL canonical dir)")
