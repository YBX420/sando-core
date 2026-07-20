"""v6 finite-sample phase-1: three-rung honesty ladder on EXISTING harvests (no new flights).

Rung 1 (status quo):   designCR shapes; episode-rank over foldE flights; testE coverage.
Rung 2 (unit fixed):   designCR shapes; SCENARIO-rank over foldE (sup over episodes within
                       scenario); testE scenario coverage. Fold contamination remains (same 49
                       scenarios in every pool) -- labelled.
Rung 3 (both fixed):   pool ALL episodes, scenario-DISJOINT split fit/cal/test; refit shapes on
                       fit scenarios only; scenario-rank on cal; scenario coverage on test.

Each rung reports lambda, n/k (max-rank flag), ped/veh q~@0.3s, coverage.
"""
import os, sys
import numpy as np

MU = "/media/boxuan/Data2/projects/sando_py/sando-core/metaurban"
sys.path.insert(0, MU); os.chdir(MU)
import calibrate_v3 as V3
from calibrate_capsule import to_segment, fit_rear, rear_sups

EPS = (0.05, 0.10)


def load_pool(name, ep_off):
    D = V3.load(name).copy()
    D["ep"] = D["ep"] + ep_off
    return D


def joint_flight_sups(Draw, sh, rsh):
    """calibrate_capsule parity: capsule (esg) flight sups joined with the rear law sup."""
    S = V3.flight_sups(to_segment(Draw), sh)
    R = rear_sups(Draw, rsh)
    return {k: max(v, R.get(k, -np.inf)) for k, v in S.items()}


def scn_sups(fs):
    out = {}
    for (scn, _ep), v in fs.items():
        out[scn] = max(out.get(scn, -np.inf), v)
    return out


def rank(vals, eps):
    v = np.sort(np.asarray(list(vals), float))
    n = len(v)
    k = int(np.ceil((n + 1) * (1 - eps)))
    if k > n:
        return None, n, k
    return float(v[k - 1]), n, k


def report(tag, sups_cal, sups_test, sh, unit):
    print(f"\n=== {tag} (unit={unit}, n_cal={len(sups_cal)}, n_test={len(sups_test)}) ===")
    bp, vp = sh[("pedestrian", "M")]
    bv, vv = sh[("vehicle", "M")]
    for eps in EPS:
        lam, n, k = rank(sups_cal.values(), eps)
        if lam is None:
            print(f"eps={eps}: UNDER_CALIBRATED (needs k={k} > n={n})")
            continue
        cov = float(np.mean([s <= lam + 1e-9 for s in sups_test.values()])) if sups_test else float("nan")
        maxr = "  [MAX-RANK: zero slack]" if k == n else ""
        print(f"eps={eps}: n={n} k={k} lambda={lam:.3f}{maxr}")
        print(f"         ped-M q~@0.3s={lam * (bp + vp * 0.3):.3f}  veh-M q~@0.3s={lam * (bv + vv * 0.3):.3f}"
              f"  test-coverage={cov:.3f}")


DCR = load_pool("designCR", 0)
FE = load_pool("foldE", 100000)
TE = load_pool("testE", 200000)

# ---- rungs 1+2: shapes from designCR exactly as production did
sh12 = V3.fit_shapes(to_segment(DCR))
rsh12 = fit_rear(DCR)
cal_f = joint_flight_sups(FE, sh12, rsh12)
tst_f = joint_flight_sups(TE, sh12, rsh12)
report("RUNG-1 status quo (episode rank, shared scenarios)", cal_f, tst_f, sh12, "flight")
report("RUNG-2 unit fixed (scenario rank; folds still share scenarios -> coverage number is "
       "same-scenario-new-noise only)", scn_sups(cal_f), scn_sups(tst_f), sh12, "scenario")

# ---- rung 3: scenario-disjoint three-way split over the pooled data
ALL = np.concatenate([DCR, FE, TE])
scns = sorted(set(str(s) for s in ALL["scn"]))
rng = np.random.default_rng(17)
rng.shuffle(scns)
n_fit = max(8, int(len(scns) * 0.33))
n_tst = max(8, int(len(scns) * 0.25))
# STRATIFIED fit fold: a pure lottery starved sparse classes (static/vehicle rows cluster in few
# scenarios -> fit_shapes q90 grid empty). Force each class's two richest scenarios into fit,
# fill the rest by lottery. Test/cal stay untouched lottery.
force = []
for cls in ("pedestrian", "vehicle", "animal", "static"):
    cnt = {}
    for s in scns:
        cnt[s] = int(((ALL["scn"] == s) & (ALL["cls"] == cls)).sum())
    force += [s for s in sorted(cnt, key=cnt.get, reverse=True)[:2]]
force = list(dict.fromkeys(force))
rest = [s for s in scns if s not in force]
fit_s = set(force + rest[:max(0, n_fit - len(force))])
rest2 = [s for s in scns if s not in fit_s]
tst_s = set(rest2[-n_tst:]); cal_s = set(rest2[:-n_tst])
m_fit = np.isin(ALL["scn"], list(fit_s)); m_cal = np.isin(ALL["scn"], list(cal_s)); m_tst = np.isin(ALL["scn"], list(tst_s))
print(f"\n[rung3 split] fit={len(fit_s)} cal={len(cal_s)} test={len(tst_s)} scenarios "
      f"({m_fit.sum()}/{m_cal.sum()}/{m_tst.sum()} rows)")
sh3 = V3.fit_shapes(to_segment(ALL[m_fit]))
rsh3 = fit_rear(ALL[m_fit])
cal3 = joint_flight_sups(ALL[m_cal], sh3, rsh3)
tst3 = joint_flight_sups(ALL[m_tst], sh3, rsh3)
report("RUNG-3 both fixed (scenario rank, scenario-disjoint folds)", scn_sups(cal3), scn_sups(tst3), sh3, "scenario")

print("\n[reference] production calib_v6.json ped-M q~@0.3s:")
import json
cj = json.load(open("../out/conformal/calib_v6.json"))
for e in ("0.05", "0.1"):
    lv = cj["groups"]["pedestrian"]["levels"][e]
    print(f"  eps={e}: {lv['q_conformal'] + lv['v_eff'] * 0.3:.3f}")
