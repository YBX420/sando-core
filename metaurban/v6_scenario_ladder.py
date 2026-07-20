"""v6 finite-sample phase-1: three-rung honesty ladder on EXISTING harvests (no new flights).

Rung 1 (status quo):   designCR shapes; episode-rank over foldE flights; testE coverage.
Rung 2 (unit fixed):   designCR shapes; SCENARIO-rank over foldE (sup over episodes within
                       scenario); testE scenario coverage. Fold contamination remains (same 49
                       scenarios in every pool) -- a TUBE-WIDTH DIAGNOSTIC only, never a
                       calibration basis (07-20 hard condition #1).
Rung 3 (both fixed):   pool ALL episodes, scenario-DISJOINT split fit/cal/test with
                       PRE-REGISTERED strata from generator metadata (hard condition #2:
                       name family named/gen_pool, proportional random within stratum,
                       never chosen from harvested residuals/row counts); the conformal
                       rank uses a UNIFORM 2 episodes per scenario (hard condition #3:
                       identical exchangeable units; dropped episodes -> sensitivity only).
                       The eps=0.10 result is written as the INTERIM formal artifact
                       (out/conformal/calib_v6_scn010_interim.json) -- final calibration
                       waits for the new-scenario harvest + estimator freeze.

Finite-sample statements (hard condition #4): n_cal is the number of CAL SCENARIOS; eps=0.05
at n_cal<39 is max-rank (legal, zero tail slack, single-split unstable); observed test coverage
on ~12 scenarios has +-1-scenario granularity -- report as observation, not as a test verdict.
"""
import hashlib
import json
import os
import sys

import numpy as np

MU = "/media/boxuan/Data2/projects/sando_py/sando-core/metaurban"
sys.path.insert(0, MU); os.chdir(MU)
import calibrate_v3 as V3
from calibrate_capsule import to_segment, fit_rear, rear_sups, N_PEARLS

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
        maxr = "  [MAX-RANK: zero tail slack]" if k == n else ""
        print(f"eps={eps}: n={n} k={k} lambda={lam:.3f}{maxr}")
        print(f"         ped-M q~@0.3s={lam * (bp + vp * 0.3):.3f}  veh-M q~@0.3s={lam * (bv + vv * 0.3):.3f}"
              f"  observed-test-coverage={cov:.3f} (n_test={len(sups_test)}, +-1 scenario = "
              f"{1.0 / max(len(sups_test), 1):.3f})")


DCR = load_pool("designCR", 0)
FE = load_pool("foldE", 100000)
TE = load_pool("testE", 200000)

# ---- rungs 1+2: shapes from designCR exactly as production did (DIAGNOSTIC rungs)
sh12 = V3.fit_shapes(to_segment(DCR))
rsh12 = fit_rear(DCR)
cal_f = joint_flight_sups(FE, sh12, rsh12)
tst_f = joint_flight_sups(TE, sh12, rsh12)
report("RUNG-1 status quo (episode rank, shared scenarios)", cal_f, tst_f, sh12, "flight")
report("RUNG-2 unit-fix DIAGNOSTIC (scenario rank; folds still share scenarios -> tube-width "
       "equivalence check only, NOT a calibration basis)", scn_sups(cal_f), scn_sups(tst_f),
       sh12, "scenario")

# ---- rung 3: scenario-disjoint split, PRE-REGISTERED strata (generator name family)
ALL = np.concatenate([DCR, FE, TE])
scns = sorted(set(str(s) for s in ALL["scn"]))
rng = np.random.default_rng(17)
STATIC_SCNS = {"canyon_deadlock", "climb_trap", "fast_canyon", "occlusion_reveal", "pincer_v2",
               "props_alley"}
#   ^ pre-registered from the scenario MANIFESTS (scenario_lib load, statics count > 0): world
#     composition is generator metadata. Only 6/49 scenarios spawn statics, so an unstratified
#     lottery starves the static shape fit ~9% of the time (rng 17 hit it).
strata = {"named_static": [s for s in scns if s in STATIC_SCNS],
          "named": [s for s in scns if not s.startswith("gen_pool") and s not in STATIC_SCNS],
          "gen": [s for s in scns if s.startswith("gen_pool")]}
FRAC_FIT, FRAC_TST = 0.33, 0.25              # registered before looking at any residuals
fit_s, cal_s, tst_s = set(), set(), set()
for _name in sorted(strata):
    g = list(strata[_name]); rng.shuffle(g)
    n_f = max(2, int(round(len(g) * FRAC_FIT)))
    n_t = max(2, int(round(len(g) * FRAC_TST)))
    fit_s |= set(g[:n_f]); tst_s |= set(g[n_f:n_f + n_t]); cal_s |= set(g[n_f + n_t:])


def uniform_episodes(D, scn_set, k=2):
    """Hard condition #3: identical exchangeable units -- exactly k episodes per scenario (seeded
    shuffle), so old 9-episode scenarios and future 2-episode harvests rank the same object."""
    keep = np.zeros(len(D), bool)
    r2 = np.random.default_rng(23)
    for s in sorted(scn_set):
        eps_ids = sorted(set(int(e) for e in D["ep"][D["scn"] == s]))
        r2.shuffle(eps_ids)
        for e in eps_ids[:k]:
            keep |= (D["scn"] == s) & (D["ep"] == e)
    return D[keep]


D_fit = ALL[np.isin(ALL["scn"], list(fit_s))]          # shape fold: all episodes (not a rank)
D_cal = uniform_episodes(ALL[np.isin(ALL["scn"], list(cal_s))], cal_s, 2)
D_tst = uniform_episodes(ALL[np.isin(ALL["scn"], list(tst_s))], tst_s, 2)
print(f"\n[rung3 split] strata named={len(strata['named'])}/gen={len(strata['gen'])} -> "
      f"fit={len(fit_s)} cal={len(cal_s)} test={len(tst_s)} scenarios "
      f"({len(D_fit)}/{len(D_cal)}/{len(D_tst)} rows; cal/test capped at 2 ep/scenario)")
sh3 = V3.fit_shapes(to_segment(D_fit))
rsh3 = fit_rear(D_fit)
cal3 = joint_flight_sups(D_cal, sh3, rsh3)
tst3 = joint_flight_sups(D_tst, sh3, rsh3)
sc_cal, sc_tst = scn_sups(cal3), scn_sups(tst3)
report("RUNG-3 both fixed (scenario rank, scenario-disjoint pre-registered folds)",
       sc_cal, sc_tst, sh3, "scenario")
cal3_all = joint_flight_sups(ALL[np.isin(ALL["scn"], list(cal_s))], sh3, rsh3)
report("RUNG-3 sensitivity: ALL episodes per cal scenario (heterogeneous units -- diagnostics only)",
       scn_sups(cal3_all), sc_tst, sh3, "scenario")

# ---- INTERIM formal artifact at eps=0.10 (07-20 ruling: September tier; final calibration
#      re-runs after the new-scenario harvest + estimator freeze)
lam, n, k = rank(sc_cal.values(), 0.10)
if lam is not None:
    cov = float(np.mean([s <= lam + 1e-9 for s in sc_tst.values()]))
    out = dict(provenance=dict(
        spec="v6-scn INTERIM (scenario-level conformal, scenario-disjoint pre-registered folds)",
        ruling="2026-07-20: September main text = eps 0.10 from scenario-disjoint calibration; "
               "eps 0.05 deferred to camera-ready (needs >=39 cal scenarios for tail slack, "
               "recommended >=59)",
        split=dict(strata="name-family (named vs gen_pool_*), generator metadata",
                   frac_fit=FRAC_FIT, frac_test=FRAC_TST, rng="default_rng(17)",
                   episodes_per_scenario=2, episode_rng="default_rng(23)",
                   fit=sorted(fit_s), cal=sorted(cal_s), test=sorted(tst_s)),
        n_cal_scenarios=n, rank_k=k, lam=round(lam, 4),
        observed_test_coverage=round(cov, 4), n_test_scenarios=len(sc_tst),
        coverage_note="observation on ~12 scenarios (+-1 scenario granularity), not a verdict",
        status="INTERIM: pending new-scenario harvest + estimator freeze"),
        capsule=True, n_pearls=N_PEARLS, groups={}, young={}, rear={}, flags=["INTERIM"])
    for cls in ("pedestrian", "vehicle", "animal", "static"):
        g = out["groups"].setdefault(cls, {"levels": {}})
        if cls == "animal":
            g["levels"]["0.1"] = dict(status="UNCALIBRATED")
            continue
        b, v = sh3[(cls, "M")] if cls != "static" else sh3[("static", "ALL")]
        g["levels"]["0.1"] = dict(q_conformal=round(lam * b, 4), v_eff=round(lam * v, 4),
                                  status=("ok_zero_slack" if k == n else "ok"))
        if cls != "static":
            by, vc = sh3[(cls, "Y")]
            out["young"].setdefault(cls, {})["0.1"] = dict(q0y=round(lam * by, 3), growth=vc)
        if cls in rsh3:
            br, vr = rsh3[cls]
            out["rear"].setdefault(cls, {})["0.1"] = dict(q0r=round(lam * br, 4),
                                                          growth=round(lam * vr, 4))
    out["young"].setdefault("static", {})["0.1"] = dict(
        q0y=round(lam * sh3[("static", "ALL")][0], 3), growth=0.0)
    path = os.path.join("..", "out", "conformal", "calib_v6_scn010_interim.json")
    json.dump(out, open(path, "w"), indent=1)
    print(f"\n[artifact] INTERIM eps=0.10 scenario-conformal -> {path} "
          f"(lambda={lam:.3f}, k={k}/{n}, observed test coverage {cov:.3f})")

print("\n[reference] production calib_v6.json ped-M q~@0.3s:")
cj = json.load(open("../out/conformal/calib_v6.json"))
for e in ("0.05", "0.1"):
    lv = cj["groups"]["pedestrian"]["levels"][e]
    print(f"  eps={e}: {lv['q_conformal'] + lv['v_eff'] * 0.3:.3f}")
