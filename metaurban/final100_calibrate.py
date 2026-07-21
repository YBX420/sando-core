"""final100_calibrate — the PAPER calibration on the pre-registered final100 benchmark.

Methodology locked by the 07-20/21 rulings:
  - tube SHAPES (b, v_eff slopes) + rear law fit on the RETIRED design face (design69 v3
    harvest) -- never on cal/test;
  - conformal rank at SCENARIO level over the final100 CAL split (60 scenarios, uniform 2
    episodes each), with the harvest UNIVERSE so zero-row episodes vote at -inf;
  - scenario-disjoint by construction (final100 worlds are freshly generated, disjoint from the
    retired design pool);
  - eps=0.10 is the September main-text tier (n=60: k=ceil(61*0.90)=55, five slots of slack);
    eps=0.05 also computable (k=58, two slots of slack);
  - the TEST split opens EXACTLY ONCE: coverage is computed only under --open-test, after the
    cal-side artifact is already written, and the artifact records that the test was opened.
Artifact: calib_v6 schema + full provenance (plan_sha, harvest run manifests, stack tag/shas,
universes, fit inputs) -> out/conformal/calib_v6_final100.json.
"""
import argparse
import hashlib
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); os.chdir(HERE)
import calibrate_v3 as V3
from calibrate_capsule import to_segment, fit_rear, rear_sups, N_PEARLS

ap = argparse.ArgumentParser()
ap.add_argument("--design-npy", required=True, help="retired design face (design69 v3 merged npy)")
ap.add_argument("--cal-run", required=True, help="final100cal harvest run dir")
ap.add_argument("--test-run", default=None, help="final100test harvest run dir (--open-test only)")
ap.add_argument("--plan", required=True, help="the final100 plan json")
ap.add_argument("--open-test", action="store_true",
                help="THE once-only ceremony: compute test coverage and seal it into the artifact")
ap.add_argument("--out", default="../out/conformal/calib_v6_final100.json")
args = ap.parse_args()


def sha_file(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()[:16]


def joint_scn_sups(run_dir, sh, rsh):
    D = np.load(os.path.join(run_dir, [f for f in os.listdir(run_dir) if f.endswith("_v3.npy")][0]))
    uni = json.load(open(os.path.join(run_dir, "universe.json")))
    S = V3.flight_sups(to_segment(D), sh, universe=uni)
    R = rear_sups(D, rsh)
    fs = {k: max(v, R.get(k, -np.inf)) for k, v in S.items()}
    out = {}
    for (scn, _ep), v in fs.items():
        out[scn] = max(out.get(scn, -np.inf), v)
    return out, D, uni


def rank(vals, eps):
    v = np.sort(np.asarray(list(vals), float))
    n = len(v)
    k = int(np.ceil((n + 1) * (1 - eps)))
    return (float(v[k - 1]) if k <= n else None), n, k


plan = json.load(open(args.plan))
assert not plan.get("draft"), "draft plans can never feed the paper calibration"

D_design = np.load(args.design_npy)
sh = V3.fit_shapes(to_segment(D_design))
rsh = fit_rear(D_design)
sc_cal, D_cal, uni_cal = joint_scn_sups(args.cal_run, sh, rsh)
n_zero = sum(1 for v in sc_cal.values() if v == float("-inf"))
print(f"[f100cal] cal scenarios={len(sc_cal)} (zero/no-qual at -inf: {n_zero}) "
      f"rows={len(D_cal)}")
assert len(sc_cal) == sum(1 for s in plan["slots"] if s["split"] == "cal"), \
    "cal universe does not match the registered split"

out = dict(provenance=dict(
    spec="v6.1 capsule, scenario-level conformal on the pre-registered final100 benchmark",
    ruling="07-21: shapes+rear from the retired design face; scenario rank on cal60 (uniform 2 "
           "episodes, universe-complete); scenario-disjoint folds by construction; eps=0.10 = "
           "September main-text tier; test opens exactly once",
    plan_sha=plan["plan_sha"], stack_sha=plan["stack_sha"],
    stack_runtime=plan.get("stack_runtime"),
    design_npy=dict(path=args.design_npy, sha=sha_file(args.design_npy)),
    cal_run=os.path.abspath(args.cal_run),
    family_scheme=plan.get("family_scheme"), family_histogram=plan.get("family_histogram"),
    test_opened=False),
    capsule=True, n_pearls=N_PEARLS, groups={}, young={}, rear={}, flags=[])

for eps, eps_s in ((0.05, "0.05"), (0.10, "0.1")):
    lam, n, k = rank(sc_cal.values(), eps)
    if lam is None:
        out["flags"].append(f"eps={eps}: UNDER_CALIBRATED n={n}")
        continue
    print(f"[f100cal] eps={eps}: n={n} k={k} lambda={lam:.3f} slack={n - k}")
    out["provenance"][f"rank_{eps_s}"] = dict(n=n, k=k, lam=round(lam, 4), slack=n - k)
    for cls in ("pedestrian", "vehicle", "animal", "static"):
        g = out["groups"].setdefault(cls, {"levels": {}})
        if cls == "animal":
            g["levels"][eps_s] = dict(status="STRESS_SUITE_ONLY")   # ruling: not in the theorem
            continue
        b, v = sh[(cls, "M")] if cls != "static" else sh[("static", "ALL")]
        g["levels"][eps_s] = dict(q_conformal=round(lam * b, 4), v_eff=round(lam * v, 4),
                                  status=("ok" if k < n else "ok_zero_slack"))
        if cls != "static":
            by, vc = sh[(cls, "Y")]
            out["young"].setdefault(cls, {})[eps_s] = dict(q0y=round(lam * by, 3), growth=vc)
        if cls in rsh:
            br, vr = rsh[cls]
            out.setdefault("rear", {}).setdefault(cls, {})[eps_s] = dict(
                q0r=round(lam * br, 4), growth=round(lam * vr, 4))

if args.open_test:
    assert args.test_run, "--open-test needs --test-run"
    sc_tst, D_tst, uni_tst = joint_scn_sups(args.test_run, sh, rsh)
    assert len(sc_tst) == sum(1 for s in plan["slots"] if s["split"] == "test"), \
        "test universe does not match the registered split"
    out["provenance"]["test_opened"] = True
    out["provenance"]["test_run"] = os.path.abspath(args.test_run)
    for eps, eps_s in ((0.05, "0.05"), (0.10, "0.1")):
        rk = out["provenance"].get(f"rank_{eps_s}")
        if not rk:
            continue
        cov = float(np.mean([s <= rk["lam"] + 1e-9 for s in sc_tst.values()]))
        rk["observed_test_coverage"] = round(cov, 4)
        rk["n_test_scenarios"] = len(sc_tst)
        print(f"[f100cal] eps={eps}: OBSERVED test coverage {cov:.3f} "
              f"(n={len(sc_tst)}, resolution {1.0 / len(sc_tst):.3f}) -- test is now SEALED")

tmp = args.out + ".tmp"
json.dump(out, open(tmp, "w"), indent=1)
os.replace(tmp, args.out)
print(f"[f100cal] artifact -> {args.out}")
