"""b_bucket_recalibrate — B-bucket CRITICAL#1: recalibrate the conformal keep-out against the
FROZEN deployment (realistic perception front-end, deployment-in-the-loop residuals).

What the old calibration got wrong (2026-06-27/07-02 audits):
  * calibration KF ran at DT=0.30/MEAS=0.07 vs deployment 0.10/0.10  -> age buckets meant different
    wall-clock; q values not transferable;
  * calibration fed DENSE GT+noise detections vs deployment's intermittent cone/occlusion/miss
    detections with NN association  -> condition-law drift;
  * quantiles pooled per-instance (temporally correlated)             -> optimistic n.

This script fixes all three at once by harvesting residuals FROM THE DEPLOYED PIPELINE ITSELF:
run the frozen evaluation (replay_core + perception.py, PERCEPT=realistic default) over the whole
scenario library x sensor seeds, with a hook recording, for every READY deployed track and horizon
Delta: || nearest-GT-mover future  -  track prediction ||  (association error included -- that IS
deployment). Then:
  1. per-class affine tubes  q0 + v_eff*Delta  via split conformal, split BY EPISODE (exchangeable
     unit; per-instance pooled numbers are also reported for continuity, labelled OPTIMISTIC);
  2. age-conditional coverage of the class tube (the fresh-track hole, on honest conditions);
  3. the NORMALIZED-conformal baseline (score = e / sigma_KF(Delta)): if the KF's own covariance
     already explains the age effect, age-conditional q is DEAD as a contribution (the make-or-break
     experiment the reviewers will demand);
  4. writes out/conformal/calib_realistic.json (with provenance) -- deployment reads calib.json, so
     promotion is an explicit copy, never silent.

Run (sando python):  python3 b_bucket_recalibrate.py [--quick]
"""
import argparse
import glob
import json
import os
import time

import numpy as np

import replay_core as RC
import scenario_lib as SLB
from conformal_calibrate import conformal_quantile, fit_affine_envelope

HERE = os.path.dirname(os.path.abspath(__file__))
OUTD = os.path.join(HERE, "out", "conformal")
DELTAS = list(RC._HARV_DELTAS)
EPS_LEVELS = [0.20, 0.10, 0.05]
AGE_BUCKETS = [(2, 3), (4, 6), (7, 12), (13, 10 ** 9)]    # trk.n buckets (ready starts at n=2)


def sigma_kf(age_n, delta, dt=0.30, meas=0.10):
    """Predicted position sigma of the deployed CA-KF after n updates, extrapolated by Delta.
    Analytic propagation of the same filter (kf_tracker) on a synthetic stream -- gives the
    normalization scale WITHOUT touching per-track state. Cached by (n, delta)."""
    key = (int(age_n), round(float(delta), 2))
    if key in _SIG_CACHE:
        return _SIG_CACHE[key]
    from kf_tracker import _AxisCAKalman
    f = _AxisCAKalman(dt, meas)
    for _ in range(int(age_n)):
        f.update(0.0)
    F = np.array([[1, delta, delta * delta / 2], [0, 1, delta], [0, 0, 1]])
    P = F @ f.P @ F.T
    s = float(np.sqrt(max(P[0, 0], 1e-12) * 2.0))          # two axes, iid
    _SIG_CACHE[key] = s
    return s


_SIG_CACHE = {}


EP_SCN = {}


def harvest(quick=False):
    RC.PERCEPT_HARVEST = []
    EP_SCN.clear()
    scns = sorted(glob.glob(os.path.join(HERE, "scenarios", "*.json"))) + \
        sorted(glob.glob(os.path.join(HERE, "scenarios", "bench", "street_*.json")))
    seeds = (1234567, 7654321) if quick else tuple([1234567, 7654321, 24680, 13579, 999331] +
                                                    [86420 + 311 * k for k in range(5)])
    VEH_HEAVY = ("vehicle_spawn_accel", "fast_overtake", "roundabout_rush", "street_")
    extra = tuple(4242 + 97 * k for k in range(12))        # vehicle statistics are starved (fast +
    #   small cone + spacing rules push cars away) -> vehicle-rich scenarios get 12 extra sensor seeds
    t0 = time.time()
    n_ep = 0
    for f in scns:
        try:
            scn = SLB.load(f)
        except Exception:
            continue
        movers = SLB.apply_rh_overrides(RC.Movers(SLB.to_movers_raw(scn)), scn)
        ep = SLB.to_episode(scn)
        sds = seeds + (extra if any(k in f for k in VEH_HEAVY) and not quick else ())
        for sd in sds:
            os.environ["PERCEPT_SEED"] = str(sd)
            try:
                RC.run_replay(movers, ep, mode="ours", record=False,
                              max_vel=float(scn["drone"].get("max_vel", 3.0)))
                EP_SCN[int(RC._HARV_EP[0])] = os.path.basename(f)   # episode -> scenario (cluster unit)
                n_ep += 1
            except Exception as e:
                print(f"[harvest] {os.path.basename(f)} seed {sd}: {type(e).__name__} {e}")
    os.environ.pop("PERCEPT_SEED", None)
    data = np.array(RC.PERCEPT_HARVEST,
                    dtype=[("d", "f4"), ("e", "f4"), ("age", "i4"), ("cls", "U12"), ("ep", "i4"),
                           ("dd", "f4")])
    RC.PERCEPT_HARVEST = None
    print(f"[harvest] {len(data)} residuals from {n_ep} episodes in {time.time()-t0:.0f}s")
    return data


def _split_eps(data, frac=0.6, seed=17):
    eps_ids = np.unique(data["ep"])
    rng = np.random.default_rng(seed)
    rng.shuffle(eps_ids)
    k = max(1, int(len(eps_ids) * frac))
    return set(eps_ids[:k].tolist()), set(eps_ids[k:].tolist())


def _tube_and_coverage(cal_rows, test_rows, normalize=False):
    """Affine tube on (possibly normalized) scores; per-episode-sup AND pooled units both reported."""
    out = {}
    for eps in EPS_LEVELS:
        qhat_ep, qhat_pool = [], []
        for d in DELTAS:
            m = np.abs(cal_rows["d"] - d) < 1e-6
            sc = cal_rows["e"][m] / (np.array([sigma_kf(a, d) for a in cal_rows["age"][m]])
                                     if normalize else 1.0)
            qhat_pool.append(conformal_quantile(sc, eps))
            per_ep = [np.max(sc[cal_rows["ep"][m] == e]) for e in np.unique(cal_rows["ep"][m])]
            qhat_ep.append(conformal_quantile(per_ep, eps))
        rep = {}
        for label, qh in (("episode_sup", qhat_ep), ("pooled_OPTIMISTIC", qhat_pool)):
            q0, sl = fit_affine_envelope(DELTAS, qh)
            cov_n = cov_d = 0
            for d in DELTAS:
                m = np.abs(test_rows["d"] - d) < 1e-6
                sc = test_rows["e"][m] / (np.array([sigma_kf(a, d) for a in test_rows["age"][m]])
                                          if normalize else 1.0)
                cov_n += int(np.sum(sc <= q0 + sl * d + 1e-12)); cov_d += len(sc)
            rep[label] = dict(q_conformal=round(q0, 4), v_eff=round(sl, 4),
                              coverage=round(cov_n / max(1, cov_d), 4), n_test=cov_d)
        out[str(eps)] = rep
    return out


def age_coverage(rows, q0, sl, normalize=False):
    """Coverage of the class tube per age bucket -- the fresh-track hole, on honest conditions."""
    out = {}
    for (lo, hi) in AGE_BUCKETS:
        m = (rows["age"] >= lo) & (rows["age"] <= hi)
        if not m.any():
            continue
        sc = rows["e"][m] / (np.array([sigma_kf(a, d) for a, d in zip(rows["age"][m], rows["d"][m])])
                             if normalize else 1.0)
        tube = q0 + sl * rows["d"][m]
        out[f"age{lo}-{'inf' if hi > 999 else hi}"] = dict(
            coverage=round(float(np.mean(sc <= tube + 1e-12)), 4), n=int(m.sum()))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="2 sensor seeds instead of 5")
    args = ap.parse_args()
    os.makedirs(OUTD, exist_ok=True)
    data = harvest(quick=args.quick)
    np.save(os.path.join(OUTD, "residuals_realistic.npy"), data)
    json.dump({str(k): v for k, v in EP_SCN.items()}, open(os.path.join(OUTD, "ep_scn.json"), "w"))
    cal_ids, test_ids = _split_eps(data)
    cal = data[np.isin(data["ep"], list(cal_ids))]
    test = data[np.isin(data["ep"], list(test_ids))]
    report = dict(provenance=dict(
        date="2026-07-03", pipeline="deployment-in-the-loop (replay_core+perception PERCEPT=realistic)",
        deltas=DELTAS, n_residuals=int(len(data)), n_episodes=int(len(np.unique(data["ep"]))),
        split="by EPISODE (exchangeable unit)", pred_model=RC.PRED_MODEL,
        percept=dict(fov_deg=45, fov_range=10, sigma0=0.05, sigma_k=0.01,
                     p_miss0=0.05, p_miss_k=0.15, occlusion=True)), groups={})
    for cls in ("all", "pedestrian", "vehicle", "animal"):
        rows_c = data if cls == "all" else data[data["cls"] == cls]
        cal_c = cal if cls == "all" else cal[cal["cls"] == cls]
        test_c = test if cls == "all" else test[test["cls"] == cls]
        if len(cal_c) < 50:
            continue
        levels = _tube_and_coverage(cal_c, test_c)
        levels_norm = _tube_and_coverage(cal_c, test_c, normalize=True)
        g = dict(levels=levels, levels_normalized=levels_norm)
        # age analysis at eps=0.05, episode_sup tube; plain vs normalized (the make-or-break)
        t5 = levels["0.05"]["episode_sup"]; t5n = levels_norm["0.05"]["episode_sup"]
        g["age_coverage_plain"] = age_coverage(test_c, t5["q_conformal"], t5["v_eff"])
        g["age_coverage_normalized"] = age_coverage(test_c, t5n["q_conformal"], t5n["v_eff"],
                                                    normalize=True)
        report["groups"][cls] = g
    out = os.path.join(OUTD, "calib_realistic.json")
    json.dump(report, open(out, "w"), indent=1)
    print(f"[recal] wrote {out}")
    for cls, g in report["groups"].items():
        t = g["levels"]["0.05"]["episode_sup"]
        print(f"  {cls:11s} eps.05 episode_sup: q={t['q_conformal']:.3f} v_eff={t['v_eff']:.3f} "
              f"cov={t['coverage']:.3f}")
        print(f"    age(plain): { {k: v['coverage'] for k, v in g['age_coverage_plain'].items()} }")
        print(f"    age(norm) : { {k: v['coverage'] for k, v in g['age_coverage_normalized'].items()} }")


if __name__ == "__main__":
    main()
