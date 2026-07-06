"""phase5_compose — the end-to-end COMPOSITION THEOREM, instantiated on deployment-in-the-loop data.

Theorem (episode-level, split conformal). Fix a tube shape T(D) = q0 + v*D. Define the per-episode
BINDING-SUP score
    S_ep = sup { e_i - v*D_i : rows i of episode ep with dd_i <= D_bind(cls_i) }
over the residual rows harvested FROM THE DEPLOYED PIPELINE (perception, association, coast all
included; sup = -inf if no binding row). Let q0 = the (1-eps) split-conformal quantile of
{S_1..S_n} over calibration episodes. Then for a fresh exchangeable episode
    P( EVERY binding prediction error stays inside T ) >= 1 - eps,
and since a collision on a certified tick REQUIRES a binding-row exceedance within the trust
window (a mover farther than D_bind = (v_drone + v_cls)*(TAU+DELTA_max) + R_keepout cannot reach
the drone inside the window), we get the end-to-end statement
    P( collision anywhere in the episode | certified flight, calibrated conditions ) <= eps.

Why this beats the naive global sup (the B-bucket 3.3 m tube): rows OUTSIDE the binding region --
a walker turning a corner 9 m away -- inflate the naive sup but can never cause a collision. The
theorem's sup only runs where physics allows an accident.

Assumptions (stated, checkable): episodes exchangeable with calibration (same frozen generator +
sensor law); certified ticks only (evade is uncertified and carries no guarantee -- reported
separately as RTA exposure); D_bind uses the class speed caps enforced by the generator.

Run: python3 phase5_compose.py            (expects out/conformal/residuals_realistic.npy w/ dd)
"""
import json
import os

import numpy as np

from conformal_calibrate import conformal_quantile

HERE = os.path.dirname(os.path.abspath(__file__))
OUTD = os.path.join(HERE, "out", "conformal")
EPS = [0.10, 0.05]
TAU, DMAX, VDRONE = 0.75, 0.85, 3.0
VCLS = dict(pedestrian=2.6, vehicle=9.0, animal=3.0, static=0.0)
RKEEP = 1.6                                     # r_obs + body + d_safe + q headroom (conservative)


def d_bind(cls, window=TAU + DMAX):
    return (VDRONE + VCLS.get(str(cls), 3.0)) * window + RKEEP


def sup_scores(rows, v_eff, binding=True, window=None):
    """per-episode sup of e - v_eff*Delta over (binding) rows; episodes with no rows -> skipped
    (an episode with no binding row trivially satisfies the tube). window: CADENCE-AWARE branch --
    when the cert re-runs every DT, a collision is always within DT+latency of the LAST certified
    tick, so only rows with Delta <= window (and correspondingly tighter D_bind) can be the
    exceedance that kills you; the rare stale-spline branch is accounted separately."""
    out = {}
    m = np.ones(len(rows), bool)
    if window is not None:
        m &= rows["d"] <= window + 1e-6
    if binding:
        w = window if window is not None else TAU + DMAX
        m &= rows["dd"] <= np.array([d_bind(c, w) for c in rows["cls"]])
    r = rows[m]
    for ep in np.unique(r["ep"]):
        rr = r[r["ep"] == ep]
        out[int(ep)] = float(np.max(rr["e"] - v_eff * rr["d"]))
    return out


def main():
    data = np.load(os.path.join(OUTD, "residuals_realistic.npy"))
    assert "dd" in data.dtype.names, "re-run b_bucket_recalibrate.py first (needs dd field)"
    # THREE-WAY split BY SCENARIO (episodes cluster within scenarios -- splitting by episode leaks
    # scenario geometry across cal/test and under-covers ~3 sigma; scenario is the exchangeable unit):
    # shape scenarios fit v_eff ONLY; cal scenarios give q0; test scenarios measure coverage.
    ep_scn = {int(k): v for k, v in json.load(
        open(os.path.join(OUTD, "ep_scn.json"))).items()}
    scns = sorted(set(ep_scn.values()))
    rng = np.random.default_rng(17)
    rng.shuffle(scns)
    k1, k2 = int(len(scns) * 0.2), int(len(scns) * 0.65)
    grp = {sc: ("shape" if i < k1 else "cal" if i < k2 else "test") for i, sc in enumerate(scns)}
    eps_ids = np.unique(data["ep"])
    shape_ids = {e for e in eps_ids if grp.get(ep_scn.get(int(e), "?"), "cal") == "shape"}
    cal_ids = {e for e in eps_ids if grp.get(ep_scn.get(int(e), "?"), "cal") == "cal"}
    test_ids = {e for e in eps_ids if grp.get(ep_scn.get(int(e), "?"), "cal") == "test"}
    shp = data[np.isin(data["ep"], list(shape_ids))]
    cal = data[np.isin(data["ep"], list(cal_ids))]
    test = data[np.isin(data["ep"], list(test_ids))]
    from conformal_calibrate import fit_affine_envelope
    ds = sorted(set(np.round(data["d"], 2)))
    qs = [np.quantile(shp["e"][np.abs(shp["d"] - d) < 1e-6], 0.9) for d in ds]
    _, v_eff = fit_affine_envelope(ds, qs)
    report = dict(provenance=dict(date="2026-07-03", theorem="episode binding-sup composition",
                                  v_eff_shape=round(float(v_eff), 4),
                                  d_bind={c: round(d_bind(c), 2) for c in VCLS},
                                  n_cal_ep=len(cal_ids), n_test_ep=len(test_ids)), levels={})
    print(f"tube shape v_eff={v_eff:.3f};  D_bind: " +
          ", ".join(f"{c}={d_bind(c):.1f}m" for c in ("pedestrian", "vehicle")))
    for eps in EPS:
        row = {}
        for label, binding, win in (("cadence_sup_W0.4", True, 0.40),
                                    ("binding_sup", True, None),
                                    ("naive_global_sup", False, None)):
            sc = sup_scores(cal, v_eff, binding, window=win)
            q0 = conformal_quantile(list(sc.values()), eps)
            # test: fraction of held-out episodes where EVERY (binding) row stays inside the tube
            ts = sup_scores(test, v_eff, binding, window=win)
            cov = float(np.mean([s <= q0 + 1e-9 for s in ts.values()])) if ts else 1.0
            wref = win if win is not None else 0.85
            row[label] = dict(q0=round(float(q0), 3), window=wref,
                              tube_at_window=round(float(q0 + v_eff * wref), 2),
                              episode_coverage=round(cov, 4), n_cal_ep=len(sc), n_test_ep=len(ts))
            print(f"eps={eps} {label:18s}: q0={q0:.3f}  tube@{wref:.2f}s={q0 + v_eff*wref:.2f}m  "
                  f"ep-coverage={cov:.3f} ({len(ts)} test eps)")
        # PER-CLASS theorem tubes: collision with ANY class -- allocate eps equally across classes
        # present; per-class cadence sup gives each class its own q0 (pedestrian should be far
        # tighter than the vehicle-dominated single tube -> buys back reach).
        clss = [c for c in ("pedestrian", "vehicle", "animal") if (cal["cls"] == c).sum() > 200]
        eps_c = eps / max(1, len(clss))
        for c in clss:
            sc = sup_scores(cal[cal["cls"] == c], v_eff, True, window=0.40)
            q0c = conformal_quantile(list(sc.values()), eps_c)
            tsc = sup_scores(test[test["cls"] == c], v_eff, True, window=0.40)
            covc = float(np.mean([x <= q0c + 1e-9 for x in tsc.values()])) if tsc else 1.0
            row[f"perclass_{c}"] = dict(q0=round(float(q0c), 3), eps_alloc=round(eps_c, 4),
                                        tube_at_040=round(float(q0c + v_eff * 0.40), 2),
                                        episode_coverage=round(covc, 4), n_cal_ep=len(sc))
            print(f"eps={eps} perclass {c:10s}: q0={q0c:.3f} (eps_c={eps_c:.3f}) "
                  f"tube@0.4s={q0c + v_eff*0.40:.2f}m  ep-cov={covc:.3f}")
        report["levels"][str(eps)] = row
    out = os.path.join(OUTD, "compose_theorem.json")
    json.dump(report, open(out, "w"), indent=1)
    print(f"[phase5] wrote {out}")


if __name__ == "__main__":
    main()
