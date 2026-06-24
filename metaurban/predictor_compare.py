"""predictor_compare — which motion model gives the TIGHTEST conformal keep-out?

The keep-out the certificate must hold is the conformal quantile of the PREDICTION RESIDUAL. A better predictor
=> smaller residual => smaller v_eff => the drone may fly tighter/faster while keeping the SAME coverage. So the
predictor is not a free choice: we pick the one that minimises the certified keep-out at a fixed coverage.

Compares, on the real harvested MetaUrban tracks, per class and eps:
  CA = constant-acceleration KF (current cert polynomial)
  CV = constant-velocity (drop the noisy filtered accel)
  CVd = velocity-damped (accel x 0.0 but velocity shrunk toward recent average -- a light hedge against turns)
Reports (q_conformal, v_eff, coverage) for each -> the winner (lowest v_eff @ >=1-eps coverage) is the deployed model.

Run:  python metaurban/predictor_compare.py
"""
import os, sys, glob, json
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from kf_tracker import MoverTracker
import conformal_calibrate as CC

OUTDIR = CC.OUTDIR
MODELS = ["ca", "cv"]


def residuals_for_track(mover, rng, model):
    t_tr = np.asarray(mover["t"], float); xy_tr = np.asarray(mover["xy"], float)
    t0, t1 = float(t_tr[0]), float(t_tr[-1])
    trk = MoverTracker(dt=CC.DT_CTRL, meas_noise=CC.MEAS)
    out = []
    for tk in np.arange(t0, t1 + 1e-9, CC.DT_CTRL):
        gt = CC._interp_xy(t_tr, xy_tr, tk)
        if gt is None:
            continue
        trk.update([gt[0] + rng.normal(0, CC.MEAS), gt[1] + rng.normal(0, CC.MEAS), 1.5])
        if not trk.ready:
            continue
        for d in CC.DELTAS:
            tf = CC._interp_xy(t_tr, xy_tr, tk + d)
            if tf is None:
                continue
            pred = trk.predict([d], model=model)[0, :2]
            out.append((float(d), float(np.linalg.norm(tf - pred))))
    return out


def main():
    files = sorted(glob.glob(os.path.join(OUTDIR, "traj_seed*.npz")))
    if not files:
        print("no harvested seeds"); return 1
    tracks = []
    for f in files:
        tracks += list(np.load(f, allow_pickle=True)["movers"])
    n = len(tracks)
    order = np.arange(n); np.random.default_rng(12345).shuffle(order)
    cal_idx = set(order[:int(round(CC.CAL_FRAC * n))].tolist())

    report = {}
    for model in MODELS:
        groups = {}
        rng = np.random.default_rng(2026)
        for i, m in enumerate(tracks):
            is_cal = i in cal_idx; cls = str(m["cls"])
            for d, e in residuals_for_track(m, rng, model):
                for grp in ("all", cls):
                    g = groups.setdefault(grp, {float(x): {"cal": [], "test": []} for x in CC.DELTAS})
                    g[float(d)]["cal" if is_cal else "test"].append(e)
        report[model] = {}
        for grp, g in groups.items():
            cal = {d: g[d]["cal"] for d in g}; test = {d: g[d]["test"] for d in g}
            report[model][grp] = CC.calibrate(cal, test, CC.DELTAS)

    # print comparison table for the eps we deploy at
    print(f"{'class':<12}{'eps':>6}  " + "  ".join(f"{m.upper():>22}" for m in MODELS))
    print(f"{'':12}{'':6}  " + "  ".join(f"{'(q, v_eff, cov)':>22}" for m in MODELS))
    for grp in ("pedestrian", "vehicle", "all"):
        for eps in ("0.1", "0.05"):
            cells = []
            for model in MODELS:
                lv = report.get(model, {}).get(grp, {}).get(eps)
                if lv:
                    cells.append(f"({lv['q_conformal']:+.2f},{lv['v_eff']:.2f},{lv['test_marginal_coverage']:.3f})")
                else:
                    cells.append(" " * 22)
            print(f"{grp:<12}{eps:>6}  " + "  ".join(f"{c:>22}" for c in cells))
        print()

    # winner per class @ eps=0.05: lowest v_eff among models meeting coverage
    winners = {}
    for grp in ("pedestrian", "vehicle", "all"):
        best = None
        for model in MODELS:
            lv = report.get(model, {}).get(grp, {}).get("0.05")
            if lv and lv["test_marginal_coverage"] >= 0.95 - 1e-3:
                if best is None or lv["v_eff"] < best[1]:
                    best = (model, lv["v_eff"])
        winners[grp] = best
        if best:
            print(f"[winner] {grp}: model={best[0].upper()} (v_eff={best[1]:.3f} m/s @>=0.95 coverage)")

    with open(os.path.join(OUTDIR, "predictor_compare.json"), "w") as f:
        json.dump(dict(report=report, winners=winners), f, indent=2)
    print(f"[pc] wrote {os.path.join(OUTDIR, 'predictor_compare.json')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
