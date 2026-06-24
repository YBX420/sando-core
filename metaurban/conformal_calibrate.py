"""conformal_calibrate — distribution-free calibration of the certificate keep-out (q_conformal, v_eff).

The certificate guards  || p_drone(tau) - c_pred(tau) || >= R + rho(tau),  rho(tau) = q_conformal + v_eff*(tau+delta).
For this to imply clearance to the TRUE (not predicted) mover with probability >= 1-eps we need the prediction
residual to stay inside the tube:   e(Delta) := || c_true(Delta) - c_pred(Delta) || <= q_conformal + v_eff*Delta
where Delta = elapsed time since the last detection (the cert evaluates this at Delta = tau + delta).

SPLIT CONFORMAL (distribution-free, finite-sample):
  - instances = (mover track, detection time t_k, horizon Delta).  Run the deployed CA-Kalman on NOISY detections
    at the real control cadence; after each update, predict c_pred(t_k+Delta) for a grid of Delta and score the
    residual e against the harvested ground-truth track (interpolated to t_k+Delta).
  - split BY TRACK into calibration / test (residuals within a track are temporally correlated; splitting by track
    keeps cal and test exchangeable at the track level -- the honest unit of exchangeability here).
  - per-horizon conformal quantile q_hat(Delta) = the ceil((n+1)(1-eps))/n empirical quantile on calibration.
  - deployable AFFINE tube: fit (q_conformal, v_eff) so q_conformal + v_eff*Delta >= q_hat(Delta) for ALL Delta
    (a conservative envelope of the per-horizon conformal quantiles -> inherits >= 1-eps coverage at every horizon).
  - VALIDATE marginal coverage on the held-out TEST tracks. This is the number that backs "P(collision) <= eps".

PER-CLASS: pedestrians (~1.5 m/s, swerve) and vehicles/robots (faster, larger CA-prediction error) have very
different residual growth, so we calibrate a separate tube per class (mirrors EGO_PERCLASS_DSAFE) AND an "all"
pooled tube. Deployment uses the per-class (q_conformal, v_eff) for each mover.

Run (any env with numpy):  python metaurban/conformal_calibrate.py
"""
import os, sys, glob, json
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from kf_tracker import MoverTracker

OUTDIR = os.path.join(os.path.dirname(HERE), "out", "conformal")

# deployment cadence / filter knobs (must match scenario.py + kf_tracker defaults so calibration == deployment)
DT_CTRL = 0.30           # control / detection period (KF dt, and the cert's delta latency)
MEAS = 0.07              # detection noise std (m) per axis, matches MoverTracker(meas_noise)
TAU = 0.75               # certificate trust window
DELTA = DT_CTRL          # perception->commit latency already accrued at tau=0
# horizon grid in ELAPSED-since-detection time. Cert reaches tau=TAU evaluated at Delta = TAU + delta.
DELTAS = np.array([0.30, 0.45, 0.60, 0.75, 0.90, 1.05])
EPS_LEVELS = [0.20, 0.10, 0.05, 0.01]
CAL_FRAC = 0.6           # fraction of tracks used for calibration (rest = test)
# deployed motion model for the predicted centre (predictor_compare.py: CV roughly HALVES the pedestrian
# keep-out vs CA at the same coverage, because CA's noisy accel extrapolation overshoots jerky pedestrians).
PRED_MODEL = os.environ.get("PRED_MODEL", "cv")


def _interp_xy(t_track, xy_track, t_query):
    """Ground-truth mover position at arbitrary time via linear interpolation of the 0.1 s harvested track.
    Returns None if t_query is outside the track's time span (can't score that horizon)."""
    if t_query < t_track[0] or t_query > t_track[-1]:
        return None
    x = np.interp(t_query, t_track, xy_track[:, 0])
    y = np.interp(t_query, t_track, xy_track[:, 1])
    return np.array([x, y])


def residuals_for_track(mover, rng):
    """Run the deployed KF on noisy detections at DT_CTRL cadence; return list of (Delta, residual) pairs."""
    t_tr = np.asarray(mover["t"], float)
    xy_tr = np.asarray(mover["xy"], float)
    t0, t1 = float(t_tr[0]), float(t_tr[-1])
    trk = MoverTracker(dt=DT_CTRL, meas_noise=MEAS)
    out = []
    for tk in np.arange(t0, t1 + 1e-9, DT_CTRL):
        gt = _interp_xy(t_tr, xy_tr, tk)
        if gt is None:
            continue
        det = gt + rng.normal(0, MEAS, 2)
        trk.update([det[0], det[1], 1.5])
        if not trk.ready:
            continue
        for d in DELTAS:
            true_future = _interp_xy(t_tr, xy_tr, tk + d)
            if true_future is None:
                continue
            pred = trk.predict([d], model=PRED_MODEL)[0, :2]
            out.append((float(d), float(np.linalg.norm(true_future - pred))))
    return out


def conformal_quantile(scores, eps):
    """Split-conformal upper quantile: smallest calibration score s.t. >= (1-eps) mass is <= it, with the
    finite-sample correction rank = ceil((n+1)(1-eps)). Returns +inf if n too small for the guarantee."""
    s = np.sort(np.asarray(scores, float))
    n = len(s)
    if n == 0:
        return float("inf")
    rank = int(np.ceil((n + 1) * (1.0 - eps)))
    if rank > n:
        return float("inf")
    return float(s[rank - 1])


def fit_affine_envelope(deltas, qhat):
    """Affine tube q0 + s*Delta DOMINATING the per-horizon conformal quantiles (conservative envelope).
    Least-squares slope (clamped >= 0), then raise the intercept until every q_hat(Delta) is covered."""
    d = np.asarray(deltas, float); q = np.asarray(qhat, float)
    finite = np.isfinite(q)
    d, q = d[finite], q[finite]
    if len(d) < 2:
        return (float(np.max(q)) if len(q) else 0.0), 0.0
    A = np.vstack([d, np.ones_like(d)]).T
    slope = max(0.0, float(np.linalg.lstsq(A, q, rcond=None)[0][0]))
    q0 = float(np.max(q - slope * d))
    return q0, slope


def calibrate(cal, test, deltas):
    """cal/test are dict[Delta] -> list of residuals. Returns the per-eps report (q_conformal, v_eff, coverage)."""
    levels = {}
    for eps in EPS_LEVELS:
        qhat = [conformal_quantile(cal[float(d)], eps) for d in deltas]
        q0, slope = fit_affine_envelope(deltas, qhat)
        per_h = {}; cov_num = cov_den = 0
        for j, d in enumerate(deltas):
            sc = np.asarray(test[float(d)], float)
            if len(sc) == 0:
                continue
            tube = q0 + slope * float(d)
            covered = int(np.sum(sc <= tube + 1e-12))
            per_h[float(d)] = dict(tube=round(tube, 4), coverage=round(covered / len(sc), 4),
                                   q_hat=round(float(qhat[j]), 4), n=int(len(sc)),
                                   p95=round(float(np.percentile(sc, 95)), 4),
                                   p99=round(float(np.percentile(sc, 99)), 4))
            cov_num += covered; cov_den += len(sc)
        overall = cov_num / cov_den if cov_den else float("nan")
        levels[str(eps)] = dict(target_coverage=round(1 - eps, 4), q_conformal=round(q0, 4),
                                v_eff=round(slope, 4), test_marginal_coverage=round(overall, 4),
                                per_horizon=per_h)
    return levels


def main():
    files = sorted(glob.glob(os.path.join(OUTDIR, "traj_seed*.npz")))
    if not files:
        print(f"[calib] no harvested trajectories in {OUTDIR}; run conformal_harvest.py first", flush=True)
        return 1
    tracks = []
    for f in files:
        dat = np.load(f, allow_pickle=True)
        for m in dat["movers"]:
            tracks.append(m)
    n_tr = len(tracks)
    order = np.arange(n_tr)
    np.random.default_rng(12345).shuffle(order)
    n_cal = int(round(CAL_FRAC * n_tr))
    cal_idx = set(order[:n_cal].tolist())

    # residual buckets: per group ("all" + each class) -> dict[Delta] -> {cal:[], test:[]}
    groups = {}

    def _bucket(group, d, e, is_cal):
        g = groups.setdefault(group, {float(x): {"cal": [], "test": []} for x in DELTAS})
        g[float(d)]["cal" if is_cal else "test"].append(e)

    rng = np.random.default_rng(2026)
    n_inst = {"all": [0, 0]}
    for i, m in enumerate(tracks):
        is_cal = i in cal_idx
        cls = str(m["cls"])
        pairs = residuals_for_track(m, rng)
        for d, e in pairs:
            _bucket("all", d, e, is_cal)
            _bucket(cls, d, e, is_cal)
        n_inst["all"][0 if is_cal else 1] += len(pairs)
        n_inst.setdefault(cls, [0, 0])[0 if is_cal else 1] += len(pairs)

    # class track counts
    cls_tracks = {}
    for m in tracks:
        cls_tracks[str(m["cls"])] = cls_tracks.get(str(m["cls"]), 0) + 1

    report = dict(n_tracks=n_tr, n_cal_tracks=n_cal, n_test_tracks=n_tr - n_cal, pred_model=PRED_MODEL,
                  dt_ctrl=DT_CTRL, meas_noise=MEAS, tau=TAU, delta=DELTA, deltas=DELTAS.tolist(),
                  class_track_counts=cls_tracks, groups={})

    for group, g in groups.items():
        cal = {d: g[d]["cal"] for d in g}
        test = {d: g[d]["test"] for d in g}
        report["groups"][group] = dict(
            n_cal_instances=n_inst.get(group, [0, 0])[0], n_test_instances=n_inst.get(group, [0, 0])[1],
            levels=calibrate(cal, test, DELTAS))
        print(f"[calib] === group '{group}' ({n_inst.get(group,[0,0])[0]} cal / "
              f"{n_inst.get(group,[0,0])[1]} test instances) ===", flush=True)
        for eps in EPS_LEVELS:
            lv = report["groups"][group]["levels"][str(eps)]
            print(f"[calib]   eps={eps:.2f} (target {1-eps:.2f}): q_conformal={lv['q_conformal']:+.3f} m  "
                  f"v_eff={lv['v_eff']:.3f} m/s  -> TEST coverage={lv['test_marginal_coverage']:.4f}", flush=True)

    os.makedirs(OUTDIR, exist_ok=True)
    with open(os.path.join(OUTDIR, "calib.json"), "w") as f:
        json.dump(report, f, indent=2)
    print(f"[calib] wrote {os.path.join(OUTDIR, 'calib.json')}", flush=True)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        order_groups = [g for g in ("pedestrian", "vehicle", "all") if g in groups]
        fig, axes = plt.subplots(1, len(order_groups), figsize=(5.2 * len(order_groups), 4.4), squeeze=False)
        for ax, group in zip(axes[0], order_groups):
            g = groups[group]
            for d in DELTAS:
                sc = np.asarray(g[float(d)]["test"], float)
                if len(sc):
                    samp = sc[:500]
                    jit = (rng.random(len(samp)) - 0.5) * 0.03
                    ax.scatter(np.full(len(samp), d) + jit, samp, s=3, alpha=0.12, color="0.6")
            for eps, color in zip([0.10, 0.05, 0.01], ["tab:blue", "tab:green", "tab:red"]):
                lv = report["groups"][group]["levels"][str(eps)]
                ax.plot(DELTAS, lv["q_conformal"] + lv["v_eff"] * DELTAS, color=color, lw=2,
                        label=f"eps={eps}: v={lv['v_eff']:.2f}, cov={lv['test_marginal_coverage']:.3f}")
            ax.set_xlabel("elapsed since detection  Delta (s)"); ax.set_ylabel("KF residual |true-pred| (m)")
            ax.set_title(f"{group}  (n_test={report['groups'][group]['n_test_instances']})")
            ax.legend(fontsize=7); ax.grid(alpha=0.3); ax.set_ylim(bottom=0)
        fig.suptitle("Conformal keep-out calibration on real MetaUrban movers", y=1.02)
        fig.tight_layout(); fig.savefig(os.path.join(OUTDIR, "calib_coverage.png"), dpi=130, bbox_inches="tight")
        print(f"[calib] wrote {os.path.join(OUTDIR, 'calib_coverage.png')}", flush=True)
    except Exception as e:
        print(f"[calib] (plot skipped: {e})", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
