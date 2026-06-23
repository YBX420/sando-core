"""KF trajectory predictor experiment — does a Kalman filter shrink q_conformal vs constant-velocity?

WHY: the safety certificate (bernstein_cert.hpp) certifies against a PREDICTED obstacle centre
ĉ(t)=c0+v·t+½a·t² (a degree-<=2 / constant-accel polynomial). The conformal tube radius
q_conformal must cover the whole-horizon prediction residual:
      q_conformal = (1-eps)-quantile over tracks of  s = sup_{t in [0,t_hi]} ||ĉ(t) - c_true(t)||.
A Kalman filter's CONSTANT-ACCELERATION state [p,v,a] feeds ĉ(t) EXACTLY in the form the cert needs,
and (unlike raw CV off noisy detections) it smooths tracker noise and estimates acceleration.

This script measures, on a SYNTHETIC pedestrian model (curved/maneuvering walkers + tracker noise):
  - prediction residual  s_H = sup_{t<=H} ||pred-true||  for 3 predictors x 3 horizons
  - the split-conformal q_conformal each would need (finite-sample (1-alpha) quantile)
  - held-out coverage (sanity: the calibrated q actually achieves >= 1-alpha out of sample)
  - the resulting tube radius (q + d_safe) so you can see if it "freezes the corridor".

WHAT IT PROVES / DOES NOT:
  + Validates the predictor-comparison + conformal machinery, and the horizon effect
    (the whole reason tau_trust=0.75s may make CV "good enough" and learning unnecessary).
  - q_conformal here is on a SYNTHETIC error model, NOT deployment-safe. The real number needs
    residuals from a real predictor on ONBOARD-RENDER tracker output (see docs/safety-layer-spec.md §5).
    To get the real number: replace gen_track() with logged (predicted, true) tracker pairs; the rest is the harness.

Run (in the conda env with numpy):  python conformal/kf_predictor_experiment.py
"""
import numpy as np

RNG = np.random.default_rng(2026)
EPS_PRED = 0.08          # the prediction-error budget (q at the (1-EPS_PRED) quantile)
D_SAFE = 0.8             # geometric safety distance the tube adds onto
HORIZONS = [0.75, 2.0, 3.0]   # 0.75 = current tau_trust; 2/3 = longer-lookahead what-ifs
DT = 0.1                 # tracker / obs rate (10 Hz)
WARMUP_S = 2.0           # observe this long to warm the filter, then predict forward
MEAS_NOISE = 0.07        # tracker position noise std (m), D435i-ish
N_TRACKS = 6000
N_CAL = 2000             # split-conformal calibration size (rest = held-out test)


# ----------------------------------------------------------------------------- synthetic pedestrian
def gen_track(rng):
    """A maneuvering pedestrian: piecewise constant turn-rate + tangential accel (CV-breaking),
    human speeds, plus position measurement noise. Returns (true_xy[N,2], obs_xy[N,2])."""
    T_total = WARMUP_S + max(HORIZONS) + 0.5
    n = int(round(T_total / DT)) + 1
    speed = max(0.2, rng.normal(1.4, 0.35))
    heading = rng.uniform(0, 2 * np.pi)
    p = rng.normal(0, 3, 2)
    true = np.zeros((n, 2)); true[0] = p
    omega = 0.0; a_tan = 0.0
    for k in range(1, n):
        if rng.random() < DT / 1.2:                       # ~ a maneuver change every ~1.2 s
            omega = float(np.clip(rng.normal(0, 0.4), -0.9, 0.9))    # rad/s turn rate
            a_tan = float(np.clip(rng.normal(0, 0.4), -0.8, 0.8))    # m/s^2 tangential accel
        heading += omega * DT
        speed = max(0.0, speed + a_tan * DT)
        p = p + speed * np.array([np.cos(heading), np.sin(heading)]) * DT
        true[k] = p
    obs = true + MEAS_NOISE * rng.standard_normal(true.shape)
    return true, obs


# ----------------------------------------------------------------------------- predictors
CV_WIN_S = 1.0          # CV estimates velocity from the most-recent ~1 s of obs (fair "current velocity")


def predict_cv_lsq(obs, i0, ts):
    """No filter: least-squares linear fit over the recent obs window -> (p0,v); predict from the
    prediction instant (last obs) forward: ĉ(ts) = pos_at_i0 + v*ts."""
    m = max(2, int(round(CV_WIN_S / DT)))
    w = obs[max(0, i0 + 1 - m): i0 + 1]
    tw = np.arange(len(w)) * DT
    A = np.column_stack([np.ones_like(tw), tw])          # [1, t]
    coef, *_ = np.linalg.lstsq(A, w, rcond=None)         # rows: [p0; v] per axis (p0 at window start)
    p0, v = coef[0], coef[1]
    p_at = p0 + v * tw[-1]                               # fitted position at the prediction instant (i0)
    return p_at[None, :] + np.outer(ts, v)               # (len(ts), 2)


def _kf_run(obs, i0, order):
    """Per-axis linear KF over obs[:i0+1]. order=1 -> CV state [p,v]; order=2 -> CA state [p,v,a].
    Returns the filtered state per axis at i0 (list of arrays)."""
    states = []
    for ax in range(2):
        z = obs[: i0 + 1, ax]
        if order == 1:
            F = np.array([[1, DT], [0, 1.0]])
            qa = 1.0                                       # white-accel spectral density
            Q = qa * np.array([[DT**3 / 3, DT**2 / 2], [DT**2 / 2, DT]])
            H = np.array([[1.0, 0.0]])
            x = np.array([z[0], 0.0]); P = np.diag([MEAS_NOISE**2, 1.0])
        else:
            F = np.array([[1, DT, DT**2 / 2], [0, 1, DT], [0, 0, 1.0]])
            qj = 2.0                                       # white-jerk spectral density
            Q = qj * np.array([[DT**5 / 20, DT**4 / 8, DT**3 / 6],
                               [DT**4 / 8,  DT**3 / 3, DT**2 / 2],
                               [DT**3 / 6,  DT**2 / 2, DT]])
            H = np.array([[1.0, 0.0, 0.0]])
            x = np.array([z[0], 0.0, 0.0]); P = np.diag([MEAS_NOISE**2, 1.0, 1.0])
        R = np.array([[MEAS_NOISE**2]])
        for k in range(1, len(z)):
            x = F @ x; P = F @ P @ F.T + Q                 # predict
            y = z[k] - (H @ x)[0]; S = (H @ P @ H.T)[0, 0] + R[0, 0]
            K = (P @ H.T)[:, 0] / S                        # update
            x = x + K * y; P = (np.eye(len(x)) - np.outer(K, H[0])) @ P
        states.append(x)
    return states


def predict_kf(obs, i0, ts, order):
    st = _kf_run(obs, i0, order)
    out = np.zeros((len(ts), 2))
    for ax in range(2):
        x = st[ax]
        if order == 1:
            out[:, ax] = x[0] + x[1] * ts
        else:
            out[:, ax] = x[0] + x[1] * ts + 0.5 * x[2] * ts**2
    return out


PREDICTORS = {
    "CV (lsq, no filter)": lambda o, i0, ts: predict_cv_lsq(o, i0, ts),
    "CV-Kalman":           lambda o, i0, ts: predict_kf(o, i0, ts, order=1),
    "CA-Kalman (cert-shaped)": lambda o, i0, ts: predict_kf(o, i0, ts, order=2),
}


# ----------------------------------------------------------------------------- conformal
def conformal_q(scores_cal, alpha):
    """finite-sample (1-alpha) quantile: the ceil((n+1)(1-alpha))-th order statistic."""
    n = len(scores_cal)
    rank = int(np.ceil((n + 1) * (1 - alpha)))
    if rank > n:
        return float("inf")                                # not enough data for this alpha
    return float(np.sort(scores_cal)[rank - 1])


# ----------------------------------------------------------------------------- experiment
def main():
    i0 = int(round(WARMUP_S / DT))                          # predict-from index
    # score[pred][H] = array of per-track sup residuals
    scores = {name: {H: [] for H in HORIZONS} for name in PREDICTORS}
    for _ in range(N_TRACKS):
        true, obs = gen_track(RNG)
        for name, fn in PREDICTORS.items():
            for H in HORIZONS:
                hsteps = int(round(H / DT))
                ts = np.arange(1, hsteps + 1) * DT          # horizon sample times (exclude 0)
                pred = fn(obs, i0, ts)                       # (hsteps, 2)
                true_future = true[i0 + 1 : i0 + 1 + hsteps]
                e = np.linalg.norm(pred - true_future, axis=1)
                scores[name][H].append(float(e.max()))       # sup-over-horizon nonconformity
    for name in scores:
        for H in HORIZONS:
            scores[name][H] = np.asarray(scores[name][H])

    print(f"\nSynthetic maneuvering pedestrians: N={N_TRACKS} tracks, obs {1/DT:.0f} Hz, "
          f"meas-noise {MEAS_NOISE} m, warmup {WARMUP_S}s.")
    print(f"q_conformal = (1-eps)-quantile of sup-over-horizon residual; eps_pred={EPS_PRED} "
          f"(also q95 shown). tube = q + d_safe({D_SAFE} m).\n")
    hdr = f"{'predictor':<26}{'horizon':>9}{'q@1-eps':>10}{'q95':>9}{'tube@1-eps':>12}{'held-out cov':>14}"
    print(hdr); print("-" * len(hdr))
    for name in PREDICTORS:
        for H in HORIZONS:
            s = scores[name][H]
            cal, test = s[:N_CAL], s[N_CAL:]
            q_eps = conformal_q(cal, EPS_PRED)
            q95 = conformal_q(cal, 0.05)
            cov = float(np.mean(test <= q_eps)) if np.isfinite(q_eps) else float("nan")
            tube = q_eps + D_SAFE
            print(f"{name:<26}{H:>8.2f}s{q_eps:>10.3f}{q95:>9.3f}{tube:>12.3f}{cov:>13.3f} "
                  f"{'OK' if cov >= 1 - EPS_PRED - 0.02 else 'LOW'}")
        print()

    print("READ:")
    print("  * Compare the H=0.75s rows: if CV's tube is already small, a KF/learning is NOT needed there.")
    print("  * Compare CA-Kalman vs CV across H=2,3s: that gap is what a better predictor buys at long horizon.")
    print("  * held-out cov >= 1-eps confirms the conformal calibration is sound on this (synthetic) distribution.")
    print("  * These q's are SYNTHETIC. Real q_conformal: feed logged (predicted,true) tracker pairs into")
    print("    `scores` per Mondrian cell (class x density); the calibration code is unchanged.")


if __name__ == "__main__":
    main()
