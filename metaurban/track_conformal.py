"""track_conformal — HCT-D tracking-tube calibration (the ego-tracking twin of conformal_calibrate.py).

Input: out/conformal/track/track_s*.npy, each rows (window, delta, ||v||, ||a||, lateral_accel) harvested by
render_3d_video.py --headless with TRACK_HARVEST=1 (delta = ||p_flown - p_plan|| of the committed set-point;
||a||, lateral = centripetal = kappa*||v||^2 are the HODOGRAPH covariates of that committed B-spline -- model-free).

Pipeline (per the deep-research synthesis):
  1. SHAPE g(feat) = c0 + c1*||a|| + c2*lateral  via non-negative least squares of delta on the covariates. The c's
     are fixed dimensionless RELATIVE weights, NOT the safety knob. (||v|| / jerk can be added; ||a||+curvature are
     the dominant drivers -- collisions live in high-accel / high-curvature evasive turns, NOT peak speed.)
  2. CONSTANT kappa via SUP-NORMALIZED split conformal: per WINDOW the score s_w = max_t delta(t)/g(t); per EPISODE
     (block) S_e = max_w s_w; kappa = ceil((n+1)(1-eps))-th order statistic of {S_e over episodes}. Then for a fresh
     exchangeable episode P(all delta(t) <= kappa*g(t)) >= 1-eps  (the all-t band a continuous-time cert needs).
  3. Validate marginal coverage on a held-out split; emit out/conformal/track_calib.json {eps: {c0,c1,c2,kappa,...}}.

Run:  python metaurban/track_conformal.py
"""
import os, sys, json, glob
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
# render_3d_video.py writes the harvest under metaurban/out/ (_HERE-relative); keep the calib alongside it.
TRACKDIR = os.path.join(HERE, "out", "conformal", "track")
OUT = os.path.join(HERE, "out", "conformal", "track_calib.json")
EPS_LIST = [0.20, 0.10, 0.05, 0.01]
RNG = np.random.default_rng(7)


def _nnls(A, b):
    try:
        from scipy.optimize import nnls
        x, _ = nnls(A, b); return x
    except Exception:
        x, *_ = np.linalg.lstsq(A, b, rcond=None)   # fallback: clip negatives
        return np.clip(x, 0.0, None)


def main():
    files = sorted(glob.glob(os.path.join(TRACKDIR, "track_s*.npy")))
    if not files:
        print("no harvest; run render_3d_video.py --headless --maneuver with TRACK_HARVEST=1 first"); return 1
    # episodes = files; each row (win, delta, nv, na, lat)
    eps_data = []
    for f in files:
        d = np.load(f)
        if len(d):
            eps_data.append(d)
    allrows = np.vstack(eps_data)
    win, delta, nv, na, lat = (allrows[:, i] for i in range(5))
    print(f"[track] {len(eps_data)} episodes, {len(allrows)} sub-steps; "
          f"delta[med/95/max]={np.median(delta):.3f}/{np.quantile(delta,0.95):.3f}/{delta.max():.3f}m")

    # ENVELOPE CLAMP (piece D): tracking error is heavy-tailed -- the tail is high-||a|| SATURATION events that
    # conformal can only cover with a huge, reach-killing kappa. Define a verified-trackable envelope A_ENV on the
    # commanded acceleration; calibrate the tube ONLY on in-envelope data (tight), and at deploy REFUSE to certify
    # any candidate whose committed B-spline exceeds A_ENV (-> HOLD / pick a gentler candidate). Choose A_ENV as the
    # accel below which the in-envelope delta tail is small.
    A_ENV = float(os.environ.get("TRACK_AENV", 12.0))
    env = na <= A_ENV
    clamp_rate = float(np.mean(~env))
    de_in = delta[env]
    print(f"[track] envelope A_ENV={A_ENV} m/s^2: in-envelope {env.mean()*100:.1f}% of sub-steps "
          f"(clamp {clamp_rate*100:.1f}%); in-env delta[95/99/max]="
          f"{np.quantile(de_in,0.95):.3f}/{np.quantile(de_in,0.99):.3f}/{de_in.max():.3f}m")

    # 1. SHAPE: NNLS of delta on [1, ||a||, lateral], IN-ENVELOPE only
    A = np.column_stack([np.ones(env.sum()), na[env], lat[env]])
    c = _nnls(A, de_in)
    c[0] = max(c[0], 0.02)                                  # floor c0>0 so g is a valid (positive) denominator
    print(f"[track] shape g = {c[0]:.3f} + {c[1]:.3f}*||a|| + {c[2]:.3f}*lateral  (in-envelope fit)")

    def g_of(na_, lat_):
        return c[0] + c[1] * na_ + c[2] * lat_

    # 2. split-conformal kappa per eps: episode-block scores, calibrate/test split for validation
    n = len(eps_data); idx = RNG.permutation(n); half = n // 2
    cal_i, test_i = idx[:max(half, 1)], idx[max(half, 1):]

    def episode_score(d):
        w, de, _nv, na_, lat_ = (d[:, i] for i in range(5))
        ein = na_ <= A_ENV                                  # out-of-envelope is clamped (HOLD), not certified
        if not ein.any():
            return 0.0
        w, de, na_, lat_ = w[ein], de[ein], na_[ein], lat_[ein]
        s = de / np.maximum(g_of(na_, lat_), 1e-6)         # sup-normalized residual per in-envelope sub-step
        return max((s[w == ww].max() for ww in np.unique(w)), default=0.0)   # per-window max -> per-episode block max

    cal_scores = np.array([episode_score(eps_data[i]) for i in cal_i])
    test_scores = np.array([episode_score(eps_data[i]) for i in test_i]) if len(test_i) else cal_scores

    # --- DEPLOYABLE margin delta_track. The CORRECT exchangeable unit for a PER-FLIGHT collision-freedom guarantee is
    # the EPISODE, not the commit-window: windows within one flight share the same quad + maneuver, so they are strongly
    # autocorrelated, NOT exchangeable. delta_track_flight(eps) = (1-eps) split-conformal quantile of the per-EPISODE-MAX
    # raw tracking error |flown - planned| over the cal episodes (n~99), validated on held-out episodes. This is the
    # number wired into the cert keep-out. (The per-WINDOW quantile -- shown for reference as delta_track_window_m -- is
    # ~10x smaller because the window pool inflates effective n ~100x; it gives only marginal-over-windows coverage and
    # at deploy ~30% of FLIGHTS breach it, so it is NOT a valid per-flight margin. Code-review 2026-06-27 critical fix.)
    def episode_maxes(idxs):
        return np.array([float(eps_data[i][:, 1].max()) for i in idxs])

    def window_maxes(idxs):
        out_w = []
        for i in idxs:
            d = eps_data[i]; w, de = d[:, 0], d[:, 1]
            out_w += [float(de[w == ww].max()) for ww in np.unique(w)]
        return np.array(out_w)
    cal_em, test_em = episode_maxes(cal_i), (episode_maxes(test_i) if len(test_i) else episode_maxes(cal_i))
    cal_wm, test_wm = window_maxes(cal_i), (window_maxes(test_i) if len(test_i) else window_maxes(cal_i))

    out = {"shape": {"c0": float(c[0]), "c1": float(c[1]), "c2": float(c[2])},
           "a_env": A_ENV, "clamp_rate": clamp_rate, "unit": "episode (per-flight; exchangeable)",
           "n_episodes": n, "n_episodes_cal": int(len(cal_em)), "n_episodes_test": int(len(test_em)),
           "n_substeps": int(len(allrows)), "n_windows_cal": int(len(cal_wm)), "levels": {}}
    print(f"\n{'eps':>5} | {'delta_track(FLIGHT)':>19} {'flight_cov':>10} | {'delta_window':>12} {'win_cov':>8}")
    for eps in EPS_LIST:
        # PER-FLIGHT (episode) conformal quantile with finite-sample (m+1) correction -> the deployed margin.
        # If ceil((m+1)(1-eps)) > m the finite-sample quantile DOES NOT EXIST (it is +inf): eps is not achievable
        # with m episodes (need m >= (1-eps)/eps, e.g. eps=0.01 needs >=99). The old silent min(rank, m) clamp
        # reported the sample max as if it were a valid (1-eps) margin -- an unbacked guarantee. Now: loud
        # UNACHIEVABLE + null in the JSON; harvest more episodes or pick a larger eps instead of shipping it.
        me = len(cal_em); rke = int(np.ceil((me + 1) * (1 - eps)))
        # per-window (reference only; NOT per-flight valid)
        mw = len(cal_wm); rkw = min(int(np.ceil((mw + 1) * (1 - eps))), mw)
        delta_window = float(np.sort(cal_wm)[rkw - 1])
        win_cov = float(np.mean(test_wm <= delta_window))
        if rke > me:
            out["levels"][str(eps)] = {"delta_track_m": None, "achievable": False, "flight_coverage": None,
                                       "delta_track_window_m": delta_window, "window_coverage": win_cov,
                                       "target": 1 - eps, "n_cal_episodes": me,
                                       "n_episodes_needed": int(np.ceil((1 - eps) / eps))}
            print(f"{eps:>5} | {'UNACHIEVABLE(m=%d)' % me:>19} {'--':>10} | {delta_window:>12.3f} {win_cov:>8.3f}")
            continue
        delta_flight = float(np.sort(cal_em)[rke - 1])
        flight_cov = float(np.mean(test_em <= delta_flight))
        out["levels"][str(eps)] = {"delta_track_m": delta_flight, "achievable": True, "flight_coverage": flight_cov,
                                   "delta_track_window_m": delta_window, "window_coverage": win_cov,
                                   "target": 1 - eps, "n_cal_episodes": me}
        print(f"{eps:>5} | {delta_flight:>19.3f} {flight_cov:>10.3f} | {delta_window:>12.3f} {win_cov:>8.3f}")

    json.dump(out, open(OUT, "w"), indent=2)
    print(f"\n[track] wrote {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
