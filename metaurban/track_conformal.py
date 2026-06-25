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
EPS_LIST = [0.20, 0.10, 0.05]
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

    out = {"shape": {"c0": float(c[0]), "c1": float(c[1]), "c2": float(c[2])},
           "a_env": A_ENV, "clamp_rate": clamp_rate,
           "n_episodes": n, "n_substeps": int(len(allrows)), "levels": {}}
    print(f"\n{'eps':>5} {'kappa':>8} {'tube_med(m)':>12} {'test_cov':>9}")
    for eps in EPS_LIST:
        m = len(cal_scores); rank = int(np.ceil((m + 1) * (1 - eps)))
        kappa = float(np.sort(cal_scores)[min(rank, m) - 1]) if rank <= m else float("inf")
        cov = float(np.mean(test_scores <= kappa)) if np.isfinite(kappa) else 1.0
        # representative tube size = kappa * median g over all sub-steps
        tube_med = kappa * float(np.median(g_of(na, lat)))
        out["levels"][str(eps)] = {"kappa": kappa, "tube_med_m": tube_med, "test_coverage": cov,
                                   "target": 1 - eps, "rank": rank, "n_cal": m}
        print(f"{eps:>5} {kappa:>8.3f} {tube_med:>12.3f} {cov:>9.3f}")

    json.dump(out, open(OUT, "w"), indent=2)
    print(f"\n[track] wrote {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
