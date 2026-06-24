"""ego_goaround — certified GO-AROUND on EGO (predict-with-Kalman, route around the future, don't HOLD).

The thesis demonstrated, sharpened (2026-06-23): the layer is not a judge that only HOLDs. We
  1. PREDICT each mover forward with a CA-Kalman filter (kf_tracker.MoverTracker),
  2. render the predicted swept footprint into EGO's grid so EGO plans a path AROUND where the human
     WILL be (EGO's solver is untouched -> still planner-agnostic; we only feed it occupancy),
  3. certify the committed B-spline against the moving sphere c(t)=c0+v*t+1/2*a*t^2 (the KF state),
  4. certified -> FLY the go-around for one DT (fastest safe progress); uncertified -> HOLD (fallback only).

Contrast baseline = ego_safe.py (feeds the obstacle's CURRENT position -> EGO can't see the future ->
binary certify-or-HOLD). Run both and compare go-around vs HOLD counts and time-to-goal.

Run headless:  python metaurban/ego_goaround.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from ego_bridge import EGOPlanner
from kf_tracker import MoverTracker


def _sphere_surface(c, r, n_th=10, dzs=(-0.3, 0.0, 0.3)):
    """occupied points on a sphere's surface (what EGO's depth-FOV would see), centred at c."""
    pts = []
    for th in np.linspace(0, 2 * np.pi, n_th, endpoint=False):
        for dz in dzs:
            pts.append([c[0] + r * np.cos(th), c[1] + r * np.sin(th), c[2] + dz])
    return pts


def _predicted_cloud(trackers, dets, p_d, plan_hi, hr, fov_r=10.0):
    """Render each mover's NEAR-TERM predicted footprint over [0, plan_hi] as occupancy (so EGO routes
    around the immediate future). plan_hi is the PLANNER-occupancy horizon, deliberately SHORTER than the
    certificate's trust window TAU: feeding the whole [0,TAU] sweep stacks every mover's future positions
    into a simultaneous phantom WALL that freezes the corridor (M1: 3 crossers -> first_optimize_step=0 ->
    spurious HOLDs). Here EGO only sees where each human will be over the next replan step(s); the growing
    deg-2 tube rho(t)=R+v_eff*(t+delta) + frequent replanning carry the rest of the window. Density ~1
    sample / 0.2 s so the thin sweep has no gaps. Falls back to the raw detection before the filter is ready."""
    pts = []
    n_samp = max(2, int(round(plan_hi / 0.2)) + 1)
    sample_ts = np.linspace(0.0, plan_hi, n_samp)
    for trk, det in zip(trackers, dets):
        if np.linalg.norm(np.asarray(det)[:2] - p_d[:2]) > fov_r:
            continue
        centres = trk.predict(sample_ts) if trk.ready else np.asarray(det, float)[None, :]
        for c in centres:
            pts.extend(_sphere_surface(c, hr))
    return np.asarray(pts, float) if pts else np.zeros((0, 3))


def main():
    DT = 0.30; HR = 0.3; D_SAFE = 0.8; TAU = 0.75; MEAS_NOISE = 0.07
    # --- risk dial (the certificate's tube rho(t) = r0 + V_EFF*(t + DELTA)) ---
    # V_EFF = how fast the tube grows to cover prediction error. 0 = trust the KF exactly over [0,TAU]
    #   (tightest/fastest/riskiest); >0 = inflate to absorb residual drift (the q_conformal stand-in until
    #   the real conformal quantile lands). 0.2 m/s grows the tube ~0.21 m over the window -> restores the
    #   d_safe margin the raw KF error (~0.12 m) was eating at M1 (clearance 0.677 -> >=0.8).
    # DELTA = perception->commit latency (tube already inflated by V_EFF*DELTA at t=0).
    # Shrink TAU (e.g. 0.30) + frequent replan = the "risky but fast" mode: certify only the next slice we
    # actually fly. Sound per-window, but needs a certified brake-to-stop fallback for recursive feasibility.
    # Both knobs are env-overridable so we can sweep conservative vs risky-fast without editing code.
    V_EFF = float(os.environ.get("EGO_VEFF", 0.2)); DELTA = DT
    TAU = float(os.environ.get("EGO_TAU", TAU))
    # PLANNER-occupancy horizon: how far ahead we render predicted humans as solid occupancy for EGO to
    # route around. Kept SHORTER than TAU so 3 crossers' futures don't stack into a corridor-freezing phantom
    # wall; the growing deg-2 tube + frequent replan cover the rest. Sweeping EGO_PLANHI in {0.30..0.75}:
    # 0.45 minimises HOLDs (8) at clearance 0.820 and fastest arrival; LONGER horizons freeze the corridor
    # MORE, and a frozen drone is then parked in a crosser's path -> a human walks into the held drone and
    # erodes TRUE clearance (0.60->0.126, all during HOLD, cert never flew). That is the core case for
    # go-around over HOLD: HOLD is NOT a safe fallback when obstacles move toward you. 1.5*DT by default.
    PLAN_HI = float(os.environ.get("EGO_PLANHI", 1.5 * DT))
    rng = np.random.default_rng(2026)
    ego = EGOPlanner(map_origin=(-30, -30, -1), map_size=(80, 80, 6), res=0.2, inflation=0.3)
    ego.set_params(max_vel=3.0, max_acc=6.0)
    start = np.array([0, 0, 1.5], float); goal = np.array([16, 0, 1.5], float)
    # same three humans crossing the corridor as ego_safe.py, so the two runs are directly comparable
    movers = [  # [centre, vel, radius]
        [np.array([6.0, 4.0, 1.5]), np.array([0.0, -1.0, 0.0]), HR],
        [np.array([10.0, -4.0, 1.5]), np.array([0.0, 1.0, 0.0]), HR],
        [np.array([13.0, 3.0, 1.5]), np.array([0.0, -0.8, 0.0]), HR],
    ]
    trackers = [MoverTracker(dt=DT, meas_noise=MEAS_NOISE) for _ in movers]

    p_d = start.copy(); v_d = np.zeros(3); a_d = np.zeros(3)
    min_clr = 1e18; max_jerk = 0.0; prev_v = 0.0; reached = False
    n_replan = 0; n_goaround = 0; n_hold = 0

    for tick in range(80):
        # 1. noisy detections of the true movers -> feed the Kalman trackers
        dets = [m[0] + rng.normal(0, MEAS_NOISE, 3) for m in movers]
        for trk, d in zip(trackers, dets):
            trk.update(d)
        # 2. feed EGO the PREDICTED swept footprint (so it plans around the future, not the present)
        cloud = _predicted_cloud(trackers, dets, p_d, PLAN_HI, HR)
        ego.update_cloud(cloud, p_d)
        # 3. EGO replans a smooth B-spline toward the goal, around the predicted occupancy
        ok = bool(ego.replan(p_d, v_d, a_d, goal))
        dur = ego.duration()
        n_replan += 1
        # 4. certify the committed B-spline against each mover's PREDICTED moving sphere (KF state)
        certified = ok and dur > 1e-3
        min_margin = float("inf")
        if certified:
            for trk in trackers:
                c0, v, a = trk.state()
                R = HR + D_SAFE                              # r0 = body+human margin; tube grows by V_EFF*(t+DELTA)
                cert, margin = ego.certify(obs_c0=c0, R=R, obs_vel=v, obs_acc=a,
                                           t_hi=TAU, v_eff=V_EFF, delta=DELTA)
                min_margin = min(min_margin, margin)
                if not cert:
                    certified = False
        # 5. certified -> fly the go-around one DT;  uncertified -> HOLD (fallback only)
        if certified:
            n_goaround += 1
            te = min(DT, max(dur - 1e-3, 0.0))
            r = ego.eval(te)
            if r is not None:
                p_new, v_new, a_new = (np.asarray(x, float) for x in r)
                jerk = abs(np.linalg.norm(v_new) - prev_v)
                max_jerk = max(max_jerk, jerk); prev_v = float(np.linalg.norm(v_new))
                p_d, v_d, a_d = p_new, v_new, a_new
        else:
            n_hold += 1
            v_d = np.zeros(3); a_d = np.zeros(3); prev_v = 0.0
        # advance the true crowd; track the TRUE executed clearance to humans
        for m in movers:
            min_clr = min(min_clr, float(np.linalg.norm(p_d - m[0]) - HR))
            m[0] = m[0] + m[1] * DT
        if np.linalg.norm(p_d[:2] - goal[:2]) < 0.8:
            reached = True; break

    print("=== EGO go-around (KF-predicted, route around the future) — headless closed loop ===")
    print(f"  replans={n_replan}  go-arounds(certified fly)={n_goaround}  held(fallback)={n_hold}  "
          f"reached={reached} @tick {tick}")
    print(f"  executed min clearance to humans = {min_clr:.3f} m  (d_safe={D_SAFE})")
    print(f"  executed max |dv|/tick (smoothness proxy) = {max_jerk:.3f} m/s")
    safe = min_clr >= -1e-6
    print("  SAFE: executed path never collided (layer gated every EGO commit)" if safe
          else "  UNSAFE: executed path collided (investigate)")
    return 0 if safe else 1


if __name__ == "__main__":
    sys.exit(main())
