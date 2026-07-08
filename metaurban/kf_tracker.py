"""kf_tracker — constant-acceleration Kalman filter per mover, shaped for the safety certificate.

WHY (the whole point of going from HOLD to go-around):
  EGO's grid has no time axis, so feeding the obstacle's CURRENT position makes EGO route around where
  the human IS, while the certificate checks where the human WILL BE  -> never certifies -> HOLD.
  Instead we PREDICT each mover forward with a CA-Kalman filter and (a) render the predicted swept
  footprint into EGO's cloud so it plans a go-AROUND, and (b) hand the certificate the predicted centre
  polynomial it already wants:  c(t) = c0 + v*t + 1/2*a*t^2  (degree <= 2 = exactly what bernstein_cert eats).

This is the deployable cousin of conformal/kf_predictor_experiment.py (which only measured residuals).
xy run a full CA filter (humans maneuver in the ground plane); z is held at the latest detection (people
don't fly), so vz = az = 0 and the certificate's z-extent stays the body/standoff term.
"""
import numpy as np


class _AxisCAKalman:
    """Per-axis constant-acceleration Kalman filter. State x=[p, v, a]; white-jerk process noise."""

    def __init__(self, dt, meas_noise, q_jerk=2.0, a_prior=2.0):
        self.dt = float(dt)                                  # NOMINAL cadence (default when no dt passed)
        self.q_jerk = float(q_jerk)
        self.R = float(meas_noise) ** 2
        self.F, self.Q = self._mats(self.dt)                 # cached nominal-dt matrices (back-compat)
        self.H = np.array([1.0, 0.0, 0.0])
        self.x = None
        self.P = None
        self._z0 = None          # first detection, kept for two-point differencing on the second
        self._gap = 0.0          # ELAPSED coast time since first detection (dt_eff bookkeeping across
                                 # misses; replaces the old _steps*dt count so VARIABLE-dt ticks stay exact)
        self._a_var = float(a_prior) ** 2
        self.last_nis = None     # innovation^2/S of the latest measurement update (self-check instrument)

    def _mats(self, dt):
        """Exact CA discretization for an arbitrary step (white-jerk Q is the exact Van Loan integral,
        so composing three 0.1 s steps == one 0.3 s step -- variable cadence stays consistent)."""
        F = np.array([[1, dt, dt * dt / 2.0],
                      [0, 1, dt],
                      [0, 0, 1.0]])
        Q = self.q_jerk * np.array([[dt**5 / 20, dt**4 / 8, dt**3 / 6],
                                    [dt**4 / 8,  dt**3 / 3, dt**2 / 2],
                                    [dt**3 / 6,  dt**2 / 2, dt]])
        return F, Q

    def update(self, z, dt=None):
        step = self.dt if dt is None else float(dt)
        if self.x is None:                                   # first detection: position only; v/a still unknown.
            # x=[z,0,0] keeps state() usable for the not-ready fallback, but the WIDE v/a variance says "unknown",
            # not "v=0" -- the old P=diag([R,1,1]) asserted a confident zero velocity, so a fast target's second
            # update arrived with NIS ~ v^2*dt^2/ (R+1e-ish) (~56 for a vehicle): an absurd prior, not information.
            self.x = np.array([z, 0.0, 0.0]); self.P = np.diag([self.R, 100.0, self._a_var])
            self._z0 = float(z); self._gap = 0.0; return
        if self._z0 is not None:                             # second detection: TWO-POINT DIFFERENCING re-init
            dte = self._gap + step                           # coasted time + this update's interval = dt_eff
            v0 = (z - self._z0) / dte
            self.x = np.array([z, v0, 0.0])
            self.P = np.array([[self.R,       self.R / dte,          0.0],
                               [self.R / dte, 2.0 * self.R / dte**2, 0.0],
                               [0.0,          0.0,                   self._a_var]])
            self._z0 = None; self.last_nis = None; return    # exact re-init: no meaningful innovation this tick
        F, Q = (self.F, self.Q) if step == self.dt else self._mats(step)
        x = F @ self.x; P = F @ self.P @ F.T + Q                          # predict
        y = z - self.H @ x; S = self.H @ P @ self.H.T + self.R           # innovation
        self.last_nis = float(y * y / S)                                 # NIS self-check (should be ~chi2_1)
        K = (P @ self.H) / S                                            # gain
        self.x = x + K * y; self.P = (np.eye(3) - np.outer(K, self.H)) @ P

    def coast(self, dt=None):
        """Pure time-update (NO measurement): advance the state and GROW the covariance. Used when the mover is
        OUT of the FOV cone so the filter keeps extrapolating with rising uncertainty instead of FREEZING at the
        last detection. Exactly one coast-or-update per tick keeps the dt bookkeeping correct, so on re-acquisition
        the innovation is small (no stale single-dt jump after k missed ticks)."""
        if self.x is None:
            return
        step = self.dt if dt is None else float(dt)
        self._gap += step
        F, Q = (self.F, self.Q) if step == self.dt else self._mats(step)
        self.x = F @ self.x; self.P = F @ self.P @ F.T + Q


class MoverTracker:
    """One CA-Kalman track for a single mover. Feed noisy detections; read back the certificate's
    centre polynomial (c0, v, a) and a predicted-position sampler for occupancy feeding."""

    def __init__(self, dt=0.30, meas_noise=0.07, a_max=2.0):
        self.fx = _AxisCAKalman(dt, meas_noise)
        self.fy = _AxisCAKalman(dt, meas_noise)
        self.z = None                                        # z held constant (people don't fly)
        self.a_max = float(a_max)                            # clamp filtered accel (KF a is noisy at long t)
        self.n = 0
        self.miss = 0                                        # consecutive ticks COASTED with no detection (out of FOV)
        self.r_obs = 0.0                                     # last-seen body radius (stashed so out-of-cone memory needs no GT)
        self.d_safe = 0.0                                    # last-seen per-class standoff (ditto)

    def update(self, det_xyz, dt=None):
        det = np.asarray(det_xyz, float)
        self.fx.update(det[0], dt); self.fy.update(det[1], dt); self.z = float(det[2]); self.n += 1
        self.miss = 0                                        # detected this tick -> reset the out-of-FOV miss counter

    def coast(self, dt=None):
        """Out-of-FOV time update: extrapolate both ground-plane axes one dt and GROW covariance; count the miss.
        Centre + velocity keep moving along the last CA estimate, P inflates -> the cert's memory keep-out grows."""
        self.fx.coast(dt); self.fy.coast(dt); self.miss += 1

    @property
    def pos_sigma(self):
        """1-sigma horizontal position uncertainty (m) from the filter covariance: ~0 just after a detection, grows
        while coasting -> drives the growing keep-out tube for a remembered (out-of-cone) mover."""
        px = float(self.fx.P[0, 0]) if self.fx.P is not None else 0.0
        py = float(self.fy.P[0, 0]) if self.fy.P is not None else 0.0
        return float(np.sqrt(max(px, 0.0) + max(py, 0.0)))

    @property
    def sigma_v(self):
        """1-sigma horizontal velocity uncertainty (m/s) from the filter covariance. Small = converged;
        big (fresh birth / long coast) = the velocity is a guess -- certifying its MOVING prediction
        is corridor poison, the frozen young plate is the honest response."""
        vx = float(self.fx.P[1, 1]) if self.fx.P is not None else 1e6
        vy = float(self.fy.P[1, 1]) if self.fy.P is not None else 1e6
        return float(np.sqrt(max(vx, vy, 0.0)))

    @property
    def ready(self):
        return self.n >= 2                                   # need >=2 obs before v/a are meaningful

    @property
    def nis(self):
        """Latest measurement-update NIS, max over the two ground axes (each ~chi2_1 if the filter is
        consistent; None right after (re-)init). A sustained NIS >> 1 = model mismatch (e.g. a CA-violating
        accelerating vehicle) -- the self-check dial the deployment can log or alarm on."""
        vals = [f.last_nis for f in (self.fx, self.fy) if f.last_nis is not None]
        return max(vals) if vals else None

    def _clamp_a(self, a):
        return float(np.clip(a, -self.a_max, self.a_max))

    @property
    def vel_smooth(self):
        """EMA-smoothed velocity for PLANNER FEED ONLY (VF_EMA alpha; the certificate keeps the raw
        KF state -- calibration was harvested against it). Kills the per-tick feed-cylinder wiggle
        that turns measurement noise into reference jitter."""
        import os as _os
        a = float(_os.environ.get("VF_EMA", "0"))
        v = np.array([self.fx.x[1], self.fy.x[1], 0.0])
        if a <= 0:
            return v
        prev = getattr(self, "_vs", None)
        self._vs = v.copy() if prev is None else (1 - a) * prev + a * v
        return self._vs.copy()

    def state(self):
        """Certificate input: centre c(t) = c0 + v*t + 1/2*a*t^2. Returns (c0[3], v[3], a[3])."""
        c0 = np.array([self.fx.x[0], self.fy.x[0], self.z])
        v = np.array([self.fx.x[1], self.fy.x[1], 0.0])
        a = np.array([self._clamp_a(self.fx.x[2]), self._clamp_a(self.fy.x[2]), 0.0])
        return c0, v, a

    def predict(self, ts, model="ca"):
        """Predicted centres at time offsets ts (1D array) -> (len(ts), 3).
          model="ca": full constant-acceleration polynomial c0 + v*t + 1/2 a*t^2 (the cert's polynomial).
          model="cv": constant-velocity c0 + v*t (drops the noisy filtered accel; for jerky pedestrians the CA
                      accel term overshoots, so CV often has a SMALLER conformal residual -> tighter keep-out).
        Conformal calibration (predictor_compare.py) picks the model that minimises the certified keep-out."""
        c0, v, a = self.state()
        if model == "cv":
            a = np.zeros(3)
        ts = np.asarray(ts, float).reshape(-1, 1)
        return c0[None, :] + ts * v[None, :] + 0.5 * ts**2 * a[None, :]


if __name__ == "__main__":
    # self-test: a human walking +y at 1 m/s, observed with noise -> KF should recover v_y ~ 1 and predict ahead
    rng = np.random.default_rng(0)
    trk = MoverTracker(dt=0.30, meas_noise=0.07)
    true = np.array([5.0, 0.0, 1.5]); vel = np.array([0.0, 1.0, 0.0])
    for _ in range(8):
        trk.update(true + rng.normal(0, 0.07, 3)); true = true + vel * 0.30
    c0, v, a = trk.state()
    pred = trk.predict([0.0, 0.75])
    print(f"[kf] recovered v = {np.round(v, 2)} (truth [0,1,0])  a = {np.round(a, 2)}")
    print(f"[kf] centre now = {np.round(c0, 2)}  predicted @0.75s = {np.round(pred[1], 2)}")
    ok = abs(v[1] - 1.0) < 0.25 and pred[1][1] > c0[1] + 0.5
    print("[kf] PASS" if ok else "[kf] FAIL (check filter)")

    # two-point init regression: a FAST straight target (vehicle-like 8 m/s). The old v=0-confident init made
    # the first real update arrive with NIS ~ 50+; two-point differencing must recover v immediately and keep
    # every subsequent NIS sane (~chi2_1, generously < 9 == 3-sigma).
    trk2 = MoverTracker(dt=0.30, meas_noise=0.07)
    true = np.array([0.0, 0.0, 1.5]); vel = np.array([8.0, 0.0, 0.0])
    nises = []
    for _ in range(6):
        trk2.update(true + rng.normal(0, 0.07, 3)); true = true + vel * 0.30
        if trk2.nis is not None: nises.append(trk2.nis)
    _, v2, _ = trk2.state()
    print(f"[kf] fast target: v recovered = {v2[0]:.2f} (truth 8.0)  NIS trail = {np.round(nises, 2)}")
    ok2 = abs(v2[0] - 8.0) < 1.0 and (max(nises) < 9.0 if nises else False)
    print("[kf] two-point init PASS" if ok2 else "[kf] two-point init FAIL")

    # VARIABLE-dt regressions (proximity-triggered cadence support, 2026-07-06):
    # (a) default-path invariance: update(z) == update(z, dt=nominal) bit-for-bit;
    # (b) exact composition: three 0.1 s coasts == one 0.3 s coast (white-jerk Q is the exact
    #     discretization, so subdividing a step must not change the state OR the covariance).
    rng = np.random.default_rng(7)
    ta, tb = MoverTracker(dt=0.30), MoverTracker(dt=0.30)
    true = np.array([2.0, -1.0, 1.5]); vel = np.array([1.2, 0.6, 0.0])
    for _ in range(4):
        z = true + rng.normal(0, 0.07, 3)
        ta.update(z); tb.update(z, dt=0.30)
        true = true + vel * 0.30
    ok3 = np.allclose(ta.fx.x, tb.fx.x, atol=0) and np.allclose(ta.fx.P, tb.fx.P, atol=0)
    print("[kf] default-path invariance PASS" if ok3 else "[kf] default-path invariance FAIL")
    import copy as _copy
    tc = _copy.deepcopy(ta)
    ta.coast()                                    # one 0.3 s coast
    for _ in range(3):
        tc.coast(dt=0.10)                         # three 0.1 s coasts
    ok4 = np.allclose(ta.fx.x, tc.fx.x, atol=1e-12) and np.allclose(ta.fx.P, tc.fx.P, atol=1e-12)
    print("[kf] 3x0.1s == 1x0.3s composition PASS" if ok4 else "[kf] composition FAIL")
