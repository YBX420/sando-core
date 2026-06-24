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

    def __init__(self, dt, meas_noise, q_jerk=2.0):
        self.dt = float(dt)
        self.R = float(meas_noise) ** 2
        self.F = np.array([[1, dt, dt * dt / 2.0],
                           [0, 1, dt],
                           [0, 0, 1.0]])
        self.Q = q_jerk * np.array([[dt**5 / 20, dt**4 / 8, dt**3 / 6],
                                    [dt**4 / 8,  dt**3 / 3, dt**2 / 2],
                                    [dt**3 / 6,  dt**2 / 2, dt]])
        self.H = np.array([1.0, 0.0, 0.0])
        self.x = None
        self.P = None

    def update(self, z):
        if self.x is None:                                   # initialise on first detection
            self.x = np.array([z, 0.0, 0.0]); self.P = np.diag([self.R, 1.0, 1.0]); return
        x = self.F @ self.x; P = self.F @ self.P @ self.F.T + self.Q      # predict
        y = z - self.H @ x; S = self.H @ P @ self.H.T + self.R           # innovation
        K = (P @ self.H) / S                                            # gain
        self.x = x + K * y; self.P = (np.eye(3) - np.outer(K, self.H)) @ P


class MoverTracker:
    """One CA-Kalman track for a single mover. Feed noisy detections; read back the certificate's
    centre polynomial (c0, v, a) and a predicted-position sampler for occupancy feeding."""

    def __init__(self, dt=0.30, meas_noise=0.07, a_max=2.0):
        self.fx = _AxisCAKalman(dt, meas_noise)
        self.fy = _AxisCAKalman(dt, meas_noise)
        self.z = None                                        # z held constant (people don't fly)
        self.a_max = float(a_max)                            # clamp filtered accel (KF a is noisy at long t)
        self.n = 0

    def update(self, det_xyz):
        det = np.asarray(det_xyz, float)
        self.fx.update(det[0]); self.fy.update(det[1]); self.z = float(det[2]); self.n += 1

    @property
    def ready(self):
        return self.n >= 2                                   # need >=2 obs before v/a are meaningful

    def _clamp_a(self, a):
        return float(np.clip(a, -self.a_max, self.a_max))

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
