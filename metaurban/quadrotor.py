"""quadrotor.py — compact but physical quadrotor flight model + tracking controller.

Flies the rendered drone from SANDO's (pos, vel, accel) set-points through real multicopter dynamics
instead of teleporting it to the set-point:

  outer-loop PD on position/velocity  ->  desired world acceleration
  desired thrust vector  f = m(a_des + g·ẑ)   (tilt-limited)
  inner-loop first-order attitude lag of the body-z toward f̂   (the fast rate/attitude controller)
  rigid-body translation:  v̇ = (T/m)·n − g·ẑ − drag·v,  with thrust T along the *current* body-z n

So the drone must TILT to accelerate, carries momentum, and is bounded by thrust-to-weight and a max
tilt angle — it reads as a quadrotor, not a kinematic point. `tilt_deg()` returns (pitch, roll) for the
visual mesh.  This is the local stand-in for a full PX4 SITL flight stack (same set-point interface),
so swapping in PX4/MAVSDK later only changes who integrates the dynamics.
"""
import numpy as np

G = 9.81


class Quadrotor:
    def __init__(self, mass=0.95, kp_pos=10.0, kd_pos=6.2, tau_att=0.06, drag=0.12,
                 max_tilt_deg=55.0, twr=3.2, yaw_tau=0.22):
        self.m = float(mass)
        self.kp = float(kp_pos); self.kd = float(kd_pos)
        self.tau = float(tau_att); self.drag = float(drag)
        self.max_tilt = np.radians(max_tilt_deg)
        self.Tmax = float(twr) * self.m * G          # thrust-to-weight ratio
        self.yaw_tau = float(yaw_tau)
        self.reset(np.zeros(3))

    def reset(self, p0, yaw=0.0):
        self.p = np.asarray(p0, float).copy()
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.n = np.array([0.0, 0.0, 1.0])           # body-z (thrust direction), starts level
        self.yaw = float(yaw)

    def step(self, p_ref, v_ref, a_ref, dt, yaw_ref=None):
        p_ref = np.asarray(p_ref, float); v_ref = np.asarray(v_ref, float); a_ref = np.asarray(a_ref, float)
        # --- outer loop: desired world acceleration --------------------------------------------------
        a_des = self.kp * (p_ref - self.p) + self.kd * (v_ref - self.v) + a_ref
        f_des = self.m * (a_des + np.array([0.0, 0.0, G]))    # thrust vector incl. gravity compensation
        # tilt limit: cap horizontal thrust so the vector leans <= max_tilt from vertical
        fz = max(f_des[2], 0.15 * self.m * G)
        max_h = np.tan(self.max_tilt) * fz
        hxy = f_des[:2]; hn = float(np.linalg.norm(hxy))
        if hn > max_h: f_des[:2] = hxy / hn * max_h
        n_des = f_des / (np.linalg.norm(f_des) + 1e-9)
        # --- inner loop: first-order attitude lag toward desired body-z ------------------------------
        alpha = 1.0 - np.exp(-dt / self.tau)
        self.n = self.n + alpha * (n_des - self.n)
        self.n /= (np.linalg.norm(self.n) + 1e-9)
        T = float(np.clip(np.dot(f_des, self.n), 0.0, self.Tmax))
        # --- rigid-body translational dynamics ------------------------------------------------------
        self.a = (T / self.m) * self.n - np.array([0.0, 0.0, G]) - self.drag * self.v
        self.v = self.v + self.a * dt
        self.p = self.p + self.v * dt
        if self.p[2] < 0.05:                          # don't sink through the ground
            self.p[2] = 0.05
            if self.v[2] < 0.0: self.v[2] = 0.0
        # --- yaw: face yaw_ref if given (e.g. toward the goal/route so the onboard view looks FORWARD,
        #         not sideways while crabbing for avoidance); else track travel heading (lagged) -------
        yd = None
        if yaw_ref is not None:
            yd = float(yaw_ref)
        elif float(np.linalg.norm(v_ref[:2])) > 0.3:
            yd = float(np.arctan2(v_ref[1], v_ref[0]))
        if yd is not None:
            dyaw = (yd - self.yaw + np.pi) % (2 * np.pi) - np.pi
            self.yaw += (1.0 - np.exp(-dt / self.yaw_tau)) * dyaw
        return self.p, self.v

    def tilt_deg(self):
        """Return (panda_P, panda_R) deg for the drone mesh whose NOSE is +X (forward=heading).
        Panda P rotates about local +X (bank), R about local +Y/left (nose pitch). From body-z `n`, yaw."""
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        fwd = c * self.n[0] + s * self.n[1]           # thrust lean along forward (+X)
        right = s * self.n[0] - c * self.n[1]         # thrust lean toward body-right
        nz = max(self.n[2], 1e-3)
        nose_pitch = -np.degrees(np.arctan2(fwd, nz))  # leaning forward -> nose down  (panda R, about +Y)
        bank = np.degrees(np.arctan2(right, nz))       # leaning right -> bank right    (panda P, about +X)
        return float(bank), float(nose_pitch)
