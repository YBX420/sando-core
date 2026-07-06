"""ground_shield — the SAME conformal certificate, adapted to a non-holonomic body.

Reuses shield.py's calibrated tube verbatim (Q0, VEFF, TAU, sample grid): what changes is only the
EGO motion model inside the check. The drone shield certifies straight constant-velocity motion;
a unicycle moves on arcs, so action_safe becomes arc_safe: roll (v, omega) forward over [0, TAU]
and require every sample to clear every track's tube R(t) = r + r_body + margin + Q0 + VEFF*t.

Fallback set (the projection): a (speed x omega) grid of feasible arcs ranked by progress toward
the goal, then a straight hard-brake. Same pass/project/brake accounting as CertShield.
"""
import numpy as np

from shield import TUBE, _TS
from ground_nav_env import DT, V_MAX, A_MAX, W_MAX

try:
    import gymnasium as gym
except ImportError:
    import gym

_R_BODY = 0.25 + 0.10                                   # same inflation as shield.action_safe


def _arc(p, theta, v, w):
    """Ego positions at the tube sample times for constant (v, w) unicycle motion."""
    if abs(w) < 1e-6:
        hd = np.array([np.cos(theta), np.sin(theta)])
        return p[None, :] + v * _TS[:, None] * hd[None, :]
    ang = theta + w * _TS
    Rr = v / w
    return p[None, :] + Rr * np.stack([np.sin(ang) - np.sin(theta),
                                       -np.cos(ang) + np.cos(theta)], 1)


def arc_safe(p, theta, v, w, tracks):
    """True iff the (v, w) arc clears every track's conformal tube over [0, TAU]."""
    ego = _arc(np.asarray(p, float), float(theta), float(v), float(w))
    for (c0, vt, r) in tracks:
        obs = np.asarray(c0, float)[None, :] + np.asarray(vt, float)[None, :] * _TS[:, None]
        if np.any(np.linalg.norm(obs - ego, axis=1) < r + _R_BODY + TUBE):
            return False
    return True


class GroundCertShield:
    """Certificate gate for the unicycle. Returns (v_next, w_next, certified, intervened)."""

    def __init__(self):
        self.n_pass = self.n_project = self.n_brake = 0
        sc = (1.0, 0.6, 0.25)
        ws = (0.0, 0.5 * W_MAX, -0.5 * W_MAX, W_MAX, -W_MAX)
        self._grid = [(s, w) for s in sc for w in ws]

    def filter(self, p, theta, spd, a_cmd, w_cmd, tracks, goal_dir):
        v_want = float(np.clip(spd + np.clip(a_cmd, -1, 1) * A_MAX * DT, 0.0, V_MAX))
        w_want = float(np.clip(w_cmd, -1, 1)) * W_MAX
        if arc_safe(p, theta, v_want, w_want, tracks):
            self.n_pass += 1
            return v_want, w_want, True, False
        # projection: feasible arcs ranked by end-of-window progress along goal_dir
        cands = []
        for s, w in self._grid:
            vv = float(np.clip(spd + s * A_MAX * DT, 0.0, V_MAX)) if s > 0 else \
                float(np.clip(spd - A_MAX * DT, 0.0, V_MAX))
            end = _arc(np.asarray(p, float), theta, vv, w)[-1]
            cands.append((float(np.dot(end - p, goal_dir)), vv, w))
        cands.sort(key=lambda t: -t[0])
        for _, vv, w in cands:
            if arc_safe(p, theta, vv, w, tracks):
                self.n_project += 1
                return vv, w, True, True
        # nothing certifies: straight hard brake = uncertified exposure (counted, not hidden)
        self.n_brake += 1
        return float(max(0.0, spd - A_MAX * DT)), 0.0, False, True

    def stats(self):
        n = self.n_pass + self.n_project + self.n_brake
        return dict(ticks=n, passed=self.n_pass, projected=self.n_project, brake=self.n_brake,
                    uncert_frac=round(self.n_brake / max(1, n), 4))


class GroundShieldedEnv(gym.Wrapper):
    """Same shield-in-training shaping as v2's ShieldedEnv (-0.5 intervene / extra -1.0 uncert)."""

    def __init__(self, env):
        super().__init__(env)
        self.sh = GroundCertShield()

    def step(self, a):
        env = self.env
        trs, _ = env._tracks()
        tracks = []
        for tr in trs:
            c0, v0, _ = tr.trk.state()
            tracks.append((np.asarray(tr.xy, float), np.asarray(v0[:2], float), float(tr.r)))
        gd = env.goal - env.p
        gd = gd / max(float(np.linalg.norm(gd)), 1e-6)
        v_next, w_next, certified, intervened = self.sh.filter(
            env.p, env.theta, env.spd, float(a[0]), float(a[1]), tracks, gd)
        a_exec = np.array([np.clip((v_next - env.spd) / (DT * A_MAX), -1, 1),
                           np.clip(w_next / W_MAX, -1, 1)])
        obs, r, term, trunc, info = env.step(a_exec)
        if intervened:
            r -= 0.5
        if not certified:
            r -= 1.0
        return obs, float(r), term, trunc, info
