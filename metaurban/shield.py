"""shield — certificate shield for learned planners (the composition the paper wants:
LEARNED proposal + CERTIFIED gate = speed from data, guarantee from the theorem).

Semantics match the certified arms: each tick, the commanded motion is checked over the trust
window against every READY track inflated by the SAME conformal tube R(t) = r_obs + r_drone +
d_safe + q0 + v_eff*t (calib.json). Re-issued every DT -> the cadence composition applies to the
shielded policy exactly as to ours: P(collision | shield-passed ticks) <= eps. Ticks where NO
action passes fall back to a brake and are counted as uncertified exposure (the evade branch).

Check: drone flies constant velocity v over [0, TAU] (kinematic parity with the env); track flies
its KF CV prediction. Sampled at 0.05s (conservative wrt the analytic min by the sampling margin
folded into d_safe).
"""
import json
import os

import numpy as np

TAU = 0.75
_TS = np.arange(0.0, TAU + 1e-9, 0.05)

_here = os.path.dirname(os.path.abspath(__file__))
_calib = json.load(open(os.path.join(os.path.dirname(_here), "out", "conformal", "calib.json")))
_EPS = os.environ.get("SHIELD_EPS", "0.05")   # certified miss level: 0.05 default; "0.1" = the
#   looser CERTIFIED dial (q 1.054->0.466, ped keep-out ~1.7->1.1 m). Still theorem-backed --
#   never hand-edit q; pick a calibrated level.
_lv = _calib["groups"]["all"]["levels"][_EPS]
Q0, VEFF = float(_lv["q_conformal"]), float(_lv["v_eff"])
TUBE = Q0 + VEFF * _TS                                    # precomputed tube radius per sample time

# SHIELD_PERCLASS=1: per-class tubes via safety_layer.load_calib (same file/semantics as the EGO
# arm, incl. its static=(0.15, 0.0) stationarity rule). Default OFF = byte-identical legacy tube.
_PERCLASS = os.environ.get("SHIELD_PERCLASS", "0") == "1"
_CALIBV2 = os.environ.get("CALIB_V2", "0") == "1"
_AGE_MIN = 4
_TUBES, _TUBES_Y = {}, {}
if _CALIBV2:
    from safety_layer import load_calib_v2
    for _c, ent in load_calib_v2(eps=float(os.environ.get("CALIB_EPS", _EPS))).items():
        if _c == "_meta":
            continue
        _q, _ve = ent["mature"]; _TUBES[_c] = _q + _ve * _TS
        _qy, _gy = ent["young"]; _TUBES_Y[_c] = _qy + _gy * _TS
elif _PERCLASS:
    from safety_layer import load_calib
    for _c, (_q, _ve) in load_calib(eps=float(_EPS)).items():
        _TUBES[_c] = _q + _ve * _TS


def _tube_for(cls):
    return _TUBES.get(cls, _TUBES.get("_all", TUBE))


def action_safe(p, v_cmd, tracks):
    """tracks: list of (xy(2,), v(2,), r[, cls]). True iff constant-velocity motion clears every
    tube. With SHIELD_PERCLASS=1 the 4th element picks the class tube; statics are additionally
    predicted STATIONARY (their KF v is measurement noise, not motion)."""
    for t in tracks:
        c0, vt, r = t[0], t[1], t[2]
        cls = t[3] if len(t) > 3 else None
        age = t[4] if len(t) > 4 else None
        if _CALIBV2 and cls is not None:
            young = age is not None and age < _AGE_MIN
            tube = (_TUBES_Y if young else _TUBES).get(cls, _TUBES.get("_all", TUBE))
            vt = np.zeros(2) if (young or cls == "static") else np.asarray(vt, float)
        else:
            tube = _tube_for(cls) if (_PERCLASS and cls is not None) else TUBE
            vt = np.zeros(2) if (_PERCLASS and cls == "static") else np.asarray(vt, float)
        rel0 = np.asarray(c0, float) - np.asarray(p, float)
        relv = vt - np.asarray(v_cmd, float)
        d = np.linalg.norm(rel0[None, :] + relv[None, :] * _TS[:, None], axis=1)
        if np.any(d < r + 0.25 + 0.10 + tube):
            return False
    return True


class CertShield:
    """Wraps a policy action. Returns (v_next, certified, intervened)."""

    def __init__(self, max_vel, max_acc, dt):
        self.max_vel, self.max_acc, self.dt = float(max_vel), float(max_acc), float(dt)
        ang = np.linspace(0, 2 * np.pi, 16, endpoint=False)
        self._dirs = np.stack([np.cos(ang), np.sin(ang)], 1)
        self.n_pass = self.n_project = self.n_brake = 0

    def _clamp(self, v):
        s = float(np.linalg.norm(v))
        return v * (self.max_vel / s) if s > self.max_vel else v

    def filter(self, p, v, a_cmd, tracks, goal_dir):
        v_want = self._clamp(v + np.clip(a_cmd, -1, 1) * self.max_acc * self.dt)
        if action_safe(p, v_want, tracks):
            self.n_pass += 1
            return v_want, True, False
        # project: certified fallback set -- 16 directions x 2 speeds + hard brake, ranked by
        # progress along goal_dir (fastest-safe, same spirit as the maneuver tournament)
        cands = []
        for sc in (1.0, 0.5):
            for u in self._dirs:
                vv = self._clamp(v + u * self.max_acc * self.dt * sc)
                cands.append(vv)
        cands.sort(key=lambda vv: -float(np.dot(vv, goal_dir)))
        for vv in cands:
            if action_safe(p, vv, tracks):
                self.n_project += 1
                return vv, True, True
        # nothing certifies: brake hard = the uncertified evade branch (counted, not hidden)
        self.n_brake += 1
        vb = v * max(0.0, 1.0 - self.max_acc * self.dt / max(np.linalg.norm(v), 1e-6))
        return vb, False, True

    def stats(self):
        n = self.n_pass + self.n_project + self.n_brake
        return dict(ticks=n, passed=self.n_pass, projected=self.n_project, brake=self.n_brake,
                    uncert_frac=round(self.n_brake / max(1, n), 4))
