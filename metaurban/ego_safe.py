"""EgoSafe — EGO-Planner (smooth) + OUR S3 certified safety layer (judge / RTA gate).

The planner-agnostic safety suite wrapped around EGO:
  EGO plans a smooth cubic B-spline  ->  we certify the COMMITTED B-spline against every detected mover's
  conformal tube with the continuous-time conformal-Bernstein deficit (P(collision)<=eps, no sampling).
  Uncertified  ->  the RTA verdict is UNSAFE; the executor HOLDS the last-good / yields (no line-correction,
  per spec: monitor node + backup, not in the policy path).  This is the main thesis demonstrated on EGO.

Run headless:  python metaurban/ego_safe.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from ego_bridge import EGOPlanner


class EgoSafe:
    def __init__(self, map_origin=(-30, -30, -1), map_size=(80, 80, 6), res=0.2, inflation=0.3,
                 max_vel=3.0, max_acc=6.0, d_safe=0.8, q_conformal=0.0, eps_track=0.0, tau_trust=0.75):
        self.ego = EGOPlanner(map_origin=map_origin, map_size=map_size, res=res, inflation=inflation)
        self.ego.set_params(max_vel=max_vel, max_acc=max_acc)
        self.d_safe = d_safe; self.q_conformal = q_conformal; self.eps_track = eps_track
        self.tau_trust = tau_trust

    def update_cloud(self, cloud, cam):
        self.ego.update_cloud(cloud, cam)

    def replan(self, p, v, a, goal, movers):
        """movers: list of (centre(3,), vel(3,), radius). Returns a verdict dict (the RTA judgement)."""
        ok = bool(self.ego.replan(p, v, a, goal))
        dur = self.ego.duration()
        verdict = {"ego_ok": ok, "duration": dur, "certified": ok and dur > 1e-3,
                   "min_margin": float("inf"), "unsafe": []}
        if ok and dur > 1e-3:
            for (c0, mv, r) in movers:
                R = float(r) + self.d_safe + self.q_conformal + self.eps_track
                cert, margin = self.ego.certify(obs_c0=c0, R=R, obs_vel=mv, t_hi=self.tau_trust)
                verdict["min_margin"] = min(verdict["min_margin"], margin)
                if not cert:
                    verdict["certified"] = False
                    verdict["unsafe"].append((tuple(np.round(c0, 2)), round(margin, 3)))
        return verdict

    def eval(self, t):
        return self.ego.eval(t)

    def duration(self):
        return self.ego.duration()


# ----------------------------- headless closed-loop demo -----------------------------
def _mover_cloud(movers, p_d, fov_r=10.0):
    """occupied points sampled on each mover's surface (what EGO's depth-FOV would see), within range."""
    pts = []
    for (c0, _v, r) in movers:
        if np.linalg.norm(np.asarray(c0)[:2] - p_d[:2]) > fov_r:
            continue
        for th in np.linspace(0, 2 * np.pi, 10, endpoint=False):
            for dz in (-0.3, 0.0, 0.3):
                pts.append([c0[0] + r * np.cos(th), c0[1] + r * np.sin(th), c0[2] + dz])
    return np.asarray(pts, float) if pts else np.zeros((0, 3))


def main():
    DT = 0.30; HR = 0.3; D_SAFE = 0.8
    es = EgoSafe(max_vel=3.0, max_acc=6.0, d_safe=D_SAFE, q_conformal=0.0, tau_trust=0.75)
    start = np.array([0, 0, 1.5], float); goal = np.array([16, 0, 1.5], float)
    # three humans crossing the corridor at different times/places
    movers = [  # [centre, vel, radius]
        [np.array([6.0, 4.0, 1.5]), np.array([0.0, -1.0, 0.0]), HR],
        [np.array([10.0, -4.0, 1.5]), np.array([0.0, 1.0, 0.0]), HR],
        [np.array([13.0, 3.0, 1.5]), np.array([0.0, -0.8, 0.0]), HR],
    ]
    p_d = start.copy(); v_d = np.zeros(3); a_d = np.zeros(3)
    min_clr = 1e18; max_jerk = 0.0; prev_v = 0.0; reached = False
    n_replan = 0; n_cert_ok = 0; n_hold = 0
    for tick in range(80):
        cloud = _mover_cloud(movers, p_d)
        es.update_cloud(cloud, p_d)
        mlist = [(m[0].tolist(), m[1].tolist(), m[2]) for m in movers]
        vd = es.replan(p_d, v_d, a_d, goal, mlist)
        n_replan += 1
        if vd["certified"]:
            n_cert_ok += 1
            # execute the certified EGO B-spline for one DT (closed loop)
            dur = vd["duration"]; te = min(DT, max(dur - 1e-3, 0.0))
            r = es.eval(te)
            if r is not None:
                p_new, v_new, a_new = (np.asarray(x, float) for x in r)
                jerk = abs(np.linalg.norm(v_new) - prev_v)        # executed |dv| (smoothness proxy)
                max_jerk = max(max_jerk, jerk); prev_v = float(np.linalg.norm(v_new))
                p_d, v_d, a_d = p_new, v_new, a_new
        else:
            n_hold += 1                                            # RTA: not certified -> HOLD (no line-correction)
            v_d = np.zeros(3); a_d = np.zeros(3); prev_v = 0.0
        # advance the crowd; track the TRUE clearance of the executed drone state
        for m in movers:
            min_clr = min(min_clr, float(np.linalg.norm(p_d - m[0]) - HR))
            m[0] = m[0] + m[1] * DT
        if np.linalg.norm(p_d[:2] - goal[:2]) < 0.8:
            reached = True; break

    print("=== EgoSafe (EGO smooth + our S3 certified safety layer) — headless closed loop ===")
    print(f"  replans={n_replan}  certified={n_cert_ok}  held(uncertified->RTA)={n_hold}  reached={reached}")
    print(f"  executed min clearance to humans = {min_clr:.3f} m  (d_safe={D_SAFE}, body+human margin folds into R)")
    print(f"  executed max |dv|/tick (smoothness proxy) = {max_jerk:.3f} m/s")
    # the layer's job: the EXECUTED path is only ever the certified EGO trajectory or a hold -> never a
    # committed-into-collision. Safety check: executed min clearance must stay >= 0 (no body collision).
    safe = min_clr >= -1e-6
    print("  SAFE: executed path never collided (layer gated every EGO commit)" if safe
          else "  UNSAFE: executed path collided (investigate)")
    return 0 if safe else 1


if __name__ == "__main__":
    sys.exit(main())
