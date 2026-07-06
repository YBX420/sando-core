"""ground_nav_env — the GENERALIZATION arm: same MetaUrban world, same certificates, DIFFERENT
embodiment. A sidewalk delivery robot (unicycle: cannot strafe, sensor cone welded to the body
heading) replaces the holonomic drone point of metaurban_nav_env. Everything else is deliberately
identical (scenes, corridor protocol, perception front-end, reward shape, DT) so a shield-on/off
campaign here reads as "the gate composes across embodiments", not "a different benchmark".

Embodiment:
  state  (x, y, theta, v)      v in [0, V_MAX] (forward-only; stop-and-turn is allowed)
  action [accel, omega] in [-1,1]^2  ->  a*A_MAX, w*W_MAX
  step   theta += w*DT;  v = clip(v + a*DT, 0, V_MAX);  p += v*[cos,sin](theta)*DT
Obs (5 + 5*K = 35), BODY frame: [goal_dir_body(2), dist/20, v/V_MAX, w/W_MAX] +
  per nearest-K tracks [rel_xy_body/10, rel_v_body/8, (r+0.3)/2].
v1 scope note: no walkable-region constraint (same open-corridor task as the drone arms).
"""
import numpy as np

from metaurban_nav_env import MetaUrbanNavEnv, DT, K_TRACKS

try:
    from gymnasium import spaces
except ImportError:
    from gym import spaces

V_MAX = 2.0          # m/s   sidewalk delivery robot cruise
A_MAX = 2.0          # m/s^2
W_MAX = 2.0          # rad/s
EP_LEN_MAX = 320     # slower body needs more ticks for the same 56 m corridor


class GroundNavEnv(MetaUrbanNavEnv):
    """Unicycle ground robot over the same live MetaUrban scenes."""

    def __init__(self, seed=0):
        super().__init__(seed=seed, max_vel=V_MAX, max_acc=A_MAX)
        self.w_max = W_MAX
        self.theta, self.w, self.spd = 0.0, 0.0, 0.0       # parent reset() calls _obs() before we
        self.observation_space = spaces.Box(-np.inf, np.inf, (5 + 5 * K_TRACKS,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (2,), np.float32)
        self._ep_len_max = EP_LEN_MAX

    # ---- episode plumbing --------------------------------------------------
    def reset(self, *, seed=None, options=None):
        obs, info = super().reset(seed=seed, options=options)
        d = self.goal - self.p
        self.theta = float(np.arctan2(d[1], d[0]))         # start facing the goal
        self.w = 0.0
        self.spd = 0.0                                     # scalar forward speed
        self.v = np.zeros(2)                               # world-frame velocity (kept for parity)
        self._hd = np.array([np.cos(self.theta), np.sin(self.theta)])
        return self._obs(), info

    # ---- perception: cone welded to the BODY heading -----------------------
    def _tracks(self):
        cyl = self._cylinders()
        hd = np.array([np.cos(self.theta), np.sin(self.theta)])
        trs = self.pfe.step(self.p, hd, cyl)
        return [t for t in trs if t.trk.ready], None

    def _obs(self):
        trs, _ = self._tracks()
        c, s = np.cos(-self.theta), np.sin(-self.theta)
        R = np.array([[c, -s], [s, c]])                    # world -> body
        d = self.goal - self.p
        dist = float(np.linalg.norm(d))
        db = R @ (d / max(dist, 1e-6))
        o = [db[0], db[1], min(dist / 20.0, 2.0), self.spd / V_MAX, self.w / W_MAX]
        trs = sorted(trs, key=lambda tr: np.linalg.norm(tr.xy - self.p))[:K_TRACKS]
        vw = self.spd * np.array([np.cos(self.theta), np.sin(self.theta)])
        for tr in trs:
            c0, v0, _ = tr.trk.state()
            rp = R @ (np.asarray(tr.xy, float) - self.p)
            rv = R @ (np.asarray(v0[:2], float) - vw)
            o += [rp[0] / 10.0, rp[1] / 10.0, rv[0] / 8.0, rv[1] / 8.0, (tr.r + 0.3) / 2.0]
        o += [0.0] * (5 * (K_TRACKS - len(trs)))
        return np.asarray(o, np.float32)

    # ---- unicycle step (parity with parent's step: same reward/terminals) --
    def step(self, action):
        for _ in range(3):                                 # MU_SUBSTEPS: world advances ~DT
            _o, _r, term, trunc, _i = self._mu.step([0.0, 0.0])
            if term or trunc:
                self._mu_done = True
                self._needs_mu_reset = True
                break
        self._movers = self._native_movers()

        a = float(np.clip(action[0], -1, 1)) * A_MAX
        self.w = float(np.clip(action[1], -1, 1)) * W_MAX
        self.theta = float((self.theta + self.w * DT + np.pi) % (2 * np.pi) - np.pi)
        self.spd = float(np.clip(self.spd + a * DT, 0.0, V_MAX))
        hd = np.array([np.cos(self.theta), np.sin(self.theta)])
        self.v = self.spd * hd                             # keep world-frame v coherent for shields
        self.p = self.p + self.v * DT
        self._hd = hd
        self.tick += 1

        clr, culprit = self._clearance_culprit()
        self.min_clr = min(self.min_clr, clr)
        d = float(np.linalg.norm(self.goal - self.p))
        r = 1.5 * (self._prev_d - d) - 0.02
        r -= 4.0 * float(np.log1p(np.exp(1.2 - clr)) - np.log1p(np.exp(1.2 - 3.0))) * 0.25
        self._prev_d = d
        terminated = truncated = False
        if clr < 0.0:
            r -= 60.0; terminated = True
        elif d < 0.6:
            r += 40.0; terminated = True
        elif self.tick >= self._ep_len_max or self._mu_done:
            truncated = True
        info = dict(min_clr=self.min_clr, reached=bool(d < 0.6), collided=bool(clr < 0.0))
        if clr < 0.0 and culprit is not None:
            cxy, cr, ccls, cv = culprit
            ready = [t for t in self.pfe.tracks if t.trk.ready]
            dists = [float(np.linalg.norm(np.asarray(t.xy, float) - cxy)) for t in ready]
            dmin = min(dists) if dists else float("inf")
            rel = cxy - self.p
            ang = float(np.degrees(np.arctan2(rel[1], rel[0]) - self.theta))
            ang = (ang + 180.0) % 360.0 - 180.0
            info["culprit"] = dict(cls=ccls, r=round(float(cr), 2),
                                   in_track=bool(dmin < float(cr) + 0.75),
                                   track_dist=(round(dmin, 2) if np.isfinite(dmin) else None),
                                   n_ready=len(ready), n_live=len(self.pfe.tracks),
                                   bearing_deg=round(ang, 1), in_cone=bool(abs(ang) <= 45.0),
                                   speed=round(float(np.linalg.norm(np.asarray(cv, float))), 2))
        return self._obs(), float(r), terminated, truncated, info


if __name__ == "__main__":
    import time
    t0 = time.time()
    env = GroundNavEnv(seed=7)
    for epi in range(2):
        obs, _ = env.reset()
        assert obs.shape == env.observation_space.shape, obs.shape
        R = 0.0
        for k in range(env._ep_len_max):
            d = env.goal - env.p
            bearing = (np.arctan2(d[1], d[0]) - env.theta + np.pi) % (2 * np.pi) - np.pi
            aa = np.array([1.0, np.clip(bearing / 0.6, -1, 1)])   # naive go-to-goal steering
            obs, rew, term, trunc, info = env.step(aa)
            R += rew
            if term or trunc:
                break
        print(f"[smoke] ep{epi} @{env.tick} reach={info['reached']} coll={info['collided']} "
              f"min_clr={info['min_clr']:.2f} ret={R:.0f} +{time.time()-t0:.1f}s", flush=True)
    env.close()
    print("[smoke] OK", flush=True)
