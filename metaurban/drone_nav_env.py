"""drone_nav_env — Gymnasium env over the SAME frozen world the benchmark uses, so a learned
policy becomes a fair benchmark arm (arm name: ppo).

Parity with the certified arms, deliberately:
  * observations come from the SAME realistic perception front-end (cone/occlusion/miss/NN
    association) -- the policy is NOT given ground truth, exactly like ours_real;
  * dynamics: kinematic point with accel/vel caps (matches the headless ours arm; the quad-tracking
    variant can be added the same way the dyn arms were);
  * scenarios: scenarios/full/*.json via scenario_lib, same spacing/legality/contested worlds;
  * episode protocol: DT=0.3, reach/collide/timeout terminals, min_clr bookkeeping for the table.

Obs (4 + 6*5 = 34): [goal_dir(2), dist/20, |v|/vmax] + per nearest-6 tracks
  [rel_xy/10 (2), rel_v/8 (2), (r+0.3)/2 (1)] padded with zeros.
Action: accel command in [-1,1]^2 * max_acc; v clamped to max_vel.
Reward: 1.5*progress(m) - 0.02/step - 4*softplus(1.2-clr) crowding shaping
        + 40 reach  - 60 collision.
"""
import glob
import os

import numpy as np

import scenario_lib as SLB
import replay_core as RC
from perception import PerceptionFrontEnd, PerceptCfg

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:                                     # gym fallback
    import gym
    from gym import spaces

DT = 0.30
K_TRACKS = 6


class DroneNavEnv(gym.Env):
    metadata = {}

    def __init__(self, scenario_glob="scenarios/full/*.json", seed=0, max_vel=3.0, max_acc=6.0):
        here = os.path.dirname(os.path.abspath(__file__))
        self.files = sorted(glob.glob(os.path.join(here, scenario_glob)))
        assert self.files, f"no scenarios at {scenario_glob}"
        self._cache = {}                                 # file -> (movers_raw, ep) compiled once
        self.rng = np.random.default_rng(seed)
        self.max_vel, self.max_acc = float(max_vel), float(max_acc)
        self.observation_space = spaces.Box(-np.inf, np.inf, (4 + 5 * K_TRACKS,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (2,), np.float32)
        self._ep_len_max = 240

    # ---- episode plumbing -------------------------------------------------
    def _load(self, f):
        if f not in self._cache:
            scn = SLB.load(f)
            self._cache[f] = (RC.Movers(SLB.to_movers_raw(scn)), SLB.to_episode(scn))
        return self._cache[f]

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        f = self.files[int(self.rng.integers(len(self.files)))]
        self.movers, ep = self._load(f)
        self.org = 0.5 * (np.asarray(ep["start"][:2]) + np.asarray(ep["goal"][:2]))
        self.p = np.asarray(ep["start"][:2], float) - self.org
        self.goal = np.asarray(ep["goal"][:2], float) - self.org
        self.v = np.zeros(2)
        self.t = float(ep.get("t0", 0.0))
        self.tick = 0
        self.min_clr = 1e18
        self.pfe = PerceptionFrontEnd(PerceptCfg.from_env(dt=DT),
                                      seed=int(self.rng.integers(1 << 30)))
        self._hd = self.goal - self.p
        self._prev_d = float(np.linalg.norm(self.goal - self.p))
        return self._obs(), {}

    # ---- perception (parity with ours_real) --------------------------------
    def _tracks(self):
        idx = [i for i in range(len(self.movers.m)) if self.movers.present(i, self.t)]
        gt = [(self.movers.pos(i, self.t) - self.org, self.movers.m[i]["r"],
               self.movers.m[i]["h"], self.movers.m[i]["cls"]) for i in idx]
        hd = self._hd if np.linalg.norm(self._hd) > 0.2 else (self.goal - self.p)
        trs = self.pfe.step(self.p, hd, gt)
        return [t for t in trs if t.trk.ready], idx

    def _obs(self):
        trs, _ = self._tracks()
        d = self.goal - self.p
        dist = float(np.linalg.norm(d))
        o = [d[0] / max(dist, 1e-6), d[1] / max(dist, 1e-6), min(dist / 20.0, 2.0),
             float(np.linalg.norm(self.v)) / self.max_vel]
        trs = sorted(trs, key=lambda tr: np.linalg.norm(tr.xy - self.p))[:K_TRACKS]
        for tr in trs:
            c0, v0, _ = tr.trk.state()
            o += [(tr.xy[0] - self.p[0]) / 10.0, (tr.xy[1] - self.p[1]) / 10.0,
                  (v0[0] - self.v[0]) / 8.0, (v0[1] - self.v[1]) / 8.0, (tr.r + 0.3) / 2.0]
        o += [0.0] * (5 * (K_TRACKS - len(trs)))
        return np.asarray(o, np.float32)

    # ---- world truth for reward/terminals (evaluator-only, not observed) ---
    def _clearance(self):
        best = 1e18
        for i in range(len(self.movers.m)):
            if not self.movers.present(i, self.t):
                continue
            c = self.movers.pos(i, self.t) - self.org
            best = min(best, float(np.linalg.norm(c - self.p)) - self.movers.m[i]["r"] - 0.25)
        return best

    def step(self, action):
        a = np.clip(np.asarray(action, float), -1, 1) * self.max_acc
        self.v = self.v + a * DT
        sp = float(np.linalg.norm(self.v))
        if sp > self.max_vel:
            self.v *= self.max_vel / sp
        if sp > 0.3:
            self._hd = self.v.copy()
        self.p = self.p + self.v * DT
        self.t += DT
        self.tick += 1
        clr = self._clearance()
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
        elif self.tick >= self._ep_len_max:
            truncated = True
        info = dict(min_clr=self.min_clr, reached=bool(d < 0.6), collided=bool(clr < 0.0))
        return self._obs(), float(r), terminated, truncated, info


if __name__ == "__main__":
    env = DroneNavEnv()
    obs, _ = env.reset(seed=7)
    R = 0.0
    for k in range(300):
        d = env.goal - env.p
        a = d / max(np.linalg.norm(d), 1e-6)            # naive go-to-goal policy as smoke
        obs, r, term, trunc, info = env.step(a)
        R += r
        if term or trunc:
            print(f"[smoke] end@{env.tick} reached={info['reached']} collided={info['collided']} "
                  f"min_clr={info['min_clr']:.2f} return={R:.1f}")
            obs, _ = env.reset()
            R = 0.0
