"""eval_oracle_shield — upper bound for the "360-degree safety ring" fix.

Same fixed-lambda policy (ppo_planner_mu), same episodes protocol as eval_shield_ab, ONE arm:
the shield certifies against OMNIDIRECTIONAL GT tracks within SENSE_R=16 m (no cone, no occlusion,
no miss, no birth delay; GT velocity), while the POLICY's observations stay the trained perception
(the wrapper still advances the KF exactly like ShieldedEnv would, so obs distribution matches
training). Compare against ab_20260706_014118: shielded 6.75%, bare 14.25%.

If the blind-side floor (~24/400) vanishes -> a 360-degree ring is worth building.
If it persists -> 8.3 m/s vehicles vs a 3 m/s drone are partly physically unavoidable; write the
failure envelope that way.
"""
import json
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, _HERE)
import numpy as np
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor, VecFrameStack
from metaurban_nav_env import MetaUrbanNavEnv, DT
from shield import CertShield

N_EP = 400
SENSE_R = 16.0
os.chdir(_HERE)
stamp = time.strftime("%Y%m%d_%H%M%S")
OUT = f"out/ppo_eval/oracle_{stamp}"
os.makedirs(OUT, exist_ok=True)


class OracleShieldedEnv(gym.Wrapper):
    """CertShield fed with omnidirectional GT tracks (the oracle sensor), policy obs unchanged."""

    def __init__(self, env, sense_r=SENSE_R):
        super().__init__(env)
        self.sh = CertShield(env.max_vel, env.max_acc, DT)
        self.sense_r = float(sense_r)

    def step(self, a):
        env = self.env
        env._tracks()                          # advance perception KF like ShieldedEnv (obs parity)
        tracks = []
        for xy_w, r, _h, _cls, v in env._movers:
            xy = xy_w - env.org
            if float(np.linalg.norm(xy - env.p)) <= self.sense_r:
                tracks.append((xy, np.asarray(v, float)[:2], float(r)))
        gd = env.goal - env.p
        gd = gd / max(float(np.linalg.norm(gd)), 1e-6)
        v_next, certified, intervened = self.sh.filter(env.p, env.v, np.asarray(a, float),
                                                       tracks, gd)
        a_exec = np.clip((v_next - env.v) / (DT * env.max_acc), -1, 1)
        obs, r, term, trunc, info = env.step(a_exec)
        return obs, float(r), term, trunc, info


def wilson(k, n, z=1.96):
    p = k / n; d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


inner = []
def mk():
    e = OracleShieldedEnv(MetaUrbanNavEnv(seed=7777))
    inner.append(e)
    return e
venv = VecFrameStack(VecMonitor(DummyVecEnv([mk])), 3)
model = PPO.load("out/ppo_planner_mu", env=venv, device="cpu")
sh = inner[0].sh

jl = open(os.path.join(OUT, "oracle.jsonl"), "a")
obs = venv.reset()
eps, t0 = [], time.time()
while len(eps) < N_EP:
    pre = (sh.n_pass, sh.n_project, sh.n_brake)
    act, _ = model.predict(obs, deterministic=True)
    obs, r, done, infos = venv.step(act)
    if done[0]:
        i = infos[0]
        rec = dict(ep=len(eps) + 1, reached=bool(i.get("reached")),
                   collided=bool(i.get("collided")),
                   min_clr=round(float(i.get("min_clr", np.nan)), 3))
        if i.get("collided") and "culprit" in i:
            d = (sh.n_pass - pre[0], sh.n_project - pre[1], sh.n_brake - pre[2])
            dec = "passed" if d[0] else ("projected" if d[1] else ("brake" if d[2] else "?"))
            rec["culprit"] = dict(i["culprit"], decision=dec)
        eps.append(rec)
        jl.write(json.dumps(rec) + "\n"); jl.flush()
        if len(eps) % 50 == 0:
            c = sum(e["collided"] for e in eps)
            print(f"[oracle] {len(eps)}/{N_EP}  coll={c} ({100*c/len(eps):.1f}%)  "
                  f"+{(time.time()-t0)/60:.0f}min", flush=True)
jl.close()
venv.close()

n = len(eps)
reach = sum(e["reached"] for e in eps); coll = sum(e["collided"] for e in eps)
clrs = np.array([e["min_clr"] for e in eps])
lo, hi = wilson(coll, n)
colls = [e["culprit"] for e in eps if e.get("culprit")]
lines = [
    f"# oracle 360-deg shield (SENSE_R={SENSE_R} m, GT tracks)  ({time.strftime('%Y-%m-%d %H:%M')})",
    f"model=ppo_planner_mu  n={n}  seed=7777   [baselines: shielded 6.75%, bare 14.25%]",
    f"reached : {reach}/{n}  ({100*reach/n:.1f}%)",
    f"collided: {coll}/{n}  ({100*coll/n:.2f}%)   Wilson95 [{100*lo:.2f}%, {100*hi:.2f}%]",
    f"timeout : {n-reach-coll}/{n}",
    f"min_clr : median {np.median(clrs):.2f}  p10 {np.percentile(clrs,10):.2f}  min {clrs.min():.2f} m",
    f"shield  : {sh.stats()}",
]
for c in colls:
    lines.append(f"  ep{c.get('ep','?') if isinstance(c, dict) else '?'}: {c}")
out = "\n".join(lines)
print(out, flush=True)
open(os.path.join(OUT, "summary.md"), "w").write(out + "\n")
print(f"[oracle] written -> {OUT}/summary.md", flush=True)
