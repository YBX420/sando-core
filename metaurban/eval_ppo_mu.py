"""eval_ppo_mu — A/B eval of ppo_planner_mu on MetaUrbanNavEnv: SHIELDED vs BARE (no shield),
same policy, same scenario seed, 60 eps each. Answers two questions at once:
  1. shield net effect: reach/collide with the certificate gate vs the raw learned policy;
  2. collision attribution (shielded arm): culprit in_track? shield decision on the fatal tick?
     -> coverage hole (not in track: fix conformal/perception)  vs  certified-but-wrong (in track,
        passed: tube/cert question)  vs  uncertified exposure (brake tick).
Note: same 20 MetaUrban scenarios as training (no held-out split yet).
"""
import os, sys, time
_HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, _HERE)
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor, VecFrameStack
from metaurban_nav_env import MetaUrbanNavEnv
from train_ppo_v2 import ShieldedEnv

N_EP = 60
os.chdir(_HERE)


def run_arm(shielded):
    inner = []
    def mk():
        e = MetaUrbanNavEnv(seed=7777)                 # same seed -> same scenario draw sequence
        e = ShieldedEnv(e) if shielded else e
        inner.append(e)
        return e
    venv = VecFrameStack(VecMonitor(DummyVecEnv([mk])), 3)
    model = PPO.load("out/ppo_planner_mu", env=venv, device="cpu")
    sh = inner[0].sh if shielded else None

    obs = venv.reset()
    eps, colls, t0 = [], [], time.time()
    while len(eps) < N_EP:
        pre = (sh.n_pass, sh.n_project, sh.n_brake) if sh else (0, 0, 0)
        act, _ = model.predict(obs, deterministic=True)
        obs, r, done, infos = venv.step(act)
        if done[0]:
            i = infos[0]
            eps.append(dict(reached=bool(i.get("reached")), collided=bool(i.get("collided")),
                            min_clr=float(i.get("min_clr", np.nan))))
            if i.get("collided") and "culprit" in i:
                if sh:
                    d = (sh.n_pass - pre[0], sh.n_project - pre[1], sh.n_brake - pre[2])
                    dec = "passed" if d[0] else ("projected" if d[1] else ("brake" if d[2] else "?"))
                else:
                    dec = "no-shield"
                colls.append(dict(ep=len(eps), decision=dec, **i["culprit"]))
            if len(eps) % 20 == 0:
                print(f"[eval:{'shield' if shielded else 'bare'}] {len(eps)}/{N_EP} "
                      f"+{(time.time()-t0)/60:.1f}min", flush=True)
    venv.close()
    return eps, colls, (sh.stats() if sh else None)


def summarize(name, eps, colls, st):
    reach = sum(e["reached"] for e in eps); coll = sum(e["collided"] for e in eps)
    clrs = np.array([e["min_clr"] for e in eps])
    lines = [f"## arm: {name}",
             f"reached : {reach}/{N_EP}  ({100*reach/N_EP:.1f}%)",
             f"collided: {coll}/{N_EP}  ({100*coll/N_EP:.1f}%)",
             f"timeout : {N_EP-reach-coll}/{N_EP}",
             f"min_clr : median {np.median(clrs):.2f}  p10 {np.percentile(clrs,10):.2f}  "
             f"min {clrs.min():.2f} m"]
    if st:
        lines.append(f"shield  : {st}")
    if colls:
        hole = sum(1 for c in colls if not c["in_track"])
        lines.append(f"attribution: coverage-hole {hole}/{len(colls)}, "
                     f"in-track {len(colls)-hole}/{len(colls)}")
        for c in colls:
            lines.append(f"  ep{c['ep']:02d}: cls={c['cls']:10s} r={c['r']:.2f} "
                         f"in_track={c['in_track']} track_dist={c['track_dist']} "
                         f"n_ready={c['n_ready']} shield={c['decision']}")
    return "\n".join(lines)


print("[eval] arm 1/2: SHIELDED", flush=True)
s_eps, s_colls, s_st = run_arm(True)
print("[eval] arm 2/2: BARE (no shield)", flush=True)
b_eps, b_colls, _ = run_arm(False)

out = "\n\n".join([
    f"# ppo_planner_mu A/B eval + attribution  ({time.strftime('%Y-%m-%d %H:%M')})",
    summarize("shielded (certificate gate ON)", s_eps, s_colls, s_st),
    summarize("bare policy (no shield)", b_eps, b_colls, None),
])
print(out, flush=True)
os.makedirs("out/ppo_eval", exist_ok=True)
open("out/ppo_eval/results_mu.md", "w").write(out + "\n")
print("[eval] written -> out/ppo_eval/results_mu.md", flush=True)
