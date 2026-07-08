"""eval_shield_ab — LARGE shield-ON vs shield-OFF campaign on MetaUrbanNavEnv.

Same policy, same scenario seed (paired), N episodes per arm. Per-episode results are APPENDED to
a jsonl as they finish (an MCE reboot loses nothing already written). Summary reports collision /
reach rates with Wilson 95% CIs, Fisher exact p (if scipy), shield stats, and the collision
attribution broken into:
    in-track            (shield's jurisdiction: it certified against a live track and still hit)
    hole: in-cone       (detected/could detect but track not ready -> birth-gate axis)
    hole: blind-side    (outside the 45-deg body-heading cone -> structural sensor limit)

Usage (metaurban env, cwd+PYTHONPATH = metaurban repo):
  python eval_shield_ab.py --n_ep 400 --model out/ppo_planner_mu
"""
import argparse
import json
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, _HERE)
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor, VecFrameStack
from ground_nav_env import GroundNavEnv as MetaUrbanNavEnv
from ground_shield import GroundShieldedEnv as ShieldedEnv

ap = argparse.ArgumentParser()
ap.add_argument("--n_ep", type=int, default=400)
ap.add_argument("--model", type=str, default="out/ppo_planner_ground")
ap.add_argument("--seed", type=int, default=7777)
ap.add_argument("--out", type=str, default="")
args = ap.parse_args()
os.chdir(_HERE)
stamp = time.strftime("%Y%m%d_%H%M%S")
FPRINT = {k: os.environ.get(k, "") for k in
          ("SHIELD_EPS", "SHIELD_PERCLASS", "DECIDE", "SMOOTH", "RADIUS_CONSIST", "CRET_GLIDE",
           "CALIB_V2", "PERCEPT", "PERCEPT_SEED", "PRED_MODEL", "DYN_TRACK")}
OUT = args.out or f"out/ppo_eval/ab_{stamp}"
os.makedirs(OUT, exist_ok=True)


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def run_arm(name, shielded):
    inner = []
    def mk():
        e = MetaUrbanNavEnv(seed=args.seed)
        e = ShieldedEnv(e) if shielded else e
        inner.append(e)
        return e
    venv = VecFrameStack(VecMonitor(DummyVecEnv([mk])), 3)
    model = PPO.load(args.model, env=venv, device="cpu")
    sh = inner[0].sh if shielded else None

    jl = open(os.path.join(OUT, f"{name}.jsonl"), "a")
    jl.write(json.dumps(dict(config_fingerprint=FPRINT, model=args.model, n_ep=args.n_ep,
                             seed=args.seed)) + "\n"); jl.flush()
    obs = venv.reset()
    eps, t0 = [], time.time()
    ep0 = (0, 0, 0)
    while len(eps) < args.n_ep:
        pre = (sh.n_pass, sh.n_project, sh.n_brake) if sh else (0, 0, 0)
        act, _ = model.predict(obs, deterministic=True)
        obs, r, done, infos = venv.step(act)
        if done[0]:
            i = infos[0]
            interv = ((sh.n_project - ep0[1]) + (sh.n_brake - ep0[2])) if sh else 0
            if sh:
                ep0 = (sh.n_pass, sh.n_project, sh.n_brake)
            rec = dict(ep=len(eps) + 1, reached=bool(i.get("reached")),
                       collided=bool(i.get("collided")),
                       min_clr=round(float(i.get("min_clr", np.nan)), 3),
                       interv=int(interv),
                       clean=bool(i.get("reached") and not i.get("collided") and interv == 0))
            if i.get("collided") and "culprit" in i:
                if sh:
                    d = (sh.n_pass - pre[0], sh.n_project - pre[1], sh.n_brake - pre[2])
                    dec = "passed" if d[0] else ("projected" if d[1] else ("brake" if d[2] else "?"))
                else:
                    dec = "no-shield"
                rec["culprit"] = dict(i["culprit"], decision=dec)
            eps.append(rec)
            jl.write(json.dumps(rec) + "\n"); jl.flush()
            if len(eps) % 50 == 0:
                c = sum(e["collided"] for e in eps)
                print(f"[{name}] {len(eps)}/{args.n_ep}  coll={c} ({100*c/len(eps):.1f}%)  "
                      f"+{(time.time()-t0)/60:.0f}min", flush=True)
    jl.close()
    st = sh.stats() if sh else None
    venv.close()
    return eps, st


def attribution(eps):
    colls = [e["culprit"] for e in eps if e.get("culprit")]
    return dict(
        n=len(colls),
        in_track=sum(1 for c in colls if c["in_track"]),
        hole_in_cone=sum(1 for c in colls if not c["in_track"] and c.get("in_cone")),
        hole_blind=sum(1 for c in colls if not c["in_track"] and not c.get("in_cone")),
        cls={k: sum(1 for c in colls if c["cls"] == k) for k in
             sorted({c["cls"] for c in colls})},
        med_speed=(float(np.median([c.get("speed", 0.0) for c in colls])) if colls else None),
    )


def block(name, eps, st):
    n = len(eps)
    reach = sum(e["reached"] for e in eps); coll = sum(e["collided"] for e in eps)
    clrs = np.array([e["min_clr"] for e in eps])
    lo, hi = wilson(coll, n)
    at = attribution(eps)
    lines = [f"## arm: {name}   (n={n})",
             f"reached : {reach}/{n}  ({100*reach/n:.1f}%)",
             f"collided: {coll}/{n}  ({100*coll/n:.2f}%)   Wilson95 [{100*lo:.2f}%, {100*hi:.2f}%]",
             f"timeout : {n-reach-coll}/{n}",
             f"min_clr : median {np.median(clrs):.2f}  p10 {np.percentile(clrs,10):.2f}  "
             f"min {clrs.min():.2f} m",
             f"attribution: in_track={at['in_track']}  hole_in_cone={at['hole_in_cone']}  "
             f"hole_blind={at['hole_blind']}  by_cls={at['cls']}  med_speed={at['med_speed']}"]
    if st:
        lines.append(f"shield  : {st}")
    return "\n".join(lines), coll


print(f"[ab] campaign start: n_ep={args.n_ep}/arm  model={args.model}  out={OUT}", flush=True)
s_eps, s_st = run_arm("shielded", True)
b_eps, _ = run_arm("bare", False)

blk_s, k_s = block("shielded (certificate gate ON)", s_eps, s_st)
blk_b, k_b = block("bare (no shield)", b_eps, None)
stat_line = ""
try:
    from scipy.stats import fisher_exact
    n = args.n_ep
    _, p = fisher_exact([[k_s, n - k_s], [k_b, n - k_b]])
    stat_line = f"Fisher exact (collision, shielded vs bare): p = {p:.4f}"
except Exception as e:
    stat_line = f"(scipy unavailable for Fisher test: {e})"

out = "\n\n".join([
    f"# shield ON/OFF campaign  ({time.strftime('%Y-%m-%d %H:%M')})  "
    f"model={args.model}  n={args.n_ep}/arm  seed={args.seed} (same-seed stream, NOT strictly paired)\n"
    f"config: {json.dumps(FPRINT)}",
    blk_s, blk_b, stat_line,
])
print(out, flush=True)
open(os.path.join(OUT, "summary.md"), "w").write(out + "\n")
print(f"[ab] written -> {OUT}/summary.md", flush=True)
