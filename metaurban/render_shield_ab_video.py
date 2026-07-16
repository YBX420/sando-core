"""render_shield_ab_video — side-by-side mp4s: the SAME PPO drone with the certificate shield ON
(left) vs OFF (right), same seed stream, top-down view. Picks episode indices where the bare arm
collided and the shielded arm survived, so the videos SHOW what the gate buys.

Drawn per tick: movers (gray=static, red=pedestrian, blue=vehicle, circle=true footprint), the
drone (black dot + heading), its 45-deg FOV cone (what perception can see), READY tracks (green
rings = what the shield certifies against), shield decision as border colour (green pass / orange
projected / red brake), trails, and a collision flash.

Run (metaurban env, cwd+PYTHONPATH = metaurban repo):  python render_shield_ab_video.py
"""
import os
import pickle
import sys

_HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, _HERE)
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor, VecFrameStack
from metaurban_nav_env import MetaUrbanNavEnv
from train_ppo_v2 import ShieldedEnv

N_EP = 24
FOV_DEG, FOV_R = 45.0, 10.0
os.chdir(_HERE)
OUT = "out/videos"
os.makedirs(OUT, exist_ok=True)


def record_arm(shielded):
    inner = []
    def mk():
        e = MetaUrbanNavEnv(seed=7777)
        e = ShieldedEnv(e) if shielded else e
        inner.append(e)
        return e
    venv = VecFrameStack(VecMonitor(DummyVecEnv([mk])), 3)
    model = PPO.load("out/ppo_planner_mu", env=venv, device="cpu")
    sh = inner[0].sh if shielded else None
    raw = inner[0].env if shielded else inner[0]

    obs = venv.reset()
    eps, frames = [], []
    while len(eps) < N_EP:
        pre = (sh.n_pass, sh.n_project, sh.n_brake) if sh else (0, 0, 0)
        act, _ = model.predict(obs, deterministic=True)
        obs, r, done, infos = venv.step(act)
        dec = "off"
        if sh:
            d = (sh.n_pass - pre[0], sh.n_project - pre[1], sh.n_brake - pre[2])
            dec = "pass" if d[0] else ("proj" if d[1] else "brake")
        movers = [((xy - raw.org).tolist(), float(rr), c)
                  for xy, rr, _h, c, _v in raw._movers
                  if np.linalg.norm((xy - raw.org) - raw.p) < 28.0]
        tracks = [np.asarray(t.xy, float).tolist() for t in raw.pfe.tracks if t.trk.ready]
        hd = raw._hd / max(float(np.linalg.norm(raw._hd)), 1e-9)
        frames.append(dict(p=raw.p.tolist(), hd=hd.tolist(), movers=movers, tracks=tracks,
                           dec=dec, clr=float(raw.min_clr)))
        if done[0]:
            i = infos[0]
            eps.append(dict(frames=frames, reached=bool(i.get("reached")),
                            collided=bool(i.get("collided")), goal=raw.goal.tolist()))
            frames = []
            if len(eps) % 8 == 0:
                print(f"[rec:{'on' if shielded else 'off'}] {len(eps)}/{N_EP}", flush=True)
    venv.close()
    return eps


print("[rec] recording shielded arm ...", flush=True)
A = record_arm(True)
print("[rec] recording bare arm ...", flush=True)
B = record_arm(False)
pickle.dump((A, B), open(f"{OUT}/ab_traces.pkl", "wb"))

pairs = [i for i in range(N_EP) if B[i]["collided"] and not A[i]["collided"]]
print(f"[pick] divergent episodes (bare collided, shielded survived): {pairs}", flush=True)
pairs = pairs[:3] if pairs else [0]

DEC_COL = dict(off="#888888", pass_="#2a9d2a", proj="#e08a00", brake="#d62020")
CLS_COL = dict(static="#9a9a9a", pedestrian="#d62728", vehicle="#1f77b4", animal="#9467bd")


def draw(ax, ep, k, title):
    ax.clear()
    fr = ep["frames"][min(k, len(ep["frames"]) - 1)]
    p = np.array(fr["p"]); hd = np.array(fr["hd"]); goal = np.array(ep["goal"])
    ang0 = np.degrees(np.arctan2(hd[1], hd[0]))
    ax.add_patch(Wedge(p, FOV_R, ang0 - FOV_DEG, ang0 + FOV_DEG, color="#ffd54d", alpha=0.25, zorder=1))
    for xy, r, c in fr["movers"]:
        ax.add_patch(Circle(xy, r, color=CLS_COL.get(c, "#666"), alpha=0.55, zorder=2))
    for xy in fr["tracks"]:
        ax.add_patch(Circle(xy, 0.65, fill=False, ec="#00b050", lw=1.6, zorder=4))
    trail = np.array([f["p"] for f in ep["frames"][:k + 1]])
    ax.plot(trail[:, 0], trail[:, 1], "-", color="#222", lw=1.0, alpha=0.7, zorder=3)
    ax.plot(*goal, marker="*", ms=16, color="#caa000", zorder=5)
    ax.plot(*p, "o", ms=7, color="#111", zorder=6)
    ax.arrow(p[0], p[1], hd[0] * 1.6, hd[1] * 1.6, head_width=0.5, color="#111", zorder=6)
    dec = fr["dec"]
    col = DEC_COL["pass_" if dec == "pass" else dec]
    ended = k >= len(ep["frames"]) - 1
    status = ("COLLISION" if ep["collided"] else ("reached" if ep["reached"] else "timeout")) if ended else ""
    if ended and ep["collided"]:
        ax.add_patch(Circle(p, 2.2, fill=False, ec="#d62020", lw=3, zorder=7))
    t = min(k, len(ep["frames"]) - 1)
    ax.set_title(f"{title}   t={t}  shield:{dec}  min_clr={fr['clr']:.2f}  {status}",
                 fontsize=10, color=col)
    ax.set_xlim(-34, 34); ax.set_ylim(-24, 24); ax.set_aspect("equal")
    ax.tick_params(labelsize=7)
    for s in ax.spines.values():
        s.set_color(col); s.set_linewidth(2)


import matplotlib.animation as anim
for idx in pairs:
    ea, eb = A[idx], B[idx]
    T = max(len(ea["frames"]), len(eb["frames"]))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.4), dpi=100)
    fig.suptitle(f"PPO drone, episode {idx + 1}: certificate shield ON (left) vs OFF (right)",
                 fontsize=12)

    def update(k, ea=ea, eb=eb, ax1=ax1, ax2=ax2):
        draw(ax1, ea, k, "shield ON")
        draw(ax2, eb, k, "shield OFF")
        return []

    a = anim.FuncAnimation(fig, update, frames=T, blit=False)
    fn = f"{OUT}/ab_ep{idx + 1:02d}.mp4"
    a.save(fn, writer=anim.FFMpegWriter(fps=8, bitrate=1800))
    plt.close(fig)
    print(f"[vid] {fn}  ({T} ticks)", flush=True)
print("[vid] done", flush=True)
