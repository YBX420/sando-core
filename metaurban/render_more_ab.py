"""render_more_ab — render extra AB pairs from the saved traces (no re-recording)."""
import os, pickle, sys
_HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, _HERE)
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from render_shield_ab_video import draw   # reuse the panel painter

os.chdir(_HERE)
A, B = pickle.load(open("out/videos/ab_traces.pkl", "rb"))
print("idx | shieldON(reach,coll) | bare(reach,coll) | lenA lenB")
for i, (a, b) in enumerate(zip(A, B)):
    print(f"{i:3d} | {int(a['reached'])},{int(a['collided'])} | "
          f"{int(b['reached'])},{int(b['collided'])} | {len(a['frames'])} {len(b['frames'])}")

both_coll = [i for i in range(len(A)) if A[i]["collided"] and B[i]["collided"]]
both_ok   = [i for i in range(len(A)) if A[i]["reached"] and B[i]["reached"]]
picks = []
if both_coll: picks.append(("boundary", both_coll[0]))
if both_ok:
    # the both-reached pair with the most path divergence
    def div(i):
        la = np.array([f["p"] for f in A[i]["frames"]]); lb = np.array([f["p"] for f in B[i]["frames"]])
        n = min(len(la), len(lb)); return float(np.mean(np.linalg.norm(la[:n]-lb[:n], axis=1)))
    both_ok.sort(key=div, reverse=True)
    picks.append(("detour", both_ok[0]))
print("picks:", picks)

for tag, idx in picks:
    ea, eb = A[idx], B[idx]
    T = max(len(ea["frames"]), len(eb["frames"]))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.4), dpi=100)
    fig.suptitle(f"PPO drone, episode {idx+1} ({tag}): shield ON (left) vs OFF (right)", fontsize=12)
    def update(k, ea=ea, eb=eb, ax1=ax1, ax2=ax2):
        draw(ax1, ea, k, "shield ON"); draw(ax2, eb, k, "shield OFF"); return []
    a = anim.FuncAnimation(fig, update, frames=T, blit=False)
    fn = f"out/videos/ab_ep{idx+1:02d}_{tag}.mp4"
    a.save(fn, writer=anim.FFMpegWriter(fps=8, bitrate=1800)); plt.close(fig)
    print(f"[vid] {fn} ({T} ticks)", flush=True)
