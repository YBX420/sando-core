"""ego_mu_render — top-down MP4 replay of EGO-on-MetaUrban harness episodes (same worlds as the
acceptance ledger: episodes are re-run SEQUENTIALLY from seed 1000 so the scene-reuse state matches)."""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, _HERE)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np

WANT = [int(x) for x in os.environ.get("RENDER_EPS", "0,4,5,6").split(",")]
from metaurban_nav_env import MetaUrbanNavEnv, DT
from ego_bridge import EGOPlanner
import safety_layer as SL
import replay_core as RC

CAL2 = SL.load_calib_v2(eps=float(os.environ.get("CALIB_EPS", 0.10)))
DELTA = float(os.environ.get("DELTA_OVR", 0.30))
env = MetaUrbanNavEnv(seed=0)
ego = EGOPlanner(map_origin=(-40, -40, -1), map_size=(80, 80, 8), res=0.2, inflation=0.1)
ego.set_params(max_vel=env.max_vel, max_acc=env.max_acc, horizon=7.5)
Z = 1.0
CLR = dict(pedestrian="#2a9d2a", vehicle="#d62728", static="#888888", animal="#c78f2c")

for ep in range(max(WANT) + 1):
    env.reset(seed=1000 + ep)
    state = {}; done = False; info = {}
    frames = []
    while not done:
        trs, _ = env._tracks()
        p3 = np.array([env.p[0], env.p[1], Z]); goal3 = np.array([env.goal[0], env.goal[1], Z])
        cloud, mlist = [], []
        for tr in trs:
            c0, vv, aa = tr.trk.state()
            cloud += RC._cyl_cloud([np.asarray(tr.xy[:2], float)], RC._plan_r(tr.r, str(tr.cls)), 0.3, min(tr.h, 2.5))
            mlist.append((np.array([c0[0], c0[1], Z]), np.array([vv[0], vv[1], 0.0]), np.zeros(3),
                          tr.r, 50.0, str(tr.cls), int(tr.trk.n), int(tr.trk.miss > 0),
                          float(getattr(tr.trk, "nis_ewma", 0.0)), float(getattr(tr.trk, "sigma_v", 0.0))))
        ego.update_cloud(np.asarray(cloud, float) if cloud else np.zeros((0, 3)), p3)
        cyl, _zt = SL.build_cylinders(mlist, None, predict=True, calib_v2=CAL2)
        kind, s = SL.maneuver_decide_v2(ego, p3, np.array([env.v[0], env.v[1], 0.0]), np.zeros(3),
                                        goal3, 60.0, cyl, state, cruise_z=Z, horizon=7.5, delta=DELTA)
        if kind == "evade":
            pos, vel = SL.evade_setpoint(p3, [np.asarray(m[0], float) for m in mlist], env.max_vel, DT,
                                         60.0, (goal3 - p3)[:2] / max(np.linalg.norm((goal3 - p3)[:2]), 1e-6))
            v_cmd = np.asarray(vel[:2], float)
        else:
            dur = max(ego.duration() - 1e-3, 0.0)
            ts = np.linspace(0.0, dur, 25)
            ds = [np.linalg.norm(np.asarray(ego.eval(float(t))[0][:2], float) - env.p) for t in ts]
            rr = ego.eval(min(float(ts[int(np.argmin(ds))]) + s * DT, dur))
            v_cmd = ((np.asarray(rr[0][:2], float) if rr is not None else env.goal) - env.p) / DT
        sp = float(np.linalg.norm(v_cmd))
        if sp > env.max_vel:
            v_cmd *= env.max_vel / sp
        act = np.clip((v_cmd - env.v) / DT / env.max_acc, -1, 1)
        if ep in WANT:
            movs = [((xy - env.org), r, c) for (xy, r, _h, c, _v) in env._movers]
            tracked = [np.asarray(tr.xy[:2], float) for tr in trs]
            frames.append(dict(p=env.p.copy(), hd=env._hd.copy(), kind=kind, s=s,
                               movs=movs, trk=tracked, t=env.tick * DT))
        _o, _r, term, trunc, info = env.step(act)
        done = term or trunc
    if ep not in WANT:
        print(f"ep{ep} skipped ({'coll' if info.get('collided') else 'ok'})", flush=True)
        continue
    tag = "COLLIDED" if info.get("collided") else ("REACHED" if info.get("reached") else "TIMEOUT")
    fig, ax = plt.subplots(figsize=(7, 7), dpi=90)
    trail = []

    def draw(i):
        ax.clear()
        f = frames[i]
        trail.append(f["p"])
        for (xy, r, c) in f["movs"]:
            ax.add_patch(plt.Circle(xy, r, color=CLR.get(c, "#666"), alpha=0.55,
                                    zorder=3 if c != "static" else 2))
        for xy in f["trk"]:
            ax.plot(*xy, "x", color="k", ms=5, zorder=6)
        hd = f["hd"] / (np.linalg.norm(f["hd"]) + 1e-9)
        for sgn in (+1, -1):
            a = np.radians(45.0) * sgn
            Rm = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
            q = f["p"] + (Rm @ hd) * 10.0
            ax.plot([f["p"][0], q[0]], [f["p"][1], q[1]], "-", color="#77aaff", lw=0.8, zorder=4)
        tr = np.array(trail)
        ax.plot(tr[:, 0], tr[:, 1], "-", color="#3355ff", lw=1.4, alpha=0.8, zorder=5)
        ax.plot(*f["p"], "o", color="#1133ee", ms=9, zorder=7)
        ax.plot(*env.goal, "*", color="#e6b800", ms=17, zorder=7)
        ax.set_title(f"ep{ep} [{tag}]  t={f['t']:.1f}s  decision={f['kind']}(s={f['s']:.2f})\n"
                     f"绿=行人 红=车 灰=静物 ×=已追踪 蓝线=45°视锥", fontsize=9)
        ax.set_xlim(-35, 35); ax.set_ylim(-35, 35); ax.set_aspect("equal")
    a = anim.FuncAnimation(fig, draw, frames=len(frames), interval=300)
    out = os.path.join(_HERE, "out", f"mu_ep{ep}_{tag.lower()}.mp4")
    a.save(out, writer=anim.FFMpegWriter(fps=int(round(1 / DT)) + 4))
    plt.close(fig)
    print(f"saved {out} ({len(frames)} ticks)", flush=True)
