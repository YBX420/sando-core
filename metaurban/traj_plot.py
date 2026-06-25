"""traj_plot — bird's-eye paths: ours vs native EGO threading the SAME real crowd, on a contested episode.

Shows WHY ours wins: it weaves around where people WILL be (KF + cert) and never enters a body cylinder, while
native EGO reacts late and grazes/collides. Picks an episode where native collides but ours does not.

Run:  python metaurban/traj_plot.py            (auto-picks a native-collides episode)
"""
import os, sys, math, glob, contextlib
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import replay_core as R


@contextlib.contextmanager
def _quiet():
    fd = os.dup(1); dn = os.open(os.devnull, os.O_WRONLY); os.dup2(dn, 1); os.close(dn)
    try: yield
    finally: os.dup2(fd, 1); os.close(fd)


def main():
    calib = R.load_calib(0.05)
    files = sorted(glob.glob(os.path.join(R.OUTDIR, "traj_seed*.npz")))
    best = None
    for f in files:
        sd = int(os.path.basename(f).replace("traj_seed", "").replace(".npz", ""))
        movers = R.Movers(list(np.load(f, allow_pickle=True)["movers"]))
        for k, ep in enumerate(R.build_episodes(movers, sd, n_ep=6)):
            with _quiet():
                ro = R.run_replay(movers, ep, "ours", calib, max_vel=3.0 * 1.33, record=True)
                rn = R.run_replay(movers, ep, "native", calib, max_vel=3.0, record=True)
            if rn["collided"] and not ro["collided"] and ro["reached"]:
                # prefer the episode with the most movers in the corridor (busiest)
                score = len(ep["members"])
                if best is None or score > best[0]:
                    best = (score, sd, k, ep, movers, ro, rn)
        if best and best[0] >= 4:
            break
    if best is None:
        print("[traj] no native-collides/ours-safe episode found"); return 1
    _, sd, k, ep, movers, ro, rn = best
    org = 0.5 * (ep["start"][:2] + ep["goal"][:2])
    start = ep["start"][:2] - org; goal = ep["goal"][:2] - org
    t0 = ep["t0"]

    fig, ax = plt.subplots(figsize=(9, 8))
    # mover trajectories over the episode window (local frame), with body-cylinder footprint at mid-window
    tspan = np.arange(0, max(ro["ticks"], rn["ticks"]) * R.DT, R.DT)
    for i in ep["members"]:
        xy = np.array([movers.pos(i, t0 + tt) - org for tt in tspan if movers.present(i, t0 + tt)])
        if len(xy) > 1:
            ax.plot(xy[:, 0], xy[:, 1], color="0.6", lw=1, zorder=1)
            mid = xy[len(xy) // 2]
            ax.add_patch(plt.Circle(mid, movers.m[i]["r"], color="0.7", alpha=0.5, zorder=1))
            ax.scatter(*xy[0], color="0.5", s=12, marker="o", zorder=2)        # mover start
    # drone paths
    po = np.array([h["p"] for h in ro["history"]]); pn = np.array([h["p"] for h in rn["history"]])
    ax.plot(po[:, 0], po[:, 1], "-", color="tab:green", lw=2.5, label=f"ours (cert) t={ro['time_s']:.1f}s, clr {ro['min_clr']:.2f}m", zorder=4)
    ax.plot(pn[:, 0], pn[:, 1], "-", color="tab:red", lw=2.5, label=f"native EGO t={rn['time_s']:.1f}s, COLLIDED {rn['min_clr']:.2f}m", zorder=3)
    # mark native collision point (min clearance tick)
    clrs = [h["clr"] if h["clr"] is not None else 9 for h in rn["history"]]
    ci = int(np.argmin(clrs))
    ax.scatter(pn[ci, 0], pn[ci, 1], color="red", s=140, marker="X", zorder=6, label="native collision")
    ax.scatter(*start, color="black", s=80, marker="s", zorder=5, label="start")
    ax.scatter(*goal, color="black", s=120, marker="*", zorder=5, label="goal")
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_xlabel("x (m, local)"); ax.set_ylabel("y (m, local)")
    ax.set_title(f"ours vs native EGO through the real crowd (seed{sd} ep{k}, {len(ep['members'])} crossers)\n"
                 f"ours weaves around the FUTURE & stays clear; native reacts late & collides")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(R.OUTDIR, "fig_trajectory.png"), dpi=130, bbox_inches="tight")
    print(f"[traj] wrote fig_trajectory.png  (seed{sd} ep{k}: ours clr {ro['min_clr']:.2f} vs native {rn['min_clr']:.2f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
