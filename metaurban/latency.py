"""latency — per-layer computation time of the safety stack (real-time feasibility).

Wraps the ACTUAL calls made during a real ours episode (no re-implementation) and times each layer:
  KF predict        — MoverTracker.update + state + predict (the prediction layer, per mover)
  occupancy build   — the predicted-cylinder cloud fed to EGO
  EGO replan        — the underlying motion generator (planner layer)
  certificate       — certify_horizontal + certify_above (the SOUND continuous-time safety check, per mover)
  full ours tick    — one control cycle = cloud + the cert-gated maneuver tournament (~5 replans x cert)
Reports mean / median / p95 / max per call + per tick, so you can see it against the DT=0.30 s control period
(and a tighter onboard budget). Optionally also times native EGO and native SANDO replans for comparison.

Run:  python metaurban/latency.py --seeds 0-2 --n_ep 3
      (add SANDO: LD_LIBRARY_PATH=~/gurobi1103/linux64/lib python metaurban/latency.py --sando)
"""
import os, sys, time, argparse
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import replay_core as R
from ego_bridge import EGOPlanner
from kf_tracker import MoverTracker

T = {}   # name -> list of per-call seconds


def _wrap(obj, name, key):
    fn = getattr(obj, name)
    def timed(*a, **k):
        t0 = time.perf_counter(); r = fn(*a, **k); T.setdefault(key, []).append(time.perf_counter() - t0); return r
    setattr(obj, name, timed)


def stats(key):
    v = np.array(T.get(key, [])) * 1e3   # ms
    if len(v) == 0:
        return None
    return dict(n=len(v), mean=round(float(v.mean()), 3), median=round(float(np.median(v)), 3),
                p95=round(float(np.percentile(v, 95)), 3), max=round(float(v.max()), 3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="0-2")
    ap.add_argument("--n_ep", type=int, default=3)
    ap.add_argument("--speedup", type=float, default=1.33)
    ap.add_argument("--sando", action="store_true", help="also time native SANDO replan (needs GUROBI LD_LIBRARY_PATH)")
    args = ap.parse_args()
    calib = R.load_calib(0.05)
    seeds = (lambda s: list(range(int(s.split('-')[0]), int(s.split('-')[1]) + 1)) if '-' in s
             else [int(x) for x in s.split(',')])(args.seeds)

    # wrap the hot methods on the CLASSES so run_replay's instances are timed
    _wrap(EGOPlanner, "replan", "ego_replan")
    _wrap(EGOPlanner, "certify_horizontal", "cert_horiz")
    _wrap(EGOPlanner, "certify_above", "cert_above")
    _wrap(EGOPlanner, "update_cloud", "cloud_feed")
    _wrap(MoverTracker, "update", "kf_update")
    _wrap(MoverTracker, "predict", "kf_predict")

    fd = os.dup(1); dn = os.open(os.devnull, os.O_WRONLY)
    n_tick = 0; tick_ms = []
    for sd in seeds:
        path = os.path.join(R.OUTDIR, f"traj_seed{sd}.npz")
        if not os.path.exists(path):
            continue
        movers = R.Movers(list(np.load(path, allow_pickle=True)["movers"]))
        for ep in R.build_episodes(movers, sd, n_ep=args.n_ep):
            os.dup2(dn, 1)
            try:
                # time the WHOLE-tick cost by recording history (each history row = one control tick)
                t0 = time.perf_counter()
                r = R.run_replay(movers, ep, "ours", calib, max_vel=3.0 * args.speedup, record=True)
            finally:
                os.dup2(fd, 1)
            nt = r["ticks"]; n_tick += nt
            if nt:
                tick_ms.append((time.perf_counter() - t0) / nt * 1e3)
    os.close(dn)

    print(f"\n=== safety-stack per-layer latency ({n_tick} control ticks over {len(seeds)} seeds) ===")
    print(f"{'layer':<22}{'n_calls':>9}{'mean_ms':>9}{'median':>9}{'p95':>9}{'max':>9}")
    for key, label in (("kf_update", "KF update / mover"), ("kf_predict", "KF predict / mover"),
                       ("cloud_feed", "occupancy feed"), ("ego_replan", "EGO replan (planner)"),
                       ("cert_horiz", "cert horizontal"), ("cert_above", "cert vertical")):
        s = stats(key)
        if s:
            print(f"{label:<22}{s['n']:>9}{s['mean']:>9}{s['median']:>9}{s['p95']:>9}{s['max']:>9}")
    if tick_ms:
        tm = np.array(tick_ms)
        print(f"\n  FULL ours tick (cloud + tournament + cert):  mean={tm.mean():.2f} ms  median={np.median(tm):.2f} ms  "
              f"p95={np.percentile(tm,95):.2f} ms")
        print(f"  control period DT = {R.DT*1e3:.0f} ms  ->  real-time margin {R.DT*1e3/tm.mean():.0f}x "
              f"(tick is {tm.mean()/(R.DT*1e3)*100:.1f}% of the budget)")

    # figure: per-layer latency bars (log scale) + the full-tick vs control-budget
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        layers = [("KF predict/mover", "kf_predict"), ("occupancy feed", "cloud_feed"),
                  ("cert vertical", "cert_above"), ("cert horizontal", "cert_horiz"),
                  ("EGO replan", "ego_replan")]
        names, meds, p95s = [], [], []
        for label, key in layers:
            s = stats(key)
            if s:
                names.append(label); meds.append(s["median"]); p95s.append(s["p95"])
        tm = np.array(tick_ms) if tick_ms else np.array([0.0])
        fig, ax = plt.subplots(1, 2, figsize=(13, 4.4))
        y = np.arange(len(names))
        ax[0].barh(y, meds, color="tab:blue", label="median")
        ax[0].barh(y, p95s, left=0, height=0.4, color="tab:orange", alpha=0.6, label="p95")
        ax[0].set_yticks(y); ax[0].set_yticklabels(names); ax[0].set_xscale("log")
        ax[0].set_xlabel("per-call latency (ms, log)"); ax[0].set_title("safety-stack per-layer compute")
        ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3, axis="x")
        ax[1].bar(["ours tick"], [tm.mean()], color="tab:green", label=f"tick {tm.mean():.1f} ms")
        ax[1].axhline(R.DT * 1e3, color="tab:red", lw=2, ls="--", label=f"control budget {R.DT*1e3:.0f} ms")
        ax[1].set_ylabel("ms"); ax[1].set_title(f"full tick vs budget  ({R.DT*1e3/tm.mean():.0f}x real-time margin)")
        ax[1].legend(); ax[1].grid(alpha=0.3, axis="y")
        fig.suptitle("safety-layer computation latency (after the certify_above fix)", y=1.02)
        fig.tight_layout(); fig.savefig(os.path.join(R.OUTDIR, "fig_latency.png"), dpi=130, bbox_inches="tight")
        print(f"[lat] wrote {os.path.join(R.OUTDIR, 'fig_latency.png')}")
    except Exception as e:
        print(f"[lat] plot skipped: {e}")

    if args.sando:
        from sando_native_bridge import SandoNative
        _wrap(SandoNative, "replan", "sando_replan")
        sn = SandoNative(overrides=dict(v_max=3.0, x_min=-60, x_max=60, y_min=-60, y_max=60, default_goal_z=1.5))
        sn.set_terminal_goal([14, 0, 1.5])
        sn.update_state([0, 0, 1.5]); sn.update_occupancy(R._ground(np.zeros(3)), 0.0)
        os.dup2(os.open(os.devnull, os.O_WRONLY), 1)
        for i in range(30):
            sn.update_state([i * 0.3, 0, 1.5]); sn.replan(0.01, i * 0.3)
        os.dup2(fd, 1)
        s = stats("sando_replan")
        print(f"\n  native SANDO replan (GUROBI):  mean={s['mean']} ms  median={s['median']} ms  p95={s['p95']} ms")
    return 0


if __name__ == "__main__":
    sys.exit(main())
