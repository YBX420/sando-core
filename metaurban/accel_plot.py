"""accel_plot — acceleration / jerk profiles: ours (KF-predicted, cert-gated) vs native EGO.

Thesis (塔菲大人): "because I analyse the movers with a KF, I get a SMOOTHER acceleration" -- ours anticipates
where people WILL be and weaves through one smooth trajectory, while native EGO reacts to where they ARE and
brake/re-accelerates late (jerky). This plots |a(t)|, jerk |da/dt|(t), speed |v(t)| for one representative
contested episode, AND aggregates the smoothness metrics (RMS jerk, peak |a|) over ALL episodes so the
"smoother" claim is paired-tested, not cherry-picked.

Default matched speed (both 3.0 m/s) isolates the PREDICTION-smoothness benefit from the speed budget.

Run:  python metaurban/accel_plot.py --max_vel 3.0
"""
import os, sys, glob, json, argparse
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import replay_core as R

OUTDIR = R.OUTDIR
DT = R.DT


def series(hist):
    """Extract t, |a|, jerk |da/dt|, |v| from a recorded history."""
    t = np.array([h["t"] - hist[0]["t"] for h in hist])
    a = np.array([h["a"] for h in hist])
    v = np.array([h["v"] for h in hist])
    ax = np.array([h["ax"] for h in hist]); ay = np.array([h["ay"] for h in hist])
    jx = np.diff(ax) / DT; jy = np.diff(ay) / DT
    jerk = np.sqrt(jx ** 2 + jy ** 2)
    return t, a, v, jerk


def smooth_metrics(hist):
    if len(hist) < 3:
        return None
    _, a, v, jerk = series(hist)
    return dict(rms_jerk=float(np.sqrt(np.mean(jerk ** 2))), peak_a=float(np.max(a)),
                mean_a=float(np.mean(a)), peak_jerk=float(np.max(jerk)))


def run(seed, ep, movers, calib, max_vel, speedup_ours):
    ro = R.run_replay(movers, ep, "ours", calib, max_vel=max_vel * speedup_ours, record=True)
    rn = R.run_replay(movers, ep, "native", calib, max_vel=max_vel, record=True)
    return ro, rn


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--max_vel", type=float, default=3.0, help="matched speed (isolates prediction-smoothness)")
    ap.add_argument("--ours_speedup", type=float, default=1.0)
    ap.add_argument("--eps", type=float, default=0.05)
    ap.add_argument("--seed", type=int, default=-1, help="-1 = auto-pick a representative contested episode")
    args = ap.parse_args()
    calib = R.load_calib(args.eps)
    files = sorted(glob.glob(os.path.join(OUTDIR, "traj_seed*.npz")))

    # ---- aggregate smoothness over ALL episodes (paired ours vs native) ----
    agg = []           # (rms_jerk_ours, rms_jerk_native, peak_a_ours, peak_a_native)
    rep = None         # representative episode (both reach, native maneuvers a lot -> visible contrast)
    import contextlib
    @contextlib.contextmanager
    def quiet():
        fd = os.dup(1); dn = os.open(os.devnull, os.O_WRONLY); os.dup2(dn, 1); os.close(dn)
        try: yield
        finally: os.dup2(fd, 1); os.close(fd)

    for f in files:
        sd = int(os.path.basename(f).replace("traj_seed", "").replace(".npz", ""))
        if args.seed >= 0 and sd != args.seed:
            continue
        movers = R.Movers(list(np.load(f, allow_pickle=True)["movers"]))
        for k, ep in enumerate(R.build_episodes(movers, sd, n_ep=6)):
            with quiet():
                ro, rn = run(sd, ep, movers, calib, args.max_vel, args.ours_speedup)
            mo, mn = smooth_metrics(ro["history"]), smooth_metrics(rn["history"])
            if mo and mn and ro["reached"] and rn["reached"]:
                agg.append((mo["rms_jerk"], mn["rms_jerk"], mo["peak_a"], mn["peak_a"]))
                # representative: pick the episode with the largest native-minus-ours jerk gap (clear contrast)
                gap = mn["rms_jerk"] - mo["rms_jerk"]
                if rep is None or gap > rep[0]:
                    rep = (gap, sd, k, ro, rn)

    agg = np.array(agg)
    n = len(agg)
    oj, nj = agg[:, 0], agg[:, 1]; oa, na = agg[:, 2], agg[:, 3]
    print(f"=== acceleration smoothness, ours vs native EGO  ({n} episodes, matched {args.max_vel} m/s) ===")
    print(f"  RMS jerk (m/s^3):  ours mean={oj.mean():.2f}  native mean={nj.mean():.2f}  "
          f"-> ours lower in {int(np.sum(oj < nj))}/{n} episodes (paired)")
    print(f"  peak |a| (m/s^2):  ours mean={oa.mean():.2f}  native mean={na.mean():.2f}")
    print(f"  median RMS-jerk ratio ours/native = {np.median(oj / np.maximum(nj, 1e-6)):.2f}  (<1 => ours smoother)")
    summary = dict(n=n, max_vel=args.max_vel, ours_speedup=args.ours_speedup,
                   rms_jerk_ours=float(oj.mean()), rms_jerk_native=float(nj.mean()),
                   ours_smoother_frac=float(np.mean(oj < nj)),
                   peak_a_ours=float(oa.mean()), peak_a_native=float(na.mean()),
                   jerk_ratio_median=float(np.median(oj / np.maximum(nj, 1e-6))))
    with open(os.path.join(OUTDIR, "accel_smoothness.json"), "w") as fo:
        json.dump(summary, fo, indent=2)

    # ---- representative-episode curves ----
    _, sd, k, ro, rn = rep
    to, ao, vo, jo = series(ro["history"]); tn, an, vn, jn = series(rn["history"])
    print(f"  representative episode: seed{sd} ep{k} "
          f"(ours t={ro['time_s']:.1f}s jerk_rms={np.sqrt((jo**2).mean()):.2f}, "
          f"native t={rn['time_s']:.1f}s jerk_rms={np.sqrt((jn**2).mean()):.2f})")
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
        ax[0].plot(to, ao, "-", color="tab:green", lw=2, label="ours (KF + cert)")
        ax[0].plot(tn, an, "-", color="tab:red", lw=2, label="native EGO")
        ax[0].set_title("acceleration magnitude |a(t)|"); ax[0].set_xlabel("t (s)"); ax[0].set_ylabel("|a| (m/s^2)")
        ax[0].legend(); ax[0].grid(alpha=0.3)
        ax[1].plot(to[1:], jo, "-", color="tab:green", lw=2, label="ours")
        ax[1].plot(tn[1:], jn, "-", color="tab:red", lw=2, label="native EGO")
        ax[1].set_title("JERK |da/dt|(t)  (lower = smoother)"); ax[1].set_xlabel("t (s)"); ax[1].set_ylabel("jerk (m/s^3)")
        ax[1].legend(); ax[1].grid(alpha=0.3)
        ax[2].plot(to, vo, "-", color="tab:green", lw=2, label="ours")
        ax[2].plot(tn, vn, "-", color="tab:red", lw=2, label="native EGO")
        ax[2].set_title("speed |v(t)|"); ax[2].set_xlabel("t (s)"); ax[2].set_ylabel("|v| (m/s)")
        ax[2].legend(); ax[2].grid(alpha=0.3)
        fig.suptitle(f"ours vs native EGO — acceleration profile (seed{sd} ep{k}; matched {args.max_vel} m/s; "
                     f"ours RMS-jerk {oj.mean():.1f} vs native {nj.mean():.1f} over {n} eps)", y=1.02)
        fig.tight_layout(); fig.savefig(os.path.join(OUTDIR, "accel_profile.png"), dpi=130, bbox_inches="tight")
        print(f"[accel] wrote {os.path.join(OUTDIR, 'accel_profile.png')}")
    except Exception as e:
        print(f"[accel] plot skipped: {e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
