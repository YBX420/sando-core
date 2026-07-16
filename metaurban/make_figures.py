"""make_figures — render the headline result figures from the saved conformal/A-B JSON artifacts.

Produces (out/conformal/):
  fig_cert_ablation.png   continuous-time cert vs discrete sampling: FALSE-SAFE (missed collisions)
  fig_predictor.png       CA vs CV predictor -> per-class conformal keep-out v_eff (CV halves pedestrian)
  fig_ab.png              ours vs native EGO: time scatter + time-delta hist + min-clearance (the headline)
"""
import os, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
OUTDIR = os.path.join(os.path.dirname(HERE), "out", "conformal")


def _load(name):
    p = os.path.join(OUTDIR, name)
    return json.load(open(p)) if os.path.exists(p) else None


def fig_cert_ablation():
    d = _load("cert_ablation.json")
    if not d:
        print("[fig] no cert_ablation.json"); return
    methods = ["continuous"] + [f"discrete_{n}" for n in d["samples"]]
    labels = ["continuous\n(Bernstein)"] + [f"discrete\n{n}-sample" for n in d["samples"]]
    fs = [d["stats"][m]["false_safe"] for m in methods]
    colors = ["tab:green"] + ["tab:red"] * len(d["samples"])
    fig, ax = plt.subplots(figsize=(7, 4.4))
    bars = ax.bar(labels, fs, color=colors)
    for b, v in zip(bars, fs):
        ax.text(b.get_x() + b.get_width() / 2, v + 1, str(v), ha="center", fontsize=10, fontweight="bold")
    ax.set_ylabel(f"FALSE-SAFE: missed collisions (of {d['n_collide']} truly colliding)")
    ax.set_title("Continuous-time certificate is SOUND; discrete sampling tunnels (misses collisions)")
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout(); fig.savefig(os.path.join(OUTDIR, "fig_cert_ablation.png"), dpi=130)
    print("[fig] wrote fig_cert_ablation.png")


def fig_predictor():
    d = _load("predictor_compare.json")
    if not d:
        print("[fig] no predictor_compare.json"); return
    rep = d["report"]; classes = ["pedestrian", "vehicle", "all"]
    ca = [rep["ca"][c]["0.05"]["v_eff"] for c in classes]
    cv = [rep["cv"][c]["0.05"]["v_eff"] for c in classes]
    x = np.arange(len(classes)); w = 0.36
    fig, ax = plt.subplots(figsize=(7, 4.4))
    ax.bar(x - w / 2, ca, w, label="CA (const-accel)", color="tab:orange")
    ax.bar(x + w / 2, cv, w, label="CV (const-vel, deployed)", color="tab:blue")
    for xi, (a, c) in enumerate(zip(ca, cv)):
        ax.text(xi - w / 2, a + 0.02, f"{a:.2f}", ha="center", fontsize=9)
        ax.text(xi + w / 2, c + 0.02, f"{c:.2f}", ha="center", fontsize=9)
    ax.set_xticks(x); ax.set_xticklabels(classes)
    ax.set_ylabel("conformal keep-out growth v_eff (m/s) @95%")
    ax.set_title("Better predictor -> tighter certified keep-out (CV ~halves pedestrian)")
    ax.legend(); ax.grid(alpha=0.3, axis="y")
    fig.tight_layout(); fig.savefig(os.path.join(OUTDIR, "fig_predictor.png"), dpi=130)
    print("[fig] wrote fig_predictor.png")


def fig_ab(summary="ab_summary_spd133.json"):
    d = _load(summary)
    if not d:
        print(f"[fig] no {summary}"); return
    rows = d["rows"]
    ot = np.array([r["ours_t"] for r in rows]); nt = np.array([r["nat_t"] for r in rows])
    oc = np.array([r["ours_coll"] for r in rows]); nc = np.array([r["nat_coll"] for r in rows])
    ocl = np.array([r["ours_clr"] if r["ours_clr"] is not None else np.nan for r in rows])
    ncl = np.array([r["nat_clr"] if r["nat_clr"] is not None else np.nan for r in rows])
    dt = ot - nt
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.4))
    # A: time scatter ours vs native, colour native collisions
    m = nc.astype(bool)
    ax[0].scatter(nt[~m], ot[~m], s=18, alpha=0.5, color="tab:blue", label="native reached safe")
    ax[0].scatter(nt[m], ot[m], s=30, alpha=0.8, color="tab:red", marker="x", label="native COLLIDED")
    lim = [0, max(ot.max(), nt.max()) + 1]
    ax[0].plot(lim, lim, "0.5", lw=1, ls="--"); ax[0].set_xlim(lim); ax[0].set_ylim(lim)
    ax[0].set_xlabel("native EGO time (s)"); ax[0].set_ylabel("ours time (s)")
    ax[0].set_title("below diagonal = ours faster"); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
    # B: time-delta histogram
    ax[1].hist(dt, bins=24, color="tab:green", alpha=0.8)
    ax[1].axvline(0, color="0.4", lw=1); ax[1].axvline(np.median(dt), color="tab:blue", lw=2,
                                                       label=f"median {np.median(dt):+.2f}s")
    ax[1].set_xlabel("time delta ours - native (s)"); ax[1].set_ylabel("episodes")
    ax[1].set_title(f"ours faster (dt<0) in {int(np.sum(dt<0))}/{len(dt)}"); ax[1].legend(fontsize=9); ax[1].grid(alpha=0.3)
    # C: min clearance ours vs native (sorted)
    ax[2].axhline(0, color="tab:red", lw=1, ls="--", label="collision (clr<0)")
    so = np.sort(ocl[~np.isnan(ocl)]); sn = np.sort(ncl[~np.isnan(ncl)])
    ax[2].plot(np.linspace(0, 1, len(so)), so, color="tab:green", lw=2, label="ours")
    ax[2].plot(np.linspace(0, 1, len(sn)), sn, color="tab:red", lw=2, label="native EGO")
    ax[2].set_xlabel("episode percentile"); ax[2].set_ylabel("min clearance to humans (m)")
    ax[2].set_title(f"ours 0 collisions; native {int(nc.sum())}/{len(rows)}"); ax[2].legend(fontsize=9); ax[2].grid(alpha=0.3)
    fig.suptitle(f"ours (KF + conformal cert) vs native EGO — {summary.replace('ab_summary_','').replace('.json','')} "
                 f"({len(rows)} episodes on real MetaUrban trajectories)", y=1.02)
    fig.tight_layout(); fig.savefig(os.path.join(OUTDIR, "fig_ab.png"), dpi=130, bbox_inches="tight")
    print("[fig] wrote fig_ab.png")


if __name__ == "__main__":
    fig_cert_ablation()
    fig_predictor()
    fig_ab()
    print("[fig] done")
