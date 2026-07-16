"""bench_run — one-command benchmark batch: every scene x {ours, native} x N sensor resamples,
emitting a quantile table (out/scenario_runs/BENCH_TABLE.md).

Honesty notes baked into the table:
  * PERCEPT=realistic governs OURS only; the native baseline consumes GT+noise detections
    (range-gated) regardless -- so under realistic perception the two arms see DIFFERENT worlds.
    The table therefore reports: ours(realistic), ours(gt), native(gt). The fair head-to-head is
    ours(gt) vs native(gt); ours(realistic) characterizes deployment honesty, not superiority.
  * min_clr aggregates: median / p05 / worst over resamples; a single draw is never cited.

Run:  python3 bench_run.py [--resample 8] [--dir scenarios/bench]
"""
import argparse
import glob
import os

import numpy as np

import replay_core as RC
import scenario_lib as SLB

HERE = os.path.dirname(os.path.abspath(__file__))
try:                                            # third baseline: MIT-ACL SANDO (GUROBI); needs
    import sando_native_bridge                  # LD_LIBRARY_PATH=~/gurobi1103/linux64/lib
    HAVE_SANDO = True
except Exception as e:                          # 2026-07-16 sweep: fallbacks must be loud
    HAVE_SANDO = False
    print(f"[bench] sando_gt baseline DISABLED: {type(e).__name__}: {e}", flush=True)


def run_arm(scn, mode, percept, seeds, dynamics=False):
    movers = SLB.apply_rh_overrides(RC.Movers(SLB.to_movers_raw(scn)), scn)
    ep = SLB.to_episode(scn)
    res = []
    os.environ["PERCEPT"] = percept
    for sd in seeds:
        os.environ["PERCEPT_SEED"] = str(sd)
        r = RC.run_replay(movers, ep, mode=mode, record=False, dynamics=dynamics,
                          max_vel=float(scn["drone"].get("max_vel", 3.0)))
        res.append(r)
    os.environ.pop("PERCEPT_SEED", None)
    os.environ.pop("PERCEPT", None)
    clr = np.array([r["min_clr"] for r in res])
    return dict(reached=sum(r["reached"] for r in res), collided=sum(r["collided"] for r in res),
                n=len(res), med=float(np.median(clr)), p05=float(np.quantile(clr, 0.05)),
                worst=float(clr.min()), nearmiss=int(np.sum(clr < 0.5)),
                clrs=clr.tolist(),
                t_med=float(np.median([r["time_s"] for r in res])))


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 1.0)
    ph = k / n
    den = 1 + z * z / n
    c = (ph + z * z / (2 * n)) / den
    h = z * np.sqrt(ph * (1 - ph) / n + z * z / (4 * n * n)) / den
    return (max(0.0, c - h), min(1.0, c + h))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--resample", type=int, default=8)
    ap.add_argument("--dir", default="scenarios/bench")
    args = ap.parse_args()
    seeds = [1234567 + 7919 * k for k in range(args.resample)]
    rows = []
    files = [f for f in sorted(glob.glob(os.path.join(args.dir, "*.json")))
             if not f.endswith("MANIFEST.json")]
    for f in files:
        scn = SLB.load(f)
        arms = {"ours_real": run_arm(scn, "ours", "realistic", seeds),
                "ours_gt": run_arm(scn, "ours", "gt", seeds),
                "ours_gt_dyn": run_arm(scn, "ours", "gt", seeds, dynamics=True),
                "ours_real_dyn": run_arm(scn, "ours", "realistic", seeds, dynamics=True),
                "native_gt": run_arm(scn, "native", "gt", seeds),
                "native_real_dyn": run_arm(scn, "native", "realistic", seeds, dynamics=True)}
        if HAVE_SANDO:
            try:
                arms["sando_gt"] = run_arm(scn, "sando", "gt", seeds)
            except Exception as e:
                print(f"  [sando arm failed on {scn['name']}: {type(e).__name__}]")
        rows.append((scn["name"], arms))
        a = arms
        print(f"{scn['name']:18s} ours(real) r{a['ours_real']['reached']}/{a['ours_real']['n']} "
              f"c{a['ours_real']['collided']} clr {a['ours_real']['med']:.2f}/{a['ours_real']['worst']:.2f} | "
              f"ours(gt) c{a['ours_gt']['collided']} clr {a['ours_gt']['med']:.2f} | "
              f"native(gt) c{a['native_gt']['collided']} clr {a['native_gt']['med']:.2f} "
              f"t {a['native_gt']['t_med']:.1f}s vs {a['ours_gt']['t_med']:.1f}s")
    # ---- suite-level summary: collision/near-miss rates w/ Wilson CI + paired clr comparison ----
    arm_names = sorted({a for _, arms in rows for a in arms})
    summary = {}
    for a in arm_names:
        col = sum(arms[a]["collided"] for _, arms in rows if a in arms)
        nm = sum(arms[a]["nearmiss"] for _, arms in rows if a in arms)
        n = sum(arms[a]["n"] for _, arms in rows if a in arms)
        allclr = np.concatenate([arms[a]["clrs"] for _, arms in rows if a in arms])
        boots = [np.median(np.random.default_rng(k).choice(allclr, len(allclr))) for k in range(500)]
        # SCENE-LEVEL bootstrap for the collision rate (episodes cluster within scenes; per-episode
        # Wilson understates the CI when one scene drives the failures, e.g. props_alley 5/7)
        sc_rates = [arms[a]["collided"] / max(1, arms[a]["n"]) for _, arms in rows if a in arms]
        sb = [np.mean(np.random.default_rng(1000 + k).choice(sc_rates, len(sc_rates))) for k in range(500)]
        col_scene_ci = (float(np.quantile(sb, .025)), float(np.quantile(sb, .975)))
        summary[a] = dict(n=n, col=col, col_ci=wilson(col, n), col_scene_ci=col_scene_ci,
                          nm=nm, nm_ci=wilson(nm, n),
                          clr_med=float(np.median(allclr)),
                          clr_ci=(float(np.quantile(boots, .025)), float(np.quantile(boots, .975))))
    pairs = []
    if "ours_gt" in arm_names and "native_gt" in arm_names:
        for name, arms in rows:
            if "ours_gt" in arms and "native_gt" in arms:
                pairs.append(arms["ours_gt"]["med"] - arms["native_gt"]["med"])
    out = os.path.join(HERE, "out", "scenario_runs", "BENCH_TABLE.md")
    with open(out, "w") as fh:
        fh.write(f"# Benchmark 批量表 · resample={args.resample}(每臂 {args.resample} 传感器种子)\n\n")
        fh.write("公平对比 = ours(gt) vs native(gt);ours(realistic) 是部署诚实性刻画(基线无感知模型,两臂世界不同)。\n\n")
        fh.write("| scene | arm | reach | collide | clr med | clr p05 | clr worst | t med |\n")
        fh.write("|---|---|---|---|---|---|---|---|\n")
        for name, arms in rows:
            for arm, a in arms.items():
                fh.write(f"| {name} | {arm} | {a['reached']}/{a['n']} | {a['collided']} "
                         f"| {a['med']:.2f} | {a['p05']:.2f} | {a['worst']:.2f} | {a['t_med']:.1f}s |\n")
        fh.write("\n## Suite 汇总(95% CI)\n\n| arm | episodes | 碰撞率 | 近失率(<0.5m) | clr 中位 |\n|---|---|---|---|---|\n")
        for a, v in summary.items():
            fh.write(f"| {a} | {v['n']} | {v['col']}/{v['n']} [{v['col_ci'][0]:.3f},{v['col_ci'][1]:.3f}] "
                     f"(scene-boot [{v['col_scene_ci'][0]:.3f},{v['col_scene_ci'][1]:.3f}]) "
                     f"| {v['nm']}/{v['n']} [{v['nm_ci'][0]:.3f},{v['nm_ci'][1]:.3f}] "
                     f"| {v['clr_med']:.2f} [{v['clr_ci'][0]:.2f},{v['clr_ci'][1]:.2f}] |\n")
        if pairs:
            w = sum(1 for x in pairs if x > 0.05)
            l = sum(1 for x in pairs if x < -0.05)
            fh.write(f"\n配对(ours_gt−native_gt, 场景级 clr 中位差):均值 {np.mean(pairs):+.2f}m, "
                     f"win/tie/loss = {w}/{len(pairs)-w-l}/{l}\n")
    print(f"\n[bench] wrote {out}")


if __name__ == "__main__":
    main()
