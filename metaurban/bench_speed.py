"""bench_speed — drone-speed sweep of the full benchmark: how does safety scale with vmax?
Overrides every scenario's drone.max_vel; note fast_canyon natively specifies 5.0 (kept overridden
too -- the sweep is THE variable). New static semantics (online mapping) baseline."""
import glob
import numpy as np
import replay_core as RC
import scenario_lib as SLB
from bench_run import run_arm, wilson

speeds = [4.0, 5.0, 6.0, 7.0, 8.0]
seeds = [1234567 + 7919 * k for k in range(4)]
ARMS = [("ours_gt", "ours", "gt", False), ("ours_real_dyn", "ours", "realistic", True),
        ("native_gt", "native", "gt", False)]
files = [f for f in sorted(glob.glob("scenarios/full/*.json")) if not f.endswith("MANIFEST.json")]
out = ["# 速度扫描 Benchmark(vmax 4-8 m/s × 28 场景 × 4 采样,在线建图静态语义)",
       "", "| vmax | arm | 碰撞率 [95%CI] | 近失率 | clr 中位 | t 中位 |", "|---|---|---|---|---|---|"]
for v in speeds:
    for name, mode, percept, dyn in ARMS:
        col = nm = n = 0; clrs = []; ts = []
        for f in files:
            scn = SLB.load(f)
            scn["drone"]["max_vel"] = v
            scn["drone"]["max_acc"] = max(6.0, float(scn["drone"].get("max_acc", 6.0)))
            a = run_arm(scn, mode, percept, seeds, dynamics=dyn)
            col += a["collided"]; nm += a["nearmiss"]; n += a["n"]
            clrs += a["clrs"]; ts.append(a["t_med"])
        lo, hi = wilson(col, n)
        line = (f"| {v:.0f} | {name} | {col}/{n} [{lo:.3f},{hi:.3f}] | {nm}/{n} "
                f"| {np.median(clrs):.2f} | {np.median(ts):.1f}s |")
        out.append(line)
        print(line, flush=True)
open("out/scenario_runs/BENCH_SPEED_SWEEP.md", "w").write("\n".join(out) + "\n")
print("[sweep] wrote out/scenario_runs/BENCH_SPEED_SWEEP.md")
