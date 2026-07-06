"""Per-node inference-latency profile of the ours pipeline (wrap-at-runtime, repo untouched).
Nodes: perception KF | EGO cloud update | build_cylinders | cert_clear (single) | ego.replan
(single) | maneuver_decide (whole tournament) | quad dynamics | full tick."""
import os, sys, time
MU = "/media/boxuan/Data2/projects/sando_py/sando-core/metaurban"
sys.path.insert(0, MU); os.chdir(MU)
import numpy as np
import perception, safety_layer as SL
import ego_bridge
from quadrotor import Quadrotor
import replay_core as RC
import scenario_lib as SLB

T = {}
def wrap(obj, name, key):
    fn = getattr(obj, name)
    lst = T.setdefault(key, [])
    def w(*a, **k):
        t0 = time.perf_counter()
        r = fn(*a, **k)
        lst.append((time.perf_counter() - t0) * 1e3)
        return r
    setattr(obj, name, w)

wrap(perception.PerceptionFrontEnd, "step", "perception_kf")
wrap(ego_bridge.EGOPlanner, "update_cloud", "ego_cloud")
wrap(ego_bridge.EGOPlanner, "replan", "ego_replan")
wrap(SL, "build_cylinders", "build_cylinders")
wrap(SL, "cert_clear", "cert_clear")
wrap(SL, "maneuver_decide", "tournament_total")
wrap(Quadrotor, "step", "quad_dyn")

tick_ms = []
_orig = RC.run_replay
SCNS = ["scenarios/full/crossers.json", "scenarios/full/crowd_dense.json",
        "scenarios/full/fast_canyon.json"]
for f in SCNS:
    scn = SLB.load(f)
    movers = SLB.apply_rh_overrides(RC.Movers(SLB.to_movers_raw(scn)), scn)
    ep = SLB.to_episode(scn)
    os.environ["PERCEPT"] = "realistic"; os.environ["PERCEPT_SEED"] = "11"
    def cb(d, _t=[None]):
        now = time.perf_counter()
        if _t[0] is not None:
            tick_ms.append((now - _t[0]) * 1e3)
        _t[0] = now
    r = RC.run_replay(movers, ep, mode="ours", record=False, dynamics=True,
                      max_vel=float(scn["drone"].get("max_vel", 3.0)), tick_cb=cb)
    print(f"[prof] {os.path.basename(f)}: reach={r['reached']} ticks={r['ticks']}", flush=True)

print(f"\n{'node':18s} {'calls':>6s} {'median':>9s} {'p95':>9s} {'max':>9s}")
for k in ("perception_kf", "ego_cloud", "ego_replan", "build_cylinders", "cert_clear",
          "tournament_total", "quad_dyn"):
    v = np.array(T.get(k, [0.0]))
    print(f"{k:18s} {len(v):6d} {np.median(v):8.3f}m {np.percentile(v,95):8.3f}m {v.max():8.3f}m")
tk = np.array(tick_ms)
print(f"{'FULL TICK':18s} {len(tk):6d} {np.median(tk):8.3f}m {np.percentile(tk,95):8.3f}m {tk.max():8.3f}m")
print(f"\n[prof] tick budget DT=300ms; median tick uses {100*np.median(tk)/300:.1f}%  "
      f"p95 {100*np.percentile(tk,95)/300:.1f}%  -> max control rate ~{1000/np.percentile(tk,95):.0f} Hz (p95)")
