"""probe_v3 — 小而快的 CPL-v3 reach 诊断:固定 8 场景,报 reach/clean/evade + v3 的 kind 分布。
用法: DECIDE=v2|v3 [DELTA_OVR=0.05 V3_VCAP=1 ...] python probe_v3.py"""
import os, sys, json
os.environ.setdefault("CALIB_V2", "1"); os.environ.setdefault("CALIB_EPS", "0.10")
os.environ.setdefault("PERCEPT", "realistic")
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); os.chdir(HERE)
import numpy as np
import replay_core as RC
import scenario_lib as SLB

SCNS = ["gauntlet", "crossers", "head_on", "crowd_dense", "wall_and_crosser",
        "fast_overtake", "speed_sweep", "vehicle_spawn_accel"]
SEED = 42
DEC = os.environ.get("DECIDE", "v2")
os.environ["PERCEPT_SEED"] = str(SEED)
agg = dict(reach=0, clean=0, coll=0, evade=0)
kinds = {}
print(f"=== DECIDE={DEC} DELTA_OVR={os.environ.get('DELTA_OVR','')} V3_VCAP={os.environ.get('V3_VCAP','0')} ===")
for name in SCNS:
    f = f"scenarios/{name}.json"
    if not os.path.exists(f):
        f = f"scenarios/bench/{name}.json"
    scn = SLB.load(f)
    movers = SLB.apply_rh_overrides(RC.Movers(SLB.to_movers_raw(scn)), scn)
    ep = SLB.to_episode(scn)
    try:
        r = RC.run_replay(movers, ep, mode="ours", record=False,
                          max_vel=float(scn["drone"].get("max_vel", 3.0)))
    except Exception as e:
        print(f"  {name:20s} ERROR {type(e).__name__}: {e}"); continue
    cnt = r.get("counts", {})
    emerg = int(cnt.get("evade", 0)) + int(cnt.get("cret", 0))
    agg["reach"] += int(r["reached"]); agg["coll"] += int(r["collided"]); agg["evade"] += emerg
    agg["clean"] += int(r["reached"] and not r["collided"] and emerg == 0)
    for k, val in cnt.items():
        if isinstance(val, (int, float)) and k in ("cpl", "brake", "evade") or str(k).startswith("v3_"):
            kinds[k] = kinds.get(k, 0) + int(val)
    print(f"  {name:20s} reach={int(r['reached'])} coll={int(r['collided'])} evade={emerg:3d} "
          f"t={r['time_s']:5.1f} clr={r['min_clr']:.2f}")
n = len(SCNS)
print(f"--- 合计: reach {agg['reach']}/{n}  clean {agg['clean']}/{n}  撞 {agg['coll']}  evade {agg['evade']}")
print(f"--- kind 分布: {kinds}")
