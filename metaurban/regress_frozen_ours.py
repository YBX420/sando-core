"""Byte-level regression: run ours-arm replay on pinned scenarios/seeds, hash full tick history."""
import hashlib, json, os, sys
shadow = sys.argv[1] if sys.argv[1] != "-" else None
MU = "/media/boxuan/Data2/projects/sando_py/sando-core/metaurban"
if shadow:
    sys.path.insert(0, shadow)      # pre-change kf_tracker/perception shadow first
sys.path.insert(1 if shadow else 0, MU)
os.chdir(MU)
# claim kf_tracker/perception in sys.modules BEFORE replay_core (which prepends its own dir)
import kf_tracker
import perception
import replay_core as RC
import scenario_lib as SLB
print(f"[regress] kf_tracker from: {kf_tracker.__file__}", flush=True)
print(f"[regress] perception from: {perception.__file__}", flush=True)
assert RC.MoverTracker is kf_tracker.MoverTracker, "replay_core bound a DIFFERENT MoverTracker!"


SCNS = ["scenarios/full/crossers.json", "scenarios/full/fast_canyon.json",
        "scenarios/full/dual_blindside.json", "scenarios/full/crowd_dense.json",
        "scenarios/bench/street_busy_s0.json", "scenarios/bench/street_busy_s1.json"]
out = {}
for f in SCNS:
    scn = SLB.load(f)
    movers = SLB.apply_rh_overrides(RC.Movers(SLB.to_movers_raw(scn)), scn)
    ep = SLB.to_episode(scn)
    for sd in (11, 42):
        os.environ["PERCEPT"] = "realistic"; os.environ["PERCEPT_SEED"] = str(sd)
        r = RC.run_replay(movers, ep, mode="ours", record=True, dynamics=False,
                          max_vel=float(scn["drone"].get("max_vel", 3.0)))
        h = hashlib.sha256(json.dumps(r.get("hist", r), sort_keys=True, default=str).encode()).hexdigest()[:16]
        key = f"{os.path.basename(f)}:{sd}"
        out[key] = dict(hash=h, reached=r["reached"], collided=r["collided"],
                        min_clr=round(r["min_clr"], 6))
        print(f"[regress] {key}: {h} reach={r['reached']} coll={r['collided']} clr={r['min_clr']:.4f}", flush=True)
json.dump(out, open(sys.argv[2], "w"), indent=1)
print(f"[regress] -> {sys.argv[2]}", flush=True)
