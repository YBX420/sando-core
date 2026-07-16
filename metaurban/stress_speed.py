"""stress_speed — theorem-v2 tube × speed × per-speed-retimed scenes; every failure logged for
autopsy (collision / near-miss / DNF, with per-episode detail)."""
import glob, json, os
import replay_core as RC
import scenario_lib as SLB

fails = []
rows = []
for v in (4, 5, 6, 7, 8):
    files = sorted(glob.glob(f"scenarios/speedtuned_v{v}/*.json"))
    for arm, mode, percept, dyn in (("ours_gt", "ours", "gt", False),
                                    ("ours_real_dyn", "ours", "realistic", True)):
        col = nm = dnf = n = 0
        os.environ["PERCEPT"] = percept
        for f in files:
            scn = SLB.load(f)
            movers = SLB.apply_rh_overrides(RC.Movers(SLB.to_movers_raw(scn)), scn)
            ep = SLB.to_episode(scn)
            for k in range(4):
                os.environ["PERCEPT_SEED"] = str(1234567 + 7919 * k)
                r = RC.run_replay(movers, ep, mode=mode, record=False, dynamics=dyn,
                                  max_vel=float(scn["drone"].get("max_vel", 3.0)))
                n += 1
                bad = r["collided"] or r["min_clr"] < 0.5 or not r["reached"]
                col += r["collided"]; nm += r["min_clr"] < 0.5; dnf += not r["reached"]
                if bad:
                    fails.append(dict(v=v, arm=arm, scn=scn["name"], seed=k,
                                      collided=bool(r["collided"]), clr=round(r["min_clr"], 2),
                                      reached=bool(r["reached"]), t=round(r["time_s"], 1),
                                      evade=r["counts"]["evade"], rta=r["rta"]))
        os.environ.pop("PERCEPT", None)
        rows.append(f"| {v} | {arm} | {col} | {nm} | {dnf} | {n} |")
        print(rows[-1], flush=True)
json.dump(fails, open("out/scenario_runs/stress_failures.json", "w"), indent=1)
print(f"[stress] {len(fails)} failure episodes logged")
