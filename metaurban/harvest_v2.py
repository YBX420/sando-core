"""harvest_v2 — FS3C-R residual harvests (fold-B fresh-seed / TEST / vehicle fold-A boost).

Config (seeds, grid, rho, VCAP) is the PRE-REGISTERED hash-locked out/conformal/calib_v2_config.json:
tampering with it after fold-B is harvested voids the calibration (spec ruling #8). One run_replay
per (scenario, seed) = one flight; per-episode manifest rows (scn, seed, rows, a2 miss/qual, status)
are written INCREMENTALLY (an MCE reboot loses nothing); zero-row episodes stay VISIBLE in the
manifest (spec ruling #5 -- no silent censoring). Cross-scenario residual-multiset fingerprints
flag scenario aliasing (ruling #17).

Run:  HARV mode via --mode foldB|test|vehA   (sets HARV_V2/HARV_DELTAS/HARV_RHO before importing
replay_core, so run from a fresh interpreter each time).
"""
import argparse
import hashlib
import json
import os
import sys
import zlib

ap = argparse.ArgumentParser()
ap.add_argument("--mode", choices=["foldB", "test", "vehA", "foldB2", "test2", "foldB3", "test3",
                                   "foldB4", "test4", "designC", "designD", "foldB5", "test5"], required=True)
ap.add_argument("--shard", type=int, default=0)
ap.add_argument("--nshard", type=int, default=1)
args = ap.parse_args()

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); os.chdir(HERE)
CFG = json.load(open("out/conformal/calib_v2_config.json"))
_chk = dict(CFG); _sha = _chk.pop("config_sha256")
assert hashlib.sha256(json.dumps(_chk, sort_keys=True).encode()).hexdigest()[:16] == _sha, \
    "config hash mismatch: preregistration violated"

os.environ["HARV_V2"] = "1"
os.environ["HARV_DELTAS"] = ",".join(str(x) for x in CFG["GRID"])
os.environ["HARV_AGE_MIN"] = str(CFG["AGE_MIN"])
os.environ["HARV_RHO"] = json.dumps(CFG["RHO"])
os.environ["PERCEPT"] = "realistic"

import numpy as np
import replay_core as RC
import scenario_lib as SLB

POOL = sorted(CFG["scenario_pool"])[args.shard::args.nshard]
if args.mode == "foldB":
    jobs = [(n, CFG["SEEDS_FOLDB"][n]) for n in POOL]
elif args.mode == "foldB2":
    jobs = [(n, CFG["SEEDS_FOLDB2"][n]) for n in POOL]
elif args.mode == "test":
    jobs = [(n, s) for n in POOL for s in CFG["SEEDS_TEST"][n]]
elif args.mode == "test2":
    jobs = [(n, s) for n in POOL for s in CFG["SEEDS_TEST2"][n]]
elif args.mode == "foldB3":
    jobs = [(n, CFG["SEEDS_FOLDB3"][n]) for n in POOL]
elif args.mode == "test3":
    jobs = [(n, s) for n in POOL for s in CFG["SEEDS_TEST3"][n]]
elif args.mode == "foldB4":
    jobs = [(n, CFG["SEEDS_FOLDB4"][n]) for n in POOL]
elif args.mode == "test4":
    jobs = [(n, s) for n in POOL for s in CFG["SEEDS_TEST4"][n]]
elif args.mode == "designC":   # theta3 shape data with coast column (design domain, fresh 1e8 seeds)
    import zlib as _z
    jobs = [(n, 100_000_000 + _z.crc32(f"{n}|C{k}".encode()) % 9_000_000) for n in POOL for k in range(4)]
elif args.mode == "designD":   # designC seeds + sigma_v column (evidence-bound young growth check)
    import zlib as _z
    jobs = [(n, 100_000_000 + _z.crc32(f"{n}|C{k}".encode()) % 9_000_000) for n in POOL for k in range(4)]
elif args.mode == "foldB5":
    jobs = [(n, CFG["SEEDS_FOLDB5"][n]) for n in POOL]
elif args.mode == "test5":
    jobs = [(n, s) for n in POOL for s in CFG["SEEDS_TEST5"][n]]
else:  # vehA: design-domain vehicle boost (veh_cal x22 + street x6 fresh-A seeds)
    veh = [n for n in POOL if n.startswith("veh_cal")]
    street = [n for n in POOL if n.startswith("street_")]
    jobs = [(n, 30_000_000 + zlib.crc32(f"{n}|A{k}".encode()) % 9_000_000)
            for n in veh for k in range(22)] + \
           [(n, 30_000_000 + zlib.crc32(f"{n}|A{k}".encode()) % 9_000_000)
            for n in street for k in range(6)]

sfx = f"_s{args.shard}" if args.nshard > 1 else ""
out_npy = f"out/conformal/harvest_{args.mode}_v2{sfx}.npy"
out_man = f"out/conformal/harvest_{args.mode}_v2{sfx}.manifest.jsonl"
rows_all, ep = [], 0
man = open(out_man, "w")
man.write(json.dumps(dict(config_sha=_sha, mode=args.mode, n_jobs=len(jobs))) + "\n"); man.flush()
for scn_name, seed in jobs:
    f = (f"scenarios/{scn_name}.json" if os.path.exists(f"scenarios/{scn_name}.json")
         else f"scenarios/bench/{scn_name}.json")
    scn = SLB.load(f)
    movers = SLB.apply_rh_overrides(RC.Movers(SLB.to_movers_raw(scn)), scn)
    epi = SLB.to_episode(scn)
    RC.PERCEPT_HARVEST = []
    RC.PERCEPT_A2 = dict(miss_ticks=0, qual_ticks=0)
    RC._HARV_EP[0] = ep
    RC._HARV_SCN[0] = scn_name
    os.environ["PERCEPT_SEED"] = str(seed)
    status = "ok"
    try:
        r = RC.run_replay(movers, epi, mode="ours", record=False,
                          max_vel=float(scn["drone"].get("max_vel", 3.0)))
    except Exception as e:
        status = f"error:{type(e).__name__}"
        r = {}
    rows = list(RC.PERCEPT_HARVEST)
    rows_all += rows
    man.write(json.dumps(dict(ep=ep, scn=scn_name, seed=seed, rows=len(rows),
                              a2_miss=RC.PERCEPT_A2["miss_ticks"], a2_qual=RC.PERCEPT_A2["qual_ticks"],
                              reached=bool(r.get("reached", False)), status=status)) + "\n"); man.flush()
    ep += 1
    if ep % 20 == 0:
        print(f"[harvest:{args.mode}] {ep}/{len(jobs)}  rows={len(rows_all)}", flush=True)
RC.PERCEPT_HARVEST = None; RC.PERCEPT_A2 = None
man.close()

data = np.array(rows_all, dtype=[("d", "f4"), ("e", "f4"), ("age", "i4"), ("cls", "U12"),
                                 ("ep", "i4"), ("dd", "f4"), ("scn", "U40"), ("qual", "i4"), ("coast", "i4"), ("sigv", "f4")])
np.save(out_npy, data)
# scenario-aliasing fingerprints (ruling #17)
fps = {}
for n in set(data["scn"].tolist()):
    m = data["scn"] == n
    fps[n] = hashlib.sha256(np.sort(np.round(data["e"][m], 4)).tobytes()).hexdigest()[:12]
dup = {}
for n, h in fps.items():
    dup.setdefault(h, []).append(n)
alias = [v for v in dup.values() if len(v) > 1]
print(f"[harvest:{args.mode}] DONE eps={ep} rows={len(data)} -> {out_npy}")
print(f"[harvest:{args.mode}] aliasing fingerprint groups >1: {alias if alias else 'none'}")
