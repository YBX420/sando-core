"""harvest_v3 — ATOMIC per-episode residual collector (07-20 harvest contract).

Contract (vs the retired harvest_v2, whose whole-shard rows_all lived in memory until one final
np.save -- an MCE reboot lost the shard, restart truncated the manifest, and /tmp logs + stale
shards could impersonate a fresh run):
  - ONE episode == ONE interpreter == ONE atomic data file: rows are written to <job>.npy.tmp,
    sha256'd, renamed, and only then a done receipt is written. No receipt = the job never
    happened (an exception exits non-zero and leaves nothing behind).
  - The job list is PRE-REGISTERED (--plan writes jobs.json first); episode id == job index by
    definition (kills the off-by-one class). Scenario names stored as U64 (kills U40 truncation).
  - Resume (--job on an existing job) re-runs unless data + receipt + checksum + job spec ALL
    match. Runs live in a VERSIONED run directory, never /tmp.
  - --merge refuses unless the receipt set EQUALS the registered job set and every checksum
    re-verifies; legal zero-row episodes still enter the scenario UNIVERSE (universe.json) so
    the conformal rank counts them at -inf instead of silently dropping the scenario.
  - Every receipt records the effective env/config knobs (fail-loud manifest ruling), the
    generator commit and the schema version.

Modes: pilot12 (first 12 pool scenarios x 2 eps -- pipeline acceptance ONLY, not statistics),
       design69 (all pool scenarios x 2 eps -- the retired-design re-harvest that fixes
       EGO_MEM_K and the other algorithm dials before the stack freeze).
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import zlib

ap = argparse.ArgumentParser()
ap.add_argument("--mode", choices=["pilot12", "design69"], required=True)
ap.add_argument("--run", required=True, help="versioned run directory")
ap.add_argument("--plan", action="store_true")
ap.add_argument("--job", type=int, default=None)
ap.add_argument("--merge", action="store_true")
args = ap.parse_args()

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); os.chdir(HERE)
SCHEMA_VER = "v3.0"
DTYPE = [("d", "f4"), ("e", "f4"), ("age", "i4"), ("cls", "U12"),
         ("ep", "i4"), ("dd", "f4"), ("scn", "U64"), ("qual", "i4"), ("coast", "i4"), ("sigv", "f4"),
         ("ea", "f4"), ("ec", "f4"), ("spd", "f4"), ("esg", "f4"), ("erb", "f4"), ("etp", "f4"),
         ("psig", "f4"), ("coast_s", "f4"), ("a_held", "f4"), ("mem_m", "f4")]

CFG = json.load(open("out/conformal/calib_v2_config.json"))
_chk = dict(CFG); _sha = _chk.pop("config_sha256")
assert hashlib.sha256(json.dumps(_chk, sort_keys=True).encode()).hexdigest()[:16] == _sha, \
    "config hash mismatch: preregistration violated"

try:
    GIT = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], cwd=HERE, text=True).strip()
except Exception:
    GIT = "unknown"


def build_jobs():
    pool = sorted(CFG["scenario_pool"])
    if args.mode == "pilot12":
        pool = pool[:12]
    tag = {"pilot12": "V3P", "design69": "V3D"}[args.mode]
    jobs = []
    for n in pool:
        for k in range(2):                       # UNIFORM 2 episodes per scenario (07-20 quota law)
            seed = 400_000_000 + zlib.crc32(f"{n}|{tag}{k}".encode()) % 9_000_000
            jobs.append(dict(job_id=len(jobs), scn=n, seed=seed, ep=len(jobs)))
    return jobs


def env_knobs():
    """The effective dials a receipt must pin (fail-loud manifest ruling)."""
    import kf_tracker as KF
    ks = {k: os.environ.get(k) for k in
          ("KF_COAST", "KF_COAST_AFLOOR", "KF_INIT", "PERCEPT_TTL_S", "PERCEPT_YTTL_S",
           "PERCEPT_TTL", "YOUNG_TTL", "CALIB_EPS", "CALIB_V2", "CAPSULE", "ELLIPSE",
           "DECIDE", "PERCEPT", "OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS")
          if os.environ.get(k) is not None}
    ks["_TAU_A"] = KF._COAST_TAU_A
    ks["_A_FLOOR"] = KF._COAST_A_FLOOR
    ks["_COAST_CA"] = KF._KF_COAST_CA
    return ks


def sha_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:16]


RUN = os.path.abspath(args.run)
JOBS_F = os.path.join(RUN, "jobs.json")

if args.plan:
    os.makedirs(os.path.join(RUN, "data"), exist_ok=True)
    os.makedirs(os.path.join(RUN, "logs"), exist_ok=True)
    if os.path.exists(JOBS_F):                   # NEVER truncate an existing registration
        old = json.load(open(JOBS_F))
        assert old["mode"] == args.mode and old["config_sha"] == _sha, \
            "run dir already registered for a DIFFERENT mode/config -- use a fresh run dir"
        print(f"[plan] existing registration kept: {len(old['jobs'])} jobs (resume run)")
        sys.exit(0)
    jobs = build_jobs()
    tmp = JOBS_F + ".tmp"
    json.dump(dict(mode=args.mode, config_sha=_sha, generator_commit=GIT,
                   schema_ver=SCHEMA_VER, dtype=[list(x) for x in DTYPE],
                   env_knobs=env_knobs(), jobs=jobs), open(tmp, "w"), indent=1)
    os.replace(tmp, JOBS_F)
    print(f"[plan] registered {len(jobs)} jobs -> {JOBS_F}")
    sys.exit(0)

REG = json.load(open(JOBS_F))
assert REG["mode"] == args.mode and REG["config_sha"] == _sha, "registration mismatch"

if args.merge:
    import numpy as np
    missing, bad, parts, universe = [], [], [], {}
    for j in REG["jobs"]:
        dp = os.path.join(RUN, "data", f"job_{j['job_id']}.npy")
        rp = os.path.join(RUN, "data", f"job_{j['job_id']}.receipt.json")
        if not (os.path.exists(dp) and os.path.exists(rp)):
            missing.append(j["job_id"]); continue
        rc = json.load(open(rp))
        if rc["job"] != j or rc["sha256"] != sha_file(dp):
            bad.append(j["job_id"]); continue
        arr = np.load(dp)
        parts.append(arr)
        universe.setdefault(j["scn"], []).append(j["ep"])   # zero-row episodes STAY in the universe
    if missing or bad:
        print(f"[merge] REFUSED: missing={missing} bad={bad} (job set must equal the "
              f"registration exactly; checksums must re-verify)", flush=True)
        sys.exit(2)
    data = np.concatenate(parts) if parts else np.empty(0, dtype=DTYPE)
    outp = os.path.join(RUN, f"harvest_{args.mode}_v3.npy")
    np.save(outp + ".tmp.npy", data); os.replace(outp + ".tmp.npy", outp)
    json.dump(universe, open(os.path.join(RUN, "universe.json"), "w"), indent=1)
    json.dump(dict(mode=args.mode, n_jobs=len(REG["jobs"]), rows=int(len(data)),
                   sha256=sha_file(outp), schema_ver=SCHEMA_VER),
              open(os.path.join(RUN, "merged.manifest.json"), "w"), indent=1)
    zero = [f"{s}:{e}" for s, eps in universe.items() for e in eps
            if not ((data["scn"] == s) & (data["ep"] == e)).any()]
    print(f"[merge] OK jobs={len(REG['jobs'])} rows={len(data)} -> {outp}")
    print(f"[merge] zero-row episodes kept in universe ({len(zero)}): {zero if zero else 'none'}")
    sys.exit(0)

assert args.job is not None, "need --plan, --job or --merge"
job = REG["jobs"][args.job]
assert job["job_id"] == args.job, "job registry corrupt (id != index)"
dp = os.path.join(RUN, "data", f"job_{job['job_id']}.npy")
rp = os.path.join(RUN, "data", f"job_{job['job_id']}.receipt.json")
if os.path.exists(dp) and os.path.exists(rp):
    rc = json.load(open(rp))
    if rc["job"] == job and rc["sha256"] == sha_file(dp):
        print(f"[job {job['job_id']}] SKIP (data+receipt+checksum match)")
        sys.exit(0)
    print(f"[job {job['job_id']}] stale/partial artefacts -> re-running")

os.environ["HARV_V2"] = "1"
os.environ["HARV_DELTAS"] = ",".join(str(x) for x in CFG["GRID"])
os.environ["HARV_AGE_MIN"] = str(CFG["AGE_MIN"])
os.environ["HARV_RHO"] = json.dumps(CFG["RHO"])
os.environ["PERCEPT"] = "realistic"
os.environ["PERCEPT_SEED"] = str(job["seed"])

import numpy as np
import replay_core as RC
import scenario_lib as SLB

f = (f"scenarios/{job['scn']}.json" if os.path.exists(f"scenarios/{job['scn']}.json")
     else f"scenarios/bench/{job['scn']}.json")
scn = SLB.load(f)
movers = SLB.apply_rh_overrides(RC.Movers(SLB.to_movers_raw(scn)), scn)
epi = SLB.to_episode(scn)
RC.PERCEPT_HARVEST = []
RC.PERCEPT_A2 = dict(miss_ticks=0, qual_ticks=0)
RC._HARV_EP[0] = job["ep"]
RC._HARV_SCN[0] = job["scn"]
r = RC.run_replay(movers, epi, mode="ours", record=False,
                  max_vel=float(scn["drone"].get("max_vel", 3.0)))
#   an exception above propagates: non-zero exit, NO receipt -> the driver counts the failure and
#   resume re-runs the job (the old collector swallowed it and still "returned 0")
rows = list(RC.PERCEPT_HARVEST)
data = np.array(rows, dtype=DTYPE) if rows else np.empty(0, dtype=DTYPE)
np.save(dp + ".tmp.npy", data)
os.replace(dp + ".tmp.npy", dp)
receipt = dict(job=job, rows=int(len(data)), sha256=sha_file(dp), status="ok",
               reached=bool(r.get("reached", False)), collided=bool(r.get("collided", False)),
               attrib=r.get("attrib", {}), schema_ver=SCHEMA_VER, git_commit=GIT,
               env_knobs=env_knobs())
tmp = rp + ".tmp"
json.dump(receipt, open(tmp, "w"), indent=1)
os.replace(tmp, rp)
print(f"[job {job['job_id']}] OK scn={job['scn']} ep={job['ep']} rows={len(data)} "
      f"reached={receipt['reached']} sha={receipt['sha256']}")
