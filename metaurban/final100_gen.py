"""final100_gen — generate pre-registered final100 slots (07-21 contract).

One SLOT at a time, from the frozen plan only:
  - GIT LOCK: refuses to run unless the working tree HEAD equals the plan's stack SHA (--draft
    plans skip the lock but can never become paper artifacts) -- mid-campaign code changes can
    not mix generations;
  - retry seeds follow the REGISTERED rule (root + 1000*attempt, attempt < 50); every reject is
    recorded in the slot receipt with its reason;
  - acceptance = safe start (REJECT-based _ensure_clear_start) + true spacetime encounter for
    contested/hard intents (+ registered pressure cap 1.5 m);
  - output is ATOMIC (tmp -> sha256 -> rename) into the scenarios/final100/ namespace + a slot
    receipt (seed used, attempt, rejects, scenario sha, stack sha); resume skips a slot only
    when scenario + receipt + sha all match;
  - a slot that exhausts its retry budget writes a FAILED receipt: the 100-slot registration
    stays intact and the failure is visible, never backfilled.
Driver: ops/final100.sh (single worker, 5 slots per fresh interpreter).
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); os.chdir(HERE)
import populate as P
PRESSURE_CAP = 1.5   # m: registered acceptance cap on corridor geometry pressure

ap = argparse.ArgumentParser()
ap.add_argument("--plan", required=True)
ap.add_argument("--slots", required=True, help="A-B inclusive slot range for this process")
args = ap.parse_args()
plan = json.load(open(args.plan))
_chk = dict(plan); _got = _chk.pop("plan_sha")
assert hashlib.sha256(json.dumps(_chk, sort_keys=True).encode()).hexdigest()[:16] == _got, \
    "PLAN INTEGRITY: plan_sha does not verify -- the registration was edited after emission"
cur = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=HERE, text=True).strip()
if not plan.get("draft"):
    assert cur == plan["stack_sha"], \
        f"GIT LOCK: tree {cur[:12]}.. != frozen plan stack {str(plan['stack_sha'])[:12]}.. -- refused"
    import stack_manifest as SM
    have = SM.runtime_shas()
    assert have == plan["stack_runtime"], (
        "RUNTIME LOCK: the loaded binary set differs from the plan's frozen stack_runtime -- "
        f"have {json.dumps(have['so_sha256'])[:120]}")
NS = "scenarios/final100_draft" if plan.get("draft") else "scenarios/final100"
#   07-21 ruling: draft output is ISOLATED -- a draft artefact can never be silently reused by a
#   real plan (different namespace + plan_sha-bound resume below)
a, b = (int(x) for x in args.slots.split("-"))
os.makedirs(NS, exist_ok=True)

_ENVS = {}


def get_env(map_seed, block):
    key = (int(map_seed), str(block))
    if key not in _ENVS:
        from scenario_designer3d import build_env
        _ENVS[key] = build_env(int(map_seed), interactive=False, block_str=str(block))
    return _ENVS[key]


def sha_file(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()[:16]


for slot in plan["slots"][a:b + 1]:
    out = os.path.join(NS, os.path.basename(slot["out"])); rp = out + ".receipt.json"
    if os.path.exists(out) and os.path.exists(rp):
        rc = json.load(open(rp))
        if (rc.get("status") == "ok" and rc.get("slot") == slot
                and rc.get("plan_sha") == plan["plan_sha"] and rc.get("stack_sha") == cur
                and rc.get("scenario_sha") == sha_file(out)):
            # 07-21: resume is PLAN-BOUND -- status + plan_sha + stack_sha + slot + file sha must
            # ALL match; a draft artefact or another plan's output can never be silently reused
            print(f"[f100 {slot['slot_id']}] SKIP (scenario+receipt+plan+stack match)")
            continue
        print(f"[f100 {slot['slot_id']}] stale/foreign artefacts -> regenerating")
    prm = dict(slot["params"])
    tier = prm.get("tier")
    nw, nc, nv = (P.TIERS[tier] if tier else (prm.get("peds", 5), prm.get("crossers", 3),
                                              prm.get("vehicles", 3)))
    env = get_env(slot["map_seed"], slot["block"])
    rejects, accepted = [], None
    for attempt in range(50):
        seed_used = slot["root_seed"] + 1000 * attempt
        scn, stats = P.populate(env.engine, seed=seed_used,
                                n_walkers=nw, n_crossers=nc, n_vehicles=nv)
        scn["_gen_seed_used"] = seed_used; scn["_gen_attempt"] = attempt
        scn["_engine"] = env.engine
        rng_a = np.random.default_rng(seed_used * 31 + 7)
        if prm.get("hard"):
            cr = [m for m in scn["movers"] if m["cls"] == "pedestrian" and len(m["path"]) == 2]
            order = list(rng_a.permutation(len(cr)))
            scn = P._contest(scn, rng_a, [cr[i] for i in order] or
                             [m for m in scn["movers"] if m["cls"] == "pedestrian"],
                             n_anchor=3, occlude=True, veh_align=True)
        elif prm.get("contested"):
            scn = P.make_contested(scn, rng_a)
        else:
            if P._ensure_clear_start(scn) is None:
                scn["_reject"] = True
        scn.pop("_engine", None)
        if scn.pop("_reject", False):
            rejects.append(dict(attempt=attempt, seed=seed_used, reason="no_safe_start"))
            continue
        if (prm.get("hard") or prm.get("contested")) and not P._encounter_ok(scn):
            rejects.append(dict(attempt=attempt, seed=seed_used, reason="no_true_encounter"))
            continue
        pr = P.corridor_pressure(scn)
        if pr > PRESSURE_CAP:
            rejects.append(dict(attempt=attempt, seed=seed_used, reason=f"pressure_{pr:.2f}"))
            continue
        accepted = (scn, stats, pr, seed_used, attempt)
        break
    if accepted is None:
        json.dump(dict(slot=slot, status="FAILED", rejects=rejects, stack_sha=cur,
                       plan_sha=plan["plan_sha"]),
                  open(rp + ".tmp", "w"), indent=1)
        os.replace(rp + ".tmp", rp)
        print(f"[f100 {slot['slot_id']}] FAILED after 50 attempts ({len(rejects)} rejects "
              f"recorded) -- slot stays visible, never backfilled", flush=True)
        continue
    scn, stats, pr, seed_used, attempt = accepted
    scn["map"] = dict(seed=slot["map_seed"], block_str=slot["block"])
    prm_rec = dict(prm); prm_rec["split"] = slot["split"]
    P._finalize_scn(scn, slot["name"], slot["family"], prm_rec,
                    slot["map_seed"], slot["block"], slot["root_seed"], cluster=slot["cluster"])
    tmp = out + ".tmp"
    json.dump(scn, open(tmp, "w"), indent=1)
    os.replace(tmp, out)
    json.dump(dict(slot=slot, status="ok", seed_used=seed_used, attempt=attempt,
                   rejects=rejects, pressure=round(pr, 2),
                   encounters_verified=int(scn.get("encounters_verified", 0)),
                   scenario_sha=sha_file(out), stack_sha=cur, plan_sha=plan["plan_sha"]),
              open(rp + ".tmp", "w"), indent=1)
    os.replace(rp + ".tmp", rp)
    print(f"[f100 {slot['slot_id']}] OK {slot['name']} attempt={attempt} seed={seed_used} "
          f"pressure={pr:.2f} rejects={len(rejects)} sha={sha_file(out)}", flush=True)

for env in _ENVS.values():
    try:
        env.close()
    except Exception:
        pass
