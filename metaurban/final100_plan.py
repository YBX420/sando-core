"""final100_plan — pre-register the NEW 100-scenario benchmark (07-21 ruling, step-6 contract).

The paper's claim is a BALANCED BENCHMARK MIXTURE, pre-registered before a single scenario is
generated:
  - 100 slots; the four family labels drawn INDEPENDENTLY, equal probability, by a frozen RNG
    (expected ~25 each; NEVER back-filled to round counts after seeing results);
  - 60 cal / 40 test split assigned by a frozen shuffle over the slots (split is a property of
    the SLOT, fixed before generation -- cal computes quantiles only, test opens exactly once);
  - per slot: root seed, the registered retry rule (seed = root + 1000*attempt, attempt < 50),
    map seed and block from registered sets, cluster == slot name (no multistarts here);
  - the stack SHA is REQUIRED and generation refuses on any other tree (--draft emits a
    tooling-test plan that can never become a paper artifact);
  - animals stay in the stress suite, not in the main coverage theorem.

Families -> populate kernel dials:
  single_timed_encounter    one clean timed crossing (contested, sparse)
  dense_multibody           dense mixed traffic through the corridor (rush tier, contested)
  occlusion_coast_reacquire occluding boards + staggered timed crossers (hard: late reveal,
                            cone-edge coast/reacquire pressure)
  high_dynamics_vehicles    vehicle-heavy, timed vehicle alignment + head-on pressure (hard)
"""
import argparse
import hashlib
import json
import os
import subprocess

import numpy as np

FAMILIES = dict(
    single_timed_encounter=dict(peds=4, crossers=2, vehicles=0, contested=True, hard=False),
    dense_multibody=dict(tier="rush", contested=True, hard=False),
    occlusion_coast_reacquire=dict(peds=6, crossers=3, vehicles=1, contested=False, hard=True),
    high_dynamics_vehicles=dict(peds=3, crossers=2, vehicles=4, contested=False, hard=True),
)
MAP_SEEDS = (3, 11, 17, 29)      # registered map-seed set (block X is the validated generator block)
BLOCKS = ("X",)
RNG_SEED = 20260721
RETRY_RULE = "seed = root_seed + 1000*attempt, attempt < 50; every reject recorded in the receipt"

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--stack-sha", default=None, help="frozen stack commit (REQUIRED unless --draft)")
ap.add_argument("--draft", action="store_true", help="tooling self-test plan; never a paper artifact")
ap.add_argument("--balanced", action="store_true",
                help="the ruling's SECOND registered option: fixed 25x4 families, frozen shuffle "
                     "(balanced by construction). Default = independent equal-probability draws.")
args = ap.parse_args()
assert not os.path.exists(args.out), \
    "plans are IMMUTABLE: emit to a fresh path, never overwrite a registration (07-21 ruling)"
import stack_manifest as SM
if not args.draft:
    assert args.stack_sha, "a real plan pins the FROZEN stack SHA (use --draft for tooling tests)"
    cur = subprocess.check_output(["git", "rev-parse", "HEAD"],
                                  cwd=os.path.dirname(os.path.abspath(__file__)), text=True).strip()
    assert cur == args.stack_sha, \
        f"plan must be emitted FROM the frozen tree (FULL sha {cur[:12]}.. != {args.stack_sha[:12]}..)"

rng = np.random.default_rng(RNG_SEED)
fams = list(FAMILIES)
if args.balanced:
    fam_seq = [f for f in fams for _ in range(25)]
    rng.shuffle(fam_seq)
slots = []
for i in range(100):
    fam = fam_seq[i] if args.balanced else fams[int(rng.integers(len(fams)))]
    slots.append(dict(slot_id=i, name=f"f100_{i:03d}_{fam}", family=fam,
                      params=dict(FAMILIES[fam]),
                      root_seed=int(rng.integers(1, 10**8)),
                      map_seed=int(MAP_SEEDS[int(rng.integers(len(MAP_SEEDS)))]),
                      block=str(BLOCKS[int(rng.integers(len(BLOCKS)))]),
                      cluster=f"f100_{i:03d}_{fam}",
                      out=f"scenarios/final100/f100_{i:03d}_{fam}.json"))
labels = ["cal"] * 60 + ["test"] * 40
rng.shuffle(labels)
for s, lab in zip(slots, labels):
    s["split"] = lab

hist = {f: sum(1 for s in slots if s["family"] == f) for f in fams}
plan = dict(schema="final100.v1", draft=bool(args.draft),
            family_scheme=("fixed_25x4_shuffled" if args.balanced else "iid_equal_prob"),
            stack_sha=(args.stack_sha or "DRAFT"),
            stack_runtime=SM.runtime_shas(),   # 07-21 #3: the BINARIES frozen with this plan --
            #   generation verifies the loaded .so set equals this record, not just the git tree
            rng_seed=RNG_SEED,
            retry_rule=RETRY_RULE, map_seeds=list(MAP_SEEDS), blocks=list(BLOCKS),
            families=FAMILIES, family_histogram=hist,
            split=dict(cal=60, test=40,
                       note="split fixed per slot BEFORE generation; cal computes quantiles "
                            "only; test opens exactly once, never for tuning"),
            animals="stress suite only, excluded from the main coverage theorem",
            slots=slots)
blob = json.dumps(plan, sort_keys=True).encode()
plan["plan_sha"] = hashlib.sha256(blob).hexdigest()[:16]
tmp = args.out + ".tmp"
json.dump(plan, open(tmp, "w"), indent=1)
os.replace(tmp, args.out)
print(f"[final100] plan {'(DRAFT) ' if args.draft else ''}-> {args.out}")
print(f"[final100] families: {hist}  split: 60 cal / 40 test  plan_sha={plan['plan_sha']}")
