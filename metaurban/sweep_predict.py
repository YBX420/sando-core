"""sweep_predict — statistical A/B/ablation over noise seeds for ONE (scene, vmax). Prints one JSON line.

Three modes, SAME cylinder cert + d_safe + maneuvering, differing only in what they know:
  predict   = ours: certify against the KF-PREDICTED mover (obs_vel=v).
  nopredict = ablation: certify against the mover's CURRENT position (obs_vel=0) — isolates prediction's value.
  native    = plain EGO reacting to current occupancy (no certificate; matched-safety grid inflation).
EGO's C++ floods stdout, so we redirect fd 1 to /dev/null around each run and print the JSON on the real stdout.

  python metaurban/sweep_predict.py <scene> <vmax> [nseed=12]   # run from sando-core root
"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from ego_maneuver import run_episode

SCENE = sys.argv[1]; VMAX = float(sys.argv[2]); NSEED = int(sys.argv[3]) if len(sys.argv) > 3 else 12
MODES = {"predict": dict(mode="ours", predict=True),
         "nopredict": dict(mode="ours", predict=False),
         "native": dict(mode="native")}


def run_quiet(**kw):
    old = os.dup(1); dn = os.open(os.devnull, os.O_WRONLY); os.dup2(dn, 1)
    try:
        return run_episode(**kw)
    finally:
        os.dup2(old, 1); os.close(dn); os.close(old)


def agg(rs):
    rc = [r for r in rs if r["reached"] and not r["collided"]]
    return dict(n=len(rs), reach=round(sum(r["reached"] for r in rs) / len(rs), 3),
                coll=round(sum(r["collided"] for r in rs) / len(rs), 3),
                t=round(float(np.mean([r["time_s"] for r in rc])), 2) if rc else None,
                clr=round(float(np.mean([r["min_clr"] for r in rs])), 3))


rows = {m: [] for m in MODES}
for seed in range(NSEED):
    for m, kw in MODES.items():
        rows[m].append(run_quiet(max_vel=VMAX, scene_name=SCENE, seed=seed, **kw))
print("JSONOUT" + json.dumps(dict(scene=SCENE, vmax=VMAX, nseed=NSEED, **{m: agg(rows[m]) for m in MODES})))
