"""tune_config — score one (INFL_X, Q_CONF) config: does ours DOMINATE native (coll=0 AND time<=native)
across scenes x vmax x noise seeds? Prints one JSON line. EGO stdout is muted via fd redirection.

  python metaurban/tune_config.py <INFL_X> <Q_CONF> [nseed=6]
"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
INFL_X, Q_CONF = sys.argv[1], sys.argv[2]
NSEED = int(sys.argv[3]) if len(sys.argv) > 3 else 6
os.environ["EGO_INFLX"] = INFL_X; os.environ["EGO_QCONF"] = Q_CONF
import numpy as np
from ego_maneuver import run_episode

SCENES = ["crossers", "head_on", "wall_and_crosser", "gauntlet"]
VMAX = [5, 8, 12]


def quiet(**kw):
    old = os.dup(1); dn = os.open(os.devnull, os.O_WRONLY); os.dup2(dn, 1)
    try:
        return run_episode(**kw)
    finally:
        os.dup2(old, 1); os.close(dn); os.close(old)


cells = []
for sc in SCENES:
    for v in VMAX:
        o = [quiet(mode="ours", max_vel=v, scene_name=sc, seed=s) for s in range(NSEED)]
        n = [quiet(mode="native", max_vel=v, scene_name=sc, seed=s) for s in range(NSEED)]
        def mt(rs):
            rc = [r for r in rs if r["reached"] and not r["collided"]]
            return float(np.mean([r["time_s"] for r in rc])) if rc else None
        oc = float(np.mean([r["collided"] for r in o])); ot = mt(o); nt = mt(n)
        oreach = float(np.mean([r["reached"] for r in o]))
        win = (oc == 0.0 and oreach == 1.0 and ot is not None and nt is not None and ot <= nt + 0.05)
        cells.append(dict(scene=sc, v=v, ocoll=round(oc, 2), oreach=round(oreach, 2),
                          ot=round(ot, 1) if ot else None, nt=round(nt, 1) if nt else None, win=win))
wins = sum(c["win"] for c in cells)
any_coll = any(c["ocoll"] > 0 for c in cells)
losses = [f"{c['scene']}v{c['v']}({'C' if c['ocoll'] else ''}{c['ot']}>{c['nt']})" for c in cells if not c["win"]]
print("JSONOUT" + json.dumps(dict(inflx=float(INFL_X), qconf=float(Q_CONF), nseed=NSEED,
                                  wins=wins, total=len(cells), any_collision=any_coll, losses=losses)))
