"""ego_mu_bench — EGO line ON THE METAURBAN ENGINE, headless (塔菲大人 2026-07-08 goal).

Reuses MetaUrbanNavEnv's live world (Bullet crowd, DT=0.3 via MU_SUBSTEPS=3) and its perception
front-end, but flies the EGO certified stack instead of a PPO policy:
    perceive (env._tracks) -> predicted-occupancy cloud -> ego.update_cloud -> SL.build_cylinders
    -> SL.maneuver_decide_v2 -> set-point -> accel action into the env.
10 episodes per version tag; per-episode row appended to a jsonl ledger so successive invocations
(one per config version) accumulate into a comparison table. 2-D world honesty: mover cylinders are
made effectively infinite-height so the vertical candidates (over/climb) can never certify — the
env's clearance is planar and a 'fly-over' would still count as a hit.
"""
import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, _HERE)
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--tag", required=True, help="version label for the ledger")
ap.add_argument("--episodes", type=int, default=10)
ap.add_argument("--ledger", default="out/ppo_eval/ego_mu_ledger.jsonl")
args = ap.parse_args()

from metaurban_nav_env import MetaUrbanNavEnv, DT
from ego_bridge import EGOPlanner
import safety_layer as SL
import replay_core as RC          # battle-tested pieces: _cyl_cloud, _plan_r (env gates read at import)

DELTA = float(os.environ.get("DELTA_OVR", 0.30))
EPS = float(os.environ.get("CALIB_EPS", 0.10))
HOR = 7.5
CAL2 = SL.load_calib_v2(eps=EPS)

env = MetaUrbanNavEnv(seed=0)
ego = EGOPlanner(map_origin=(-40, -40, -1), map_size=(80, 80, 8), res=0.2, inflation=0.1)
ego.set_params(max_vel=env.max_vel, max_acc=env.max_acc, horizon=HOR)
Z = 1.0                                        # planar world: fixed cruise plane

os.makedirs(os.path.dirname(os.path.join(_HERE, args.ledger)), exist_ok=True)
led = open(os.path.join(_HERE, args.ledger), "a")
rows = []
for ep in range(args.episodes):
    env.reset(seed=1000 + ep)                  # same scene sequence for every version tag
    _start0 = [round(float(x), 2) for x in np.asarray(env.p, float)[:2]]
    _goal0 = [round(float(x), 2) for x in np.asarray(env.goal, float)[:2]]
    state = {}
    evade = 0
    done = False
    info = {}
    while not done:
        trs, _ = env._tracks()
        p3 = np.array([env.p[0], env.p[1], Z])
        v3 = np.array([env.v[0], env.v[1], 0.0])
        goal3 = np.array([env.goal[0], env.goal[1], Z])
        cloud, mlist = [], []
        for tr in trs:
            c0, vv, aa = tr.trk.state()
            cloud += RC._cyl_cloud([np.asarray(tr.xy[:2], float)], RC._plan_r(tr.r, str(tr.cls)),
                                   0.3, min(tr.h, 2.5))
            mlist.append((np.array([c0[0], c0[1], Z]), np.array([vv[0], vv[1], 0.0]),
                          np.zeros(3), tr.r, 50.0, str(tr.cls), int(tr.trk.n),
                          int(tr.trk.miss > 0), float(getattr(tr.trk, "nis", None) or 0.0),   # was nis_ewma: nonexistent attr, column was constant 0.0 (2026-07-16 KF audit; .nis is None pre-update)
                          float(getattr(tr.trk, "sigma_v", 0.0))))
        ego.update_cloud(np.asarray(cloud, float) if cloud else np.zeros((0, 3)), p3)
        cyl, _zt = SL.build_cylinders(mlist, None, predict=True, calib_v2=CAL2)
        kind, s = SL.maneuver_decide_v2(ego, p3, v3, np.zeros(3), goal3, 60.0, cyl, state,
                                        cruise_z=Z, horizon=HOR, delta=DELTA)
        if kind in ("around_l2", "around_r2"):
            kind = kind[:-1]
        if kind == "evade":
            evade += 1
            pos, vel = SL.evade_setpoint(p3, [np.asarray(m[0], float) for m in mlist],
                                         env.max_vel, DT, 60.0, (goal3 - p3)[:2] /
                                         max(np.linalg.norm((goal3 - p3)[:2]), 1e-6))
            v_cmd = np.asarray(vel[:2], float)
        else:
            # executor parity with replay_core: the incumbent path is NOT replanned every tick, so
            # the reference must be taken s*DT ahead of the NEAREST point on the committed spline
            # (naive eval at absolute t=s*DT pins a zero-speed start to the spline origin forever --
            # the ep7 perpetual-stall bug, 2026-07-08).
            dur = max(ego.duration() - 1e-3, 0.0)
            ts = np.linspace(0.0, dur, 25)
            pts = [ego.eval(float(t)) for t in ts]
            dists = [np.linalg.norm(np.asarray(r[0][:2], float) - env.p) if r is not None else 1e9
                     for r in pts]
            t_near = float(ts[int(np.argmin(dists))])
            rr = ego.eval(min(t_near + s * DT, dur))
            p_ref = np.asarray(rr[0][:2], float) if rr is not None else env.goal
            v_cmd = (p_ref - env.p) / DT
        sp = float(np.linalg.norm(v_cmd))
        if sp > env.max_vel:
            v_cmd *= env.max_vel / sp
        act = np.clip((v_cmd - env.v) / DT / env.max_acc, -1, 1)
        _o, _r, term, trunc, info = env.step(act)
        done = term or trunc
    rec = dict(tag=args.tag, ep=ep, reached=bool(info.get("reached")),
               start=_start0, goal=_goal0,
               collided=bool(info.get("collided")), evade=int(evade),
               min_clr=round(float(info.get("min_clr", np.nan)), 2), ticks=int(env.tick),
               clean=bool(info.get("reached") and not info.get("collided") and evade == 0))
    rows.append(rec)
    led.write(json.dumps(rec) + "\n"); led.flush()
    print(f"[{args.tag}] ep{ep}: reach={rec['reached']} coll={rec['collided']} "
          f"evade={evade} clr={rec['min_clr']} t={env.tick * DT:.0f}s", flush=True)

n = len(rows)
print(f"\n[{args.tag}] SUMMARY n={n}: clean={sum(r['clean'] for r in rows)}/{n} "
      f"reach={sum(r['reached'] for r in rows)}/{n} coll={sum(r['collided'] for r in rows)} "
      f"evade_total={sum(r['evade'] for r in rows)}", flush=True)
