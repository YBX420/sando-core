"""Offline racing-line workbench (2026-07-16, 塔菲大人's min-jerk formulation probe).

Reads a GTDUMP JSONL (render_3d_video.py, env GTDUMP=path) and answers three questions OFFLINE:
  1. FLOOR   -- straight-line time-optimal arrival under (v_max, a_max), movers ignored:
               the kinodynamic floor any planner is measured against.
  2. FLOWN   -- the harness flight re-scored from the dump (t_reach, RMS jerk, min mover clearance):
               the online stack's actual grade, same ruler as the offline solve.
  3. RACELINE-- committed min-jerk trajectory against the TRUE mover trajectories (perfect
               prediction, whole-episode commitment): direct transcription, positions at N time
               nodes, jerk^2 objective + hinge penalties (separation / v_max / a_max / goal),
               homotopy seeds {straight, arc-L, arc-R, slow-early(yield), fast-early(preempt)},
               outer bisection on total time T. Feasibility = penalties ~ 0 + dense-sample
               clearance check (0.02 s grid; SAMPLED check, NOT the Bernstein certificate --
               offline theory ceiling, not a certified flight).

Honesty notes: the offline solve knows the FULL future (better than any online CV prediction) and
commits once (no replan loop) -- it is the theoretical ceiling of "predict perfectly, decide once,
never hesitate", i.e. an upper bound on what the racing-line architecture could buy online.
Clearances are SURFACE-to-surface (minus both radii), same convention as the harness min_clr.
"""
import json
import sys

import numpy as np

DRONE_R = 0.25          # body radius (drone_bbox/2 -- matches par.drone_radius)
CERT_MARGIN = 0.45      # thin-arm cert-equivalent standoff above surfaces: MAN_DSAFE 0.15 + q 0.05 + body 0.25
NEAR_GATE = 8.0         # movers whose whole GT track stays farther than this off the straight route are ignored


def load_dump(path):
    meta = None; statics = None; ticks = []
    with open(path) as f:
        for line in f:
            row = json.loads(line)
            if "meta" in row:
                meta = row["meta"]; statics = np.asarray(row.get("statics", []), float)
            else:
                ticks.append(row)
    assert meta is not None and ticks, f"empty/withoutmeta dump: {path}"
    return meta, statics, ticks


def mover_tracks(ticks):
    """oid -> dict(t[], x[], y[], r, cls). GT positions per planner tick."""
    trk = {}
    for row in ticks:
        for (oid, cls, x, y, vx, vy, r, h) in row["movers"]:
            d = trk.setdefault(oid, dict(t=[], x=[], y=[], r=float(r), cls=cls, h=float(h)))
            d["t"].append(float(row["t"])); d["x"].append(float(x)); d["y"].append(float(y))
    for d in trk.values():
        d["t"] = np.asarray(d["t"]); d["x"] = np.asarray(d["x"]); d["y"] = np.asarray(d["y"])
    return trk


def mover_pos(d, tq):
    """piecewise-linear GT interpolation, clamped outside the recorded window."""
    return (np.interp(tq, d["t"], d["x"]), np.interp(tq, d["t"], d["y"]))


def flown_report(meta, ticks, trk):
    goal = np.asarray(meta["goal"][:2], float); mg = float(meta["min_goal"])
    ts = np.asarray([r["t"] for r in ticks], float)
    P = np.asarray([r["drone"][:2] for r in ticks], float)
    V = np.asarray([r["drone"][3:5] for r in ticks], float)
    reach_i = next((i for i in range(len(ts)) if np.linalg.norm(P[i] - goal) <= mg), len(ts) - 1)
    t_reach = ts[reach_i]
    # jerk from flown velocity (2nd diff of v), same spirit as the harness _rms_jerk
    dt = np.diff(ts[:reach_i + 1]); dt[dt < 1e-6] = 1e-6
    acc = np.diff(V[:reach_i + 1], axis=0) / dt[:, None]
    jerk = np.diff(acc, axis=0) / dt[1:, None]
    rms_j = float(np.sqrt(np.mean(np.sum(jerk ** 2, axis=1)))) if len(jerk) else 0.0
    clr = np.inf
    for d in trk.values():
        mx, my = mover_pos(d, ts[:reach_i + 1])
        sep = np.hypot(P[:reach_i + 1, 0] - mx, P[:reach_i + 1, 1] - my) - d["r"] - DRONE_R
        clr = min(clr, float(sep.min()))
    return dict(t_reach=float(t_reach), rms_jerk=rms_j, min_clr=float(clr), P=P[:reach_i + 1], ts=ts[:reach_i + 1])


def kino_floor(meta):
    """1-D time-optimal from rest to the min_goal boundary (arrival speed free)."""
    s0 = np.asarray(meta["start"][:2], float); g = np.asarray(meta["goal"][:2], float)
    D = max(np.linalg.norm(g - s0) - float(meta["min_goal"]), 0.0)
    vm, am = float(meta["v_max"]), float(meta["a_max"])
    if D <= vm * vm / (2 * am):                      # never reaches v_max
        return float(np.sqrt(2 * D / am))
    return float(vm / am + (D - vm * vm / (2 * am)) / vm)


# ---------------- racing line: direct transcription, min-jerk at fixed T ----------------

def solve_fixed_T(meta, trk, T, seed_kind, N=64, w_pen=4e2, verbose=False):
    from scipy.optimize import minimize
    s0 = np.asarray(meta["start"][:2], float); g = np.asarray(meta["goal"][:2], float)
    mg = float(meta["min_goal"]); vm = float(meta["v_max"]); am = float(meta["a_max"])
    tt = np.linspace(0.0, T, N); h = tt[1] - tt[0]
    # movers that ever come near the corridor (cheap gate vs the straight segment)
    seg = g - s0; L = np.linalg.norm(seg); u = seg / L
    act = []
    for d in trk.values():
        mx, my = mover_pos(d, tt)
        w = np.c_[mx, my] - s0
        along = np.clip(w @ u, 0.0, L)
        lat = np.linalg.norm(w - along[:, None] * u[None, :], axis=1)
        if lat.min() < NEAR_GATE:
            act.append((np.c_[mx, my], d["r"] + DRONE_R + CERT_MARGIN))
    # seed trajectories: straight with a time profile / lateral arc bulge
    frac = tt / T
    prof = dict(straight=frac,
                yield_=np.clip(frac ** 1.6, 0, 1),        # slow early, fast late (pass behind)
                preempt=np.clip(frac ** 0.65, 0, 1))      # fast early (pass in front)
    n = np.array([-u[1], u[0]])
    P0 = s0[None, :] + prof.get(seed_kind, frac)[:, None] * (L * u)[None, :]
    if seed_kind in ("arcL", "arcR"):
        bulge = (3.0 if seed_kind == "arcL" else -3.0) * np.sin(np.pi * frac)
        P0 = P0 + bulge[:, None] * n[None, :]
    x0 = P0[1:].ravel()                                   # node 0 pinned at start

    def unpack(z):
        return np.vstack([s0[None, :], z.reshape(N - 1, 2)])

    def cost(z):
        P = unpack(z)
        v = np.diff(P, axis=0) / h
        a = np.diff(v, axis=0) / h
        j = np.diff(a, axis=0) / h
        J = h * float(np.sum(j ** 2)) * 1e-3              # min-jerk core (scaled)
        pen = 0.0
        pen += float(np.sum(np.maximum(np.linalg.norm(v, axis=1) - vm, 0.0) ** 2))
        pen += float(np.sum(np.maximum(np.linalg.norm(a, axis=1) - am, 0.0) ** 2)) * 0.25
        for (M, R) in act:
            sep = np.hypot(P[:, 0] - M[:, 0], P[:, 1] - M[:, 1]) - R
            pen += float(np.sum(np.maximum(-sep, 0.0) ** 2)) * 4.0
        dg = np.linalg.norm(P[-1] - g)
        pen += max(dg - mg * 0.5, 0.0) ** 2 * 4.0
        # start at rest-ish (the harness starts from hover)
        pen += float(np.sum(v[0] ** 2)) * 0.05
        return J + w_pen * pen

    res = minimize(cost, x0, method="L-BFGS-B", options=dict(maxiter=600, maxfun=200000))
    P = unpack(res.x)
    # dense SAMPLED feasibility audit (cubic-ish: linear interp between nodes at 0.02 s)
    td = np.arange(0.0, T, 0.02)
    Pd = np.c_[np.interp(td, tt, P[:, 0]), np.interp(td, tt, P[:, 1])]
    vd = np.diff(Pd, axis=0) / 0.02
    v_ok = float(np.linalg.norm(vd, axis=1).max()) <= vm * 1.03
    clr = np.inf
    for d in trk.values():
        mx, my = mover_pos(d, td)
        sep = np.hypot(Pd[:, 0] - mx, Pd[:, 1] - my) - d["r"] - DRONE_R
        clr = min(clr, float(sep.min()))
    reach = np.linalg.norm(P[-1] - g) <= mg
    feas = reach and v_ok and clr >= CERT_MARGIN - DRONE_R - 1e-3   # surface clearance >= standoff(0.2)+q
    v = np.diff(P, axis=0) / h; a = np.diff(v, axis=0) / h; j = np.diff(a, axis=0) / h
    rms_j = float(np.sqrt(np.mean(np.sum(j ** 2, axis=1))))
    if verbose:
        print(f"    T={T:5.2f} {seed_kind:9s} feas={feas} clr={clr:6.3f} vmax={np.linalg.norm(vd,axis=1).max():5.2f} "
              f"reach={reach} rms_j={rms_j:6.1f}")
    return dict(feasible=bool(feas), clr=float(clr), rms_jerk=rms_j, P=P, T=float(T), seed=seed_kind)


def raceline(meta, trk, T_lo, T_hi, verbose=True):
    seeds = ("straight", "yield_", "preempt", "arcL", "arcR")
    best = None
    for T in np.arange(T_lo, T_hi + 1e-9, 0.2):           # coarse ascending scan: first feasible T wins
        sols = [solve_fixed_T(meta, trk, float(T), s, verbose=verbose) for s in seeds]
        ok = [s for s in sols if s["feasible"]]
        if ok:
            best = min(ok, key=lambda s: s["rms_jerk"])
            break
    return best


def main(path):
    meta, statics, ticks = load_dump(path)
    trk = mover_tracks(ticks)
    fl = flown_report(meta, ticks, trk)
    floor = kino_floor(meta)
    print(f"== {path}")
    print(f"scene: |start->goal|={np.linalg.norm(np.asarray(meta['goal'][:2])-np.asarray(meta['start'][:2])):.1f}m "
          f"v_max={meta['v_max']} a_max={meta['a_max']} movers={len(trk)}")
    print(f"FLOOR  (straight, no movers)   : T={floor:5.2f}s")
    print(f"FLOWN  (online stack, from dump): T={fl['t_reach']:5.2f}s  rms_jerk={fl['rms_jerk']:6.1f}  "
          f"min_clr={fl['min_clr']:6.3f}m")
    best = raceline(meta, trk, T_lo=max(floor - 0.2, 0.5), T_hi=floor + 6.0)
    if best is None:
        print("RACELINE: no feasible committed trajectory found in T window (?!)")
    else:
        print(f"RACELINE (offline min-jerk, perfect prediction, committed once): "
              f"T={best['T']:5.2f}s  seed={best['seed']}  rms_jerk={best['rms_jerk']:6.1f}  "
              f"min_clr={best['clr']:6.3f}m")
        np.save(path.replace(".jsonl", "_raceline.npy"), best["P"])
    return 0


if __name__ == "__main__":
    for p in sys.argv[1:]:
        main(p)
