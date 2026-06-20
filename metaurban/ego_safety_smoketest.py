"""Demo: OUR S3 safety envelope ported ONTO EGO-Planner (planner-agnostic certified safety layer).

EGO produces a smooth cubic B-spline. We run the SAME continuous-time conformal-Bernstein deficit
certificate (used for MINCO) on EGO's B-spline -> P(collision)<=eps verdict, no sampling. Demonstrates:
 - an obstacle ON EGO's path  -> certificate FAILS  (the safety layer catches EGO's unsafe trajectory)
 - an obstacle OFF EGO's path -> certificate PASSES
 - SOUNDNESS cross-check: certified => dense-sampled min distance >= R (never a false certify).
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from ego_bridge import EGOPlanner


def brute_min_dist(pl, c0, dur, vel=(0, 0, 0), acc=(0, 0, 0), n=4000):
    c0 = np.asarray(c0, float); vel = np.asarray(vel, float); acc = np.asarray(acc, float)
    m = 1e18
    for t in np.linspace(0, dur, n):
        r = pl.eval(t)
        if r is None:
            continue
        c = c0 + vel * t + 0.5 * acc * t * t
        m = min(m, float(np.linalg.norm(r[0] - c)))
    return m


def main():
    pl = EGOPlanner(map_origin=(-5, -10, -1), map_size=(30, 20, 5), res=0.15, inflation=0.3)
    pl.set_params(max_vel=3.0, max_acc=6.0)
    ok = pl.replan(start=[0, 0, 1.5], vel=[0, 0, 0], acc=[0, 0, 0], goal=[10, 0, 1.5])
    dur = pl.duration()
    print(f"[ego+S3] EGO replan ok={ok} duration={dur:.2f}s  (smooth cubic B-spline)\n")
    if not ok or dur <= 0:
        print("FAIL: EGO produced no trajectory"); return 1

    # (centre, radius R, vel, name).  Verdict checked AGAINST the dense-sampled spatiotemporal min
    # distance (no naive hardcoded expectations): SOUND (cert => bm>=R), reject clear collisions,
    # certify clear-by-margin cases. The continuous-time certificate accounts for TIMING (a crossing
    # mover the drone passes before it arrives is genuinely safe).
    cases = [
        ((5.0, 0.0, 1.5), 1.0, (0, 0, 0), "static ON path (R=1.0)"),
        ((5.0, 8.0, 1.5), 1.0, (0, 0, 0), "static OFF path (R=1.0)"),
        ((5.0, 0.0, 1.5), 0.4, (0, 0, 0), "static ON path (R=0.4)"),
        ((8.0, 0.0, 1.5), 1.0, (-2.0, 0.0, 0.0), "mover HEAD-ON (collision)"),
        ((5.0, 9.0, 1.5), 1.0, (0.0, 1.0, 0.0),  "mover staying OFF path"),
    ]
    fails = 0
    for c0, R, v, name in cases:
        cert, margin = pl.certify(obs_c0=c0, R=R, obs_vel=v)
        bm = brute_min_dist(pl, c0, dur, vel=v)
        sound = (not cert) or (bm >= R - 1e-6)          # certified => truly clear (NO false certify) -- critical
        vacuous = (bm >= R + 0.5) and (not cert)        # clearly safe but not certified
        certified_unsafe = (bm <= R - 0.1) and cert     # certified a clearly-colliding trajectory
        ok = sound and not vacuous and not certified_unsafe
        if not ok:
            fails += 1
        print(f"  [{'OK' if ok else 'FAIL'}] {name:28s} cert={int(cert)}  margin={margin:+.3f}  "
              f"spatiotemporal_min_dist={bm:.3f} vs R={R}  sound={sound}")

    print(f"\n[ego+S3] {len(cases)} cases, {fails} fail")
    print("ALL PASS — our certified safety layer works on EGO's B-spline" if fails == 0
          else "FAIL")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
