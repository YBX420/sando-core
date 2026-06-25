"""graft_demo — wrap 5 DIFFERENT planner trajectory representations with our continuous-time Bernstein certificate.

These 5 representations cover all ~10 surveyed portable planners:
  1. RapidQuadrocopterTrajectories (Mueller) -- closed-form min-jerk QUINTIC (power basis)
  2. min-snap (Mellinger/Richter, mav_trajectory_generation) -- boundary-constrained SEPTIC (deg-7 power basis)
  3. GCOPTER / MINCO (Wang) -- min-jerk quintic emitted as a 3x(D+1) DESCENDING-power CoefficientMat
  4. cubic B-spline (Fast-Planner / EGO) -- uniform cubic B-spline control points
  5. native piecewise Bezier (TGK-Planner / Btraj) -- Bernstein control points directly

Each: generate a trajectory in the planner's OWN representation, adapt to BSeg (the only planner-specific step),
certify with the SAME cert_capi, and CROSS-CHECK the verdict against a dense brute-force min-distance. Demonstrates
the certificate is planner-agnostic: one cert, five representations, all sound.

Run:  python metaurban/graft_demo.py
"""
import os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from cert_bridge import (Certifier, monomial_to_bseg, native_bezier_to_bseg,
                         minco_descending_to_bseg, bspline_to_bseg, _bezier_eval)

OBS = np.array([3.0, 0.55])   # static cylinder centre (xy); R below
R = 0.8


# ---- planner-faithful trajectory generators (each in ITS native representation) ----
def rapidquad_quintic(p0, v0, a0, pf, vf, af, T):
    """Mueller's closed-form minimum-jerk quintic per axis. Returns ascending-power monomial coeffs (3, 6)."""
    p0, v0, a0, pf, vf, af = map(lambda x: np.asarray(x, float), (p0, v0, a0, pf, vf, af))
    dp = pf - p0 - v0 * T - 0.5 * a0 * T * T
    dv = vf - v0 - a0 * T
    da = af - a0
    al = (720 * dp - 360 * T * dv + 60 * T * T * da) / T ** 5
    be = (-360 * T * dp + 168 * T * T * dv - 24 * T ** 3 * da) / T ** 5
    ga = (60 * T * T * dp - 24 * T ** 3 * dv + 3 * T ** 4 * da) / T ** 5
    # p(t) = p0 + v0 t + a0/2 t^2 + ga/6 t^3 + be/24 t^4 + al/120 t^5  -> ascending coeffs a_k (per axis)
    a = np.stack([p0, v0, a0 / 2, ga / 6, be / 24, al / 120], axis=0)   # (6,3)
    return a


def minsnap_septic(p0, v0, a0, j0, pf, vf, af, jf, T):
    """deg-7 polynomial with full position/vel/acc/jerk boundary at both ends (min-snap single segment).
    Returns ascending-power monomial coeffs (8,3)."""
    c = np.zeros((8, 3))
    c[0] = p0; c[1] = v0; c[2] = np.asarray(a0) / 2; c[3] = np.asarray(j0) / 6   # start boundary fixes c0..c3
    # end boundary: 4 eqns in c4..c7. Build M (4x4) for [p,v,a,j](T) contributions of c4..c7, subtract c0..c3 part.
    ks = np.arange(8)
    def derivs(coeffs):
        p = sum(coeffs[k] * T ** k for k in ks)
        v = sum(k * coeffs[k] * T ** (k - 1) for k in ks if k >= 1)
        a = sum(k * (k - 1) * coeffs[k] * T ** (k - 2) for k in ks if k >= 2)
        jj = sum(k * (k - 1) * (k - 2) * coeffs[k] * T ** (k - 3) for k in ks if k >= 3)
        return np.array([p, v, a, jj])
    base = derivs(c)                                                   # contribution of known c0..c3
    M = np.zeros((4, 4))
    for col, k in enumerate(range(4, 8)):
        e = np.zeros((8, 3)); e[k] = 1.0
        M[:, col] = derivs(e)[:, 0]                                    # same for all axes
    target = np.stack([pf, vf, af, jf], axis=0)                        # (4,3)
    rhs = target - base                                               # (4,3)
    c47 = np.linalg.solve(M, rhs)                                     # (4,3)
    c[4:8] = c47
    return c


def minco_quintic_descending(p0, v0, a0, pf, vf, af, T):
    """Same min-jerk quintic but emitted as GCOPTER's 3 x 6 DESCENDING-power CoefficientMat (col0 = tau^5),
    real time tau in [0,T] -- to exercise minco_descending_to_bseg (column reversal + dur^j)."""
    a_asc = rapidquad_quintic(p0, v0, a0, pf, vf, af, T)   # (6,3) ascending, REAL time (no dur normalisation)
    C = np.zeros((3, 6))
    for j in range(6):
        C[:, 5 - j] = a_asc[j]                              # descending: col(5-j) = ascending coeff a_j
    return C


# ---- the demo: adapt each -> certify -> brute-force cross-check ----
def _brute_clear(ctrl_pts, t0s, durs):
    """dense min horizontal distance of the piecewise-Bezier trajectory to OBS over its whole span."""
    mind = 1e18
    for seg in ctrl_pts:
        for s in np.linspace(0, 1, 1500):
            p = _bezier_eval(seg, s)
            mind = min(mind, float(np.hypot(p[0] - OBS[0], p[1] - OBS[1])))
    return mind


def run_case(name, ctrl_pts, t0s, durs):
    okc, mg = Certifier.certify_horizontal(ctrl_pts, t0s, durs, [OBS[0], OBS[1], 1.5], R, t_hi=-1, n_axes=2)
    clr = _brute_clear(ctrl_pts, t0s, durs)
    truth = clr >= R
    sound = (not okc) or truth
    print(f"  {name:<34} cert={'PASS' if okc else 'reject':<6} margin={mg:+.3f} | brute min_dist={clr:.3f} (R={R}) "
          f"truth_clear={truth}  {'OK' if sound else 'UNSOUND!!'}")
    return sound


def main():
    print("=== planner-agnostic graft: 5 representations, ONE Bernstein certificate ===")
    Tt = 2.0
    # a clearing trajectory (passes ABOVE the obstacle in y) and a grazing one (through it) per representation
    start = [0, 0, 1.5]; goal = [6, 0, 1.5]
    clearP = [3, 1.6]   # waypoint that clears OBS (y=1.6 vs OBS y=0.55, dist 1.05 > R)
    allsound = True

    # 1. RapidQuad quintic, bowed UP in y to clear the obstacle (ascending coeffs (6,3) -> 1 segment)
    a_up = rapidquad_quintic([0, 0, 1.5], [3, 2.2, 0], [0, 0, 0], [6, 0, 1.5], [3, -2.2, 0], [0, 0, 0], Tt)
    cb, t0, du = monomial_to_bseg(a_up[None], [Tt])
    allsound &= run_case("1. RapidQuad quintic (clears)", cb, t0, du)

    # 2. min-snap septic, straight through (grazes)
    c = minsnap_septic([0, 0, 1.5], [3, 0, 0], [0, 0, 0], [0, 0, 0],
                       [6, 0, 1.5], [3, 0, 0], [0, 0, 0], [0, 0, 0], Tt)
    cb2, t02, du2 = monomial_to_bseg(c[None], [Tt])
    allsound &= run_case("2. min-snap septic (straight)", cb2, t02, du2)

    # 3. GCOPTER/MINCO descending-power quintic (clears, up-arc)
    C = minco_quintic_descending([0, 0, 1.5], [3, 2.2, 0], [0, 0, 0], [6, 0, 1.5], [3, -2.2, 0], [0, 0, 0], Tt)
    cb3, t03, du3 = minco_descending_to_bseg([C], [Tt])
    allsound &= run_case("3. GCOPTER/MINCO (descending pow)", cb3, t03, du3)

    # 4. cubic B-spline (Fast-Planner): control points arcing up over the obstacle
    ctrlbs = np.array([[0, 0, 1.5], [1.5, 1.2, 1.5], [3, 1.8, 1.5], [4.5, 1.2, 1.5], [6, 0, 1.5], [7, 0, 1.5]], float)
    cb4, t04, du4 = bspline_to_bseg(ctrlbs, [Tt / 3] * (len(ctrlbs) - 3))
    allsound &= run_case("4. cubic B-spline (Fast-Planner)", cb4, t04, du4)

    # 5. native piecewise Bezier (TGK/Btraj): straight-ish, grazes
    bez = np.array([[[0, 0, 1.5], [2, 0.3, 1.5], [4, 0.3, 1.5], [6, 0, 1.5]]], float)
    cb5, t05, du5 = native_bezier_to_bseg(bez, [Tt])
    allsound &= run_case("5. native Bezier (TGK/Btraj)", cb5, t05, du5)

    print(f"\n=== {'ALL SOUND' if allsound else 'A CASE WAS UNSOUND'}: one continuous-time Bernstein cert wraps "
          f"5 distinct planner representations (covers the ~10 surveyed portable planners). ===")
    return 0 if allsound else 1


if __name__ == "__main__":
    sys.exit(main())
