"""cert_bridge — PLANNER-AGNOSTIC certificate: certify ANY committed trajectory given as piecewise polynomial.

This is the graft point for wrapping external planners. Each planner outputs a trajectory in some representation
(cubic B-spline, MINCO/min-jerk quintic, min-snap monomial polynomial, MINVO, native piecewise Bezier). The
adapters below convert each to BSeg = per-segment position Bezier/Bernstein control points, which our continuous-
time Bernstein collision certificate (cpp/capi/cert_capi.so) eats directly -- no planner-specific cert code.

  certify_horizontal / certify_above : the cylinder-disjunction halves on an arbitrary trajectory.
  monomial_to_bseg   : power-basis polynomial coeffs -> Bernstein control points (min-snap, MINCO power form).
  native_bezier_to_bseg : passthrough (TGK-Planner / Btraj already emit Bezier control points).
"""
import os, ctypes as C
from math import comb
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SO = os.path.join(os.path.dirname(_HERE), "cpp", "capi", "cert_capi.so")
_lib = C.CDLL(_SO)
_d = C.POINTER(C.c_double)


def _sig(name, *args):
    f = getattr(_lib, name); f.restype = C.c_int; f.argtypes = list(args); return f


_horiz = _sig("cert_horizontal_bseg", _d, C.c_int, C.c_int, _d, _d, _d, _d, _d,
              C.c_double, C.c_double, C.c_double, C.c_double, C.c_int, _d)
_above = _sig("cert_above_bseg", _d, C.c_int, C.c_int, _d, _d,
              C.c_double, C.c_double, C.c_double, C.c_double, C.c_double, _d)


def _p(a):
    a = np.ascontiguousarray(np.asarray(a, np.float64).reshape(-1)); return a, a.ctypes.data_as(_d)


def _power_to_bernstein(deg):
    """Matrix M (deg+1 x deg+1): Bernstein control points c = M @ a for a monomial p(s)=sum a_k s^k on s in [0,1].
    c_j = sum_{k<=j} C(j,k)/C(deg,k) a_k."""
    M = np.zeros((deg + 1, deg + 1))
    for j in range(deg + 1):
        for k in range(j + 1):
            M[j, k] = comb(j, k) / comb(deg, k)
    return M


_M_CACHE = {}
def monomial_to_bseg(coeffs, durs):
    """coeffs: (n_seg, deg+1, 3) power-basis coeffs a_k for p(tau)=sum a_k tau^k, tau in [0, dur] (segment-local).
    durs: (n_seg,). Returns ctrl_pts (n_seg, deg+1, 3) Bernstein control points + t0s + durs.
    Reparametrise tau=s*dur (a_k -> a_k*dur^k) then change basis to Bernstein."""
    coeffs = np.asarray(coeffs, float); durs = np.asarray(durs, float)
    n_seg, npts, _ = coeffs.shape; deg = npts - 1
    M = _M_CACHE.setdefault(deg, _power_to_bernstein(deg))
    pw = np.array([[durs[i] ** k for k in range(npts)] for i in range(n_seg)])   # (n_seg, npts)
    ctrl = np.empty_like(coeffs)
    for i in range(n_seg):
        norm = coeffs[i] * pw[i][:, None]            # a_k * dur^k  -> (npts,3)
        ctrl[i] = M @ norm                            # Bernstein control points (npts,3)
    t0s = np.concatenate([[0.0], np.cumsum(durs)[:-1]])
    return ctrl, t0s, durs


def native_bezier_to_bseg(ctrl_pts, durs):
    """ctrl_pts already Bernstein/Bezier control points (n_seg, deg+1, 3) -> passthrough + cumulative t0s."""
    ctrl_pts = np.asarray(ctrl_pts, float); durs = np.asarray(durs, float)
    t0s = np.concatenate([[0.0], np.cumsum(durs)[:-1]])
    return ctrl_pts, t0s, durs


class Certifier:
    """Run the continuous-time Bernstein cert on arbitrary BSeg (ctrl_pts, t0s, durs)."""

    @staticmethod
    def certify_horizontal(ctrl_pts, t0s, durs, c0, R, vel=(0, 0, 0), acc=(0, 0, 0),
                           t_hi=-1.0, v_eff=0.0, delta=0.0, n_axes=2):
        ctrl_pts = np.asarray(ctrl_pts, float)
        n_seg, npts, _ = ctrl_pts.shape; deg = npts - 1
        _cp, cp = _p(ctrl_pts); _t, t0 = _p(t0s); _du, du = _p(durs)
        _c, cc = _p(c0); _v, vv = _p(vel); _a, aa = _p(acc)
        m = C.c_double(0.0)
        ok = _horiz(cp, n_seg, deg, t0, du, cc, vv, aa, float(R), float(t_hi),
                    float(v_eff), float(delta), int(n_axes), C.byref(m))
        return bool(ok), float(m.value)

    @staticmethod
    def certify_above(ctrl_pts, t0s, durs, z_clear, t_hi=-1.0, v_eff_z=0.0, delta=0.0, bez_pad=1e-9):
        ctrl_pts = np.asarray(ctrl_pts, float)
        n_seg, npts, _ = ctrl_pts.shape; deg = npts - 1
        _cp, cp = _p(ctrl_pts); _t, t0 = _p(t0s); _du, du = _p(durs)
        m = C.c_double(0.0)
        ok = _above(cp, n_seg, deg, t0, du, float(z_clear), float(t_hi),
                    float(v_eff_z), float(delta), float(bez_pad), C.byref(m))
        return bool(ok), float(m.value)


def _bezier_eval(ctrl, s):
    """de Casteljau eval of one Bezier segment (npts,3) at s in [0,1]."""
    pts = np.array(ctrl, float)
    while len(pts) > 1:
        pts = (1 - s) * pts[:-1] + s * pts[1:]
    return pts[0]


if __name__ == "__main__":
    # self-test: a cubic Bezier that grazes a static obstacle; cert verdict must match a dense brute-force min-dist.
    # straight-ish arc from (0,0,1.5) to (6,0,1.5), bulging to y=1.0 at the middle; obstacle at (3,0.4) R=0.8
    ctrl = np.array([[[0, 0, 1.5], [2, 1.5, 1.5], [4, 1.5, 1.5], [6, 0, 1.5]]], float)  # 1 segment, deg 3
    durs = np.array([2.0]); t0s = np.array([0.0])
    obs = [3.0, 0.4, 1.5]; R = 0.8
    okc, mg = Certifier.certify_horizontal(ctrl, t0s, durs, obs, R, t_hi=2.0, n_axes=2)
    # brute force min horizontal distance over the segment
    ds = [np.hypot(*( _bezier_eval(ctrl[0], s)[:2] - np.array(obs[:2]) )) for s in np.linspace(0, 1, 4000)]
    truth_clear = min(ds) >= R
    print(f"[cert] cubic-Bezier vs static obstacle: cert={okc} margin={mg:+.3f} | brute min_dist={min(ds):.3f} R={R} "
          f"-> truth_clear={truth_clear}")
    print("[cert] PASS (sound: cert => truth_clear)" if (not okc) or truth_clear else "[cert] FAIL: false-certify!")
    # monomial adapter check: same curve as a monomial cubic, convert -> should give a consistent verdict
    # p(s)= sum a_k s^k from the bezier (just sanity that monomial_to_bseg runs)
    coeffs = np.zeros((1, 4, 3)); coeffs[0, 0] = [0, 0, 1.5]; coeffs[0, 1] = [6, 0, 0]  # linear x ramp as a smoke
    cb, t0b, db = monomial_to_bseg(coeffs, durs)
    print(f"[cert] monomial_to_bseg ok: ctrl shape {cb.shape}, endpoints {np.round(cb[0,0],2)} -> {np.round(cb[0,-1],2)}")
