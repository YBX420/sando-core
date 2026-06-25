"""gcopter_bridge — ctypes binding to the de-ROS'd ZJU GCOPTER (gcopter/capi/gcopter_capi.so).

gcopter_plan(cloud, start, goal, ...) -> (coeffs (n_piece,3,6) DESCENDING-power MINCO, durs (n_piece,)) or None.
The coeffs feed cert_bridge.minco_descending_to_bseg directly, so our continuous-time Bernstein cert certifies
GCOPTER's committed trajectory unchanged. The point cloud is the obstacle occupancy (same as we feed EGO).
"""
import os, ctypes as C
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SO = os.path.join(os.path.dirname(_HERE), "gcopter", "capi", "gcopter_capi.so")
_lib = C.CDLL(_SO)
_d = C.POINTER(C.c_double)
_plan = _lib.gcopter_plan
_plan.restype = C.c_int
_plan.argtypes = [_d, C.c_int, _d, _d, _d, C.c_double, C.c_double, C.c_double, C.c_double, C.c_int, _d, _d]


def _p(a):
    a = np.ascontiguousarray(np.asarray(a, np.float64).reshape(-1))
    return a, a.ctypes.data_as(_d)


def gcopter_plan(cloud, start, goal, mapbound, voxel_width=0.25, dilate=0.5,
                 vmax=4.0, weight_t=20.0, max_pieces=64):
    """cloud: (n,3) obstacle points. start/goal: (3,). mapbound: [xlo,xhi,ylo,yhi,zlo,zhi]. Returns
    (coeffs (np,3,6) descending-power, durs (np,)) or None on failure."""
    cloud = np.ascontiguousarray(np.asarray(cloud, np.float64).reshape(-1, 3)) if len(np.asarray(cloud).reshape(-1)) else np.zeros((0, 3))
    n = int(cloud.shape[0])
    cl, clp = _p(cloud if n else np.zeros(3))
    _s, sp = _p(start); _g, gp = _p(goal); _mb, mbp = _p(mapbound)
    out_coeffs = np.zeros(max_pieces * 18, np.float64); _oc, ocp = out_coeffs, out_coeffs.ctypes.data_as(_d)
    out_durs = np.zeros(max_pieces, np.float64); _od, odp = out_durs, out_durs.ctypes.data_as(_d)
    npx = _plan(clp, n, sp, gp, mbp, float(voxel_width), float(dilate), float(vmax), float(weight_t),
                int(max_pieces), ocp, odp)
    if npx <= 0:
        return None
    coeffs = out_coeffs[:npx * 18].reshape(npx, 3, 6).copy()
    durs = out_durs[:npx].copy()
    return coeffs, durs


if __name__ == "__main__":
    # smoke: a wall of points at x=3 (gap-free across y in [-1,1]); fly (0,0,1.5)->(6,0,1.5); GCOPTER must route
    import sys
    sys.path.insert(0, _HERE)
    from cert_bridge import minco_descending_to_bseg, _bezier_eval
    wall = np.array([[3.0, y, z] for y in np.linspace(-1.0, 1.0, 21) for z in np.linspace(0.3, 2.5, 12)])
    r = gcopter_plan(wall, [0, 0, 1.5], [6, 0, 1.5], [-10, 10, -10, 10, 0, 5], vmax=4.0)
    if r is None:
        print("[gcopter] plan FAILED"); sys.exit(1)
    coeffs, durs = r
    print(f"[gcopter] OK: {len(durs)} pieces, total T={durs.sum():.2f}s, durs={np.round(durs,2)}")
    # eval the trajectory at a few times via the descending-power coeffs (col0 = t^5 ... col5 = t^0)
    def evalpos(coeffs, durs, t):
        i = 0
        while i < len(durs) - 1 and t > durs[i]:
            t -= durs[i]; i += 1
        C6 = coeffs[i]; powers = np.array([t**5, t**4, t**3, t**2, t, 1.0])
        return C6 @ powers
    T = durs.sum()
    pts = np.array([evalpos(coeffs, durs, s) for s in np.linspace(0, T - 1e-6, 30)])
    print(f"[gcopter]   start={np.round(pts[0],2)} mid={np.round(pts[15],2)} end={np.round(pts[-1],2)}")
    # min distance to the wall line x=3 over the path, and lateral detour
    miny = pts[:, 1].min(); maxy = pts[:, 1].max()
    near_wall = pts[np.abs(pts[:, 0] - 3.0) < 0.25]
    clr = np.abs(near_wall[:, 1]).min() if len(near_wall) else np.nan
    print(f"[gcopter]   lateral span y=[{miny:.2f},{maxy:.2f}]; |y| at x=3 wall crossing = {clr:.2f} "
          f"(routes around the y in [-1,1] wall) {'OK' if (np.isnan(clr) or clr > 0.9) else 'GRAZE'}")
    # cert adapter sanity: convert to BSeg
    cb, t0, du = minco_descending_to_bseg(list(coeffs), list(durs))
    print(f"[gcopter]   cert adapter: BSeg {cb.shape} -> feeds the continuous-time Bernstein cert")
