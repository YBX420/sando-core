"""ego_bridge — ctypes wrapper for the standalone (de-ROS'd) EGO-Planner core (ego/capi/ego_capi.so).

Mirrors sando_cpp_bridge. Feed a gridmap config + the drone's depth-FOV point cloud + start/goal, get back
a B-spline trajectory you sample with traj_eval(t) -> (pos, vel, acc). This is the EGO-Planner alternative
to the SANDO core, for the MetaUrban loop (ESDF-free, point-cloud/depth based — matches the FOV-limited perception).
"""
import os, ctypes as C
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SO = os.path.join(os.path.dirname(_HERE), "ego", "capi", "ego_capi.so")
_lib = C.CDLL(_SO)
_d = C.POINTER(C.c_double)


def _sig(name, restype, *args):
    f = getattr(_lib, name); f.restype = restype; f.argtypes = list(args); return f


_create = _sig("ego_create", C.c_void_p)
_set_gridmap = _sig("ego_set_gridmap", None, C.c_void_p, _d, _d, C.c_double, C.c_double)
_set_params = _sig("ego_set_params", None, C.c_void_p, C.c_double, C.c_double, C.c_double, C.c_double,
                   C.c_double, C.c_double, C.c_double, C.c_double, C.c_double, C.c_double, C.c_int)
_update_cloud = _sig("ego_update_cloud", None, C.c_void_p, _d, C.c_int, _d)
_replan = _sig("ego_replan", C.c_int, C.c_void_p, _d, _d, _d, _d, _d, C.c_int, C.c_int)
_duration = _sig("ego_traj_duration", C.c_double, C.c_void_p)
_eval = _sig("ego_traj_eval", C.c_int, C.c_void_p, C.c_double, _d)
_certify = _sig("ego_certify", C.c_int, C.c_void_p, _d, _d, _d, C.c_double, C.c_double,
                C.c_double, C.c_double, _d)
_certify_horiz = _sig("ego_certify_horizontal", C.c_int, C.c_void_p, _d, _d, _d, C.c_double, C.c_double,
                      C.c_double, C.c_double, _d)
_certify_above = _sig("ego_certify_above", C.c_int, C.c_void_p, C.c_double, C.c_double, C.c_double,
                      C.c_double, C.c_double, _d)
_destroy = _sig("ego_destroy", None, C.c_void_p)


def _p(a):
    a = np.ascontiguousarray(np.asarray(a, np.float64).reshape(-1)); return a, a.ctypes.data_as(_d)


class EGOPlanner:
    def __init__(self, map_origin=(-60, -60, -1), map_size=(120, 120, 8), res=0.15, inflation=0.3):
        self._h = _create()
        self.set_gridmap(map_origin, map_size, res, inflation)
        self._poly_init = True            # first call uses polynomial init; then warm-starts

    def set_gridmap(self, origin, size, res, inflation):
        _o, op = _p(origin); _s, sp = _p(size); _set_gridmap(self._h, op, sp, float(res), float(inflation))

    def set_params(self, max_vel=3.0, max_acc=6.0, max_jerk=4.0, ctrl_pt_dist=0.5, horizon=7.5,
                   l_smooth=1.0, l_collision=0.5, l_feasibility=0.1, l_fitness=1.0, dist0=0.5, order=3):
        _set_params(self._h, max_vel, max_acc, max_jerk, ctrl_pt_dist, horizon,
                    l_smooth, l_collision, l_feasibility, l_fitness, dist0, int(order))

    def update_cloud(self, cloud_xyz, cam_pos):
        cloud = np.ascontiguousarray(np.asarray(cloud_xyz, np.float64).reshape(-1, 3))
        n = cloud.shape[0]; cp = cloud.ctypes.data_as(_d) if n else _d()
        _c, cam = _p(cam_pos); _update_cloud(self._h, cp, int(n), cam)

    def replan(self, start, vel, acc, goal, goal_vel=(0, 0, 0), random_poly=False):
        _a, sp = _p(start); _b, sv = _p(vel); _c, sa = _p(acc); _d2, gp = _p(goal); _e, gv = _p(goal_vel)
        ok = _replan(self._h, sp, sv, sa, gp, gv, 1 if self._poly_init else 1, 1 if random_poly else 0)
        return bool(ok)

    def duration(self): return float(_duration(self._h))

    def eval(self, t):
        out = (C.c_double * 9)()
        if not _eval(self._h, float(t), C.cast(out, _d)): return None
        a = np.array(out, float); return a[0:3], a[3:6], a[6:9]

    def certify(self, obs_c0, R, obs_vel=(0, 0, 0), obs_acc=(0, 0, 0), t_hi=-1.0, v_eff=0.0, delta=0.0):
        """OUR S3 safety envelope ON EGO (certified safety layer): certify EGO's committed B-spline clears
        a sphere obstacle (centre obs_c0 + obs_vel*t + 0.5*obs_acc*t^2) for ALL continuous t up to t_hi
        (<=0 => whole trajectory). The tube radius GROWS as rho(t) = R + v_eff*(t + delta):
          v_eff=0  -> trust the prediction over [0,t_hi] exactly (tightest / fastest / riskiest);
          v_eff>0  -> inflate to cover reachable / conformal prediction drift (safer / slower).
        delta = perception->commit latency. Returns (certified: bool, margin: float)."""
        _a, c0 = _p(obs_c0); _b, vv = _p(obs_vel); _c, aa = _p(obs_acc)
        margin = C.c_double(0.0)
        ok = _certify(self._h, c0, vv, aa, float(R), float(t_hi), float(v_eff), float(delta), C.byref(margin))
        return bool(ok), float(margin.value)

    def certify_horizontal(self, obs_c0, R, obs_vel=(0, 0, 0), obs_acc=(0, 0, 0), t_hi=-1.0, v_eff=0.0, delta=0.0):
        """AROUND half of the CYLINDER disjunction: certify EGO's committed B-spline keeps 2-D HORIZONTAL
        separation sqrt(dx^2+dy^2) >= rho(t)=R+v_eff*(t+delta) from the moving cylinder axis (obs_c0+vel*t+
        0.5*acc*t^2), z ignored. SOUND for a full-height cylinder where the 3-D sphere cert is not. (bool, margin)."""
        _a, c0 = _p(obs_c0); _b, vv = _p(obs_vel); _c, aa = _p(obs_acc)
        margin = C.c_double(0.0)
        ok = _certify_horiz(self._h, c0, vv, aa, float(R), float(t_hi), float(v_eff), float(delta), C.byref(margin))
        return bool(ok), float(margin.value)

    def certify_above(self, z_clear, t_hi=-1.0, v_eff_z=0.0, delta=0.0, bez_pad=1e-9):
        """OVER half of the CYLINDER disjunction: certify EGO's committed B-spline stays vertically above the
        cylinder, p_z(t) >= z_clear for ALL t up to t_hi. z_clear = head_top + reach_pad + r_body + d_safe_v.
        v_eff_z grows the floor; bez_pad outward-bounds the B-spline->Bezier rounding. Returns (bool, margin)."""
        margin = C.c_double(0.0)
        ok = _certify_above(self._h, float(z_clear), float(t_hi), float(v_eff_z), float(delta),
                            float(bez_pad), C.byref(margin))
        return bool(ok), float(margin.value)

    def __del__(self):
        try: _destroy(self._h)
        except Exception: pass


if __name__ == "__main__":
    print("[ego] standalone self-test: plan around an obstacle wall")
    pl = EGOPlanner(map_origin=(-5, -10, -1), map_size=(30, 20, 5), res=0.15, inflation=0.3)
    pl.set_params(max_vel=3.0, max_acc=6.0)
    # an obstacle wall at x=5, spanning y in [-1.5,1.5] at z=1.5, as a depth-FOV point cloud
    wall = [[5.0, y, z] for y in np.arange(-1.5, 1.5, 0.1) for z in np.arange(0.5, 2.5, 0.1)]
    pl.update_cloud(wall, cam_pos=[0, 0, 1.5])
    ok = pl.replan(start=[0, 0, 1.5], vel=[0, 0, 0], acc=[0, 0, 0], goal=[10, 0, 1.5])
    print(f"[ego] replan ok={ok}  duration={pl.duration():.2f}s")
    if ok and pl.duration() > 0:
        dur = pl.duration(); maxlat = 0.0; minx_at_wall = None
        for t in np.linspace(0, dur, 25):
            r = pl.eval(t)
            if r is None: continue
            p = r[0]; maxlat = max(maxlat, abs(p[1]))
            if abs(p[0] - 5.0) < 0.3: minx_at_wall = p[1]
        print(f"[ego] traj lateral swerve max|y|={maxlat:.2f}m ; y at x=5 (the wall) = "
              f"{minx_at_wall if minx_at_wall is None else round(minx_at_wall,2)}")
        print("[ego] PASS: produced a collision-avoiding B-spline" if maxlat > 0.3 else
              "[ego] traj is straight (check obstacle feeding/params)")
    else:
        print("[ego] FAIL: no trajectory")
