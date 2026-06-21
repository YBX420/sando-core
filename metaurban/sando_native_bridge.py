"""sando_native_bridge — ctypes wrapper for the de-ROS'd NATIVE MIT-ACL SANDO planner
(sando_native/capi/sando_native_capi.so). This is the ORIGINAL github.com/mit-acl/sando baseline
(heat-A* global + DecompUtil safe-flight-corridors + GUROBI local solver), NOT our MINCO.

Mirrors sando_cpp_bridge / ego_bridge. Params load from native SANDO's own config/sando.yaml.
Requires GUROBI (WLS license at ~/gurobi.lic; LD_LIBRARY_PATH must include ~/gurobi1103/linux64/lib).
"""
import os, ctypes as C
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SO = os.path.join(os.path.dirname(_HERE), "sando_native", "capi", "sando_native_capi.so")
_YAML = os.path.join(os.path.dirname(_HERE), "sando_native", "sando.yaml")
_lib = C.CDLL(_SO)
_d = C.POINTER(C.c_double)
_i = C.POINTER(C.c_int)


def _sig(name, restype, *args):
    f = getattr(_lib, name); f.restype = restype; f.argtypes = list(args); return f


_params_from_yaml = _sig("sn_params_from_yaml", C.c_void_p, C.c_char_p)
_params_set_double = _sig("sn_params_set_double", None, C.c_void_p, C.c_char_p, C.c_double)
_params_destroy = _sig("sn_params_destroy", None, C.c_void_p)
_create = _sig("sn_create", C.c_void_p, C.c_void_p)
_destroy = _sig("sn_destroy", None, C.c_void_p)
_update_state = _sig("sn_update_state", None, C.c_void_p, _d, _d, _d, C.c_double)
_set_goal = _sig("sn_set_terminal_goal", None, C.c_void_p, _d)
_add_traj = _sig("sn_add_traj", None, C.c_void_p, C.c_int, C.c_double, C.c_double, C.c_double,
                 C.c_char_p, C.c_char_p, C.c_char_p, C.c_char_p, C.c_char_p, C.c_char_p, C.c_int, C.c_double)
_clean_trajs = _sig("sn_clean_old_trajs", None, C.c_void_p, C.c_double)
_update_occ = _sig("sn_update_occupancy", None, C.c_void_p, _d, C.c_int, C.c_double)
_replan = _sig("sn_replan", C.c_int, C.c_void_p, C.c_double, C.c_double)
_get_next_goal = _sig("sn_get_next_goal", C.c_int, C.c_void_p, _d)
_get_status = _sig("sn_get_drone_status", C.c_int, C.c_void_p)
_get_global_path = _sig("sn_get_global_path", C.c_int, C.c_void_p, _d, C.c_int)
_get_setpoints = _sig("sn_get_setpoints", C.c_int, C.c_void_p, _d, C.c_int)


def _p(a):
    a = np.ascontiguousarray(np.asarray(a, np.float64).reshape(-1)); return a, a.ctypes.data_as(_d)


def _b(s):
    return s.encode() if isinstance(s, str) else (s or b"")


class SandoNative:
    def __init__(self, yaml_path=None, overrides=None):
        yp = (yaml_path or _YAML).encode()
        self._par = _params_from_yaml(yp)
        if not self._par:
            raise RuntimeError(f"native SANDO: failed to load params from {yaml_path or _YAML}")
        for k, v in (overrides or {}).items():
            _params_set_double(self._par, k.encode(), float(v))
        self._h = _create(self._par)
        if not self._h:
            raise RuntimeError("native SANDO: sn_create failed (GUROBI/license? see stderr)")

    def update_state(self, pos, vel=(0, 0, 0), acc=(0, 0, 0), yaw=0.0):
        _a, pp = _p(pos); _b1, vp = _p(vel); _c, ap = _p(acc)
        _update_state(self._h, pp, vp, ap, float(yaw))

    def set_terminal_goal(self, pos):
        _a, pp = _p(pos); _set_goal(self._h, pp)

    def add_traj(self, tid, bbox, tx, ty, tz, vx="", vy="", vz="", is_agent=False, t=0.0):
        _add_traj(self._h, int(tid), float(bbox[0]), float(bbox[1]), float(bbox[2]),
                  _b(tx), _b(ty), _b(tz), _b(vx), _b(vy), _b(vz), 1 if is_agent else 0, float(t))

    def clean_old_trajs(self, t):
        _clean_trajs(self._h, float(t))

    def update_occupancy(self, cloud_xyz, t):
        cloud = np.ascontiguousarray(np.asarray(cloud_xyz, np.float64).reshape(-1, 3))
        n = cloud.shape[0]; cp = cloud.ctypes.data_as(_d) if n else _d()
        _update_occ(self._h, cp, int(n), float(t))

    def replan(self, last_rt, t):
        r = _replan(self._h, float(last_rt), float(t))
        return bool(r & 1), bool(r & 2)   # (success, attempted)

    def get_next_goal(self):
        out = (C.c_double * 9)()
        if not _get_next_goal(self._h, C.cast(out, _d)):
            return False, None
        a = np.array(out, float)
        return True, (a[0:3], a[3:6], a[6:9])   # (pos, vel, accel)

    def get_drone_status(self):
        return int(_get_status(self._h))

    def get_global_path(self, max_pts=4000):
        out = (C.c_double * (3 * max_pts))()
        n = _get_global_path(self._h, C.cast(out, _d), max_pts)
        return np.array(out[:3 * n], float).reshape(-1, 3) if n else np.zeros((0, 3))

    def get_setpoints(self, max_pts=2000):
        out = (C.c_double * (3 * max_pts))()
        n = _get_setpoints(self._h, C.cast(out, _d), max_pts)
        return np.array(out[:3 * n], float).reshape(-1, 3) if n else np.zeros((0, 3))

    def __del__(self):
        try:
            _destroy(self._h); _params_destroy(self._par)
        except Exception:
            pass


if __name__ == "__main__":
    print("[sn] native SANDO self-test: load .so, params, create, one replan toward a goal")
    sn = SandoNative(overrides={"v_max": 4.0, "x_min": -50, "x_max": 50, "y_min": -50, "y_max": 50})
    print("[sn] created OK (GUROBI license valid)")
    sn.update_state([0, 0, 1.5], [0, 0, 0], [0, 0, 0], 0.0)
    sn.set_terminal_goal([10, 0, 1.5])
    wall = np.array([[5.0, y, z] for y in np.arange(-2, 2, 0.2) for z in np.arange(0.5, 3, 0.2)])
    sn.update_occupancy(wall, 0.0)
    ok, att = sn.replan(0.0, 0.0)
    print(f"[sn] replan success={ok} attempted={att}  status={sn.get_drone_status()}")
    gp = sn.get_global_path(); sp = sn.get_setpoints()
    print(f"[sn] global_path pts={len(gp)}  committed setpoints={len(sp)}")
    if len(gp): print(f"[sn] gp[0]={np.round(gp[0],2)} gp[-1]={np.round(gp[-1],2)}")
    okn, ng = sn.get_next_goal()
    print(f"[sn] next_goal ok={okn}" + (f" pos={np.round(ng[0],2)}" if okn else ""))
    print("[sn] PASS" if ok or att else "[sn] replan did not succeed (check params/obstacles)")
