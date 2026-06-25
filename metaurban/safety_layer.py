"""safety_layer — the ONE shared certified-maneuver decision, used by BOTH the headless harness (replay_core) and
the 3-D renderer (render_3d_video). Previously each had its OWN copy with DRIFTED params (render: hand-set
MAN_QCONF/MAN_DSAFE/MAN_VEFF + climb-only escape; headless: calib.json q/v_eff + evade) -> the same EGO behaved
differently in render vs headless. This module makes the keep-out (conformal q, v_eff, d_safe), the cert gate, the
fastest-safe tournament (straight / around +-25/50 / over / climb) and the evade fallback a single source of truth.

The planner is duck-typed: it only needs the ego_bridge.EGOPlanner API both already use —
  replan(p,v,a,goal) -> truthy ; duration() -> float ; eval(t) -> (pos,vel,acc) ;
  certify_horizontal(obs_c0,R,obs_vel,obs_acc,t_hi,v_eff,delta) -> (ok,margin) ;
  certify_above(z_clear,t_hi,v_eff_z,delta) -> (ok,margin).
Cloud-building and execution (3-D static scene + quad dynamics in render vs mover cylinders + point-mass headless)
stay caller-specific -- that is the legitimate scenario/realism difference, not the avoidable code drift.
"""
import os, json
import numpy as np

# ---- canonical geometry / cert window (the SAME numbers headless replay_core used) ----
TAU = 0.75          # cert trust window (s)
R_DRONE = 0.25      # drone body radius (m)
D_SAFE_H = float(os.environ.get("REP_DSAFE", 0.10))    # horizontal comfort standoff (the conformal tube carries safety)
D_SAFE_V = float(os.environ.get("REP_DSAFEV", 0.30))   # vertical standoff for fly-over
REACH_PAD = 0.30    # pedestrian reach pad on the fly-over clearance height
PHI = np.radians(25.0)
CRUISE_Z = 1.5
Z_CEIL = 4.6
HORIZON = 7.5

_OUTDIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "out", "conformal")


def load_calib(eps=0.05):
    """class -> (q_conformal, v_eff) at this eps from out/conformal/calib.json (the SAME file both paths read).
    Falls back to a conservative hand value if missing."""
    path = os.path.join(_OUTDIR, "calib.json")
    out = {}
    if os.path.exists(path):
        rep = json.load(open(path)); g = rep.get("groups", {})
        allv = g.get("all", {}).get("levels", {}).get(str(eps))
        for cls in ("pedestrian", "vehicle", "animal"):
            lv = g.get(cls, {}).get("levels", {}).get(str(eps)) or allv
            if lv:
                out[cls] = (max(0.0, lv["q_conformal"]), lv["v_eff"])
        if allv:
            out["_all"] = (max(0.0, allv["q_conformal"]), allv["v_eff"])
    for cls in ("pedestrian", "vehicle", "animal"):
        out.setdefault(cls, (0.15, 0.6))
    out.setdefault("_all", (0.15, 0.6))
    return out


def build_cylinders(movers, calib, predict=True):
    """movers: list of (c0(3,), vel(3,), acc(3,), r_obs, head_height, cls) -- vel/acc are the caller's predictor
    output (CV: acc already 0; CA: the KF acceleration). Returns (cyls, ztop) where each cyl is
    (c0, vel, acc, R, z_clear, v_eff) -- the conformal per-class keep-out the cert is run against."""
    cyl = []; ztop = CRUISE_Z
    for (c0, vel, acc, r_obs, h, cls) in movers:
        q, veff = calib.get(cls, calib["_all"])
        vv = np.asarray(vel, float).copy(); aa = np.asarray(acc, float).copy()
        if not predict:
            vv = np.zeros(3); aa = np.zeros(3)
        R = float(r_obs) + R_DRONE + D_SAFE_H + q
        zc = float(h) + REACH_PAD + R_DRONE + D_SAFE_V + q
        cyl.append((np.asarray(c0, float), vv, aa, R, zc, veff)); ztop = max(ztop, zc + 0.2)
    return cyl, min(Z_CEIL, ztop)


def cert_clear(ego, cyl, tau=TAU, delta=None):
    """The cylinder disjunction on ego's CURRENTLY-committed B-spline: per mover, (horiz-predicted AND
    horiz-current) OR above. AND across movers."""
    d = tau if delta is None else delta
    for (c0, vv, aa, R, zc, veff) in cyl:
        hp, _ = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=vv, obs_acc=aa, t_hi=tau, v_eff=veff, delta=d)
        hc, _ = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=(0, 0, 0), t_hi=tau, v_eff=veff, delta=d)
        vo, _ = ego.certify_above(z_clear=zc, t_hi=tau, v_eff_z=0.0, delta=d)
        if not ((hp and hc) or vo):
            return False
    return True


def _rot(v2, ang):
    c, s = np.cos(ang), np.sin(ang)
    return np.array([c * v2[0] - s * v2[1], s * v2[0] + c * v2[1]])


def maneuver_decide(ego, p_d, v_d, a_d, goal, ztop, clear_fn, cruise_z=CRUISE_Z, horizon=HORIZON,
                    straight_clip=None):
    """Run the fastest-safe tournament and LEAVE ego holding the chosen B-spline. `clear_fn()` -> bool gates each
    committed candidate (normally cert_clear(ego, cyl); the discrete-sampling ablation passes its own). Returns the
    kind 'straight'|'around_l'|'around_r'|'over'|'climb'|'evade'. 'evade' = nothing certified -> caller flees via
    evade_setpoint(); any other kind = ego holds a certified (or, for 'climb', the no-freeze) plan.
    straight_clip: if set (the renderer's EGO_HOR), the STRAIGHT goal is clipped to a receding horizon too -- a 75 m
    raw goal makes EGO extrapolate past the perceived region and fail; the headless corridor goal is short so it
    leaves it None (replan straight to the true goal)."""
    p_d = np.asarray(p_d, float); goal = np.asarray(goal, float)
    gxy = goal[:2] - p_d[:2]; dist = float(np.linalg.norm(gxy))
    gdir = gxy / dist if dist > 1e-6 else np.array([1.0, 0.0])
    L = min(horizon, max(dist, 1.0))
    straight_goal = (np.array([goal[0], goal[1], cruise_z]) if straight_clip is None
                     else np.array([*(p_d[:2] + gdir * min(straight_clip, dist)), cruise_z]))

    for gk, gsub in (("straight", straight_goal),
                     ("around_l", np.array([*(p_d[:2] + L * _rot(gdir, PHI)), cruise_z])),
                     ("around_r", np.array([*(p_d[:2] + L * _rot(gdir, -PHI)), cruise_z])),
                     ("around_l", np.array([*(p_d[:2] + L * _rot(gdir, 2 * PHI)), cruise_z])),
                     ("around_r", np.array([*(p_d[:2] + L * _rot(gdir, -2 * PHI)), cruise_z]))):
        if ego.replan(p_d, v_d, a_d, gsub) and ego.duration() > 1e-3 and clear_fn():
            return gk
    # ground blocked -> fly OVER (certified); boxed -> climb straight up (no-freeze, uncertified)
    if ego.replan(p_d, v_d, a_d, np.array([goal[0], goal[1], ztop])) and ego.duration() > 1e-3 and clear_fn():
        return "over"
    if ego.replan(p_d, v_d, a_d, np.array([p_d[0], p_d[1], ztop])) and ego.duration() > 1e-3:
        return "climb"
    return "evade"


def evade_setpoint(p_d, movers_xy, max_vel, dt, ztop, gdir):
    """Nothing certified + movers present -> FLEE the nearest mover (never FREEZE into a collision). Returns the
    (pos, vel) set-point the caller should fly this tick."""
    p_d = np.asarray(p_d, float)
    if len(movers_xy):
        nn = min(movers_xy, key=lambda c: np.linalg.norm(np.asarray(c)[:2] - p_d[:2]))
        away = p_d[:2] - np.asarray(nn)[:2]; n = float(np.linalg.norm(away))
        away = away / n if n > 1e-6 else np.asarray(gdir, float)
    else:
        away = np.asarray(gdir, float)
    pos = p_d + np.array([away[0] * 0.6 * max_vel * dt, away[1] * 0.6 * max_vel * dt,
                          min(0.4 * max_vel * dt, max(0.0, ztop - p_d[2]))])
    vel = np.array([away[0] * max_vel, away[1] * max_vel, 0.0])
    return pos, vel
