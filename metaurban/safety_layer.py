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
import math
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
    # MAPPED STATICS (online-mapping perception): position uncertainty only, NO growing tube --
    # a remembered tree does not move; giving it the mover v_eff seals every corridor it borders.
    out.setdefault("static", (0.15, 0.0))
    return out


def build_cylinders(movers, calib, predict=True, track_margin=0.0, calib_v2=None, age_min=4):
    """movers: list of (c0(3,), vel(3,), acc(3,), r_obs, head_height, cls) -- vel/acc are the caller's predictor
    output (CV: acc already 0; CA: the KF acceleration). Returns (cyls, ztop) where each cyl is
    (c0, vel, acc, R, z_clear, v_eff) -- the conformal per-class keep-out the cert is run against.
    track_margin: plan->flown tracking allowance (audit #9-2 2026-07-07: the renderer certs carry
    MAN_TRACK=0.473 but the headless DYN arms certified the PLANNED spline with NO margin while the
    quad FLIES up to ~delta_track away -- 'flown == certified' hole). Kinematic arms pass 0."""
    use_plates = os.environ.get("PLATES", "0") == "1"
    use_ellipse = os.environ.get("ELLIPSE", "0") == "1"   # v5 motion-frame ELLIPTICAL keep-out
    #   (mature moving tracks only; needs calib_v5 entries carrying 'kappa' -- see load_calib_v5)
    use_capsule = os.environ.get("CAPSULE", "0") == "1"   # v6 CAPSULE keep-out (segment + pearls;
    #   needs calib_v6 entries carrying capsule=True -- see load_calib_v6)
    assert not (use_ellipse and use_capsule), "ELLIPSE=1 and CAPSULE=1 are mutually exclusive"
    agemin_ped = int(os.environ.get("AGEMIN_PED", "0")) or age_min   # ped early maturity (age2-3
    #   q95 covered by the mature tube at every horizon, designC 2026-07-07) -- gated exploratory
    cyl = []; ztop = CRUISE_Z
    for mv in movers:
        (c0, vel, acc, r_obs, h, cls) = mv[:6]
        age = mv[6] if len(mv) > 6 else None
        coast = mv[7] if len(mv) > 7 else None
        nis = mv[8] if len(mv) > 8 else None
        sigv = mv[9] if len(mv) > 9 else None
        _mature_arm = False
        if calib_v2 is not None:
            ent = calib_v2.get(cls) or dict(mature=(1e6, 0.0), young=(1e6, 0.0), plates=[])
            _amin = agemin_ped if cls == "pedestrian" else age_min
            _nis_th = float(os.environ.get("NIS_GATE", "0"))
            _zombie = (_nis_th > 0.0 and nis is not None and nis > _nis_th)
            _sv_th = float(os.environ.get("SIGV_GATE", "0"))
            if _sv_th > 0.0 and sigv is not None and sigv > _sv_th:
                _zombie = True     # P-based demotion: NIS can't see a self-aware zombie (coast inflates
                #   S so NIS stays small); the velocity covariance is the honest convergence signal
            # NIS zombie filter: a track whose filter self-check (chi2_1 NIS EWMA) is inconsistent
            # carries a garbage velocity (mis-association / model mismatch) -- certifying its MOVING
            # prediction poisons corridors worse than the frozen young plate. Demote (fatter=sound).
            if age is not None and (age < _amin or _zombie):
                q, veff = ent["young"]
                vel = np.zeros(3); acc = np.zeros(3)   # young plate: FROZEN centre + fat growth
            else:
                q, veff = ent["mature"]
                _mature_arm = True
                if use_plates and age is not None:
                    for (lo, hi, co, pq, pv) in ent.get("plates", []):
                        if lo <= age <= hi and (co is None or coast is None or int(co) == int(coast)):
                            q, veff = pq, pv
                            break
        else:
            q, veff = calib.get(cls, calib["_all"])
        vv = np.asarray(vel, float).copy(); aa = np.asarray(acc, float).copy()
        if not predict:
            vv = np.zeros(3); aa = np.zeros(3)
        if os.environ.get("CALIB_V2", "0") == "1" and str(cls) == "static":
            vv = np.zeros(3); aa = np.zeros(3)     # FS3C-R #13: statics are STATIONARY at code level
            #   (KF v on a static is measurement noise; calib static law is v=0 -- must match)
        R = float(r_obs) + R_DRONE + D_SAFE_H + q + track_margin
        zc = float(h) + REACH_PAD + R_DRONE + D_SAFE_V + q + track_margin
        ent7 = ()
        if (use_capsule and predict and _mature_arm and calib_v2 is not None
                and str(cls) in ("pedestrian", "vehicle")
                and (calib_v2.get(cls) or {}).get("capsule")):
            # CAPSULE keep-out (v6): segment [mover's current position, KF tip c0+v*t] ⊕ q̃ --
            # the along-axis endpoints are the mover's BACK and the prediction's APEX (no wall
            # behind a walker). Certified as a K-pearl cover: obs_vel = s*v per pearl, the pearl
            # gap |v|*t/(2(K-1)) folded into v_eff. q̃/veff here are dist-to-segment calibrated.
            _e6 = calib_v2.get(cls) or {}
            _K = int(_e6.get("n_pearls", 4))
            _sp = float(np.hypot(vv[0], vv[1]))
            veff = veff + _sp / (2.0 * max(_K - 1, 1))
            _rr = _e6.get("rear")
            if _rr is not None and _sp > 1e-6:
                # v6.1 REAR-PLANE disjunct: certifying 'the drone stays behind the mover's rear
                # plane the whole window' is an alternative pass -- the wake needs only the
                # (constant) rear-overrun quantile + body/standoff, not the growing q̃ wall.
                _th0 = float(_rr[0]) + float(r_obs) + R_DRONE + D_SAFE_H + track_margin
                ent7 = (("cap", _K, float(vv[0]) / _sp, float(vv[1]) / _sp, _th0, float(_rr[1])),)
            else:
                ent7 = (("cap", _K),)
        if (use_ellipse and predict and _mature_arm and calib_v2 is not None
                and str(cls) in ("pedestrian", "vehicle")):
            # ELLIPTICAL keep-out (v4): trust the KF direction only when the track is mature,
            # non-zombie AND actually moving (calibration mirrors this exact predicate on the
            # harvested KF anchor speed). Cross-track semi-axis = along/kappa; the drone-side
            # geometry (body+standoff+tracking) must survive the cross-axis stretch, hence
            # R_warp = kappa*r_geom + q (see certify_horizontal_aniso).
            _e = calib_v2.get(cls) or {}
            _kap = float(_e.get("kappa", 1.0))
            _sp = float(np.hypot(vv[0], vv[1]))
            if _kap > 1.0 + 1e-9 and _sp >= float(_e.get("v_min_dir", 0.5)):
                _rw = _kap * (float(r_obs) + R_DRONE + D_SAFE_H + track_margin) + q
                ent7 = ((_kap, float(vv[0]) / _sp, float(vv[1]) / _sp, _rw),)
        cyl.append((np.asarray(c0, float), vv, aa, R, zc, veff) + ent7); ztop = max(ztop, zc + 0.2)
    return cyl, min(Z_CEIL, ztop)


def _ell_of(ent, ego):
    """7th optional cyl field = (kappa, ux, uy, R_warp) from build_cylinders under ELLIPSE=1.
    None when absent OR the planner lacks the aniso entry point (isotropic R is a superset of the
    ellipse at the same calibrated q, so falling back is sound, just fatter)."""
    if (len(ent) > 6 and ent[6] is not None and not isinstance(ent[6][0], str)
            and getattr(ego, "certify_horizontal_aniso", None) is not None):
        return ent[6]
    return None


def _cap_of(ent):
    """7th field == ("cap", K): v6 capsule pearl count, else None. NB there is NO sound isotropic
    fallback here -- q̃ is a dist-to-segment quantity; a mover tagged cap MUST be pearl-certified."""
    if len(ent) > 6 and ent[6] is not None and isinstance(ent[6][0], str) and ent[6][0] == "cap":
        return int(ent[6][1])
    return None


def _cap_grid(K):
    return [k / (K - 1.0) for k in range(K)] if K > 1 else [1.0]


def _cap_behind(ent, ego, tau, d, warp=1.0):
    """v6.1 rear-plane disjunct: True if the committed spline stays BEHIND the mover's rear plane
    over the whole (possibly retimed) window. False when the tag carries no direction (slow mover)
    or the planner lacks the entry point (pearls-only = the sound v6.0 behaviour)."""
    tag = ent[6]
    if len(tag) < 6 or getattr(ego, "certify_behind", None) is None:
        return False
    _, _K, ux, uy, th0, rate = tag
    ok, _ = ego.certify_behind(obs_c0=ent[0], ux=ux, uy=uy, thresh0=th0, rate=rate / warp,
                               t_hi=warp * tau, delta=d * warp)
    return ok


def cert_clear(ego, cyl, tau=TAU, delta=None):
    """The cylinder disjunction on ego's CURRENTLY-committed B-spline: per mover, (horiz-predicted AND
    horiz-current) OR above. AND across movers. A mover carrying the v4 ellipse field is judged in the
    whitened motion frame (cross-track semi-axis = along/kappa) -- same disjunction shape."""
    d = tau if delta is None else delta
    for ent in cyl:
        (c0, vv, aa, R, zc, veff) = ent[:6]
        cap = _cap_of(ent)
        if cap is not None:
            # v6 capsule: pearl-string cover of segment [c0, c0+v*t] ⊕ q̃ -- s=0 IS the old
            # frozen-current conjunct, s=1 the predicted tube, at the collapsed radius.
            hb = True
            for s in _cap_grid(cap):
                hs, _ = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=tuple(np.asarray(vv, float) * s),
                                               obs_acc=(0, 0, 0), t_hi=tau, v_eff=veff, delta=d)
                if not hs:
                    hb = False
                    break
            if not hb:
                hb = _cap_behind(ent, ego, tau, d)   # v6.1: flying in the WAKE is an alternative pass
            hp = hc = hb
        else:
            ell = _ell_of(ent, ego)
            if ell is not None:
                _k, _ux, _uy, _rw = ell
                hp, _ = ego.certify_horizontal_aniso(obs_c0=c0, R=_rw, obs_vel=vv, obs_acc=aa, t_hi=tau,
                                                     v_eff=veff, delta=d, ux=_ux, uy=_uy, kappa=_k)
                hc, _ = ego.certify_horizontal_aniso(obs_c0=c0, R=_rw, obs_vel=(0, 0, 0), t_hi=tau,
                                                     v_eff=veff, delta=d, ux=_ux, uy=_uy, kappa=_k)
            else:
                hp, _ = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=vv, obs_acc=aa, t_hi=tau, v_eff=veff, delta=d)
                hc, _ = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=(0, 0, 0), t_hi=tau, v_eff=veff, delta=d)
        vo, _ = ego.certify_above(z_clear=zc, t_hi=tau, v_eff_z=0.0, delta=d)
        if not ((hp and hc) or vo):
            return False
    return True


def cert_verdict3(ego, cyl, tau=TAU, delta=None):
    """DIAGNOSTIC three-valued twin of cert_clear (does NOT drive the decision -- cert_clear stays the gate).
    Returns (overall, details): overall in 'certified'|'refuted'|'unknown'; details = one dict per mover with
    the three sub-verdicts. Three-valued logic: AND = refuted if any refuted else certified if all certified
    else unknown; OR = certified if any certified else refuted if all refuted else unknown; movers are ANDed.
    'refuted' = the keep-out is PROVABLY violated (truly blocked); 'unknown' = envelope too loose / budget out
    -- the only verdict where an adaptive deeper budget could still certify. Instrument for the DNF seeds."""
    def _and(a, b):
        if "refuted" in (a, b): return "refuted"
        return "certified" if a == b == "certified" else "unknown"
    def _or(a, b):
        if "certified" in (a, b): return "certified"
        return "refuted" if a == b == "refuted" else "unknown"
    d = tau if delta is None else delta
    overall = "certified"; details = []
    for ent in cyl:                                   # NB diagnostic twin stays ISOTROPIC for the ellipse
        (c0, vv, aa, R, zc, veff) = ent[:6]           # (conservative); CAPSULE movers get the pearl AND --
        cap = _cap_of(ent)                            # q̃ as a plain centred circle would be optimistic
        if cap is not None:
            hs3 = "certified"
            for sg in _cap_grid(cap):
                h3, _m3 = ego.certify_horizontal3(obs_c0=c0, R=R, obs_vel=tuple(np.asarray(vv, float) * sg),
                                                  obs_acc=(0, 0, 0), t_hi=tau, v_eff=veff, delta=d)
                hs3 = _and(hs3, h3)
            vo3, mv3 = ego.certify_above3(z_clear=zc, t_hi=tau, v_eff_z=0.0, delta=d)
            mover = _or(hs3, vo3)
            details.append(dict(horiz_pred=hs3, horiz_cur=hs3, above=vo3, verdict=mover,
                                margins=(0.0, 0.0, round(float(mv3), 4))))
            overall = _and(overall, mover)
            continue
        hp, mp = ego.certify_horizontal3(obs_c0=c0, R=R, obs_vel=vv, obs_acc=aa, t_hi=tau, v_eff=veff, delta=d)
        hc, mc = ego.certify_horizontal3(obs_c0=c0, R=R, obs_vel=(0, 0, 0), t_hi=tau, v_eff=veff, delta=d)
        vo, mv = ego.certify_above3(z_clear=zc, t_hi=tau, v_eff_z=0.0, delta=d)
        mover = _or(_and(hp, hc), vo)
        details.append(dict(horiz_pred=hp, horiz_cur=hc, above=vo, verdict=mover,
                            margins=(round(mp, 4), round(mc, 4), round(mv, 4))))
        overall = _and(overall, mover)
    return overall, details


def _rot(v2, ang):
    c, s = np.cos(ang), np.sin(ang)
    return np.array([c * v2[0] - s * v2[1], s * v2[0] + c * v2[1]])


def maneuver_decide(ego, p_d, v_d, a_d, goal, ztop, clear_fn, cruise_z=CRUISE_Z, horizon=HORIZON,
                    straight_clip=None):
    """Run the fastest-safe tournament and LEAVE ego holding the chosen B-spline. `clear_fn()` -> bool gates each
    committed candidate (normally cert_clear(ego, cyl); the discrete-sampling ablation passes its own). Returns the
    kind 'straight'|'around_l'|'around_r'|'over'|'climb'|'evade'. 'evade' = nothing certified -> caller flees via
    evade_setpoint(); any other kind = ego holds a CERTIFIED plan ('climb' included -- an uncertified climb was
    the seed-56 soundness hole the renderer fixed at 6/27; this shared layer now applies the same gate: NEVER
    fly uncertified, fall through to 'evade' instead).
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
    # ground blocked -> fly OVER (certified); boxed -> CERTIFIED vertical climb-escape; else evade (the caller's
    # no-freeze fallback). The climb MUST pass clear_fn() too -- flying an uncertified climb was the seed-56
    # soundness hole (a labelled-certified lap could collide); renderer fixed 6/27, ported here.
    if ego.replan(p_d, v_d, a_d, np.array([goal[0], goal[1], ztop])) and ego.duration() > 1e-3 and clear_fn():
        return "over"
    if ego.replan(p_d, v_d, a_d, np.array([p_d[0], p_d[1], ztop])) and ego.duration() > 1e-3 and clear_fn():
        return "climb"
    return "evade"


def evade_setpoint_blend(p_d, v_d, movers_xy, max_vel, dt, gdir, a_brake=3.0, beta=0.35):
    """VECTOR-COMPOSED graceful fallback (EVADE_BLEND=1): the previous tick's certificate already
    covers braking along the committed course (speed-scaled trust window semantics), so the sound
    and smooth response to 'nothing certifies' is DECELERATE ALONG the current velocity and ADD a
    bounded away-component -- never overwrite the motion vector with a panic flee/stop."""
    p_d = np.asarray(p_d, float); v = np.asarray(v_d, float)[:2]
    sp = float(np.linalg.norm(v))
    v_dec = v * max(0.0, 1.0 - (a_brake * dt) / max(sp, 1e-6)) if sp > 1e-6 \
        else np.asarray(gdir, float) * 0.0
    away = np.zeros(2)
    if len(movers_xy):
        nn = min(movers_xy, key=lambda c: np.linalg.norm(np.asarray(c)[:2] - p_d[:2]))
        d = p_d[:2] - np.asarray(nn)[:2]; n = float(np.linalg.norm(d))
        if n > 1e-6:
            away = d / n * min(beta * max_vel, beta * max_vel * (4.0 / max(n, 1.0)))
    v_cmd = v_dec + away                                   # 加法合成,不覆盖
    spc = float(np.linalg.norm(v_cmd))
    if spc > max_vel:
        v_cmd *= max_vel / spc
    pos = p_d + np.array([v_cmd[0] * dt, v_cmd[1] * dt, 0.0])
    return pos, np.array([v_cmd[0], v_cmd[1], 0.0])


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


def maneuver_decide_sticky(ego, p_d, v_d, a_d, goal, ztop, clear_fn, state,
                           cruise_z=CRUISE_Z, horizon=HORIZON, straight_clip=None,
                           dwell_ticks=3, clear_fn_strict=None):
    """Event-triggered smoothing wrapper around maneuver_decide (SMOOTH=1; anti-chatter).

    The per-tick tournament re-picks the first-certified candidate from scratch, so when two
    candidates certify marginally the winner flip-flops tick to tick (seed12: 24 switches,
    spchurn 69). Here the INCUMBENT maneuver keeps flying while it still certifies; the full
    tournament runs only on EVENTS:
      * incumbent cert fails  -> immediate full tournament (safety path identical to before);
      * every `dwell_ticks`   -> probe an UPGRADE (higher-preference kind, e.g. back to straight),
        accepted only through the STRICT gate (clear_fn_strict = tube inflated by an extra delta)
        so a marginal certificate cannot yank the drone out of a detour it just committed to.
    'evade' is never sticky. Same contract as maneuver_decide: ego ends holding a CERTIFIED plan.
    state: dict persisted by the caller across ticks; keys kind/age."""
    p_d = np.asarray(p_d, float); goal = np.asarray(goal, float)
    gxy = goal[:2] - p_d[:2]; dist = float(np.linalg.norm(gxy))
    gdir = gxy / dist if dist > 1e-6 else np.array([1.0, 0.0])
    L = min(horizon, max(dist, 1.0))

    _gaps = {}
    # GAP_CARROT is v2-only: the world-aimed carrot builder needs the mover cylinders (cyl), which
    # this sticky signature does not receive. A copy of the v2 block lived here referencing the
    # UNDEFINED name `cyl` -- an armed NameError whenever GAP_CARROT=1 reached the replay sticky path
    # (pyflakes sweep 2026-07-16, same family as the dead calib loader). _gaps stays empty: _gsub
    # returns None for gap keys and the grid skips them, the sound carrot-absent behaviour.

    def _gsub(kind):
        if kind in _gaps:
            return _gaps[kind]
        if kind == "straight":
            return (np.array([goal[0], goal[1], cruise_z]) if straight_clip is None
                    else np.array([*(p_d[:2] + gdir * min(straight_clip, dist)), cruise_z]))
        ang = {"around_l": PHI, "around_r": -PHI}.get(kind)
        if ang is not None:
            return np.array([*(p_d[:2] + L * _rot(gdir, ang)), cruise_z])
        if kind == "soar":
            # CLIMB-FORWARD (vector composition, vertical edition): keep goal-ward progress WHILE
            # ascending just above the certified columns -- the graceful crowd-escape that pure
            # 'climb' (freeze-and-rise) and far-high 'over' both miss. Goal z stays controlled by
            # the mission; this is a transit carrot only.
            return np.array([*(p_d[:2] + gdir * (0.6 * L)), ztop])
        if kind == "over":
            return np.array([goal[0], goal[1], ztop])
        if kind == "climb":
            return np.array([p_d[0], p_d[1], ztop])
        return None

    PREF = ("straight", "around_l", "around_r", "over", "climb")
    inc = state.get("kind")
    # WORLD-FROZEN sub-goal: the incumbent's carrot is fixed at commit time. Recomputing it every
    # tick (v1) made lateral carrots ROTATE with the drone -> spiral wandering (fast_canyon +10s).
    gs_inc = state.get("gsub")
    if inc in PREF and gs_inc is not None and float(np.linalg.norm(gs_inc[:2] - p_d[:2])) < 1.5:
        inc = None                                            # carrot reached -> re-decide
    if inc in PREF and gs_inc is not None:
        state["age"] = state.get("age", 0) + 1
        if state["age"] >= dwell_ticks and inc != "straight":
            state["age"] = 0                                  # probe cadence: once per dwell window
            strict = clear_fn_strict or clear_fn
            for uk in PREF[:PREF.index(inc)]:
                gs = _gsub(uk)
                if ego.replan(p_d, v_d, a_d, gs) and ego.duration() > 1e-3 and strict():
                    state.update(kind=uk, gsub=gs, age=0)
                    return uk
        if ego.replan(p_d, v_d, a_d, gs_inc) and ego.duration() > 1e-3 and clear_fn():
            return inc                                        # incumbent retry toward FROZEN carrot
    kind = maneuver_decide(ego, p_d, v_d, a_d, goal, ztop, clear_fn,
                           cruise_z=cruise_z, horizon=horizon, straight_clip=straight_clip)
    state.update(kind=kind, gsub=_gsub(kind), age=0)
    return kind


def cert_clear_warp(ego, cyl, s, tau=TAU, delta=None):
    """Constant-slip RETIME certification of the committed spline at warp s<=1 -- the headless twin
    of the renderer's slip-behind identity (sound substitution obs_vel=v/s, t_hi=s*tau, v_eff=veff/s,
    delta=d*s). Predicted-only: the frozen-at-current conjunct of cert_clear would forbid every
    yield. Statics (vel=0, veff=0) are warp-invariant -- slowing never fixes a static conflict."""
    d = (tau if delta is None else delta)
    s = float(s)
    for ent in cyl:
        (c0, vv, aa, R, zc, veff) = ent[:6]
        cap = _cap_of(ent)
        if cap is not None:                     # retime commutes with the pearl cover too
            hp = True
            for sg in _cap_grid(cap):
                hs, _ = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=tuple(np.asarray(vv, float) * sg / s),
                                               obs_acc=(0, 0, 0), t_hi=s * tau, v_eff=veff / s, delta=d * s)
                if not hs:
                    hp = False
                    break
            if not hp:
                hp = _cap_behind(ent, ego, tau, d, warp=s)
        else:
            ell = _ell_of(ent, ego)
            if ell is not None:                 # retime and the constant whitening map commute:
                _k, _ux, _uy, _rw = ell         # slip identity + ellipse compose soundly
                hp, _ = ego.certify_horizontal_aniso(obs_c0=c0, R=_rw, obs_vel=tuple(np.asarray(vv, float) / s),
                                                     obs_acc=(0, 0, 0), t_hi=s * tau, v_eff=veff / s, delta=d * s,
                                                     ux=_ux, uy=_uy, kappa=_k)
            else:
                hp, _ = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=tuple(np.asarray(vv, float) / s),
                                               obs_acc=(0, 0, 0), t_hi=s * tau, v_eff=veff / s, delta=d * s)
        vo, _ = ego.certify_above(z_clear=zc, t_hi=s * tau, v_eff_z=0.0, delta=d * s)
        if not (hp or vo):
            return False
    return True


def cert_clear_margin(ego, cyl, tau=TAU, delta=None):
    """Margin sister of cert_clear: (ok, m) where m ~ metres of surplus clearance beyond the
    certified floor (min over movers; deficit-squared margins normalised by 2R). inf when no cyl."""
    d = tau if delta is None else delta
    ok_all, m_min = True, float("inf")
    for ent in cyl:
        (c0, vv, aa, R, zc, veff) = ent[:6]
        cap = _cap_of(ent)
        ell = None if cap is not None else _ell_of(ent, ego)
        if cap is not None:
            hp = hc = True; mp = mc = float("inf")
            for sg in _cap_grid(cap):
                hs, ms = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=tuple(np.asarray(vv, float) * sg),
                                                obs_acc=(0, 0, 0), t_hi=tau, v_eff=veff, delta=d)
                mp = mc = min(mp, float(ms))
                if not hs:
                    hp = hc = _cap_behind(ent, ego, tau, d)
                    if hp:
                        mp = mc = 0.0        # wake pass: neutral margin (no surplus claimed)
                    break
        elif ell is not None:
            _k, _ux, _uy, _rw = ell             # margins in the warped metric, normalised by 2*R_warp
            hp, mp = ego.certify_horizontal_aniso(obs_c0=c0, R=_rw, obs_vel=vv, obs_acc=aa, t_hi=tau,
                                                  v_eff=veff, delta=d, ux=_ux, uy=_uy, kappa=_k)
            hc, mc = ego.certify_horizontal_aniso(obs_c0=c0, R=_rw, obs_vel=(0, 0, 0), t_hi=tau,
                                                  v_eff=veff, delta=d, ux=_ux, uy=_uy, kappa=_k)
            R = _rw
        else:
            hp, mp = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=vv, obs_acc=aa, t_hi=tau, v_eff=veff, delta=d)
            hc, mc = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=(0, 0, 0), t_hi=tau, v_eff=veff, delta=d)
        vo, mv = ego.certify_above(z_clear=zc, t_hi=tau, v_eff_z=0.0, delta=d)
        ok = (hp and hc) or vo
        ok_all &= ok
        if not ok:
            return False, -1.0
        branch = max(min(float(mp), float(mc)), float(mv)) / max(2.0 * float(R), 1e-6)
        m_min = min(m_min, branch)
    return ok_all, m_min


def cert_clear_warp_margin(ego, cyl, s, tau=TAU, delta=None):
    """Margin sister of cert_clear_warp (retime margins x s back to world scale)."""
    d = (tau if delta is None else delta)
    s = float(s)
    m_min = float("inf")
    for ent in cyl:
        (c0, vv, aa, R, zc, veff) = ent[:6]
        cap = _cap_of(ent)
        ell = None if cap is not None else _ell_of(ent, ego)
        if cap is not None:
            hp = True; mp = float("inf")
            for sg in _cap_grid(cap):
                hs, ms = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=tuple(np.asarray(vv, float) * sg / s),
                                                obs_acc=(0, 0, 0), t_hi=s * tau, v_eff=veff / s, delta=d * s)
                mp = min(mp, float(ms))
                if not hs:
                    hp = _cap_behind(ent, ego, tau, d, warp=s)
                    if hp:
                        mp = 0.0
                    break
        elif ell is not None:
            _k, _ux, _uy, _rw = ell
            hp, mp = ego.certify_horizontal_aniso(obs_c0=c0, R=_rw, obs_vel=tuple(np.asarray(vv, float) / s),
                                                  obs_acc=(0, 0, 0), t_hi=s * tau, v_eff=veff / s, delta=d * s,
                                                  ux=_ux, uy=_uy, kappa=_k)
            R = _rw
        else:
            hp, mp = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=tuple(np.asarray(vv, float) / s),
                                            obs_acc=(0, 0, 0), t_hi=s * tau, v_eff=veff / s, delta=d * s)
        vo, mv = ego.certify_above(z_clear=zc, t_hi=s * tau, v_eff_z=0.0, delta=d * s)
        if not (hp or vo):
            return False, -1.0
        m_min = min(m_min, s * max(float(mp), float(mv)) / max(2.0 * float(R), 1e-6))
    return True, m_min


_ST_ON = os.environ.get("ST_SPEED", "0") == "1"   # M2-4 (2026-07-17): ST-graph speed stage in the
#   tournament -- per direction candidate, project the cyl movers into (param-station, time)
#   forbidden boxes over the commitment window tau, DP the fastest schedule (gear set = tournament
#   grid + stop, ONE-NOTCH transitions = SPEED_SLEW-legal by construction), certify it with the
#   piecewise-warp composer (st_cert, Route B). DP PROPOSES, the certificate JUDGES; an uncertified
#   or not-strictly-better schedule falls back to the constant-gear _best_s -- worst case = today.
_ST_MODS = [None]                                  # lazy (st_speed, st_cert) | False = loudly disabled


def maneuver_decide_v2(ego, p_d, v_d, a_d, goal, ztop, cyl, state,
                       cruise_z=CRUISE_Z, horizon=HORIZON, straight_clip=None,
                       tau=TAU, delta=None, speeds=None,  # default (1.0,0.6,0.3); 0.3 ABSORBS evades (without it evade 40->106)
                       dwell_ticks=3, strict_margin=0.15, extra_gate=None, carrot="frozen"):
    # carrot="frozen": incumbent sub-goal fixed in the WORLD (short-corridor benchmark: moving
    #   carrots spiral, A/B 07-06). carrot="angle": incumbent DEFLECTION re-anchored to the current
    #   goal direction each tick (long routes: a frozen point goes stale; the renderer's CCF rule).
    """UNIFIED tournament (task#8): candidates = (direction x speed) grid, ONE implementation for
    both the headless benchmark and (stage-2) the renderer. Absorbs the four bolt-on speed
    governors (slip / CCF-warp / CRET / FOVCAP): speed is a first-class tournament dimension, so
    "yield behind a crosser" is just the (same-direction, s<1) candidate winning -- not a fallback
    patch firing after the full-speed-only tournament already failed.

    Directions: straight, +-PHI, +-2PHI, over, climb (one ego.replan each -- the expensive node).
    Speeds: s=1 certified by cert_clear (frozen-at-current conjunct kept: full-speed semantics
    unchanged); s<1 by cert_clear_warp (slip identity, predicted-only -- the renderer's yield rule).
    Score: goal-ward progress of the RETIMED flight over the trust window, so a fast detour can
    honestly beat a slow straight. Anti-chatter built in (world-frozen incumbent + dwell-gated
    strict upgrades + one-grid-step speed release). SPEEDS_CRAWL=1 appends a 0.15 crawl gear:
    certified slow progress (0.45 m/s -> 0.34m inside the whole trust window) absorbs the 1-3-tick
    "flicker" evades -- a certified crawl is elegant persistence, not an emergency (dream metric).
    Returns (kind, s); 'evade' when nothing
    certifies at any (direction, speed) -- caller flees, s meaningless.
    state: caller-persisted dict (kind / gsub / s / age)."""
    if speeds is None:
        speeds = (1.0, 0.6, 0.3, 0.15) if os.environ.get("SPEEDS_CRAWL", "0") == "1" else (1.0, 0.6, 0.3)
    d = (tau if delta is None else delta)
    p_d = np.asarray(p_d, float); goal = np.asarray(goal, float)
    gxy = goal[:2] - p_d[:2]; dist = float(np.linalg.norm(gxy))
    gdir = gxy / dist if dist > 1e-6 else np.array([1.0, 0.0])
    L = min(horizon, max(dist, 1.0))

    _gaps = {}
    if os.environ.get("GAP_CARROT", "0") == "1" and cyl:
        # WORLD-AIMED carrots (GapWeave S3): the fixed +-PHI fan is BLIND -- when a crosser owns the
        # corridor the tournament flip-flops between left/right guesses. Aim instead at the crosser's
        # WAKE: the point just behind its closest-approach position (pass-behind, the human road-
        # crossing move); pass-ahead as the alternate. Speed-first ranking untouched.
        _cand = None
        for c in cyl:
            _vv = np.asarray(c[1][:2], float)
            if float(np.hypot(*_vv)) < 0.5:
                continue
            _d = np.asarray(c[0][:2], float) - p_d[:2]
            _dn = float(np.linalg.norm(_d))
            if _dn < 1e-6 or float(_d @ gdir) / _dn < 0.2:
                continue
            if _cand is None or _dn < _cand[0]:
                _cand = (_dn, np.asarray(c[0][:2], float), _vv, float(c[3]))
        if _cand is not None:
            _, _c0, _vv, _R = _cand
            _vn = _vv / max(float(np.hypot(*_vv)), 1e-6)
            _tc = max(0.0, min(2.5, float(-((_c0 - p_d[:2]) @ (_vv - gdir * 3.0))
                                          / max(float((_vv - gdir * 3.0) @ (_vv - gdir * 3.0)), 1e-6))))
            _pc = _c0 + _vv * _tc
            for _tag, _sgn in (("gap_b", -1.0), ("gap_a", +1.0)):
                _pt = _pc + _vn * _sgn * (_R + 0.8)
                _dirv = _pt - p_d[:2]
                _dl = float(np.linalg.norm(_dirv))
                if _dl > 0.5:
                    _gaps[_tag] = np.array([*(p_d[:2] + _dirv / _dl * L), cruise_z])

    def _gsub(kind):
        if kind in _gaps:
            return _gaps[kind]
        if kind == "straight":
            return (np.array([goal[0], goal[1], cruise_z]) if straight_clip is None
                    else np.array([*(p_d[:2] + gdir * min(straight_clip, dist)), cruise_z]))
        ang = {"around_l": PHI, "around_r": -PHI, "around_l2": 2 * PHI, "around_r2": -2 * PHI}.get(kind)
        if ang is not None:
            return np.array([*(p_d[:2] + L * _rot(gdir, ang)), cruise_z])
        if kind == "soar":
            # CLIMB-FORWARD (vector composition, vertical edition): keep goal-ward progress WHILE
            # ascending just above the certified columns -- the graceful crowd-escape that pure
            # 'climb' (freeze-and-rise) and far-high 'over' both miss. Goal z stays controlled by
            # the mission; this is a transit carrot only.
            return np.array([*(p_d[:2] + gdir * (0.6 * L)), ztop])
        if kind == "over":
            return np.array([goal[0], goal[1], ztop])
        if kind == "climb":
            return np.array([p_d[0], p_d[1], ztop])
        return None

    def _ok_plan():
        return extra_gate() if extra_gate is not None else True

    _tau_speed = os.environ.get("TAU_SPEED", "0") == "1"
    _fov_on = os.environ.get("FOV_RET", "0") == "1"
    if _fov_on:
        # sensor cone mirrored from the perception front-end defaults (perception.py from_env);
        # the tournament has no live handle on the front-end object, only its published dials
        _fov_cos = math.cos(math.radians(min(float(os.environ.get("PERCEPT_FOV_DEG", "45.0")), 180.0)))
        _fov_rng = float(os.environ.get("PERCEPT_RANGE", "10.0"))

    def _cert_at(s, strict=False):
        dd = d + (strict_margin if strict else 0.0)
        # TAU_SPEED=1: brake-safety trust window -- the certificate must cover one decision tick
        # plus the STOP from the committed speed (v=3s, a=6 -> t_stop=s/2). Full speed keeps the
        # frozen 0.75s; a crawl honestly needs only ~0.38s, so its tube grows half as much: the
        # "even standing still is uncertifiable" flicker ticks become certifiable slow progress.
        t_c = min(tau, 0.30 + 0.5 * s + 0.05) if _tau_speed else tau
        if s >= 0.999:
            return cert_clear(ego, cyl, tau=t_c, delta=dd)
        return cert_clear_warp(ego, cyl, s, tau=t_c, delta=dd)

    def _best_s(smax=1.0, strict=False):
        """Fastest certified speed for the CURRENT ego spline, scanning the grid down from smax."""
        for s in speeds:
            if s > smax + 1e-9:
                continue
            if _cert_at(s, strict):
                return s
        return 0.0

    def _st_stage(v0_gear):
        """M2-4 ST-graph speed schedule for the CURRENTLY-held candidate spline. Boxes only within
        the commitment window tau (the review's influence cap: conflicts beyond the commit scale
        must not steer the per-tick recommit -- gap-law v1 lesson). Returns (s_rank, head_gear,
        pieces) or None. The DP proposes; certify_profile (piecewise-warp Bernstein composer)
        judges; caller falls back to the constant-gear path when this returns None."""
        if _ST_MODS[0] is False:
            return None
        if _ST_MODS[0] is None:
            try:
                import st_speed as _STS
                import st_cert as _STC
                if getattr(ego, "get_bsegs", None) is None:
                    raise RuntimeError("EGOPlanner lacks get_bsegs (stale bridge?)")
                _ST_MODS[0] = (_STS, _STC)
            except Exception as e:   # LOUD disable, never silent (2026-07-16 law)
                print(f"[st] ST_SPEED DISABLED: {type(e).__name__}: {e}", flush=True)
                _ST_MODS[0] = False
                return None
        _STS, _STC = _ST_MODS[0]
        dur = ego.duration()
        if dur <= 1e-3:
            return None
        dt_dp = 0.15
        t_rows = np.arange(0.0, tau + 1e-9, dt_dp)
        if len(t_rows) < 3:
            return None
        du = dt_dp / 8.0
        u_hi = min(dur - 1e-3, tau)                      # max param reachable at gear 1.0 in-window
        us = np.arange(0.0, u_hi + 1e-9, du)
        if len(us) < 8:
            return None
        pts = np.asarray([ego.eval(float(u))[0][:2] for u in us], float)
        blocked = np.zeros((len(t_rows), len(us)), bool)
        for ent in cyl:
            c0e = np.asarray(ent[0], float); vve = np.asarray(ent[1], float); Re = float(ent[3])
            for j, tj in enumerate(t_rows):
                cc = c0e[:2] + vve[:2] * tj
                blocked[j] |= (np.hypot(pts[:, 0] - cc[0], pts[:, 1] - cc[1]) < Re)
        g_set = tuple(sorted(set(tuple(speeds) + (0.0,)), reverse=True))
        r = _STS.dp_profile(blocked, us, t_rows, v0=float(v0_gear), v_max=1.0, a_max=0.0,
                            gears=g_set)                 # a_max=0 => one-notch-per-step ONLY
        if r is None or len(r["gear_seq"]) < 2:
            return None
        pieces = [(dt_dp, float(g)) for g in r["gear_seq"][1:]]
        total = sum(p[0] for p in pieces)
        if total < tau - 1e-6:                           # goal hit early: pad to tile [0, tau]
            pieces.append((tau - total, pieces[-1][1]))
        for pad_gear in (None, 0.0):                     # retry once with a hover pad if we overrun
            if pad_gear is not None:
                pieces[-1] = (pieces[-1][0], pad_gear)
            try:
                okc, _m = _STC.certify_profile(ego, cyl, pieces, tau, d)
                break
            except ValueError:
                okc = False
                continue
        if not okc:
            return None
        s_rank = float(r["s_end"]) / max(u_hi, 1e-6)
        return (s_rank, float(pieces[0][1]), pieces)

    def _progress(s):
        rr = ego.eval(min(s * tau, max(ego.duration() - 1e-3, 0.0)))
        if rr is None:
            return -1e9
        return float(np.dot(np.asarray(rr[0], float)[:2] - p_d[:2], gdir))

    def _fov_ret(s):
        """FOV-retention of the CURRENT ego spline flown at warp s: the share of (moving keep-out,
        sample time) pairs the yaw-to-path sensor cone keeps in view across the trust window.
        Heading proxy = plan velocity direction (the executor yaws to the commanded path, so the
        cone follows it within a tick). Statics are skipped (a remembered tree cannot be 'lost');
        young frozen plates DO count via veff>0 -- they are exactly the tracks that need looks to
        mature. Samples beyond sensor range score neither way; no relevant threat -> neutral 1.0."""
        hits = tot = 0
        hd = gdir
        for k in range(1, 6):
            tw = tau * k / 5.0
            rr = ego.eval(min(s * tw, max(ego.duration() - 1e-3, 0.0)))
            if rr is None:
                continue
            pp = np.asarray(rr[0], float)[:2]
            ve = np.asarray(rr[1], float)[:2]
            if float(np.hypot(*ve)) > 0.3:
                hd = ve / float(np.hypot(*ve))
            for ent in cyl:
                (c0, vv, aa, R, zc, veff) = ent[:6]
                if float(np.hypot(vv[0], vv[1])) <= 0.3 and veff <= 1e-6:
                    continue
                ct = (np.asarray(c0, float)[:2] + np.asarray(vv, float)[:2] * tw
                      + 0.5 * np.asarray(aa, float)[:2] * tw * tw)
                dv = ct - pp; dn = float(np.linalg.norm(dv))
                if dn < 1e-6 or dn > _fov_rng:
                    continue
                tot += 1
                if float(dv @ hd) / dn >= _fov_cos:
                    hits += 1
        return hits / tot if tot else 1.0

    # ESCAPE-PRESSURE TRIGGER (ESC_TRIG=1): a closing pocket kills EVERY candidate once shut --
    # vertical/wide escapes must be taken while they still certify. Signal = the gap to the nearest
    # MOVING keep-out surface shrinking across consecutive ticks; response = prefer the escape
    # family THIS tick (soar first, then wide arounds). Preference only: every candidate still has
    # to pass the certificate, so safety semantics are untouched.
    _esc_on = os.environ.get("ESC_TRIG", "0") == "1"
    _esc_hot = False
    if _esc_on and cyl:
        # v2 signal (directional projection): only movers that squeeze the FORWARD corridor count --
        # inside the +-70deg goal cone AND with relative velocity pointing at the drone. A pedestrian
        # passing behind must not scramble the escape family.
        _gxy0 = goal[:2] - p_d[:2]
        _gd0 = _gxy0 / max(float(np.linalg.norm(_gxy0)), 1e-6)
        def _squeezes(c):
            _d = np.asarray(c[0][:2], float) - np.asarray(p_d[:2], float)
            _dn = float(np.linalg.norm(_d))
            if _dn < 1e-6 or float(np.hypot(c[1][0], c[1][1])) <= 0.3:
                return False
            if float(_d @ _gd0) / _dn < 0.34:              # outside the forward cone (~70deg half)
                return False
            return float(np.asarray(c[1][:2], float) @ (-_d / _dn)) > 0.2   # closing on the drone
        _g = min((float(np.hypot(c[0][0] - p_d[0], c[0][1] - p_d[1])) - float(c[3])
                  for c in cyl if _squeezes(c)), default=1e9)
        _gh = state.setdefault("g_hist", [])
        _gh.append(_g)
        del _gh[:-4]
        if len(_gh) >= 3:
            _rate = (_gh[0] - _gh[-1]) / (0.3 * (len(_gh) - 1))
            _esc_hot = (_g < float(os.environ.get("ESC_G", "2.5"))
                        and _rate > float(os.environ.get("ESC_RATE", "0.8")))
        if _esc_hot:
            state["esc_fired"] = state.get("esc_fired", 0) + 1

    _soar = os.environ.get("SOAR", "0") == "1"
    _soar_eager = os.environ.get("SOAR_EAGER", "0") == "1"
    DIRS = (("straight", "around_l", "around_r", "soar", "around_l2", "around_r2", "over", "climb")
            if (_soar and _soar_eager) else
            ("straight", "around_l", "around_r", "around_l2", "around_r2", "soar", "over", "climb")
            if _soar else
            ("straight", "around_l", "around_r", "around_l2", "around_r2", "over", "climb"))
    if os.environ.get("GAP_CARROT", "0") == "1":
        DIRS = ("straight", "gap_b", "gap_a") + tuple(k for k in DIRS if k != "straight")
        #   gap keys ALWAYS in DIRS when the feature is on (blueprint rule): with _gaps empty,
        #   _gsub returns None and the grid SKIPS them -- incumbent lookups never ValueError
    if _esc_hot:
        # escape family first; WITHIN the family no paternal ordering -- the speed-lexicographic
        # rank decides (user ruling 2026-07-08: "if both certify, the faster gear arrives faster
        # and still wins"; a laterals-first reorder was tried and was a verified no-op anyway)
        _escf = tuple(k for k in ("soar", "around_l2", "around_r2", "over") if k in DIRS)
        DIRS = _escf + tuple(k for k in DIRS if k not in _escf)
    inc, inc_s = state.get("kind"), float(state.get("s", 1.0))
    gs_inc = state.get("gsub")
    if carrot == "angle" and inc in DIRS:
        gs_inc = _gsub(inc)                                 # re-anchor the deflection to the current gdir
    if inc in DIRS and gs_inc is not None and float(np.linalg.norm(gs_inc[:2] - p_d[:2])) < 1.5:
        inc = None                                          # carrot reached -> re-decide

    # --- incumbent retry (anti-chatter): keep flying the committed carrot at the fastest certified
    #     speed; release speed upward one grid step per tick (brake fast, release slow).
    if inc in DIRS and gs_inc is not None:
        state["age"] = state.get("age", 0) + 1
        if state["age"] >= dwell_ticks and inc != "straight":
            state["age"] = 0                                # dwell-gated STRICT upgrade probe
            for uk in DIRS[:DIRS.index(inc)]:
                gs = _gsub(uk)
                if gs is not None and ego.replan(p_d, v_d, a_d, gs) and ego.duration() > 1e-3 and _ok_plan():
                    s_up = _best_s(strict=True)
                    if s_up >= max(inc_s, speeds[-1]) - 1e-9 and s_up > 0.0:
                        # upgrade must be certified-with-margin AND not slower than the incumbent
                        state.update(kind=uk, gsub=gs, s=s_up)
                        return uk, s_up
        if ego.replan(p_d, v_d, a_d, gs_inc) and ego.duration() > 1e-3 and _ok_plan():
            idx = max(0, speeds.index(inc_s) - 1) if inc_s in speeds else 0
            s_now = _best_s(smax=speeds[idx])               # may rise ONE grid step above last tick
            if s_now > 0.0:
                state["s"] = s_now
                return inc, s_now

    # --- full grid: replan per direction (expensive), scan speeds (cheap), score retimed progress
    best = (None, 0.0, -1e9)
    # TIE_KEEP=eps (塔菲大人 2026-07-16, 平手裁决=变动最小): quantise the progress score to eps-wide
    # buckets and, inside a bucket, prefer the INCUMBENT direction -- switching arms must be worth a
    # real progress difference. (History: built chasing the 07-16 "import flips the flight" case and
    # booked NEGATIVE for it -- the true cause was the dead per-class calib loader in the renderer,
    # not score ties. Kept, default 0 = byte-identical, as the deterministic commitment hook for the
    # racing-line work: every switch is a jerk event. Certificates untouched -- reorders CERTIFIED
    # candidates only.)
    _tie_eps = float(os.environ.get("TIE_KEEP", "0"))
    _inc0 = state.get("kind")
    _st_by = {}                             # dk -> (s_rank, head_gear, pieces) when the ST stage won
    last_replanned = None
    for dk in DIRS:
        gs = _gsub(dk)
        if gs is None:
            continue                        # gap carrot with binding mover gone: skip, NEVER replan(None)
        if not (ego.replan(p_d, v_d, a_d, gs) and ego.duration() > 1e-3):
            continue
        last_replanned = dk
        if not _ok_plan():
            continue
        s_ok = _best_s()
        if s_ok <= 0.0:
            continue
        if _ST_ON:
            # ST stage (M2-4): a certified piecewise schedule replaces the constant gear ONLY when
            # STRICTLY better (window progress rank > constant gear + 0.02); ties keep the tree.
            _str = _st_stage(float(state.get("s_prev") or state.get("s", 1.0)))
            if _str is not None and _str[0] > s_ok + 0.02:
                _st_by[dk] = _str
                s_ok = float(_str[0])       # rank by schedule progress (window-normalized)
        sc = _progress(s_ok)
        # LEXICOGRAPHIC rank (speed first): a full-speed detour beats ANY slowdown -- pure
        # window-progress scoring is myopic (slow-and-straight outscores fast-but-sideways over
        # 0.75 s, then stays slow: harness time +45%). Slowdowns only beat evade.
        _band = os.environ.get("SAFETY_BAND", "0") == "1"
        if _band or _fov_on:
            key = [s_ok, round(sc / 0.15)]
            if _fov_on:
                # FOV-retention tiebreak (perception-aware tournament, Mueller lineage 2026-07-13):
                # among equal-speed equal-progress candidates prefer the detour that KEEPS moving
                # threats inside the sensor cone. Losing the threat starts a KF coast; coast
                # inflates the calibrated tube; fat tubes kill the NEXT certificates. Preference
                # only -- speed-first rank and every certificate untouched.
                key.append(round(_fov_ret(s_ok), 2))
            if _band:
                # GapWeave S2 (user band ruling: surplus clearance beyond floor+0.5m is WASTE --
                # trade it for straightness/speed): ties broken by SMALLER excess margin.
                _tc = min(tau, 0.30 + 0.5 * s_ok + 0.05) if _tau_speed else tau
                _dd = d
                _mok, _m = (cert_clear_margin(ego, cyl, tau=_tc, delta=_dd) if s_ok >= 0.999
                            else cert_clear_warp_margin(ego, cyl, s_ok, tau=_tc, delta=_dd))
                _excess = max(0.0, (_m if np.isfinite(_m) else 5.0) - 0.5)
                key.append(-min(_excess, 5.0))
            key = tuple(key)
            _prev = tuple(best[1:]) + (-1e9,) * (len(key) - len(best) + 1)
            if key > _prev:
                best = (dk,) + key
        elif _tie_eps > 0.0:
            key = (s_ok, round(sc / _tie_eps), 1 if dk == _inc0 else 0)
            if key > tuple(best[1:]) + ((-1e9,) * (len(key) - len(best) + 1)):
                best = (dk,) + key
        elif (s_ok, sc) > (best[1], best[2]):
            best = (dk, s_ok, sc)
        if dk == "straight" and s_ok >= 0.999:
            break                                           # full-speed straight certified: done
    if best[0] is None:
        state.update(kind=None, gsub=None, s=1.0, age=0)
        return "evade", 0.0
    dk, s_ok = best[0], best[1]
    gs = _gsub(dk)
    _st_win = _st_by.get(dk) if _ST_ON else None
    if last_replanned != dk:
        ego.replan(p_d, v_d, a_d, gs)                       # restore the WINNER's spline (loop clobbered ego)
        # SOUNDNESS (GapWeave audit 2026-07-08 + M2-4): the restored spline is a FRESH replan, not
        # the one certified in the loop. ST schedule: RE-propose + RE-judge on the restored spline
        # (never fly a schedule certified on a clobbered spline); on failure fall to the plain
        # constant-gear path; still uncertified -> evade. Plain path: original re-certify unchanged.
        if _st_win is not None:
            _st_win = _st_stage(float(state.get("s_prev") or state.get("s", 1.0)))
            if _st_win is None:
                s_plain = _best_s()
                if s_plain <= 0.0:
                    state.update(kind=None, gsub=None, s=1.0, age=0)
                    return "evade", 0.0
                state.update(kind=dk, gsub=gs, s=s_plain, age=0)
                return dk, s_plain
        elif not _cert_at(s_ok):
            state.update(kind=None, gsub=None, s=1.0, age=0)
            return "evade", 0.0
    if _st_win is not None:
        # fly the schedule HEAD this tick (receding horizon: next tick re-decides); whole schedule
        # carries the piecewise-warp certificate over [0, tau]
        state.update(kind=dk, gsub=gs, s=float(_st_win[1]), age=0)
        return dk, float(_st_win[1])
    state.update(kind=dk, gsub=gs, s=s_ok, age=0)
    return dk, s_ok


def load_calib_v2(eps=0.05):
    """FS3C-R consumer: class -> dict(mature=(q0, v_eff), young=(q0y, growth), status).
    FAIL-CLOSED: an UNCALIBRATED / missing class gets q0=1e6 (nothing near it certifies) and a
    loud log line -- never a silent optimistic fallback (spec ruling #20 / #13)."""
    path = (os.environ.get("CALIB_FILE") or os.path.join(_OUTDIR, "calib_v2.json"))
    rep = json.load(open(path))                     # missing file = hard crash, intended
    out = {}
    for cls in ("pedestrian", "vehicle", "animal", "static"):
        lv = rep.get("groups", {}).get(cls, {}).get("levels", {}).get(str(eps), {})
        if "q_conformal" not in lv:
            print(f"[calib_v2] class '{cls}' UNCALIBRATED at eps={eps} -> FAIL-CLOSED (uncertifiable)",
                  flush=True)
            out[cls] = dict(mature=(1e6, 0.0), young=(1e6, 0.0), status="UNCALIBRATED")
            continue
        yy = rep.get("young", {}).get(cls, {}).get(str(eps))
        plates = [(int(d["age_lo"]), int(d["age_hi"]), d.get("coast"),
                   float(d["q_conformal"]), float(d["v_eff"]))
                  for d in (lv.get("plates") or {}).values()]
        out[cls] = dict(mature=(float(lv["q_conformal"]), float(lv["v_eff"])),
                        young=((float(yy["q0y"]), float(yy["growth"])) if yy
                               else (float(lv["q_conformal"]), float(lv["v_eff"]))),
                        plates=plates, status=lv.get("status", "ok"))
    out["_meta"] = dict(sha=rep.get("provenance", {}).get("config_sha"), eps=eps)
    return out


def load_calib_v6(eps=0.05):
    """v6 CAPSULE (segment conformal) consumer: same entry shape as v2 plus capsule=True and
    n_pearls. The mature law's q̃/v_eff are DIST-TO-SEGMENT quantities -- only sound when the cert
    covers the whole segment [c0, c0+v*t] (pearl string), never as a plain centred circle."""
    path = (os.environ.get("CALIB_FILE_V6") or os.path.join(_OUTDIR, "calib_v6.json"))
    rep = json.load(open(path))
    out = {}
    npearl = int(rep.get("n_pearls", 4))
    for cls in ("pedestrian", "vehicle", "animal", "static"):
        lv = rep.get("groups", {}).get(cls, {}).get("levels", {}).get(str(eps), {})
        if "q_conformal" not in lv:
            print(f"[calib_v6] class '{cls}' UNCALIBRATED at eps={eps} -> FAIL-CLOSED (uncertifiable)",
                  flush=True)
            out[cls] = dict(mature=(1e6, 0.0), young=(1e6, 0.0), plates=[], status="UNCALIBRATED",
                            capsule=False, n_pearls=npearl)
            continue
        yy = rep.get("young", {}).get(cls, {}).get(str(eps))
        rr = rep.get("rear", {}).get(cls, {}).get(str(eps))
        out[cls] = dict(mature=(float(lv["q_conformal"]), float(lv["v_eff"])),
                        young=((float(yy["q0y"]), float(yy["growth"])) if yy
                               else (float(lv["q_conformal"]), float(lv["v_eff"]))),
                        plates=[], status=lv.get("status", "ok"),
                        capsule=(cls in ("pedestrian", "vehicle")), n_pearls=npearl,
                        rear=((float(rr["q0r"]), float(rr["growth"])) if rr else None))
    out["_meta"] = dict(sha=rep.get("provenance", {}).get("shape_hash"), eps=eps, gen="v6")
    return out


def load_calib_v5(eps=0.05):
    """lambda-SHAPE-HE (v5, ELLIPTICAL motion-frame conformal) consumer: same entry shape as
    load_calib_v2 PLUS per-class 'kappa' (frozen along/cross aspect, >=1) and the shared
    'v_min_dir' (KF speed below which the direction is untrusted -> isotropic circle).
    FAIL-CLOSED like v2: missing class = q0 1e6, kappa 1."""
    path = (os.environ.get("CALIB_FILE_V5") or os.path.join(_OUTDIR, "calib_v5.json"))
    rep = json.load(open(path))                     # missing file = hard crash, intended
    out = {}
    vmin = float(rep.get("v_min_dir", 0.5))
    for cls in ("pedestrian", "vehicle", "animal", "static"):
        lv = rep.get("groups", {}).get(cls, {}).get("levels", {}).get(str(eps), {})
        if "q_conformal" not in lv:
            print(f"[calib_v5] class '{cls}' UNCALIBRATED at eps={eps} -> FAIL-CLOSED (uncertifiable)",
                  flush=True)
            out[cls] = dict(mature=(1e6, 0.0), young=(1e6, 0.0), plates=[], status="UNCALIBRATED",
                            kappa=1.0, v_min_dir=vmin)
            continue
        yy = rep.get("young", {}).get(cls, {}).get(str(eps))
        out[cls] = dict(mature=(float(lv["q_conformal"]), float(lv["v_eff"])),
                        young=((float(yy["q0y"]), float(yy["growth"])) if yy
                               else (float(lv["q_conformal"]), float(lv["v_eff"]))),
                        plates=[], status=lv.get("status", "ok"),
                        kappa=float(rep.get("kappa", {}).get(cls, 1.0)), v_min_dir=vmin)
    out["_meta"] = dict(sha=rep.get("provenance", {}).get("shape_hash"), eps=eps, gen="v5")
    return out


def v_cap(d_free, a_max, t_react=0.30, margin=1.0):
    """Braking-envelope speed cap (task#4, memo #10): the fastest v such that reaction distance +
    braking distance fits inside the perceived free distance:  v*t_react + v^2/(2a) <= d_free-margin.
    Closed form: v = -a*t + sqrt(a^2 t^2 + 2 a (d_free - margin)).  'Don't outrun your sensor' as an
    explicit, planner-side dial (all candidates planned at the same cap -- no executor warp flip)."""
    d = max(float(d_free) - float(margin), 0.0)
    a, t = float(a_max), float(t_react)
    return max(0.0, -a * t + math.sqrt(a * a * t * t + 2.0 * a * d))


_decide_v2_core = maneuver_decide_v2


def maneuver_decide_v2(ego, p_d, v_d, a_d, goal, ztop, cyl, state,
                       cruise_z=CRUISE_Z, horizon=HORIZON, straight_clip=None,
                       tau=TAU, delta=None, speeds=None, **kw):
    """SPEED_SLEW=1 wrapper: the winner speed moves at most ONE grid step per tick from the last
    flown gear (incremental vector edit, no jumps -> kills speed churn). A slewed gear is a
    different trajectory-in-time so it must RE-PASS the certificate on the winner plan; if it
    fails, the winner's certified gear is used unchanged (comfort never trades soundness)."""
    kind, s = _decide_v2_core(ego, p_d, v_d, a_d, goal, ztop, cyl, state, cruise_z=cruise_z,
                              horizon=horizon, straight_clip=straight_clip, tau=tau, delta=delta,
                              speeds=speeds, **kw)
    if os.environ.get("SPEED_SLEW", "0") != "1" or kind == "evade":
        state["s_prev"] = s
        return kind, s
    grid = list(speeds) if speeds is not None else (
        [1.0, 0.6, 0.3, 0.15] if os.environ.get("SPEEDS_CRAWL", "0") == "1" else [1.0, 0.6, 0.3])
    sp = state.get("s_prev")
    if sp is None or s not in grid or sp not in grid:
        state["s_prev"] = s
        return kind, s
    i_w, i_p = grid.index(s), grid.index(sp)
    _dwell = int(os.environ.get("SPEED_DWELL", "0"))
    if _dwell > 0 and i_w < i_p:                      # winner is FASTER (grid is descending)
        # upgrade dwell: a marginal cert that flips each tick makes the gear flap (one pitch event
        # per flap). Only release upward after the faster gear has won _dwell consecutive ticks;
        # braking (slower) stays IMMEDIATE -- the sound direction is never delayed.
        _cnt = state.get("up_cnt", 0) + 1
        state["up_cnt"] = _cnt
        if _cnt < _dwell:
            s_c = sp                                   # hold current gear (re-certified below)
        else:
            state["up_cnt"] = 0
            s_c = grid[i_p + max(-1, min(1, i_w - i_p))]
    else:
        state["up_cnt"] = 0
        s_c = grid[i_p + max(-1, min(1, i_w - i_p))]
    if s_c != s:
        d = (delta if delta is not None else 0.0)
        t_c = min(tau, 0.30 + 0.5 * s_c + 0.05) if os.environ.get("TAU_SPEED", "0") == "1" else tau
        ok = cert_clear(ego, cyl, tau=t_c, delta=d) if s_c >= 0.999 else \
            cert_clear_warp(ego, cyl, s_c, tau=t_c, delta=d)
        if not ok:
            s_c = s
    state["s_prev"] = s_c
    return kind, s_c
