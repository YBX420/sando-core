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
    # MAPPED STATICS (online-mapping perception): position uncertainty only, NO growing tube --
    # a remembered tree does not move; giving it the mover v_eff seals every corridor it borders.
    out.setdefault("static", (0.15, 0.0))
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
    for (c0, vv, aa, R, zc, veff) in cyl:
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

    def _gsub(kind):
        if kind == "straight":
            return (np.array([goal[0], goal[1], cruise_z]) if straight_clip is None
                    else np.array([*(p_d[:2] + gdir * min(straight_clip, dist)), cruise_z]))
        ang = {"around_l": PHI, "around_r": -PHI}.get(kind)
        if ang is not None:
            return np.array([*(p_d[:2] + L * _rot(gdir, ang)), cruise_z])
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
    for (c0, vv, aa, R, zc, veff) in cyl:
        hp, _ = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=tuple(np.asarray(vv, float) / s),
                                       obs_acc=(0, 0, 0), t_hi=s * tau, v_eff=veff / s, delta=d * s)
        vo, _ = ego.certify_above(z_clear=zc, t_hi=s * tau, v_eff_z=0.0, delta=d * s)
        if not (hp or vo):
            return False
    return True


def maneuver_decide_v2(ego, p_d, v_d, a_d, goal, ztop, cyl, state,
                       cruise_z=CRUISE_Z, horizon=HORIZON, straight_clip=None,
                       tau=TAU, delta=None, speeds=(1.0, 0.6, 0.3),  # 0.3 kept: it ABSORBS evades (without it evade 40->106); the time cost is the honest price of persistence
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
    strict upgrades + one-grid-step speed release). Returns (kind, s); 'evade' when nothing
    certifies at any (direction, speed) -- caller flees, s meaningless.
    state: caller-persisted dict (kind / gsub / s / age)."""
    d = (tau if delta is None else delta)
    p_d = np.asarray(p_d, float); goal = np.asarray(goal, float)
    gxy = goal[:2] - p_d[:2]; dist = float(np.linalg.norm(gxy))
    gdir = gxy / dist if dist > 1e-6 else np.array([1.0, 0.0])
    L = min(horizon, max(dist, 1.0))

    def _gsub(kind):
        if kind == "straight":
            return (np.array([goal[0], goal[1], cruise_z]) if straight_clip is None
                    else np.array([*(p_d[:2] + gdir * min(straight_clip, dist)), cruise_z]))
        ang = {"around_l": PHI, "around_r": -PHI, "around_l2": 2 * PHI, "around_r2": -2 * PHI}.get(kind)
        if ang is not None:
            return np.array([*(p_d[:2] + L * _rot(gdir, ang)), cruise_z])
        if kind == "over":
            return np.array([goal[0], goal[1], ztop])
        if kind == "climb":
            return np.array([p_d[0], p_d[1], ztop])
        return None

    def _ok_plan():
        return extra_gate() if extra_gate is not None else True

    def _cert_at(s, strict=False):
        dd = d + (strict_margin if strict else 0.0)
        if s >= 0.999:
            return cert_clear(ego, cyl, tau=tau, delta=dd)
        return cert_clear_warp(ego, cyl, s, tau=tau, delta=dd)

    def _best_s(smax=1.0, strict=False):
        """Fastest certified speed for the CURRENT ego spline, scanning the grid down from smax."""
        for s in speeds:
            if s > smax + 1e-9:
                continue
            if _cert_at(s, strict):
                return s
        return 0.0

    def _progress(s):
        rr = ego.eval(min(s * tau, max(ego.duration() - 1e-3, 0.0)))
        if rr is None:
            return -1e9
        return float(np.dot(np.asarray(rr[0], float)[:2] - p_d[:2], gdir))

    DIRS = ("straight", "around_l", "around_r", "around_l2", "around_r2", "over", "climb")
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
                if ego.replan(p_d, v_d, a_d, gs) and ego.duration() > 1e-3 and _ok_plan():
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
    last_replanned = None
    for dk in DIRS:
        gs = _gsub(dk)
        if not (ego.replan(p_d, v_d, a_d, gs) and ego.duration() > 1e-3):
            continue
        last_replanned = dk
        if not _ok_plan():
            continue
        s_ok = _best_s()
        if s_ok <= 0.0:
            continue
        sc = _progress(s_ok)
        # LEXICOGRAPHIC rank (speed first): a full-speed detour beats ANY slowdown -- pure
        # window-progress scoring is myopic (slow-and-straight outscores fast-but-sideways over
        # 0.75 s, then stays slow: harness time +45%). Slowdowns only beat evade.
        if (s_ok, sc) > (best[1], best[2]):
            best = (dk, s_ok, sc)
        if dk == "straight" and s_ok >= 0.999:
            break                                           # full-speed straight certified: done
    if best[0] is None:
        state.update(kind=None, gsub=None, s=1.0, age=0)
        return "evade", 0.0
    dk, s_ok, _sc = best
    gs = _gsub(dk)
    if last_replanned != dk:
        ego.replan(p_d, v_d, a_d, gs)                       # restore the WINNER's spline (loop clobbered ego)
    state.update(kind=dk, gsub=gs, s=s_ok, age=0)
    return dk, s_ok
