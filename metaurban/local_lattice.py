"""local_lattice — CPL-v3 certified local trajectory primitives (Stage 1: offline + self-cert).

Blueprint: docs/design-CPL-v3-cert-local-planner.md.

Each CANDIDATE is a Composite = the [0, DT] prefix of an intent primitive + a jerk-limited brake
branch from the eval(DT) state, certified AS ONE trajectory over t_cert = DT + t_brake(v_DT) +
SLACK (matches TAU_SPEED brake-safety window semantics). The flown segment is always inside the
certificate; the fallback is executing the already-certified brake branch (recursive feasibility).
The primitive tail [DT, T_p] is intent / warm-start seed only -- never flown, never certified.

This module is import-side-effect free (no env reads, no RNG) so v1/v2 stay byte-identical.
"""
import numpy as np

from cert_bridge import Certifier, monomial_to_bseg

DT = 0.30
T_P = 1.5           # intent primitive horizon
SLACK = 0.05
J_BRK = 40.0        # emergency brake jerk cap
J_C = 4.0           # comfort jerk cap (commit primitive)


# ---- quintic boundary-value primitive (per axis, closed form) --------------------------------

def quintic(p0, v0, a0, pT, vT, aT, T):
    """Power-basis coeffs a_k (6,) for p(t)=sum a_k t^k on [0,T] meeting the 6 boundary conditions.
    Scalar axis. c0..c2 direct; c3..c5 from a 3x3 terminal solve."""
    c0, c1, c2 = p0, v0, a0 / 2.0
    T2, T3, T4, T5 = T * T, T ** 3, T ** 4, T ** 5
    A = np.array([[T3, T4, T5], [3 * T2, 4 * T3, 5 * T4], [6 * T, 12 * T2, 20 * T3]])
    b = np.array([pT - c0 - c1 * T - c2 * T2, vT - c1 - 2 * c2 * T, aT - 2 * c2])
    c3, c4, c5 = np.linalg.solve(A, b)
    return np.array([c0, c1, c2, c3, c4, c5])


def quintic3(p0, v0, a0, pT, vT, aT, T):
    """3-axis quintic -> (6,3) coeffs."""
    return np.stack([quintic(p0[i], v0[i], a0[i], pT[i], vT[i], aT[i], T) for i in range(3)], axis=1)


def _poly_eval(coeffs, t, der=0):
    """coeffs (deg+1, naxes) power basis; eval der-th derivative at scalar/array t -> (…, naxes)."""
    coeffs = np.asarray(coeffs, float)
    n = coeffs.shape[0]
    t = np.atleast_1d(np.asarray(t, float))
    out = np.zeros((len(t), coeffs.shape[1]))
    for k in range(der, n):
        fall = 1.0
        for j in range(der):
            fall *= (k - j)
        out += fall * coeffs[k][None, :] * (t ** (k - der))[:, None]
    return out


# ---- brake branch (jerk-limited stop from an eval state) --------------------------------------

def t_brake(vmag, a_max):
    """Stopping time of the emergency brake: constant-decel core + jerk ramp."""
    return vmag / a_max + a_max / J_BRK


def brake_segment(p, v, a, a_max):
    """A single jerk-limited quintic stop from (p,v,a) to rest. Terminal position = p + v_dir*s,
    s = stopping distance for the brake profile. Returns (coeffs (6,3), dur). Peak accel validated
    by the feasibility filter; on violation the caller widens dur (see make_composite)."""
    vmag = float(np.linalg.norm(v[:2]))
    if vmag < 1e-6:
        # already stopped: a degenerate hold segment (deg-5 zero, feasible trivially)
        dur = DT
        return np.tile(np.array([p]).T, (1, 6)).T * 0 + np.stack([p, [0, 0, 0], [0, 0, 0],
                                                                   [0, 0, 0], [0, 0, 0], [0, 0, 0]]), dur
    tb = t_brake(vmag, a_max)
    vdir = v / max(np.linalg.norm(v), 1e-9)
    s = vmag * tb * 0.5                                  # avg-velocity stopping displacement
    p_stop = p + vdir * s
    return quintic3(p, v, a, p_stop, np.zeros(3), np.zeros(3), tb), tb


# ---- composite assembly -----------------------------------------------------------------------

def make_composite(prim, a_max, dt=DT):
    """prim = (6,3) intent quintic on [0, T_P]. Build the flown-this-tick composite: quintic prefix
    on [0, dt] + brake branch from the eval(dt) state. Returns (seg_coeffs list, durs list,
    t_cert). Brake dur widened until peak |a| <= a_max (soundness: we certify the actual segment)."""
    p_dt = _poly_eval(prim, dt, 0)[0]
    v_dt = _poly_eval(prim, dt, 1)[0]
    a_dt = _poly_eval(prim, dt, 2)[0]
    segs = [prim]
    durs = [dt]
    bc, bdur = brake_segment(p_dt, v_dt, a_dt, a_max)
    vmag = float(np.linalg.norm(v_dt[:2]))
    vdir = v_dt / max(np.linalg.norm(v_dt), 1e-9)
    p_stop = p_dt + vdir * (vmag * bdur * 0.5)
    for _ in range(10):                                 # widen (ACCUMULATING) until accel AND jerk fit
        _, pa, pj = _sampled_deriv_bounds(bc, bdur)     # quintic stop over a short window has high
        if pa <= a_max + 1e-6 and pj <= J_BRK + 1e-6:   # jerk -> widen on BOTH (bang-bang would be
            break                                       # tighter; the widened quintic is sound+feasible)
        bdur *= 1.3
        p_stop = p_dt + vdir * (vmag * bdur * 0.5)
        bc = quintic3(p_dt, v_dt, a_dt, p_stop, np.zeros(3), np.zeros(3), bdur)
    segs.append(bc)
    durs.append(bdur)
    # POST-STOP HOLD covering the SLACK: recursive feasibility requires that HOLDING at p_stop is
    # ALSO certified over the window (the frozen keep-out tube keeps growing R+v_eff*(t+delta), so a
    # stop near an obstacle is only sound if the hold is checked too). Constant deg-5 segment.
    hold = np.zeros((6, 3)); hold[0] = p_stop
    segs.append(hold)
    durs.append(SLACK)
    t_cert = dt + bdur + SLACK                          # == sum(durs): window covers the whole traj
    return segs, durs, t_cert


# ---- escape branches (V3_ESC pre-certified contingency tree) ----------------------------------
# The single straight brake makes recursive feasibility demand "can stop IN PLACE clear of the
# keep-out" -- in head-on geometry the stop line sits inside the mover tube and every candidate
# dies with it (diag_v3: ped inside ~1.75m refutes even hover). The escape tree weakens that to
# "SOME certified transition-to-rest exists": straight brake first, then veer-brakes whose stopping
# displacement is rotated off the velocity direction (moving case) or sidestep hops away from the
# refuting mover (rest case). Any one certifying admits the candidate; the certified escape IS the
# fallback branch, so the flown prefix + escape stays a fully certified composite (same guarantee
# story as the straight brake, K directions instead of 1).

def _rot_xy(d, ang):
    """Rotate the xy part of displacement d by ang (rad), keep z."""
    c, s = np.cos(ang), np.sin(ang)
    return np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])


def _widen_stop(p, v, a, a_max, target_fn, dur0):
    """Quintic to rest at target_fn(dur), duration widened until peak |a|<=a_max and jerk<=J_BRK
    (same soundness discipline as make_composite: we certify the actual widened segment)."""
    dur = dur0
    bc = quintic3(p, v, a, target_fn(dur), np.zeros(3), np.zeros(3), dur)
    for _ in range(10):
        _, pa, pj = _sampled_deriv_bounds(bc, dur)
        if pa <= a_max + 1e-6 and pj <= J_BRK + 1e-6:
            break
        dur *= 1.3
        bc = quintic3(p, v, a, target_fn(dur), np.zeros(3), np.zeros(3), dur)
    return bc, dur


def make_composite_esc(prim, a_max, dt=DT, esc_dir=None, hop=1.5):
    """Escape variant of make_composite: prefix [0,dt] + a certified dodge-to-rest branch + hold.
    The escape targets rest at p_dt + esc_dir*hop (esc_dir unit 3-vector, z kept level), duration
    starting from the kinematic minimum and widened until |a|<=a_max, jerk<=J_BRK. The quintic
    boundary solve absorbs the initial (v,a), so the same construction covers veer-brake (moving)
    and sidestep-hop (rest)."""
    p_dt = _poly_eval(prim, dt, 0)[0]
    v_dt = _poly_eval(prim, dt, 1)[0]
    a_dt = _poly_eval(prim, dt, 2)[0]
    vmag = float(np.linalg.norm(v_dt[:2]))
    tgt = p_dt + np.asarray(esc_dir, float) * hop
    dur0 = max(2.0 * np.sqrt(hop / a_max), t_brake(vmag, a_max)) * 1.05
    bc, bdur = _widen_stop(p_dt, v_dt, a_dt, a_max, lambda d: tgt, dur0)
    p_stop = _poly_eval(bc, bdur, 0)[0]
    hold = np.zeros((6, 3)); hold[0] = p_stop
    return [prim, bc, hold], [dt, bdur, SLACK], dt + bdur + SLACK


def _esc_dirs(angles, prim, dt, cyl, who):
    """Escape directions: flee direction (away from the refuter) rotated by 0, +/-angles; the sign
    nearer the current velocity's off-side is tried first. Unit 3-vectors, level z."""
    p_dt = _poly_eval(prim, dt, 0)[0]
    v_dt = _poly_eval(prim, dt, 1)[0]
    if 0 <= who < len(cyl):
        away = p_dt[:2] - np.asarray(cyl[who][0], float)[:2]
    else:
        away = -v_dt[:2] if np.linalg.norm(v_dt[:2]) > 1e-6 else np.array([1.0, 0.0])
    away = away / max(np.linalg.norm(away), 1e-9)
    side = 1.0
    if np.linalg.norm(v_dt[:2]) > 1e-6:
        vd = v_dt[:2] / np.linalg.norm(v_dt[:2])
        side = -1.0 if (vd[0] * away[1] - vd[1] * away[0]) > 0 else 1.0
    h = np.array([away[0], away[1], 0.0])
    dirs = [h]
    for a in angles:
        if a > 0:
            dirs.extend([_rot_xy(h, np.radians(side * a)), _rot_xy(h, np.radians(-side * a))])
    return dirs


def try_escapes(prim, a_max, dt, cyl, delta, who, angles, hops, v_max):
    """Escape tree for one refuted candidate. The prefix must certify on its own (a candidate whose
    flown segment is already inside a keep-out is dead -- no branch can save it); then dodge-to-rest
    branches over a direction x distance grid, first certified composite wins. Returns
    (segs, durs, t_cert, margin, esc_tag) or None."""
    pok, _ = certify_composite([prim], [dt], cyl, dt, delta)
    if not pok:
        return None
    dirs = _esc_dirs(angles, prim, dt, cyl, who)
    for hop in hops:
        for i, d3 in enumerate(dirs):
            segs, durs, t_cert = make_composite_esc(prim, a_max, dt, esc_dir=d3, hop=hop)
            if not feasible(segs, durs, v_max, a_max):
                continue
            ok, m = certify_composite(segs, durs, cyl, t_cert, delta)
            if ok:
                return segs, durs, t_cert, m, f"e{i}@{hop:.1f}"
    return None


def escape_fallback(p, v, a, cyl, a_max, dt=DT, delta=DT, angles=(45.0, 90.0), hops=(1.5, 3.0),
                    v_max=3.0):
    """L1 fallback escape tree: the caller already tried (and failed) the straight brake from the
    current state; try the dodge-to-rest escape grid. Returns a certified plan or None."""
    prim = quintic3(p, v, a, p, v * 0.0, np.zeros(3), T_P)
    r = try_escapes(prim, a_max, dt, cyl, delta, _nearest_cyl(p, cyl), angles, hops, v_max)
    if r is None:
        return None
    segs, durs, t_cert, _, _ = r
    return (segs, durs, t_cert)


def precertify_branch(p, v, a, cyl, a_max, dt=DT, delta=DT, angles=(45.0, 90.0), hops=(1.5, 3.0),
                      v_max=3.0):
    """Pre-certify a contingency branch that STARTS one tick ahead (v2 escape tree): (p,v,a) is the
    predicted commit-end state; obstacles are advanced by dt and the keep-out staleness by dt
    (c(t)=c0+vel*t at absolute t equals (c0+vel*dt)+vel*t' at branch time t', and R+v_eff*(t+delta)
    equals R+v_eff*(t'+(delta+dt)) -- exact time-frame shift, no new assumptions). Straight
    decelerate-to-rest first, then the dodge grid. Returns (segs, durs, t_cert) or None."""
    cyl_adv = [(np.asarray(c0, float) + np.asarray(vel, float) * dt + 0.5 * np.asarray(acc, float) * dt * dt,
                np.asarray(vel, float) + np.asarray(acc, float) * dt, acc, R, zc, veff)
               for (c0, vel, acc, R, zc, veff) in cyl]
    d_eff = delta + dt
    prim = quintic3(p, v, a, p, v * 0.0, np.zeros(3), T_P)
    segs, durs, tc = make_composite(prim, a_max, dt)
    ok, _m, who = certify_composite(segs, durs, cyl_adv, tc, d_eff, want_who=True)
    if ok:
        return (segs, durs, tc)
    r = try_escapes(prim, a_max, dt, cyl_adv, d_eff, who, angles, hops, v_max)
    if r is None:
        return None
    segs, durs, tc, _, _ = r
    return (segs, durs, tc)


def _nearest_cyl(p, cyl):
    """Index of the planar-nearest cylinder (refuter proxy for the fallback escape order)."""
    if not cyl:
        return -1
    p = np.asarray(p, float)
    d = [float(np.linalg.norm(np.asarray(c[0], float)[:2] - p[:2])) for c in cyl]
    return int(np.argmin(d))


def _bernstein_deriv_bounds(coeffs, dur):
    """Max |v|, |a|, |j| over a segment via Bernstein control-point bounds on the derivatives
    (control points bound the polynomial on [0,dur]). Returns (vmax, amax, jmax) planar."""
    ctrl, _, _ = monomial_to_bseg(coeffs[None, ...], np.array([dur]))
    c = ctrl[0]                                          # (6,3) Bernstein pts on normalized s
    deg = c.shape[0] - 1
    # velocity control points (deg-1): deg*(c_{i+1}-c_i)/dur
    v = deg * (c[1:] - c[:-1]) / dur
    a = (deg - 1) * (v[1:] - v[:-1]) / dur
    j = (deg - 2) * (a[1:] - a[:-1]) / dur
    pl = lambda x: float(np.max(np.linalg.norm(x[:, :2], axis=1))) if len(x) else 0.0
    return pl(v), pl(a), pl(j)


def _sampled_deriv_bounds(coeffs, dur, n=48):
    """Exact-enough max |v|,|a|,|j| by dense sampling of the derivative polynomials (planar).
    Feasibility is an EXECUTABILITY filter, not a safety gate, so the tight sampled max is used
    instead of the conservative Bernstein control-point bound (which over-rejected)."""
    ts = np.linspace(0, dur, n)
    v = _poly_eval(coeffs, ts, 1)
    a = _poly_eval(coeffs, ts, 2)
    j = _poly_eval(coeffs, ts, 3)
    pl = lambda x: float(np.max(np.linalg.norm(x[:, :2], axis=1)))
    return pl(v), pl(a), pl(j)


def _peak_accel(coeffs, dur):
    return _sampled_deriv_bounds(coeffs, dur)[1]


def feasible(segs, durs, v_max, a_max, j_c=J_C, j_brk=J_BRK):
    """Kinematic feasibility: commit prefix (seg 0) under comfort jerk; brake segs under J_BRK."""
    v, a, j = _sampled_deriv_bounds(segs[0], durs[0])
    if v > v_max + 1e-3 or a > a_max + 1e-3 or j > j_c + 1e-3:
        return False
    for c, d in zip(segs[1:], durs[1:]):
        v, a, j = _sampled_deriv_bounds(c, d)
        if a > a_max + 1e-3 or j > j_brk + 1e-3:
            return False
    return True


# ---- certification (full disjunction, matches cert_clear semantics) ---------------------------

def certify_composite(segs, durs, cyl, t_cert, delta, tau=None, want_who=False):
    """(ok, margin): the cylinder disjunction (hp AND hc) OR vo on the composite BSeg, AND over
    movers. cyl entries: (c0, vel, acc, R, zc, v_eff). tau defaults to t_cert (whole window).
    want_who=True additionally returns the index of the first refuting cylinder (-1 if none)."""
    coeffs = np.stack(segs, axis=0)                     # (n_seg, 6, 3)
    ctrl, t0s, du = monomial_to_bseg(coeffs, np.asarray(durs, float))
    th = t_cert if tau is None else tau
    m_min = np.inf
    for i, (c0, vel, acc, R, zc, veff) in enumerate(cyl):
        hp, mp = Certifier.certify_horizontal(ctrl, t0s, du, c0, R, vel=vel, acc=acc,
                                              t_hi=th, v_eff=veff, delta=delta, n_axes=2)
        hc, mc = Certifier.certify_horizontal(ctrl, t0s, du, c0, R, vel=(0, 0, 0),
                                              t_hi=th, v_eff=veff, delta=delta, n_axes=2)
        vo, mv = Certifier.certify_above(ctrl, t0s, du, z_clear=zc, t_hi=th, v_eff_z=0.0, delta=delta)
        if not ((hp and hc) or vo):
            return (False, -1.0, i) if want_who else (False, -1.0)
        m_min = min(m_min, max(min(mp, mc), mv) / max(2.0 * R, 1e-6))
    m_out = m_min if np.isfinite(m_min) else np.inf
    return (True, m_out, -1) if want_who else (True, m_out)


# ---- structured candidate lattice + local planner (blueprint 2-3) -----------------------------

PSI_GRID = np.radians([0, 15, -15, 30, -30, 50, -50, 75, -75, 105, -105, 135, -135])
DV_GRID = (1.2, 0.6, 0.0, -0.9, -1.8)
Z_CRUISE = 1.0


def candidate_primitives(p, v, a, goal, ztop, v_max, incumbent=None, guide=None):
    """Structured reachable-set lattice of intent quintics (blueprint 2). Terminal velocity from a
    RELATIVE speed grid, heading from the psi grid off the goal direction; terminals derived from
    the velocity profile (trapezoidal displacement) so quintics stay smooth. Verticals: cruise /
    soar (forward+up) / climb (up in place). Privileged: incumbent tail, EGO guide. Returns list of
    (coeffs (6,3), tag)."""
    p = np.asarray(p, float); v = np.asarray(v, float); a = np.asarray(a, float)
    g = np.asarray(goal, float)[:2] - p[:2]
    gdir = g / max(np.linalg.norm(g), 1e-6)
    gpsi = float(np.arctan2(gdir[1], gdir[0]))
    sp0 = float(np.linalg.norm(v[:2]))
    L = min(T_P, max(np.linalg.norm(g), 1.0) / max(sp0, 1.0))
    out = []
    # z-grid (blueprint + user 2026-07-08: a DRONE should use altitude -- climb over movers when the
    # horizontal is blocked; the certificate's above-branch keeps it sound). Forward headings get a
    # cruise-altitude grid so "fly higher and continue" is a first-class candidate, not a last resort.
    import os as _os
    z_grid = [Z_CRUISE, 2.5, 4.0] if _os.environ.get("V3_ZGRID", "1") == "1" else [Z_CRUISE]
    z_cap = float(_os.environ.get("V3_ZMAX", str(max(ztop, 5.0))))
    for dpsi in PSI_GRID:
        psi = gpsi + dpsi
        u = np.array([np.cos(psi), np.sin(psi), 0.0])
        wide = abs(dpsi) > np.radians(50)               # only near-forward headings get the z-grid
        for dv in DV_GRID:
            vT = float(np.clip(sp0 + dv, 0.0, v_max))
            if vT < 0.05 and dv != DV_GRID[-1]:
                continue
            vTv = u * vT
            for zc in ([Z_CRUISE] if wide else z_grid):
                zc = min(zc, z_cap)
                pT = p + 0.5 * (v + vTv) * T_P
                pT[2] = zc
                out.append((quintic3(p, v, a, pT, vTv, np.zeros(3), T_P),
                            f"g{int(np.degrees(dpsi))}_{vT:.1f}_z{zc:.0f}"))
    # hover (stop in place)
    out.append((quintic3(p, v, a, p, np.zeros(3), np.zeros(3), T_P), "hover"))
    # soar: forward + climb ; climb: up in place
    fwd = p + np.array([gdir[0], gdir[1], 0.0]) * (0.6 * L); fwd[2] = ztop
    out.append((quintic3(p, v, a, fwd, np.array([gdir[0], gdir[1], 0.0]) * min(sp0, v_max), np.zeros(3), T_P), "soar"))
    up = p.copy(); up[2] = ztop
    out.append((quintic3(p, v, a, up, np.zeros(3), np.zeros(3), T_P), "climb"))
    if incumbent is not None:
        out.append((incumbent, "incumbent"))           # warm-start: last tick's winning intent
    if guide is not None:
        out.append((guide, "ego"))                     # EGO global guide (escapes local minima)
    return out


def plan_local(p, v, a, goal, ztop, cyl, v_max=3.0, a_max=6.0, dt=DT, delta=DT,
               incumbent=None, guide=None, band=0.5, bucket=0.15, w_smooth=(0.4, 0.4, 0.2),
               tau_speed=True):
    """CPL-v3 local planner: build lattice -> composite -> feasibility -> certify -> lexicographic
    score (progress bucket, clearance BAND, smoothness) -> pick. Returns (plan, kind, diag).
    plan = (segs, durs, t_cert); None kind -> caller runs the fallback chain."""
    import os as _os
    bucket = float(_os.environ.get("V3_BUCKET", str(bucket)))   # coarser bucket -> similar-progress
    band = float(_os.environ.get("V3_BAND", str(band)))          # candidates tie on progress, clearance
    #   (excess, capped at band) breaks the tie -> the drone detours around a mover instead of greedily
    #   creeping straight into it and then failing to certify a stop (evade). Tuning lever, reach gap.
    g = np.asarray(goal, float)[:2] - np.asarray(p, float)[:2]
    gdir = g / max(np.linalg.norm(g), 1e-6)
    esc_on = _os.environ.get("V3_ESC", "0") == "1"      # pre-certified contingency tree (escape set)
    esc_angs = tuple(float(x) for x in _os.environ.get("V3_ESC_ANG", "45,90").split(","))
    esc_hops = tuple(float(x) for x in _os.environ.get("V3_ESC_HOP", "1.5,3.0").split(","))
    cands = candidate_primitives(p, v, a, goal, ztop, v_max, incumbent, guide)
    scored = []
    n_feas = n_cert = n_esc = 0
    refuted = []

    def _admit(prim, tag, segs, durs, t_cert, m):
        # progress = the INTENT primitive's goal-ward reach (where this plan WANTS to go), NOT the
        # braked composite endpoint (all composites stop, so that can't tell forward from hover).
        # We only fly the certified commit; the intent expresses sustained navigation preference.
        p_int = _poly_eval(prim, T_P, 0)[0]
        prog = float((p_int[:2] - np.asarray(p, float)[:2]) @ gdir)
        excess = float(np.clip(m if np.isfinite(m) else band, 0, band))
        vend = _poly_eval(prim, T_P, 1)[0]
        vT = float(np.linalg.norm(vend[:2]))
        dpsi = abs(float(np.arctan2(vend[1], vend[0]) - np.arctan2(gdir[1], gdir[0])))
        dpsi = min(dpsi, 2 * np.pi - dpsi)
        smooth = -(w_smooth[0] * dpsi / (np.pi / 2) + w_smooth[1] * abs(vT - float(np.linalg.norm(v[:2]))) / v_max)
        key = (round(prog / bucket), excess, smooth)
        scored.append((key, (segs, durs, t_cert), prim, tag))

    for prim, tag in cands:
        segs, durs, t_cert = make_composite(prim, a_max, dt)
        if not feasible(segs, durs, v_max, a_max):
            continue
        n_feas += 1
        th = min(t_cert, 0.30 + 0.5 * float(np.linalg.norm(_poly_eval(prim, dt, 1)[0][:2])) + 0.05) \
            if tau_speed else t_cert
        if esc_on:
            ok, m, who = certify_composite(segs, durs, cyl, t_cert, delta, tau=None, want_who=True)
            if not ok:
                refuted.append((prim, tag, who))
                continue
        else:
            ok, m = certify_composite(segs, durs, cyl, t_cert, delta, tau=None)
            if not ok:
                continue
        n_cert += 1
        _admit(prim, tag, segs, durs, t_cert, m)
    if esc_on and refuted:
        # escape tree pass: straight-brake-refuted candidates get dodge-to-rest branches, best
        # intent progress first (that is exactly where the recursive-feasibility tax bites), budget
        # capped; hover is always retried (it is the last stand before the fallback chain)
        p2 = np.asarray(p, float)[:2]
        refuted.sort(key=lambda r: float((_poly_eval(r[0], T_P, 0)[0][:2] - p2) @ gdir), reverse=True)
        topk = int(_os.environ.get("V3_ESC_TOPK", "12"))
        pool = refuted[:topk] + [r for r in refuted[topk:] if r[1] == "hover"]
        for prim, tag, who in pool:
            r = try_escapes(prim, a_max, dt, cyl, delta, who, esc_angs, esc_hops, v_max)
            if r is None:
                continue
            segs, durs, t_cert, m, _etag = r
            n_cert += 1
            n_esc += 1
            _admit(prim, tag, segs, durs, t_cert, m)
    diag = dict(n_cand=len(cands), n_feas=n_feas, n_cert=n_cert)
    if esc_on:
        diag["n_esc"] = n_esc                            # candidates admitted via an escape branch
    if not scored:
        return None, None, None, diag
    scored.sort(key=lambda x: x[0], reverse=True)
    _, plan, prim_win, tag = scored[0]
    return plan, prim_win, tag, diag


def composite_eval(segs, durs, t):
    """Eval the composite position at wall time t (piecewise). For the executor: usually t=DT."""
    t0 = 0.0
    for c, d in zip(segs, durs):
        if t <= t0 + d or (c is segs[-1]):
            return _poly_eval(c, min(t - t0, d), 0)[0]
        t0 += d
    return _poly_eval(segs[-1], durs[-1], 0)[0]


def plan_eval(plan, t):
    """(pos, vel, acc) of the plan composite at wall time t -- the executor set-point at t=DT."""
    segs, durs, _ = plan
    t0 = 0.0
    for c, d in zip(segs, durs):
        if t <= t0 + d or (c is segs[-1]):
            lt = min(t - t0, d)
            return (_poly_eval(c, lt, 0)[0], _poly_eval(c, lt, 1)[0], _poly_eval(c, lt, 2)[0])
        t0 += d
    return (_poly_eval(segs[-1], durs[-1], 0)[0], np.zeros(3), np.zeros(3))
