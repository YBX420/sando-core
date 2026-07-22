"""predictive_guide — the north-star guide-path generator (2026-07-22 ruling).

THE missing algorithm the 07-22 external diagnosis named: KF prediction must become ONE stable
reference line handed to EGO ("plan along this"), not walls + sub-goal carrots + a maneuver
tournament. This module is that algorithm, standalone and renderer-free:

  input : drone state, the ORIGINAL route direction (base line), KF movers with their CERT-scale
          keep-out radii, a natural-ETA map (when will the drone reach arc s, from the CURRENT
          committed trajectory -- never a nominal cruise guess), cross-tick state.
  output: Pguide -- a polyline that deviates from the base line with MINIMAL lateral deformation,
          minimal curvature (cosine bumps), and bounded per-tick change; plus a meta dict naming
          every conflict (oid, s*, t*, side, offset) -- the explain artifact.

Laws (from the ruling):
  * one conflict event keeps ONE side across ticks (sticky; no 0.1-s side flipping);
  * per-tick profile change is rate-limited (GUIDE_SLEW) -- the guide is a commitment, not a twitch;
  * ETA is used ONLY to place conflicts on the line; it never steers speed;
  * pass-BEHIND preference: dodge toward where the mover came from (its wake), the human road-
    crossing move;
  * the guide NEVER claims safety -- the certificate stays the only judge; radii here are the
    cert keep-outs plus a small band so the plan the guide produces certifies comfortably
    (plan-once-certify-once; the plan-small/cert-big mismatch was the old kill->hold chain).
"""
import os

import numpy as np


class GuideCfg:
    def __init__(self):
        e = os.environ.get
        self.ds = float(e("GUIDE_DS", "0.4"))            # conflict-scan arc step (m)
        self.out_ds = float(e("GUIDE_OUT_DS", "0.5"))    # emitted polyline spacing (m)
        self.omax = float(e("GUIDE_OMAX", "4.0"))        # lateral deformation cap (m) -- roomy by
        #   default: the cap must not be the binding constraint while chasing 0-hold (the
        #   tournament's wide arcs reached ~4m lateral); off_max is reported every tick
        self.slew = float(e("GUIDE_SLEW", "0.45"))       # max per-tick profile change (m)
        self.ramp_min = float(e("GUIDE_RAMP", "2.0"))    # min bump ramp length (m)
        self.vref_floor = float(e("GUIDE_VREF", "1.2"))  # ETA fallback speed floor (m/s)
        self.side_vmin = 0.3                             # |lateral mover vel| that defines a crossing
        self.side_ttl = int(e("GUIDE_SIDE_TTL", "6"))    # ticks a sticky side survives without conflict
        self.o_step = 0.1                                # offset search resolution (m)
        self.verify_pad = 0.05                           # slack accepted in the verify pass (m)


def _bump(S, s0, s1, ramp):
    """Smooth 0->1->0 cosine bump supported on [s0-ramp, s1+ramp], flat 1 on [s0, s1]."""
    w = np.zeros_like(S)
    up = (S >= s0 - ramp) & (S < s0)
    w[up] = 0.5 * (1.0 - np.cos(np.pi * (S[up] - (s0 - ramp)) / ramp))
    w[(S >= s0) & (S <= s1)] = 1.0
    dn = (S > s1) & (S <= s1 + ramp)
    w[dn] = 0.5 * (1.0 + np.cos(np.pi * (S[dn] - s1) / ramp))
    return w


def build_guide(p_d, v_d, goal_xy, movers, state, cruise_z=1.5, eta=None, cfg=None,
                tw=0.75, delta=0.1):
    """-> (pts (N,3) float array, meta dict).
    movers: [(oid, c0_xy(2,), v_xy(2,), R0, veff[, t_max])] -- R0 at CERT scale (+band); veff
            grows the keep-out as the SLIDING-WINDOW MAX the certificate will ever apply to this
            conflict: R(s) = R0 + veff*(min(tau(s), tw) + delta)  (the M3 grid law -- a guide that
            ignores the growth plans into a tube the cert then kills = the acid-1 hold storm).
            t_max (optional): conflict only binds while tau(s) <= t_max (frozen-conjunct twins).
    state:  caller-persisted dict across ticks (sides / previous profile); {} to start fresh.
    eta:    callable s -> seconds (natural ETA along the CURRENT committed trajectory);
            None -> s / max(|v_d|, vref_floor)."""
    cfg = cfg or GuideCfg()
    p = np.asarray(p_d, float)[:2]
    g = np.asarray(goal_xy, float)[:2]
    L = float(np.linalg.norm(g - p))
    meta = dict(conflicts=[], infeasible=False, off_max=0.0)
    if L < 1e-6:
        return np.array([[p[0], p[1], cruise_z]]), meta
    u = (g - p) / L                                   # along-track unit
    n = np.array([-u[1], u[0]])                       # left normal
    S = np.arange(0.0, L + cfg.ds, cfg.ds)
    if eta is None:
        vref = max(float(np.hypot(*np.asarray(v_d, float)[:2])), cfg.vref_floor)
        tau = S / vref
    else:
        tau = np.asarray([float(eta(float(s))) for s in S], float)
    base = p[None, :] + S[:, None] * u[None, :]

    sides = state.setdefault("side", {})
    side_age = state.setdefault("side_age", {})
    events = []
    for row in movers:
        (oid, c0, mv, R0), veff = row[:4], (float(row[4]) if len(row) > 4 else 0.0)
        t_max = float(row[5]) if len(row) > 5 and row[5] is not None else None
        c0 = np.asarray(c0, float)[:2]
        mv = np.asarray(mv, float)[:2]
        cpos = c0[None, :] + tau[:, None] * mv[None, :]        # mover centre when the DRONE is at s
        R = R0 + veff * (np.minimum(tau, tw) + delta)          # sliding-window MAX tube (per-s array)
        d = np.linalg.norm(base - cpos, axis=1)
        hit = d < R
        if t_max is not None:
            hit &= (tau <= t_max)
        if not hit.any():
            continue
        idx = np.where(hit)[0]
        s0, s1 = float(S[idx[0]]), float(S[idx[-1]])
        k_star = int(idx[np.argmin((d - R)[idx])])
        s_star, t_star = float(S[k_star]), float(tau[k_star])
        side = sides.get(oid)
        if side is None:
            vn = float(mv @ n)
            if abs(vn) > cfg.side_vmin:
                side = -1.0 if vn > 0 else 1.0          # pass BEHIND: toward where it came from
            else:
                e = float((cpos[k_star] - base[k_star]) @ n)
                side = -1.0 if e > 0 else 1.0           # slow/static-ish: away from the body
        sides[oid] = side
        side_age[oid] = 0
        # smallest lateral offset that clears this mover over its conflict window
        def _o_need(sd):
            for o in np.arange(cfg.o_step, cfg.omax + 1e-9, cfg.o_step):
                gpt = base[idx] + (sd * o) * n[None, :]
                if float(np.min(np.linalg.norm(gpt - cpos[idx], axis=1) - R[idx])) >= 0.0:
                    return float(o)
            return cfg.omax
        o_need = _o_need(side)
        if o_need >= cfg.omax - 1e-9:
            # ESCAPE HATCH (gtxy 0-hold campaign): the committed side is CAPPED-infeasible --
            # commitment must not ride a dead side into a hold; try the other side, switch if it
            # actually clears. (The only sanctioned side switch: feasibility beats stickiness.)
            o_alt = _o_need(-side)
            if o_alt < cfg.omax - 1e-9:
                side = -side
                sides[oid] = side
                o_need = o_alt
        ramp = max(cfg.ramp_min, 0.5 * (s1 - s0))
        events.append(dict(oid=oid, s0=s0, s1=s1, s=s_star, t=t_star, side=float(side),
                           o=o_need, ramp=ramp, R=R, c0=c0, v=mv, idx=idx))
    for oid in list(side_age):
        if not any(ev["oid"] == oid for ev in events):
            side_age[oid] += 1
            if side_age[oid] > cfg.side_ttl:
                side_age.pop(oid, None)
                sides.pop(oid, None)
    off = np.zeros_like(S)
    if events:
        for _tries in range(3):
            oL = np.zeros_like(S)
            oR = np.zeros_like(S)
            for ev in events:
                b = _bump(S, ev["s0"], ev["s1"], ev["ramp"]) * ev["o"]
                if ev["side"] > 0:
                    oL = np.maximum(oL, b)
                else:
                    oR = np.maximum(oR, b)
            off = np.where(oL >= oR, oL, -oR)           # squeeze: the DOMINANT side wins locally
            bad = False
            for ev in events:
                m = ev["idx"]
                gpt = base[m] + off[m, None] * n[None, :]
                cpos = ev["c0"][None, :] + tau[m][:, None] * ev["v"][None, :]
                if float(np.min(np.linalg.norm(gpt - cpos, axis=1) - ev["R"][m])) < -cfg.verify_pad:
                    ev["o"] = min(ev["o"] + 0.25, cfg.omax)
                    bad = True
            if not bad:
                break
        else:
            meta["infeasible"] = True
        if bad:
            meta["infeasible"] = True
    # cross-tick commitment: rate-limit the profile against the previous tick's (resampled)
    prev = state.get("prof")
    if prev is not None:
        S_p, off_p = prev
        off_prev = np.interp(S, S_p, off_p, left=off_p[0] if len(off_p) else 0.0, right=0.0)
        off = np.clip(off, off_prev - cfg.slew, off_prev + cfg.slew)
    state["prof"] = (S.copy(), off.copy())
    meta["off_max"] = float(np.max(np.abs(off))) if len(off) else 0.0
    meta["conflicts"] = [dict(oid=ev["oid"], s=round(ev["s"], 2), t=round(ev["t"], 2),
                              side=("L" if ev["side"] > 0 else "R"), o=round(ev["o"], 2))
                         for ev in events]
    step = max(1, int(round(cfg.out_ds / cfg.ds)))
    ks = list(range(0, len(S), step))
    if ks[-1] != len(S) - 1:
        ks.append(len(S) - 1)
    pts = np.array([[base[k][0] + off[k] * n[0], base[k][1] + off[k] * n[1], cruise_z] for k in ks])
    return pts, meta


if __name__ == "__main__":
    # 1) empty world -> dead straight, zero offset
    st = {}
    pts, meta = build_guide([0, 0, 1.5], [1, 0, 0], [10, 0], [], st)
    assert meta["off_max"] == 0.0 and abs(pts[:, 1]).max() < 1e-9
    print("[guide] empty world: straight OK")

    # 2) crosser from the LEFT walking RIGHT, timed to meet mid-line -> bump LEFT (pass behind)
    st = {}
    mv = [("ped1", (5.0, 2.4), (0.0, -0.8), 1.2)]      # reaches y=0 at t=3.0 = drone ETA at s=5 @1.67m/s
    pts, meta = build_guide([0, 0, 1.5], [1.67, 0, 0], [10, 0], mv, st)
    c = meta["conflicts"]
    assert len(c) == 1 and c[0]["side"] == "L" and meta["off_max"] > 0.4, (c, meta)
    ymax = pts[:, 1].max()
    assert ymax > 0.3 and pts[:, 1].min() > -1e-9, "bump must be on +y (behind the walker) only"
    print(f"[guide] crossing walker: bump L apex {ymax:.2f}m at s~{c[0]['s']} t~{c[0]['t']} OK")

    # 3) sticky side: next tick the walker has advanced (naive side would flip) -- side must HOLD
    mv2 = [("ped1", (5.0, 1.6), (0.0, -0.8), 1.2)]
    pts2, meta2 = build_guide([0.17, 0, 1.5], [1.67, 0, 0], [10, 0], mv2, st)
    assert meta2["conflicts"] and meta2["conflicts"][0]["side"] == "L", meta2
    print("[guide] sticky side across ticks OK")

    # 4) rate limit: a fresh huge conflict cannot yank the profile more than slew per tick
    st4 = {}
    build_guide([0, 0, 1.5], [1.67, 0, 0], [10, 0], [], st4)                    # tick 1: straight
    _, m4 = build_guide([0, 0, 1.5], [1.67, 0, 0], [10, 0],
                        [("van", (5.0, 0.0), (0.0, 0.0), 2.2)], st4)            # tick 2: wall appears
    assert m4["off_max"] <= GuideCfg().slew + 1e-9, m4["off_max"]
    print(f"[guide] rate limit: first-tick offset {m4['off_max']:.2f} <= slew OK")

    # 5) squeeze (two movers, opposite sides demanded) -> single dominant side, no cancel-through-middle
    st5 = {}
    mv5 = [("a", (5.0, 1.2), (0.0, 0.0), 1.4), ("b", (5.0, -1.2), (0.0, 0.0), 1.4)]
    pts5, m5 = build_guide([0, 0, 1.5], [1.67, 0, 0], [10, 0], mv5, st5)
    prof = pts5[:, 1]
    assert (prof.max() > 0.2) != (prof.min() < -0.2), "must commit to ONE side, not thread the middle"
    print("[guide] squeeze -> dominant side OK")
    print("[guide] ALL PASS")
