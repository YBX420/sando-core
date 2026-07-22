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

gtxy 0-hold campaign amendments (this file's second day):
  * SIDE KEY strips any '#...' suffix: a mover's frozen-conjunct twin must share its parent's
    sticky side -- independent away-from-body assignment could demand the OPPOSITE side of the
    same physical object (s4 forensic: pred R o=3.6 vs #frz L o=1.6 at the same station = an
    impossible profile the gates then killed at speed).
  * CLUSTER one-side law: conflicts whose bump supports overlap are resolved to ONE side
    (min total offset, capped events penalised; a singleton keeps its sticky side unless capped
    and the other side clears -- feasibility beats stickiness, same hatch as before). The old
    per-station max(oL,oR) merge flipped sign DISCONTINUOUSLY inside overlaps (s7 forensic).
  * SCAN EXTENSION: conflicts are detected on a base line extended past the carrot by up to
    scan_t seconds of current speed (cap scan_ext_max) so a fast approach opens its profile
    EARLY (slew needs ~8 ticks for a 3.5 m bump); emitted points still span [0, L] only.
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
        self.scan_t = float(e("GUIDE_SCAN_T", "2.5"))    # conflict-scan horizon (s of current speed)
        self.scan_ext_max = float(e("GUIDE_SCAN_EXT", "8.0"))  # cap on the beyond-carrot extension (m)


def _bump(S, s0, s1, ramp):
    """Smooth 0->1->0 cosine bump supported on [s0-ramp, s1+ramp], flat 1 on [s0, s1]."""
    w = np.zeros_like(S)
    up = (S >= s0 - ramp) & (S < s0)
    w[up] = 0.5 * (1.0 - np.cos(np.pi * (S[up] - (s0 - ramp)) / ramp))
    w[(S >= s0) & (S <= s1)] = 1.0
    dn = (S > s1) & (S <= s1 + ramp)
    w[dn] = 0.5 * (1.0 + np.cos(np.pi * (S[dn] - s1) / ramp))
    return w


def _side_key(oid):
    """Sticky-side key: the twin '#frz' rows share their parent's side commitment."""
    return oid.split("#")[0] if isinstance(oid, str) else oid


def build_guide(p_d, v_d, goal_xy, movers, state, cruise_z=1.5, eta=None, cfg=None,
                tw=0.75, delta=0.1):
    """-> (pts (N,3) float array, meta dict).
    movers: [(oid, c0_xy(2,), v_xy(2,), R0, veff[, t_max])] -- R0 at CERT scale (+band); veff
            grows the keep-out as the SLIDING-WINDOW MAX the certificate will ever apply to this
            conflict: R(s) = R0 + veff*(min(tau(s), tw) + delta)  (the M3 grid law -- a guide that
            ignores the growth plans into a tube the cert then kills = the acid-1 hold storm).
            t_max (optional): conflict only binds while tau(s) <= t_max (frozen-conjunct twins).
            v=0, veff=0 rows are STATICS (gtxy F1): pure geometry, no growth, no twin.
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
    v0 = float(np.hypot(*np.asarray(v_d, float)[:2]))
    L_scan = L + max(0.0, min(cfg.scan_ext_max, v0 * cfg.scan_t - L))
    S = np.arange(0.0, L_scan + cfg.ds, cfg.ds)
    if eta is None:
        vref = max(v0, cfg.vref_floor)
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
        skey = _side_key(oid)
        side = sides.get(skey)
        if side is None:
            vn = float(mv @ n)
            if abs(vn) > cfg.side_vmin:
                side = -1.0 if vn > 0 else 1.0          # pass BEHIND: toward where it came from
            else:
                e = float((cpos[k_star] - base[k_star]) @ n)
                side = -1.0 if e > 0 else 1.0           # slow/static-ish: away from the body
            sides[skey] = side
        side_age[skey] = 0
        ramp = max(cfg.ramp_min, 0.5 * (s1 - s0))
        events.append(dict(oid=oid, skey=skey, s0=s0, s1=s1, s=s_star, t=t_star,
                           side=float(side), o=None, ramp=ramp, R=R, c0=c0, v=mv, idx=idx))

    def _o_need(ev, sd, t_ahead=0.0):
        """Smallest lateral offset on side sd clearing ev over its conflict window (omax = capped).
        t_ahead > 0 evaluates the mover advanced by that many seconds -- the near-future demand.
        (A speed-scaled static margin REQUIREMENT was tried here and reverted: like the o-pad it
        inflated every static demand at speed and cost more park holds than the knife-edge threads
        it refused -- s14 11->24 vs s6 5->1. The learned per-object berth stays the only inflater.)"""
        cpos = (ev["c0"][None, :] + (tau[ev["idx"], None] + t_ahead) * ev["v"][None, :])
        gpt0 = base[ev["idx"]]
        for o in np.arange(cfg.o_step, cfg.omax + 1e-9, cfg.o_step):
            gpt = gpt0 + (sd * o) * n[None, :]
            if float(np.min(np.linalg.norm(gpt - cpos, axis=1) - ev["R"][ev["idx"]])) >= 0.0:
                return float(o)
        return cfg.omax

    # ---- CLUSTER one-side law (gtxy F3): overlapping bump supports must agree on a side.
    # The per-station max(oL,oR) merge is kept for DISJOINT clusters only, where it is continuous.
    events.sort(key=lambda ev: ev["s0"] - ev["ramp"])
    clusters = []
    for ev in events:
        lo, hi = ev["s0"] - ev["ramp"], ev["s1"] + ev["ramp"]
        if clusters and lo <= clusters[-1]["hi"] + 1e-9:
            clusters[-1]["evs"].append(ev)
            clusters[-1]["hi"] = max(clusters[-1]["hi"], hi)
        else:
            clusters.append(dict(evs=[ev], hi=hi))
    for cl in clusters:
        evs = cl["evs"]
        cost = {}
        for sd in (1.0, -1.0):
            os_ = [_o_need(ev, sd) for ev in evs]
            # ANTI-CHASE term (s4 forensic): a walker drifting toward the committed side grows
            # its demand faster than the slew can follow -- ride it and you brake into a hold
            # while it walks into you. Score each side by NOW + 1s-AHEAD demand so the drift
            # side loses while the encounter is still seconds out.
            of_ = [_o_need(ev, sd, t_ahead=1.0) for ev in evs]
            cap_pen = sum(100.0 for o in os_ + of_ if o >= cfg.omax - 1e-9)
            cost[sd] = (sum(os_) + sum(of_) + cap_pen, os_)
        cur = evs[0]["side"] if len({ev["side"] for ev in evs}) == 1 else None
        locks = state.setdefault("side_lock", {})
        locked_sides = {ev["side"] for ev in evs if locks.get(ev["skey"], 0) > 0}
        if cur is None and len(locked_sides) == 1:
            # MIXED cluster with a locked member (s10 forensic): re-clustering (a new mover joins)
            # must not reopen a side decision the lock is protecting -- at 7 m/s the L->R profile
            # collapse swung the drone through the crowd's middle. The locked side IS the
            # incumbent for the whole cluster.
            cur = locked_sides.pop()
        if cur is not None:
            # coherent cluster: keep the committed side; switch when it is capped-infeasible and
            # the other side clears (feasibility beats stickiness) OR when the other side is
            # CLEARLY cheaper in the now+ahead score (anti-chase; 1.2 m hysteresis margin keeps
            # marginal flips out) AND the drone is slow enough to rebuild the profile (a flip at
            # speed IS the crowd-swing crash). A fresh switch takes a REFRACTORY lock (s8
            # forensic: per-tick cost seesaw flip-flopped L/R/L through the slew and zeroed the
            # profile mid-dodge); only the capped-infeasible hatch may override lock or speed.
            sd = cur
            locked = any(locks.get(ev["skey"], 0) > 0 for ev in evs)
            hatch = (cost[cur][0] >= 100.0 and cost[-cur][0] < 100.0)
            t_near = min(ev["t"] for ev in evs)
            want = hatch or (cost[-cur][0] + 1.2 < cost[cur][0] and v0 < 3.0 and t_near > 1.2)
            #   ^ anti-chase flips need BOTH a slow drone and a non-imminent conflict (s10: a flip
            #     at 2.8 m/s with the chaser 0.6 s out left no time to rebuild the profile)
            if want and (not locked or hatch):
                sd = -cur
                for ev in evs:
                    locks[ev["skey"]] = 8
        else:
            sd = 1.0 if cost[1.0][0] <= cost[-1.0][0] else -1.0   # mixed sides: min combined score
        for ev, o in zip(evs, cost[sd][1]):
            ev["side"] = sd
            # STATIC O-PAD: a static's minimal clearing offset leaves the line hugging the exact
            # berth boundary; the planner's ~0.3 m wobble then loses to the gate by centimetres
            # (s14 tree gate). Pad the RESPONSE (not the detection radius -- no new conflicts, no
            # corridor sealing) where the cap allows; the verify pass still owns joint feasibility.
            ev["o"] = float(o)
            #   (a blanket static o-pad was tried and REVERTED: padding demands toward the cap
            #    turned s14's passable park into capped weaving -- the learned per-object berth
            #    (F9) is the targeted version of the same idea and stays)
            sides[ev["skey"]] = sd

    for skey in list(side_age):
        if not any(ev["skey"] == skey for ev in events):
            side_age[skey] += 1
            if side_age[skey] > cfg.side_ttl:
                side_age.pop(skey, None)
                sides.pop(skey, None)
    locks = state.get("side_lock", {})
    for skey in list(locks):
        locks[skey] -= 1
        if locks[skey] <= 0:
            locks.pop(skey, None)
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
            off = np.where(oL >= oR, oL, -oR)           # disjoint clusters: dominant side, continuous
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
    # cross-tick commitment: rate-limit the profile against the previous tick's, WORLD-ANCHORED
    # (gtxy fix: comparing at the same drone-relative arc let every bump recede with the drone --
    # a world-fixed obstacle's dodge kept being re-limited as if it were new. Map this tick's
    # stations into the previous tick's frame first; the slew then bounds GENUINE profile change.)
    prev = state.get("prof")
    if prev is not None:
        S_p, off_p, p_prev, u_prev = prev
        s_old = (base - p_prev[None, :]) @ u_prev
        off_prev = np.interp(s_old, S_p, off_p, left=off_p[0] if len(off_p) else 0.0, right=0.0)
        # ASYMMETRIC slew (s4 forensic): DEEPENING an already-committed dodge (same sign, larger
        # |off|) is the maneuver maturing, not a twitch -- let it open at 2x; sign flips, returns
        # to centre and brand-new deflections keep the tight limit.
        deep = 2.0 * cfg.slew
        up = np.where((off_prev > 0.05), deep, cfg.slew)
        dn = np.where((off_prev < -0.05), deep, cfg.slew)
        off = np.clip(off, off_prev - dn, off_prev + up)
    state["prof"] = (S.copy(), off.copy(), p.copy(), u.copy())
    meta["off_max"] = float(np.max(np.abs(off))) if len(off) else 0.0
    meta["conflicts"] = [dict(oid=ev["oid"], s=round(ev["s"], 2), t=round(ev["t"], 2),
                              side=("L" if ev["side"] > 0 else "R"), o=round(ev["o"], 2))
                         for ev in events]
    n_emit = int(np.searchsorted(S, L - 1e-9, side="left")) + 1   # stations spanning [0, L]
    step = max(1, int(round(cfg.out_ds / cfg.ds)))
    ks = list(range(0, n_emit, step))
    if ks[-1] != n_emit - 1:
        ks.append(n_emit - 1)
    pts = np.array([[base[k][0] + off[k] * n[0], base[k][1] + off[k] * n[1], cruise_z] for k in ks])
    return pts, meta


if __name__ == "__main__":
    # 1) empty world -> dead straight, zero offset
    st = {}
    pts, meta = build_guide([0, 0, 1.5], [1, 0, 0], [10, 0], [], st)
    assert meta["off_max"] == 0.0 and abs(pts[:, 1]).max() < 1e-9
    assert abs(pts[-1][0] - 10.0) < 0.5, pts[-1]      # emitted line still ends at the carrot
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

    # 6) F2: the #frz twin INHERITS its parent's sticky side (no L-vs-R self-contradiction)
    st6 = {}
    mv6 = [("w1", (6.0, 1.5), (0.0, -0.9), 1.3),        # crosser: pass-behind side L
           ("w1#frz", (6.0, 1.2), (0.0, 0.0), 1.3, 0.0, 3.0)]   # its frozen disc, ON the line's left
    _, m6 = build_guide([0, 0, 1.5], [3.0, 0, 0], [12, 0], mv6, st6)
    sd6 = {c["oid"]: c["side"] for c in m6["conflicts"]}
    assert len(set(sd6.values())) == 1, f"twin must share the parent side, got {sd6}"
    print(f"[guide] twin side inheritance {sd6} OK")

    # 7) F3: overlapping conflicts from DIFFERENT movers demanding opposite sides -> one side,
    #    and the profile has no discontinuous sign flip (max per-station jump bounded)
    st7 = {}
    mv7 = [("a", (5.0, 1.0), (0.0, 0.0), 1.6), ("b", (7.0, -1.0), (0.0, 0.0), 1.6)]
    for _ in range(12):                                  # let slew converge
        pts7, m7 = build_guide([0, 0, 1.5], [2.0, 0, 0], [14, 0], mv7, st7)
    sd7 = {c["oid"]: c["side"] for c in m7["conflicts"]}
    assert len(set(sd7.values())) == 1, f"overlapping cluster must be one-sided, got {sd7}"
    dj = np.abs(np.diff(pts7[:, 1]))
    assert dj.max() < 1.2, f"profile jump {dj.max():.2f}m -- discontinuity survived"
    print(f"[guide] cluster one-side {sd7} max-jump {dj.max():.2f}m OK")

    # 8) F1 contract: a STATIC row (v=0, veff=0) bends the line like geometry, twin-free
    st8 = {}
    mv8 = [("st42", (6.0, 0.3), (0.0, 0.0), 2.0, 0.0)]
    for _ in range(10):
        pts8, m8 = build_guide([0, 0, 1.5], [2.0, 0, 0], [12, 0], mv8, st8)
    assert m8["conflicts"] and abs(pts8[:, 1]).max() > 1.2, (m8, pts8[:, 1])
    print(f"[guide] static row bends line (apex {np.abs(pts8[:,1]).max():.2f}m) OK")

    # 9) F4 + world anchor: a conflict BEYOND the carrot matures in the extended profile while
    #    still far; when the drone advances, the full dodge is available IMMEDIATELY (no slew fight)
    st9 = {}
    mv9 = [("far", (16.0, 0.0), (0.0, 0.0), 2.0, 0.0)]   # 16m out; carrot L=12
    for _ in range(10):
        pts9, m9 = build_guide([0, 0, 1.5], [7.0, 0, 0], [12, 0], mv9, st9)
    assert m9["conflicts"], "far conflict must be scanned at speed"
    assert abs(pts9[-1][0] - 12.0) < 0.5, "emitted line must still end at the carrot"
    pts9b, m9b = build_guide([4.0, 0, 1.5], [7.0, 0, 0], [16.0, 0], mv9, st9)   # advanced 4 m
    apex = float(np.abs(pts9b[:, 1]).max())
    assert apex > 1.5, f"world-anchored profile must carry the matured dodge, apex {apex:.2f}"
    print(f"[guide] scan extension + world anchor: matured apex {apex:.2f}m after advance OK")

    print("[guide] ALL PASS")
