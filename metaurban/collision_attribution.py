"""collision_attribution — the ONE shared U/P/Q/D collision algebra (07-20 ruling).

Used by BOTH the headless harness (replay_core) and the 3-D renderer (render_3d_video): one
algebra, one implementation, so a collision books identically on every measurement face.

Primary class (mutually exclusive, judged in this order):
  U  no valid certificate bound to the EXECUTED schedule at contact time -- missing/uncertified
     receipt, contact outside the receipt's validity window, or the executor flew a different
     schedule (HOLD / evade / escape / recovery / overridden set-point).  Owner: RTA/execution.
  P  a valid executed certificate exists but the striking object has NO row in that
     certificate's obstacle snapshot.  Owner: perception/tracking.
  Q  object in snapshot, execution matches, but its TRUE trajectory left the calibrated tube.
     Owner: the conformal epsilon (this is the class the theorem budgets).
  D  object in snapshot, TRUE trajectory still inside the tube, collision anyway --
     deterministic proof-chain violation (wrong spline / radius miscount / window gap /
     tracking bound exceeded).  Zero tolerance: P(D)=0 must be VERIFIED before
     P(C_cert,represented) <= eps may be claimed.

Auxiliary reasons are NEVER discarded: a collision that is both missed-detection AND
execution-mismatch keeps both tags (primary U, aux 'not_in_snapshot').  domain_status
(ID | OOD | UNKNOWN) is frozen per EPISODE before it runs and merely annotates records --
an OOD collision still books U/P/Q/D, it just enjoys no ID guarantee.
"""
import hashlib

import numpy as np

R_DRONE = 0.25          # body radius: keep in lockstep with safety_layer/replay conventions
SNAP_MATCH_R = 1.5      # m: striking object <-> snapshot row spatial match gate (COLLDBG's gate)

P_SUBCODES = ("out_of_sensing", "occluded_or_missed", "assoc_or_id_swap", "track_killed_recently",
              "snapshot_gap", "unknown")


def swept_clearance(p0, p1, m0, m1, r, h):
    """Minimum BODY clearance and its earliest time-fraction over one tick: drone chord p0->p1 vs
    mover chord m0->m1, both linear in s in [0,1].  Candidates: both endpoints, the exact
    minimiser of the relative-xy quadratic, and the cylinder-top z-crossing.  Returns
    (clearance, s_at_min).  Endpoint-only measurement missed within-tick penetrations (0.6 m of
    mover motion per tick at 6 m/s) and could book the contact against the wrong tick."""
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
    m0 = np.asarray(m0, float)[:2]; m1 = np.asarray(m1, float)[:2]
    d0 = p0[:2] - m0; dd = (p1[:2] - p0[:2]) - (m1 - m0)
    cands = [0.0, 1.0]
    a = float(dd @ dd)
    if a > 1e-12:
        s_star = -float(d0 @ dd) / a
        if 0.0 < s_star < 1.0:
            cands.append(s_star)
    z0, z1 = float(p0[2]), float(p1[2])
    if abs(z1 - z0) > 1e-9:
        s_h = (h - z0) / (z1 - z0)
        if 0.0 < s_h < 1.0:
            cands.append(s_h)
    best, s_best = 1e18, 0.0
    for s in sorted(cands):
        p = p0 + (p1 - p0) * s
        m = m0 + (m1 - m0) * s
        horiz = float(np.hypot(p[0] - m[0], p[1] - m[1]))
        if p[2] <= h:
            cl = horiz - r - R_DRONE
        else:
            cl = float(np.hypot(max(0.0, horiz - r), p[2] - h)) - R_DRONE
        if cl < best - 1e-12:
            best, s_best = cl, s
    return best, s_best


def executed_hash(p0, p1, src):
    """Hash of what was ACTUALLY flown this tick (chord + source tag) -- receipt field
    executed_segment_hash. src: 'plan' = the certified schedule; anything else = override."""
    hh = hashlib.sha256(np.round(np.asarray(p0, float), 4).tobytes())
    hh.update(np.round(np.asarray(p1, float), 4).tobytes())
    hh.update(str(src).encode())
    return hh.hexdigest()[:16]


def snapshot_row(receipt, m_xy_at_t0, match_r=SNAP_MATCH_R):
    """The snapshot row covering the striking object: spatial match of the object's TRUE position
    at the receipt's decision time against the receipt's track rows.  None = not represented."""
    if not receipt:
        return None
    best, bd = None, match_r
    for row in receipt.get("tracks") or []:
        d = float(np.hypot(row[1] - m_xy_at_t0[0], row[2] - m_xy_at_t0[1]))
        if d < bd:
            best, bd = row, d
    return best


def tube_excess(row, tau_c, m_true_xy, delta):
    """Signed excess of the striking object's TRUE position outside its calibrated keep-out at
    contact time (tau_c seconds after the receipt's decision).  row = (tid, x, y, vx, vy, R,
    veff, cap).  cap=True -> v6 capsule law: distance to the swept SEGMENT [c0, c0+v*tau_c];
    cap=False -> CV point law: distance to c0+v*tau_c.  rho = R + veff*(tau_c+delta).
    excess > 0 => the true trajectory left the tube (Q); excess <= 0 => still covered (D)."""
    _tid, x, y, vx, vy, R, veff = row[:7]
    cap = bool(row[7]) if len(row) > 7 else False
    rho = float(R) + float(veff) * (float(tau_c) + float(delta))
    p = np.asarray(m_true_xy, float)
    a = np.array([x, y], float)
    tip = a + np.array([vx, vy], float) * float(tau_c)
    if cap:
        ab = tip - a
        L2 = float(ab @ ab)
        s = 0.0 if L2 < 1e-12 else min(1.0, max(0.0, float((p - a) @ ab) / L2))
        d = float(np.linalg.norm(p - (a + s * ab)))
    else:
        d = float(np.linalg.norm(p - tip))
    return d - rho, rho, d


def attribute(*, receipt, exec_src, contact_t, valid_from, valid_until, m_xy_at_t0, m_true_xy,
              delta, track_state=None, domain="UNKNOWN", p_subcode="unknown"):
    """One collision -> primary class + auxiliary reasons (order U -> P -> Q -> D).
    contact_t is absolute; valid_from/valid_until come from the governing receipt (the receipt of
    the tick the contact fell in).  m_xy_at_t0 = striking object's TRUE xy at the receipt's
    decision time (snapshot membership is judged where the certificate LOOKED); m_true_xy = its
    TRUE xy at contact (tube membership is judged where the collision HAPPENED)."""
    aux = []
    certified = bool(receipt and receipt.get("certified"))
    in_window = certified and (valid_from - 1e-9 <= contact_t <= valid_until + 1e-9)
    exec_ok = (exec_src == "plan") or (exec_src == "hold"
                                       and (receipt or {}).get("kind") == "hold_cert")
    #   ^ a CERTIFIED hover (hold_cert receipt, 07-20 #5c) is a plan of its own: holding position
    #   IS the certified schedule, so a hold tick with that receipt is not an override.
    if not certified:
        aux.append("no_certificate")
    elif not in_window:
        aux.append("outside_validity")
    if not exec_ok:
        aux.append(f"exec_override:{exec_src}")
    if receipt is not None and receipt.get("exec_verified") is False:
        aux.append("exec_envelope_violation")   # flew the plan but OUTSIDE the certified tracking
        #   envelope (07-20 #5e) -- kept as evidence; a D verdict with this tag names the broken link
    row = snapshot_row(receipt, m_xy_at_t0) if certified else None
    if certified and row is None:
        aux.append("not_in_snapshot")
    exc = rho = dist = None
    if row is not None:
        tau_c = max(0.0, contact_t - valid_from)
        exc, rho, dist = tube_excess(row, tau_c, m_true_xy, delta)
        if exc is not None and exc > 0:
            aux.append("left_tube")
    if not certified or not in_window or not exec_ok:
        primary = "U"
    elif row is None:
        primary = "P"
        aux.append(f"p_subcode:{p_subcode}")
    elif exc > 0:
        primary = "Q"
    else:
        primary = "D"
    return dict(primary=primary, aux=aux, domain=domain,
                tube_excess=(round(exc, 4) if exc is not None else None),
                tube_rho=(round(rho, 4) if rho is not None else None),
                pred_dist=(round(dist, 4) if dist is not None else None),
                snapshot_row=(list(row) if row is not None else None),
                track_state=(dict(track_state) if track_state else None),
                cert_id=(receipt or {}).get("cert_id"),
                exec_src=exec_src, contact_t=round(float(contact_t), 3))


def collision_record(*, tick, contact_t, collider_id, collider_cls, clearance, obj_scope,
                     verdict, all_colliders):
    """The ledger row: EVERY contacting object is listed (all_colliders), the theorem may later
    scope to dynamic movers but the ledger never drops static/terrain contacts (07-20 ruling)."""
    return dict(tick=int(tick), contact_t=round(float(contact_t), 3),
                collider=dict(id=collider_id, cls=str(collider_cls), scope=str(obj_scope)),
                clearance=round(float(clearance), 4), verdict=verdict,
                all_colliders=[dict(id=c[0], cls=str(c[1]), clearance=round(float(c[2]), 4),
                                    scope=str(c[3])) for c in all_colliders])
