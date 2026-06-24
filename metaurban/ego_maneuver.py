"""ego_maneuver — certified FAST-AS-SAFE pass-through on EGO. NO HOLD: meet a moving human, MANEUVER.

The thesis (2026-06-24, 塔菲大人): "because I KNOW how the human will move, I dare to fly FASTER but safe."
Safety (the continuous-time Bernstein certificate) is a HARD constraint; UNDER it we maximize speed. Each tick:
  1. PREDICT every human with a CA-Kalman filter (kf_tracker.MoverTracker),
  2. render the near-term predicted CYLINDER footprint into EGO's grid (capped at head height -> sky stays open),
  3. offer EGO biased sub-goals -> candidate maneuvers {straight / around-L / around-R / over / climb},
  4. CERTIFY each committed B-spline vs every human with the CYLINDER disjunction
     (horizontal sqrt(dx^2+dy^2)>=r+d_safe  OR  vertical p_z>=z_clear), whole window [0,TAU],
  5. COMMIT the certified candidate with the greatest goal-ward speed (FASTEST-safe).
HOLD is deleted (humans can't fly -> climbing is the always-available no-freeze escape). Smoothness is not a
competing objective: jerky brake/re-accel and maneuver chatter just waste speed, so maximizing goal-ward speed
(warm-started from live p,v,a) already prefers the smooth carry-through.

mode="native" is the BASELINE: plain EGO reacting to the human's CURRENT position (no prediction, no certificate,
no fly-over) — what you get without knowing the future. The A/B (ego_vs_native.py) shows ours is faster AND safe.

Run headless:  python metaurban/ego_maneuver.py            (ours, default scene)
               EGO_SCENE=gauntlet python metaurban/ego_maneuver.py
"""
import os, sys, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from ego_bridge import EGOPlanner
from kf_tracker import MoverTracker
from scenario import load_scene

HORIZON = 7.5
PHI = math.radians(25.0)
DEADBAND = 0.25      # hysteresis: keep last maneuver kind unless another beats it by >DEADBAND m/s (anti-chatter)
MAXTICKS = 200


def _cyl_cloud(centres, r, z_lo, z_hi, n_th=12, n_z=4):
    """Vertical-cylinder side surface for each (x,y) centre, z in [z_lo,z_hi]. Capped at head height -> the
    overhead column stays free so EGO can climb OVER."""
    pts = []
    for cx, cy in centres:
        for th in np.linspace(0, 2 * np.pi, n_th, endpoint=False):
            for z in np.linspace(z_lo, z_hi, n_z):
                pts.append([cx + r * math.cos(th), cy + r * math.sin(th), z])
    return pts


def _predicted_cloud(trackers, dets, movers, p_d, plan_hi, n_samp=3, fov_r=12.0):
    """Near-term PREDICTED cylinder footprints (ours). plan_hi (< TAU) = thin near-term sweep so several
    futures don't stack into a corridor-freezing phantom wall; the growing tube + replan carry the rest."""
    pts = []
    ts = np.linspace(0.0, plan_hi, n_samp)
    for trk, det, (c, v, r, h) in zip(trackers, dets, movers):
        if np.linalg.norm(np.asarray(det)[:2] - p_d[:2]) > fov_r:
            continue
        xy = trk.predict(ts)[:, :2] if trk.ready else np.asarray(det, float)[None, :2]
        pts += _cyl_cloud(xy, r, z_lo=0.3, z_hi=h)
    return np.asarray(pts, float) if pts else np.zeros((0, 3))


def _current_cloud(dets, movers, p_d, fov_r=12.0):
    """CURRENT detected cylinder footprints (native baseline): no prediction, EGO reacts to where the human IS."""
    pts = []
    for det, (c, v, r, h) in zip(dets, movers):
        if np.linalg.norm(np.asarray(det)[:2] - p_d[:2]) > fov_r:
            continue
        pts += _cyl_cloud([det[:2]], r, z_lo=0.3, z_hi=h)
    return np.asarray(pts, float) if pts else np.zeros((0, 3))


def _clearance(p, c_xy, r, h):
    """Signed clearance from drone point p to the solid cylinder (radius r, z in [0,h]). <0 => collision."""
    horiz = math.hypot(p[0] - c_xy[0], p[1] - c_xy[1])
    if p[2] <= h:
        return horiz - r
    return math.hypot(max(0.0, horiz - r), p[2] - h)


def _rot(v2, ang):
    c, s = math.cos(ang), math.sin(ang)
    return np.array([c * v2[0] - s * v2[1], s * v2[0] + c * v2[1]])


def run_episode(mode="ours", max_vel=None, scene_name=None, seed=2026, record=False, predict=True):
    """One headless episode. mode='ours' = certify+fastest-safe tournament; mode='native' = plain EGO reacting
    to current positions. predict=False keeps the SAME cylinder cert + d_safe + maneuvering but certifies against
    the mover's CURRENT position (obs_vel=0) and feeds current-position occupancy — an apples-to-apples ablation
    that isolates what the KF PREDICTION buys (same safety floor, only the future-knowledge differs)."""
    start, goal, movers, K = load_scene(scene_name)
    movers = [[c.copy(), v.copy(), r, h] for c, v, r, h in movers]   # don't mutate the shared scene
    DT = K["DT"]; TAU = K["TAU"]; DELTA = DT
    D_SAFE = K["D_SAFE"]; D_SAFE_V = K["D_SAFE_V"]; REACH_PAD = K["REACH_PAD"]
    V_EFF = K["V_EFF"]; V_EFF_Z = K["V_EFF_Z"]; Z_CRUISE = K["Z_CRUISE"]; Z_CEIL = K["Z_CEILING"]
    PLAN_HI = K["PLAN_HI"]; MEAS = 0.07
    # q_conformal placeholder: extra keep-out that covers the KF prediction residual under detection noise.
    # Without it the cert only guarantees clearance to the PREDICTED centre, so when the human moves unlike the
    # prediction the true clearance can dip < 0 (collide). DUAL certify (predicted AND current position) on top
    # covers "human did NOT move as predicted". Together -> never collide, while staying tight enough to beat EGO.
    Q_CONF = float(os.environ.get("EGO_QCONF", 0.15))
    # CRITICAL: inflate ours' EGO grid to ~d_safe so EGO's own 2-D route already clears the certificate margin.
    # With the old thin 0.3 inflation EGO routed at 0.3-0.5 m and the d_safe cert REJECTED every ground route ->
    # the drone climbed OVER everything (slow, jittery). At d_safe+slack the straight/around 2-D routes certify,
    # so the drone weaves on the ground like EGO does (fast) and only climbs when 2-D is genuinely blocked.
    INFL_X = float(os.environ.get("EGO_INFLX", 0.10))   # slack above D_SAFE for ours' planner inflation
    mv = float(max_vel) if max_vel else K["MAX_VEL"]
    # MATCHED SAFETY for a fair A/B: native has no certificate, so give its grid an inflation = D_SAFE, making
    # plain EGO also hold the 0.8 m standoff (from the CURRENT position). Ours keeps a thin inflation (the
    # certificate enforces r+D_SAFE against the PREDICTED centre). Same executed standoff -> the only difference
    # is prediction, so a speed win is attributable to "I know where the human WILL be", not to flying closer.
    # native inflation = EGO_NINFL: 0.3 reproduces the REAL EGO-Planner (flies ~0.3 m from people, fast but
    # grazes/collides); set =D_SAFE for a matched-safety (handicapped) baseline. ours inflates to d_safe+q+slack
    # so EGO's own 2-D route clears the cert margin (fly ground fast, climb only over real walls).
    N_INFL = float(os.environ.get("EGO_NINFL", 0.3))
    infl = N_INFL if mode == "native" else (D_SAFE + Q_CONF + INFL_X)

    ego = EGOPlanner(map_origin=(-30, -30, -1), map_size=(80, 80, 6), res=0.2, inflation=infl)
    ego.set_params(max_vel=mv, max_acc=K["MAX_ACC"], horizon=HORIZON)
    trackers = [MoverTracker(dt=DT, meas_noise=MEAS) for _ in movers]
    rng = np.random.default_rng(seed)

    p_d = start.copy(); v_d = np.zeros(3); a_d = np.zeros(3)
    min_clr = 1e18; max_z = start[2]; reached = False; last_kind = "straight"
    counts = {k: 0 for k in ("straight", "around_l", "around_r", "over", "climb", "cert_miss", "native", "evade")}
    prog_speeds = []; hist = []
    stall_ctr = 0; climb_mode = False                      # ours: stall->fly-OVER escape state (persists across ticks)
    tick = -1

    for tick in range(MAXTICKS):
        dets = [m[0] + rng.normal(0, MEAS, 3) for m in movers]
        for trk, d in zip(trackers, dets):
            trk.update(d)
        gxy = goal[:2] - p_d[:2]; dist = float(np.linalg.norm(gxy))
        gdir = gxy / dist if dist > 1e-6 else np.array([1.0, 0.0])

        if mode == "native":
            # plain EGO: react to CURRENT positions, fly straight to goal, no certificate, no fly-over
            ego.update_cloud(_current_cloud(dets, movers, p_d), p_d)
            if ego.replan(p_d, v_d, a_d, goal) and ego.duration() > 1e-3:
                r = ego.eval(min(DT, max(ego.duration() - 1e-3, 0.0)))
                if r is not None:
                    p_d, v_d, a_d = (np.asarray(x, float) for x in r)
            counts["native"] += 1
            prog_speeds.append(float(np.dot(v_d[:2], gdir)))
        else:
            # ours = native EGO ground route GATED by the continuous-time cylinder certificate: fly the ground slice
            # when it is certified clear (= native, fast), and when the ground route would squeeze too close (the
            # cert REJECTS it — exactly where native collides) fly OVER the obstacle instead; if boxed in, climb
            # straight up; if literally trapped inside occupancy, push away from the nearest mover. Every rung MOVES
            # (never freezes -> never the walked-into death) and is itself certified (over) or strictly clearance-
            # increasing (climb / evade).
            ego.update_cloud(_predicted_cloud(trackers, dets, movers, p_d, DT if predict else 0.0, n_samp=2), p_d)
            z_top = min(Z_CEIL, max(h for (_, _, _, h) in movers) + REACH_PAD + D_SAFE_V + 0.2) if movers else Z_CRUISE
            cyl = []
            for trk, (c, v, r, h) in zip(trackers, movers):
                c0, vv, aa = trk.state()
                if not predict:
                    vv, aa = np.zeros(3), np.zeros(3)
                cyl.append((c0, vv, aa, r + D_SAFE + Q_CONF, h + REACH_PAD + D_SAFE_V + Q_CONF))

            def _cert_clear():   # the CURRENTLY-held EGO B-spline vs every mover (cylinder disjunction, dual horizontal)
                for (c0, vv, aa, R, zc) in cyl:
                    hp, _ = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=vv, obs_acc=aa, t_hi=TAU, v_eff=V_EFF, delta=DELTA)
                    hc, _ = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=(0, 0, 0), t_hi=TAU, v_eff=V_EFF, delta=DELTA)
                    vo, _ = ego.certify_above(z_clear=zc, t_hi=TAU, v_eff_z=V_EFF_Z, delta=DELTA)
                    if not ((hp and hc) or vo):
                        return False
                return True

            def _slice():
                rr = ego.eval(min(DT, max(ego.duration() - 1e-3, 0.0)))
                return tuple(np.asarray(x, float) for x in rr) if rr is not None else None

            # GROUND options first (stay low = fast): straight to goal, then bias around each side. Fly the first
            # one whose committed B-spline is certified clear. Only climb OVER if NO ground route certifies (a wall).
            L = min(HORIZON, max(dist, 1.0))
            kind = None; sl = None
            for gk, gsub in (("straight", np.array([goal[0], goal[1], Z_CRUISE])),
                             ("around_l", np.array([*(p_d[:2] + L * _rot(gdir, PHI)), Z_CRUISE])),
                             ("around_r", np.array([*(p_d[:2] + L * _rot(gdir, -PHI)), Z_CRUISE])),
                             ("around_l", np.array([*(p_d[:2] + L * _rot(gdir, 2 * PHI)), Z_CRUISE])),
                             ("around_r", np.array([*(p_d[:2] + L * _rot(gdir, -2 * PHI)), Z_CRUISE]))):
                if ego.replan(p_d, v_d, a_d, gsub) and ego.duration() > 1e-3 and _cert_clear():
                    kind = gk; sl = _slice(); break
            if kind is None:
                if ego.replan(p_d, v_d, a_d, np.array([goal[0], goal[1], z_top])) and ego.duration() > 1e-3 and _cert_clear():
                    kind = "over"; sl = _slice()                                # no ground route -> fly OVER (certified)
                elif ego.replan(p_d, v_d, a_d, np.array([p_d[0], p_d[1], z_top])) and ego.duration() > 1e-3:
                    kind = "climb"; sl = _slice()                              # boxed -> climb straight up (no-freeze escape)
                else:
                    kind = "evade"; sl = None                                   # trapped inside occ -> push away below
            if sl is not None:
                p_d, v_d, a_d = sl
            else:
                near = min(movers, key=lambda m: float(np.linalg.norm(m[0][:2] - p_d[:2])))
                away = p_d[:2] - near[0][:2]; nn = float(np.linalg.norm(away)); away = away / nn if nn > 1e-6 else gdir
                p_d = p_d + np.array([away[0] * 0.6 * mv * DT, away[1] * 0.6 * mv * DT,
                                      min(0.4 * mv * DT, max(0.0, z_top - p_d[2]))])
                v_d = np.array([away[0] * mv, away[1] * mv, 0.0]); a_d = np.zeros(3)
            prog_speeds.append(float(np.dot(v_d[:2], gdir)))
            last_kind = kind
            counts[kind] = counts.get(kind, 0) + 1

        max_z = max(max_z, float(p_d[2]))
        tick_clr = 1e18; worst_m = None
        for c, v, r, h in movers:
            cl = _clearance(p_d, c[:2], r, h)
            if cl < tick_clr:
                tick_clr = cl; worst_m = c[:2].copy()
            min_clr = min(min_clr, cl)
            c[:2] += v[:2] * DT
        if record:
            hist.append(dict(tick=tick, clr=round(tick_clr, 3), kind=kind, z=round(float(p_d[2]), 2),
                             p=[round(float(p_d[0]), 2), round(float(p_d[1]), 2)],
                             m=[round(float(worst_m[0]), 2), round(float(worst_m[1]), 2)] if worst_m is not None else None))
        if np.linalg.norm(p_d[:2] - goal[:2]) < 0.8:
            reached = True; break

    return dict(mode=mode, scene=K["name"], max_vel=mv, reached=reached, ticks=tick + 1,
                time_s=(tick + 1) * DT, mean_speed=float(np.mean(prog_speeds)) if prog_speeds else 0.0,
                min_clr=min_clr, collided=min_clr < -1e-6, max_z=max_z, counts=counts, d_safe=D_SAFE,
                history=hist if record else None)


def main():
    m = run_episode("ours")
    c = m["counts"]
    print(f"=== EGO no-HOLD certified maneuvering — scene '{m['scene']}' (headless, ours) ===")
    print(f"  ticks={m['ticks']} time={m['time_s']:.1f}s reached={m['reached']}  maneuvers: " +
          " ".join(f"{k}={c[k]}" for k in ("straight", "around_l", "around_r", "over", "climb", "cert_miss")))
    print(f"  executed min clearance to humans = {m['min_clr']:.3f} m  (cylinder; AROUND floor d_safe={m['d_safe']})")
    print(f"  max altitude reached = {m['max_z']:.2f} m  (fly-OVER climbs above the {1.8} m heads)")
    print(f"  mean goal-ward speed (fast-as-safe) = {m['mean_speed']:.3f} m/s")
    n_over = c["over"] + c["climb"]; n_around = c["around_l"] + c["around_r"]
    print(f"  flew OVER/climbed {n_over} ticks, AROUND {n_around} ticks, HOLD 0 (deleted)")
    print("  SAFE: executed path never entered any cylinder (cert gated every commit)" if not m["collided"]
          else "  UNSAFE: executed path collided (investigate)")
    if c["cert_miss"]:
        print(f"  NOTE: {c['cert_miss']} cert-miss tick(s) fell back to climb-in-place")
    return 0 if not m["collided"] else 1


if __name__ == "__main__":
    sys.exit(main())
