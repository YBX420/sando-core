"""replay_core — replay REAL harvested MetaUrban mover trajectories against ours vs native EGO.

The movers follow their RECORDED ground-truth motion (they do NOT yield to the drone -> strict, worst-case eval).
The drone flies a start->goal corridor THROUGH a cluster of crossing movers, seeing only NOISY detections of
their CURRENT positions; it runs its own CA-Kalman + the continuous-time cylinder certificate with the
CONFORMAL-CALIBRATED per-class keep-out (q_conformal, v_eff from out/conformal/calib.json).

  mode="ours"   : KF-predict every mover, cert-gated maneuver tournament (ground around / fly-over / climb), pick
                  the certified candidate with the greatest goal-ward speed.  R = r_mover+r_drone+d_safe+q_conf,
                  tube grows v_eff*(t+delta) -> P(collision) <= eps by the conformal guarantee.
  mode="native" : plain EGO reacting to the movers' CURRENT positions (reckless real EGO, grid inflation 0.3).

Shared by ab_replay.py (the A/B) and conformal_ablation.py (continuous-time-vs-sampling + predict on/off).
"""
import os, sys, math, json, glob
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from ego_bridge import EGOPlanner
from kf_tracker import MoverTracker

OUTDIR = os.path.join(os.path.dirname(HERE), "out", "conformal")

# ---- geometry / dynamics ----
DT = 0.30; TAU = 0.75; DELTA = DT
CRUISE_Z = 1.5; Z_CEIL = 4.6
R_DRONE = 0.25
MEAS = 0.07
PHI = math.radians(25.0)
HORIZON = 7.5
MAXTICKS = 240
PRED_MODEL = os.environ.get("PRED_MODEL", "cv")   # deployed predictor (CV: tighter conformal keep-out, see calib)
FOV_R = 14.0          # perception/cert range: only movers within FOV_R are fed to EGO + certified (a TAU=0.75s,
#                       3 m/s mover >14 m away can't reach the drone within the trust window). Matches a real
#                       onboard depth sensor's useful range and keeps the per-tick cert/cloud cost bounded.
REACH_PAD = 0.3
D_SAFE_H = float(os.environ.get("REP_DSAFE", 0.10))    # small comfort standoff (the conformal tube carries safety)
D_SAFE_V = float(os.environ.get("REP_DSAFEV", 0.30))
# per-class physical body radius + head-top height (vehicles/robots are bigger & faster than pedestrians)
CLS_R = {"pedestrian": 0.30, "vehicle": 0.60, "animal": 0.40}
CLS_H = {"pedestrian": 1.80, "vehicle": 1.60, "animal": 1.00}
WIN = 9.0                                              # episode window length (s)


def load_calib(eps=0.05):
    """Return dict class -> (q_conformal, v_eff) at the given eps, from calib.json. Falls back to 'all', then
    to a conservative hand value if a class is missing."""
    path = os.path.join(OUTDIR, "calib.json")
    out = {}
    if os.path.exists(path):
        rep = json.load(open(path))
        g = rep.get("groups", {})
        allv = g.get("all", {}).get("levels", {}).get(str(eps))
        for cls in ("pedestrian", "vehicle", "animal"):
            lv = g.get(cls, {}).get("levels", {}).get(str(eps))
            lv = lv or allv
            if lv:
                out[cls] = (max(0.0, lv["q_conformal"]), lv["v_eff"])
        if allv:
            out["_all"] = (max(0.0, allv["q_conformal"]), allv["v_eff"])
    for cls in ("pedestrian", "vehicle", "animal"):
        out.setdefault(cls, (0.15, 0.6))
    out.setdefault("_all", (0.15, 0.6))
    return out


class Movers:
    """Harvested movers with time-interpolated ground-truth lookup + presence test."""

    def __init__(self, raw):
        self.m = []
        for d in raw:
            t = np.asarray(d["t"], float); xy = np.asarray(d["xy"], float)
            cls = str(d["cls"])
            self.m.append(dict(t=t, xy=xy, cls=cls, t0=float(t[0]), t1=float(t[-1]),
                               r=CLS_R.get(cls, 0.4), h=CLS_H.get(cls, 1.6)))

    def present(self, i, t):
        return self.m[i]["t0"] <= t <= self.m[i]["t1"]

    def pos(self, i, t):
        d = self.m[i]
        return np.array([np.interp(t, d["t"], d["xy"][:, 0]), np.interp(t, d["t"], d["xy"][:, 1])])

    def maxspeed(self, i, t0, t1):
        d = self.m[i]
        mask = (d["t"] >= t0) & (d["t"] <= t1)
        if mask.sum() < 2:
            return 0.0
        xy = d["xy"][mask]
        return float(np.max(np.linalg.norm(np.diff(xy, axis=0), axis=1)) / 0.1)


CORRIDOR_L = 14.0     # fixed demo corridor length (m) -> ~5 s flight, comparable across seeds
V_NOM = 2.5           # nominal cruise used only to TIME the drone's arrival at the crossing point


def build_episodes(movers, seed, n_ep=6):
    """Pick SHORT corridors (length CORRIDOR_L) GUARANTEED to be contested: the drone is timed so it reaches the
    corridor centre at the same instant the anchor crosser does, with the corridor laid PERPENDICULAR to the
    crosser's motion -> a genuine head-crossing the planner must resolve. Returns dict(start, goal, t0, members)."""
    import random
    t_lo = min(d["t0"] for d in movers.m); t_hi = max(d["t1"] for d in movers.m)
    transit_half = 0.5 * CORRIDOR_L / V_NOM          # time for the drone to fly start -> centre
    eps = []
    for e in range(n_ep * 12):
        if len(eps) >= n_ep:
            break
        rng = random.Random(seed * 1000 + e)
        movers_moving = [i for i in range(len(movers.m))
                         if movers.maxspeed(i, movers.m[i]["t0"], movers.m[i]["t1"]) > 0.8]
        if not movers_moving:
            break
        a = rng.choice(movers_moving)
        da = movers.m[a]
        # a crossing instant well inside a's window so the drone has runway before and after
        if da["t1"] - da["t0"] < 2 * transit_half + 1.0:
            continue
        t_cross = rng.uniform(da["t0"] + transit_half + 0.3, da["t1"] - transit_half - 0.3)
        t0 = t_cross - transit_half
        center = movers.pos(a, t_cross)
        va = movers.pos(a, min(t_cross + 0.3, da["t1"])) - movers.pos(a, max(t_cross - 0.3, da["t0"]))
        if np.linalg.norm(va) < 1e-3:
            continue
        va = va / np.linalg.norm(va); dirv = np.array([-va[1], va[0]])             # corridor perpendicular to motion
        start = np.array([*(center - 0.5 * CORRIDOR_L * dirv), CRUISE_Z])
        goal = np.array([*(center + 0.5 * CORRIDOR_L * dirv), CRUISE_Z])
        if not movers.present(a, t0):
            continue
        clr0 = min([np.linalg.norm(start[:2] - movers.pos(i, t0)) - movers.m[i]["r"]
                    for i in range(len(movers.m)) if movers.present(i, t0)] or [9.9])
        if clr0 < 0.8:
            continue
        members = [i for i in range(len(movers.m))
                   if movers.present(i, t_cross) and np.linalg.norm(movers.pos(i, t_cross) - center) < 8.0]
        eps.append(dict(start=start, goal=goal, t0=t0, members=members, anchor=a, t_cross=t_cross))
    return eps


def _cyl_cloud(centres, r, z_lo, z_hi, n_th=12, n_z=3):
    pts = []
    for cx, cy in centres:
        for th in np.linspace(0, 2 * np.pi, n_th, endpoint=False):
            for z in np.linspace(z_lo, z_hi, n_z):
                pts.append([cx + r * math.cos(th), cy + r * math.sin(th), z])
    return pts


def _clearance(p, c_xy, r, h):
    horiz = math.hypot(p[0] - c_xy[0], p[1] - c_xy[1])
    if p[2] <= h:
        return horiz - r
    return math.hypot(max(0.0, horiz - r), p[2] - h)


def _rot(v2, ang):
    c, s = math.cos(ang), math.sin(ang)
    return np.array([c * v2[0] - s * v2[1], s * v2[0] + c * v2[1]])


def run_replay(movers, ep, mode="ours", calib=None, predict=True, max_vel=3.0, max_acc=6.0,
               cont_cert=True, n_sample=0, record=False):
    """One replay episode. cont_cert=True uses the continuous-time Bernstein cylinder cert; if False (ablation)
    the gate uses n_sample fixed-rate samples of the committed B-spline instead. Returns a result dict."""
    calib = calib or load_calib()
    # work in a LOCAL frame centred on the corridor midpoint: MetaUrban world coords span hundreds of metres,
    # so a global grid would be billions of voxels. Translate everything by -org -> a small local map suffices.
    org = 0.5 * (ep["start"][:2] + ep["goal"][:2])
    start = ep["start"].copy(); start[:2] -= org
    goal = ep["goal"].copy(); goal[:2] -= org
    t_base = ep["t0"]

    def pos_l(i, t):                                   # mover position in the local frame
        return movers.pos(i, t) - org

    infl = 0.3 if mode == "native" else float(os.environ.get("REP_OURS_INFL", 0.45))
    ego = EGOPlanner(map_origin=(-40, -40, -1), map_size=(80, 80, 8), res=0.2, inflation=infl)
    ego.set_params(max_vel=max_vel, max_acc=max_acc, horizon=HORIZON)
    trackers = {}                                   # mover idx -> MoverTracker (created on first detection)
    rng = np.random.default_rng(1234567)

    p_d = start.copy(); v_d = np.zeros(3); a_d = np.zeros(3)
    min_clr = 1e18; max_z = start[2]; reached = False
    counts = {k: 0 for k in ("straight", "around_l", "around_r", "over", "climb", "evade", "native")}
    hist = []
    best_d = 1e18; stall = 0                            # early-stop degenerate episodes (EGO can't plan -> evade spins)

    def present_idx(t):
        return [i for i in range(len(movers.m)) if movers.present(i, t)]

    for tick in range(MAXTICKS):
        t = t_base + tick * DT
        idx = present_idx(t)
        # noisy detections + KF update
        dets = {}
        for i in idx:
            gt = pos_l(i, t)
            det = gt + rng.normal(0, MEAS, 2)
            dets[i] = det
            if i not in trackers:
                trackers[i] = MoverTracker(dt=DT, meas_noise=MEAS)
            trackers[i].update([det[0], det[1], 1.5])

        near = [i for i in idx if np.linalg.norm(dets[i] - p_d[:2]) < FOV_R]   # only perceive/certify nearby movers
        gxy = goal[:2] - p_d[:2]; dist = float(np.linalg.norm(gxy))
        gdir = gxy / dist if dist > 1e-6 else np.array([1.0, 0.0])
        kind = None

        if mode == "native":
            cloud = []
            for i in near:
                cloud += _cyl_cloud([dets[i][:2]], movers.m[i]["r"], 0.3, movers.m[i]["h"])
            ego.update_cloud(np.asarray(cloud, float) if cloud else np.zeros((0, 3)), p_d)
            if ego.replan(p_d, v_d, a_d, goal) and ego.duration() > 1e-3:
                r = ego.eval(min(DT, max(ego.duration() - 1e-3, 0.0)))
                if r is not None:
                    p_d, v_d, a_d = (np.asarray(x, float) for x in r)
            counts["native"] += 1; kind = "native"
        else:
            # near-term predicted cloud for EGO's grid
            cloud = []
            for i in near:
                trk = trackers[i]
                xy = trk.predict(np.linspace(0, DT, 2), model=PRED_MODEL)[:, :2] if trk.ready else dets[i][None, :2]
                cloud += _cyl_cloud(xy, movers.m[i]["r"], 0.3, movers.m[i]["h"])
            ego.update_cloud(np.asarray(cloud, float) if cloud else np.zeros((0, 3)), p_d)

            # cylinder params per present mover (predicted polynomial + conformal per-class keep-out)
            cyl = []
            ztop = CRUISE_Z
            for i in near:
                cls = movers.m[i]["cls"]; q, veff = calib.get(cls, calib["_all"])
                c0, vv, aa = trackers[i].state()
                if PRED_MODEL == "cv":
                    aa = np.zeros(3)                     # CV deployment: cert polynomial matches the CV-calibrated tube
                if not predict:
                    vv, aa = np.zeros(3), np.zeros(3)
                R = movers.m[i]["r"] + R_DRONE + D_SAFE_H + q
                zc = movers.m[i]["h"] + REACH_PAD + R_DRONE + D_SAFE_V + q
                cyl.append((c0, vv, aa, R, zc, veff))
                ztop = max(ztop, zc + 0.2)
            ztop = min(Z_CEIL, ztop)

            def cert_clear():
                for (c0, vv, aa, R, zc, veff) in cyl:
                    if cont_cert:
                        hp, _ = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=vv, obs_acc=aa, t_hi=TAU, v_eff=veff, delta=DELTA)
                        hc, _ = ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=(0, 0, 0), t_hi=TAU, v_eff=veff, delta=DELTA)
                        vo, _ = ego.certify_above(z_clear=zc, t_hi=TAU, v_eff_z=0.0, delta=DELTA)
                        if not ((hp and hc) or vo):
                            return False
                    else:
                        # discrete-sampling gate (ablation): check only n_sample points of the committed B-spline
                        dur = ego.duration()
                        ok = True
                        for s in np.linspace(0.0, min(TAU, dur), max(2, n_sample)):
                            r = ego.eval(s)
                            if r is None:
                                continue
                            pp = r[0]
                            cc = c0 + vv * s + 0.5 * aa * s * s
                            rho = R + veff * (s + DELTA)
                            horiz = math.hypot(pp[0] - cc[0], pp[1] - cc[1])
                            if not (horiz >= rho or pp[2] >= zc):
                                ok = False; break
                        if not ok:
                            return False
                return True

            def sl():
                rr = ego.eval(min(DT, max(ego.duration() - 1e-3, 0.0)))
                return tuple(np.asarray(x, float) for x in rr) if rr is not None else None

            L = min(HORIZON, max(dist, 1.0)); chosen = None
            for gk, gsub in (("straight", np.array([goal[0], goal[1], CRUISE_Z])),
                             ("around_l", np.array([*(p_d[:2] + L * _rot(gdir, PHI)), CRUISE_Z])),
                             ("around_r", np.array([*(p_d[:2] + L * _rot(gdir, -PHI)), CRUISE_Z])),
                             ("around_l", np.array([*(p_d[:2] + L * _rot(gdir, 2 * PHI)), CRUISE_Z])),
                             ("around_r", np.array([*(p_d[:2] + L * _rot(gdir, -2 * PHI)), CRUISE_Z]))):
                if ego.replan(p_d, v_d, a_d, gsub) and ego.duration() > 1e-3 and cert_clear():
                    kind = gk; chosen = sl(); break
            if kind is None:
                if ego.replan(p_d, v_d, a_d, np.array([goal[0], goal[1], ztop])) and ego.duration() > 1e-3 and cert_clear():
                    kind = "over"; chosen = sl()
                elif ego.replan(p_d, v_d, a_d, np.array([p_d[0], p_d[1], ztop])) and ego.duration() > 1e-3:
                    kind = "climb"; chosen = sl()
                else:
                    kind = "evade"; chosen = None
            if chosen is not None:
                p_d, v_d, a_d = chosen
            elif idx:
                nn_i = min(idx, key=lambda i: np.linalg.norm(dets[i][:2] - p_d[:2]))
                away = p_d[:2] - dets[nn_i][:2]; nn = np.linalg.norm(away)
                away = away / nn if nn > 1e-6 else gdir
                p_d = p_d + np.array([away[0] * 0.6 * max_vel * DT, away[1] * 0.6 * max_vel * DT,
                                      min(0.4 * max_vel * DT, max(0.0, ztop - p_d[2]))])
                v_d = np.array([away[0] * max_vel, away[1] * max_vel, 0.0]); a_d = np.zeros(3)
            counts[kind] = counts.get(kind, 0) + 1

        max_z = max(max_z, float(p_d[2]))
        tick_clr = 1e18
        for i in present_idx(t):
            cl = _clearance(p_d, pos_l(i, t), movers.m[i]["r"], movers.m[i]["h"])
            tick_clr = min(tick_clr, cl); min_clr = min(min_clr, cl)
        if record:
            hist.append(dict(tick=tick, t=round(t, 2), clr=round(tick_clr, 3) if tick_clr < 1e17 else None,
                             kind=kind, z=round(float(p_d[2]), 2),
                             p=[round(float(p_d[0]), 2), round(float(p_d[1]), 2)]))
        dgoal = float(np.linalg.norm(p_d[:2] - goal[:2]))
        if dgoal < 0.8:
            reached = True; break
        if dgoal < best_d - 0.1:
            best_d = dgoal; stall = 0
        else:
            stall += 1
        if stall > 40:                                 # ~12 s without net progress -> give up (not reached)
            break

    return dict(mode=mode, reached=reached, ticks=tick + 1, time_s=(tick + 1) * DT,
                min_clr=float(min_clr) if min_clr < 1e17 else None,
                collided=(min_clr < -1e-6), max_z=max_z, counts=counts,
                predict=predict, cont_cert=cont_cert, history=hist if record else None)


if __name__ == "__main__":
    # smoke test on the first available seed
    files = sorted(glob.glob(os.path.join(OUTDIR, "traj_seed*.npz")))
    if not files:
        print("no harvested seeds; run conformal_harvest.py first"); sys.exit(1)
    dat = np.load(files[0], allow_pickle=True)
    movers = Movers(list(dat["movers"]))
    seed = int(os.path.basename(files[0]).replace("traj_seed", "").replace(".npz", ""))
    eps = build_episodes(movers, seed, n_ep=3)
    print(f"[replay] seed {seed}: {len(movers.m)} movers, {len(eps)} episodes")
    calib = load_calib(0.05)
    print(f"[replay] calib eps=0.05: " + "  ".join(f"{c}=(q{calib[c][0]:.2f},v{calib[c][1]:.2f})"
                                                    for c in ("pedestrian", "vehicle")))
    for k, ep in enumerate(eps):
        ro = run_replay(movers, ep, "ours", calib)
        rn = run_replay(movers, ep, "native", calib)
        print(f"  ep{k}: ours t={ro['time_s']:.1f}s clr={ro['min_clr']} reach={ro['reached']} coll={ro['collided']} "
              f"| native t={rn['time_s']:.1f}s clr={rn['min_clr']} reach={rn['reached']} coll={rn['collided']}")
