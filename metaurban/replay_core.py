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
from quadrotor import Quadrotor   # real multicopter dynamics (the SAME model the renderer/PX4 seam flies)
import safety_layer as SL          # the ONE shared certified-maneuver decision (render + headless call this)

OUTDIR = os.path.join(os.path.dirname(HERE), "out", "conformal")

# B-bucket deployment-in-the-loop residual harvest (activated by b_bucket_recalibrate.py, not env):
PERCEPT_HARVEST = None            # set to a list to collect (Delta, resid, age, cls, ep_id) tuples
_HARV_DELTAS = (tuple(float(x) for x in os.environ["HARV_DELTAS"].split(","))
                if "HARV_DELTAS" in os.environ else
                (0.10, 0.25, 0.40, 0.55, 0.70, 0.85))   # legacy grid; FS3C-R passes 0.00..1.05/0.05
_HARV_EP = [0]
# FS3C-R harvest v2 (HARV_V2=1): per-row scenario name, young-arm frozen scoring, A2 presence-miss
_HARV_V2 = os.environ.get("HARV_V2", "0") == "1"
_HARV_AGEMIN = int(os.environ.get("HARV_AGE_MIN", "4"))
_HARV_SCN = [""]
_HARV_RHO = json.loads(os.environ["HARV_RHO"]) if "HARV_RHO" in os.environ else None
PERCEPT_A2 = None                 # set to dict(miss_ticks=0, qual_ticks=0) by the v2 driver
_QUAL_MEMO = {}                   # per-tick mover-index -> future-reach qualification (theta2)

# ---- geometry / dynamics ----
DT = 0.30; TAU = 0.75
DELTA = float(os.environ.get("DELTA_OVR", DT))   # anchor-staleness charge in the tube v_eff*(t+DELTA).
#   Historical DT=0.30 assumed the decision consumes LAST tick's tracks; this loop perceives, decides
#   and commits within the SAME tick (anchor fresh, ~7ms compute), so the calibration-consistent
#   charge is ~pipeline latency. DELTA_OVR=0.05 = honest thinning candidate (audit 2026-07-07); the
#   harvest Delta axis is measured FROM the anchor, matching this semantics exactly.
# SMOOTH=1: event-triggered maneuver smoothing (sticky incumbent + dwell-gated strict upgrades);
# default OFF -> byte-identical frozen behaviour (regress_frozen_ours.py guards this).
SMOOTH = os.environ.get("SMOOTH", "0") == "1"
SMOOTH_DWELL = int(os.environ.get("SMOOTH_DWELL", "3"))
SMOOTH_MARGIN = float(os.environ.get("SMOOTH_MARGIN", "0.15"))   # extra delta(s) on the upgrade gate
# RADIUS_CONSIST=1 (task#2, audit 半径一致化): feed EGO the SAME Delta=0 keep-out the cert gate
# enforces (r + D_SAFE_H + q0_cls) instead of the bare mover radius -- planning against bare r
# invites plans the gate must reject -> replan/evade churn. Default OFF = frozen behaviour.
CRET_GLIDE = os.environ.get("CRET_GLIDE", "0") == "1"   # task#7: on 'evade', try a certified RETIME
#   glide (SL.cert_clear_warp, slip identity) along a fresh straight plan before fleeing -- converts
#   mover-speed-limited freezes into certified slow progress. Default OFF = frozen behaviour.
CRET_GLIDE_SMIN = float(os.environ.get("CRET_GLIDE_SMIN", "0.10"))
VERDICT3_LOG = os.environ.get("VERDICT3_LOG", "0") == "1"
CALIB_V2 = os.environ.get("CALIB_V2", "0") == "1"
V_CAP = os.environ.get("V_CAP", "0") == "1"
PING = os.environ.get("PING", "0") == "1"   # 终末嗡鸣 v0: when a track is NEAR, split the tick into
#   3x0.1s sub-chunks -- re-perceive (variable-dt KF) + re-certify each chunk (delta=0.1 -> thinner
#   tube), re-decide ONLY on cert failure (event-driven). Perception must speed up WITH the cert
#   (re-anchoring without fresh observations would fake-shrink the tube -- the honesty rule).
PING_NEAR = float(os.environ.get("PING_NEAR", "5.0"))
PING_DECIDE = os.environ.get("PING_DECIDE", "0") == "1"   # v1: when near, re-DECIDE every sub-chunk
#   (fresh 0.1s anchor + thin delta at the tournament itself), not only on cert failure   # task#4: planner-side braking-envelope speed cap
#   v <= SL.v_cap(FOV_R, max_acc, DT) -- honest 'don't outrun the sensor' dial, default OFF   # FS3C-R era switch: SL loader (static/animal keys,
#   fail-closed semantics) + code-level static stationarity in cylinders. Flips WHOLESALE with the
#   new calib at Stage-C validation; default OFF = frozen benchmark behaviour.
DECIDE = os.environ.get("DECIDE", "v1")   # "v1" frozen tournament | "v2" unified direction-x-speed
#   grid (SL.maneuver_decide_v2: speed as a first-class dimension, built-in commitment; task#8)   # log the 3-valued cert verdict on evade
#   ticks into hist (DNF-seed diagnosis: UNKNOWN -> deepen budget; REFUTED -> genuinely boxed)
RADIUS_CONSIST = os.environ.get("RADIUS_CONSIST", "0")   # "0" off | "1" full (r+d_safe+q) | "dsafe" (r+d_safe only:
#   align the DETERMINISTIC standoff, leave the stochastic tube q to the gate -- full alignment with the
#   placeholder q=1.054 seals corridors at the PLANNING level (A/B 2026-07-07: evade 64->87, time +12s))
_RC_CAL = None


def _plan_r(r, cls=None):
    global _RC_CAL
    if RADIUS_CONSIST == "0":
        return r
    if RADIUS_CONSIST == "dsafe":
        return float(r) + SL.D_SAFE_H
    if _RC_CAL is None:
        _RC_CAL = SL.load_calib()
    q, _ve = _RC_CAL.get(cls, _RC_CAL["_all"]) if cls else _RC_CAL["_all"]
    return float(r) + SL.D_SAFE_H + q
CRUISE_Z = 1.5; Z_CEIL = 4.6
R_DRONE = 0.25
MEAS = 0.07
PHI = math.radians(25.0)
HORIZON = 7.5
MAXTICKS = 240
PRED_MODEL = os.environ.get("PRED_MODEL", "cv")   # deployed predictor (CV: tighter conformal keep-out, see calib)
FOV_R = 10.0          # FROZEN 2026-07-03: unified sensing range (= PERCEPT_RANGE = abr --fov_range).
#                       Was 14.0 -- the headless gt/native gate saw further than the frozen sensor claim.
#                       (perception/cert range: only movers within FOV_R are fed to EGO + certified (a TAU=0.75s,
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


_GP = None
def _ground(p_d, radius=18.0, step=1.0):
    """Flat ground patch (z=0) under the drone — SANDO needs an occupancy map each tick or it is 'not ready'."""
    global _GP
    if _GP is None:
        xs = np.arange(-radius, radius + 1e-6, step)
        gx, gy = np.meshgrid(xs, xs); m = (gx * gx + gy * gy) <= radius * radius
        _GP = np.column_stack([gx[m], gy[m]])
    out = np.zeros((_GP.shape[0], 3)); out[:, 0] = _GP[:, 0] + p_d[0]; out[:, 1] = _GP[:, 1] + p_d[1]
    return out


def _clearance(p, c_xy, r, h):
    horiz = math.hypot(p[0] - c_xy[0], p[1] - c_xy[1])
    if p[2] <= h:
        return horiz - r
    return math.hypot(max(0.0, horiz - r), p[2] - h)


def _rot(v2, ang):
    c, s = math.cos(ang), math.sin(ang)
    return np.array([c * v2[0] - s * v2[1], s * v2[0] + c * v2[1]])


def run_replay(movers, ep, mode="ours", calib=None, predict=True, max_vel=3.0, max_acc=6.0,
               cont_cert=True, n_sample=0, record=False, dynamics=False, flier=None, tick_cb=None):
    """One replay episode. cont_cert=True uses the continuous-time Bernstein cylinder cert; if False (ablation)
    the gate uses n_sample fixed-rate samples of the committed B-spline instead.
    dynamics=True flies the planned set-points through real QUADROTOR dynamics (tilt-to-accel, inertia, thrust
    limit) and measures clearance on the FLOWN position -- the renderer/PX4 reality (what flies != what's planned).
    With dynamics=False the drone is a perfect-tracking point mass (the optimistic headless number). Returns a dict.
    tick_cb(info): optional per-tick hook for live/3D visualisation -- called after each decision+step with
    dict(tick, t, p (LOCAL frame; add back org=midpoint(start,goal) for world), kind, clr). Keep it fast;
    it runs inside the control loop."""
    calib = calib or (SL.load_calib() if CALIB_V2 else load_calib())
    calib_v2 = SL.load_calib_v2() if CALIB_V2 else None
    # work in a LOCAL frame centred on the corridor midpoint: MetaUrban world coords span hundreds of metres,
    # so a global grid would be billions of voxels. Translate everything by -org -> a small local map suffices.
    org = 0.5 * (ep["start"][:2] + ep["goal"][:2])
    start = ep["start"].copy(); start[:2] -= org
    goal = ep["goal"].copy(); goal[:2] -= org
    t_base = ep["t0"]

    def pos_l(i, t):                                   # mover position in the local frame
        return movers.pos(i, t) - org

    ego = sn = None
    if mode == "sando":
        from sando_native_bridge import SandoNative                # native MIT-ACL SANDO (GUROBI); LD_LIBRARY_PATH req
        sn = SandoNative(overrides=dict(v_max=max_vel, a_max=max_acc, j_max=30.0,
                                        x_min=-60, x_max=60, y_min=-60, y_max=60, z_min=0.0, z_max=6.0,
                                        default_goal_z=CRUISE_Z, drone_radius=R_DRONE, goal_radius=0.8, horizon=8.0))
        sn.set_terminal_goal([float(goal[0]), float(goal[1]), CRUISE_Z])
    else:
        infl = 0.3 if mode == "native" else float(os.environ.get("REP_OURS_INFL", 0.45))
        ego = EGOPlanner(map_origin=(-40, -40, -1), map_size=(80, 80, 8), res=0.2, inflation=infl)
        if V_CAP:
            _vc = SL.v_cap(FOV_R, max_acc, DT)
            if _vc < max_vel:
                print(f"[v_cap] max_vel {max_vel:.1f} -> {_vc:.2f} (FOV_R={FOV_R}, a={max_acc})", flush=True)
            max_vel = min(max_vel, _vc)
        ego.set_params(max_vel=max_vel, max_acc=max_acc, horizon=HORIZON)
    last_rt = 0.0
    sando_path = None          # SANDO's last committed path; replan periodically + EXECUTE it (not replan every tick)
    trackers = {}                                   # mover idx -> MoverTracker (created on first detection)
    rng = np.random.default_rng(int(os.environ.get("PERCEPT_SEED", 1234567)))
    # ^ detection-noise stream follows PERCEPT_SEED in BOTH gt and realistic modes: otherwise the gt
    #   arms are deterministic and N "resamples" are one run copied N times (fake sample size).
    if PERCEPT_HARVEST is not None:
        _HARV_EP[0] += 1                                   # episode id for exchangeable-unit splitting
    # PERCEPT=realistic swaps OUR mover perception for the shared front-end (perception.py): FOV cone +
    # occlusion + distance miss/noise + NN-associated tracks with NO GT identity. Default stays "gt"
    # (omniscient control) until the B-bucket recalibration -- never silently change a headline's meaning.
    # Baselines (native/sando) keep their own perception either way: PERCEPT only governs "ours".
    percept_fe = None                                  # NB: name must not collide with `pf, vf = ...` below
    if os.environ.get("PERCEPT", "realistic") == "realistic" and mode in ("ours", "native"):
        # FROZEN 2026-07-03 (+7-04 baseline PERCEPTION PARITY: native can now consume the SAME
        # realistic is the DEFAULT deployment claim now; PERCEPT=gt is the explicit omniscient control.
        from perception import PerceptionFrontEnd, PerceptCfg
        # PERCEPT_SEED varies the sensor's random draws (miss/noise/clutter) so one scenario can be
        # RESAMPLED: perception outcomes are heavy-tailed, single-draw min_clr numbers are not citable.
        percept_fe = PerceptionFrontEnd(PerceptCfg.from_env(dt=DT),
                                        seed=int(os.environ.get("PERCEPT_SEED", 1234567)))

    p_d = start.copy(); v_d = np.zeros(3); a_d = np.zeros(3)
    _hd_cmd = [None]     # yaw-to-path: cone follows LAST tick's commanded set-point direction
    min_clr = 1e18; max_z = start[2]; reached = False
    counts = {k: 0 for k in ("straight", "around_l", "around_r", "over", "climb", "evade", "cret", "native", "sando")}
    _stick = {}                                      # SMOOTH=1 incumbent-maneuver state (kind/age)
    _v3st = {}                                        # DECIDE=v3 CPL incumbent (warm-start) state
    rta = dict(certified_ticks=0, violations=0)      # RTA failure rate: cert-passed tick followed by
    #                                                  a clearance violation within the SAME trust window
    hist = []
    best_d = 1e18; stall = 0                            # early-stop degenerate episodes (EGO can't plan -> evade spins)
    quad = Quadrotor() if dynamics else None            # real flight dynamics (what FLIES != what's planned)
    if quad is not None:
        quad.reset(start)
    track_err = []                                      # per-tick plan->flown deviation (to calibrate the margin)

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
        p_ref, v_ref, a_ref = p_d.copy(), np.zeros(3), np.zeros(3)   # the set-point this tick (flown via quad if dynamics)

        if mode == "native":
            cloud = []
            if percept_fe is not None:
                # PERCEPTION PARITY: native sees the SAME realistic front-end tracks ours does
                # (cone/occlusion/miss/NN association) -- the fair native_real_dyn arm.
                hd = v_d[:2] if float(np.hypot(v_d[0], v_d[1])) > 0.3 else gdir
                gt_cyl = [(pos_l(i, t), movers.m[i]["r"], movers.m[i]["h"], movers.m[i]["cls"])
                          for i in idx]
                for tr in percept_fe.step(p_d[:2], hd, gt_cyl):
                    cloud += _cyl_cloud([tr.xy[:2]], _plan_r(tr.r, str(tr.cls)), 0.3, tr.h)
            else:
                for i in near:
                    cloud += _cyl_cloud([dets[i][:2]], _plan_r(movers.m[i]["r"], movers.m[i]["cls"]),
                                        0.3, movers.m[i]["h"])
            ego.update_cloud(np.asarray(cloud, float) if cloud else np.zeros((0, 3)), p_d)
            if ego.replan(p_d, v_d, a_d, goal) and ego.duration() > 1e-3:
                r = ego.eval(min(DT, max(ego.duration() - 1e-3, 0.0)))
                if r is not None:
                    p_ref, v_ref, a_ref = (np.asarray(x, float) for x in r)
            counts["native"] += 1; kind = "native"
        elif mode == "sando":
            # native MIT-ACL SANDO baseline: feed current state + each mover as an analytic linear DynTraj (the
            # KF position+velocity it perceives), then heat-A* + DecompUtil SFC + GUROBI local solve. SANDO runs
            # its OWN prediction/avoidance on the trajectories; we read back its committed next set-point. SANDO
            # is time-aware via the t argument, so advancing t by DT each tick paces its plan. No cert (baseline).
            import time as _time
            t_loc = tick * DT
            sn.update_state(p_d, v_d, a_d, float(math.atan2(gdir[1], gdir[0])))
            # replan every 5 ticks (1.5 s) and EXECUTE the committed path in between; replanning every tick made
            # SANDO re-decide constantly and wander (dgoal oscillated). Periodic replan = stable plan to follow.
            if tick % 5 == 0 or sando_path is None or len(sando_path) < 2:
                for i in near:
                    c = dets[i]; cls = movers.m[i]["cls"]; rr = movers.m[i]["r"]; hh = movers.m[i]["h"]
                    vv = trackers[i].state()[1] if trackers[i].ready else np.zeros(3)
                    bb = (rr + R_DRONE + D_SAFE_H, rr + R_DRONE + D_SAFE_H, hh)
                    sn.add_traj(i, bb, tx=f"{c[0]:.3f}+({vv[0]:.3f})*(t-({t_loc:.3f}))",
                                ty=f"{c[1]:.3f}+({vv[1]:.3f})*(t-({t_loc:.3f}))", tz="1.5",
                                vx=f"{vv[0]:.3f}", vy=f"{vv[1]:.3f}", vz="0", is_agent=(cls == "pedestrian"), t=t_loc)
                sn.update_occupancy(_ground(p_d), t_loc)      # SANDO needs an occupancy map to be 'ready'
                sn.clean_old_trajs(t_loc)
                _t0 = _time.perf_counter()
                sn.replan(last_rt, t_loc); last_rt = _time.perf_counter() - _t0
                sando_path = sn.get_setpoints()
            # FLY the committed path at cruise speed: walk max_vel*DT of arc-length from the nearest point
            # (get_next_goal only advances ~0.05 m/call -> crawl). SANDO = path planner here; a fair DT-step.
            sp = sando_path
            if sp is not None and len(sp) >= 2:
                d = np.linalg.norm(sp[:, :2] - p_d[:2], axis=1); j = int(np.argmin(d))
                acc = 0.0; tgt = max_vel * DT
                while j < len(sp) - 1 and acc < tgt:
                    acc += float(np.linalg.norm(sp[j + 1, :2] - sp[j, :2])); j += 1
                # cap the per-tick step at max_vel*DT so an erratic/looping committed path can't fling the drone
                step = np.asarray(sp[j], float)[:2] - p_d[:2]; ns = float(np.linalg.norm(step))
                if ns > max_vel * DT:
                    step = step * (max_vel * DT / ns); ns = max_vel * DT
                p_ref = np.array([p_d[0] + step[0], p_d[1] + step[1], CRUISE_Z])
                v_ref = np.array([step[0] / DT, step[1] / DT, 0.0]) if ns > 1e-6 else np.zeros(3)
                a_ref = np.zeros(3)
            else:
                okn, ng = sn.get_next_goal()
                if okn:
                    p_ref, v_ref, a_ref = (np.asarray(ng[0], float), np.asarray(ng[1], float), np.asarray(ng[2], float))
            counts["sando"] += 1; kind = "sando"
        else:
            # WHAT ours perceives this tick: (tracker, last_xy, r, h, cls) per perceived mover.
            #   gt (default): omniscient control -- trackers keyed by GT index, plain range gate.
            #   realistic:    shared front-end tracks -- cone + occlusion + miss/noise + NN association.
            if percept_fe is not None:
                # YAW-TO-PATH (2026-07-04): the cone follows where the drone is GOING, not its
                # instantaneous velocity. During aggressive detours velocity-aligned heading points
                # the sensor AWAY from the swept region -> blind lateral entry (props_alley seed0:
                # 0 detections for 15 ticks while arcing into an unseen canopy at 5 m/s). Real
                # quads yaw toward the path for exactly this reason.
                hd = _hd_cmd[0] if _hd_cmd[0] is not None else (v_d[:2] if float(np.hypot(v_d[0], v_d[1])) > 0.3 else gdir)
                gt_cyl = [(pos_l(i, t), movers.m[i]["r"], movers.m[i]["h"], movers.m[i]["cls"])
                          for i in idx]
                ptracks = percept_fe.step(p_d[:2], hd, gt_cyl)
                percepts = [(tr.trk, tr.xy, tr.r, tr.h, tr.cls) for tr in ptracks]
                if PERCEPT_A2 is not None:
                    # A2 presence-miss, FUTURE-REACH qualification (theta2 2026-07-07): a mover is
                    # dangerous at tick t iff its ACTUAL GT future enters the drone's reachable ball
                    # within the horizon -- the isotropic rho_c counted receding 11 m/s vehicles as
                    # "missed danger" (91% of flights, vacuous ledger). Sound for the lemma: any
                    # actual culprit trivially satisfies it at the last certified tick.
                    _rxy = [np.asarray(tr.xy[:2], float) for tr in ptracks if tr.trk.ready]
                    _anyq = False

                    def _qual(_i):
                        _r = movers.m[_i]["r"]
                        for _dh in (0.0, 0.35, 0.70, 1.05):
                            if not movers.present(_i, t + _dh):
                                continue
                            if float(np.hypot(*(pos_l(_i, t + _dh) - p_d[:2]))) <=                                     3.0 * _dh + float(_r) + 0.35 + 0.50:
                                return True
                        return False

                    _QUAL_MEMO.clear()
                    for _i in idx:
                        _QUAL_MEMO[_i] = _qual(_i)
                        if not _QUAL_MEMO[_i]:
                            continue
                        _anyq = True
                        _gp = pos_l(_i, t)
                        if not any(float(np.hypot(*(_gp - _q))) < 2.0 for _q in _rxy):
                            PERCEPT_A2["miss_ticks"] += 1
                    if _anyq:
                        PERCEPT_A2["qual_ticks"] += 1
                if PERCEPT_HARVEST is not None:
                    # DEPLOYMENT-IN-THE-LOOP residual harvest (B-bucket CRITICAL#1): score the DEPLOYED
                    # tracks' predictions against the nearest GT mover's true future -- association error,
                    # intermittency and coast are all part of the residual, exactly as flown.
                    for tr in ptracks:
                        if not tr.trk.ready:
                            continue
                        # associate the track to its GT mover NOW (gated): ghost/expired tracks must
                        # not be scored against some unrelated far-away mover's future.
                        gi, gd, g2 = None, 1.0, 1e9
                        for i in idx:
                            dd = float(np.hypot(*(pos_l(i, t) - tr.xy[:2])))
                            if dd < gd:
                                gi, g2, gd = i, gd, dd
                            elif dd < g2:
                                g2 = dd
                        if gi is None or g2 < gd + 0.5:
                            continue                        # ambiguous association (ID-swap risk):
                            # that failure mode belongs to the multiplicity layer, not this residual law
                        d_drone = float(np.hypot(*(tr.xy[:2] - p_d[:2])))   # BINDING-REGION field:
                        # only movers close enough to collide within the trust window can make a
                        # certified tick unsafe -- the composition theorem's sup runs over these rows
                        for dh in _HARV_DELTAS:
                            if not movers.present(gi, t + dh):
                                continue                    # mover leaves the world: nothing to predict
                            if str(tr.cls) == "static" or (_HARV_V2 and int(tr.trk.n) < _HARV_AGEMIN):
                                # STATIC-STATIONARY predictor (2026-07-07): a static's future = its
                                # present. Scoring statics with the CV extrapolation charged them
                                # for KF velocity noise -> mover-sized tubes -> keep-out walls.
                                # Calibration must match deployment: PERCLASS=1 deploys the same v=0.
                                c0s, _v, _a = tr.trk.state()
                                pred = np.asarray(c0s[:2], float)
                            else:
                                pred = tr.trk.predict([dh], model=PRED_MODEL)[0, :2]
                            resid = float(np.hypot(*(pos_l(gi, t + dh) - pred)))
                            PERCEPT_HARVEST.append((float(dh), resid, int(tr.trk.n),
                                                    str(tr.cls), int(_HARV_EP[0]), d_drone)
                                                   + ((_HARV_SCN[0], int(_QUAL_MEMO.get(gi, True)),
                                                       int(tr.trk.miss > 0),          # coast flag (theta3)
                                                       float(getattr(tr.trk, "sigma_v", 0.0)))
                                                      if _HARV_V2 else ()))
            else:
                percepts = [(trackers[i], dets[i], movers.m[i]["r"], movers.m[i]["h"], movers.m[i]["cls"])
                            for i in near]
            # near-term predicted cloud for EGO's grid
            cloud = []
            _cpa = os.environ.get("CPA_CLOUD", "0") == "1"
            for (trk, dxy, r_o, h_o, _c) in percepts:
                if _cpa and trk.ready and int(getattr(trk, 'n', 0)) >= 4:
                    # ANTICIPATORY OCCUPANCY -- MATURE TRACKS ONLY (vehicle_spawn_accel autopsy
                    # 2026-07-08: a just-spawned accelerating vehicle has a stale-low KF velocity;
                    # CPA placement trusted it and threaded the plan into its acceleration path.
                    # Anticipate only agents whose running logic is CONVERGED; newborns keep the
                    # conservative current-position block): block the
                    # mover where it WILL BE at closest approach of the relative motion -- the
                    # planner threads the predicted gap instead of dodging the past; the
                    # certificate still judges the true tubes (safety semantics untouched).
                    _c0, _vv, _ = trk.state()
                    _sp = float(np.hypot(v_d[0], v_d[1]))
                    _gd = (np.asarray(v_d[:2], float) / _sp) if _sp > 0.3 else \
                        (goal[:2] - p_d[:2]) / max(np.linalg.norm(goal[:2] - p_d[:2]), 1e-6)
                    _dp = np.asarray(_c0[:2], float) - np.asarray(p_d[:2], float)
                    _dv = np.asarray(_vv[:2], float) - _gd * max_vel
                    _dvn = float(_dv @ _dv)
                    _tc = float(np.clip(-(_dp @ _dv) / _dvn, 0.0, 2.5)) if _dvn > 1e-6 else 0.0
                    xy = (np.asarray(_c0[:2], float) + np.asarray(_vv[:2], float) * _tc)[None, :]
                else:
                    xy = trk.predict(np.linspace(0, DT, 2), model=PRED_MODEL)[:, :2] if trk.ready \
                        else np.asarray(dxy, float)[None, :2]
                cloud += _cyl_cloud(xy, _plan_r(r_o, str(_c)), 0.3, h_o)
            ego.update_cloud(np.asarray(cloud, float) if cloud else np.zeros((0, 3)), p_d)

            # safety_layer decision (NB: the renderer still runs its OWN tournament copy in render_3d_video.py
            # ~L1148 with extra static/flown gates -- the two are behaviourally aligned on the climb cert gate
            # since 2026-07-02 but NOT the same code): conformal per-class keep-out + tournament + evade.
            mlist = []
            for (trk, _d, r_o, h_o, cls_o) in percepts:
                c0, vv, aa = trk.state()
                if PRED_MODEL == "cv":
                    aa = np.zeros(3)                     # CV deployment: cert polynomial matches the CV-calibrated tube
                mlist.append((c0, vv, aa, r_o, h_o, cls_o, int(getattr(trk, "n", 99)),
                              int(getattr(trk, "miss", 0) > 0),
                              float(getattr(trk, "nis_ewma", 0.0)), float(getattr(trk, "sigma_v", 0.0))))
            cyl, ztop = SL.build_cylinders(mlist, calib, predict=predict,
                                           track_margin=(float(os.environ.get("DYN_TRACK", "0.473"))
                                                         if dynamics else 0.0),
                                           calib_v2=calib_v2)
            if cont_cert:
                clear_fn = lambda: SL.cert_clear(ego, cyl, tau=TAU, delta=DELTA)
            else:
                def clear_fn():                          # discrete-sampling gate (ablation): n_sample points only
                    dur = ego.duration()
                    for (c0, vv, aa, R, zc, veff) in cyl:
                        for s in np.linspace(0.0, min(TAU, dur), max(2, n_sample)):
                            r = ego.eval(s)
                            if r is None:
                                continue
                            pp = r[0]; cc = c0 + vv * s + 0.5 * aa * s * s; rho = R + veff * (s + DELTA)
                            if not (math.hypot(pp[0] - cc[0], pp[1] - cc[1]) >= rho or pp[2] >= zc):
                                return False
                    return True
            _v2s = 1.0
            if DECIDE == "v3" and cont_cert:
                # CPL-v3: certified LOCAL TRAJECTORY planner -- plan INSIDE the certified set (no
                # discrete tournament/arbitration). EGO supplies the GLOBAL guide candidate (escapes
                # local minima); last tick's winner is the warm-start incumbent (temporal
                # consistency -> churn dies). Returns a composite plan or None -> fallback.
                import local_lattice as _LL
                _guide = None
                if ego.replan(p_d, v_d, a_d, goal) and ego.duration() > 1e-3:
                    _ge = ego.eval(min(_LL.T_P, ego.duration() - 1e-3))
                    if _ge is not None:
                        _guide = _LL.quintic3(p_d, v_d, a_d, np.asarray(_ge[0], float),
                                              np.asarray(_ge[1], float), np.zeros(3), _LL.T_P)
                _plan, _prim, _tag, _diag = _LL.plan_local(
                    p_d, v_d, a_d, goal, ztop, cyl, v_max=max_vel, a_max=max_acc, dt=DT, delta=DELTA,
                    incumbent=_v3st.get("prim"), guide=_guide)
                if _plan is not None:
                    p_ref, v_ref, a_ref = _LL.plan_eval(_plan, DT)
                    _v3st["prim"] = _prim                   # warm-start next tick
                    kind = "cpl"
                else:
                    # FALLBACK L1 (blueprint 5): nothing in the lattice certified -> fly a CERTIFIED
                    # brake from the current state. The jerk-limited brake is SMOOTH (continuous
                    # deceleration, not a discrete hold) AND sound (re-certified this tick). Only if
                    # even the brake fails to certify do we evade (L3). This resolves the
                    # continuity-vs-safety tension: smooth AND safe.
                    _v3st.pop("prim", None)
                    _bsegs, _bdurs, _btc = _LL.make_composite(
                        _LL.quintic3(p_d, v_d, a_d, p_d, v_d * 0.0, np.zeros(3), _LL.T_P), max_acc, DT)
                    _bok, _ = _LL.certify_composite(_bsegs, _bdurs, cyl, _btc, DELTA)
                    if _bok:
                        p_ref, v_ref, a_ref = _LL.plan_eval((_bsegs, _bdurs, _btc), DT)
                        kind = "brake"
                    else:
                        kind = "evade"                      # even braking uncertifiable -> flee (L3)
                counts["cpl_cert"] = counts.get("cpl_cert", 0) + _diag["n_cert"]
            elif DECIDE == "v2" and cont_cert:
                kind, _v2s = SL.maneuver_decide_v2(ego, p_d, v_d, a_d, goal, ztop, cyl, _stick,
                                                   cruise_z=CRUISE_Z, horizon=HORIZON, delta=DELTA)
                if kind in ("around_l2", "around_r2"):
                    kind = kind[:-1]                        # counts/HUD keep the l/r bucket names
            elif SMOOTH and cont_cert:
                _strict = lambda: SL.cert_clear(ego, cyl, tau=TAU, delta=DELTA + SMOOTH_MARGIN)
                kind = SL.maneuver_decide_sticky(ego, p_d, v_d, a_d, goal, ztop, clear_fn, _stick,
                                                 cruise_z=CRUISE_Z, horizon=HORIZON,
                                                 dwell_ticks=SMOOTH_DWELL, clear_fn_strict=_strict)
            else:
                kind = SL.maneuver_decide(ego, p_d, v_d, a_d, goal, ztop, clear_fn, cruise_z=CRUISE_Z, horizon=HORIZON)
            if kind in ("cpl", "brake"):
                pass                                        # p_ref/v_ref/a_ref already set (plan_eval / v3 brake)
            elif kind != "evade":
                rr = ego.eval(min(_v2s * DT, max(ego.duration() - 1e-3, 0.0)))
                if rr is not None:
                    p_ref = np.asarray(rr[0], float)
                    v_ref = _v2s * np.asarray(rr[1], float)
                    a_ref = _v2s * _v2s * np.asarray(rr[2], float)
                if (PING and cont_cert and percept_fe is not None and not dynamics and flier is None
                        and kind != "evade"):
                    _near = any(float(np.hypot(*(np.asarray(tr.xy[:2], float) - p_d[:2]))) < PING_NEAR
                                for tr in ptracks if tr.trk.ready)
                    if _near:
                        # terminal buzz: fly the tick in 3 sub-chunks with fresh perception + re-cert
                        _sub = DT / 3.0
                        _p_sub = np.asarray(p_d, float).copy()
                        for _j in (1, 2):
                            _rrj = ego.eval(min(_v2s * _sub * _j, max(ego.duration() - 1e-3, 0.0)))
                            if _rrj is not None:
                                _p_sub = np.asarray(_rrj[0], float)
                            _tj = t + _sub * _j
                            _gt_j = [(pos_l(i, _tj), movers.m[i]["r"], movers.m[i]["h"],
                                      movers.m[i]["cls"]) for i in idx if movers.present(i, _tj)]
                            _hd_j = _hd_cmd[0] if _hd_cmd[0] is not None else gdir
                            _ptr_j = percept_fe.step(_p_sub[:2], _hd_j, _gt_j, dt=_sub)
                            _ml_j = []
                            for _tr in _ptr_j:
                                if not _tr.trk.ready:
                                    continue
                                _c0j, _vvj, _aaj = _tr.trk.state()
                                if PRED_MODEL == "cv":
                                    _aaj = np.zeros(3)
                                _ml_j.append((_c0j, _vvj, _aaj, _tr.r, _tr.h, _tr.cls,
                                              int(_tr.trk.n), int(_tr.trk.miss > 0)))
                            _cyl_j, _zt_j = SL.build_cylinders(_ml_j, calib, predict=predict,
                                                               calib_v2=calib_v2)
                            if PING_DECIDE or not SL.cert_clear(ego, _cyl_j, tau=TAU, delta=_sub):
                                # v1: proactive sub-cadence decision / v0: cert broke mid-tick
                                kind, _v2s = SL.maneuver_decide_v2(
                                    ego, _p_sub, v_ref, a_ref, goal, _zt_j, _cyl_j, _stick,
                                    cruise_z=CRUISE_Z, horizon=HORIZON, delta=_sub)
                                if kind in ("around_l2", "around_r2"):
                                    kind = kind[:-1]
                                if kind != "evade":
                                    _rrn = ego.eval(min(_v2s * (DT - _sub * _j),
                                                        max(ego.duration() - 1e-3, 0.0)))
                                    if _rrn is not None:
                                        p_ref = np.asarray(_rrn[0], float)
                                        v_ref = _v2s * np.asarray(_rrn[1], float)
                                        a_ref = _v2s * _v2s * np.asarray(_rrn[2], float)
                                break
            elif idx:
                _glid = False
                if CRET_GLIDE and cont_cert:
                    # CRET-glide: the tournament certified candidates at FULL speed only. Replan
                    # straight and scan certified retimes s<1 (slip identity) -- a certified crawl
                    # toward the goal beats a blind flee. Statics are warp-invariant, so this only
                    # converts mover-speed-limited freezes (the honest population).
                    gsub = np.array([goal[0], goal[1], CRUISE_Z])
                    if ego.replan(p_d, v_d, a_d, gsub) and ego.duration() > 1e-3:
                        for _s in (0.8, 0.6, 0.4, 0.25, CRET_GLIDE_SMIN):
                            if _s < CRET_GLIDE_SMIN - 1e-9:
                                break
                            if SL.cert_clear_warp(ego, cyl, _s, tau=TAU, delta=DELTA):
                                rr = ego.eval(min(_s * DT, max(ego.duration() - 1e-3, 0.0)))
                                if rr is not None:
                                    p_ref = np.asarray(rr[0], float)
                                    v_ref = _s * np.asarray(rr[1], float)
                                    a_ref = _s * _s * np.asarray(rr[2], float)
                                    kind = "cret"; _glid = True
                                break
                if not _glid:
                    # realistic mode flees only what it TRACKS (fleeing an unseen mover would be
                    # omniscient); with nothing tracked evade_setpoint falls back to fleeing along gdir.
                    flee = ([tuple(d) for (_t, d, *_r) in percepts] if percept_fe is not None
                            else [dets[i] for i in idx])
                    if os.environ.get("EVADE_BLEND", "0") == "1":
                        pos, vel = SL.evade_setpoint_blend(p_d, v_d, flee, max_vel, DT, gdir)
                    else:
                        pos, vel = SL.evade_setpoint(p_d, flee, max_vel, DT, ztop, gdir)
                    p_ref, v_ref, a_ref = pos, vel, np.zeros(3)
            counts[kind] = counts.get(kind, 0) + 1

        # apply the set-point: real PX4 SITL (flier) > local quadrotor model (dynamics) > teleport (optimistic)
        if flier is not None:                              # real PX4 SITL in the loop (flier streams the set-point)
            pf, vf = flier(p_ref, v_ref, a_ref, DT)
            track_err.append(float(np.linalg.norm(np.asarray(pf)[:2] - p_ref[:2])))
            p_d, v_d, a_d = np.asarray(pf, float), np.asarray(vf, float), np.zeros(3)
        elif dynamics:
            pf, vf = quad.step(p_ref, v_ref, a_ref, DT)
            track_err.append(float(np.linalg.norm(np.asarray(pf)[:2] - p_ref[:2])))
            p_d, v_d, a_d = np.asarray(pf, float), np.asarray(vf, float), quad.a.copy()
        else:
            p_d, v_d, a_d = np.asarray(p_ref, float), np.asarray(v_ref, float), np.asarray(a_ref, float)

        max_z = max(max_z, float(p_d[2]))
        tick_clr = 1e18
        for i in present_idx(t):
            cl = _clearance(p_d, pos_l(i, t), movers.m[i]["r"], movers.m[i]["h"])
            tick_clr = min(tick_clr, cl); min_clr = min(min_clr, cl)
        _dcmd = np.asarray(p_ref, float)[:2] - p_d[:2]
        if float(np.hypot(*_dcmd)) > 0.15:
            _hd_cmd[0] = _dcmd.copy()      # look where you are COMMANDED to go (real quads yaw-to-path)
        if kind in ("straight", "around_l", "around_r", "over", "climb", "cret"):
            rta["certified_ticks"] += 1
            if tick_clr < 1e17 and tick_clr < 0.0:
                rta["violations"] += 1               # flew a CERT-PASSED plan into a violation
        if tick_cb is not None:
            tick_cb(dict(tick=tick, t=t, p=p_d.copy(), kind=kind,
                         clr=(tick_clr if tick_clr < 1e17 else None)))
        if record:
            _v3 = None
            if VERDICT3_LOG and kind == "evade" and cont_cert:
                try:
                    _v3 = SL.cert_verdict3(ego, cyl, tau=TAU, delta=DELTA)
                except Exception:
                    _v3 = "err"
            hist.append(dict(tick=tick, t=round(t, 2),
                             clr=round(tick_clr, 3) if tick_clr < 1e17 else None,
                             kind=kind, z=round(float(p_d[2]), 2),
                             p=[round(float(p_d[0]), 2), round(float(p_d[1]), 2)],
                             pref=[round(float(p_ref[0]), 2), round(float(p_ref[1]), 2)],
                             dgoal=round(float(np.linalg.norm(p_d[:2] - goal[:2])), 2),
                             a=round(float(np.linalg.norm(a_d)), 4), v=round(float(np.linalg.norm(v_d)), 4),
                             ax=round(float(a_d[0]), 4), ay=round(float(a_d[1]), 4)))
            if _v3 is not None:
                hist[-1]["v3"] = _v3                   # only stamped when VERDICT3_LOG fires: default hist shape frozen
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
                collided=(min_clr < -1e-6), max_z=max_z, counts=counts, dynamics=dynamics,
                track_err_med=float(np.median(track_err)) if track_err else 0.0,
                track_err_max=float(np.max(track_err)) if track_err else 0.0,
                predict=predict, cont_cert=cont_cert, rta=rta, history=hist if record else None)


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
