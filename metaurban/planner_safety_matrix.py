"""planner_safety_matrix — headless A/B of SEVERAL planners WITH vs WITHOUT our certified safety layer.

The safety layer (KF prediction + continuous-time Bernstein cylinder certificate + fastest-safe maneuver
tournament) is PLANNER-AGNOSTIC: it gates whatever trajectory the planner commits. This harness flies each
planner through the SAME harvested-GT crossing corridors (replay_core episodes; NO d435i, NO render -- pure CPU)
in two modes:

  safety=off : commit the planner's straight-to-goal trajectory and FLY it (no prediction, no cert) -> reckless.
  safety=on  : each tick run the fastest-safe tournament (straight / around +-25/50 / over / climb), CERTIFY each
               candidate against every KF-predicted mover with the conformal per-class keep-out, FLY the
               goal-ward certified one; HOLD/evade if none certifies -> P(collision) <= eps.

Planners (each its OWN trajectory representation, all gated by the ONE certificate):
  ego        : ZJU EGO-Planner (cubic B-spline, ESDF gradient avoidance) -- the real closed-loop planner.
  quintic    : RapidQuadrocopterTrajectories min-jerk QUINTIC to goal (no avoidance front-end; power basis).
  septic     : min-snap SEPTIC to goal (mav_trajectory_generation style; deg-7 power basis).
  bspline    : a plain cubic B-spline to goal (Fast-Planner representation, no optimisation).
The non-EGO planners have NO obstacle avoidance of their own; with safety=off they fly the smooth arc straight
through movers (so the certificate is the ONLY thing keeping them safe -- the cleanest planner-agnostic claim).

Run one chunk (one planner, one safety setting) as a FRESH process (sidesteps EGOPlanner's per-episode grid leak)
and APPEND its rows to the CSV:
  python metaurban/planner_safety_matrix.py --planner ego --safety on  --seeds 0-9 --n_ep 6 --csv out/conformal/planner_safety.csv
"""
import os, sys, math, json, glob, argparse, csv as _csv
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import replay_core as RC
from cert_bridge import Certifier, monomial_to_bseg, bspline_to_bseg, _bezier_eval
from graft_demo import rapidquad_quintic, minsnap_septic
from ego_bridge import EGOPlanner
from kf_tracker import MoverTracker
from quadrotor import Quadrotor

TAU, DT, DELTA = RC.TAU, RC.DT, RC.DELTA
CRUISE_Z, Z_CEIL, R_DRONE = RC.CRUISE_Z, RC.Z_CEIL, RC.R_DRONE
D_SAFE_H, D_SAFE_V, REACH_PAD = RC.D_SAFE_H, RC.D_SAFE_V, RC.REACH_PAD
FOV_R, HORIZON, MAXTICKS, MEAS, PHI = RC.FOV_R, RC.HORIZON, RC.MAXTICKS, RC.MEAS, RC.PHI
PRED_MODEL = RC.PRED_MODEL


# ----------------------------------------------------------------------------------------------------------------
# planner adapters: uniform interface plan()/duration()/eval()/cert_h()/cert_a()/update_cloud()/reset()
# ----------------------------------------------------------------------------------------------------------------
class EgoP:
    name = "ego"
    def __init__(self, max_vel, max_acc):
        self.max_vel = max_vel
        self.ego = EGOPlanner(map_origin=(-40, -40, -1), map_size=(80, 80, 8), res=0.2,
                              inflation=float(os.environ.get("REP_OURS_INFL", 0.45)))
        self.ego.set_params(max_vel=max_vel, max_acc=max_acc, horizon=HORIZON)
    def reset(self): pass
    def update_cloud(self, cloud, p): self.ego.update_cloud(cloud, p)
    def plan(self, p, v, a, goal): return bool(self.ego.replan(p, v, a, goal)) and self.ego.duration() > 1e-3
    def duration(self): return self.ego.duration()
    def eval(self, t): return self.ego.eval(t)
    def cert_h(self, c0, R, vel, acc, t_hi, v_eff, delta):
        return self.ego.certify_horizontal(obs_c0=c0, R=R, obs_vel=vel, obs_acc=acc, t_hi=t_hi, v_eff=v_eff, delta=delta)
    def cert_a(self, z_clear, t_hi, delta):
        return self.ego.certify_above(z_clear=z_clear, t_hi=t_hi, v_eff_z=0.0, delta=delta)


class PolyP:
    """Analytic single-segment polynomial planner (min-jerk quintic / min-snap septic) to the goal. No avoidance."""
    def __init__(self, name, max_vel, kind):
        self.name = name; self.max_vel = max_vel; self.kind = kind
        self._coef = None; self._T = 0.6; self._bseg = None
    def reset(self): self._coef = None
    def update_cloud(self, cloud, p): pass
    def plan(self, p, v, a, goal):
        p = np.asarray(p, float); v = np.asarray(v, float); a = np.asarray(a, float); goal = np.asarray(goal, float)
        T = max(float(np.linalg.norm(goal - p)) / max(self.max_vel, 0.5), 0.6)
        if self.kind == "quintic":
            self._coef = rapidquad_quintic(p, v, a, goal, [0, 0, 0], [0, 0, 0], T)          # (6,3) ascending
        else:
            self._coef = minsnap_septic(p, v, a, [0, 0, 0], goal, [0, 0, 0], [0, 0, 0], [0, 0, 0], T)  # (8,3)
        self._T = T
        self._bseg = monomial_to_bseg(self._coef[None], [T])
        return True
    def duration(self): return self._T
    def eval(self, t):
        t = min(max(float(t), 0.0), self._T); c = self._coef; n = c.shape[0]; ks = np.arange(n)
        pos = np.array([float(np.sum(c[:, d] * t ** ks)) for d in range(3)])
        vel = np.array([float(np.sum(ks[1:] * c[1:, d] * t ** (ks[1:] - 1))) for d in range(3)])
        acc = np.array([float(np.sum(ks[2:] * (ks[2:] - 1) * c[2:, d] * t ** (ks[2:] - 2))) for d in range(3)])
        return pos, vel, acc
    def cert_h(self, c0, R, vel, acc, t_hi, v_eff, delta):
        cp, t0, du = self._bseg
        return Certifier.certify_horizontal(cp, t0, du, [c0[0], c0[1], c0[2]], R, vel=vel, acc=acc,
                                            t_hi=t_hi, v_eff=v_eff, delta=delta, n_axes=2)
    def cert_a(self, z_clear, t_hi, delta):
        cp, t0, du = self._bseg
        return Certifier.certify_above(cp, t0, du, z_clear, t_hi=t_hi, v_eff_z=0.0, delta=delta)


class BSplineP:
    """Plain cubic B-spline straight to the goal (Fast-Planner representation, no optimisation / avoidance)."""
    name = "bspline"
    def __init__(self, max_vel, _max_acc=None):
        self.max_vel = max_vel; self._bez = None; self._t0 = None; self._du = None; self._T = 0.6
    def reset(self): self._bez = None
    def update_cloud(self, cloud, p): pass
    def plan(self, p, v, a, goal):
        p = np.asarray(p, float); goal = np.asarray(goal, float); d = goal - p
        ctrl = np.array([p, p + d / 4, p + d / 2, p + 3 * d / 4, goal, goal], float)        # 6 ctrl -> 3 cubic segs
        T = max(float(np.linalg.norm(d)) / max(self.max_vel, 0.5), 0.6)
        n_seg = ctrl.shape[0] - 3
        self._bez, self._t0, self._du = bspline_to_bseg(ctrl, [T / n_seg] * n_seg)
        self._T = float(np.sum(self._du))
        return True
    def duration(self): return self._T
    def _eval_pos(self, t):
        t = min(max(float(t), 0.0), self._T)
        i = int(np.searchsorted(np.append(self._t0, self._T), t) - 1); i = max(0, min(i, len(self._bez) - 1))
        s = (t - self._t0[i]) / max(self._du[i], 1e-9)
        return np.asarray(_bezier_eval(self._bez[i], min(max(s, 0.0), 1.0)), float)
    def eval(self, t):
        h = 1e-3; pos = self._eval_pos(t)
        vel = (self._eval_pos(t + h) - self._eval_pos(max(t - h, 0.0))) / (2 * h)
        acc = (self._eval_pos(t + h) - 2 * pos + self._eval_pos(max(t - h, 0.0))) / (h * h)
        return pos, vel, acc
    def cert_h(self, c0, R, vel, acc, t_hi, v_eff, delta):
        return Certifier.certify_horizontal(self._bez, self._t0, self._du, [c0[0], c0[1], c0[2]], R,
                                            vel=vel, acc=acc, t_hi=t_hi, v_eff=v_eff, delta=delta, n_axes=2)
    def cert_a(self, z_clear, t_hi, delta):
        return Certifier.certify_above(self._bez, self._t0, self._du, z_clear, t_hi=t_hi, v_eff_z=0.0, delta=delta)


def make_planner(name, max_vel, max_acc):
    if name == "ego": return EgoP(max_vel, max_acc)
    if name == "quintic": return PolyP("quintic", max_vel, "quintic")
    if name == "septic": return PolyP("septic", max_vel, "septic")
    if name == "bspline": return BSplineP(max_vel)
    raise ValueError(name)


# ----------------------------------------------------------------------------------------------------------------
# one episode, one planner, safety on/off
# ----------------------------------------------------------------------------------------------------------------
def run_episode(movers, ep, planner, safety, calib, max_vel=3.0, predict=True, dynamics=False):
    org = 0.5 * (ep["start"][:2] + ep["goal"][:2])
    start = ep["start"].copy(); start[:2] -= org
    goal = ep["goal"].copy(); goal[:2] -= org
    t_base = ep["t0"]
    pos_l = lambda i, t: movers.pos(i, t) - org

    planner.reset()
    p_d = start.copy(); v_d = np.zeros(3); a_d = np.zeros(3)
    min_clr = 1e18; max_z = start[2]; reached = False; n_hold = 0
    trackers = {}; rng = np.random.default_rng(1234567)
    best_d = 1e18; stall = 0
    quad = Quadrotor() if dynamics else None
    if quad is not None: quad.reset(start)
    present_idx = lambda t: [i for i in range(len(movers.m)) if movers.present(i, t)]

    for tick in range(MAXTICKS):
        t = t_base + tick * DT
        idx = present_idx(t); dets = {}
        for i in idx:
            det = pos_l(i, t) + rng.normal(0, MEAS, 2); dets[i] = det
            trackers.setdefault(i, MoverTracker(dt=DT, meas_noise=MEAS)).update([det[0], det[1], 1.5])
        near = [i for i in idx if np.linalg.norm(dets[i] - p_d[:2]) < FOV_R]
        gxy = goal[:2] - p_d[:2]; dist = float(np.linalg.norm(gxy))
        gdir = gxy / dist if dist > 1e-6 else np.array([1.0, 0.0])

        # predicted near-term cloud for EGO's ESDF (grafts ignore it)
        cloud = []
        for i in near:
            trk = trackers[i]
            xy = trk.predict(np.linspace(0, DT, 2), model=PRED_MODEL)[:, :2] if trk.ready else dets[i][None, :2]
            cloud += RC._cyl_cloud(xy, movers.m[i]["r"], 0.3, movers.m[i]["h"])
        planner.update_cloud(np.asarray(cloud, float) if cloud else np.zeros((0, 3)), p_d)

        p_ref = p_d.copy(); v_ref = np.zeros(3); a_ref = np.zeros(3); kind = "hold"

        if not safety:
            # RECKLESS: commit straight to goal and fly the first step. No prediction, no certificate.
            if planner.plan(p_d, v_d, a_d, np.array([goal[0], goal[1], CRUISE_Z])) and planner.duration() > 1e-3:
                r = planner.eval(min(DT, max(planner.duration() - 1e-3, 0.0)))
                if r is not None:
                    p_ref, v_ref, a_ref = (np.asarray(x, float) for x in r); kind = "go"
        else:
            # SAFE: cylinder params (conformal per-class keep-out) for every nearby mover
            cyl = []; ztop = CRUISE_Z
            for i in near:
                cls = movers.m[i]["cls"]; q, veff = calib.get(cls, calib["_all"])
                c0, vv, aa = trackers[i].state()
                if PRED_MODEL == "cv": aa = np.zeros(3)
                if not predict: vv, aa = np.zeros(3), np.zeros(3)
                R = movers.m[i]["r"] + R_DRONE + D_SAFE_H + q
                zc = movers.m[i]["h"] + REACH_PAD + R_DRONE + D_SAFE_V + q
                cyl.append((c0, vv, aa, R, zc, veff)); ztop = max(ztop, zc + 0.2)
            ztop = min(Z_CEIL, ztop)

            def cert_clear():
                for (c0, vv, aa, R, zc, veff) in cyl:
                    hp, _ = planner.cert_h(c0, R, vv, aa, TAU, veff, DELTA)
                    hc, _ = planner.cert_h(c0, R, (0, 0, 0), (0, 0, 0), TAU, veff, DELTA)
                    vo, _ = planner.cert_a(zc, TAU, DELTA)
                    if not ((hp and hc) or vo):
                        return False
                return True

            def first():
                rr = planner.eval(min(DT, max(planner.duration() - 1e-3, 0.0)))
                return tuple(np.asarray(x, float) for x in rr) if rr is not None else None

            L = min(HORIZON, max(dist, 1.0)); chosen = None
            for gk, gsub in (("straight", np.array([goal[0], goal[1], CRUISE_Z])),
                             ("around_l", np.array([*(p_d[:2] + L * RC._rot(gdir, PHI)), CRUISE_Z])),
                             ("around_r", np.array([*(p_d[:2] + L * RC._rot(gdir, -PHI)), CRUISE_Z])),
                             ("around_l2", np.array([*(p_d[:2] + L * RC._rot(gdir, 2 * PHI)), CRUISE_Z])),
                             ("around_r2", np.array([*(p_d[:2] + L * RC._rot(gdir, -2 * PHI)), CRUISE_Z]))):
                if planner.plan(p_d, v_d, a_d, gsub) and planner.duration() > 1e-3 and cert_clear():
                    kind = gk; chosen = first(); break
            if chosen is None:
                if planner.plan(p_d, v_d, a_d, np.array([goal[0], goal[1], ztop])) and planner.duration() > 1e-3 and cert_clear():
                    kind = "over"; chosen = first()
                elif planner.plan(p_d, v_d, a_d, np.array([p_d[0], p_d[1], ztop])) and planner.duration() > 1e-3:
                    kind = "climb"; chosen = first()
            if chosen is not None:
                p_ref, v_ref, a_ref = chosen
            elif idx:
                # nothing certified but movers present -> FLEE the nearest (never FREEZE into a collision: a
                # stationary HOLD lets an approaching mover walk into the drone). Mirrors replay_core's evade.
                nn_i = min(idx, key=lambda i: np.linalg.norm(dets[i][:2] - p_d[:2]))
                away = p_d[:2] - dets[nn_i][:2]; nn = float(np.linalg.norm(away))
                away = away / nn if nn > 1e-6 else gdir
                p_ref = p_d + np.array([away[0] * 0.6 * max_vel * DT, away[1] * 0.6 * max_vel * DT,
                                        min(0.4 * max_vel * DT, max(0.0, ztop - p_d[2]))])
                v_ref = np.array([away[0] * max_vel, away[1] * max_vel, 0.0]); kind = "evade"; n_hold += 1
            else:
                n_hold += 1; kind = "hold"   # no movers near -> HOLD in place

        # fly the set-point
        if dynamics:
            pf, vf = quad.step(p_ref, v_ref, a_ref, DT)
            p_d, v_d, a_d = np.asarray(pf, float), np.asarray(vf, float), quad.a.copy()
        else:
            p_d, v_d, a_d = np.asarray(p_ref, float), np.asarray(v_ref, float), np.asarray(a_ref, float)

        max_z = max(max_z, float(p_d[2]))
        for i in present_idx(t):
            min_clr = min(min_clr, RC._clearance(p_d, pos_l(i, t), movers.m[i]["r"], movers.m[i]["h"]))
        dgoal = float(np.linalg.norm(p_d[:2] - goal[:2]))
        if dgoal < 0.8:
            reached = True; break
        if dgoal < best_d - 0.1: best_d = dgoal; stall = 0
        else: stall += 1
        if stall > 40: break

    return dict(planner=planner.name, safety=("on" if safety else "off"), reached=reached,
                time_s=round((tick + 1) * DT, 2), min_clr=(round(float(min_clr), 3) if min_clr < 1e17 else None),
                collided=int(min_clr < -1e-6), max_z=round(max_z, 2), n_hold=n_hold, dynamics=int(dynamics))


def parse_seeds(s):
    out = []
    for tok in s.split(","):
        if "-" in tok:
            a, b = tok.split("-"); out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(tok))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--planner", required=True, choices=["ego", "quintic", "septic", "bspline"])
    ap.add_argument("--safety", required=True, choices=["on", "off"])
    ap.add_argument("--seeds", default="0-9")
    ap.add_argument("--n_ep", type=int, default=6)
    ap.add_argument("--max_vel", type=float, default=3.0)
    ap.add_argument("--speeds", default="", help="comma list of matched max_vel to sweep (overrides --max_vel)")
    ap.add_argument("--max_acc", type=float, default=6.0)
    ap.add_argument("--eps", type=float, default=0.05)
    ap.add_argument("--dynamics", action="store_true")
    ap.add_argument("--csv", default="out/conformal/planner_safety.csv")
    args = ap.parse_args()

    calib = RC.load_calib(args.eps)
    safety = args.safety == "on"
    speeds = [float(x) for x in args.speeds.split(",")] if args.speeds else [args.max_vel]
    # pre-load every seed's movers + contested episodes once
    seed_eps = []
    for s in parse_seeds(args.seeds):
        f = os.path.join(RC.OUTDIR, f"traj_seed{s}.npz")
        if not os.path.exists(f):
            continue
        movers = RC.Movers(list(np.load(f, allow_pickle=True)["movers"]))
        seed_eps.append((s, movers, RC.build_episodes(movers, s, n_ep=args.n_ep)))
    rows = []
    for vmax in speeds:
        planner = make_planner(args.planner, vmax, args.max_acc)   # ONE planner per speed, reused across all
        for s, movers, eps in seed_eps:                            # episodes (EGOPlanner C++ grid leak is per-object)
            for k, ep in enumerate(eps):
                r = run_episode(movers, ep, planner, safety, calib, max_vel=vmax, dynamics=args.dynamics)
                r.update(seed=s, ep=k, max_vel=vmax, eps=args.eps)
                rows.append(r)

    # append to CSV (header only if new/empty)
    path = os.path.join(os.path.dirname(HERE), args.csv) if not os.path.isabs(args.csv) else args.csv
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fields = ["planner", "safety", "seed", "ep", "max_vel", "eps", "reached", "collided", "min_clr",
              "time_s", "max_z", "n_hold", "dynamics"]
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, "a", newline="") as fh:
        w = _csv.DictWriter(fh, fieldnames=fields);
        if new: w.writeheader()
        for r in rows: w.writerow({k: r.get(k) for k in fields})

    n = len(rows); coll = sum(r["collided"] for r in rows); reach = sum(r["reached"] for r in rows)
    clrs = [r["min_clr"] for r in rows if r["min_clr"] is not None]
    print(f"[psm] planner={args.planner} safety={args.safety}: n={n} reach={reach}/{n} collide={coll}/{n} "
          f"min_clr_med={np.median(clrs):.3f} -> appended {path}", flush=True)


if __name__ == "__main__":
    main()
