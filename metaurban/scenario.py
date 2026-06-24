"""scenario.py — simple, HAND-EDITABLE dynamic-obstacle scenes for the no-HOLD maneuver loop.

A human is a VERTICAL CYLINDER (the safety body the certificate guards): ground-plane (x,y) + xy velocity,
radius r, head-top height h. z is held (people don't fly). Edit SCENES below or add a preset; pick one with
EGO_SCENE=<name> (or the default). All numeric knobs are env-overridable so sweeping needs zero code edits.

load_scene(name) -> (start[3], goal[3], movers, knobs)
  movers: list of [centre(np3, z=cruise plane), vel(np3, vz=0), radius r, head-top h]  (KF tracks x,y).
"""
import os
import numpy as np


def _f(name, default):
    return float(os.environ.get(name, default))


def knobs():
    """All tunables in one place. EGO_* env vars override. The certificate's safety floor lives here."""
    return dict(
        HR=_f("EGO_HR", 0.3),            # cylinder radius (human body)
        H_HUMAN=_f("EGO_HHUMAN", 1.8),   # head-top height of a standing human
        D_SAFE=_f("EGO_DSAFE", 0.8),     # HORIZONTAL standoff (the AROUND half clears r+D_SAFE)
        D_SAFE_V=_f("EGO_DSAFEV", 0.5),  # VERTICAL standoff above the head (the OVER half)
        REACH_PAD=_f("EGO_REACHPAD", 0.3),  # posture/arm/jump reach added to head-top for z_clear
        TAU=_f("EGO_TAU", 0.75),         # certificate trust window [0,TAU]
        # CONFORMAL-CALIBRATED (was a hand-set 0.2 placeholder). v_eff is the (1-eps) conformal quantile slope of
        # the KF prediction residual on REAL MetaUrban pedestrians, CV predictor, eps=0.05 -> 0.61 m/s (validated
        # to 0.952 marginal coverage on held-out tracks). See conformal_calibrate.py + docs/conformal-results-2026-06.md.
        V_EFF=_f("EGO_VEFF", 0.61),      # horizontal tube growth rho(t)=q_conformal+V_EFF*(t+delta), gives P(collision)<=eps
        V_EFF_Z=_f("EGO_VEFFZ", 0.0),    # vertical floor growth (0: KF pins vz=az=0, height-bounded body)
        DT=_f("EGO_DT", 0.30),           # control period = perception->commit latency delta
        MAX_VEL=_f("EGO_MAXVEL", 3.0),
        MAX_ACC=_f("EGO_MAXACC", 6.0),
        Z_CRUISE=_f("EGO_ZCRUISE", 1.5),
        Z_CEILING=_f("EGO_ZCEIL", 4.6),  # < map z-extent top (~5.0); caps the climb
        PLAN_HI=_f("EGO_PLANHI", 0.45),  # planner-occupancy horizon (< TAU: avoid corridor-freezing phantom wall)
    )


# ---- scenes: edit freely. h = head-top height; v = [vx,vy] ground-plane velocity. ----
SCENES = {
    # default: ONE episode that forces BOTH a fly-OVER (standing wall) and an AROUND (lone crosser)
    "wall_and_crosser": dict(
        start=[0, 0, 1.5], goal=[16, 0, 1.5],
        humans=[
            dict(p=[8, -1.6], v=[0, 0], r=0.3, h=1.8),   # standing wall spanning y in [-1.6,1.6]
            dict(p=[8, -0.8], v=[0, 0], r=0.3, h=1.8),   #   -> lateral blocked, overhead free -> fly OVER
            dict(p=[8, 0.0], v=[0, 0], r=0.3, h=1.8),
            dict(p=[8, 0.8], v=[0, 0], r=0.3, h=1.8),
            dict(p=[8, 1.6], v=[0, 0], r=0.3, h=1.8),
            dict(p=[13, 3.0], v=[0, -1.0], r=0.3, h=1.8),  # lone crosser walking -y -> go AROUND (cheaper than climb)
        ]),
    # the original 3 crossers (direct A/B vs ego_goaround.py); around-dominant
    "crossers": dict(
        start=[0, 0, 1.5], goal=[16, 0, 1.5],
        humans=[
            dict(p=[6, 4.0], v=[0, -1.0], r=0.3, h=1.8),
            dict(p=[10, -4.0], v=[0, 1.0], r=0.3, h=1.8),
            dict(p=[13, 3.0], v=[0, -0.8], r=0.3, h=1.8),
        ]),
    # full-width standing line: lateral is impossible -> must climb over (the no-HOLD escape under a hard wall)
    "gauntlet": dict(
        start=[0, 0, 1.5], goal=[16, 0, 1.5],
        humans=[dict(p=[8, y], v=[0, 0], r=0.3, h=1.8) for y in np.arange(-4.0, 4.01, 0.8)]),
    # head-on: a human walking straight at the drone down the corridor
    "head_on": dict(
        start=[0, 0, 1.5], goal=[16, 0, 1.5],
        humans=[dict(p=[10, 0.0], v=[-1.2, 0], r=0.3, h=1.8)]),
}


def load_scene(name=None):
    k = knobs()
    name = name or os.environ.get("EGO_SCENE", "wall_and_crosser")
    sc = SCENES[name]
    start = np.array(sc["start"], float)
    goal = np.array(sc["goal"], float)
    movers = []
    for hh in sc["humans"]:
        c = np.array([hh["p"][0], hh["p"][1], k["Z_CRUISE"]], float)   # KF tracks x,y; z = cruise plane for occ
        v = np.array([hh["v"][0], hh["v"][1], 0.0], float)
        movers.append([c, v, float(hh["r"]), float(hh["h"])])
    return start, goal, movers, dict(k, name=name)


if __name__ == "__main__":
    s, g, mv, k = load_scene()
    print(f"[scene] {k['name']}: start={s} goal={g} | {len(mv)} humans")
    for c, v, r, h in mv:
        print(f"   human xy=({c[0]:.1f},{c[1]:.1f}) v=({v[0]:.1f},{v[1]:.1f}) r={r} head={h}")
    print(f"[scene] cert floor: AROUND r+D_SAFE={k['HR']+k['D_SAFE']:.2f}m  "
          f"OVER z_clear=h+REACH_PAD+D_SAFE_V={1.8+k['REACH_PAD']+k['D_SAFE_V']:.2f}m")
