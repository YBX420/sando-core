"""Per-class ablation: SAME geometry, DIFFERENT class label -> DIFFERENT avoidance.

A drone flies a 30 m straight corridor with ONE obstacle of identical size sitting on the centre
line. We run it four times, labelling that obstacle static / pedestrian / vehicle / animal, and
record the min body clearance + the max lateral swerve. Because only the conformal label changes,
any difference is purely the C++ per-class d_safe (human 0.8 > animal 0.7 > vehicle 0.5 > static soft).

This isolates the 4-class differentiation that run_demo.py exercises inside the full MetaUrban crowd.
Pure C++ core via the ctypes bridge — no MetaUrban sim needed.
Run: ~/miniconda3/envs/metaurban/bin/python ablation.py   (any env with numpy works)
"""
import os
import sys
import json
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "isaac"))
import yaml
from sando_cpp_bridge import (Parameters, RobotState, DynTraj, SANDO,
                              DroneStatus_GOAL_REACHED as GOAL_REACHED)

with open(os.path.join(_HERE, "metaurban_sando.yaml"), encoding="utf-8") as f:
    PLN = yaml.safe_load(f)["planner"]

START = np.array([0.0, 0.0, 1.5])
GOAL = np.array([30.0, 0.0, 1.5])
OBST_C = np.array([15.0, 0.0, 1.5])      # dead on the path
OBST_SZ = np.array([0.9, 0.9, 1.6])
CASES = {"static": None, "pedestrian": [0], "vehicle": [1], "animal": [2]}


def run_case(name, label):
    par = Parameters()
    for k, v in PLN.items():
        if hasattr(par, k):
            setattr(par, k, v)
    par.replan_dt = 0.1
    sando = SANDO(par)
    st = RobotState(); st.pos = START.copy(); st.vel = np.zeros(3)
    sando.update_state(st)
    sando.update_occupancy_map_ptr(np.zeros((0, 3)))
    g = RobotState(); g.pos = GOAL.copy(); sando.set_terminal_goal(g)

    dt = DynTraj(); dt.mode = "Analytic"; dt.bbox = OBST_SZ.copy()
    dt.traj_x = f"{OBST_C[0]}"; dt.traj_y = f"{OBST_C[1]}"; dt.traj_z = f"{OBST_C[2]}"
    dt.traj_vx = dt.traj_vy = dt.traj_vz = "0.0"
    if label is None:
        dt.id = 250; dt.label_set = []            # id>=200 + empty set -> "wall" (soft)
    else:
        dt.id = 7; dt.label_set = list(label)     # conformal label -> human/vehicle/animal (hard)

    DT = float(par.dc); REPLAN = 0.1
    p = START.copy(); v = np.zeros(3); a = np.zeros(3)
    t = 0.0; nextrp = 0.0; reached = False
    rad = float(par.drone_radius)
    min_clr = np.inf; max_lat = 0.0; traj = []
    while t < 45.0 and not reached:
        if t >= nextrp - 1e-9:
            s2 = RobotState(); s2.pos = p.copy(); s2.vel = v.copy(); s2.accel = a.copy()
            sando.update_state(s2)
            dt.compile_analytic(); sando.add_traj(dt, t)
            sando.replan(0.0, t)
            nextrp = t + REPLAN
        ok, ng = sando.get_next_goal()
        if ok:
            p = np.asarray(ng.pos, float); v = np.asarray(ng.vel, float); a = np.asarray(ng.accel, float)
        d = p - OBST_C
        half = 0.5 * OBST_SZ
        outside = np.maximum(np.abs(d) - half, 0.0)
        sd = (np.linalg.norm(outside) if np.any(outside > 0) else -np.min(half - np.abs(d))) - rad
        min_clr = min(min_clr, sd)
        max_lat = max(max_lat, abs(p[1]))
        traj.append((float(p[0]), float(p[1]), float(p[2])))
        t += DT
        if sando.get_drone_status() == GOAL_REACHED and np.linalg.norm(p - GOAL) < float(par.goal_radius):
            reached = True
    return {"class": name, "label": label, "reached": bool(reached),
            "min_clearance_m": round(float(min_clr), 3), "max_lateral_swerve_m": round(float(max_lat), 3),
            "expected_d_safe": {"pedestrian": 0.8, "animal": 0.7, "vehicle": 0.6, "static": "soft"}[name],
            "traj": traj}


results = {}
for name, label in CASES.items():
    r = run_case(name, label)
    results[name] = r
    print(f"[ablation] {name:11s} label={str(label):6s} reached={r['reached']} "
          f"min_clr={r['min_clearance_m']:+.3f} m  max_swerve={r['max_lateral_swerve_m']:.3f} m  "
          f"(d_safe={r['expected_d_safe']})", flush=True)

os.makedirs(os.path.join(_HERE, "out"), exist_ok=True)
with open(os.path.join(_HERE, "out", "ablation.json"), "w") as f:
    json.dump({k: {kk: vv for kk, vv in v.items() if kk != "traj"} for k, v in results.items()}, f, indent=2)

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = {"static": "#7f7f7f", "pedestrian": "#d62728", "vehicle": "#1f77b4", "animal": "#9467bd"}
    fig, ax = plt.subplots(figsize=(11, 6))
    ax.add_patch(plt.Rectangle((OBST_C[0] - OBST_SZ[0] / 2, OBST_C[1] - OBST_SZ[1] / 2),
                               OBST_SZ[0], OBST_SZ[1], color="k", alpha=0.5, label="obstacle"))
    for name, r in results.items():
        a = np.array(r["traj"])
        ax.plot(a[:, 0], a[:, 1], "-", color=colors[name], lw=2,
                label=f"{name} (clr={r['min_clearance_m']:.2f}, swerve={r['max_lateral_swerve_m']:.2f})")
    ax.axhline(0, color="gray", ls=":", lw=0.8)
    ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("lateral y [m]")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Per-class avoidance ablation — SAME obstacle, label drives d_safe (C++ core)")
    fig.savefig(os.path.join(_HERE, "out", "ablation.png"), dpi=120, bbox_inches="tight")
    print(f"[ablation] plot -> {os.path.join(_HERE, 'out', 'ablation.png')}", flush=True)
except Exception as e:
    print(f"[ablation] (plot skipped: {e})", flush=True)
