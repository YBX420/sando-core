"""Offline ST-graph prototype (M2-2): same flown geometry, ST-optimal timing vs actual timing.

Reads a GTDUMP jsonl, takes the ACTUAL flown path as the direction candidate (isolating the speed
axis exactly), builds (station, time) forbidden boxes from the movers and DPs the fastest monotone
profile. Two prediction models per scene:
  CV+sigma : movers linearized at t=0 (what the online stage will see), sigma-widened;
  GT-traj  : boxes from the movers' TRUE recorded trajectories (perfect-prediction upper bound).
Usage: python st_offline.py <dump.jsonl> [R_pad]
"""
import json
import sys

import numpy as np

import st_speed as ST


def load(path):
    meta = None; ticks = []
    for line in open(path):
        row = json.loads(line)
        if "meta" in row:
            meta = row["meta"]
        else:
            ticks.append(row)
    return meta, ticks


def main(path, r_pad=0.75):
    meta, ticks = load(path)
    v_max = float(meta["v_max"]); a_max = float(meta["a_max"])
    drone = np.asarray([r["drone"][:2] for r in ticks], float)
    tt = np.asarray([r["t"] for r in ticks], float)
    t_flown = float(tt[-1]) + 0.1
    pts, s_grid = ST.path_stations(drone, ds=v_max * 0.1 / 8.0)
    L = float(s_grid[-1])
    t_grid = np.arange(0.0, max(t_flown * 2.5, 30.0), 0.1)

    # movers at t=0 (CV model) and full trajectories (GT model)
    first = ticks[0]["movers"]
    movers_cv = []
    trajs = {}
    for row in ticks:
        for (oid, cls, x, y, vx, vy, r, h) in row["movers"]:
            trajs.setdefault(oid, {"t": [], "x": [], "y": [], "r": float(r)})
            trajs[oid]["t"].append(row["t"]); trajs[oid]["x"].append(x); trajs[oid]["y"].append(y)
    for (oid, cls, x, y, vx, vy, r, h) in first:
        movers_cv.append(((x, y), (vx, vy), float(r) + r_pad, 0.1, 0.4))

    # sigma capped at 1.2m (the M1b guardrail): an unbounded horn at 30s is 12m wide and blankets the
    # whole corridor -- the review's corridor-starvation warning, reproduced live on first run
    _sig_cap = lambda mi, t: min(0.1 + 0.4 * t, 1.2)
    blocked_cv = ST.forbidden_grid(pts, movers_cv, t_grid, k_sigma=1.0, sigma_fn=_sig_cap)
    # GT boxes: interpolate recorded positions, clamp outside the record
    blocked_gt = np.zeros((len(t_grid), len(pts)), bool)
    for oid, d in trajs.items():
        ta = np.asarray(d["t"]); xa = np.asarray(d["x"]); ya = np.asarray(d["y"])
        r = d["r"] + r_pad
        for j, t in enumerate(t_grid):
            cx = np.interp(t, ta, xa); cy = np.interp(t, ta, ya)
            blocked_gt[j] |= (np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) < r)

    # RECEDING-HORIZON simulation (the online-faithful model): every 0.1s rebuild boxes from the
    # movers' CURRENT recorded state (pos + vel), horizon capped at the commitment scale 1.5s
    # (sigma(1.5)~0.7 under the horn), DP the window, fly ONE step. This is what ST_SPEED=1 will do.
    def rh_simulate(horizon=1.5, dt=0.1):
        v_cur, s_cur, t_cur = 0.0, 0.0, 0.0
        tg_w = np.arange(0.0, horizon + 1e-9, dt)
        gears = np.asarray(sorted(set(ST.GEARS_DEFAULT), reverse=True), float)
        for step in range(int(len(t_grid))):
            if s_cur >= L - 0.3:
                return t_cur
            mv = []
            for oid, d in trajs.items():
                ta = np.asarray(d["t"]); xa = np.asarray(d["x"]); ya = np.asarray(d["y"])
                cx = np.interp(t_cur, ta, xa); cy = np.interp(t_cur, ta, ya)
                t2 = min(t_cur + dt, ta[-1])
                vx = (np.interp(t2, ta, xa) - cx) / max(t2 - t_cur, 1e-6)
                vy = (np.interp(t2, ta, ya) - cy) / max(t2 - t_cur, 1e-6)
                mv.append(((cx, cy), (vx, vy), d["r"] + r_pad, 0.1, 0.4))
            i0 = int(np.searchsorted(s_grid, s_cur))
            pts_w = pts[i0:]; s_w = s_grid[i0:] - s_grid[min(i0, len(s_grid) - 1)]
            if len(s_w) < 2:
                return t_cur
            blocked_w = ST.forbidden_grid(pts_w, mv, tg_w, k_sigma=1.0)
            r = ST.dp_profile(blocked_w, s_w, tg_w, v0=v_cur, v_max=v_max, a_max=a_max)
            if r is None:
                v_cur = 0.0; t_cur += dt; continue          # boxed in: hover this tick
            g = r["gear_seq"][1] if len(r["gear_seq"]) > 1 else r["gear_seq"][0]
            v_cur = g * v_max
            s_cur += v_cur * dt; t_cur += dt
        return None

    t_rh = rh_simulate()
    if t_rh is not None:
        print(f"[st_offline] RH-online : t_arr={t_rh:5.1f}s vs flown {t_flown:5.1f}s "
              f"({100 * (t_flown - t_rh) / t_flown:+.0f}%)  (receding 1.5s window, CV+sigma capped)")
    else:
        print("[st_offline] RH-online : DNF within grid")

    out = {"dump": path.split("/")[-1], "L": L, "t_flown": t_flown}
    for name, blocked in (("CV+sigma", blocked_cv), ("GT-traj", blocked_gt)):
        r = ST.dp_profile(blocked, s_grid, t_grid, v0=0.0, v_max=v_max, a_max=a_max)
        if r is None or not r["reached"]:
            out[name] = None
            print(f"[st_offline] {name:9s}: NO feasible profile within horizon "
                  f"(progress {r['s_end']:.1f}/{L:.1f}m)" if r else f"[st_offline] {name}: none")
            continue
        movers_for_dec = movers_cv if name == "CV+sigma" else movers_cv
        dec = ST.pass_decisions(pts, r["s_seq"], t_grid, movers_for_dec)
        n_ahead = dec.count("ahead"); n_behind = dec.count("behind")
        out[name] = r["t_arr"]
        print(f"[st_offline] {name:9s}: t_arr={r['t_arr']:5.1f}s vs flown {t_flown:5.1f}s "
              f"({100 * (t_flown - r['t_arr']) / t_flown:+.0f}%)  pass: {n_ahead} ahead / {n_behind} behind "
              f"/ {dec.count('clear')} clear")
    return out


if __name__ == "__main__":
    main(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 0.75)
