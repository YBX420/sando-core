"""ST-graph speed core (M2-2, 2026-07-17 timespace plan).

The classic path-velocity decomposition: fix a geometric path, project every KF-predicted mover
crossing into (station s, time t) FORBIDDEN BOXES, then dynamic-program the fastest monotone
speed profile through the free region. "Pass in front" = the profile goes OVER a box (arrives
before the crosser), "yield behind" = under it. Speed optimization becomes 2-D geometry.

Design contracts (from the 2026-07-17 feasibility review):
- The gear candidate set INCLUDES the production 8-gear grid + stop, so the DP optimum weakly
  dominates current behavior by construction (worst case = what we fly today).
- sigma widening: box radius = R + K * sigma(t) -- time-indexed, matches the M1a horn's shape.
- The DP only PROPOSES. It certifies nothing. Every flown profile must pass the Bernstein
  certificate (piecewise-warp composer, st_cert.py) -- judge and athlete stay separate.
- Pure python + numpy, no new heavy deps; importable by both the offline workbench and the
  tournament stage.
"""
import numpy as np

# superset of the production grid (1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3) + crawl + stop
GEARS_DEFAULT = (1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.15, 0.0)


def path_stations(path_xy, ds=0.25):
    """Resample a polyline to ~ds-spaced stations. Returns (pts (M,2), cum_s (M,)).
    QUANTIZATION NOTE: pick ds ~= v_max*dt/8 so the full gear advances an integer cell count per
    DP step -- with coarse ds, round() systematically shaves distance (measured -6%% at ds=0.25,
    v=8, dt=0.1) and biases every arrival-time comparison."""
    p = np.asarray(path_xy, float)
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    cs = np.concatenate([[0.0], np.cumsum(seg)])
    L = float(cs[-1])
    if L < 1e-6:
        return p[:1], np.zeros(1)
    s = np.arange(0.0, L + ds * 0.5, ds)
    x = np.interp(s, cs, p[:, 0]); y = np.interp(s, cs, p[:, 1])
    return np.c_[x, y], s


def forbidden_grid(pts, movers, t_grid, k_sigma=0.0, sigma_fn=None):
    """blocked[j, i] = station i is inside mover keep-out at time t_j.
    movers: iterable of (c0(2,), v(2,), R_keepout, sigma0, sigma_v); prediction is CV.
    Radius at horizon t: R + k_sigma * sigma(t), sigma(t) = sigma0 + sigma_v * t unless sigma_fn
    (mover_idx, t) -> sigma is given (online passes the real predict_sigma horn here)."""
    M = len(pts)
    blocked = np.zeros((len(t_grid), M), bool)
    for mi, (c0, v, R, s0, sv) in enumerate(movers):
        c0 = np.asarray(c0, float); v = np.asarray(v, float)
        for j, t in enumerate(t_grid):
            sig = sigma_fn(mi, t) if sigma_fn is not None else (s0 + sv * t)
            r = R + k_sigma * sig
            c = c0 + v * t
            blocked[j] |= (np.linalg.norm(pts - c[None, :], axis=1) < r)
    return blocked


def dp_profile(blocked, s_grid, t_grid, v0, v_max, a_max, gears=GEARS_DEFAULT, s_goal=None):
    """Fastest monotone-station profile through the free (s,t) region.
    State = (station index, gear index) reachable at time step j; transition v' with
    |v'-v| <= a_max*dt (reachability, the gap-law v3 lesson), sweep [s, s+v'*dt] must be free at
    BOTH endpoints' time rows (conservative). Returns dict(t_arr, gear_seq, s_seq) or None.
    gear_seq[k] = gear flown during (t_k, t_k+1)."""
    s_grid = np.asarray(s_grid, float); nt, ns = blocked.shape
    if s_goal is None:
        s_goal = s_grid[-1]
    dt = float(t_grid[1] - t_grid[0]) if len(t_grid) > 1 else 0.1
    gears = np.asarray(sorted(set(gears), reverse=True), float)   # descending
    speeds = gears * v_max
    ds = float(s_grid[1] - s_grid[0]) if ns > 1 else 0.25
    # closest gear to current speed marks the start state
    g0 = int(np.argmin(np.abs(speeds - min(v0, v_max))))
    reach = np.zeros((ns, len(gears)), bool)
    if blocked[0, 0]:
        return None                                            # standing in a box at t=0
    reach[0, g0] = True
    parent = {}
    for j in range(nt - 1):
        nxt = np.zeros_like(reach)
        rows = np.argwhere(reach)
        if not len(rows):
            return None
        for (i, g) in rows:
            for g2, sp2 in enumerate(speeds):
                # reachable in one step: physical accel bound, OR one grid step (the production
                # SPEED_SLEW law: gear moves at most one notch per tick -- the executor slews
                # continuously underneath, so adjacent-notch hops are always flyable)
                if abs(sp2 - speeds[g]) > a_max * dt + 1e-9 and abs(g2 - g) > 1:
                    continue
                i2 = min(ns - 1, i + int(round(sp2 * dt / ds)))
                # swept cells free at t_j and t_{j+1} (conservative double-row check)
                if blocked[j, i:i2 + 1].any() or blocked[j + 1, i:i2 + 1].any():
                    continue
                if not nxt[i2, g2]:
                    nxt[i2, g2] = True
                    parent[(j + 1, i2, g2)] = (i, g)
                if s_grid[i2] >= s_goal - 1e-9:
                    return _reconstruct(parent, gears, s_grid, t_grid, j + 1, i2, g2, reached=True)
        reach = nxt
    # horizon exhausted without reaching the goal: return the max-progress profile (the online
    # tournament scores PROGRESS over a short horizon; only the offline arrival comparison needs
    # reached=True)
    rows = np.argwhere(reach)
    if not len(rows):
        return None
    i, g = max(rows, key=lambda r: (r[0], -abs(int(r[1]) - 0)))
    return _reconstruct(parent, gears, s_grid, t_grid, nt - 1, int(i), int(g), reached=False)


def _reconstruct(parent, gears, s_grid, t_grid, j_end, i_end, g_end, reached):
    seq = [(i_end, g_end)]
    jj, ii, gg = j_end, i_end, g_end
    while jj > 0:
        ii, gg = parent[(jj, ii, gg)]
        jj -= 1
        seq.append((ii, gg))
    seq.reverse()
    return dict(reached=reached, t_arr=(float(t_grid[j_end]) if reached else None),
                s_end=float(s_grid[i_end]),
                gear_seq=[float(gears[g_]) for (_i, g_) in seq],
                s_seq=[float(s_grid[i_]) for (i_, _g) in seq])


def pass_decisions(pts, s_seq, t_grid, movers):
    """For each mover: did the profile cross the mover's path-crossing station BEFORE or AFTER the
    mover occupies it? Returns list of 'ahead'|'behind'|'clear' per mover (diagnostic)."""
    out = []
    for (c0, v, R, s0, sv) in movers:
        c0 = np.asarray(c0, float); v = np.asarray(v, float)
        d0 = np.linalg.norm(pts - c0[None, :], axis=1)
        # mover's closest approach to the path over the horizon
        best = (1e9, None, None)
        for j, t in enumerate(t_grid[:len(s_seq)]):
            c = c0 + v * t
            d = np.linalg.norm(pts - c[None, :], axis=1)
            i = int(np.argmin(d))
            if d[i] < best[0]:
                best = (float(d[i]), i, float(t))
        dmin, i_cross, t_cross = best
        if dmin > R + 0.5:
            out.append("clear"); continue
        # when does the DRONE reach that station?
        s_cross = i_cross * (pts.shape[0] and (np.linalg.norm(pts[1] - pts[0]) if len(pts) > 1 else 0.25))
        j_dr = next((jj for jj, s in enumerate(s_seq) if s >= s_cross - 1e-9), None)
        if j_dr is None:
            out.append("behind"); continue
        out.append("ahead" if t_grid[j_dr] < t_cross else "behind")
    return out


if __name__ == "__main__":
    # self-test: straight 40m path, one crosser owns s=20m during t in ~[3.2, 4.8].
    pts, s = path_stations([(0, 0), (40, 0)], ds=0.1)   # = v_max*dt/8 for v_max 8, dt 0.1
    movers = [((20.0, 8.0), (0.0, -2.0), 1.5, 0.1, 0.2)]      # crosses y=0 at t=4.0
    tg = np.arange(0.0, 25.0, 0.1)
    blocked = forbidden_grid(pts, movers, tg, k_sigma=1.0)
    assert blocked.any(), "crosser must block something"
    # fast drone (v_max 8): should sprint AHEAD of the crosser (arrive s=20 before t~3)
    r_fast = dp_profile(blocked, s, tg, v0=8.0, v_max=8.0, a_max=16.0)
    assert r_fast is not None and r_fast["reached"] and r_fast["t_arr"] < 6.0, r_fast
    dec = pass_decisions(pts, r_fast["s_seq"], tg, movers)
    assert dec[0] == "ahead", dec
    # slow drone (v_max 2.4 from rest): cannot beat the crosser -> must yield behind, still arrives
    r_slow = dp_profile(blocked, s, tg, v0=0.0, v_max=2.4, a_max=3.0)
    assert r_slow is not None and r_slow["reached"], "slow profile must exist (yield)"
    dec2 = pass_decisions(pts, r_slow["s_seq"], tg, movers)
    assert dec2[0] in ("behind", "clear"), dec2
    assert r_slow["t_arr"] > r_fast["t_arr"]
    # gear superset sanity: constant-1.0 profile time equals L/v when unblocked
    free = np.zeros_like(blocked)
    r_free = dp_profile(free, s, tg, v0=8.0, v_max=8.0, a_max=16.0)
    assert abs(r_free["t_arr"] - 40.0 / 8.0) < 0.3, r_free["t_arr"]
    print("[st_speed] self-test PASS: fast=ahead@%.1fs slow=%s@%.1fs free=%.1fs"
          % (r_fast["t_arr"], dec2[0], r_slow["t_arr"], r_free["t_arr"]))
