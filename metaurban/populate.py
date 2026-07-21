"""populate — procedural street-life generator: pedestrians & vehicles with REALISTIC paths
sampled from the map's own semantics, with HARD pairwise-spacing control.

Human/traffic habits enforced (现实习性):
  pedestrians  walk ALONG sidewalk bands (urban_rules-verified), speed ~ N(1.3, 0.25) m/s clipped
               to [0.7, 2.0]; crossers cross PERPENDICULAR near lane ends (where crosswalks live),
               brief pre-cross pause is emulated by spawn_t stagger; walkers despawn at path end.
  vehicles     drive the LANE CENTERLINE (road_network geometry, never invented paths), urban speed
               ~ U(4, 8) m/s; same-lane followers keep a >= 2 s TIME headway at spawn.
  spacing      two layers: (1) spawn positions pairwise-separated, (2) the COMPILED trajectories are
               checked over the whole timeline -- min pairwise distance must clear per-class-pair
               floors (ped-ped 0.8 m, ped-veh 2.0 m, veh-veh 4.0 m beyond body radii); offenders are
               resampled up to K times, else dropped (never silently kept).

Usage (metaurban conda env; builds the map to read its geometry):
  ./metadrone.sh gen --seed 3 --peds 8 --vehicles 3 --out scenarios/street_life.json
"""
import argparse
import hashlib
import json
import os
import subprocess

import numpy as np

import scenario_lib as SLB
import urban_rules as UR

try:
    _GEN_COMMIT = subprocess.check_output(
        ["git", "rev-parse", "--short", "HEAD"],
        cwd=os.path.dirname(os.path.abspath(__file__)), text=True).strip()
except Exception:
    _GEN_COMMIT = "unknown"


def _encounter_ok(scn, tol_s=1.0):
    """SPACETIME encounter acceptance (07-20 generator fix): at least one retimed mover truly
    crosses the FINAL corridor within tol_s of the drone's own arrival at that crossing point
    (v_nom cruise from the FINAL start). Timing bookkeeping is not proof; this is the check."""
    st = np.asarray(scn["drone"]["start"][:2], float)
    gl = np.asarray(scn["drone"]["goal"][:2], float)
    L = float(np.linalg.norm(gl - st))
    v_nom = float(scn.get("_v_nom", 2.4))
    n_ok = 0
    for m in scn["movers"]:
        if not m.get("_retimed"):
            continue
        path = np.asarray(m["path"], float)
        spd = m["speed"] if not isinstance(m["speed"], dict) else m["speed"].get("v1", 5.0)
        spd = max(0.3, float(spd) if not isinstance(spd, str) else 5.0)
        seg_off = 0.0
        hit_ok = False
        for k in range(len(path) - 1):
            # check EVERY corridor crossing, not just the first geometric one (07-21 ruling:
            # a mover whose first crossing mistimes may still meet the drone at a later one)
            frac = _seg_cross(st, gl, path[k], path[k + 1])
            if frac is not None:
                cross = st + frac * (gl - st)
                t_drone = frac * L / v_nom
                t_mover = float(m["spawn_t"]) + (seg_off + float(np.linalg.norm(cross - path[k]))) / spd
                if abs(t_mover - t_drone) <= tol_s:
                    hit_ok = True
                    break
            seg_off += float(np.linalg.norm(path[k + 1] - path[k]))
        if hit_ok:
            n_ok += 1
    scn["encounters_verified"] = n_ok
    return n_ok >= 1


def _finalize_scn(scn, name, family, params, map_seed, block, gen_seed, cluster=None):
    """Strip working markers and stamp the provenance block (07-20 generator fix): generator
    commit, seed, family, params, map seed, block, cluster (same-world multistarts share one
    cluster -- they must never masquerade as independent scenarios) and the scenario SHA."""
    for m in scn["movers"]:
        m.pop("_retimed", None)
    scn.pop("_v_nom", None)
    used = int(scn.pop("_gen_seed_used", gen_seed))
    att = int(scn.pop("_gen_attempt", 0))
    scn["name"] = name
    scn["provenance"] = dict(generator_commit=_GEN_COMMIT,
                             root_seed=int(gen_seed), gen_seed=used, attempt=att,
                             family=str(family), params=dict(params),
                             map_seed=int(map_seed), block=str(block),
                             cluster=str(cluster or name))
    blob = json.dumps({k: v for k, v in scn.items() if k != "provenance"},
                      sort_keys=True, default=str)
    scn["provenance"]["scenario_sha"] = hashlib.sha256(blob.encode()).hexdigest()[:16]
    return scn

PED_SPEED_MU, PED_SPEED_SD = 1.3, 0.25
VEH_SPEED_LO, VEH_SPEED_HI = 4.0, 8.0
HEADWAY_S = 2.0                                   # same-lane vehicle time gap
FLOORS = {("pedestrian", "pedestrian"): 0.8, ("pedestrian", "vehicle"): 2.0,
          ("vehicle", "vehicle"): 4.0}            # centre-to-centre beyond both body radii


def _lanes(engine, min_len=18.0):
    out = []
    for ln in UR._lanes(engine):
        try:
            if ln.length >= min_len:
                out.append(ln)
        except Exception:
            continue
    return out


def _lane_path(ln, s0, s1, k=8, lat=0.0):
    pts = []
    for s in np.linspace(s0, s1, k):
        q = ln.position(float(s), float(lat))
        pts.append([round(float(q[0]), 2), round(float(q[1]), 2)])
    return pts


def _sidewalk_lat(engine, ln, s, rng, band=(1.0, 4.5)):
    """A verified sidewalk lateral offset at longitudinal s (tries both sides, walks outward)."""
    for _ in range(14):
        sgn = rng.choice([-1.0, 1.0])
        lat = sgn * (ln.width / 2.0 + rng.uniform(*band))
        q = ln.position(float(s), float(lat))
        if UR.region_of(engine, float(q[0]), float(q[1])) in ("nearroad", "main", "farfromroad"):
            return lat
    return None


def gen_walker(engine, ln, rng):
    s0 = rng.uniform(0.0, max(1.0, ln.length - 14.0))
    s1 = min(ln.length, s0 + rng.uniform(12.0, 30.0))
    lat = _sidewalk_lat(engine, ln, (s0 + s1) / 2.0, rng)
    if lat is None:
        return None
    return dict(cls="pedestrian", spawn_t=round(float(rng.uniform(0.0, 6.0)), 1),
                path=_lane_path(ln, s0, s1, k=6, lat=lat),
                speed=round(float(np.clip(rng.normal(PED_SPEED_MU, PED_SPEED_SD), 0.7, 2.0)), 2),
                hold_end=False)


def gen_crosser(engine, ln, rng):
    """Sidewalk-to-sidewalk road crossing: the far endpoint WALKS OUTWARD until it clears the WHOLE
    roadway (multi-lane roads need more than one lane-width; near intersections the naive one-lane
    hop lands on a crossing lane's asphalt -- the old version rejected ~all crossers on X maps)."""
    s = rng.uniform(0.15, 0.85) * ln.length
    lat_a = _sidewalk_lat(engine, ln, s, rng, band=(1.0, 3.0))
    if lat_a is None:
        return None
    qa = ln.position(float(s), float(lat_a))
    for extra in np.arange(1.0, 22.0, 0.8):
        lat_b = -np.sign(lat_a) * (ln.width / 2.0 + extra)
        qb = ln.position(float(s), float(lat_b))
        if UR.region_of(engine, float(qb[0]), float(qb[1])) in ("nearroad", "main", "farfromroad"):
            return dict(cls="pedestrian", spawn_t=round(float(rng.uniform(0.0, 8.0)), 1),
                        path=[[round(float(qa[0]), 2), round(float(qa[1]), 2)],
                              [round(float(qb[0]), 2), round(float(qb[1]), 2)]],
                        speed=round(float(np.clip(rng.normal(PED_SPEED_MU, PED_SPEED_SD), 0.7, 2.0)), 2),
                        hold_end=False)
    return None


def gen_vehicle(engine, ln, rng, lane_slots):
    v = round(float(rng.uniform(VEH_SPEED_LO, VEH_SPEED_HI)), 1)
    spawn = round(float(rng.uniform(0.0, 5.0)), 1)
    key = id(ln)
    for (t0, v0) in lane_slots.get(key, []):       # same-lane headway: follower enters HEADWAY_S later
        if abs(spawn - t0) < HEADWAY_S:
            spawn = round(t0 + HEADWAY_S + rng.uniform(0.0, 1.5), 1)
    lane_slots.setdefault(key, []).append((spawn, v))
    return dict(cls="vehicle", spawn_t=spawn,
                path=_lane_path(ln, 0.0, ln.length, k=10, lat=0.0),
                speed=v, hold_end=False)


# ---------------------------------------------------------------- spacing verification
def min_pair_dist(scn):
    """Compile ALL movers; return the worst (i, j, d_min_beyond_radii, floor) over the timeline."""
    tracks = [SLB.compile_mover(m, float(scn.get("t_max", 30.0))) for m in scn["movers"]]
    rs = [m.get("r") or SLB.CLS_R[m["cls"]] for m in scn["movers"]]
    worst = None
    tl = np.arange(0.0, float(scn.get("t_max", 30.0)), 0.2)
    pos = []
    for tr in tracks:
        x = np.interp(tl, tr["t"], tr["xy"][:, 0], left=np.nan, right=np.nan)
        y = np.interp(tl, tr["t"], tr["xy"][:, 1], left=np.nan, right=np.nan)
        x[tl < tr["t"][0]] = np.nan; x[tl > tr["t"][-1]] = np.nan
        pos.append(np.stack([x, y], 1))
    n = len(tracks)
    for i in range(n):
        for j in range(i + 1, n):
            d = np.hypot(*(pos[i] - pos[j]).T) - rs[i] - rs[j]
            if np.all(np.isnan(d)):
                continue
            dmin = float(np.nanmin(d))
            key = tuple(sorted([scn["movers"][i]["cls"], scn["movers"][j]["cls"]]))
            floor = FLOORS.get(key, 0.8)
            if worst is None or (dmin - floor) < (worst[2] - worst[3]):
                worst = (i, j, dmin, floor)
    return worst


def populate(engine, seed=0, n_walkers=5, n_crossers=3, n_vehicles=3, t_max=35.0,
             drone=None, retries=40):
    rng = np.random.default_rng(seed)
    lanes = _lanes(engine)
    if not lanes:
        raise RuntimeError("no usable lanes on this map")
    scn = dict(schema=SLB.SCHEMA, name=f"street_life_s{seed}",
               description=f"procedurally populated street life (seed {seed}): sidewalk walkers, "
                           f"crosswalk-area crossers, lane-following vehicles; pairwise spacing "
                           f"verified over the compiled timeline (floors ped-ped 0.8 / ped-veh 2.0 "
                           f"/ veh-veh 4.0 m beyond radii).",
               map=dict(seed=3, block_str="X"),
               drone=drone or dict(start=[-12.0, 0.0, 1.5], goal=[12.0, 0.0, 1.5], cruise_z=1.5,
                                   max_vel=3.0, max_acc=6.0),
               statics=[], movers=[], params={}, t_max=t_max)
    lane_slots = {}
    want = ([("walker", None)] * n_walkers + [("crosser", None)] * n_crossers
            + [("vehicle", None)] * n_vehicles)
    dropped = 0
    for kind, _ in want:
        placed = False
        for _ in range(retries):
            ln = lanes[rng.integers(len(lanes))]
            m = (gen_walker(engine, ln, rng) if kind == "walker" else
                 gen_crosser(engine, ln, rng) if kind == "crosser" else
                 gen_vehicle(engine, ln, rng, lane_slots))
            if m is None:
                continue
            scn["movers"].append(m)
            w = min_pair_dist(scn)
            if w is None or w[2] >= w[3]:
                placed = True
                break
            scn["movers"].pop()                     # spacing violated -> resample this mover
        if not placed:
            dropped += 1
    w = min_pair_dist(scn)
    stats = dict(n=len(scn["movers"]), dropped=dropped,
                 worst_pair=(None if w is None else
                             dict(i=w[0], j=w[1], clearance=round(w[2], 2), floor=w[3])))
    return scn, stats


TIERS = {                                       # density presets (walkers, crossers, vehicles)
    "light": (5, 3, 2),
    "busy": (10, 5, 4),
    "rush": (16, 8, 6),
    "packed": (22, 12, 8),
}


def corridor_pressure(scn, n_samp=24):
    """STATIC encounter-geometry check: min distance from the drone corridor segment to any mover
    path polyline (sampled). > ~1 m means NO mover ever comes near the corridor -- a free-pass scene
    no amount of retiming can fix (the s1/s2 geometry-lock finding)."""
    st = np.asarray(scn["drone"]["start"][:2], float)
    gl = np.asarray(scn["drone"]["goal"][:2], float)
    cs = np.linspace(0, 1, n_samp)[:, None] * (gl - st)[None, :] + st[None, :]
    best = 1e9
    for m in scn["movers"]:
        path = np.asarray(m["path"], float)
        if len(path) == 1:
            d = np.min(np.hypot(*(cs - path[0]).T))
        else:
            pts = np.concatenate([path[k] + np.linspace(0, 1, 8)[:, None] * (path[k + 1] - path[k])
                                  for k in range(len(path) - 1)])
            d = min(np.min(np.hypot(*(cs - q).T)) for q in pts)
        best = min(best, float(d))
    return best


def make_contested(scn, rng):
    """Route the drone THROUGH the action: corridor perpendicular through a crosser's crossing point,
    with that crosser's spawn_t re-timed so both arrive together (same trick as replay_core's
    build_episodes -- a benchmark scene must be CONTESTED or min_clr is just empty space)."""
    crossers = [m for m in scn["movers"] if m["cls"] == "pedestrian" and len(m["path"]) == 2
                and abs(m["path"][0][0] - m["path"][1][0]) + abs(m["path"][0][1] - m["path"][1][1]) > 4.0]
    if not crossers:                               # fall back: anchor on a walker instead
        crossers = [m for m in scn["movers"] if m["cls"] == "pedestrian"]
        if not crossers:
            return scn
    order = list(rng.permutation(len(crossers)))   # try EVERY anchor until one retimes legally --
    crossers = [crossers[i] for i in order]        # the old revert-on-breach left 3/6 scenes with
    anchor = crossers[0]                           # empty encounters (zero discriminative power)
    return _contest(scn, rng, crossers, n_anchor=1, occlude=False, veh_align=False)


def _seg_cross(p0, p1, q0, q1):
    """segment intersection param on p (0..1) or None."""
    d1 = p1 - p0; d2 = q1 - q0
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < 1e-9:
        return None
    t = ((q0[0] - p0[0]) * d2[1] - (q0[1] - p0[1]) * d2[0]) / den
    u = ((q0[0] - p0[0]) * d1[1] - (q0[1] - p0[1]) * d1[0]) / den
    return t if (0.0 <= t <= 1.0 and 0.0 <= u <= 1.0) else None


def _contest(scn, rng, crossers, n_anchor=3, occlude=True, veh_align=True, L=36.0, v_nom=2.4):
    """HARD contested: primary anchor fixes the corridor; then up to n_anchor crossers AND every
    corridor-crossing vehicle are re-timed to meet the drone at their own crossing points (staggered
    gauntlet, not one encounter); optional legal-band advertising boards occlude crosser origins
    (late reveal). Every single retime is spacing-verified and reverted on breach."""
    eng = scn.get("_engine")
    timed = False
    for anchor in crossers:
        a, b = np.asarray(anchor["path"][0], float), np.asarray(anchor["path"][1], float)
        mid = 0.5 * (a + b)
        cdir = (b - a) / max(1e-6, np.linalg.norm(b - a))
        perp = np.array([-cdir[1], cdir[0]])
        L = 26.0
        scn["drone"]["start"] = [round(float(mid[0] - 0.5 * L * perp[0]), 2),
                                 round(float(mid[1] - 0.5 * L * perp[1]), 2), 1.5]
        scn["drone"]["goal"] = [round(float(mid[0] + 0.5 * L * perp[0]), 2),
                                round(float(mid[1] + 0.5 * L * perp[1]), 2), 1.5]
        t_drone = (L / 2.0) / 2.4
        t_walk = float(np.linalg.norm(mid - a)) / float(anchor["speed"])
        old_spawn = anchor["spawn_t"]
        anchor["spawn_t"] = round(max(0.0, t_drone - t_walk), 1)
        w = min_pair_dist(scn)
        if w is None or w[2] >= w[3]:
            timed = True
            anchor["_retimed"] = True                     # encounter bookkeeping (07-20 generator fix):
            scn["_v_nom"] = 2.4                           #   pushback re-timing + spacetime validation
            break
        anchor["spawn_t"] = old_spawn                     # this anchor can't retime legally: next one
    if timed and n_anchor > 1:
        st = np.asarray(scn["drone"]["start"][:2], float)
        gl = np.asarray(scn["drone"]["goal"][:2], float)
        n_extra = 0
        movers_sorted = sorted(scn["movers"], key=lambda m: rng.uniform())
        for m in movers_sorted:
            if n_extra >= n_anchor - 1 and not veh_align:
                break
            path = np.asarray(m["path"], float)
            if len(path) < 2 or m is anchor:
                continue
            hit = None
            for k in range(len(path) - 1):
                tt = _seg_cross(st, gl, path[k], path[k + 1])
                if tt is not None:
                    hit = (tt, k)
                    break
            if hit is None:
                continue
            frac, k = hit
            if m["cls"] == "pedestrian" and n_extra >= n_anchor - 1:
                continue
            if m["cls"] == "vehicle" and not veh_align:
                continue
            cross_pt = st + frac * (gl - st)
            t_drone = frac * L / v_nom
            seg = np.linalg.norm(np.diff(path[:k + 1], axis=0), axis=1).sum() if k > 0 else 0.0
            dist_to_cross = seg + float(np.linalg.norm(cross_pt - path[k]))
            spd = m["speed"] if not isinstance(m["speed"], dict) else m["speed"].get("v1", 5.0)
            t_travel = dist_to_cross / max(0.3, float(spd) if not isinstance(spd, str) else 5.0)
            old_sp = m["spawn_t"]
            m["spawn_t"] = round(max(0.0, t_drone - t_travel + rng.uniform(-0.4, 0.4)), 1)
            w = min_pair_dist(scn)
            if w is not None and w[2] < w[3]:
                m["spawn_t"] = old_sp                     # breach -> revert this one
            else:
                m["_retimed"] = True
                if m["cls"] == "pedestrian":
                    n_extra += 1
        if occlude and eng is not None:
            import urban_rules as UR
            boards = 0
            for m in scn["movers"]:
                if boards >= 2 or m["cls"] != "pedestrian" or len(np.asarray(m["path"])) != 2:
                    continue
                x0, y0 = float(m["path"][0][0]), float(m["path"][0][1])
                q = UR.snap_legal(eng, "Advertising_board-0a4cd8e975884115bfb62b5fc0d72f09.glb", x0, y0)
                if q is None or float(np.hypot(q[0] - x0, q[1] - y0)) > 4.0:
                    continue                              # occluder must sit NEXT to the reveal point
                scn["statics"].append(dict(c=[round(q[0], 2), round(q[1], 2)], r=1.12, h=2.70,
                                           hdg=round(UR.lane_heading_deg(eng, q[0], q[1]), 1),
                                           asset="Advertising_board-0a4cd8e975884115bfb62b5fc0d72f09.glb"))
                if min_pair_dist(scn) and min_pair_dist(scn)[2] < min_pair_dist(scn)[3]:
                    scn["statics"].pop()
                else:
                    boards += 1
    scn["description"] += (" CONTESTED: corridor crosses the anchor crosser, arrivals time-aligned."
                           if timed else
                           " CONTESTED(spatial): no anchor could retime within spacing floors.")
    if _ensure_clear_start(scn) is None:
        scn["_reject"] = True                             # 07-20: no safe start / broken retime = REJECT
    return scn


def _ensure_clear_start(scn, r_clear=3.0, t_window=3.0, max_back=32.0):
    """SAFE TAKEOFF: the drone start must be clear of every mover during the launch window
    (t in [0, t_window]) and of every static, by r_clear beyond body radii. If not, walk the
    start BACKWARD along the corridor axis (goal fixed, corridor grows) until it is."""
    d = scn["drone"]
    st = np.asarray(d["start"][:2], float); gl = np.asarray(d["goal"][:2], float)
    axis = gl - st; axis = axis / max(1e-6, np.linalg.norm(axis))
    tracks = [SLB.compile_mover(m, float(scn.get("t_max", 30.0))) for m in scn["movers"]]
    rs = [m.get("r") or SLB.CLS_R[m["cls"]] for m in scn["movers"]]
    stat = [(np.asarray(x["c"], float), float(x.get("r", 0.4))) for x in scn.get("statics", [])]

    clss = [m["cls"] for m in scn["movers"]]
    eng = scn.pop("_engine", None)                        # populate() stashes it for the region check

    def clear(p):
        if eng is not None:
            try:
                if UR.region_of(eng, float(p[0]), float(p[1])) == "road":
                    return False                          # NEVER take off from the roadway
            except Exception:
                pass
        for (c, r) in stat:
            if float(np.hypot(*(p - c))) - r < r_clear:
                return False
        for tr, r, cl in zip(tracks, rs, clss):
            guard = 8.0 if cl == "vehicle" else r_clear   # vehicles: wide no-traffic bubble at takeoff
            for t in np.arange(0.0, t_window + 1e-6, 0.5):
                if not (tr["t"][0] <= t <= tr["t"][-1]):
                    continue
                q = np.array([np.interp(t, tr["t"], tr["xy"][:, 0]),
                              np.interp(t, tr["t"], tr["xy"][:, 1])])
                if float(np.hypot(*(p - q))) - r < guard:
                    return False
        return True

    # 07-21 campaign fix: backward-only search starved dense worlds (165/300 attempts died on
    # 'takeoff zone NOT clear' -- the OLD generator merely WARNED and launched inside traffic).
    # The standard stays strict; the SEARCH widens: backoff to 32 m with lateral +-2/+-4 m probes
    # at each step. Any start move is later re-validated by the spacetime encounter check on the
    # FINAL geometry, so the probe cannot fake an encounter.
    perp = np.array([-axis[1], axis[0]])
    back, lat = 0.0, 0.0
    found = False
    while back <= max_back:
        for lat in (0.0, 2.0, -2.0, 4.0, -4.0):
            if clear(st - axis * back + perp * lat):
                found = True
                break
        if found:
            break
        back += 2.0
    if not found:
        scn["description"] += f" [REJECT: no clear takeoff spot within {max_back:.0f} m backoff]"
        print(f"[populate] REJECT {scn.get('name')}: takeoff zone NOT clear after {max_back} m "
              f"backoff (+-4 m lateral)")
        return None                                        # 07-20 generator fix: no safe start = REJECT
    if back > 0 or abs(lat) > 0:
        st_new = st - axis * back + perp * lat
        d["start"] = [round(float(st_new[0]), 2), round(float(st_new[1]), 2), d["start"][2]]
        # RE-TIME after the pushback (07-20 generator fix): every retimed encounter was aligned to
        # the ORIGINAL start; moving the start back by `back` delays the drone at every crossing
        # point by back/v_nom -- shift the retimed movers' spawn_t by the same amount, then
        # re-verify the launch window (a shifted mover may now conflict with takeoff).
        v_nom = float(scn.get("_v_nom", 2.4))
        n_shift = 0
        for m in scn["movers"]:
            if m.get("_retimed"):
                m["spawn_t"] = round(float(m["spawn_t"]) + back / v_nom, 1)
                n_shift += 1
        if n_shift:
            print(f"[populate] {scn.get('name')}: start moved back {back:.0f} m; "
                  f"{n_shift} retimed movers shifted +{back / v_nom:.1f}s to keep the encounter", flush=True)
            st2 = np.asarray(d["start"][:2], float)
            tracks[:] = [SLB.compile_mover(m, float(scn.get("t_max", 30.0))) for m in scn["movers"]]
            if not clear(st2):
                scn["description"] += " [REJECT: shifted encounter re-entered the takeoff window]"
                print(f"[populate] REJECT {scn.get('name')}: retime shift broke the takeoff window", flush=True)
                return None
            w = min_pair_dist(scn)
            if w is not None and w[2] < w[3]:
                # 07-21 ruling: the spawn-t shift must RE-PASS the mover spacing floors too --
                # the contest verified spacing at the OLD times; shifting can re-collide movers.
                scn["description"] += " [REJECT: retime shift broke mover spacing]"
                print(f"[populate] REJECT {scn.get('name')}: spawn shift broke min-pair spacing", flush=True)
                return None
        else:
            print(f"[populate] {scn.get('name')}: start moved back {back:.0f} m for a clear takeoff zone")
    return back


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--map-seed", type=int, default=3)
    ap.add_argument("--block", type=str, default="X")
    ap.add_argument("--peds", type=int, default=5)
    ap.add_argument("--crossers", type=int, default=3)
    ap.add_argument("--vehicles", type=int, default=3)
    ap.add_argument("--tier", choices=list(TIERS), default=None, help="density preset (overrides counts)")
    ap.add_argument("--contested", action="store_true", help="route the drone through the action")
    ap.add_argument("--hard", action="store_true",
                    help="HARD contested: 3 staggered timed crossers + timed vehicles + occluding boards")
    ap.add_argument("--multistart", type=int, default=0,
                    help="per bench scene, also emit K variants with DIFFERENT contested corridors "
                         "(same world, different departure points -- 一个场景不同地方出发)")
    ap.add_argument("--bench", type=int, default=0,
                    help="generate a BENCHMARK suite: N seeds x (busy,rush) contested scenes in one env session")
    ap.add_argument("--out", type=str, default="scenarios/street_life.json")
    args = ap.parse_args()
    from scenario_designer3d import build_env
    env = build_env(args.map_seed, interactive=False, block_str=args.block)

    def one(seed, tier, out):
        nw, nc, nv = TIERS[tier] if tier else (args.peds, args.crossers, args.vehicles)
        best_scn, best_stats, best_p = None, None, 1e9
        for attempt in range(6):                            # encounter-geometry filter: regenerate
            scn, stats = populate(env.engine, seed=seed + 1000 * attempt,
                                  n_walkers=nw, n_crossers=nc, n_vehicles=nv)
            scn["_gen_seed_used"] = seed + 1000 * attempt   # provenance must record the ACTUAL
            scn["_gen_attempt"] = attempt                   # attempt seed, not the root (07-21)
            scn["_engine"] = env.engine
            rng_a = np.random.default_rng(seed * 31 + 7 + attempt)
            if args.hard:
                cr = [m for m in scn["movers"] if m["cls"] == "pedestrian" and len(m["path"]) == 2]
                order = list(rng_a.permutation(len(cr)))
                scn = _contest(scn, rng_a, [cr[i] for i in order] or
                               [m for m in scn["movers"] if m["cls"] == "pedestrian"],
                               n_anchor=3, occlude=True, veh_align=True)
            elif args.contested:
                # 07-20 generator fix: --contested finally GATES this branch (it was unconditional)
                scn = make_contested(scn, rng_a)
            else:
                if _ensure_clear_start(scn) is None:        # plain scene still needs a safe start
                    scn["_reject"] = True
            scn.pop("_engine", None)
            if scn.pop("_reject", False):
                continue                                    # REJECT this attempt (no safe start)
            if (args.hard or args.contested) and not _encounter_ok(scn):
                continue                                    # contested INTENT without a true encounter
            pr = corridor_pressure(scn)
            if pr < best_p:
                best_scn, best_stats, best_p = scn, stats, pr
            if pr <= 1.0:
                break
        if best_scn is None:
            print(f"[populate] REJECT {out}: all 6 attempts failed eligibility "
                  f"(no safe start / no true encounter)", flush=True)
            return None
        scn, stats = best_scn, best_stats
        stats["pressure"] = round(best_p, 2)
        if best_p > 1.0:
            print(f"[populate] WARN {out}: geometry pressure {best_p:.1f}m > 1.0 after 6 tries")
        scn["map"] = dict(seed=args.map_seed, block_str=args.block)
        _finalize_scn(scn, os.path.splitext(os.path.basename(out))[0], (tier or "custom"),
                      dict(peds=nw, crossers=nc, vehicles=nv, hard=bool(args.hard),
                           contested=bool(args.contested)),
                      args.map_seed, args.block, seed)
        json.dump(scn, open(out, "w"), indent=1)
        print(f"[populate] {scn['name']}: {stats['n']} movers ({stats['dropped']} dropped), "
              f"worst pair {stats['worst_pair']}")
        return stats

    if args.bench:
        os.makedirs("scenarios/bench", exist_ok=True)
        manifest = []
        for tier in ("busy", "rush"):
            for k in range(args.bench):
                out = f"scenarios/bench/street_{tier}_s{k}.json"
                if os.path.exists(out) and os.environ.get("POPULATE_FORCE") != "1":
                    # 07-21 ruling: a chunked re-run restarted at k=0 and silently OVERWROTE the
                    # existing street_* pool and MANIFEST -- never clobber a generated scenario
                    print(f"[populate] REFUSE overwrite {out} (set POPULATE_FORCE=1 to override)",
                          flush=True)
                    continue
                st = one(k, tier, out)
                if st is None:
                    continue                                # rejected scene: no file, no manifest row
                manifest.append(dict(file=out, tier=tier, seed=k, **st))
                for ms in range(1, args.multistart + 1):    # same world, different departure corridor
                    scn = json.load(open(out))
                    rng2 = np.random.default_rng(k * 977 + ms * 131)
                    scn["_engine"] = env.engine
                    scn = make_contested(scn, rng2)
                    scn.pop("_engine", None)
                    if scn.pop("_reject", False) or not _encounter_ok(scn):
                        continue                            # alt corridor failed eligibility
                    _finalize_scn(scn, f"street_{tier}_s{k}_alt{ms}", tier,
                                  dict(alt=ms, contested=True), args.map_seed, args.block, k,
                                  cluster=f"street_{tier}_s{k}")
                    #   ^ SAME WORLD = ONE CLUSTER (07-20): a multistart must never masquerade as
                    #     an independent scenario in any exchangeability-sensitive split
                    out2 = f"scenarios/bench/street_{tier}_s{k}_alt{ms}.json"
                    json.dump(scn, open(out2, "w"), indent=1)
                    manifest.append(dict(file=out2, tier=tier, seed=k, alt=ms,
                                         cluster=f"street_{tier}_s{k}"))
        mf = "scenarios/bench/MANIFEST.json"
        if os.path.exists(mf):                              # 07-21: MERGE, never truncate the ledger
            old_rows = json.load(open(mf))
            new_files = {r["file"] for r in manifest}
            manifest = [r for r in old_rows if r.get("file") not in new_files] + manifest
        json.dump(manifest, open(mf, "w"), indent=1)
        print(f"[populate] benchmark suite: {len(manifest)} scenes + MANIFEST.json (merged)")
    else:
        one(args.seed, args.tier, args.out)
    env.close()


if __name__ == "__main__":
    main()
