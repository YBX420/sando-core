"""scenario_lib — the ONE scenario contract between the designer UI and the evaluation harnesses.

Schema "sando-scenario-v1" (JSON):
  name/description: str
  map:    {"seed": int|null,            MetaUrban scene index (0-19); null = abstract corridor
           "block_str": "X"}            PG block letters (C/S/X/T/O/B/y/R/P/$): "O"=roundabout, "CXO"=chain
  drone:  {"start":[x,y,z], "goal":[x,y,z], "cruise_z":1.5, "max_vel":3.0, "max_acc":6.0,
           "waypoints":[[x,y,z],...]?}                  optional via-points (3D designer edits them; the
                                                        replay harness still flies start->goal direct and
                                                        WARNS until leg-chaining lands -- honesty first)
  statics:[{"c":[x,y], "r":0.4, "h":3.0}, ...]          scripted static cylinders (besides the map's own)
  movers: [{"cls":"pedestrian"|"vehicle"|"animal",
            "asset": str?,                              3D model variant (designer/renderer only; sim ignores)
            "r":float?, "h":float?,                     default = class table below
            "spawn_t":0.0,                              sim time the mover appears
            "path":[[x,y],...],                         waypoint polyline (>=1 point; 1 = standing)
            "speed": v  |  {"v0":0,"v1":8,"a":2.8},     constant, or accelerate v0->v1 at a then hold
            "hold_end": true}, ...]                     stay at last waypoint after finishing (else despawn)
  params: {"<name>": {"value":v, "jitter":j}, ...}      "$<name>" strings in numeric fields substitute value;
                                                        expand_variants() jitters uniformly +-j per variant
  t_max:  float                                         compile horizon (s)

The compiler turns each mover into a time-sampled track dict(t, xy, cls) at DT_SAMPLE — EXACTLY the raw
format replay_core.Movers already eats (harvested-npz compatible), so a designed scenario runs through the
untouched evaluation pipeline: Movers(to_movers_raw(scn)) + to_episode(scn) -> run_replay(...).

Vehicle spawn acceleration ({"v0":0,"v1":8,"a":2.8}) is the CV!=CA stressor no previous scenario source had
(scenario.py / EGO_ADV_CROSSER / Animal are all constant-velocity).
"""
import copy
import json
import os

import numpy as np

SCHEMA = "sando-scenario-v1"
DT_SAMPLE = 0.10                      # track sampling step; matches deployment REPLAN_DT
CLS_R = {"pedestrian": 0.30, "vehicle": 0.60, "animal": 0.40, "static": 0.40}
CLS_H = {"pedestrian": 1.80, "vehicle": 1.60, "animal": 1.00, "static": 3.00}
CLS_LIST = ("pedestrian", "vehicle", "animal", "static")


# ---------------------------------------------------------------- load / validate
def _subst(node, params):
    """Recursively replace "$name" strings with params[name]["value"] (grid-only specs without an
    explicit "value" resolve to grid[0] so the template also runs standalone, not just via --variants)."""
    if isinstance(node, str) and node.startswith("$"):
        key = node[1:]
        if key not in params:
            raise ValueError(f"unknown param reference '{node}'")
        spec = params[key]
        if "value" in spec:
            return float(spec["value"])
        if spec.get("grid"):
            return float(spec["grid"][0])
        raise ValueError(f"param '{key}' has neither 'value' nor 'grid'")
    if isinstance(node, dict):
        return {k: _subst(v, params) for k, v in node.items()}
    if isinstance(node, list):
        return [_subst(v, params) for v in node]
    return node


def load(path_or_dict):
    """Load + validate + param-substitute a scenario. Returns the resolved dict (params applied)."""
    scn = (json.load(open(path_or_dict)) if isinstance(path_or_dict, str) else
           copy.deepcopy(path_or_dict))
    if scn.get("schema") != SCHEMA:
        raise ValueError(f"not a {SCHEMA} file (schema={scn.get('schema')!r})")
    params = scn.get("params", {})
    resolved = _subst({k: v for k, v in scn.items() if k != "params"}, params)
    resolved["params"] = params
    validate(resolved)
    return resolved


def validate(scn):
    d = scn.get("drone", {})
    for k in ("start", "goal"):
        if len(d.get(k, ())) != 3:
            raise ValueError(f"drone.{k} must be [x,y,z]")
    for i, m in enumerate(scn.get("movers", [])):
        if m.get("cls") not in CLS_LIST:
            raise ValueError(f"mover[{i}].cls {m.get('cls')!r} not in {CLS_LIST}")
        path = m.get("path", [])
        if len(path) < 1 or any(len(p) != 2 for p in path):
            raise ValueError(f"mover[{i}].path needs >=1 [x,y] waypoints")
        sp = m.get("speed", 0.0)
        if isinstance(sp, dict):
            if sp.get("a", 0.0) <= 0.0 and sp.get("v1") != sp.get("v0"):
                raise ValueError(f"mover[{i}].speed accel profile needs a>0")
        elif len(path) >= 2 and float(sp) <= 0.0:
            raise ValueError(f"mover[{i}] has a multi-point path but speed<=0")
    if float(scn.get("t_max", 30.0)) <= 0:
        raise ValueError("t_max must be > 0")


# ---------------------------------------------------------------- compile
def _speed_profile(sp):
    """-> function v(t) and (optionally) the time it stops accelerating."""
    if isinstance(sp, dict):
        v0, v1, a = float(sp.get("v0", 0.0)), float(sp["v1"]), float(sp.get("a", 1.0))
        t_ramp = abs(v1 - v0) / a if a > 0 else 0.0
        sgn = 1.0 if v1 >= v0 else -1.0
        return lambda t: (v0 + sgn * a * t) if t < t_ramp else v1
    v = float(sp)
    return lambda t: v


def compile_mover(m, t_max):
    """One mover spec -> track dict(t, xy, cls) sampled at DT_SAMPLE (absolute sim time; starts at spawn_t)."""
    path = np.asarray(m["path"], float)
    spawn_t = float(m.get("spawn_t", 0.0))
    cls = m["cls"]
    if len(path) == 1:                                   # standing (a wall element / static-as-mover)
        t = np.arange(spawn_t, t_max + DT_SAMPLE, DT_SAMPLE)
        return dict(t=t, xy=np.repeat(path, len(t), axis=0), cls=cls)
    seg = np.diff(path, axis=0)
    seg_len = np.linalg.norm(seg, axis=1)
    keep = seg_len > 1e-9
    path = np.vstack([path[0], path[1:][keep]])
    seg_len = seg_len[keep]
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = float(cum[-1])
    vfun = _speed_profile(m.get("speed", 1.0))
    hold = bool(m.get("hold_end", True))
    ts, ss = [0.0], [0.0]
    s = 0.0
    t = 0.0
    while t < t_max - spawn_t and s < total:
        v = max(0.0, vfun(t))
        s = min(total, s + v * DT_SAMPLE)
        t += DT_SAMPLE
        ts.append(t)
        ss.append(s)
    if hold and t < t_max - spawn_t:                     # stand at the endpoint until t_max
        ts.append(t_max - spawn_t)
        ss.append(s)
    ss = np.asarray(ss)
    xy = np.stack([np.interp(ss, cum, path[:, 0]), np.interp(ss, cum, path[:, 1])], axis=1)
    return dict(t=np.asarray(ts) + spawn_t, xy=xy, cls=cls)


def to_movers_raw(scn):
    """Scenario -> the raw track list replay_core.Movers eats. Scripted statics ride along as standing
    'static'-class movers (replay_core has no separate static channel; a standing cylinder is exactly how
    scenario.py built its walls)."""
    t_max = float(scn.get("t_max", 30.0))
    raw = [compile_mover(m, t_max) for m in scn.get("movers", [])]
    for st in scn.get("statics", []):
        raw.append(compile_mover(dict(cls="static", path=[list(st["c"])], spawn_t=0.0), t_max))
    return raw


def apply_rh_overrides(movers_obj, scn):
    """Movers defaults r/h by class table; per-mover 'r'/'h' overrides (and static radii) are patched here."""
    specs = list(scn.get("movers", [])) + [dict(cls="static", r=st.get("r"), h=st.get("h"))
                                           for st in scn.get("statics", [])]
    for i, sp in enumerate(specs):
        if i >= len(movers_obj.m):
            break
        movers_obj.m[i]["r"] = float(sp.get("r") or CLS_R[sp["cls"]])
        movers_obj.m[i]["h"] = float(sp.get("h") or CLS_H[sp["cls"]])
    return movers_obj


def to_episode(scn):
    """Scenario -> the ep dict run_replay consumes (t0=0, all movers are members)."""
    d = scn["drone"]
    n = len(scn.get("movers", [])) + len(scn.get("statics", []))
    return dict(start=np.asarray(d["start"], float), goal=np.asarray(d["goal"], float),
                t0=0.0, members=list(range(n)), anchor=None, t_cross=None)


# ---------------------------------------------------------------- batch variants
def expand_variants(scn_or_path, n, seed=0):
    """Template + params -> n resolved scenario dicts. Per param spec:
      {"value": v, "jitter": j}   -> uniform value+-j, deterministic per (seed, variant index)
      {"value": v, "grid": [...]} -> variant k takes grid[k % len(grid)] (sweep, e.g. a vmax ladder)
    Grid params cycle in lockstep with k; jitter params stay independent draws."""
    base = (json.load(open(scn_or_path)) if isinstance(scn_or_path, str) else
            copy.deepcopy(scn_or_path))
    params = base.get("params", {})
    out = []
    for k in range(n):
        rng = np.random.default_rng(seed * 10007 + k)
        var = copy.deepcopy(base)
        for name, spec in params.items():
            if spec.get("grid"):
                var["params"][name]["value"] = float(spec["grid"][k % len(spec["grid"])])
            else:
                j = float(spec.get("jitter", 0.0))
                var["params"][name]["value"] = float(spec["value"]) + (rng.uniform(-j, j) if j > 0 else 0.0)
        var["name"] = f"{base.get('name', 'scenario')}_v{k:03d}"
        out.append(load(var))
    return out


# ---------------------------------------------------------------- CLI: compile-check / expand
if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("usage: scenario_lib.py <scenario.json> [n_variants seed]"); sys.exit(1)
    scn = load(sys.argv[1])
    raw = to_movers_raw(scn)
    ep = to_episode(scn)
    print(f"[scn] {scn['name']}: {len(scn.get('movers', []))} movers + {len(scn.get('statics', []))} statics"
          f" -> {len(raw)} tracks; corridor {np.linalg.norm(np.asarray(ep['goal'][:2]) - ep['start'][:2]):.1f} m")
    for i, tr in enumerate(raw):
        v = (np.linalg.norm(np.diff(tr['xy'], axis=0), axis=1) / DT_SAMPLE) if len(tr['xy']) > 1 else [0.0]
        print(f"  [{i}] {tr['cls']:10s} t=[{tr['t'][0]:5.1f},{tr['t'][-1]:5.1f}]"
              f" vmax={max(v):.2f} m/s  pts={len(tr['t'])}")
    if len(sys.argv) >= 3:
        n = int(sys.argv[2]); sd = int(sys.argv[3]) if len(sys.argv) > 3 else 0
        outdir = os.path.join(os.path.dirname(os.path.abspath(sys.argv[1])), "variants")
        os.makedirs(outdir, exist_ok=True)
        for var in expand_variants(sys.argv[1], n, sd):
            p = os.path.join(outdir, var["name"] + ".json")
            json.dump({**var, "schema": SCHEMA}, open(p, "w"), indent=1, default=list)
            print(f"  wrote {p}")
