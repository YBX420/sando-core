"""urban_rules — city-planning legality oracle for prop placement.

The region rules are COPIED from MetaUrban's own AssetManager
(metaurban/manager/sidewalk_manager.py `regular_objects`): that table is how the official
pipeline scatters street furniture, i.e. the de-facto 城建标准 of this simulator:
  nearroad band   : Tree, Lamp_post, TrashCan/Trash_bin, FireHydrant, Traffic_sign, Bollard
  main band       : Mailbox, Telephone_booth, Vending_machine, Table, Bag, Cone, dog
  farfromroad band: Bench, Chair, Advertising_board, Bonsai, Vegetation
  valid (outer)   : Building, FoodTruck, Bike/Scooter/Motorcycle/Wheelchair
Roadway itself is legal for NOTHING in that table (cones are main_sidewalk officially; if you
want a construction-zone cone ON the road, that is a deliberate override, and the editor will
warn rather than forbid).

Geometry: a point's lateral offset beyond the outermost lane edge classifies it:
  <0 on-road | 0..NEAR on nearroad | NEAR..MAIN main | MAIN..FAR farfromroad | beyond: valid
Band widths approximate MetaUrban's calculate_lateral_range defaults; they are dials, not law.
"""
import numpy as np

# lateral bands beyond the road edge (m) -- tune to taste
NEAR_END, MAIN_END, FAR_END = 2.0, 5.5, 9.0

REGION_OF_CATEGORY = {
    "Tree": "nearroad", "Lamp_post": "nearroad", "TrashCan": "nearroad", "Trash_bin": "nearroad",
    "FireHydrant": "nearroad", "Traffic_sign": "nearroad", "Bollard": "nearroad",
    "Traffic_light": "nearroad", "Cone": "main",
    "Mailbox": "main", "Telephone_booth": "main", "Vending_machine": "main",
    "Table": "main", "Bag": "main", "dog": "main", "Tents": "main",
    "Bench": "farfromroad", "Chair": "farfromroad", "Advertising_board": "farfromroad",
    "Bonsai": "farfromroad", "Vegetation": "farfromroad",
    "Building": "valid", "FoodTruck": "valid", "Bicycle": "valid", "Scooter": "valid",
    "Motorcycle": "valid", "Wheelchair": "valid", "Wall": "valid",
}
_BANDS = {"nearroad": (0.3, NEAR_END), "main": (NEAR_END, MAIN_END),
          "farfromroad": (MAIN_END, FAR_END), "valid": (0.3, FAR_END + 6.0)}


def _lanes(engine):
    try:
        graph = engine.current_map.road_network.graph
    except Exception:
        return
    for frm in graph.values():
        for lanes in frm.values():
            for ln in lanes:
                yield ln


def road_offset(engine, x, y):
    """Signed metres beyond the nearest road edge: <0 = ON the roadway, >0 = off-road distance.
    Also returns the (long, lane) of the closest lane for lateral projection."""
    best = (1e9, None, None)                                 # (edge_offset, lane, long)
    p = np.array([x, y])
    for ln in _lanes(engine):
        try:
            lng, lat = ln.local_coordinates(p)
        except Exception:
            continue
        if lng < -2.0 or lng > ln.length + 2.0:
            continue
        off = abs(lat) - ln.width / 2.0                      # beyond THIS lane's edge
        if off < best[0]:
            best = (off, ln, float(np.clip(lng, 0.0, ln.length)), float(np.sign(lat) or 1.0))
    return best                                              # (offset, lane, long, side)


def region_of(engine, x, y):
    off = road_offset(engine, x, y)[0]
    if off == 1e9:
        return "unknown"
    if off < 0:
        return "road"
    for name, (lo, hi) in (("nearroad", _BANDS["nearroad"]), ("main", _BANDS["main"]),
                           ("farfromroad", _BANDS["farfromroad"])):
        if lo <= off < hi:
            return name
    return "valid"


def category_of_asset(asset_fn):
    return (asset_fn or "").split("-", 1)[0] if asset_fn else None


def check(engine, asset_fn, x, y):
    """-> (legal: bool|None, want_region, is_region). None = no rule known (plain pillar etc.)."""
    cat = category_of_asset(asset_fn)
    want = REGION_OF_CATEGORY.get(cat)
    if want is None:
        return None, None, region_of(engine, x, y)
    here = region_of(engine, x, y)
    lo, hi = _BANDS[want]
    off = road_offset(engine, x, y)[0]
    return (off != 1e9 and lo <= off < hi), want, here


def snap_legal(engine, asset_fn, x, y):
    """Project (x,y) laterally toward its category's legal band, VERIFYING against the whole road
    network each step (at intersections a single-lane projection can land inside a CROSSING lane's
    roadway, so we walk outward until the global region check actually passes). None = no rule/lane."""
    cat = category_of_asset(asset_fn)
    want = REGION_OF_CATEGORY.get(cat)
    if want is None:
        return None
    off, ln, lng, side = road_offset(engine, x, y)
    if ln is None:
        return None
    lo, hi = _BANDS[want]
    for extra in np.arange(0.0, 18.0, 0.75):                 # walk outward past crossing lanes
        for sgn in (side, -side):                            # try this side first, then the other
            target_lat = sgn * (ln.width / 2.0 + 0.5 * (lo + hi) + extra)
            try:
                q = ln.position(lng, target_lat)
            except Exception:
                continue
            o2 = road_offset(engine, float(q[0]), float(q[1]))[0]
            if o2 != 1e9 and lo <= o2 < hi:
                return float(q[0]), float(q[1])
    return None


def lane_heading_deg(engine, x, y):
    """Street direction (deg, panda H convention: 0=+Y) at the nearest lane -- used to orient props
    parallel to the road, matching the official manager's 'parallel_only' placement."""
    off, ln, lng, side = road_offset(engine, x, y)
    if ln is None:
        return 0.0
    a = ln.position(max(0.0, lng - 0.5), 0.0)
    b = ln.position(min(ln.length, lng + 0.5), 0.0)
    import math
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) - 90.0
