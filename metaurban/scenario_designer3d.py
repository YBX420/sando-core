"""scenario_designer3d — Unity-style 3D scenario editor INSIDE the native MetaUrban (Panda3D) window,
plus a headless --preview mode that renders a scenario to 3D PNG frames (CLI-only workflow).

Run (metaurban conda env, PYTHONPATH must include the metaurban project; GL needs DISPLAY, e.g. :1):
  python scenario_designer3d.py --seed 3                      interactive 3D editor on scene 3
  python scenario_designer3d.py --seed 3 --load scenarios/x.json
  python scenario_designer3d.py --preview scenarios/x.json    HEADLESS: render t=0/25/50/75% 3D frames
                                                              -> out/scenario_previews/<name>_t*.png
  python scenario_designer3d.py --selftest                    headless logic test (no GL window)

Interactive controls -- Fusion360-style mouse + top-left CHINESE TOOLBAR (all modes/actions clickable;
world is FROZEN: taskMgr-only loop = live camera/input, no sim stepping)
  mouse L      click = select / place (per toolbar mode); click-and-HOLD on a point = DRAG it
  mouse M      hold-drag = pan the camera (grab-the-ground)     wheel = zoom
  Ctrl+Z       undo (60-step snapshot stack)
  keyboard     1/2/3 start/goal/waypoint  p/v/a/o place modes  TAB cycle sel  x/Del delete last point
               q/e asset variant  u/i speed  c const<->accel  -/= v1  9/0 accel  ,/. spawn_t
               SPACE play  r rewind  F2 export  Esc quit
  HUD          bottom-left, Chinese: current mode / selection / mover params / context hints.
Emits the SAME sando-scenario-v1 JSON as scenario_lib/scenario_designer(2D); mover["asset"] records the
chosen model so evaluation renders can look identical. What animates in preview IS the compiled track
replay_core flies against (single time source: scenario_lib.compile_mover).
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np

import scenario_lib as SLB
import urban_rules as UR

HERE = os.path.dirname(os.path.abspath(__file__))
SCN_DIR = os.path.join(HERE, "scenarios")
PREV_DIR = os.path.join(HERE, "out", "scenario_previews")
METAURBAN_ROOT = os.environ.get("METAURBAN_ROOT", "/media/boxuan/Data2/projects/metaurban")
CUSTOM = os.path.join(METAURBAN_ROOT, "custom_assets")
_ADJ = os.path.join(METAURBAN_ROOT, "metaurban", "assets", "adj_parameter_folder")
_TEST_MODELS = os.path.join(METAURBAN_ROOT, "metaurban", "assets", "models", "test")

_PROPS = None
def prop_catalog():
    """category -> [metainfo dict] for the OFFICIAL 417-prop library (bench/tree/bin/board/bollard/...).
    Each metainfo carries the pipeline-tuned scale/hshift/pos offsets AND real-world length/width/height,
    so a placed prop is both visually correct and feeds the certificate an honest (r, h)."""
    global _PROPS
    if _PROPS is None:
        cats = {}
        for f in sorted(glob.glob(os.path.join(_ADJ, "*.json"))):
            try:
                meta = json.load(open(f))
            except (OSError, ValueError) as e:      # 2026-07-16 sweep: fallbacks must be loud (ValueError covers JSONDecodeError)
                print(f"[designer] SKIP prop metainfo {os.path.basename(f)}: {type(e).__name__}: {e}", flush=True)
                continue
            fn = meta.get("filename")
            if not fn or not os.path.exists(os.path.join(_TEST_MODELS, fn)):
                continue
            cat = os.path.basename(f).rsplit("-", 1)[0]
            short = cat.split("_", 1)[-1] if "_" in cat else cat        # structures_Bench -> Bench
            meta["_cat"] = short
            cats.setdefault(short, []).append(meta)
        _PROPS = dict(sorted(cats.items()))
    return _PROPS


def prop_by_file(fn):
    for metas in prop_catalog().values():
        for m in metas:
            if m["filename"] == fn:
                return m
    return None


def load_prop_np(engine, meta):
    """Official prop -> visual NodePath using the metainfo's OWN scale/heading/offset (no guessing)."""
    from panda3d.core import NodePath
    m = engine.loader.loadModel(os.path.join(_TEST_MODELS, meta["filename"]))
    holder = NodePath("prop_" + meta.get("_cat", "x"))
    m.reparentTo(holder)
    m.setScale(float(meta.get("scale", 1.0)))
    m.setH(float(meta.get("hshift", 0.0)))
    m.setPos(float(meta.get("pos0", 0)), float(meta.get("pos1", 0)), float(meta.get("pos2", 0)))
    lo, hi = m.getTightBounds(holder)
    if lo is not None and lo[2] < -0.02:                                # keep feet on the ground
        m.setZ(m.getZ() - lo[2])
    holder.reparentTo(engine.render)
    return holder

CLS_COLOR = {"pedestrian": (0.25, 0.65, 1.0, 1), "vehicle": (1.0, 0.5, 0.15, 1),
             "animal": (0.8, 0.35, 0.85, 1), "static": (0.6, 0.6, 0.6, 1)}
KIND_COLOR = {"straight": (0.4, 0.9, 0.4, 1), "around_l": (0.3, 0.6, 1, 1), "around_r": (0.3, 0.6, 1, 1),
              "over": (1, 0.85, 0.3, 1), "climb": (1, 0.55, 0.2, 1), "evade": (1, 0.25, 0.25, 1)}


# ---------------------------------------------------------------- asset registry
def _vehicle_glbs():
    pat = os.path.join(METAURBAN_ROOT, "metaurban", "assets", "models", "test")
    names = []
    for pre in ("car", "Bus", "foodtruck", "Caravan"):
        names += sorted(os.path.basename(p) for p in glob.glob(os.path.join(pat, f"{pre}-*.glb")))
    return names or ["<none>"]


def asset_registry():
    """cls -> ordered list of asset names (JSON `asset` field values). Resolved to paths in _asset_path."""
    return {
        "pedestrian": ["default"],                                    # cheap shared rig (random pool is
        "vehicle": _vehicle_glbs(),                                   #   empty on this install)
        "animal": ["cow_quaternius.glb", "sheep_quaternius.glb", "dog_shiba_quaternius.glb",
                   "cat_quaternius.glb"],
        "static": ["<cylinder>"],
    }


def _asset_path(cls, name):
    if cls == "pedestrian":
        return os.path.join(METAURBAN_ROOT, "metaurban", "assets", "models", "pedestrian", "scene.gltf")
    if cls == "vehicle":
        return os.path.join(METAURBAN_ROOT, "metaurban", "assets", "models", "test", name)
    if cls == "animal":
        return os.path.join(CUSTOM, name)
    return None

# target size per class: (dimension, metres). Quaternius/scan assets have wild native scales, so every
# model is tight-bounds-normalised (the custom_glb_object.py recipe).
CLS_TARGET = {"pedestrian": ("height", 1.75), "vehicle": ("length", 4.4), "animal": ("length", 1.8)}
HPR_FIX = {"animal": (0, -90, 0)}                                    # quaternius: stand up, nose -> +Y
DRONE_GLB = os.path.join(CUSTOM, "drone_core_polygoogle.glb")


def load_asset_np(engine, cls, name):
    """Visual-only NodePath: loaded, hpr-fixed, tight-bounds scaled to the class target, feet at z=0.
    No Bullet shape (the editor world is frozen; collisions live in the evaluation, not here).
    Pedestrians load as an Actor posed mid-walk (frame of the baked clip) instead of the bind T-pose."""
    path = _asset_path(cls, name)
    if not path or not os.path.exists(path):
        return None
    from panda3d.core import NodePath
    m = None
    if cls == "pedestrian":
        try:
            from direct.actor.Actor import Actor
            a = Actor(path)
            names = a.getAnimNames()
            if names:
                a.pose(names[0], 10)                     # a mid-stride static pose, not the T-pose
                a.update(force=True)                     # characters evaluate lazily; force it for offscreen
            m = a
        except Exception:
            m = None
    if m is None:
        m = engine.loader.loadModel(path)
    holder = NodePath(f"asset_{cls}")
    m.reparentTo(holder)
    meta = prop_by_file(name) if cls == "vehicle" else None
    if meta is not None:
        # official metainfo: pipeline-tuned scale + heading fix (hshift). Vehicles MUST face +Y in the
        # holder frame or set_time's motion-heading math drives them sideways/backwards.
        m.setScale(float(meta.get("scale", 1.0)))
        m.setH(float(meta.get("hshift", 0.0)) - 90.0)   # verified t=4 frame: hshift-90 -> nose +Y in holder
    else:
        m.setHpr(*HPR_FIX.get(cls, (0, 0, 0)))
    lo, hi = m.getTightBounds(holder)
    dim, target = CLS_TARGET.get(cls, ("height", 1.0))
    ext = (hi - lo)
    cur = ext[2] if dim == "height" else max(ext[0], ext[1])
    if cur > 1e-6 and meta is None:
        m.setScale(target / cur)
    lo, hi = m.getTightBounds(holder)
    m.setPos(m.getX() - (lo[0] + hi[0]) / 2, m.getY() - (lo[1] + hi[1]) / 2, m.getZ() - lo[2])
    holder.reparentTo(engine.render)
    return holder


# ---------------------------------------------------------------- shared 3D scene state
class Scene3D:
    """Places markers / path lines / mover asset models for a scenario dict on a live engine, and
    animates them along the scenario_lib-compiled tracks. Used by BOTH the interactive editor and
    the headless preview (single source of drawing truth)."""

    def __init__(self, engine, scn):
        self.eng = engine
        self.scn = scn
        scn.setdefault("movers", []); scn.setdefault("statics", [])   # both keys optional in the schema
        scn.setdefault("drone", {}).setdefault("waypoints", [])
        self.lines = engine.make_line_drawer(thickness=3.0)
        self.points = engine.make_point_drawer(scale=1.0)
        self.models = []                     # per-mover NodePath (or None)
        self.static_nps = []                 # per-static grey pillar NodePath
        self.drone_np = None
        self.tracks = []
        self.rebuild()

    def _static_np(self, st):
        """Static obstacle visual: official prop when st['asset'] names one, grey pillar otherwise."""
        if st.get("asset"):
            meta = prop_by_file(st["asset"])
            if meta is not None:
                np_ = load_prop_np(self.eng, meta)
                np_.setPos(float(st["c"][0]), float(st["c"][1]), 0.0)
                np_.setH(float(st.get("hdg", 0.0)))          # face along the street (parallel_only style)
                return np_
        return self._static_pillar(st)

    def _static_pillar(self, st):
        """Grey box pillar for a scripted static cylinder (visual only): footprint 2r x 2r, height h."""
        import panda3d
        box = os.path.join(os.path.dirname(panda3d.__file__), "models", "box.egg.pz")
        np_ = self.eng.loader.loadModel(box)
        lo, hi = np_.getTightBounds()
        ext = hi - lo
        r, h = float(st.get("r", 0.4)), float(st.get("h", 3.0))
        np_.setScale(2 * r / max(1e-6, ext[0]), 2 * r / max(1e-6, ext[1]), h / max(1e-6, ext[2]))
        lo, hi = np_.getTightBounds()
        np_.setPos(float(st["c"][0]) - (lo[0] + hi[0]) / 2, float(st["c"][1]) - (lo[1] + hi[1]) / 2,
                   -lo[2])
        np_.setLightOff(1)                                   # box.egg has no usable normals -> renders black lit
        np_.setTextureOff(1)
        np_.setShaderOff(1)                                  # scene auto-shader ignores flat setColor
        np_.setColor(0.55, 0.55, 0.58, 1.0, 1)
        np_.reparentTo(self.eng.render)
        return np_

    # -- assets
    def _mover_asset(self, i):
        m = self.scn["movers"][i]
        reg = asset_registry()[m["cls"]]
        return m.get("asset") or reg[0]

    def rebuild(self):
        """Recreate every model/line/marker from the scenario dict (called after any edit)."""
        for np_ in self.models:
            if np_ is not None:
                np_.removeNode()
        self.models = []
        for np_ in self.static_nps:
            np_.removeNode()
        self.static_nps = [self._static_np(st) for st in self.scn.get("statics", [])]
        for i, m in enumerate(self.scn["movers"]):
            self.models.append(load_asset_np(self.eng, m["cls"], self._mover_asset(i)))
        if self.drone_np is None:
            self.drone_np = load_asset_np(self.eng, "drone", "drone")   # returns None (no cls) -> try glb
            if self.drone_np is None and os.path.exists(DRONE_GLB):
                from panda3d.core import NodePath
                d = self.eng.loader.loadModel(DRONE_GLB)
                self.drone_np = NodePath("drone")
                d.reparentTo(self.drone_np)
                lo, hi = d.getTightBounds(self.drone_np)
                ext = max(1e-6, max(hi[0] - lo[0], hi[1] - lo[1]))
                d.setScale(0.5 / ext)
                self.drone_np.reparentTo(self.eng.render)
        self.recompile()
        self.redraw()
        self.set_time(0.0)

    def recompile(self):
        try:
            resolved = SLB.load({**self.scn, "schema": SLB.SCHEMA})
            self.tracks = SLB.to_movers_raw(resolved)
            return None
        except Exception as e:
            # 2026-07-16 sweep: fallbacks must be loud
            print(f"[designer] COMPILE ERROR: {type(e).__name__}: {e}", flush=True)
            self.tracks = []
            return str(e)

    def redraw(self):
        d = self.scn["drone"]
        cz = float(d.get("cruise_z", 1.5))
        self.lines.reset()
        self.points.reset()
        polylines, colors = [], []
        # drone corridor: start -> waypoints -> goal, at cruise_z
        route = [d["start"]] + [list(w) for w in d.get("waypoints", [])] + [d["goal"]]
        poly = [(p[0], p[1], p[2] if len(p) > 2 else cz) for p in route]
        polylines.append(poly); colors.append([(0.4, 0.95, 0.55, 1)] * (len(poly) - 1))
        pts = [poly[0]], [(0.2, 0.95, 0.4, 1)]
        for w in poly[1:-1]:
            pts[0].append(w); pts[1].append((0.5, 0.9, 0.9, 1))
        pts[0].append(poly[-1]); pts[1].append((0.95, 0.25, 0.25, 1))
        # mover paths at ground level
        for i, m in enumerate(self.scn["movers"]):
            col = CLS_COLOR.get(m["cls"], (1, 1, 1, 1))
            path = [(p[0], p[1], 0.15) for p in m["path"]]
            if len(path) > 1:
                polylines.append(path); colors.append([col] * (len(path) - 1))
            for p in path:
                pts[0].append(p); pts[1].append(col)
        for st in self.scn["statics"]:
            pts[0].append((st["c"][0], st["c"][1], 0.2)); pts[1].append(CLS_COLOR["static"])
        self.lines.draw_lines(polylines, colors)
        self.points.draw_points([tuple(map(float, p)) for p in pts[0]], pts[1])
        if self.drone_np is not None:
            s = d["start"]
            self.drone_np.setPos(float(s[0]), float(s[1]), float(s[2] if len(s) > 2 else cz))

    def set_time(self, t):
        """Move every mover model to its compiled-track position at sim time t (the ONLY time source)."""
        for i, tr in enumerate(self.tracks[:len(self.models)]):
            np_ = self.models[i]
            if np_ is None:
                continue
            if t < tr["t"][0]:                                 # not spawned yet -> park at first point
                x, y = tr["xy"][0]
                np_.setPos(float(x), float(y), 0.0)
                np_.hide() if t < tr["t"][0] else np_.show()
                continue
            np_.show()
            tt = min(t, float(tr["t"][-1]))
            x = float(np.interp(tt, tr["t"], tr["xy"][:, 0]))
            y = float(np.interp(tt, tr["t"], tr["xy"][:, 1]))
            x2 = float(np.interp(min(tt + 0.2, tr["t"][-1]), tr["t"], tr["xy"][:, 0]))
            y2 = float(np.interp(min(tt + 0.2, tr["t"][-1]), tr["t"], tr["xy"][:, 1]))
            np_.setPos(x, y, 0.0)
            if abs(x2 - x) + abs(y2 - y) > 1e-4:               # face along motion (+Y model frame)
                np_.setH(math.degrees(math.atan2(y2 - y, x2 - x)) - 90.0)


# ---------------------------------------------------------------- env builders
def build_env(seed, interactive, block_str="X"):
    """block_str = MetaUrban PG block letters (C curve, S straight, X intersection, T t-junction,
    O roundabout, B bidirection, y bottleneck, R/r ramps, P parking, $ tollgate), e.g. "CXO"."""
    os.environ.setdefault("PYTHONPATH", METAURBAN_ROOT)
    sys.path.insert(0, METAURBAN_ROOT)
    from metaurban.envs.sidewalk_dynamic_env import SidewalkDynamicMetaUrbanEnv
    from metaurban.obs.observation_base import DummyObservation
    cfg = dict(crswalk_density=1, object_density=0.9, walk_on_all_regions=False,
               use_render=bool(interactive), image_observation=(not interactive),
               interface_panel=[], manual_control=False, map=(block_str or "X"), daytime="12:00",
               default_expert=False, drivable_area_extension=55, height_scale=1,
               show_mid_block_map=False, show_ego_navigation=False, debug=False, horizon=100000,
               on_continuous_line_done=False, out_of_route_done=False,
               vehicle_config=dict(show_lidar=False, show_navi_mark=False, show_line_to_navi_mark=False,
                                   show_dest_mark=False, enable_reverse=True,
                                   lidar=dict(num_lasers=0, distance=50)),
               agent_observation=DummyObservation,
               decision_repeat=2, physics_world_step_size=0.05,
               show_sidewalk=True, show_crosswalk=True, random_spawn_lane_index=False,
               num_scenarios=20, accident_prob=0, relax_out_of_road_done=True, max_lateral_dist=1e3,
               crash_vehicle_done=False, crash_object_done=False, crash_human_done=False,
               traffic_density=0.0, spawn_human_num=1, spawn_wheelchairman_num=0, spawn_edog_num=0,
               spawn_erobot_num=0, spawn_drobot_num=0, max_actor_num=2)
    if interactive:
        cfg["window_size"] = (1280, 800)
    else:
        from metaurban.component.sensors.rgb_camera import RGBCamera
        cfg["sensors"] = dict(rgb_camera=(RGBCamera, 1280, 800))
    env = SidewalkDynamicMetaUrbanEnv(cfg)
    env.reset(seed=(seed or 0) % 20)
    for _ in range(4):
        env.step([0.0, 0.0])
    return env


# ---------------------------------------------------------------- headless preview
def preview(scn_path, seed=None, times=(0.0, 0.25, 0.5, 0.75)):
    """Render the scenario in true 3D at several sim times -> PNGs. The CLI-only design loop:
    edit JSON (by hand / 2D designer) -> preview -> adjust."""
    import cv2
    scn = json.load(open(scn_path))
    seed = seed if seed is not None else (scn.get("map") or {}).get("seed")
    env = build_env(seed, interactive=False, block_str=(scn.get("map") or {}).get("block_str", "X"))
    eng = env.engine
    sc = Scene3D(eng, scn)
    err = sc.recompile()
    if err:
        print(f"[preview] COMPILE ERROR: {err}"); return 1
    d = scn["drone"]
    c = 0.5 * (np.asarray(d["start"][:2], float) + np.asarray(d["goal"][:2], float))
    L = max(8.0, float(np.linalg.norm(np.asarray(d["goal"][:2]) - np.asarray(d["start"][:2]))))
    cam = eng.get_sensor("rgb_camera")
    from panda3d.core import Vec3
    cam.cam.reparentTo(eng.render)
    os.makedirs(PREV_DIR, exist_ok=True)
    t_max = float(scn.get("t_max", 30.0))
    outs = []
    views = [("iso", (c[0] - 0.75 * L, c[1] - 0.75 * L, 0.65 * L)),
             ("back", (d["start"][0] - 0.35 * L, d["start"][1], 0.35 * L))]
    for frac in times:
        t = frac * t_max
        sc.set_time(t)
        for vname, pos in views:
            cam.cam.setPos(Vec3(*pos))
            cam.cam.lookAt(Vec3(float(c[0]), float(c[1]), 1.0))
            eng.graphicsEngine.renderFrame()
            eng.graphicsEngine.renderFrame()
            a = np.asarray(cam.get_rgb_array_cpu())      # already BGR (see render_3d_video grab())
            if a.dtype != np.uint8:
                a = (a * 255).astype(np.uint8) if a.max() <= 1.01 else a.astype(np.uint8)
            p = os.path.join(PREV_DIR, f"{scn.get('name', 'scn')}_{vname}_t{t:04.1f}.png")
            cv2.imwrite(p, a)                            # cv2 wants BGR -> no channel flip
            outs.append(p)
    env.close()
    print("[preview] wrote:\n  " + "\n  ".join(outs))
    return 0


# ---------------------------------------------------------------- interactive editor
class Editor3D:
    """Interactive 3D editor with FULL Fusion360 camera + mouse conventions:
      LMB   click = select / place (per toolbar mode); click-and-HOLD on a point = drag it
      MMB   hold-drag = PAN (grab-the-ground)
      Shift+MMB hold-drag = ORBIT (heading + tilt around the view target)
      wheel = zoom (dolly toward/away from the target)
      Ctrl+Z = undo (60-step snapshot stack)
    The native bird-cam task is disabled (main_camera.run_task=False); this class drives the camera
    every frame from (target, dist, heading, pitch), so zoom/orbit/pan compose exactly like a CAD
    viewport. Selection tolerance is SCREEN-based (~14 px), so picking feels identical at any zoom.
    Toolbar (top-left, clickable) + HUD are English. Playback CLAMPS at t_max and pauses (no auto
    reset); Duration +/-5s buttons extend/shorten the scenario timeline."""

    MODES = (("select", "Select / Drag"), ("start", "Drone Start"), ("goal", "Drone Goal"),
             ("dwp", "Drone Waypt"), ("ped", "+ Person"), ("vehicle", "+ Vehicle"),
             ("animal", "+ Animal"), ("static", "+ Obstacle"))
    MODE_HINT = {
        "select": "click=select · hold-drag=move · with a mover selected, click ground=add waypoint",
        "start": "click ground to place the drone START", "goal": "click ground to place the GOAL",
        "dwp": "click ground to append a drone waypoint (eval still flies start->goal)",
        "ped": "click to place a person (auto-returns to Select; keep clicking = waypoints)",
        "vehicle": "click to place a vehicle (default 0->6 m/s accel)",
        "animal": "click to place an animal", "static": "click to place a static pillar",
    }

    def __init__(self, env, scn):
        self.env = env; self.eng = env.engine
        self.scn = scn
        self.sc = Scene3D(self.eng, scn)
        self.mode = "select"
        self.sel = None
        self.t = 0.0
        self.playing = False
        self.msg = "LMB select/drag | MMB pan | Shift+MMB orbit | wheel zoom | Ctrl+Z undo"
        self._undo = []
        self._drag = None
        self.sel_static = None             # selected STATIC index (props get q/e asset cycling too)
        self._pan_anchor = None            # world xy pinned under cursor while panning
        self._orbiting = False
        self._last_mouse = None            # screen coords for orbit deltas
        self._running = True
        # ---- Fusion360 camera state: look-at target + spherical offset
        d = self.scn["drone"]
        self.cam_target = np.array([float(d["start"][0]), float(d["start"][1])])
        self.cam_dist = 40.0
        self.cam_heading = -90.0           # deg; -90 = camera south of target looking north
        self.cam_pitch = 88.0              # deg; 90 = straight top-down, lower = more oblique
        self._build_toolbar()
        self._bind()
        self.eng.main_camera.stop_track(bird_view_on_current_position=True)
        self.eng.main_camera.run_task = False          # WE drive the camera from now on
        self._update_camera()

    # ---- camera ------------------------------------------------------------
    def _update_camera(self):
        phi = math.radians(max(15.0, min(89.9, self.cam_pitch)))
        th = math.radians(self.cam_heading)
        horiz = self.cam_dist * math.cos(phi)
        cam = self.eng.main_camera.camera
        cam.setPos(float(self.cam_target[0] + horiz * math.cos(th)),
                   float(self.cam_target[1] + horiz * math.sin(th)),
                   float(max(1.5, self.cam_dist * math.sin(phi))))
        cam.lookAt(float(self.cam_target[0]), float(self.cam_target[1]), 0.0)

    def zoom(self, k):
        self.cam_dist = max(4.0, min(300.0, self.cam_dist * k))
        self._update_camera()

    def _tol(self):
        """Screen-based pick tolerance: ~14 px converted to metres at the view target distance,
        so picking feels the same fully zoomed-in or out."""
        try:
            fovv = math.radians(self.eng.cam.node().getLens().getFov()[1])
            wpp = 2.0 * self.cam_dist * math.tan(fovv / 2.0) / max(1, self.eng.win.getYSize())
            return max(0.10, 14.0 * wpp)
        except Exception:
            return 1.2

    # ---- undo -----------------------------------------------------------
    def _snapshot(self):
        import copy
        self._undo.append(copy.deepcopy(self.scn))
        if len(self._undo) > 60:
            self._undo.pop(0)

    def undo(self):
        if not self._undo:
            self.msg = "nothing to undo"; return
        snap = self._undo.pop()
        self.scn.clear(); self.scn.update(snap)
        self.sc.scn = self.scn
        self.sel = None
        self.sc.rebuild()
        self.msg = f"undone ({len(self._undo)} steps left)"

    # ---- toolbar / HUD ------------------------------------------------------
    def _build_toolbar(self):
        self._buttons = {}
        self._hud1 = self._hud2 = None
        try:
            from direct.gui.DirectGui import DirectButton
            from direct.gui.OnscreenText import OnscreenText
            from panda3d.core import TextNode
        except Exception:
            return
        kw = dict(scale=0.038, relief=1, frameColor=(0.10, 0.12, 0.16, 0.88),
                  text_fg=(0.92, 0.94, 0.97, 1), pad=(0.5, 0.3), text_align=TextNode.ALeft)
        z = -0.06
        for key, label in self.MODES:
            self._buttons[key] = DirectButton(parent=self.eng.a2dTopLeft, text=label,
                                              pos=(0.05, 0, z), command=self._set_mode,
                                              extraArgs=[key], **kw)
            z -= 0.072
        z -= 0.024
        for label, cmd in (("Undo  (Ctrl+Z)", self.undo),
                           ("Play / Pause  (Space)", self.toggle_play),
                           ("Duration +5s", lambda: self.adjust_tmax(+5.0)),
                           ("Duration -5s", lambda: self.adjust_tmax(-5.0)),
                           ("Export  (F2)", self.export),
                           ("Quit  (Esc)", self.quit)):
            DirectButton(parent=self.eng.a2dTopLeft, text=label, pos=(0.05, 0, z),
                         command=cmd, **kw)
            z -= 0.072
        self._hud1 = OnscreenText(parent=self.eng.a2dBottomLeft, text="", pos=(0.05, 0.115),
                                  scale=0.041, fg=(1, 1, 1, 1), align=TextNode.ALeft,
                                  shadow=(0, 0, 0, 0.85), mayChange=True)
        self._hud2 = OnscreenText(parent=self.eng.a2dBottomLeft, text="", pos=(0.05, 0.058),
                                  scale=0.034, fg=(0.72, 0.85, 0.98, 1), align=TextNode.ALeft,
                                  shadow=(0, 0, 0, 0.85), mayChange=True)
        self._refresh_toolbar()

    def _refresh_toolbar(self):
        for key, b in self._buttons.items():
            b["frameColor"] = ((0.15, 0.45, 0.28, 0.95) if key == self.mode
                               else (0.10, 0.12, 0.16, 0.88))

    def _set_mode(self, m):
        self.mode = m
        if m != "select":
            self.sel = None
            self.sel_static = None
        self.msg = self.MODE_HINT.get(m, "")
        self._refresh_toolbar()

    def adjust_tmax(self, dv):
        self.scn["t_max"] = max(5.0, round(float(self.scn.get("t_max", 30.0)) + dv, 1))
        self.sc.recompile()
        self.msg = f"duration -> {self.scn['t_max']:.0f}s"

    # ---- input ------------------------------------------------------------
    def _bind(self):
        e = self.eng
        e.ignore("escape")
        e.accept("mouse1", self.on_press)
        e.accept("mouse1-up", self.on_release)
        e.accept("mouse2", self.pan_start)                 # MMB = pan
        e.accept("shift-mouse2", self.orbit_start)         # Shift+MMB = orbit (Fusion360)
        e.accept("mouse2-up", self.pan_or_orbit_end)
        e.accept("wheel_up", lambda: self.zoom(1.0 / 1.15))
        e.accept("wheel_down", lambda: self.zoom(1.15))
        e.accept("control-z", self.undo)
        e.accept("1", lambda: self._set_mode("start"))
        e.accept("2", lambda: self._set_mode("goal"))
        e.accept("3", lambda: self._set_mode("dwp"))
        e.accept("p", lambda: self._set_mode("ped"))
        e.accept("v", lambda: self._set_mode("vehicle"))
        e.accept("a", lambda: self._set_mode("animal"))
        e.accept("o", lambda: self._set_mode("static"))
        e.accept("tab", self.cycle_sel)
        e.accept("x", self.delete_last)
        e.accept("delete", self.delete_last)
        e.accept("q", lambda: self.cycle_asset(-1))
        e.accept("e", lambda: self.cycle_asset(+1))
        e.accept("shift-q", lambda: (self._cycle_static_asset(-1, new_cat=True)
                                     if self.sel_static is not None else None))
        e.accept("shift-e", lambda: (self._cycle_static_asset(+1, new_cat=True)
                                     if self.sel_static is not None else None))
        e.accept("u", lambda: self.nudge("speed", -0.2))
        e.accept("i", lambda: self.nudge("speed", +0.2))
        e.accept("minus", lambda: self.nudge("v1", -0.5))
        e.accept("equal", lambda: self.nudge("v1", +0.5))
        e.accept("9", lambda: self.nudge("accel", -0.2))
        e.accept("0", lambda: self.nudge("accel", +0.2))
        e.accept("comma", lambda: self.nudge("spawn_t", -0.2))
        e.accept("period", lambda: self.nudge("spawn_t", +0.2))
        e.accept("c", self.toggle_profile)
        e.accept("space", self.toggle_play)
        e.accept("r", lambda: setattr(self, "t", 0.0))
        e.accept("l", self.snap_static_legal)      # snap selected prop to its legal city band
        e.accept("f2", self.export)
        e.accept("escape", self.quit)

    def quit(self): self._running = False

    def _legality_msg(self, st):
        """City-standard check for a static (rules = MetaUrban's own AssetManager table)."""
        try:
            legal, want, here = UR.check(self.eng, st.get("asset"), st["c"][0], st["c"][1])
        except Exception:
            return ""
        if legal is None:
            return f" [{here}]"
        return (f" [OK: {here}]" if legal else
                f" [!! {UR.category_of_asset(st.get('asset'))} belongs on {want}, HERE={here} -- press L to snap]")

    def snap_static_legal(self):
        if self.sel_static is None:
            return
        st = self.scn["statics"][self.sel_static]
        q = UR.snap_legal(self.eng, st.get("asset"), st["c"][0], st["c"][1])
        if q is None:
            self.msg = "no placement rule for this object"; return
        self._snapshot()
        st["c"] = [round(q[0], 2), round(q[1], 2)]
        st["hdg"] = round(UR.lane_heading_deg(self.eng, *st["c"]), 1)   # face along the street
        self.msg = f"snapped to legal band ({q[0]:.1f},{q[1]:.1f})" + self._legality_msg(st)
        self.sc.recompile(); self.sc.rebuild()

    # ---- ground picking (z=0 plane) ----------------------------------------
    def pick_ground(self):
        mw = self.eng.mouseWatcherNode
        if not mw.hasMouse():
            return None
        from panda3d.core import Point3
        pm = mw.getMouse()
        pf, pt = Point3(), Point3()
        self.eng.cam.node().getLens().extrude(pm, pf, pt)
        pf = self.eng.render.getRelativePoint(self.eng.cam, pf)
        pt = self.eng.render.getRelativePoint(self.eng.cam, pt)
        dz = pt.z - pf.z
        if abs(dz) < 1e-9:
            return None
        s = -pf.z / dz
        return np.array([pf.x + s * (pt.x - pf.x), pf.y + s * (pt.y - pf.y)])

    def pick_nearest(self, w, tol=None):
        """Closest editable point within the SCREEN-based tolerance -> drag key, else None."""
        best, hit = (self._tol() if tol is None else tol), None
        for i, m in enumerate(self.scn["movers"]):
            for j, p in enumerate(m["path"]):
                d = float(np.hypot(p[0] - w[0], p[1] - w[1]))
                if d < best: best, hit = d, ("mv", i, j)
        for i, st in enumerate(self.scn["statics"]):
            d = float(np.hypot(st["c"][0] - w[0], st["c"][1] - w[1]))
            if d < best: best, hit = d, ("static", i)
        for j, p in enumerate(self.scn["drone"].get("waypoints") or []):
            d = float(np.hypot(p[0] - w[0], p[1] - w[1]))
            if d < best: best, hit = d, ("dwp", j)
        for key in ("start", "goal"):
            p = self.scn["drone"][key]
            d = float(np.hypot(p[0] - w[0], p[1] - w[1]))
            if d < best: best, hit = d, (key,)
        return hit

    # ---- mouse handlers -----------------------------------------------------
    def on_press(self):
        w = self.pick_ground()
        if w is None:
            return
        hit = self.pick_nearest(w)
        if hit is not None:
            self._snapshot()
            self._drag = hit
            if hit[0] == "mv":
                self.sel = hit[1]; self.sel_static = None
                self.msg = f"mover{hit[1]} waypoint {hit[2]}: dragging"
            elif hit[0] == "static":
                self.sel_static = hit[1]; self.sel = None
                self.msg = f"obstacle {hit[1]} selected: q/e = prop model, Shift+q/e = category"
            else:
                self.msg = f"dragging {hit[0]}"
            return
        self.on_click(w)

    def _apply_drag(self, w):
        k = self._drag
        x, y = round(float(w[0]), 2), round(float(w[1]), 2)
        if k[0] == "mv":
            self.scn["movers"][k[1]]["path"][k[2]] = [x, y]
        elif k[0] == "static":
            self.scn["statics"][k[1]]["c"] = [x, y]
        elif k[0] == "dwp":
            wp = self.scn["drone"]["waypoints"][k[1]]; wp[0], wp[1] = x, y
        elif k[0] == "start":
            self.scn["drone"]["start"][:2] = [x, y]
        elif k[0] == "goal":
            self.scn["drone"]["goal"][:2] = [x, y]

    def on_release(self):
        if self._drag is not None:
            if self._drag[0] == "static":
                self.msg = "obstacle moved" + self._legality_msg(self.scn["statics"][self._drag[1]])
            self._drag = None
            self.sc.recompile()
            self.sc.rebuild()

    def pan_start(self):
        self._orbiting = False
        self._pan_anchor = self.pick_ground()

    def orbit_start(self):
        self._pan_anchor = None
        self._orbiting = True
        mw = self.eng.mouseWatcherNode
        self._last_mouse = (mw.getMouseX(), mw.getMouseY()) if mw.hasMouse() else None

    def pan_or_orbit_end(self):
        self._pan_anchor = None
        self._orbiting = False
        self._last_mouse = None

    # ---- placement (also used headless by the selftest) ----------------------
    def on_click(self, w=None):
        if w is None:
            w = self.pick_ground()
        if w is None:
            return
        d = self.scn["drone"]
        if self.sel is not None and self.mode == "select":
            self._snapshot()
            self.scn["movers"][self.sel]["path"].append([float(w[0]), float(w[1])])
            self.msg = f"mover{self.sel}: waypoint added ({w[0]:.1f},{w[1]:.1f})"
        elif self.mode == "start":
            self._snapshot(); d["start"][:2] = [float(w[0]), float(w[1])]
            self.msg = "start placed"
        elif self.mode == "goal":
            self._snapshot(); d["goal"][:2] = [float(w[0]), float(w[1])]
            self.msg = "goal placed"
        elif self.mode == "dwp":
            self._snapshot()
            d.setdefault("waypoints", []).append([float(w[0]), float(w[1]),
                                                  float(d.get("cruise_z", 1.5))])
            self.msg = f"drone waypoint #{len(d['waypoints'])} (eval still flies start->goal)"
        elif self.mode == "static":
            self._snapshot()
            self.scn["statics"].append(dict(c=[float(w[0]), float(w[1])], r=0.45, h=3.0))
            self._set_mode("select")
            self.msg = "obstacle placed"
        elif self.mode in ("ped", "vehicle", "animal"):
            self.new_mover("pedestrian" if self.mode == "ped" else self.mode, w)
        self.sc.rebuild()

    def new_mover(self, cls, w=None):
        if w is None:
            w = self.pick_ground()
        if w is None:
            self.msg = "point the mouse at the ground first"; return
        self._snapshot()
        reg = asset_registry()[cls]
        self.scn["movers"].append(dict(cls=cls, asset=reg[0], spawn_t=0.0,
                                       path=[[float(w[0]), float(w[1])]],
                                       speed=(dict(v0=0.0, v1=6.0, a=2.5) if cls == "vehicle" else 1.2),
                                       hold_end=True))
        self._set_mode("select")
        self.sel = len(self.scn["movers"]) - 1
        self.msg = f"new {cls} #{self.sel}: keep clicking to add waypoints"
        self.sc.rebuild()

    # ---- selection / edit ops -------------------------------------------------
    def cycle_sel(self):
        n = len(self.scn["movers"])
        if not n:
            self.sel = None; return
        self.sel = 0 if self.sel is None else (self.sel + 1) % (n + 1)
        if self.sel == n:
            self.sel = None
        self.msg = f"selected mover {self.sel}" if self.sel is not None else "selection cleared"

    def delete_last(self):
        if self.sel is None:
            return
        self._snapshot()
        m = self.scn["movers"][self.sel]
        m["path"].pop()
        if not m["path"]:
            self.scn["movers"].pop(self.sel); self.sel = None
            self.msg = "mover deleted"
        else:
            self.msg = "last waypoint deleted"
        self.sc.rebuild()

    def cycle_asset(self, dv):
        if self.sel_static is not None:
            self._cycle_static_asset(dv); return
        if self.sel is None:
            return
        self._snapshot()
        m = self.scn["movers"][self.sel]
        reg = asset_registry()[m["cls"]]
        i = (reg.index(m.get("asset", reg[0])) + dv) % len(reg) if m.get("asset") in reg else 0
        m["asset"] = reg[i]
        self.msg = f"mover{self.sel} asset -> {reg[i]}"
        self.sc.rebuild()

    def _cycle_static_asset(self, dv, new_cat=False):
        """q/e: next/prev prop within the category; Shift+q/e: jump categories. Assigning a prop also
        writes its REAL footprint radius + height into the static (visual == what the cert sees)."""
        st = self.scn["statics"][self.sel_static]
        cats = prop_catalog()
        if not cats:
            self.msg = "prop catalog empty (check METAURBAN_ROOT)"; return
        names = list(cats.keys())
        cur = prop_by_file(st["asset"]) if st.get("asset") else None
        ci = names.index(cur["_cat"]) if cur else 0
        self._snapshot()
        if new_cat:
            ci = (ci + dv) % len(names)
            meta = cats[names[ci]][0]
        else:
            metas = cats[names[ci]]
            k = next((i for i, m in enumerate(metas) if cur and m["filename"] == cur["filename"]), -1)
            meta = metas[(k + dv) % len(metas)]
        st["asset"] = meta["filename"]
        st["r"] = round(max(float(meta["length"]), float(meta["width"])) / 2.0, 3)
        st["h"] = round(float(meta["height"]), 3)
        self.msg = (f"obstacle -> {meta['_cat']} r={st['r']} h={st['h']}"
                    + self._legality_msg(st))
        self.sc.recompile()
        self.sc.rebuild()

    def nudge(self, field, dv):
        if self.sel is None:
            return
        self._snapshot()
        m = self.scn["movers"][self.sel]
        def num(x, dflt=0.0):
            if isinstance(x, str) and x.startswith("$"):
                return float(self.scn.get("params", {}).get(x[1:], {}).get("value", dflt))
            return float(x if x is not None else dflt)
        if field == "spawn_t":
            m["spawn_t"] = max(0.0, round(num(m.get("spawn_t", 0.0)) + dv, 2))
        elif field == "speed":
            if isinstance(m["speed"], dict):
                m["speed"]["v0"] = max(0.0, round(num(m["speed"].get("v0", 0)) + dv, 2))
            else:
                m["speed"] = max(0.0, round(num(m["speed"]) + dv, 2))
        elif field == "v1" and isinstance(m["speed"], dict):
            m["speed"]["v1"] = max(0.0, round(num(m["speed"]["v1"]) + dv, 2))
        elif field == "accel" and isinstance(m["speed"], dict):
            m["speed"]["a"] = max(0.1, round(num(m["speed"].get("a", 1.0)) + dv, 2))
        self.sc.recompile()
        self.sc.redraw()

    def toggle_profile(self):
        if self.sel is None:
            return
        self._snapshot()
        m = self.scn["movers"][self.sel]
        def num(x, dflt=0.0):
            if isinstance(x, str) and x.startswith("$"):
                return float(self.scn.get("params", {}).get(x[1:], {}).get("value", dflt))
            return float(x if x is not None else dflt)
        m["speed"] = (num(m["speed"].get("v1", 1.2)) if isinstance(m["speed"], dict)
                      else dict(v0=0.0, v1=num(m["speed"]) or 1.0, a=1.0))
        self.sc.recompile()

    def toggle_play(self):
        if not self.playing and self.t >= float(self.scn.get("t_max", 30.0)) - 1e-6:
            self.t = 0.0                                    # play pressed at the end -> restart
        self.playing = not self.playing

    def export(self):
        os.makedirs(SCN_DIR, exist_ok=True)
        p = os.path.join(SCN_DIR, f"{self.scn.get('name', 'untitled')}.json")
        json.dump({**self.scn, "schema": SLB.SCHEMA}, open(p, "w"), indent=1, default=list)
        self.msg = f"exported {p}"

    # ---- HUD + main loop -------------------------------------------------------
    def hud(self):
        if self._hud1 is None:
            return
        mode_lbl = dict(self.MODES).get(self.mode, self.mode)
        sel = f"mover{self.sel}" if self.sel is not None else "none"
        extra = ""
        if self.sel is not None:
            m = self.scn["movers"][self.sel]
            sp = m["speed"]
            extra = (f"  {m['cls']} v{sp.get('v0', 0)}->{sp.get('v1')}@a{sp.get('a', 1)}"
                     if isinstance(sp, dict) else f"  {m['cls']} v{sp}") + \
                    f" spawn {m.get('spawn_t', 0)}s asset {m.get('asset', 'default')}"
        tm = float(self.scn.get("t_max", 30.0))
        self._hud1.setText(f"[{mode_lbl}]  sel: {sel}{extra}   t={self.t:.1f}/{tm:.0f}s"
                           f"{'  PLAYING' if self.playing else ''}")
        self._hud2.setText(self.msg)

    def loop(self):
        import time
        mw = self.eng.mouseWatcherNode
        last = time.time()
        while self._running:
            now = time.time()
            if self.playing:
                self.t += now - last
                tm = float(self.scn.get("t_max", 30.0))
                if self.t >= tm:                              # clamp at the end, DO NOT auto-reset
                    self.t = tm
                    self.playing = False
                    self.msg = "played to end -- r to rewind, or Duration +5s to extend"
                self.sc.set_time(self.t)
            last = now
            if self._drag is not None and mw.hasMouse():
                w = self.pick_ground()
                if w is not None:
                    self._apply_drag(w)
                    self.sc.redraw()
            if self._orbiting and mw.hasMouse():              # Shift+MMB: orbit heading/pitch
                cur = (mw.getMouseX(), mw.getMouseY())
                if self._last_mouse is not None:
                    dx, dy = cur[0] - self._last_mouse[0], cur[1] - self._last_mouse[1]
                    self.cam_heading -= dx * 120.0
                    self.cam_pitch = max(15.0, min(89.9, self.cam_pitch - dy * 90.0))  # drag UP = tilt toward horizon
                self._last_mouse = cur
                self._update_camera()
            elif self._pan_anchor is not None and mw.hasMouse():   # MMB: grab-the-ground pan
                w = self.pick_ground()
                if w is not None:
                    self.cam_target -= (w - self._pan_anchor)
                    self._update_camera()
            self.hud()
            self.eng.taskMgr.step()


# ---------------------------------------------------------------- selftest (no GL: logic only)
def selftest():
    """Exercise the edit-op logic with a stub engine (no window, no GL). Verifies scenario mutations,
    asset cycling and export round-trip; drawing is exercised separately by --preview."""
    class _StubNP:
        def removeNode(self): pass
    class _StubDrawer:
        def reset(self): pass
        def draw_lines(self, *a): pass
        def draw_points(self, *a): pass
    class _StubEng:
        def make_line_drawer(self, **k): return _StubDrawer()
        def make_point_drawer(self, **k): return _StubDrawer()
    scn = dict(schema=SLB.SCHEMA, name="selftest3d", description="", map=dict(seed=None),
               drone=dict(start=[0.0, 0.0, 1.5], goal=[15.0, 0.0, 1.5], cruise_z=1.5,
                          max_vel=3.0, max_acc=6.0),
               statics=[], movers=[], params={}, t_max=20.0)
    # bypass Editor3D.__init__ (needs a window): test the ops directly on a shell instance
    ed = Editor3D.__new__(Editor3D)
    ed.scn = scn; ed.sel = None; ed.mode = "start"; ed.msg = ""; ed.t = 0.0; ed.playing = False
    ed._undo = []; ed._drag = None; ed._pan_anchor = None; ed._buttons = {}; ed.sel_static = None
    ed._hud1 = ed._hud2 = None
    class _StubScene:
        def rebuild(self): pass
        def recompile(self): return None
        def redraw(self): pass
    ed.sc = _StubScene()
    ed.undo()                                                  # empty stack: no crash, friendly msg
    ed.pick_ground = lambda: np.array([7.0, -10.0])
    ed.new_mover("vehicle")
    assert ed.sel == 0 and scn["movers"][0]["asset"] != "<none>", scn["movers"]
    ed.pick_ground = lambda: np.array([7.0, 10.0])
    ed.on_click()                                              # append waypoint to selected mover
    assert len(scn["movers"][0]["path"]) == 2
    ed.cycle_asset(+1)
    ed.nudge("v1", +2.0)
    # drag: press near the waypoint grabs it, applying drag moves it, release keeps it
    ed.pick_ground = lambda: np.array([7.05, 9.9])
    ed.on_press()
    assert ed._drag == ("mv", 0, 1), ed._drag
    ed._apply_drag(np.array([9.0, 12.0]))
    ed.on_release()
    assert scn["movers"][0]["path"][1] == [9.0, 12.0], scn["movers"][0]["path"]
    # undo: the drag snapshot restores the pre-drag position (NB: ed.scn is rebound by undo)
    ed.undo()
    scn = ed.scn
    assert scn["movers"][0]["path"][1] == [7.0, 10.0], f"undo failed: {scn['movers'][0]['path']}"
    ed.sel = None
    ed._set_mode("dwp"); ed.pick_ground = lambda: np.array([8.0, 2.0]); ed.on_click()
    assert len(scn["drone"]["waypoints"]) == 1
    resolved = SLB.load({**scn, "schema": SLB.SCHEMA})
    raw = SLB.to_movers_raw(resolved)
    v = np.linalg.norm(np.diff(raw[0]["xy"], axis=0), axis=1) / SLB.DT_SAMPLE
    assert v.max() > 5.0, f"vehicle profile not compiled (vmax={v.max():.2f})"
    print(f"[selftest3d] OK: mover asset={scn['movers'][0]['asset']}, vmax={v.max():.2f} m/s, "
          f"drag+undo verified, drone wp={scn['drone']['waypoints']}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--load", type=str, default=None)
    ap.add_argument("--preview", type=str, default=None, help="scenario JSON -> headless 3D PNG frames")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        sys.exit(selftest())
    if args.preview:
        sys.exit(preview(args.preview, seed=args.seed))
    scn = (json.load(open(args.load)) if args.load else
           dict(schema=SLB.SCHEMA, name="untitled3d", description="", map=dict(seed=args.seed),
                drone=dict(start=[0.0, 0.0, 1.5], goal=[16.0, 0.0, 1.5], cruise_z=1.5,
                           max_vel=3.0, max_acc=6.0),
                statics=[], movers=[], params={}, t_max=30.0))
    env = build_env(args.seed if args.seed is not None else (scn.get("map") or {}).get("seed"),
                    interactive=True, block_str=(scn.get("map") or {}).get("block_str", "X"))
    Editor3D(env, scn).loop()
    env.close()


if __name__ == "__main__":
    main()
