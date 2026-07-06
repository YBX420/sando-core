"""scenario_designer — interactive scenario editor ON the native MetaUrban map (pygame).

Run (metaurban conda env; DISPLAY needed for the interactive window):
  python scenario_designer.py --seed 3                 design on MetaUrban scene 3 (0-19); first run renders
                                                       the full map once (~40 s) then caches it -> instant after
  python scenario_designer.py                          abstract corridor mode (grid backdrop, no env needed)
  python scenario_designer.py --load scenarios/x.json  edit an existing scenario (seed read from its map.seed)
  python scenario_designer.py --replay out/h.json      overlay a recorded run_replay hist (drone path by kind)
  python scenario_designer.py --selftest               headless self-test (no display; exercises edit+export)

Controls
  mouse L        place (per mode) / select+drag nearest waypoint
  mouse R drag   pan          wheel  zoom
  s / g          set drone START / GOAL at cursor
  p / v / a / o  new Pedestrian / Vehicle / Animal / static Obstacle at cursor (then L-clicks append waypoints)
  TAB / x        cycle selection / delete selected waypoint (mover when last one goes)
  [ ]            speed -/+ 0.2         c   toggle const-speed <-> accel profile {v0,v1,a}
  9 0            accel -/+ 0.2         { } profile v1 -/+ 0.5
  , .            spawn_t -/+ 0.2       e   toggle hold_end
  SPACE          play/pause preview    LEFT/RIGHT scrub time   r  rewind
  F2             export JSON           h   toggle help
The designer edits the SAME schema scenario_lib compiles for replay_core -- what you see animate here is
exactly what the evaluation flies against.
"""
import argparse
import json
import os
import sys

import numpy as np

import scenario_lib as SLB

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, "out", "designer_cache")
SCN_DIR = os.path.join(HERE, "scenarios")
CLS_COLOR = {"pedestrian": (60, 170, 255), "vehicle": (255, 120, 40),
             "animal": (200, 90, 220), "static": (150, 150, 150)}
FILM = 4000            # full-map film pixels
PPM = 8.0              # film pixels per metre (4000/8 = 500 m coverage)


# ---------------------------------------------------------------- native map film (cached)
def build_map_film(seed):
    """Render MetaUrban scene <seed> top-down ONCE to a full-map film; cache PNG + world transform."""
    os.makedirs(CACHE, exist_ok=True)
    png = os.path.join(CACHE, f"seed{seed}.png")
    meta = os.path.join(CACHE, f"seed{seed}.json")
    if os.path.exists(png) and os.path.exists(meta):
        return png, json.load(open(meta))
    print(f"[designer] no cache for seed {seed}: constructing MetaUrban (~40 s once) ...", flush=True)
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")   # env's own pygame must not open a window
    import cv2
    from metaurban.envs.sidewalk_dynamic_env import SidewalkDynamicMetaUrbanEnv
    from metaurban.obs.observation_base import DummyObservation
    cfg = dict(crswalk_density=1, object_density=0.9, walk_on_all_regions=False,
               use_render=False, image_observation=False, sensors=dict(),
               interface_panel=[], manual_control=False, map='X', daytime="12:00",
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
               spawn_erobot_num=0, spawn_drobot_num=0, max_actor_num=2)   # near-empty: ORCA needs >=1 agent
    env = SidewalkDynamicMetaUrbanEnv(cfg)
    env.reset(seed=seed % 20)
    center = [float(env.agent.position[0]), float(env.agent.position[1])]
    img = env.render(mode="topdown", window=False, screen_size=(FILM, FILM), film_size=(FILM, FILM),
                     scaling=PPM, camera_position=tuple(center))
    env.close()
    cv2.imwrite(png, img[..., ::-1])
    m = dict(seed=seed, center=center, ppm=PPM, film=FILM)
    json.dump(m, open(meta, "w"))
    print(f"[designer] cached {png}")
    return png, m


# ---------------------------------------------------------------- editor
class Designer:
    def __init__(self, scn, map_meta=None, film_path=None, replay=None, headless=False):
        import pygame
        self.pg = pygame
        self.scn = scn
        self.sel = None                        # (mover_idx, wp_idx) | ("static", i) | None
        self.t = 0.0
        self.playing = False
        self.dirty = True
        self.tracks = []
        self.help = True
        self.replay = replay or []
        self.msg = "h: help  |  F2: export"
        if headless:
            os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.init()
        self.W, self.H = 1400, 900
        self.screen = pygame.display.set_mode((self.W, self.H))
        pygame.display.set_caption(f"sando scenario designer — {scn.get('name', 'untitled')}")
        self.font = pygame.font.SysFont("monospace", 13)
        # world<->screen: screen = (world - view_c) * ppm * zoom + (W/2, H/2), y flipped
        self.view_c = np.asarray(scn["drone"]["start"][:2], float).copy()
        self.zoom = 3.0
        self.film = None
        self.mmeta = map_meta
        if film_path and os.path.exists(film_path):
            self.film = pygame.image.load(film_path)

    # ---- transforms
    def w2s(self, w):
        z = self.mmeta["ppm"] * self.zoom / 8.0 if self.mmeta else self.zoom * 10.0
        return (int((w[0] - self.view_c[0]) * z + self.W / 2),
                int(-(w[1] - self.view_c[1]) * z + self.H / 2))

    def s2w(self, s):
        z = self.mmeta["ppm"] * self.zoom / 8.0 if self.mmeta else self.zoom * 10.0
        return np.array([(s[0] - self.W / 2) / z + self.view_c[0],
                         -(s[1] - self.H / 2) / z + self.view_c[1]])

    def _pxpm(self):
        return self.mmeta["ppm"] * self.zoom / 8.0 if self.mmeta else self.zoom * 10.0

    # ---- edit ops (all pure on self.scn; designer state only)
    def set_start(self, w):  self.scn["drone"]["start"][:2] = [float(w[0]), float(w[1])]; self.dirty = True
    def set_goal(self, w):   self.scn["drone"]["goal"][:2] = [float(w[0]), float(w[1])]; self.dirty = True

    def add_mover(self, cls, w):
        self.scn["movers"].append(dict(cls=cls, spawn_t=0.0, path=[[float(w[0]), float(w[1])]],
                                       speed=(0.0 if cls == "static" else 1.2), hold_end=True))
        self.sel = (len(self.scn["movers"]) - 1, 0)
        self.dirty = True

    def add_static(self, w):
        self.scn["statics"].append(dict(c=[float(w[0]), float(w[1])], r=0.4, h=3.0))
        self.sel = ("static", len(self.scn["statics"]) - 1)
        self.dirty = True

    def append_wp(self, w):
        if isinstance(self.sel, tuple) and self.sel[0] != "static" and self.sel[0] is not None \
                and isinstance(self.sel[0], int):
            m = self.scn["movers"][self.sel[0]]
            m["path"].append([float(w[0]), float(w[1])])
            if len(m["path"]) > 1 and not (isinstance(m["speed"], dict) or float(m["speed"]) > 0):
                m["speed"] = 1.2                      # a path implies motion
            self.sel = (self.sel[0], len(m["path"]) - 1)
            self.dirty = True
            return True
        return False

    def nearest(self, w, r_px=14):
        """closest editable point (waypoint / static / start / goal) within r_px -> selection key."""
        best, bk = r_px / self._pxpm(), None
        for i, m in enumerate(self.scn["movers"]):
            for j, p in enumerate(m["path"]):
                d = float(np.hypot(p[0] - w[0], p[1] - w[1]))
                if d < best: best, bk = d, (i, j)
        for i, st in enumerate(self.scn["statics"]):
            d = float(np.hypot(st["c"][0] - w[0], st["c"][1] - w[1]))
            if d < best: best, bk = d, ("static", i)
        for key in ("start", "goal"):
            p = self.scn["drone"][key]
            d = float(np.hypot(p[0] - w[0], p[1] - w[1]))
            if d < best: best, bk = d, (key, 0)
        return bk

    def move_sel(self, w):
        if not self.sel: return
        k = self.sel
        if k[0] == "static":
            self.scn["statics"][k[1]]["c"] = [float(w[0]), float(w[1])]
        elif k[0] in ("start", "goal"):
            self.scn["drone"][k[0]][:2] = [float(w[0]), float(w[1])]
        else:
            self.scn["movers"][k[0]]["path"][k[1]] = [float(w[0]), float(w[1])]
        self.dirty = True

    def delete_sel(self):
        k = self.sel
        if not k or k[0] in ("start", "goal"): return
        if k[0] == "static":
            self.scn["statics"].pop(k[1])
        else:
            m = self.scn["movers"][k[0]]
            m["path"].pop(k[1])
            if not m["path"]:
                self.scn["movers"].pop(k[0])
        self.sel = None
        self.dirty = True

    def _num(self, x, default=0.0):
        """Resolve a numeric field that may be a '$param' reference (editing a templated field detaches it
        from the template at its current resolved value)."""
        if isinstance(x, str) and x.startswith("$"):
            return float(self.scn.get("params", {}).get(x[1:], {}).get("value", default))
        return float(x if x is not None else default)

    def nudge(self, field, dv):
        k = self.sel
        if not k or k[0] in ("start", "goal", "static"): return
        m = self.scn["movers"][k[0]]
        if field == "spawn_t":
            m["spawn_t"] = max(0.0, round(self._num(m.get("spawn_t", 0.0)) + dv, 2))
        elif field == "speed":
            if isinstance(m["speed"], dict): m["speed"]["v0"] = max(0.0, round(self._num(m["speed"].get("v0", 0)) + dv, 2))
            else: m["speed"] = max(0.0, round(self._num(m["speed"]) + dv, 2))
        elif field == "v1" and isinstance(m["speed"], dict):
            m["speed"]["v1"] = max(0.0, round(self._num(m["speed"]["v1"]) + dv, 2))
        elif field == "accel" and isinstance(m["speed"], dict):
            m["speed"]["a"] = max(0.1, round(self._num(m["speed"].get("a", 1.0)) + dv, 2))
        self.dirty = True

    def toggle_profile(self):
        k = self.sel
        if not k or not isinstance(k[0], int): return
        m = self.scn["movers"][k[0]]
        if isinstance(m["speed"], dict):
            m["speed"] = self._num(m["speed"]["v1"])
        else:
            v = self._num(m["speed"]) or 1.0
            m["speed"] = dict(v0=0.0, v1=v, a=1.0)
        self.dirty = True

    def toggle_hold(self):
        k = self.sel
        if k and isinstance(k[0], int):
            m = self.scn["movers"][k[0]]
            m["hold_end"] = not m.get("hold_end", True)
            self.dirty = True

    def recompile(self):
        try:
            resolved = SLB.load({**self.scn, "schema": SLB.SCHEMA})
            self.tracks = SLB.to_movers_raw(resolved)
            self.msg = f"compiled {len(self.tracks)} tracks   t_max={self.scn.get('t_max', 30)}s"
        except Exception as e:
            self.tracks = []
            self.msg = f"COMPILE ERROR: {e}"
        self.dirty = False

    def export(self, path=None):
        path = path or os.path.join(SCN_DIR, f"{self.scn.get('name', 'untitled')}.json")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        json.dump({**self.scn, "schema": SLB.SCHEMA}, open(path, "w"), indent=1, default=list)
        self.msg = f"exported {path}"
        return path

    # ---- drawing
    def draw(self):
        pg = self.pg
        self.screen.fill((24, 26, 30))
        z = self._pxpm()
        if self.film is not None and self.mmeta:
            fw = self.film.get_width()
            scale = z / self.mmeta["ppm"]
            c = self.mmeta["center"]
            tl = self.w2s((c[0] - fw / self.mmeta["ppm"] / 2, c[1] + fw / self.mmeta["ppm"] / 2))
            img = pg.transform.smoothscale(self.film, (max(1, int(fw * scale)),) * 2)
            self.screen.blit(img, tl)
        else:                                        # abstract grid, 5 m pitch
            step = 5.0
            x0, y0 = self.s2w((0, self.H)); x1, y1 = self.s2w((self.W, 0))
            for gx in np.arange(np.floor(x0 / step) * step, x1, step):
                a, b = self.w2s((gx, y0)), self.w2s((gx, y1))
                pg.draw.line(self.screen, (44, 46, 52), a, b, 1)
            for gy in np.arange(np.floor(y0 / step) * step, y1, step):
                a, b = self.w2s((x0, gy)), self.w2s((x1, gy))
                pg.draw.line(self.screen, (44, 46, 52), a, b, 1)
        # statics
        for i, st in enumerate(self.scn["statics"]):
            p = self.w2s(st["c"]); r = max(3, int(st["r"] * z))
            pg.draw.circle(self.screen, CLS_COLOR["static"], p, r, 0)
            if self.sel == ("static", i):
                pg.draw.circle(self.screen, (255, 255, 90), p, r + 3, 2)
        # movers: path + waypoints + live preview position
        for i, m in enumerate(self.scn["movers"]):
            col = CLS_COLOR.get(m["cls"], (200, 200, 200))
            pts = [self.w2s(p) for p in m["path"]]
            if len(pts) > 1:
                pg.draw.lines(self.screen, col, False, pts, 2)
            for j, p in enumerate(pts):
                sel = (self.sel == (i, j))
                pg.draw.circle(self.screen, (255, 255, 90) if sel else col, p, 6 if sel else 4,
                               0 if j == 0 else 1)
            sp = m["speed"]
            lab = f"{m['cls'][:3]}{i} t{self._num(m.get('spawn_t', 0)):.1f} " + \
                  (f"v{self._num(sp.get('v0', 0)):.0f}>{self._num(sp['v1']):.0f}@a{self._num(sp.get('a', 1)):.1f}"
                   if isinstance(sp, dict) else f"v{self._num(sp):.1f}")
            self.screen.blit(self.font.render(lab, True, col), (pts[0][0] + 8, pts[0][1] - 16))
        if self.tracks:
            for i, tr in enumerate(self.tracks):
                if tr["t"][0] <= self.t <= tr["t"][-1]:
                    x = np.interp(self.t, tr["t"], tr["xy"][:, 0])
                    y = np.interp(self.t, tr["t"], tr["xy"][:, 1])
                    col = CLS_COLOR.get(tr["cls"], (220, 220, 220))
                    rr = (self.scn["movers"][i].get("r") if i < len(self.scn["movers"]) else None) \
                        or SLB.CLS_R.get(tr["cls"], 0.4)
                    pg.draw.circle(self.screen, col, self.w2s((x, y)), max(3, int(rr * z)), 0)
        # drone start/goal + straight corridor
        s, g = self.scn["drone"]["start"], self.scn["drone"]["goal"]
        ps, pgl = self.w2s(s), self.w2s(g)
        pg.draw.line(self.screen, (90, 90, 110), ps, pgl, 1)
        pg.draw.circle(self.screen, (70, 230, 120), ps, 7)
        pg.draw.circle(self.screen, (240, 80, 80), pgl, 7)
        # replay overlay: drone path coloured by decision kind
        KC = {"straight": (120, 220, 120), "around_l": (80, 160, 255), "around_r": (80, 160, 255),
              "over": (250, 210, 80), "climb": (250, 140, 60), "evade": (255, 70, 70), "hold": (255, 70, 70)}
        for h in self.replay:
            if h.get("t", 1e9) <= self.t:
                pg.draw.circle(self.screen, KC.get(h.get("kind"), (200, 200, 200)),
                               self.w2s(h["p"]), 2)
        # HUD
        hud = [f"t={self.t:5.1f}s  {'PLAY' if self.playing else 'PAUSE'}   zoom={self.zoom:.1f}   {self.msg}"]
        if self.sel is not None:
            hud.append(f"sel={self.sel}")
        if self.help:
            hud += ["s/g start/goal  p/v/a/o new ped/veh/animal/static  L-click place/drag  TAB cycle  x del",
                    "[ ] speed  c const<->accel  {{ }} v1  9/0 accel  ,/. spawn_t  e hold_end",
                    "SPACE play  arrows scrub  r rewind  F2 export  wheel zoom  R-drag pan  q quit"]
        strip = pg.Surface((self.W, 10 + 16 * len(hud)))
        strip.set_alpha(185); strip.fill((18, 20, 24))
        self.screen.blit(strip, (0, 0))
        for k, line in enumerate(hud):
            self.screen.blit(self.font.render(line, True, (235, 235, 235)), (8, 6 + 16 * k))
        pg.display.flip()

    # ---- main loop
    def loop(self):
        pg = self.pg
        clock = pg.time.Clock()
        panning = False
        dragging = False
        while True:
            for ev in pg.event.get():
                if ev.type == pg.QUIT:
                    return
                if ev.type == pg.KEYDOWN:
                    k = ev.key
                    w = self.s2w(pg.mouse.get_pos())
                    if k == pg.K_q: return
                    elif k == pg.K_h: self.help = not self.help
                    elif k == pg.K_s: self.set_start(w)
                    elif k == pg.K_g: self.set_goal(w)
                    elif k == pg.K_p: self.add_mover("pedestrian", w)
                    elif k == pg.K_v: self.add_mover("vehicle", w)
                    elif k == pg.K_a: self.add_mover("animal", w)
                    elif k == pg.K_o: self.add_static(w)
                    elif k == pg.K_TAB:
                        n = len(self.scn["movers"])
                        if n:
                            i = (self.sel[0] + 1) % n if (self.sel and isinstance(self.sel[0], int)) else 0
                            self.sel = (i, len(self.scn["movers"][i]["path"]) - 1)
                    elif k in (pg.K_x, pg.K_DELETE): self.delete_sel()
                    elif ev.unicode == "{": self.nudge("v1", -0.5)   # MUST precede K_LEFTBRACKET: shift+[ ==
                    elif ev.unicode == "}": self.nudge("v1", +0.5)   # same keycode, unicode disambiguates
                    elif k == pg.K_LEFTBRACKET: self.nudge("speed", -0.2)
                    elif k == pg.K_RIGHTBRACKET: self.nudge("speed", +0.2)
                    elif k == pg.K_9: self.nudge("accel", -0.2)
                    elif k == pg.K_0: self.nudge("accel", +0.2)
                    elif ev.unicode == "{": self.nudge("v1", -0.5)
                    elif ev.unicode == "}": self.nudge("v1", +0.5)
                    elif k == pg.K_COMMA: self.nudge("spawn_t", -0.2)
                    elif k == pg.K_PERIOD: self.nudge("spawn_t", +0.2)
                    elif k == pg.K_c: self.toggle_profile()
                    elif k == pg.K_e: self.toggle_hold()
                    elif k == pg.K_SPACE: self.playing = not self.playing
                    elif k == pg.K_LEFT: self.t = max(0.0, self.t - 0.5)
                    elif k == pg.K_RIGHT: self.t += 0.5
                    elif k == pg.K_r: self.t = 0.0
                    elif k == pg.K_F2: self.export()
                if ev.type == pg.MOUSEBUTTONDOWN:
                    w = self.s2w(ev.pos)
                    if ev.button == 1:
                        hit = self.nearest(w)
                        if hit is not None:
                            self.sel = hit; dragging = True
                        elif not self.append_wp(w):
                            self.msg = "no mover selected: p/v/a/o to create one first"
                    elif ev.button == 3:
                        panning = True
                    elif ev.button == 4:
                        self.zoom = min(30.0, self.zoom * 1.15)
                    elif ev.button == 5:
                        self.zoom = max(0.3, self.zoom / 1.15)
                if ev.type == pg.MOUSEBUTTONUP:
                    if ev.button == 1: dragging = False
                    if ev.button == 3: panning = False
                if ev.type == pg.MOUSEMOTION:
                    if dragging and self.sel is not None:
                        self.move_sel(self.s2w(ev.pos))
                    elif panning:
                        z = self._pxpm()
                        self.view_c -= np.array([ev.rel[0] / z, -ev.rel[1] / z])
            if self.dirty:
                self.recompile()
            if self.playing:
                self.t += clock.get_time() / 1000.0
                if self.t > float(self.scn.get("t_max", 30.0)):
                    self.t = 0.0
            self.draw()
            clock.tick(60)


# ---------------------------------------------------------------- entry
def blank_scenario(seed):
    return dict(schema=SLB.SCHEMA, name="untitled", description="",
                map=dict(seed=seed),
                drone=dict(start=[0.0, 0.0, 1.5], goal=[16.0, 0.0, 1.5], cruise_z=1.5,
                           max_vel=3.0, max_acc=6.0),
                statics=[], movers=[], params={}, t_max=30.0)


def selftest():
    """Headless: exercise the edit ops end-to-end and verify the export round-trips through the compiler."""
    d = Designer(blank_scenario(None), headless=True)
    d.set_start((0, 0)); d.set_goal((18, 0))
    d.add_mover("vehicle", (8, -12)); d.append_wp((8, 12))
    d.toggle_profile()                                    # const -> {v0,v1,a}
    d.nudge("v1", +3.0); d.nudge("accel", +1.0)           # v1=4.2->7.2? (1.2+3.0); a=2.0
    d.add_mover("pedestrian", (12, 4)); d.append_wp((12, -4))
    d.add_static((5, 2))
    d.scn["name"] = "selftest_scene"
    d.recompile()
    assert d.tracks and len(d.tracks) == 3, f"expected 3 tracks, got {len(d.tracks)}"
    p = d.export(os.path.join(SCN_DIR, "selftest_scene.json"))
    scn = SLB.load(p)
    raw = SLB.to_movers_raw(scn)
    v = np.linalg.norm(np.diff(raw[0]["xy"], axis=0), axis=1) / SLB.DT_SAMPLE
    assert v.max() > 3.0, f"vehicle never accelerated (vmax={v.max():.2f})"
    ep = SLB.to_episode(scn)
    assert np.allclose(ep["goal"][:2], [18, 0]), ep["goal"]
    print(f"[selftest] OK: 3 tracks, vehicle vmax={v.max():.2f} m/s, export round-trip {p}")
    os.remove(p)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=None, help="MetaUrban scene (0-19); omit = abstract grid")
    ap.add_argument("--load", type=str, default=None, help="edit an existing scenario JSON")
    ap.add_argument("--replay", type=str, default=None, help="overlay a run_replay hist JSON")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        sys.exit(selftest())
    scn = json.load(open(args.load)) if args.load else blank_scenario(args.seed)
    seed = args.seed if args.seed is not None else (scn.get("map") or {}).get("seed")
    film_path = mmeta = None
    if seed is not None:
        film_path, mmeta = build_map_film(int(seed))
        scn.setdefault("map", {})["seed"] = int(seed)
    replay = None
    if args.replay:
        rp = json.load(open(args.replay))
        replay = rp.get("history", rp) if isinstance(rp, dict) else rp   # run_scenario wrapper or bare list
    Designer(scn, map_meta=mmeta, film_path=film_path, replay=replay).loop()


if __name__ == "__main__":
    main()
