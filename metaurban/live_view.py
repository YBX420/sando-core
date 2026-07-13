"""live_view — RViz-style REALTIME viewer for the replay-face stack (pygame).

The 'rviz for sando': runs the EXACT deployment decision stack (replay_core.run_replay —
the same code path behind every headline number) paced to the WALL CLOCK, and draws it live:

  semantic backdrop    cached MetaUrban top-down film (scenario_designer cache, if present)
  GT movers            every scenario mover at sim-time t (colour by class)
  keep-out cylinders   the conformal keep-outs fed to the certificate: bright = now,
                       dark ghost = end-of-trust-window position/size (drift + v_eff growth)
  committed traj       the B-spline the drone has COMMITTED to fly (green polyline)
  drone + trail        trail coloured by the per-tick decision kind (straight/around/brake/evade/…)
  HUD                  t / kind / clearance / speed / counts / measured realtime factor

By default the ACTIVE stack (V11 + escape tree, --stack v11esc) is applied via os.environ.setdefault —
any knob you export yourself still wins, and --stack none runs the frozen v1 default.

  ./ops/live.sh scenarios/bench/street_busy_s0.json            # one-liner (sets DISPLAY + env)
  ~/miniconda3/envs/metaurban/bin/python live_view.py scenarios/full/gauntlet.json
  python live_view.py scenarios/full/crossers.json --dummy --ticks 30 --shot 18:/tmp/s.png   # headless self-test

keys: SPACE pause | F follow drone | +/- sim speed | S screenshot | Q/ESC quit
"""
import argparse
import json
import os
import sys
import threading
import time
import traceback

import numpy as np

MU = os.path.dirname(os.path.abspath(__file__))
os.chdir(MU)
sys.path.insert(0, MU)

KINDC = {"straight": (90, 220, 120), "around_l": (80, 200, 230), "around_r": (80, 200, 230),
         "over": (120, 140, 255), "climb": (120, 140, 255), "brake": (255, 170, 60),
         "cret": (200, 200, 120), "evade": (240, 80, 80), "cpl": (220, 110, 240),
         "native": (160, 160, 170), "sando": (160, 160, 170)}
CLSC = {"pedestrian": (110, 175, 255), "ped": (110, 175, 255),
        "vehicle": (255, 165, 90), "veh": (255, 165, 90), "static": (130, 130, 140)}

# switch presets (applied with setdefault: your own env exports always win). CALIB_FILE is consumed
# verbatim by safety_layer.load_calib_v2 -> must be ABSOLUTE (R/out/conformal, not M/out/conformal).
_CALIB_V3 = os.path.join(os.path.dirname(MU), "out", "conformal", "calib_v3.json")
_V11 = dict(DECIDE="v2", CALIB_V2="1", CALIB_FILE=_CALIB_V3, DELTA_OVR="0.05",
            YOUNG_TTL="2", SPEEDS_CRAWL="1", TAU_SPEED="1", CPA_CLOUD="1", ESC_TRIG="1")
STACKS = {
    "v11esc": dict(_V11, V2_ESC="1"),                              # active V11 + escape tree
    "v11": _V11,
    "none": {},                                                    # frozen v1 default
}


class Live:
    """Shared state between the control-loop thread (cb) and the pygame draw loop."""

    def __init__(self, speed):
        self.lock = threading.Lock()
        self.latest = None
        self.trail = []                       # [(xy_world, colour)]
        self.done = None
        self.err = None
        self.stop = False
        self.run_evt = threading.Event(); self.run_evt.set()
        self.speed = speed
        self._lt = None; self._lw = None      # incremental pacing anchors (sim t, wall t)
        self.rtf = None                       # measured realtime factor (EMA)

    def cb(self, info):                       # runs INSIDE the control loop
        if self.stop:
            raise SystemExit
        with self.lock:
            self.latest = info
            self.trail.append((np.asarray(info["p"][:2], float) + info["org"],
                               KINDC.get(info["kind"], (200, 200, 200))))
        while not self.run_evt.is_set():      # paused: block the control loop itself
            if self.stop:
                raise SystemExit
            time.sleep(0.05)
            self._lt = self._lw = None        # don't fast-forward on resume
        now = time.perf_counter()
        if self._lt is not None and self.speed > 0:
            target = self._lw + (info["t"] - self._lt) / self.speed
            if target > now:
                time.sleep(target - now)
        now2 = time.perf_counter()
        if self._lt is not None and now2 - self._lw > 1e-9:
            f = (info["t"] - self._lt) / (now2 - self._lw)   # sleep INCLUDED: true wall pace
            self.rtf = f if self.rtf is None else 0.9 * self.rtf + 0.1 * f
        self._lt = info["t"]; self._lw = now2


def _build_film(seed, block, png, meta):
    """Build the top-down film in a SUBPROCESS with the SAME env constructor the scenario generator
    used (scenario_designer3d.build_env) -> the film is guaranteed to be the same world. Subprocess
    keeps the heavy metaurban import + its SDL-dummy pygame out of the viewer process."""
    import subprocess
    code = f"""
import os; os.environ['SDL_VIDEODRIVER'] = 'dummy'
import sys, json; sys.path.insert(0, {MU!r})
import cv2
from scenario_designer3d import build_env
env = build_env({seed!r}, interactive=False, block_str={block!r})
c = [float(env.agent.position[0]), float(env.agent.position[1])]
img = env.render(mode='topdown', window=False, screen_size=(4000, 4000), film_size=(4000, 4000),
                 scaling=8.0, camera_position=tuple(c))
env.close()
cv2.imwrite({png!r}, img[..., ::-1])
json.dump(dict(seed={seed!r}, block={block!r}, center=c, ppm=8.0, film=4000), open({meta!r}, 'w'))
"""
    subprocess.run([sys.executable, "-c", code], check=True, cwd=MU)


def load_film(seed, block, build=False):
    """Map film keyed by (seed, block_str) — a film from a DIFFERENT block is a different world,
    so we never overlay a mismatched one. Legacy designer cache name (seedN) covers block 'X'."""
    block = block or "X"
    tag = f"seed{seed}" if block == "X" else f"seed{seed}_{block}"
    png = os.path.join(MU, "out", "designer_cache", f"{tag}.png")
    meta = os.path.join(MU, "out", "designer_cache", f"{tag}.json")
    if not (os.path.exists(png) and os.path.exists(meta)):
        if not build:
            print(f"[live] no cached film for {tag} (run once with --build_film) -> grid backdrop")
            return None, None
        print(f"[live] building map film {tag} (~40 s once, subprocess) ...", flush=True)
        os.makedirs(os.path.dirname(png), exist_ok=True)
        try:
            _build_film(seed, block, png, meta)
        except Exception as e:
            print(f"[live] film build failed ({e}) -> grid backdrop")
            return None, None
    return png, json.load(open(meta))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario")
    ap.add_argument("--mode", default="ours", choices=["ours", "native"])
    ap.add_argument("--speed", type=float, default=1.0, help="sim speed vs wall clock (0 = flat out)")
    ap.add_argument("--w", type=int, default=1100)
    ap.add_argument("--h", type=int, default=800)
    ap.add_argument("--follow", action="store_true", help="camera follows the drone")
    ap.add_argument("--build_film", action="store_true",
                    help="build the map film if uncached (~40 s; needs the metaurban package importable)")
    ap.add_argument("--ticks", type=int, default=0, help="auto-quit after N ticks (self-test)")
    ap.add_argument("--shot", default=None, help="TICK:PATH — save a screenshot once tick >= TICK")
    ap.add_argument("--dummy", action="store_true", help="SDL dummy video driver (headless self-test)")
    ap.add_argument("--stack", default="v11esc", choices=sorted(STACKS),
                    help="switch preset applied via setdefault (default: active V11 + escape tree)")
    args = ap.parse_args()
    if args.dummy:
        os.environ["SDL_VIDEODRIVER"] = "dummy"
    for k, v in STACKS[args.stack].items():
        os.environ.setdefault(k, v)                    # BEFORE importing replay_core (knobs bind at import)
    print(f"[live] stack={args.stack} " +
          " ".join(f"{k}={os.environ[k]}" for k in STACKS["v11esc"] if k in os.environ), flush=True)

    import pygame as pg
    import replay_core as RC
    import scenario_lib as SLB

    scn = SLB.load(args.scenario)
    movers = SLB.apply_rh_overrides(RC.Movers(SLB.to_movers_raw(scn)), scn)
    ep = SLB.to_episode(scn)
    start = np.asarray(ep["start"][:2], float); goal = np.asarray(ep["goal"][:2], float)
    org = 0.5 * (start + goal)
    tau_ghost = float(getattr(RC, "TAU", 0.75))

    _map = scn.get("map") or {}
    film_png, film_meta = (load_film(_map.get("seed"), _map.get("block_str"), build=args.build_film)
                           if _map.get("seed") is not None else (None, None))

    live = Live(args.speed)

    def worker():
        try:
            live.done = RC.run_replay(movers, ep, mode=args.mode, tick_cb=live.cb,
                                      max_vel=float(scn["drone"].get("max_vel", 3.0)))
        except SystemExit:
            pass
        except Exception:
            live.err = traceback.format_exc()

    pg.init()
    screen = pg.display.set_mode((args.w, args.h))
    pg.display.set_caption(f"sando live — {scn.get('name', args.scenario)} [{args.mode}]")
    font = pg.font.SysFont("monospace", 14)
    film = None
    if film_png:
        film = pg.image.load(film_png).convert()
        dark = pg.Surface(film.get_size()); dark.fill((0, 0, 0)); dark.set_alpha(120)
        film.blit(dark, (0, 0))               # dim the backdrop so markers pop (RViz-dark look)

    span = float(np.linalg.norm(goal - start)) + 30.0
    ppm0 = min(args.w, args.h) / span
    shot = None
    if args.shot:
        st, sp = args.shot.split(":", 1)
        shot = [int(st), sp, False]

    th = threading.Thread(target=worker, daemon=True)
    th.start()
    clock = pg.time.Clock()
    follow = args.follow
    scaled_film = None                        # (surface, topleft_world) cache, rebuilt on zoom change

    def w2s(w, c, ppm):
        return (int((w[0] - c[0]) * ppm + args.w / 2), int(-(w[1] - c[1]) * ppm + args.h / 2))

    running = True
    while running:
        for e in pg.event.get():
            if e.type == pg.QUIT:
                running = False
            elif e.type == pg.KEYDOWN:
                if e.key in (pg.K_q, pg.K_ESCAPE):
                    running = False
                elif e.key == pg.K_SPACE:
                    (live.run_evt.clear if live.run_evt.is_set() else live.run_evt.set)()
                elif e.key == pg.K_f:
                    follow = not follow
                elif e.key in (pg.K_PLUS, pg.K_EQUALS):
                    live.speed = min(8.0, (live.speed or 0.25) * 2)
                elif e.key == pg.K_MINUS:
                    live.speed = max(0.125, live.speed / 2)
                elif e.key == pg.K_s:
                    pg.image.save(screen, os.path.join(MU, "out", f"live_shot_{int(time.time())}.png"))

        with live.lock:
            info = live.latest
            trail = list(live.trail)
        t_now = info["t"] if info else ep["t0"]
        p_dw = (np.asarray(info["p"][:2], float) + info["org"]) if info else start
        cen = p_dw if follow else org
        ppm = ppm0

        screen.fill((16, 17, 20))
        if film is not None and film_meta:
            scale = ppm / film_meta["ppm"]
            key = (round(scale, 4),)
            if scaled_film is None or scaled_film[0] != key:
                fw = film.get_width()
                img = pg.transform.smoothscale(film, (max(1, int(fw * scale)),) * 2)
                scaled_film = (key, img)
            fc = film_meta["center"]; half = film.get_width() / film_meta["ppm"] / 2
            screen.blit(scaled_film[1], w2s((fc[0] - half, fc[1] + half), cen, ppm))
        else:                                  # 5 m grid
            x0 = cen[0] - args.w / 2 / ppm; x1 = cen[0] + args.w / 2 / ppm
            y0 = cen[1] - args.h / 2 / ppm; y1 = cen[1] + args.h / 2 / ppm
            for gx in np.arange(np.floor(x0 / 5) * 5, x1, 5.0):
                pg.draw.line(screen, (35, 37, 43), w2s((gx, y0), cen, ppm), w2s((gx, y1), cen, ppm), 1)
            for gy in np.arange(np.floor(y0 / 5) * 5, y1, 5.0):
                pg.draw.line(screen, (35, 37, 43), w2s((x0, gy), cen, ppm), w2s((x1, gy), cen, ppm), 1)

        # start / goal
        pg.draw.circle(screen, (90, 220, 120), w2s(goal, cen, ppm), max(3, int(0.8 * ppm)), 2)
        pg.draw.circle(screen, (120, 120, 130), w2s(start, cen, ppm), 4, 1)

        # GT movers at sim time
        for i in range(len(movers.m)):
            if not movers.present(i, t_now):
                continue
            m = movers.m[i]
            col = CLSC.get(str(m.get("cls", "ped")), (150, 150, 160))
            s = w2s(movers.pos(i, t_now), cen, ppm)
            pg.draw.circle(screen, col, s, max(2, int(float(m.get("r", 0.3)) * ppm)), 2)
            pg.draw.circle(screen, col, s, 2)

        if info:
            # keep-out cylinders: now (bright red) + trust-window ghost (dark red)
            for (c0, vv, R, veff) in info["cyl"]:
                cw = c0 + info["org"]
                gw = cw + vv * tau_ghost
                pg.draw.circle(screen, (120, 40, 40), w2s(gw, cen, ppm),
                               max(2, int((R + veff * tau_ghost) * ppm)), 1)
                pg.draw.circle(screen, (235, 80, 80), w2s(cw, cen, ppm), max(2, int(R * ppm)), 2)
                pg.draw.line(screen, (235, 80, 80), w2s(cw, cen, ppm), w2s(cw + vv, cen, ppm), 1)
            # committed trajectory
            if info["traj"]:
                pts = [w2s(np.asarray(q[:2], float) + info["org"], cen, ppm) for q in info["traj"]]
                if len(pts) > 1:
                    pg.draw.lines(screen, (80, 230, 130), False, pts, 2)
            # trail + drone
            for (xy, col) in trail[-800:]:
                pg.draw.circle(screen, col, w2s(xy, cen, ppm), 2)
            pg.draw.circle(screen, (245, 245, 245), w2s(p_dw, cen, ppm), max(3, int(0.25 * ppm)))
            v = np.asarray(info["v"][:2], float)
            pg.draw.line(screen, (250, 220, 90), w2s(p_dw, cen, ppm), w2s(p_dw + v * 0.8, cen, ppm), 2)

        # HUD
        hud = []
        if info:
            kc = KINDC.get(info["kind"], (220, 220, 220))
            clr = f"{info['clr']:.2f}m" if info["clr"] is not None else "--"
            hud.append((f"t={info['t']:.1f}s  tick={info['tick']}  kind={info['kind']:<9}"
                        f" clr={clr}  |v|={np.linalg.norm(info['v']):.2f}m/s", kc))
            cnt = {k: v for k, v in info["counts"].items() if v}
            hud.append((f"counts {cnt}", (170, 170, 180)))
        rtf = f"{live.rtf:.2f}x" if live.rtf else "--"
        hud.append((f"speed={live.speed:g}x  rt={rtf}  "
                    f"{'PAUSED' if not live.run_evt.is_set() else ''}", (150, 200, 250)))
        if live.done:
            d = live.done
            hud.append((f"DONE reached={d.get('reached')} collided={d.get('collided')}"
                        f" min_clr={d.get('min_clr', 0):.2f}", (255, 210, 90)))
        if live.err:
            hud.append(("STACK ERROR — see terminal", (255, 80, 80)))
            if not getattr(live, "err_printed", False):
                print(live.err, file=sys.stderr)
                live.err_printed = True             # keep live.err set: the --ticks exit check needs it
        if hud:
            back = pg.Surface((max(font.size(t)[0] for t, _ in hud) + 16, 17 * len(hud) + 10))
            back.fill((12, 13, 16)); back.set_alpha(200)
            screen.blit(back, (4, 4))
        for j, (txt, col) in enumerate(hud):
            screen.blit(font.render(txt, True, col), (10, 8 + 17 * j))

        pg.display.flip()

        if shot and info and info["tick"] >= shot[0] and not shot[2]:
            pg.image.save(screen, shot[1]); shot[2] = True
            print(f"[live] screenshot -> {shot[1]}", flush=True)
        if args.ticks and ((info and info["tick"] >= args.ticks) or live.done or live.err):
            running = False
        clock.tick(30)

    live.stop = True
    th.join(timeout=2.0)
    pg.quit()
    if live.done:
        print(f"[live] result: {json.dumps({k: v for k, v in live.done.items() if k != 'hist'}, default=str)[:400]}")


if __name__ == "__main__":
    main()
