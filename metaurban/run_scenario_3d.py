"""run_scenario_3d — fly a designed scenario with the REAL safety layer and watch it in native 3D,
live (realtime window) and/or as an mp4. Closes the design loop: scenario_designer(3d) -> here.

  # realtime window on $DISPLAY (ESC/q to quit early), plus mp4:
  env DISPLAY=:1 PYTHONPATH=/media/boxuan/Data2/projects/metaurban \
      LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6 \
    ~/miniconda3/envs/metaurban/bin/python run_scenario_3d.py scenarios/crowd_dense.json \
      --seed 3 --live --mp4
  # headless mp4 only (CLI workflow):    ... run_scenario_3d.py scenarios/x.json --seed 3 --mp4
  # overlay a PREVIOUS run's path:       ... --overlay out/scenario_runs/x_ours_hist.json

The control loop IS replay_core.run_replay (single source of truth — the same ticks, KF, conformal
keep-out, cert tournament as the headless evaluation); this script only *renders* each tick via the
tick_cb hook: scripted movers animate along their compiled tracks, the drone model flies the actual
decided set-points, and the flown trail is colored by decision kind (green straight / blue around /
yellow over / orange climb / red evade). LD_PRELOAD of sando's libstdc++ is REQUIRED when mixing
ego_capi.so with MetaUrban's python (same as render_3d_video; missing it segfaults ego.replan).

--live paces the loop to wall clock (1 sim tick = 0.1 s); rendering at ~10 fps is comfortably
inside the budget, the cert itself is ~us-scale. --speed 2.0 runs 2x faster than real time.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

import scenario_lib as SLB
from run_scenario import run_one, OUTD
from scenario_designer3d import Scene3D, build_env, KIND_COLOR

HERE = os.path.dirname(os.path.abspath(__file__))
VIDD = os.path.join(HERE, "out", "scenario_videos")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario")
    ap.add_argument("--seed", type=int, default=None, help="MetaUrban map scene (default: scenario map.seed)")
    ap.add_argument("--mode", default="ours", choices=["ours", "native", "sando"])
    ap.add_argument("--dynamics", action="store_true", help="fly through real quadrotor dynamics")
    ap.add_argument("--live", action="store_true", help="realtime cv2 window on $DISPLAY (ESC/q quits)")
    ap.add_argument("--mp4", action="store_true", help="write out/scenario_videos/<name>_<mode>.mp4")
    ap.add_argument("--speed", type=float, default=1.0, help="realtime factor for --live (2.0 = 2x)")
    ap.add_argument("--view", default="iso", choices=["iso", "chase"])
    ap.add_argument("--w", type=int, default=1280)
    ap.add_argument("--h", type=int, default=800)
    ap.add_argument("--overlay", type=str, default=None, help="draw a previous run's hist path (static)")
    args = ap.parse_args()
    if not (args.live or args.mp4):
        args.mp4 = True                                        # doing neither would fly invisibly

    import cv2
    scn = SLB.load(args.scenario)
    win = f"metadrone - {scn['name']} [{args.mode}]"
    if args.live:                                          # splash IMMEDIATELY: world build takes ~40 s,
        splash = np.zeros((240, 760, 3), np.uint8)         # a silent black period reads as "nothing rendered"
        splash[:] = (30, 42, 30)
        cv2.putText(splash, "metadrone: building world... (~40s first time)", (28, 105),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (240, 240, 240), 2)
        cv2.putText(splash, "window will switch to the live flight automatically", (28, 155),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (170, 200, 170), 1)
        cv2.imshow(win, splash); cv2.waitKey(1)
    seed = args.seed if args.seed is not None else (scn.get("map") or {}).get("seed")
    env = build_env(seed, interactive=False, block_str=(scn.get("map") or {}).get("block_str", "X"))
    eng = env.engine
    sc = Scene3D(eng, scn)

    d = scn["drone"]
    org = 0.5 * (np.asarray(d["start"][:2], float) + np.asarray(d["goal"][:2], float))
    ctr = np.array([org[0], org[1], 1.0])
    L = max(8.0, float(np.linalg.norm(np.asarray(d["goal"][:2]) - np.asarray(d["start"][:2]))))
    gdir = (np.asarray(d["goal"][:2], float) - np.asarray(d["start"][:2], float)) / L

    from panda3d.core import Vec3
    cam = eng.get_sensor("rgb_camera")
    cam.cam.reparentTo(eng.render)

    trail = eng.make_line_drawer(thickness=5.0)
    trail_pts, trail_cols = [], []

    # static overlay of a previous run (e.g. compare native vs ours paths)
    if args.overlay:
        rp = json.load(open(args.overlay))
        hist = rp.get("history", rp)
        ov = eng.make_line_drawer(thickness=3.0)
        pts = [(h["p"][0], h["p"][1], h.get("z", 1.5)) for h in hist]
        if len(pts) > 1:
            ov.draw_lines([pts], [[(0.75, 0.75, 0.75, 1)] * (len(pts) - 1)])

    from replay_core import DT                             # the ACTUAL sim tick (0.3 s) -- pacing and mp4
    writer = [None]                                        # fps must derive from it or playback runs 3x fast
    mp4_path = os.path.join(VIDD, f"{scn['name']}_{args.mode}.mp4")

    def _writer_for(frame):                                # lazy: size from the real frame (camera is fixed
        os.makedirs(VIDD, exist_ok=True)                   # 1280x800 regardless of --w/--h; a mismatched
        h, w = frame.shape[:2]                             # VideoWriter silently drops every frame)
        return cv2.VideoWriter(mp4_path, cv2.VideoWriter_fourcc(*"mp4v"),
                               max(1.0, args.speed / DT), (w, h))
    aborted = []
    last_frame = [None]
    t_wall0 = time.time()

    def tick_cb(info):
        t = info["t"]
        sc.set_time(t)                                         # scripted movers at sim time t
        pw = np.array([info["p"][0] + org[0], info["p"][1] + org[1], info["p"][2]])
        if sc.drone_np is not None:
            sc.drone_np.setPos(*map(float, pw))
        col = KIND_COLOR.get(info["kind"], (0.9, 0.9, 0.9, 1))
        trail_pts.append(tuple(map(float, pw)))
        if len(trail_pts) > 1:
            trail_cols.append(col)
            trail.reset()
            trail.draw_lines([trail_pts], [trail_cols])
        if args.view == "chase":
            back = pw[:2] - gdir * 6.0
            cam.cam.setPos(Vec3(float(back[0]), float(back[1]), float(pw[2] + 3.0)))
            cam.cam.lookAt(Vec3(*map(float, pw)))
        else:
            cam.cam.setPos(Vec3(float(ctr[0] - 0.7 * L), float(ctr[1] - 0.7 * L), float(0.6 * L)))
            cam.cam.lookAt(Vec3(*map(float, ctr)))
        eng.graphicsEngine.renderFrame()
        eng.graphicsEngine.renderFrame()
        a = np.array(cam.get_rgb_array_cpu())                  # BGR already; copy -> writable for putText
        if a.dtype != np.uint8:
            a = (a * 255).astype(np.uint8) if a.max() <= 1.01 else a.astype(np.uint8)
        a = np.ascontiguousarray(a)
        hud = f"t={t:5.1f}s  kind={info['kind']:<9s}  clr={info['clr'] if info['clr'] is not None else '-'}"
        cv2.putText(a, hud, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        last_frame[0] = a
        if args.mp4:
            if writer[0] is None:
                writer[0] = _writer_for(a)
            writer[0].write(a)
        if args.live:
            cv2.imshow(win, a)
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord("q")):
                aborted.append(True)
                raise KeyboardInterrupt                        # unwind out of run_replay cleanly
            # pace to wall clock: sim tick DT vs elapsed
            lag = (info["tick"] + 1) * DT / args.speed - (time.time() - t_wall0)
            if lag > 0:
                time.sleep(lag)

    try:
        res, hist = run_one(scn, mode=args.mode, dynamics=args.dynamics, record=True, tick_cb=tick_cb)
    except KeyboardInterrupt:
        res, hist = {"aborted": True}, []
        print("[3d] aborted by user")
    if writer[0] is not None:
        writer[0].release()
        print(f"[3d] wrote {mp4_path}")
    if args.live:
        if last_frame[0] is not None and not aborted:      # hold the final frame instead of vanishing
            a = last_frame[0].copy()
            cv2.putText(a, "DONE - press any key to close", (10, a.shape[0] - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80, 255, 120), 2)
            cv2.imshow(win, a)
            cv2.waitKey(15000)
        cv2.destroyAllWindows()
    if not aborted and "reached" in res:
        os.makedirs(OUTD, exist_ok=True)
        out = os.path.join(OUTD, f"{scn['name']}_{args.mode}_hist.json")
        json.dump(dict(scenario=scn["name"], mode=args.mode, result=res, history=hist),
                  open(out, "w"), indent=None, default=float)
        print(f"[{scn['name']}] reached={res['reached']} t={res['time_s']}s min_clr={res['min_clr']:.3f} "
              f"collided={res['collided']} counts={res['counts']}")
    env.close()


if __name__ == "__main__":
    main()
