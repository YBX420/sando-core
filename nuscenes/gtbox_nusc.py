"""gtbox_nusc -- the PERFECT-DETECTOR control arm: GT annotations rendered into 2-D bboxes, then the
EXACT same downstream as the YOLO arm ("依旧用 bbx", 塔菲大人 2026-07-14).

What is identical to bytetrack_nusc: bbox -> bottom-centre ground back-projection, r_obs from the
box, innovation gate, face q_jerk, MotionLatch, v_feed, keyframe-only exam sheet, capsule overlay.
What changes: detections = GT 3-D boxes (interpolated to every 12 Hz sweep, projected to CAM_FRONT,
8 corners -> tight 2-D bbox, same border/size/range filters) and association = instance identity.
So: YOLO-arm error minus this arm's error = the DETECTOR's tax; this arm's error alone = the tax of
the bbox-bottom monocular geometry chain itself.

Caveats: GT boxes exist through occlusion (an x-ray detector) and have zero pixel jitter; rotation
is taken from the nearest keyframe.

Run:  ~/miniconda3/envs/metaurban/bin/python sando-core/nuscenes/gtbox_nusc.py [--scenes ...]
"""
import argparse
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "metaurban"))
sys.path.insert(0, _HERE)
from kf_tracker import MoverTracker                                  # noqa: E402
from kf_nusc import Nusc, DATA, quat_rot, world_to_px, HORIZONS      # noqa: E402
from yolo_nusc import RANGE_MAX, SIG_RANGE, SIGV_YOUNG, px_to_ground  # noqa: E402
from bytetrack_nusc import (MotionLatch, capsule_ring, R_CLAMP, R_EMA, VMAX,  # noqa: E402
                            MISS_SEC, Q_JERK_FACE, VFEED_EMA)

ATTR_MOTION = {"vehicle.moving": "MOVING", "vehicle.stopped": "STATIC", "vehicle.parked": "STATIC",
               "pedestrian.moving": "MOVING", "pedestrian.standing": "STATIC",
               "pedestrian.sitting_lying_down": "STATIC",
               "cycle.with_rider": "MOVING", "cycle.without_rider": "STATIC"}


def gt_bbox(nusc, sd_rec, c_xyz, size, rot_q):
    """GT 3-D box -> tight 2-D CAM_FRONT bbox (None if behind / degenerate). nuScenes size=[w,l,h]."""
    w, l, h = size
    R = quat_rot(rot_q)
    corners = np.array([[sx * l / 2, sy * w / 2, sz * h / 2]
                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    pts_w = np.asarray(c_xyz)[None, :] + corners @ R.T
    px, ok = world_to_px(pts_w, sd_rec, nusc)
    if ok.sum() < 4:
        return None
    px = px[ok]
    return float(px[:, 0].min()), float(px[:, 1].min()), float(px[:, 0].max()), float(px[:, 1].max())


def main(scene_names, render):
    import cv2
    nusc = Nusc()
    out_dir = os.path.join(_HERE, "out")
    os.makedirs(out_dir, exist_ok=True)
    sample_scene = {}
    for sc in nusc.scene:
        for s in nusc.sample_chain(sc):
            sample_scene[s["token"]] = sc["name"]
    frames = {}
    for d in nusc.sd:
        if "CAM_FRONT/" not in d["filename"]:
            continue
        sname = sample_scene.get(d["sample_token"])
        if sname:
            frames.setdefault(sname, []).append(d)
    for v in frames.values():
        v.sort(key=lambda d: d["timestamp"])

    rows = []
    col = {"pedestrian": (60, 140, 255), "vehicle": (80, 220, 80), "cycle": (255, 200, 0)}
    for scene in nusc.scene:
        if scene_names and scene["name"] not in scene_names:
            continue
        chain = nusc.sample_chain(scene)
        ts_key = {s["token"]: s["timestamp"] / 1e6 for s in chain}
        gt = {}
        for s in chain:
            for a in nusc.anns_by_sample.get(s["token"], []):
                if a["grp"]:
                    gt.setdefault(a["instance_token"], []).append((ts_key[s["token"]], a))
        gt = {k: sorted(v, key=lambda x: x[0]) for k, v in gt.items()}

        def gt_state_at(inst, t):
            """(xyz interp, size, rot, attr) at time t, None outside the chain."""
            obs = gt[inst]
            if t < obs[0][0] - 1e-9 or t > obs[-1][0] + 1e-9:
                return None
            near = min(obs, key=lambda o: abs(o[0] - t))[1]
            # x-ray filter: annotations exist through walls (visibility 1 = 0-40%). A detector can
            # never see those; scoring them charges the GEOMETRY chain for the OCCLUSION problem.
            if int(near.get("visibility_token", 4)) < 2:
                return None
            for (t0, a0), (t1, a1) in zip(obs, obs[1:]):
                if t0 - 1e-9 <= t <= t1 + 1e-9:
                    wgt = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                    p = (1 - wgt) * np.asarray(a0["translation"]) + wgt * np.asarray(a1["translation"])
                    attr = nusc.attr[near["attribute_tokens"][0]] if near["attribute_tokens"] else "-"
                    return p, near["size"], near["rotation"], near["grp"], attr
            a = obs[-1][1]
            attr = nusc.attr[a["attribute_tokens"][0]] if a["attribute_tokens"] else "-"
            return np.asarray(a["translation"]), a["size"], a["rotation"], a["grp"], attr

        world, vw = {}, None
        for sd_rec in frames[scene["name"]]:
            t_now = sd_rec["timestamp"] / 1e6
            W, H = sd_rec.get("width", 1600), sd_rec.get("height", 900)
            ego_xy = np.asarray(nusc.ego[sd_rec["ego_pose_token"]]["translation"][:2])
            K_f = float(np.asarray(nusc.cs[sd_rec["calibrated_sensor_token"]]["camera_intrinsic"])[0][0])
            seen = set()
            for inst in gt:
                st = gt_state_at(inst, t_now)
                if st is None:
                    continue
                c_xyz, size, rot, grp, attr = st
                bb = gt_bbox(nusc, sd_rec, c_xyz, size, rot)
                if bb is None:
                    continue
                x0, y0, x1, y1 = bb
                x0 = max(x0, 0.0); y0 = max(y0, 0.0); x1 = min(x1, W - 1.0); y1 = min(y1, H - 1.0)
                if x1 - x0 < 2 or y1 - y0 < 2:
                    continue
                if x0 < 4 or x1 > W - 4 or y1 > H - 4 or (y1 - y0) < 18:   # same filters as YOLO arm
                    continue
                g = px_to_ground(((x0 + x1) / 2.0, y1), sd_rec, nusc)
                if g is None:
                    continue
                d_ego = float(np.linalg.norm(g[:2] - ego_xy))
                if d_ego > RANGE_MAX:
                    continue
                lo_r, hi_r = R_CLAMP[grp]
                ext_px = 0.62 * (y1 - y0) if grp == "vehicle" else 0.5 * (x1 - x0)
                r_det = float(np.clip(ext_px * d_ego / K_f, lo_r, hi_r))
                w = world.get(inst)
                if w is None:
                    w = world[inst] = {"trk": MoverTracker(dt=0.083, meas_noise=0.5),
                                       "t_last": None, "grp": grp, "hist": [], "bbox": None,
                                       "latch": MotionLatch(grp), "r_obs": r_det,
                                       "v_feed": np.zeros(2), "attr": attr}
                    for _ax in (w["trk"].fx, w["trk"].fy):
                        _ax.q_jerk = Q_JERK_FACE
                        _ax.F, _ax.Q = _ax._mats(_ax.dt)
                trk = w["trk"]
                sig = SIG_RANGE(d_ego)
                trk.fx.R = trk.fy.R = sig * sig
                det = np.array([g[0], g[1], 0.0])
                if w["t_last"] is not None and w["hist"]:
                    dtj = max(1e-3, t_now - w["t_last"])
                    jump = float(np.linalg.norm(det[:2] - w["hist"][-1][:2]))
                    if jump > VMAX[grp] * dtj + 3.0 * sig:
                        w["bbox"] = (x0, y0, x1, y1); seen.add(inst)
                        continue
                if w["t_last"] is None:
                    trk.update(det)
                else:
                    trk.update(det, dt=max(1e-3, t_now - w["t_last"]))
                w["t_last"] = t_now; w["hist"].append(det); w["bbox"] = (x0, y0, x1, y1)
                w["r_obs"] = (1 - R_EMA) * w["r_obs"] + R_EMA * r_det
                w["latch"].add(t_now, det[:2], sig)
                v_raw = w["latch"].vel_win if w["latch"].state == "MOVING" else np.zeros(2)
                w["v_feed"] = (1 - VFEED_EMA) * w["v_feed"] + VFEED_EMA * np.asarray(v_raw[:2], float)
                w["attr"] = attr
                seen.add(inst)
            for inst, w in list(world.items()):
                if inst not in seen and w["t_last"] is not None and t_now - w["t_last"] > MISS_SEC:
                    del world[inst]

            if bool(sd_rec["is_key_frame"]):
                for inst, w in world.items():
                    if inst not in seen or not w["trk"].ready:
                        continue
                    obs_xy = w["hist"][-1][:2]
                    st_l = w["latch"].state

                    def gxy_at(t):
                        r = gt_state_at(inst, t)
                        return None if r is None else r[0][:2]
                    for h in HORIZONS:
                        gxy = gxy_at(t_now + h)
                        if gxy is None:
                            continue
                        pxy = obs_xy + w["v_feed"] * h if st_l == "MOVING" else obs_xy
                        rows.append((scene["name"], w["grp"], h,
                                     float(np.linalg.norm(pxy - gxy)),
                                     float(np.linalg.norm(obs_xy - gxy)),
                                     st_l, ATTR_MOTION.get(w["attr"], "-"),
                                     float(np.linalg.norm(obs_xy - gxy_at(t_now)))))

            if render:
                img = cv2.imread(os.path.join(DATA, sd_rec["filename"]))
                for inst, w in world.items():
                    if inst not in seen or w["bbox"] is None:
                        continue
                    trk = w["trk"]
                    frz = (not trk.ready) or trk.sigma_v > SIGV_YOUNG[w["grp"]]
                    c = (160, 160, 160) if frz else col[w["grp"]]
                    x0, y0, x1, y1 = [int(v) for v in w["bbox"]]
                    cv2.rectangle(img, (x0, y0), (x1, y1), col[w["grp"]], 2)
                    if not trk.ready:
                        continue
                    c0, v, _ = trk.state()
                    ring_c, rad_c = capsule_ring(c0, w["v_feed"], w["r_obs"], vmax=VMAX[w["grp"]])
                    ring3 = np.array([[px_, py_, 0.0] for px_, py_ in ring_c])
                    px_cap, okcap = world_to_px(ring3, sd_rec, nusc)
                    pc = px_cap[okcap].astype(int)
                    for p0, p1 in zip(pc, pc[1:]):
                        cv2.line(img, tuple(p0), tuple(p1), (255, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(img, f"{scene['name']} GT-BBOX (perfect detector) -> same chain", (16, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
                if vw is None:
                    vw = cv2.VideoWriter(os.path.join(out_dir, f"gtbox_{scene['name']}.mp4"),
                                         cv2.VideoWriter_fourcc(*"mp4v"), 12,
                                         (img.shape[1], img.shape[0]))
                vw.write(img)
        if vw is not None:
            vw.release()
            print("wrote", os.path.join(out_dir, f"gtbox_{scene['name']}.mp4"))

    if not rows:
        print("no scored rows"); return
    rows = np.core.records.fromrecords(rows, names="scene,grp,h,err,still,latch,gtmot,obs_err")
    print(f"\nobs error (backprojected GT-bbox vs GT centre, h=now): "
          f"mean {np.mean(rows.obs_err):.2f} median {np.median(rows.obs_err):.2f}")
    hdr = f"{'bucket (mean/median m)':30s}"
    for h in HORIZONS:
        hdr += f"   h={h:.1f}s kf     |still"
    print(hdr)
    for grp in ("pedestrian", "vehicle", "cycle"):
        sub = rows[rows.grp == grp]
        if not len(sub):
            continue
        line = f"{grp:30s}"
        for h in HORIZONS:
            e = sub[sub.h == h]
            line += f" {np.mean(e.err):5.2f}/{np.median(e.err):5.2f} |{np.mean(e.still):5.2f}" if len(e) else "    -"
        print(line)
    print("\n--- motion-latch confusion (h=2.0s rows) ---")
    e = rows[rows.h == 2.0]
    for lt in ("STATIC", "MOVING", "UNKNOWN"):
        for gm in ("STATIC", "MOVING"):
            n = int(np.sum((e.latch == lt) & (e.gtmot == gm)))
            if n:
                print(f"latch={lt:8s} gt={gm:7s} n={n}")
    print("\n--- error by latch state (h=2.0s, vs still) ---")
    for lt in ("STATIC", "MOVING", "UNKNOWN"):
        sub = e[e.latch == lt]
        if len(sub):
            print(f"{lt:8s} n={len(sub):4d}  policy {np.mean(sub.err):5.2f}/{np.median(sub.err):5.2f}"
                  f"  still {np.mean(sub.still):5.2f}/{np.median(sub.still):5.2f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", type=str, default="scene-0103,scene-1094")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--no_render", action="store_true")
    args = ap.parse_args()
    main(None if args.all else set(args.scenes.split(",")), not args.no_render)
