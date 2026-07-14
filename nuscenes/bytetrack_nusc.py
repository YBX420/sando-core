"""bytetrack_nusc -- YOLO + ByteTrack (FoundationVision, via the ultralytics port) + the production KF.

Upgrade over yolo_nusc (naive world-xy greedy NN):
  - association happens in IMAGE space by ByteTrack (IoU + pixel-KF + the BYTE low-confidence second
    pass), which is what it's good at -- our world-frame KF only ever sees a CLEAN single-ID chain.
  - runs on the 12 Hz CAM_FRONT sweeps, not the 2 Hz keyframes: ByteTrack needs inter-frame overlap,
    and the 6x cadence also attacks the "0.5 s monocular velocity is unmeasurable" wall head-on.
Scoring stays ON KEYFRAMES ONLY with the same horizons/still baseline as kf_nusc/yolo_nusc -> the
three arms share one exam sheet (GT-fed / naive-NN@2Hz / ByteTrack@12Hz).

Run:  ~/miniconda3/envs/metaurban/bin/python sando-core/nuscenes/bytetrack_nusc.py [--scenes ...]
"""
import argparse
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "metaurban"))
sys.path.insert(0, _HERE)
from kf_tracker import MoverTracker                     # noqa: E402
from kf_nusc import Nusc, DATA, world_to_px, HORIZONS   # noqa: E402
from yolo_nusc import (WEIGHTS, COCO2GRP, RANGE_MAX, SIG_RANGE, SIGV_YOUNG,  # noqa: E402
                       px_to_ground)

MISS_SEC = 2.0                                          # kill a world-track after 2 s unseen


def main(scene_names, render):
    import cv2
    from ultralytics import YOLO
    nusc = Nusc()
    out_dir = os.path.join(_HERE, "out")
    os.makedirs(out_dir, exist_ok=True)
    # ALL CAM_FRONT records (sweeps + keyframes) grouped per scene, time-ordered
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

    rows, n_fp = [], 0
    col = {"pedestrian": (60, 140, 255), "vehicle": (80, 220, 80), "cycle": (255, 200, 0)}
    for scene in nusc.scene:
        if scene_names and scene["name"] not in scene_names:
            continue
        model = YOLO(WEIGHTS)                            # fresh model per scene = fresh ByteTrack state
        chain = nusc.sample_chain(scene)
        ts_key = {s["token"]: s["timestamp"] / 1e6 for s in chain}
        gt = {}
        for s in chain:
            for a in nusc.anns_by_sample.get(s["token"], []):
                if a["grp"]:
                    gt.setdefault(a["instance_token"], []).append((ts_key[s["token"]], a))
        gt = {k: sorted(v, key=lambda x: x[0]) for k, v in gt.items()}

        def gt_xy_at(inst, t):
            obs = gt[inst]
            if t < obs[0][0] - 1e-9 or t > obs[-1][0] + 1e-9:
                return None
            for (t0, a0), (t1, a1) in zip(obs, obs[1:]):
                if t0 - 1e-9 <= t <= t1 + 1e-9:
                    w = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                    return (1 - w) * np.asarray(a0["translation"][:2]) + w * np.asarray(a1["translation"][:2])
            return np.asarray(obs[-1][1]["translation"][:2])

        world = {}                                       # ByteTrack id -> dict(trk, t_last, grp, hist)
        vw = None
        for sd_rec in frames[scene["name"]]:
            t_now = sd_rec["timestamp"] / 1e6
            img_path = os.path.join(DATA, sd_rec["filename"])
            res = model.track(img_path, persist=True, tracker="bytetrack.yaml",
                              conf=0.1, verbose=False)[0]
            W, H = sd_rec.get("width", 1600), sd_rec.get("height", 900)
            seen = set()
            for b in res.boxes:
                if b.id is None:
                    continue
                grp = COCO2GRP.get(model.names[int(b.cls)])
                if grp is None:
                    continue
                x0, y0, x1, y1 = [float(v) for v in b.xyxy[0]]
                if x0 < 4 or x1 > W - 4 or y1 > H - 4 or (y1 - y0) < 18:
                    continue
                g = px_to_ground(((x0 + x1) / 2.0, y1), sd_rec, nusc)
                if g is None:
                    continue
                ego_xy = np.asarray(nusc.ego[sd_rec["ego_pose_token"]]["translation"][:2])
                d_ego = float(np.linalg.norm(g[:2] - ego_xy))
                if d_ego > RANGE_MAX:
                    continue
                tid = int(b.id)
                w = world.get(tid)
                if w is None:
                    w = world[tid] = {"trk": MoverTracker(dt=0.083, meas_noise=0.5),
                                      "t_last": None, "grp": grp, "hist": [], "bbox": None}
                trk = w["trk"]
                sig = SIG_RANGE(d_ego)
                trk.fx.R = trk.fy.R = sig * sig
                det = np.array([g[0], g[1], 0.0])
                if w["t_last"] is None:
                    trk.update(det)
                else:
                    trk.update(det, dt=max(1e-3, t_now - w["t_last"]))
                w["t_last"] = t_now; w["hist"].append(det); w["bbox"] = (x0, y0, x1, y1)
                seen.add(tid)
            for tid, w in list(world.items()):
                if tid not in seen and w["t_last"] is not None:
                    if t_now - w["t_last"] > MISS_SEC:
                        del world[tid]

            is_key = bool(sd_rec["is_key_frame"])
            if is_key:                                   # same exam sheet as the other two arms
                for tid, w in world.items():
                    if tid not in seen or not w["trk"].ready:
                        continue
                    obs_xy = w["hist"][-1][:2]
                    cand = [(np.linalg.norm(gt_xy_at(k, t_now) - obs_xy), k) for k in gt
                            if gt_xy_at(k, t_now) is not None]
                    cand = [c for c in cand if c[0] < 2.5]
                    if not cand:
                        n_fp += 1
                        continue
                    inst = min(cand)[1]
                    frozen = w["trk"].sigma_v > SIGV_YOUNG[w["grp"]]
                    for h in HORIZONS:
                        gxy = gt_xy_at(inst, t_now + h)
                        if gxy is None:
                            continue
                        raw = w["trk"].predict([h], model="cv")[0][:2]      # shadow: ungated prediction
                        pxy = obs_xy if frozen else raw
                        rows.append((scene["name"], w["grp"], h,
                                     float(np.linalg.norm(pxy - gxy)),
                                     float(np.linalg.norm(obs_xy - gxy)),
                                     float(np.linalg.norm(raw - gxy)),
                                     float(w["trk"].sigma_v)))

            if render:
                img = cv2.imread(img_path)
                for tid, w in world.items():
                    if tid in seen and w["bbox"] is not None:
                        x0, y0, x1, y1 = [int(v) for v in w["bbox"]]
                        cv2.rectangle(img, (x0, y0), (x1, y1), col[w["grp"]], 2)
                        cv2.putText(img, f"#{tid}", (x0, y0 - 6), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6, col[w["grp"]], 2, cv2.LINE_AA)
                    if (tid in seen and w["trk"].ready and w["trk"].n >= 3
                            and w["trk"].sigma_v <= SIGV_YOUNG[w["grp"]]):
                        fut = w["trk"].predict(np.arange(0.0, 3.01, 0.25), model="cv")
                        px_f, ok = world_to_px(fut, sd_rec, nusc)
                        pts = px_f[ok].astype(int)
                        for p0, p1 in zip(pts, pts[1:]):
                            cv2.line(img, tuple(p0), tuple(p1), col[w["grp"]], 2, cv2.LINE_AA)
                        if len(pts):
                            cv2.circle(img, tuple(pts[-1]), 5, col[w["grp"]], -1, cv2.LINE_AA)
                cv2.putText(img, f"{scene['name']} YOLO26s + ByteTrack @12Hz -> prod KF", (16, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
                if vw is None:
                    vw = cv2.VideoWriter(os.path.join(out_dir, f"byte_{scene['name']}.mp4"),
                                         cv2.VideoWriter_fourcc(*"mp4v"), 12,
                                         (img.shape[1], img.shape[0]))
                vw.write(img)
        if vw is not None:
            vw.release()
            print("wrote", os.path.join(out_dir, f"byte_{scene['name']}.mp4"))

    if not rows:
        print("no scored rows"); return
    rows = np.core.records.fromrecords(rows, names="scene,grp,h,err,still,raw,sigv")
    print(f"\nkeyframe-scored updates FP (no GT mover within 2.5m)={n_fp}")
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
    print("\n--- shadow: UNGATED prediction vs still, bucketed by sigma_v (h=2.0s) ---")
    for grp in ("pedestrian", "vehicle"):
        for lo, hi in ((0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.5), (2.5, 99.0)):
            e = rows[(rows.grp == grp) & (rows.h == 2.0) & (rows.sigv >= lo) & (rows.sigv < hi)]
            if len(e):
                print(f"{grp:12s} sigv[{lo:3.1f},{hi:3.1f}) n={len(e):4d}  raw {np.mean(e.raw):5.2f}/{np.median(e.raw):5.2f}  still {np.mean(e.still):5.2f}/{np.median(e.still):5.2f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", type=str, default="scene-0103,scene-1094")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--no_render", action="store_true")
    args = ap.parse_args()
    main(None if args.all else set(args.scenes.split(",")), not args.no_render)
