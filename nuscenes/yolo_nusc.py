"""yolo_nusc -- the NO-ANNOTATION twin of kf_nusc: YOLO detections instead of GT boxes.

Pipeline (annotations are NEVER an input -- eval only):
  CAM_FRONT keyframe -> YOLO26 (COCO classes person/car/truck/bus/bicycle/motorcycle)
  -> bbox bottom-centre back-projected onto the world ground plane (z=0; nuScenes map frame is ~ground)
  -> per-class greedy nearest-neighbour association against each track's dt-predicted position (OUR
     association now, not instance_token) -> the PRODUCTION MoverTracker (coast on miss, kill after 4)
  -> 3 s CV prediction projected back into CAM_FRONT (overlay jpgs + per-scene mp4).

Eval (annotation as exam sheet only): each update is matched to the nearest GT mover within 2.5 m at
observation time; prediction error vs that instance's GT future -- same table as kf_nusc, so the
GT-fed vs YOLO-fed delta IS the detector+backprojection tax.

Honesty:
  - ground-plane back-projection puts the point at the box's NEAR edge, not the body centre ->
    a systematic ~half-length bias for close vehicles; reported as-is.
  - meas_noise=0.5 m (backprojection noise), vs 0.07 on the GT face.
  - boxes touching the image border / tiny boxes / range >40 m are dropped (no honest ground point).

Run:  ~/miniconda3/envs/metaurban/bin/python sando-core/nuscenes/yolo_nusc.py [--scenes scene-0103,...]
"""
import argparse
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "metaurban"))
sys.path.insert(0, _HERE)
from kf_tracker import MoverTracker            # noqa: E402  the production estimator
from kf_nusc import Nusc, DATA, quat_rot, world_to_px, HORIZONS  # noqa: E402

WEIGHTS = os.environ.get("YOLO_W", "/media/boxuan/Data2/projects/cvmusecore/yolo26s.pt")
COCO2GRP = {"person": "pedestrian", "car": "vehicle", "truck": "vehicle", "bus": "vehicle",
            "bicycle": "cycle", "motorcycle": "cycle"}
GATE_V = {"pedestrian": 3.0, "cycle": 8.0, "vehicle": 20.0}   # m/s association gate growth
CONF, MISS_MAX, RANGE_MAX = 0.40, 4, 40.0
# monocular ground-projection noise GROWS with range: sigma_d ~ d^2 * px_err / (f * cam_h)
# (f=1266, cam_h~1.5 m, ~2 px bottom-edge jitter -> d^2/950). A constant meas_noise is a MetaUrban
# assumption that real perception breaks -- booked for the KF campaign.
SIG_RANGE = lambda d: 0.3 + d * d / 950.0
# young honesty gate (production KF cut 1, same law on the real-data face): while sigma_v hasn't
# converged, the honest prediction is FROZEN at the last observation, not a garbage velocity.
# Per-class relaxed gates (ped 0.5 / cycle 1.5 / vehicle 3.0) tested NEGATIVE (2026-07-14): opening
# the vehicle gate made 3s error 15.4 vs 6.0 frozen -- the monocular velocity error is a systematic
# BIAS (bbox bottom = the vehicle's near edge, sliding along the body as perspective changes), which
# sigma_v (variance) cannot see. Uniform production gate; the fix belongs in the OBSERVATION, not the gate.
_SG = float(os.environ.get("KF_SIGV_YOUNG", "0.5"))
SIGV_YOUNG = {"pedestrian": _SG, "cycle": _SG, "vehicle": _SG}


def px_to_ground(px_uv, sd_rec, nusc):
    """Pixel -> world point on the z=0 ground plane (None if the ray points above the horizon)."""
    ego = nusc.ego[sd_rec["ego_pose_token"]]
    cs = nusc.cs[sd_rec["calibrated_sensor_token"]]
    K = np.asarray(cs["camera_intrinsic"])
    d_cam = np.array([(px_uv[0] - K[0, 2]) / K[0, 0], (px_uv[1] - K[1, 2]) / K[1, 1], 1.0])
    R_e, R_w = quat_rot(cs["rotation"]), quat_rot(ego["rotation"])
    d_w = R_w @ (R_e @ d_cam)
    o_w = R_w @ np.asarray(cs["translation"]) + np.asarray(ego["translation"])
    if d_w[2] > -1e-6:
        return None
    lam = -o_w[2] / d_w[2]
    if lam <= 0 or lam > RANGE_MAX * 2:
        return None
    return o_w + lam * d_w


class Track:
    _next = 1

    def __init__(self, grp, det_xyz, t):
        self.id = Track._next; Track._next += 1
        self.grp = grp
        self.trk = MoverTracker(dt=0.5, meas_noise=0.5)
        self.trk.update(det_xyz)
        self.t_last = t
        self.hist = [np.asarray(det_xyz)]


def main(scene_names, render, conf):
    import cv2
    from ultralytics import YOLO
    nusc = Nusc()
    model = YOLO(WEIGHTS)
    out_dir = os.path.join(_HERE, "out")
    os.makedirs(out_dir, exist_ok=True)
    rows, n_fp, n_det = [], 0, 0
    col = {"pedestrian": (60, 140, 255), "vehicle": (80, 220, 80), "cycle": (255, 200, 0)}

    for scene in nusc.scene:
        if scene_names and scene["name"] not in scene_names:
            continue
        chain = nusc.sample_chain(scene)
        ts = {s["token"]: s["timestamp"] / 1e6 for s in chain}
        gt = {}                                             # eval-only GT chains (same as kf_nusc)
        for s in chain:
            for a in nusc.anns_by_sample.get(s["token"], []):
                if a["grp"]:
                    gt.setdefault(a["instance_token"], []).append((ts[s["token"]], a))
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

        tracks, vw = [], None
        for si, s in enumerate(chain):
            t_now = ts[s["token"]]
            sd_rec = nusc.cam_front[s["token"]]
            img_path = os.path.join(DATA, sd_rec["filename"])
            res = model.predict(img_path, verbose=False, conf=conf)[0]
            W, H = sd_rec["width"], sd_rec["height"]
            dets = []                                       # (grp, world_xyz, bbox)
            for b in res.boxes:
                grp = COCO2GRP.get(model.names[int(b.cls)])
                if grp is None:
                    continue
                x0, y0, x1, y1 = [float(v) for v in b.xyxy[0]]
                if x0 < 4 or x1 > W - 4 or y1 > H - 4 or (y1 - y0) < 18:   # border-cut box: bottom is not ground
                    continue
                g = px_to_ground(((x0 + x1) / 2.0, y1), sd_rec, nusc)
                if g is None:
                    continue
                ego_xy = np.asarray(nusc.ego[sd_rec["ego_pose_token"]]["translation"][:2])
                if np.linalg.norm(g[:2] - ego_xy) > RANGE_MAX:
                    continue
                dets.append((grp, np.array([g[0], g[1], 0.0]), (x0, y0, x1, y1)))
            n_det += len(dets)

            # ---- OUR association: greedy NN, per class, against dt-predicted track positions ----
            unmatched = list(range(len(dets)))
            for tr in tracks:
                dt = t_now - tr.t_last
                pred = tr.trk.predict([dt], model="cv")[0][:2] if tr.trk.ready else tr.hist[-1][:2]
                gate = 1.0 + GATE_V[tr.grp] * dt
                best, best_d = None, gate
                for i in unmatched:
                    if dets[i][0] != tr.grp:
                        continue
                    d = float(np.linalg.norm(dets[i][1][:2] - pred))
                    if d < best_d:
                        best, best_d = i, d
                if best is not None:
                    unmatched.remove(best)
                    ego_xy = np.asarray(nusc.ego[sd_rec["ego_pose_token"]]["translation"][:2])
                    sig = SIG_RANGE(float(np.linalg.norm(dets[best][1][:2] - ego_xy)))
                    tr.trk.fx.R = tr.trk.fy.R = sig * sig      # range-dependent measurement noise
                    tr.trk.update(dets[best][1], dt=dt)
                    tr.t_last = t_now
                    tr.hist.append(dets[best][1])
                    tr.bbox = dets[best][2]
                else:
                    tr.trk.coast(dt=dt)
                    tr.t_last = t_now
                    tr.bbox = None
            tracks = [tr for tr in tracks if tr.trk.miss <= MISS_MAX]
            for i in unmatched:
                tr = Track(dets[i][0], dets[i][1], t_now)
                tr.bbox = dets[i][2]
                tracks.append(tr)

            # ---- eval only: score predictions against the nearest GT mover ----
            for tr in tracks:
                if tr.bbox is None or not tr.trk.ready:
                    continue
                obs_xy = tr.hist[-1][:2]
                cand = [(np.linalg.norm(gt_xy_at(k, t_now) - obs_xy), k) for k in gt
                        if gt_xy_at(k, t_now) is not None]
                cand = [c for c in cand if c[0] < 2.5]
                if not cand:
                    n_fp += 1
                    continue
                inst = min(cand)[1]
                frozen = tr.trk.sigma_v > SIGV_YOUNG[tr.grp]         # young honesty gate: no converged v -> stay put
                for h in HORIZONS:
                    gxy = gt_xy_at(inst, t_now + h)
                    if gxy is None:
                        continue
                    pxy = obs_xy if frozen else tr.trk.predict([h], model="cv")[0][:2]
                    rows.append((scene["name"], tr.grp, tr.trk.n, h,
                                 float(np.linalg.norm(pxy - gxy)),
                                 float(np.linalg.norm(obs_xy - gxy))))

            if render:
                img = cv2.imread(img_path)
                for tr in tracks:
                    if tr.bbox is not None:
                        x0, y0, x1, y1 = [int(v) for v in tr.bbox]
                        cv2.rectangle(img, (x0, y0), (x1, y1), col[tr.grp], 2)
                        cv2.putText(img, f"#{tr.id}", (x0, y0 - 6), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6, col[tr.grp], 2, cv2.LINE_AA)
                    if tr.trk.ready and tr.trk.n >= 3 and tr.trk.sigma_v <= SIGV_YOUNG[tr.grp]:
                        fut = tr.trk.predict(np.arange(0.0, 3.01, 0.25), model="cv")
                        px_f, ok = world_to_px(fut, sd_rec, nusc)
                        pts = px_f[ok].astype(int)
                        for p0, p1 in zip(pts, pts[1:]):
                            cv2.line(img, tuple(p0), tuple(p1), col[tr.grp], 2, cv2.LINE_AA)
                        if len(pts):
                            cv2.circle(img, tuple(pts[-1]), 5, col[tr.grp], -1, cv2.LINE_AA)
                cv2.putText(img, f"{scene['name']} YOLO26s -> prod KF (no annotations)", (16, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
                if vw is None:
                    vw = cv2.VideoWriter(os.path.join(out_dir, f"yolo_{scene['name']}.mp4"),
                                         cv2.VideoWriter_fourcc(*"mp4v"), 2, (img.shape[1], img.shape[0]))
                vw.write(img)
                cv2.imwrite(os.path.join(out_dir, f"yolo_{scene['name']}_f{si:02d}.jpg"), img)
        if vw is not None:
            vw.release()
            print("wrote", os.path.join(out_dir, f"yolo_{scene['name']}.mp4"))

    rows = np.core.records.fromrecords(rows, names="scene,grp,n,h,err,still") if rows else None
    if rows is None:
        print("no scored rows"); return
    print(f"\ndetections kept={n_det}  scored-updates FP (no GT mover within 2.5m)={n_fp}")
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


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", type=str, default="scene-0103,scene-1094")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--conf", type=float, default=CONF)
    ap.add_argument("--no_render", action="store_true")
    args = ap.parse_args()
    main(None if args.all else set(args.scenes.split(",")), not args.no_render, args.conf)
