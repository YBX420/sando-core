"""kf_nusc -- run the PRODUCTION MoverTracker (metaurban/kf_tracker.py, unmodified import) on nuScenes
v1.0-mini and score its trajectory predictions against the GT annotation chains.

Faces / honesty:
  - Feed = GT 3D-box centres (global frame) at the REAL keyframe cadence (~0.5 s, jittery timestamps),
    associated by instance_token (GT association). This is the real-data twin of the render face's
    GT_ORACLE association arm: it isolates the ESTIMATOR, not the detector/associator.
  - Scenes are processed independently (each scene is one continuous 20 s clip; no cross-scene stitching).
  - Metric = horizontal (xy) prediction error at horizon h, against the GT future linearly interpolated
    along the same instance's annotation chain. ADE-style mean + median, split young (n==2) vs mature.
  - Baseline "still" = predict the mover stays where it is: any useful estimator must beat it on movers.

Run:  ~/miniconda3/envs/metaurban/bin/python sando-core/nuscenes/kf_nusc.py [--overlay]
"""
import argparse
import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "metaurban"))
from kf_tracker import MoverTracker  # noqa: E402  (the production estimator, byte-identical)

DATA = os.environ.get("NUSC_DATA", "/media/boxuan/Data2/projects/sando_py/data/v1.0-mini")
TABLE_DIR = os.path.join(DATA, "v1.0-mini")
HORIZONS = (0.5, 1.0, 2.0, 3.0)
PRED_MODELS = ("cv", "ca")


def _load(name):
    with open(os.path.join(TABLE_DIR, f"{name}.json")) as f:
        return json.load(f)


def class_group(cat_name):
    """nuScenes category -> our per-class buckets; None = not a mover we track."""
    if cat_name.startswith("human.pedestrian"):
        return "pedestrian"
    if cat_name in ("vehicle.bicycle", "vehicle.motorcycle"):
        return "cycle"
    if cat_name.startswith("vehicle"):
        return "vehicle"
    if cat_name == "animal":
        return "animal"
    return None


def quat_rot(q):
    """nuScenes [w,x,y,z] quaternion -> 3x3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class Nusc:
    def __init__(self):
        self.scene = _load("scene")
        self.sample = {s["token"]: s for s in _load("sample")}
        self.sd = _load("sample_data")
        self.ego = {e["token"]: e for e in _load("ego_pose")}
        self.cs = {c["token"]: c for c in _load("calibrated_sensor")}
        cats = {c["token"]: c["name"] for c in _load("category")}
        inst = {i["token"]: i for i in _load("instance")}
        self.attr = {a["token"]: a["name"] for a in _load("attribute")}
        self.anns = {}
        self.anns_by_sample = {}
        for a in _load("sample_annotation"):
            a["cat"] = cats[inst[a["instance_token"]]["category_token"]]
            a["grp"] = class_group(a["cat"])
            self.anns[a["token"]] = a
            self.anns_by_sample.setdefault(a["sample_token"], []).append(a)
        # CAM_FRONT keyframe per sample
        self.cam_front = {}
        for d in self.sd:
            if d["is_key_frame"] and "CAM_FRONT/" in d["filename"]:
                self.cam_front[d["sample_token"]] = d

    def sample_chain(self, scene):
        out, tok = [], scene["first_sample_token"]
        while tok:
            s = self.sample[tok]
            out.append(s)
            tok = s["next"] or None
        return out


def world_to_px(pts_w, sd_rec, nusc):
    """Global-frame Nx3 -> (Nx2 pixels, in-front mask) for one CAM_FRONT sample_data record."""
    ego = nusc.ego[sd_rec["ego_pose_token"]]
    cs = nusc.cs[sd_rec["calibrated_sensor_token"]]
    p = np.asarray(pts_w, float) - np.asarray(ego["translation"])
    p = p @ quat_rot(ego["rotation"])            # world -> ego (R^T applied via right-multiplication)
    p = (p - np.asarray(cs["translation"])) @ quat_rot(cs["rotation"])  # ego -> cam
    K = np.asarray(cs["camera_intrinsic"])
    infront = p[:, 2] > 0.5
    z = np.where(infront, p[:, 2], 1.0)
    px = np.stack([K[0, 0] * p[:, 0] / z + K[0, 2], K[1, 1] * p[:, 1] / z + K[1, 2]], axis=1)
    return px, infront


def run(overlay_scenes=(), out_dir=os.path.join(_HERE, "out")):
    nusc = Nusc()
    rows = []          # (scene, grp, attr, n_at_pred, sigma_v, horizon, model, err, still_err)
    overlays = {}      # (scene_name, sample_idx) -> list of drawables
    for scene in nusc.scene:
        chain = nusc.sample_chain(scene)
        ts = {s["token"]: s["timestamp"] / 1e6 for s in chain}
        # per-instance GT chains inside this scene (for future interpolation)
        gt = {}
        for s in chain:
            for a in nusc.anns_by_sample.get(s["token"], []):
                if a["grp"]:
                    gt.setdefault(a["instance_token"], []).append((ts[s["token"]], a))
        gt = {k: sorted(v) for k, v in gt.items()}

        def gt_xy_at(inst, t):
            obs = gt[inst]
            if t < obs[0][0] - 1e-9 or t > obs[-1][0] + 1e-9:
                return None
            for (t0, a0), (t1, a1) in zip(obs, obs[1:]):
                if t0 - 1e-9 <= t <= t1 + 1e-9:
                    w = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                    p0 = np.asarray(a0["translation"][:2])
                    p1 = np.asarray(a1["translation"][:2])
                    return (1 - w) * p0 + w * p1
            return np.asarray(obs[-1][1]["translation"][:2])

        trackers, last_t, hist = {}, {}, {}
        for si, s in enumerate(chain):
            t_now = ts[s["token"]]
            draw = []
            for a in nusc.anns_by_sample.get(s["token"], []):
                if not a["grp"]:
                    continue
                inst = a["instance_token"]
                det = np.asarray(a["translation"], float)
                trk = trackers.get(inst)
                if trk is None:
                    trk = trackers[inst] = MoverTracker(dt=0.5, meas_noise=0.07)
                    trk.update(det)
                else:
                    trk.update(det, dt=t_now - last_t[inst])
                last_t[inst] = t_now
                hist.setdefault(inst, []).append(det.copy())
                attr = nusc.attr[a["attribute_tokens"][0]] if a["attribute_tokens"] else "-"
                if trk.ready:
                    for h in HORIZONS:
                        gxy = gt_xy_at(inst, t_now + h)
                        if gxy is None:
                            continue
                        still = float(np.linalg.norm(det[:2] - gxy))
                        for m in PRED_MODELS:
                            pxy = trk.predict([h], model=m)[0][:2]
                            rows.append((scene["name"], a["grp"], attr, trk.n, trk.sigma_v, h, m,
                                         float(np.linalg.norm(pxy - gxy)), still))
                if trk.ready and trk.n >= 3:
                    fut = trk.predict(np.arange(0.0, 3.01, 0.25), model="cv")
                    draw.append((a["grp"], np.asarray(hist[inst]), fut, a))
            if scene["name"] in overlay_scenes and draw:
                overlays[(scene["name"], si)] = (s, draw)

    rows = np.core.records.fromrecords(
        rows, names="scene,grp,attr,n,sigma_v,h,model,err,still")

    def agg(mask, label):
        sub = rows[mask]
        if not len(sub):
            return
        line = f"{label:38s}"
        for h in HORIZONS:
            for m in PRED_MODELS:
                e = sub[(sub.h == h) & (sub.model == m)].err
                line += f" {np.mean(e):5.2f}/{np.median(e):5.2f}" if len(e) else "    - /  -  "
            st = sub[(sub.h == h) & (sub.model == "cv")].still
            line += f" |{np.mean(st):5.2f}" if len(st) else " |  -  "
        print(line)

    hdr = f"{'bucket (mean/median m)':38s}"
    for h in HORIZONS:
        hdr += f"   h={h:.1f}s cv      ca    |still"
    print(hdr)
    moving_attrs = ("vehicle.moving", "pedestrian.moving", "cycle.with_rider")
    for grp in ("pedestrian", "vehicle", "cycle", "animal"):
        agg(rows.grp == grp, f"{grp} (all)")
        agg((rows.grp == grp) & np.isin(rows.attr, moving_attrs), f"{grp} MOVING")
    agg(np.isin(rows.attr, moving_attrs) & (rows.n == 2), "MOVING young (n=2)")
    agg(np.isin(rows.attr, moving_attrs) & (rows.n >= 4), "MOVING mature (n>=4)")
    print(f"\n{len(rows)//len(PRED_MODELS)} 个(观测,horizon)评分点; scenes={len(nusc.scene)}")

    # ---- CAM_FRONT overlays ----
    if overlays:
        import cv2
        os.makedirs(out_dir, exist_ok=True)
        col = {"pedestrian": (60, 140, 255), "vehicle": (80, 220, 80),
               "cycle": (255, 200, 0), "animal": (255, 80, 255)}
        for (sname, si), (s, draw) in overlays.items():
            sd_rec = nusc.cam_front[s["token"]]
            img = cv2.imread(os.path.join(DATA, sd_rec["filename"]))
            for grp, past, fut, a in draw:
                px_f, ok_f = world_to_px(fut, sd_rec, nusc)
                pts = px_f[ok_f].astype(int)
                for p0, p1 in zip(pts, pts[1:]):
                    cv2.line(img, tuple(p0), tuple(p1), col[grp], 2, cv2.LINE_AA)
                if len(pts):
                    cv2.circle(img, tuple(pts[-1]), 5, col[grp], -1, cv2.LINE_AA)
                px_p, ok_p = world_to_px(past[-6:], sd_rec, nusc)
                for p in px_p[ok_p].astype(int):
                    cv2.circle(img, tuple(p), 2, (255, 255, 255), -1, cv2.LINE_AA)
            cv2.putText(img, f"{sname} kf(prod) cv 3s", (16, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
            fp = os.path.join(out_dir, f"kf_{sname}_f{si:02d}.jpg")
            cv2.imwrite(fp, img)
            print("wrote", fp)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--overlay", action="store_true", help="also write CAM_FRONT overlay jpgs")
    args = ap.parse_args()
    run(overlay_scenes=("scene-0103", "scene-1094") if args.overlay else ())
