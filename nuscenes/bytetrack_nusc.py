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

# ---- the deployed v6.1 CAPSULE keep-out, THIN (oracle-arm) sizing, transplanted verbatim from
# render_3d_video._cap_ring so what we draw here IS the algorithm's avoidance body ----
# ILLUSTRATION ONLY on this face: the thin calibration was harvested on MetaUrban and carries no
# estimation-error budget; on monocular nuScenes it shows the deployed SIZE, it certifies nothing.
CAP_TAU, CAP_RDT = 0.75, 0.1            # trust window + replan_dt (metaurban_sando.yaml)
CAP_Q, CAP_VEFF, CAP_DSAFE, CAP_K = 0.05, 0.1, 0.15, 6      # calib_v6_thin.json + MANDSAFE=0.15
CAP_REAR = (0.05, 0.0)                  # v6.1 flat rear: rear-overrun quantile q0r + growth
R_OBS = {"pedestrian": 0.30, "cycle": 0.40, "vehicle": 1.00}


def capsule_ring(c0, vel, r_obs):
    """Ground outline of the deployed thin capsule: stadium from the mover's flat BACK to the
    KF-apex cap at c0 + v*tau. Verbatim geometry of render_3d_video._cap_ring (v6.1)."""
    sp = float(np.hypot(vel[0], vel[1]))
    veff = CAP_VEFF + sp / (2.0 * (CAP_K - 1))
    rad = r_obs + CAP_DSAFE + CAP_Q + veff * (CAP_TAU + CAP_RDT)
    c0 = np.asarray(c0[:2], float)
    tip = c0 + np.asarray(vel[:2], float) * CAP_TAU
    u = (tip - c0) / sp / CAP_TAU if sp > 1e-6 else np.array([1.0, 0.0])
    a0 = float(np.arctan2(u[1], u[0]))
    if sp > 1e-6:
        back = CAP_REAR[0] + CAP_REAR[1] * (CAP_TAU + CAP_RDT) + r_obs + CAP_DSAFE
        bl = c0 - u * back
        n = np.array([-u[1], u[0]])
        pts = [tuple(bl + n * rad)]
        pts += [(float(tip[0] + rad * np.cos(a0 + th)), float(tip[1] + rad * np.sin(a0 + th)))
                for th in np.linspace(np.pi / 2, -np.pi / 2, 13)]
        pts += [tuple(bl - n * rad)]
        return pts + [pts[0]], rad
    pts = [(float(c0[0] + rad * np.cos(th)), float(c0[1] + rad * np.sin(th)))
           for th in np.linspace(0, 2 * np.pi, 25)]
    return pts, rad

# bbox-bottom slide budget (m): passing a STATIC object slides its ground point along the body, so
# "moved" must mean displacement beyond noise AND beyond this systematic slide -- per class.
SLIDE = {"pedestrian": 0.3, "cycle": 0.8, "vehicle": 2.0}


class MotionLatch:
    """3-state motion certificate from DISPLACEMENT over a window, not instantaneous velocity.
    The wall was: v-SNR at one tick is hopeless (sigma_d ~ 1 m vs 0.05 m of true motion). But over
    T seconds true displacement grows ~v*T while the noise of a windowed mean SHRINKS ~sigma/sqrt(n):
    split the window in half, compare the two means. STATIC latches v=0 + weighted-mean position;
    MOVING is the licence to speak; hysteresis (2 consecutive verdicts to flip) kills flicker."""

    def __init__(self, grp, win=3.0):
        self.slide = SLIDE[grp]; self.win = float(win)
        self.obs = []                                   # (t, x, y, sig)
        self.state = "UNKNOWN"; self._pend = None; self._npend = 0

    def add(self, t, xy, sig):
        self.obs.append((t, float(xy[0]), float(xy[1]), float(sig)))
        t0 = t - self.win
        while self.obs and self.obs[0][0] < t0:
            self.obs.pop(0)
        self._tick()

    def _halves(self):
        n = len(self.obs)
        if n < 6:
            return None
        a, b = self.obs[: n // 2], self.obs[n // 2:]
        ma = np.array([[o[1], o[2]] for o in a]).mean(axis=0)
        mb = np.array([[o[1], o[2]] for o in b]).mean(axis=0)
        sa = np.mean([o[3] for o in a]) / max(1.0, len(a)) ** 0.5
        sb = np.mean([o[3] for o in b]) / max(1.0, len(b)) ** 0.5
        return float(np.linalg.norm(mb - ma)), float((sa * sa + sb * sb) ** 0.5)

    def _tick(self):
        h = self._halves()
        if h is None:
            return
        d, sd = h
        if d > 3.0 * sd + self.slide:
            v = "MOVING"
        elif d < 1.5 * sd + 0.5 * self.slide:
            v = "STATIC"
        else:
            v = None
        if v is None or v == self.state:
            self._pend, self._npend = None, 0; return
        if v == self._pend:
            self._npend += 1
        else:
            self._pend, self._npend = v, 1
        if self._npend >= 2:                            # hysteresis: two consecutive verdicts to flip
            self.state, self._pend, self._npend = v, None, 0

    @property
    def vel_win(self):
        """Displacement velocity: (mean of 2nd half - mean of 1st half) / dt between half centres --
        the best-SNR velocity this observation chain can produce (noise shrinks with sqrt(n), true
        displacement grows with T). This, not the KF instantaneous v, is MOVING's licence to speak."""
        n = len(self.obs)
        if n < 6:
            return np.zeros(2)
        a, b = self.obs[: n // 2], self.obs[n // 2:]
        ma = np.array([[o[1], o[2]] for o in a]).mean(axis=0)
        mb = np.array([[o[1], o[2]] for o in b]).mean(axis=0)
        ta = np.mean([o[0] for o in a]); tb = np.mean([o[0] for o in b])
        return (mb - ma) / max(0.2, tb - ta)

    @property
    def mean_pos(self):
        w = np.array([1.0 / max(o[3], 0.1) ** 2 for o in self.obs])
        pts = np.array([[o[1], o[2]] for o in self.obs])
        return (pts * w[:, None]).sum(axis=0) / w.sum()


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
    ATTR_MOTION = {"vehicle.moving": "MOVING", "vehicle.stopped": "STATIC", "vehicle.parked": "STATIC",
                   "pedestrian.moving": "MOVING", "pedestrian.standing": "STATIC",
                   "pedestrian.sitting_lying_down": "STATIC",
                   "cycle.with_rider": "MOVING", "cycle.without_rider": "STATIC"}
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

        def gt_attr_at(inst, t):
            obs = gt[inst]
            a = min(obs, key=lambda o: abs(o[0] - t))[1]
            return nusc.attr[a["attribute_tokens"][0]] if a["attribute_tokens"] else "-"

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
                                      "t_last": None, "grp": grp, "hist": [], "bbox": None,
                                      "latch": MotionLatch(grp)}
                trk = w["trk"]
                sig = SIG_RANGE(d_ego)
                trk.fx.R = trk.fy.R = sig * sig
                det = np.array([g[0], g[1], 0.0])
                if w["t_last"] is None:
                    trk.update(det)
                else:
                    trk.update(det, dt=max(1e-3, t_now - w["t_last"]))
                w["t_last"] = t_now; w["hist"].append(det); w["bbox"] = (x0, y0, x1, y1)
                w["latch"].add(t_now, det[:2], sig)
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
                    st = w["latch"].state
                    # motion-latch policy: STATIC -> weighted-mean anchor (noise-averaged, v=0);
                    # MOVING -> the latch IS the licence, speak KF-CV; UNKNOWN -> frozen last obs.
                    for h in HORIZONS:
                        gxy = gt_xy_at(inst, t_now + h)
                        if gxy is None:
                            continue
                        raw = w["trk"].predict([h], model="cv")[0][:2]      # shadow: ungated prediction
                        # VERDICT (2026-07-14): on bbox-bottom monocular depth, velocity is unmeasurable
                        # in EVERY form tried -- (1) KF instantaneous v: loses to still in every sigma_v
                        # bucket; (2) STATIC mean-anchor: 3.35 vs 2.85 still (error is BIAS, averaging
                        # can't wash it); (3) window displacement velocity: 8.07/8.36 vs still 8.11/3.71
                        # (mean wash, median worse). Ship the no-harm floor; the latch stays as a
                        # DIAGNOSTIC/classifier (vehicles: only 5 false-moving rows; slow peds: 31 missed,
                        # their true displacement sits under the noise+slide floor at range).
                        if st == "MOVING":
                            pxy = obs_xy + w["latch"].vel_win * h   # kept for the shadow record only
                        else:
                            pxy = obs_xy
                        rows.append((scene["name"], w["grp"], h,
                                     float(np.linalg.norm(pxy - gxy)),
                                     float(np.linalg.norm(obs_xy - gxy)),
                                     float(np.linalg.norm(raw - gxy)),
                                     float(w["trk"].sigma_v), st,
                                     ATTR_MOTION.get(gt_attr_at(inst, t_now), "-")))

            if render:
                img = cv2.imread(img_path)
                for tid, w in world.items():
                    if tid not in seen:
                        continue
                    trk = w["trk"]
                    frz = (not trk.ready) or trk.sigma_v > SIGV_YOUNG[w["grp"]]
                    c = (160, 160, 160) if frz else col[w["grp"]]        # grey = young gate CLOSED
                    if w["bbox"] is not None:
                        x0, y0, x1, y1 = [int(v) for v in w["bbox"]]
                        cv2.rectangle(img, (x0, y0), (x1, y1), col[w["grp"]], 2)
                        tag = f"#{tid} sv{trk.sigma_v:.1f}" + (" FRZ" if frz else "")
                        cv2.putText(img, tag, (x0, y0 - 6), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.55, c, 2, cv2.LINE_AA)
                    if not trk.ready:
                        continue
                    # --- the KF's own state, made visible ---
                    c0, v, _ = trk.state()
                    ctr_px, okc = world_to_px(np.array([[c0[0], c0[1], 0.0]]), sd_rec, nusc)
                    if okc[0]:
                        cx, cy = ctr_px[0].astype(int)
                        cv2.drawMarker(img, (cx, cy), c, cv2.MARKER_TILTED_CROSS, 14, 2)  # KF centre
                    sig_p = min(trk.pos_sigma, 6.0)
                    ang = np.linspace(0, 2 * np.pi, 17)
                    ring = np.stack([c0[0] + sig_p * np.cos(ang), c0[1] + sig_p * np.sin(ang),
                                     np.zeros_like(ang)], axis=1)
                    px_r, okr = world_to_px(ring, sd_rec, nusc)
                    pr = px_r[okr].astype(int)
                    for p0, p1 in zip(pr, pr[1:]):                       # 1-sigma position ring
                        cv2.line(img, tuple(p0), tuple(p1), c, 1, cv2.LINE_AA)
                    # --- the algorithm's avoidance body: deployed THIN capsule (cyan) ---
                    v_cap = w["latch"].vel_win if w["latch"].state == "MOVING" else np.zeros(2)
                    ring_c, rad_c = capsule_ring(c0, v_cap, R_OBS[w["grp"]])
                    ring3 = np.array([[px_, py_, 0.0] for px_, py_ in ring_c])
                    px_cap, okcap = world_to_px(ring3, sd_rec, nusc)
                    pc = px_cap[okcap].astype(int)
                    for p0, p1 in zip(pc, pc[1:]):
                        cv2.line(img, tuple(p0), tuple(p1), (255, 255, 0), 2, cv2.LINE_AA)
                    if okc[0]:
                        cv2.putText(img, f"r{rad_c:.2f}", (cx + 8, cy + 16),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
                    tip = np.array([[c0[0] + v[0], c0[1] + v[1], 0.0]])  # velocity arrow (1 s)
                    px_t, okt = world_to_px(tip, sd_rec, nusc)
                    if okc[0] and okt[0]:
                        cv2.arrowedLine(img, tuple(ctr_px[0].astype(int)), tuple(px_t[0].astype(int)),
                                        c, 2, cv2.LINE_AA, tipLength=0.25)
                    if not frz and trk.n >= 3:                           # certified-to-speak prediction
                        fut = trk.predict(np.arange(0.0, 3.01, 0.25), model="cv")
                        px_f, ok = world_to_px(fut, sd_rec, nusc)
                        pts = px_f[ok].astype(int)
                        for p0, p1 in zip(pts, pts[1:]):
                            cv2.line(img, tuple(p0), tuple(p1), col[w["grp"]], 2, cv2.LINE_AA)
                        if len(pts):
                            cv2.circle(img, tuple(pts[-1]), 5, col[w["grp"]], -1, cv2.LINE_AA)
                cv2.putText(img, f"{scene['name']} YOLO26s+ByteTrack@12Hz -> KF | x=centre ring=1sig arrow=v*1s grey=FRZ cyan=THIN capsule (deployed size, illustrative)", (16, 40),
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
    rows = np.core.records.fromrecords(rows, names="scene,grp,h,err,still,raw,sigv,latch,gtmot")
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
    print("\n--- motion-latch confusion vs GT attribute (per scored row, h=2.0s) ---")
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
            print(f"{lt:8s} n={len(sub):4d}  policy {np.mean(sub.err):5.2f}/{np.median(sub.err):5.2f}  still {np.mean(sub.still):5.2f}/{np.median(sub.still):5.2f}")
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
