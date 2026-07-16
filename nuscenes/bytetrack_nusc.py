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
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "metaurban"))
sys.path.insert(0, _HERE)
from kf_tracker import MoverTracker                     # noqa: E402
from kf_nusc import Nusc, DATA, world_to_px, HORIZONS   # noqa: E402
from yolo_nusc import (WEIGHTS, COCO2GRP, RANGE_MAX, SIG_RANGE, SIGV_YOUNG,  # noqa: E402
                       px_to_ground)

MISS_SEC = 2.0                                          # kill a world-track after 2 s unseen
# bbox-HEIGHT depth prior: depth = f * H_class / h_px. Bearing from the bbox centre is EXACT; depth
# error ~ prior spread (~10%) and -- unlike bottom-centre ground projection -- IMMUNE to ground
# relief (the flat-earth bias that parked the reborn #174 capsule metres in front of the car).
H_PRIOR = {"pedestrian": 1.70, "cycle": 1.60, "vehicle": 1.60}


def locate_hprior(bbox, sd_rec, nusc, grp, H=None):
    """World point from bbox centre bearing + height depth prior. H: the track's OWN lidar-calibrated
    height (H = z_lidar * h_px / f, EMA'd while lidar lives) beats the class prior -- #174 is a ~1.8 m
    MPV, the 1.6 m class prior under-ranged it 12% (4 m at 30 m) as soon as the laser dried up."""
    from kf_nusc import quat_rot as _qr
    x0, y0, x1, y1 = bbox
    cs = nusc.cs[sd_rec["calibrated_sensor_token"]]
    ego = nusc.ego[sd_rec["ego_pose_token"]]
    K = np.asarray(cs["camera_intrinsic"])
    z = K[1, 1] * (H if H else H_PRIOR[grp]) / max(1.0, (y1 - y0))
    u, v = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    p_cam = np.array([(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z])
    p = _qr(cs["rotation"]) @ p_cam + np.asarray(cs["translation"])
    p = _qr(ego["rotation"]) @ p + np.asarray(ego["translation"])
    return p


def world_to_cam(pts_w, sd_rec, nusc):
    """Global Nx3 -> (Nx2 pixels, Nx1 cam-depth). Same maths as world_to_px but returns z."""
    from kf_nusc import quat_rot as _qr
    ego = nusc.ego[sd_rec["ego_pose_token"]]
    cs = nusc.cs[sd_rec["calibrated_sensor_token"]]
    p = (np.asarray(pts_w, float) - np.asarray(ego["translation"])) @ _qr(ego["rotation"])
    p = (p - np.asarray(cs["translation"])) @ _qr(cs["rotation"])
    K = np.asarray(cs["camera_intrinsic"])
    z = p[:, 2]
    zs = np.where(z > 0.1, z, 1.0)
    px = np.stack([K[0, 0] * p[:, 0] / zs + K[0, 2], K[1, 1] * p[:, 1] / zs + K[1, 2]], axis=1)
    return px, z


class LidarDepth:
    """LIDAR_TOP -> world points near a camera frame; median-of-nearest-cluster depth per bbox.
    Kills the flat-earth wall: position comes from the laser, the ground can do what it wants."""

    def __init__(self, nusc):
        from kf_nusc import quat_rot as _qr
        self._qr = _qr
        self.nusc = nusc
        self.recs = sorted((d for d in nusc.sd if "LIDAR_TOP" in d["filename"]),
                           key=lambda d: d["timestamp"])
        self.ts = np.array([d["timestamp"] for d in self.recs])
        self._cache = (None, None)

    def world_cloud(self, t_us):
        i = int(np.clip(np.searchsorted(self.ts, t_us), 1, len(self.ts) - 1))
        rec = self.recs[i] if abs(self.ts[i] - t_us) < abs(self.ts[i - 1] - t_us) else self.recs[i - 1]
        if self._cache[0] == rec["token"]:
            return self._cache[1]
        pts = np.fromfile(os.path.join(DATA, rec["filename"]), dtype=np.float32).reshape(-1, 5)[:, :3]
        cs = self.nusc.cs[rec["calibrated_sensor_token"]]
        ego = self.nusc.ego[rec["ego_pose_token"]]
        pw = pts.astype(float) @ self._qr(cs["rotation"]).T + np.asarray(cs["translation"])
        pw = pw @ self._qr(ego["rotation"]).T + np.asarray(ego["translation"])
        self._cache = (rec["token"], pw)
        return pw

    def locate_box3d(self, t_us, c_xyz, size, rot_q):
        """GT-3D-box point selection: the SELECTION CEILING -- membership is exact (point inside the
        annotated box, 0.2 m margin), no projection, no silhouette, no contamination. What remains
        in the error is pure surface-sampling physics (self-occlusion, sparsity, timing)."""
        from kf_nusc import quat_rot as _qr
        pw = self.world_cloud(t_us)
        q = (pw - np.asarray(c_xyz)) @ _qr(rot_q)         # world -> box frame
        wd, ln, ht = size
        m = ((np.abs(q[:, 0]) < ln / 2 + 0.2) & (np.abs(q[:, 1]) < wd / 2 + 0.2)
             & (np.abs(q[:, 2]) < ht / 2 + 0.3))
        if m.sum() < 3:
            return None
        return np.median(pw[m], axis=0)

    def locate(self, bbox, sd_rec, t_us, mask=None):
        """Median world-xy of the nearest lidar cluster inside the (shrunk) bbox, or None.
        mask: optional (bitmap, x0, y0) instance mask -- points must fall ON the object's silhouette
        instead of merely inside the rectangle (kills occluder/background/neighbour contamination)."""
        pw = self.world_cloud(t_us)
        px, z = world_to_cam(pw, sd_rec, self.nusc)
        x0, y0, x1, y1 = bbox
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        hw, hh = 0.375 * (x1 - x0), 0.4 * (y1 - y0)      # shrink 25/20% against edge bleed
        m = ((z > 1.0) & (z < 60.0) & (np.abs(px[:, 0] - cx) < hw) & (np.abs(px[:, 1] - cy) < hh))
        if mask is not None and m.any():
            bm, mx0, my0 = mask
            iy = np.clip((px[m, 1] - my0).astype(int), 0, bm.shape[0] - 1)
            ix = np.clip((px[m, 0] - mx0).astype(int), 0, bm.shape[1] - 1)
            keep = bm[iy, ix] > 0
            mm = m.copy(); mm[m] = keep
            if mm.sum() >= 3:
                m = mm                                     # fall back to the window if mask too sparse
        if m.sum() < 3:
            return None
        zz = z[m]
        z_front = np.percentile(zz, 25)                   # nearest cluster beats the background wall
        sel = m.copy(); sel[m] &= np.abs(zz - z_front) < 1.5
        if sel.sum() < 3:
            return None
        return np.median(pw[sel], axis=0)

class RadarVel:
    """RADAR_FRONT -> ego-motion-compensated DIRECT velocity per return, world frame (M1c radar port;
    07-14 audit: vx_comp/vy_comp noise 0.1-0.4 m/s, ZERO lag -- treats 'velocity by position
    differencing' at the root). World-frame gate match around a track's position; the median of the
    matched returns feeds MoverTracker.update_velocity (H=[0,1,0] same-instant fusion)."""

    _DT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("dyn_prop", "i1"), ("id", "<i2"),
                    ("rcs", "<f4"), ("vx", "<f4"), ("vy", "<f4"), ("vx_comp", "<f4"), ("vy_comp", "<f4"),
                    ("q", "i1"), ("ambig", "i1"), ("x_rms", "i1"), ("y_rms", "i1"), ("invalid", "i1"),
                    ("pdh0", "i1"), ("vx_rms", "i1"), ("vy_rms", "i1")])

    def __init__(self, nusc):
        from kf_nusc import quat_rot as _qr
        self._qr = _qr
        self.nusc = nusc
        self.recs = sorted((d for d in nusc.sd if "/RADAR_FRONT/" in d["filename"]),
                           key=lambda d: d["timestamp"])
        self.ts = np.array([d["timestamp"] for d in self.recs])
        self._cache = (None, None)

    def _sweep(self, t_us):
        if not len(self.recs):
            return None
        i = int(np.clip(np.searchsorted(self.ts, t_us), 1, len(self.ts) - 1))
        rec = self.recs[i] if abs(self.ts[i] - t_us) < abs(self.ts[i - 1] - t_us) else self.recs[i - 1]
        if abs(rec["timestamp"] - t_us) > 0.25e6:          # no radar sweep within 0.25 s -> no fusion
            return None
        if self._cache[0] == rec["token"]:
            return self._cache[1]
        raw = open(os.path.join(DATA, rec["filename"]), "rb").read()
        k = raw.find(b"DATA binary\n")
        if k < 0:
            return None
        try:
            npts = int(raw[:k].split(b"POINTS")[1].split(b"\n")[0])
        except Exception as e:
            print(f"[radar] WARNING: bad PCD header {rec['filename']}: {e}", flush=True)
            return None
        body = raw[k + 12: k + 12 + npts * self._DT.itemsize]
        if len(body) < npts * self._DT.itemsize:
            print(f"[radar] WARNING: truncated PCD {rec['filename']}", flush=True)
            return None
        pts = np.frombuffer(body, dtype=self._DT)
        # validity (2026-07-17 review): devkit-default invalid_state==0 & ambig_state==3, plus
        # dyn_prop<=6 (7=undefined) and an rms-code sanity cut -- the PCD carries per-point
        # vx_rms/vy_rms QUALITY CODES; without the devkit LUT vendored we use them as a relative
        # filter (drop the worst codes), not as metric sigmas. COMPROMISE, documented.
        pts = pts[(pts["invalid"] == 0) & (pts["ambig"] == 3) & (pts["dyn_prop"] <= 6)
                  & (pts["vx_rms"] < 24) & (pts["vy_rms"] < 24)]
        cs = self.nusc.cs[rec["calibrated_sensor_token"]]
        ego = self.nusc.ego[rec["ego_pose_token"]]
        Rcs, Rego = self._qr(cs["rotation"]), self._qr(ego["rotation"])
        pw = np.c_[pts["x"], pts["y"], pts["z"]].astype(float) @ Rcs.T + np.asarray(cs["translation"])
        pw = pw @ Rego.T + np.asarray(ego["translation"])
        vs = np.c_[pts["vx_comp"], pts["vy_comp"], np.zeros(len(pts))].astype(float)
        vw = vs @ Rcs.T @ Rego.T                            # rotation only: velocities are vectors
        out = (pw[:, :2], vw[:, :2])
        self._cache = (rec["token"], out)
        return out

    def velocity_at(self, xy, t_us, gate_m=2.5):
        """(median world-xy velocity, n_matched) of returns within gate_m of xy, or None."""
        sw = self._sweep(t_us)
        if sw is None:
            return None
        pw, vw = sw
        d = np.hypot(pw[:, 0] - xy[0], pw[:, 1] - xy[1])
        m = d < gate_m
        if not m.any():
            return None
        v = np.median(vw[m], axis=0)
        if not np.isfinite(v).all() or np.hypot(v[0], v[1]) > 20.0:
            return None
        return v, int(m.sum())


# ---- the deployed v6.1 CAPSULE keep-out, THIN (oracle-arm) sizing, transplanted verbatim from
# render_3d_video._cap_ring so what we draw here IS the algorithm's avoidance body ----
# ILLUSTRATION ONLY on this face: the thin calibration was harvested on MetaUrban and carries no
# estimation-error budget; on monocular nuScenes it shows the deployed SIZE, it certifies nothing.
CAP_TAU, CAP_RDT = 0.75, 0.1            # trust window + replan_dt (metaurban_sando.yaml)
CAP_Q, CAP_VEFF, CAP_DSAFE, CAP_K = 0.05, 0.1, 0.15, 6      # calib_v6_thin.json + MANDSAFE=0.15
CAP_REAR = (0.05, 0.0)                  # v6.1 flat rear: rear-overrun quantile q0r + growth
# r_obs from the DETECTION, not a class constant (user 2026-07-14: fixed 1.0 m looked too wide).
# Pedestrian/cycle: half bbox-WIDTH (no side-length problem). Vehicle: bbox-width is the car's
# LENGTH in side view (first cut ballooned a sedan to r2.5) -> infer width from bbox HEIGHT, which
# is view-invariant (car ~1.45 m tall, ~1.8 m wide => half-width ~ 0.62 * height).
# Monocular honesty: a frozen side-view car's nose/tail poke
# out of the point-law circle -- on the deployment face r_obs comes from the sensor body, not this.
R_CLAMP = {"pedestrian": (0.20, 0.45), "cycle": (0.30, 0.80), "vehicle": (0.70, 1.40)}
R_EMA = 0.3
VMAX = {"pedestrian": 2.5, "cycle": 8.0, "vehicle": 15.0}   # class top speed: innovation gate + sp cap
# DATA-level smoothing (user 2026-07-14: smooth the real content, not the pixels):
# (a) the KF itself is the smoother -- q_jerk 2.0 was calibrated for MetaUrban's 0.07 m noise; this
#     face has 10x the noise and slower dynamics, so a lower process noise IS the honest smoother
#     (covariance/NIS semantics intact, unlike post-hoc EMA on observations);
# (b) v_feed = EMA of the latch window velocity -- the production vel_smooth/VF_EMA law verbatim:
#     smooth what the PLANNER eats (capsule geometry + eval), the certificate keeps raw state.
Q_JERK_FACE = 0.5
VFEED_EMA = 0.2


def capsule_ring(c0, vel, r_obs, vmax=None):
    """Ground outline of the deployed thin capsule: stadium from the mover's flat BACK to the
    KF-apex cap at c0 + v*tau. Verbatim geometry of render_3d_video._cap_ring (v6.1)."""
    sp = float(np.hypot(vel[0], vel[1]))
    if vmax is not None:
        sp = min(sp, float(vmax))          # a phantom 10 m/s must not fatten the pearl-gap term
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


SEG_W = "/media/boxuan/Data2/projects/cvmusecore/yolo11s-seg.pt"


def main(scene_names, render, use_lidar=False, det_mode="yolo", use_seg=False, gt_mask=False, use_radar=False):
    """det_mode: yolo (real detector) | gtbox (GT boxes projected to 2D, SAME sensing chain) |
    gt3d (oracle GT centres straight into the same KF/latch/capsule downstream)."""
    import cv2
    if det_mode == "yolo":
        from ultralytics import YOLO
    pfx = {"yolo": ("seg" if use_seg else "byte"),
           "gtbox": ("gtmask" if gt_mask else "gtbbx"), "gt3d": "gt"}[det_mode]
    nusc = Nusc()
    lidar = LidarDepth(nusc) if use_lidar else None
    rvel = RadarVel(nusc) if use_radar else None
    # gate must match estimation quality (the tdyn law): with lidar R but production q_jerk=2.0 the
    # sigma_v floor sits just above 0.5, and the [0.5,1.0) shadow bucket BEATS still (ped 2.02/0.83
    # vs 2.49/2.47, veh mean 4.07 vs 7.61 @2s) -> the lidar face earns a 1.0 gate. Monocular keeps 0.5.
    sigv_gate = {k: (1.0 if use_lidar else v) for k, v in SIGV_YOUNG.items()}
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
        model = YOLO(SEG_W if use_seg else WEIGHTS) if det_mode == "yolo" else None
        chain = nusc.sample_chain(scene)
        ts_key = {s["token"]: s["timestamp"] / 1e6 for s in chain}
        gt = {}
        for s in chain:
            for a in nusc.anns_by_sample.get(s["token"], []):
                if a["grp"]:
                    gt.setdefault(a["instance_token"], []).append((ts_key[s["token"]], a))
        gt = {k: sorted(v, key=lambda x: x[0]) for k, v in gt.items()}

        _iid = {}                                        # instance_token -> stable small int id

        def gt_full_at(inst, t):
            obs = gt[inst]
            if t < obs[0][0] - 1e-9 or t > obs[-1][0] + 1e-9:
                return None
            near = min(obs, key=lambda o: abs(o[0] - t))[1]
            for (t0, a0), (t1, a1) in zip(obs, obs[1:]):
                if t0 - 1e-9 <= t <= t1 + 1e-9:
                    wgt = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                    pp = (1 - wgt) * np.asarray(a0["translation"]) + wgt * np.asarray(a1["translation"])
                    return pp, near["size"], near["rotation"], near["grp"], int(near.get("visibility_token", 4))
            a = obs[-1][1]
            return (np.asarray(a["translation"]), a["size"], a["rotation"], a["grp"],
                    int(a.get("visibility_token", 4)))

        def gt_box2d(sdr, pp, size, rot):
            from kf_nusc import quat_rot as _qr
            wd, ln, ht = size
            R = _qr(rot)
            cor = np.array([[sx * ln / 2, sy * wd / 2, sz * ht / 2]
                            for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
            px, ok = world_to_px(np.asarray(pp)[None, :] + cor @ R.T, sdr, nusc)
            if ok.sum() < 4:
                return None
            px = px[ok]
            return float(px[:, 0].min()), float(px[:, 1].min()), float(px[:, 0].max()), float(px[:, 1].max())

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
        _bt = {"yolo": 0.0, "ours": 0.0, "draw": 0.0, "n": 0}   # BENCH=1 stage timers
        for sd_rec in frames[scene["name"]]:
            t_now = sd_rec["timestamp"] / 1e6
            img_path = os.path.join(DATA, sd_rec["filename"])
            _t0 = time.perf_counter()
            W, H = sd_rec.get("width", 1600), sd_rec.get("height", 900)
            cands = []                                   # (tid, grp, bbox, gt_p, gt_size)
            if det_mode == "yolo":
                res = model.track(img_path, persist=True, tracker="bytetrack.yaml",
                                  conf=0.1, verbose=False)[0]
                for bi, b in enumerate(res.boxes):
                    if b.id is None:
                        continue
                    grp = COCO2GRP.get(model.names[int(b.cls)])
                    if grp is None:
                        continue
                    x0, y0, x1, y1 = [float(v) for v in b.xyxy[0]]
                    if x0 < 4 or x1 > W - 4 or y1 > H - 4 or (y1 - y0) < 18:
                        continue
                    msk = None
                    if use_seg and res.masks is not None and bi < len(res.masks.xy):
                        poly = res.masks.xy[bi]
                        if poly is not None and len(poly):
                            bw, bh = int(x1 - x0) + 2, int(y1 - y0) + 2
                            bm = np.zeros((bh, bw), np.uint8)
                            cv2.fillPoly(bm, [np.round(poly - [x0, y0]).astype(np.int32)], 1)
                            msk = (bm, x0, y0)
                    cands.append((int(b.id), grp, (x0, y0, x1, y1), None, None, msk))
            else:                                        # GT twins: same downstream, perfect association
                for inst in gt:
                    st = gt_full_at(inst, t_now)
                    if st is None:
                        continue
                    pp, size, rot, grp, vis = st
                    bb = gt_box2d(sd_rec, pp, size, rot)
                    tid = _iid.setdefault(inst, len(_iid) + 1)
                    if det_mode == "gtbox":              # a PERFECT DETECTOR: what one could see
                        if vis < 2 or bb is None:
                            continue
                        x0 = max(bb[0], 0.0); y0 = max(bb[1], 0.0)
                        x1 = min(bb[2], W - 1.0); y1 = min(bb[3], H - 1.0)
                        if x0 < 4 or x1 > W - 4 or y1 > H - 4 or (y1 - y0) < 18:
                            continue
                        m3 = ("box3d", pp, size, rot) if gt_mask else None
                        cands.append((tid, grp, (x0, y0, x1, y1), None, None, m3))
                    else:                                # gt3d: the omniscient oracle
                        cands.append((tid, grp, bb, pp, size, None))
            _t1 = time.perf_counter()
            seen = set()
            for tid, grp, _bb, _gtp, _gtsz, _msk in cands:
                if _bb is not None:
                    x0, y0, x1, y1 = _bb
                else:
                    x0 = y0 = x1 = y1 = 0.0
                _w_pre = world.get(tid)
                if _gtp is not None:                  # gt3d: the oracle position, no sensing at all
                    g = np.asarray(_gtp, float)
                    by_lidar = False
                elif _msk is not None and isinstance(_msk, tuple) and len(_msk) == 4 and _msk[0] == "box3d":
                    g = lidar.locate_box3d(sd_rec["timestamp"], _msk[1], _msk[2], _msk[3]) if lidar else None
                    by_lidar = g is not None
                elif True:
                    g = lidar.locate((x0, y0, x1, y1), sd_rec, sd_rec["timestamp"], mask=_msk) if lidar else None
                    by_lidar = g is not None
                if _gtp is None and g is None:
                    # lidar dry (small far box / sparse returns) -> HEIGHT-PRIOR observation instead
                    # of flat-earth (whose slope bias parked #174's reborn capsule metres off) or
                    # blind coasting (whose stale velocity was the 88 m drift). Bearing exact, depth
                    # ~10%, R inflated to match -> the track stays anchored, no starvation, no
                    # euthanasia/rebirth churn.
                    g = locate_hprior((x0, y0, x1, y1), sd_rec, nusc, grp,
                                      H=_w_pre.get("H_cal") if _w_pre else None)
                if g is None:
                    continue
                ego_xy = np.asarray(nusc.ego[sd_rec["ego_pose_token"]]["translation"][:2])
                # NEAR-SURFACE -> CENTRE de-bias: any single-viewpoint sensor (lidar cluster, height
                # prior) ranges the VISIBLE FACE; the GT/visual centre sits half a body deeper (diag:
                # e_det flat at 2.2 m = half a car length -- the capsule looked pushed toward ego).
                # Shift along the viewing ray by half the class extent, view-angle aware.
                if _gtp is not None:
                    z_grd = float(g[2]) - 0.5 * float(_gtsz[2])
                _u = g[:2] - ego_xy
                _u = _u / max(1e-6, float(np.linalg.norm(_u)))
                if grp == "vehicle":
                    _v = _w_pre["trk"].state()[1][:2] if (_w_pre and _w_pre["trk"].ready) else None
                    if _v is not None and float(np.hypot(*_v)) > 1.0:
                        _c = abs(float(np.dot(_v / np.linalg.norm(_v), _u)))
                        _off = 2.2 * _c + 0.9 * (max(0.0, 1 - _c * _c)) ** 0.5
                    else:                                     # parked: wide box = side view
                        _off = 0.9 if (x1 - x0) > 1.8 * (y1 - y0) else 2.2
                elif grp == "cycle":
                    _off = 0.5
                else:
                    _off = 0.15
                if _gtp is None:
                    g = g + np.array([_u[0], _u[1], 0.0]) * _off
                # the object's LOCAL ground height (cluster mid-body z minus half height). The data
                # chain dropped flat-earth long ago but the DRAWING still painted rings at z=0 --
                # on this road (true ground up to +1 m) that pushed every ring toward the ego.
                if _gtp is None:
                    _H0 = (_w_pre.get("H_cal") if _w_pre else None) or H_PRIOR[grp]
                    z_grd = float(g[2]) - 0.5 * _H0
                d_ego = float(np.linalg.norm(g[:2] - ego_xy))
                if d_ego > RANGE_MAX:
                    continue
                K_f = float(np.asarray(nusc.cs[sd_rec["calibrated_sensor_token"]]["camera_intrinsic"])[0][0])
                lo_r, hi_r = R_CLAMP[grp]
                if _gtsz is not None:
                    r_det = float(np.clip(0.5 * float(_gtsz[0]), lo_r, hi_r))   # GT half-width
                else:
                    ext_px = 0.62 * (y1 - y0) if grp == "vehicle" else 0.5 * (x1 - x0)
                    r_det = float(np.clip(ext_px * d_ego / K_f, lo_r, hi_r))
                w = world.get(tid)
                if w is None:
                    w = world[tid] = {"trk": MoverTracker(dt=0.083, meas_noise=0.5),
                                      "t_last": None, "grp": grp, "hist": [], "bbox": None,
                                      "latch": MotionLatch(grp), "r_obs": r_det,
                                      "v_feed": np.zeros(2), "n_lidar": 0}
                    # q_jerk 0.5 was the MONOCULAR smoother (fat R, slow targets). On the lidar face
                    # it just adds manoeuvre lag -- keep the production 2.0 there.
                    for _ax in (w["trk"].fx, w["trk"].fy):
                        _ax.q_jerk = 2.0 if (lidar or det_mode == "gt3d") else Q_JERK_FACE
                        _ax.F, _ax.Q = _ax._mats(_ax.dt)
                trk = w["trk"]
                # SOURCE-STEP RE-INIT: a track born on flat-earth obs carries metre-level bias AND a
                # garbage two-point velocity (diag: sigv 44 -> coasted 88 m away). Its FIRST lidar fix
                # is the first honest data -> restart the filter and the latch there, don't drag.
                if by_lidar and w.get("n_lidar", 0) == 0 and w["t_last"] is not None:
                    w["trk"] = trk = MoverTracker(dt=0.083, meas_noise=0.5)
                    for _ax in (trk.fx, trk.fy):
                        _ax.q_jerk = 2.0
                        _ax.F, _ax.Q = _ax._mats(_ax.dt)
                    w["latch"] = MotionLatch(grp)
                    w["t_last"] = None; w["hist"] = []; w["v_feed"] = np.zeros(2)
                sig = (0.05 if _gtp is not None else
                       (0.15 + 0.01 * d_ego) if by_lidar else (0.4 + 0.12 * d_ego))
                trk.fx.R = trk.fy.R = sig * sig
                det = np.array([g[0], g[1], z_grd])
                # INNOVATION GATE vs the KF's CURRENT prediction (incl. coast), threshold carrying
                # pos_sigma -- a coasting track's gate SELF-REOPENS as P grows. The old form (det vs
                # stale hist[-1], no P term) was a rejection DEADLOCK: diag showed e_kf climbing
                # 11->53 m at constant e_det~5 while the gate never let go. 4 straight rejections
                # force a re-accept (memory-expiry backstop).
                if w["t_last"] is not None and w["hist"]:
                    dtj = max(1e-3, t_now - w["t_last"])
                    pred_xy = trk.predict([0.0], model="cv")[0][:2]
                    jump = float(np.linalg.norm(det[:2] - pred_xy))
                    if jump > VMAX[grp] * dtj + 3.0 * sig + trk.pos_sigma and w.get("n_rej", 0) < 4:
                        trk.coast(dt=dtj)                 # keep the state MOVING through the rejection
                        w["t_last"] = t_now
                        w["n_rej"] = w.get("n_rej", 0) + 1
                        w["bbox"] = (x0, y0, x1, y1)      # keep the box on screen, drop the sample
                        w["src"] = "gated"
                        seen.add(tid)
                        continue
                    w["n_rej"] = 0
                if w["t_last"] is None:
                    trk.update(det)
                else:
                    trk.update(det, dt=max(1e-3, t_now - w["t_last"]))
                w["src"] = "gt3d" if _gtp is not None else ("lidar" if by_lidar else "hprior")
                if rvel is not None:
                    # same-instant Doppler fusion; position gate widened by the track's own pos_sigma
                    # (a half-converged track's radar lives further from its estimate than 2.5 m),
                    # velocity INNOVATION gate inside update_velocity (3*sqrt(S)+0.5) rejects
                    # neighbour/clutter returns -- the double insurance of the 2026-07-17 review.
                    _rv = rvel.velocity_at(det[:2], sd_rec["timestamp"],
                                           gate_m=2.5 + min(float(trk.pos_sigma), 2.5))
                    if _rv is not None:
                        _vr, _nr = _rv
                        # r_vel: sensor floor 0.1-0.4 m/s; fewer matched returns -> trust less
                        if trk.update_velocity(_vr, r_vel=0.3 if _nr >= 3 else 0.5):
                            w["src"] += "+rad"
                w["t_last"] = t_now; w["hist"].append(det)
                w["bbox"] = (x0, y0, x1, y1) if _bb is not None else None
                w["msk"] = _msk
                w["r_obs"] = (1 - R_EMA) * w["r_obs"] + R_EMA * r_det
                w["latch"].add(t_now, det[:2], sig)
                if by_lidar:
                    w["n_lidar"] = w.get("n_lidar", 0) + 1
                    w["t_lid"] = t_now
                    # self-calibrated object height: laser depth x pixel height / f. This track now
                    # carries its own scale for the lidar-dry days (class prior = newborns only).
                    _Hn = d_ego * (y1 - y0) / K_f
                    if 0.8 < _Hn < 4.5:
                        w["H_cal"] = _Hn if "H_cal" not in w else 0.8 * w["H_cal"] + 0.2 * _Hn
                # planner-feed velocity: once sigma_v passes the production gate the KF's OWN velocity
                # is the freshest trustworthy source (the latch window velocity is ~1 s stale by
                # construction -- monocular-era medicine, the capsule-lag bug). Gate closed -> latch.
                if trk.ready and trk.sigma_v <= sigv_gate[grp]:
                    _c0, _v, _ = trk.state()
                    v_raw = _v[:2]
                else:
                    v_raw = w["latch"].vel_win if w["latch"].state == "MOVING" else np.zeros(2)
                w["v_feed"] = (1 - VFEED_EMA) * w["v_feed"] + VFEED_EMA * np.asarray(v_raw[:2], float)
                seen.add(tid)
            for tid, w in list(world.items()):
                if tid not in seen and w["t_last"] is not None:
                    if t_now - w["t_last"] > MISS_SEC:
                        del world[tid]

            if os.environ.get("DIAG") and world:
                if not hasattr(main, "_diag"):
                    main._diag = open(os.path.join(_HERE, "out", "diag.csv"), "w")
                    main._diag.write("scene,t,tid,grp,src,det_x,det_y,kf_x,kf_y,gt_x,gt_y,e_det,e_kf,sigv,possig,latch,nlid\n")
                for tid, w in world.items():
                    if tid not in seen or not w["trk"].ready or not w["hist"]:
                        continue
                    dxy = w["hist"][-1][:2]
                    kf_xy = w["trk"].state()[0][:2]
                    best, bxy = None, None
                    for k in gt:
                        gxy = gt_xy_at(k, t_now)
                        if gxy is None:
                            continue
                        dd = float(np.linalg.norm(gxy - dxy))
                        if best is None or dd < best:
                            best, bxy = dd, gxy
                    if bxy is None or best > 6.0:
                        continue
                    main._diag.write(f"{scene['name']},{t_now:.3f},{tid},{w['grp']},{w.get('src','?')},"
                                     f"{dxy[0]:.2f},{dxy[1]:.2f},{kf_xy[0]:.2f},{kf_xy[1]:.2f},"
                                     f"{bxy[0]:.2f},{bxy[1]:.2f},{best:.2f},"
                                     f"{float(np.linalg.norm(kf_xy - bxy)):.2f},"
                                     f"{w['trk'].sigma_v:.2f},{w['trk'].pos_sigma:.2f},"
                                     f"{w['latch'].state},{w.get('n_lidar',0)}\n")
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
                            pxy = obs_xy + w["v_feed"] * h          # the planner-feed velocity (shadow record)
                        else:
                            pxy = obs_xy
                        rows.append((scene["name"], w["grp"], h,
                                     float(np.linalg.norm(pxy - gxy)),
                                     float(np.linalg.norm(obs_xy - gxy)),
                                     float(np.linalg.norm(raw - gxy)),
                                     float(w["trk"].sigma_v), st,
                                     ATTR_MOTION.get(gt_attr_at(inst, t_now), "-")))

            _t2 = time.perf_counter()
            _bt["yolo"] += _t1 - _t0; _bt["ours"] += _t2 - _t1; _bt["n"] += 1
            if render:
                img = cv2.imread(img_path)
                for tid, w in world.items():
                    if tid not in seen:
                        continue
                    if w.get("msk") is not None and len(w["msk"]) == 3:   # bitmap silhouette only
                        bm, mx0, my0 = w["msk"]
                        x0i, y0i = max(0, int(mx0)), max(0, int(my0))
                        y1i = min(img.shape[0], y0i + bm.shape[0])
                        x1i = min(img.shape[1], x0i + bm.shape[1])
                        if y1i > y0i and x1i > x0i:
                            sub = bm[:y1i - y0i, :x1i - x0i] > 0
                            roi = img[y0i:y1i, x0i:x1i]
                            cc = np.array(col[w["grp"]], np.float32)
                            roi[sub] = (0.55 * roi[sub] + 0.45 * cc).astype(np.uint8)
                    trk = w["trk"]
                    frz = (not trk.ready) or trk.sigma_v > sigv_gate[w["grp"]]
                    # colour hysteresis only -- everything drawn below IS the data (smoothing now
                    # lives in the data itself: Q_JERK_FACE in the KF + v_feed for the planner side).
                    D = w.setdefault("draw", {"frz": frz, "nflip": 0})
                    if frz != D["frz"]:
                        D["nflip"] += 1
                        if D["nflip"] >= 4:                       # colour flips only after 4 agreeing frames
                            D["frz"], D["nflip"] = frz, 0
                    else:
                        D["nflip"] = 0
                    frz = D["frz"]
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
                    zg = float(c0[2])                       # tracker z = the object's local ground
                    ctr_px, okc = world_to_px(np.array([[c0[0], c0[1], zg]]), sd_rec, nusc)
                    if okc[0]:
                        cx, cy = ctr_px[0].astype(int)
                        cv2.drawMarker(img, (cx, cy), c, cv2.MARKER_TILTED_CROSS, 14, 2)  # KF centre
                    sig_p = min(trk.pos_sigma, 6.0)
                    ang = np.linspace(0, 2 * np.pi, 17)
                    ring = np.stack([c0[0] + sig_p * np.cos(ang), c0[1] + sig_p * np.sin(ang),
                                     np.full_like(ang, zg)], axis=1)
                    px_r, okr = world_to_px(ring, sd_rec, nusc)
                    pr = px_r[okr].astype(int)
                    for p0, p1 in zip(pr, pr[1:]):                       # 1-sigma position ring
                        cv2.line(img, tuple(p0), tuple(p1), c, 1, cv2.LINE_AA)
                    # --- the algorithm's avoidance body: deployed THIN capsule (cyan) ---
                    ring_c, rad_c = capsule_ring(c0, w["v_feed"], w["r_obs"], vmax=VMAX[w["grp"]])
                    ring3 = np.array([[px_, py_, zg] for px_, py_ in ring_c])
                    px_cap, okcap = world_to_px(ring3, sd_rec, nusc)
                    pc = px_cap[okcap].astype(int)
                    for p0, p1 in zip(pc, pc[1:]):
                        cv2.line(img, tuple(p0), tuple(p1), (255, 255, 0), 2, cv2.LINE_AA)
                    if okc[0]:
                        cv2.putText(img, f"r{rad_c:.2f}", (cx + 8, cy + 16),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)
                    if not frz:
                        v = np.array([w["v_feed"][0], w["v_feed"][1], 0.0])   # arrow shows the planner feed
                    tip = np.array([[c0[0] + v[0], c0[1] + v[1], zg]])  # velocity arrow (1 s)
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
                _src_tag = {"yolo": ("YOLO11s-SEG+ByteTrack@12Hz mask-select" if use_seg else "YOLO26s+ByteTrack@12Hz"),
                            "gtbox": "GT-BBOX (perfect detector, same chain)",
                            "gt3d": "GT-3D ORACLE"}[det_mode]
                cv2.putText(img, f"{scene['name']} {_src_tag} -> KF | x=centre ring=1sig arrow=v*1s grey=FRZ cyan=THIN capsule", (16, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
                if vw is None:
                    vw = cv2.VideoWriter(os.path.join(out_dir, f"{pfx}_{scene['name']}.mp4"),
                                         cv2.VideoWriter_fourcc(*"mp4v"), 12,
                                         (img.shape[1], img.shape[0]))
                vw.write(img)
                _bt["draw"] += time.perf_counter() - _t2
        if vw is not None:
            vw.release()
            print("wrote", os.path.join(out_dir, f"{pfx}_{scene['name']}.mp4"))
        if os.environ.get("BENCH") == "1" and _bt["n"]:
            n = _bt["n"]
            tot = _bt["yolo"] + _bt["ours"] + (_bt["draw"] if render else 0.0)
            print(f"[bench] {scene['name']} n={n}  yolo {1e3*_bt['yolo']/n:6.1f} ms"
                  f"  ours {1e3*_bt['ours']/n:6.1f} ms  draw {1e3*_bt['draw']/n:6.1f} ms"
                  f"  -> {n/tot:5.1f} fps (detect+track only: {n/_bt['yolo']:5.1f} fps)")

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
    ap.add_argument("--lidar", action="store_true", help="bbox depth from LIDAR_TOP (nearest cluster) instead of flat-ground back-projection")
    ap.add_argument("--det", type=str, default="yolo", choices=["yolo", "gtbox", "gt3d"],
                    help="detection source: yolo | gtbox (GT boxes, same sensing chain) | gt3d (oracle)")
    ap.add_argument("--seg", action="store_true", help="yolo11s-seg instance masks select the lidar points (silhouette instead of rectangle)")
    ap.add_argument("--gtmask", action="store_true", help="with --det gtbox: select lidar points by GT 3-D box membership (the selection CEILING)")
    ap.add_argument("--radar", action="store_true", help="fuse RADAR_FRONT ego-compensated Doppler as a DIRECT velocity observation (H=[0,1,0]) -- the M1c radar port")
    args = ap.parse_args()
    main(None if args.all else set(args.scenes.split(",")), not args.no_render, args.lidar, args.det, args.seg, args.gtmask,
         use_radar=args.radar)
