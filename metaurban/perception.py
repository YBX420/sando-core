"""perception — realistic mover-perception front-end (the de-GT-ification of the harness).

The audit finding this replaces: every harness perceived movers as det = GT_position + Gaussian,
with visibility decided ON the GT position, no occlusion, no missed detections, no track identity
problem (trackers keyed by GT mover index = perfect association). That makes all mover headlines
"GT-ish". This module is the ONE shared front-end (replay_core first, render_3d_video next — never
two drifting copies again):

  observe():  FOV cone (half-angle + range)  ->  hard occlusion (2-D ray vs other cylinders)
              ->  distance-dependent miss  P_miss(d) = p0 + p1*(d/R)^2
              ->  distance-dependent noise sigma(d) = s0 + s1*d
import os
  step():     observe + greedy nearest-neighbour association (NO GT identity) -> per-track KF
              (kf_tracker.MoverTracker: two-point init + coast) with birth / coast / kill.

Deliberate honesty notes:
  * the cone/occlusion GEOMETRY is evaluated on GT positions — that is the sensor model (physics
    decides visibility), not a leak; what the estimator sees is only the noisy surviving detections.
  * classification is passed through (cls/r/h from the last matched detection) — classifier errors
    are a later dial, size estimation error likewise.
  * association is greedy NN with a gate: crossing movers CAN swap identities, occlusion CAN split
    one mover into two tracks over time. That is the point.
Config via env (PERCEPT_*) so scenarios/manifests can pin it. PERCEPT=gt keeps the old omniscient
path (control condition; still the default in replay_core until the B-bucket recalibration lands —
never silently change what a headline number means).
"""
import math
import os

import numpy as np

from kf_tracker import MoverTracker


class PerceptCfg:
    """All dials in one bag; from_env() reads PERCEPT_* overrides."""

    def __init__(self, fov_deg=45.0, fov_range=10.0, sigma0=0.05, sigma_k=0.01,
                 p_miss0=0.05, p_miss_k=0.15, occlusion=True, gate_m=1.2, ttl_ticks=8, dt=0.1,
                 fp_rate=0.0, cls_err=0.0, size_err=0.05):
        self.fov_deg = float(fov_deg)          # cone HALF-angle (matches render_3d_video --fov_deg)
        self.fov_range = float(fov_range)      # detection range (m)
        self.sigma0 = float(sigma0)            # centre-position noise at 0 m
        self.sigma_k = float(sigma_k)          # noise growth per metre
        self.p_miss0 = float(p_miss0)          # missed-detection prob at 0 m
        self.p_miss_k = float(p_miss_k)        # + p_miss_k * (d/range)^2
        self.occlusion = bool(int(occlusion))  # hard 2-D ray occlusion by other cylinders
        self.gate_m = float(gate_m)            # association gate (m) around predicted track position
        self.ttl_ticks = int(ttl_ticks)        # coast this many missed ticks, then kill the track
        self.dt = float(dt)
        self.fp_rate = float(fp_rate)          # Poisson clutter detections per tick, uniform in the cone (0=off)
        self.cls_err = float(cls_err)          # P(class label flips to another class) per detection (0=off)
        self.size_err = float(size_err)        # multiplicative log-normal-ish error on estimated r/h (0.05=5%)
        self.birth_vmax = 10.0                 # m/s: a 1-detection track has NO velocity estimate yet, so its
                                               # association gate must cover birth_vmax*dt of unmodelled motion
                                               # (a fixed gate_m never re-associates a fast mover at coarse dt:
                                               # 8 m/s * 0.3 s = 2.4 m > 1.2 m -> endless newborn ghost tracks)

    @classmethod
    def from_env(cls, dt=0.1):
        e = os.environ.get
        return cls(fov_deg=e("PERCEPT_FOV_DEG", 45.0), fov_range=e("PERCEPT_RANGE", 10.0),
                   sigma0=e("PERCEPT_SIGMA0", 0.05), sigma_k=e("PERCEPT_SIGMA_K", 0.01),
                   p_miss0=e("PERCEPT_PMISS0", 0.05), p_miss_k=e("PERCEPT_PMISS_K", 0.15),
                   occlusion=e("PERCEPT_OCCLUSION", 1), gate_m=e("PERCEPT_GATE", 1.2),
                   ttl_ticks=e("PERCEPT_TTL", 8), dt=dt,
                   fp_rate=e("PERCEPT_FP_RATE", 0.0), cls_err=e("PERCEPT_CLS_ERR", 0.0),
                   size_err=e("PERCEPT_SIZE_ERR", 0.05))

    @property
    def birth_gate(self):
        return self.gate_m + self.birth_vmax * self.dt


def _seg_blocks(p, q, c, r):
    """True if the 2-D segment p->q passes through the circle (c, r) STRICTLY between the endpoints
    (a blocker behind the target or behind the sensor does not occlude)."""
    d = q - p
    L2 = float(d @ d)
    if L2 < 1e-12:
        return False
    u = float((c - p) @ d) / L2
    if u <= 0.02 or u >= 0.98:                 # endpoint slack: the target/sensor own discs don't block
        return False
    closest = p + u * d
    return float(np.hypot(*(c - closest))) < r


class Track:
    """One perceived mover: KF + last-matched detection attributes. NO GT identity anywhere."""
    _next_id = 0

    def __init__(self, det, cfg):
        self.id = Track._next_id; Track._next_id += 1
        self.trk = MoverTracker(dt=cfg.dt, meas_noise=max(cfg.sigma0 + cfg.sigma_k * 5.0, 0.05))
        self.cls = det["cls"]; self.r = det["r"]; self.h = det["h"]
        self.xy = np.asarray(det["xy"], float)
        self.miss = 0
        self.trk.update([det["xy"][0], det["xy"][1], 1.5])

    def predicted_xy(self, dt=None):
        c0, v, _ = self.trk.state()
        step = float(dt) if dt is not None else self.trk.fx.dt
        return np.array([c0[0] + v[0] * step, c0[1] + v[1] * step])

    def update(self, det, dt=None):
        self.cls = det["cls"]; self.r = det["r"]; self.h = det["h"]
        self.xy = np.asarray(det["xy"], float)
        self.miss = 0
        self.trk.update([det["xy"][0], det["xy"][1], 1.5], dt)

    def coast(self, dt=None):
        if self.cls == "static":                       # mapped static: frozen, no covariance growth
            self.miss += 1
            return
        self.trk.coast(dt); self.miss += 1
        c0, _, _ = self.trk.state()
        self.xy = np.asarray(c0[:2], float)


class PerceptionFrontEnd:
    def __init__(self, cfg=None, seed=0):
        self.cfg = cfg or PerceptCfg()
        self.rng = np.random.default_rng(seed)
        self.tracks = []

    # ---- sensor -------------------------------------------------------------
    def observe(self, p_xy, heading_xy, cylinders):
        """cylinders: [(xy(2,), r, h, cls)] GT world state. -> list of detection dicts that SURVIVE
        the cone, occlusion and miss draws, with distance-scaled noise applied."""
        cfg = self.cfg
        p = np.asarray(p_xy, float)
        hd = np.asarray(heading_xy, float)
        n = float(np.hypot(*hd)); hd = hd / n if n > 1e-9 else np.array([1.0, 0.0])
        cos_lim = math.cos(math.radians(cfg.fov_deg))
        out = []
        for k, (xy, r, h, cls) in enumerate(cylinders):
            rel = np.asarray(xy, float) - p
            d = float(np.hypot(*rel))
            if d < 1e-9:
                continue
            # SURFACE-honest visibility (2026-07-04): a depth camera sees the NEAREST SURFACE, not
            # the centroid -- for a large-footprint object (tree canopy r=4.7) the centre can sit
            # outside the cone while the canopy edge is dead ahead (props_alley v=5 certified crash:
            # corridor cuts 2.3m INTO an untracked canopy). Gate on the closest surface point; the
            # reported detection stays the centre (estimator contract unchanged).
            d_surf = max(d - float(r), 0.0)
            if d_surf > cfg.fov_range:
                continue
            rel_s = rel * (d_surf / d) if d_surf > 1e-9 else rel * 0.0
            surf = rel_s if d_surf > 1e-9 else rel * (0.05 / d)
            ds = float(np.hypot(*surf)) if d_surf > 1e-9 else 0.05
            if d_surf > 1e-9 and float(surf @ hd) / ds < cos_lim and float(rel @ hd) / d < cos_lim:
                continue                                    # neither surface point nor centre in cone
            # SILHOUETTE visibility: a depth camera sees the whole visible ARC, not one ray -- a
            # single surface ray deterministically blocked by foreground props kept a 4.7m canopy
            # invisible through entire flights (props_alley: tree tracked 0 ticks, certified crash).
            # Sample 5 points on the near arc; the object is visible if ANY is in-cone + unoccluded.
            if cfg.occlusion:
                u = rel / d
                tang = np.array([-u[1], u[0]])
                vis = False
                for a in (0.0, 0.45, -0.45, 0.85, -0.85):
                    pt = np.asarray(xy, float) - u * float(r) * float(np.cos(a))                         + tang * float(r) * float(np.sin(a))
                    rp = pt - p
                    dp = float(np.hypot(*rp))
                    if dp < 1e-9 or dp > cfg.fov_range or float(rp @ hd) / dp < cos_lim:
                        continue
                    if any(_seg_blocks(p, pt, np.asarray(oxy, float), orr)
                           for j, (oxy, orr, _, _) in enumerate(cylinders) if j != k):
                        continue
                    vis = True
                    break
                if not vis:
                    continue                                # entire visible arc blocked / out of cone
            d_eff = min(d, d_surf + float(r) * 0.0 + max(d_surf, 0.05))   # miss prob by surface range
            if self.rng.uniform() < min(0.95, cfg.p_miss0 + cfg.p_miss_k * (max(d_surf, 0.05) / cfg.fov_range) ** 2):
                continue                                    # missed this tick
            sig = cfg.sigma0 + cfg.sigma_k * d
            cls_out = str(cls)
            if cfg.cls_err > 0 and self.rng.uniform() < cfg.cls_err:      # classifier flip
                others = [c for c in ("pedestrian", "vehicle", "animal") if c != cls_out]
                cls_out = others[self.rng.integers(len(others))]
            se = cfg.size_err
            r_est = float(r) * float(np.exp(self.rng.normal(0.0, se))) if se > 0 else float(r)
            h_est = float(h) * float(np.exp(self.rng.normal(0.0, se))) if se > 0 else float(h)
            out.append(dict(xy=np.asarray(xy, float) + self.rng.normal(0.0, sig, 2),
                            r=r_est, h=h_est, cls=cls_out))
        # clutter: Poisson false positives, area-uniform inside the cone (small ped-ish blobs)
        if cfg.fp_rate > 0:
            for _ in range(self.rng.poisson(cfg.fp_rate)):
                d = cfg.fov_range * math.sqrt(self.rng.uniform())
                a = math.radians(cfg.fov_deg) * self.rng.uniform(-1.0, 1.0)
                base = math.atan2(hd[1], hd[0]) + a
                out.append(dict(xy=p + d * np.array([math.cos(base), math.sin(base)]),
                                r=0.3, h=1.7, cls="pedestrian"))
        return out

    # ---- track management ---------------------------------------------------
    def step(self, p_xy, heading_xy, cylinders, dt=None):
        """One perception tick: observe + associate + update/coast/kill. Returns live tracks.
        dt: actual tick duration (None = nominal cfg.dt). Proximity-triggered cadence passes the
        sub-tick duration here so KF propagation, association look-ahead and the birth gate all
        use the TRUE elapsed time (re-anchoring without honest dt would fake-shrink the tube)."""
        dets = self.observe(p_xy, heading_xy, cylinders)
        unmatched = list(range(len(dets)))
        # greedy NN: repeatedly take the globally closest (track, det) pair inside the gate
        pred = {tr.id: tr.predicted_xy(dt) for tr in self.tracks}
        pairs = sorted(((float(np.hypot(*(pred[tr.id] - dets[di]["xy"]))), ti, di)
                        for ti, tr in enumerate(self.tracks) for di in unmatched), key=lambda x: x[0])
        used_t, used_d = set(), set()
        for dist, ti, di in pairs:
            # ready tracks predict their motion -> tight gate; a 1-detection track has v=0 by construction,
            # so it gets the wide birth gate (covers birth_vmax*dt) or fast movers never re-associate.
            step_dt = float(dt) if dt is not None else self.cfg.dt
            gate = self.cfg.gate_m if self.tracks[ti].trk.ready else \
                (self.cfg.gate_m + self.cfg.birth_vmax * step_dt)   # birth gate covers TRUE unmodelled motion
            _kg = float(os.environ.get("ASSOC_KGATE", "0"))
            if _kg > 0.0 and self.tracks[ti].trk.ready:
                # convergence-aware gate (chi2-gating spirit): open by the track's OWN filter position
                # uncertainty -- a half-converged fast mover that coasted once has pos_sigma of metres
                # (fixed 1.2m gate = death spiral: bad prediction -> assoc fail -> more coast -> worse);
                # a converged track keeps sigma~0.1 so the gate stays tight (no new ID-swap risk).
                gate += _kg * float(self.tracks[ti].trk.pos_sigma)
            if dist > gate or ti in used_t or di in used_d:
                continue
            self.tracks[ti].update(dets[di], dt)
            used_t.add(ti); used_d.add(di)
        for ti, tr in enumerate(self.tracks):
            if ti not in used_t:
                tr.coast(dt)
        for di in range(len(dets)):
            if di not in used_d:
                self.tracks.append(Track(dets[di], self.cfg))
        # ONLINE-MAPPING semantics (real deployment: no prior map, the map is BUILT): a track whose
        # class is static is a MAP observation, not a mover hypothesis -- once seen it persists
        # forever at its frozen position (occlusion must not erase a tree you already mapped).
        # Mover tracks keep the usual coast/TTL decay.
        for tr in self.tracks:
            if tr.cls == "static" and tr.miss > 0:
                tr.miss = min(tr.miss, self.cfg.ttl_ticks)   # never expire; freeze instead of coast
                c0, _v, _a = tr.trk.state()
                tr.xy = np.asarray(c0[:2], float)
        self.tracks = [tr for tr in self.tracks
                       if tr.cls == "static" or tr.miss <= self.cfg.ttl_ticks]
        return self.tracks


# ---------------------------------------------------------------- self-test
if __name__ == "__main__":
    # occlusion_reveal in miniature: a wall of cylinders hides a burster until it clears the wall.
    cfg = PerceptCfg(p_miss0=0.0, p_miss_k=0.0, sigma0=0.02, sigma_k=0.0)   # deterministic visibility
    pf = PerceptionFrontEnd(cfg, seed=1)
    wall = [((8.0, 1.2), 0.45, 3.2, "static"), ((8.0, 2.1), 0.45, 3.2, "static")]
    drone = np.array([2.0, 0.0]); heading = (1.0, 0.0)
    seen_at = None
    for k in range(30):
        y = 1.4 - 0.1 * k                                   # burster walks -y from behind the wall
        cyls = wall + [((8.45, y), 0.3, 1.8, "pedestrian")]
        tracks = pf.step(drone, heading, cyls)
        ped = [t for t in tracks if t.cls == "pedestrian"]
        if ped and seen_at is None:
            seen_at = y
    assert seen_at is not None and seen_at < 1.0, f"burster should be revealed only below the wall (got {seen_at})"
    print(f"[percept] occlusion: burster first tracked at y={seen_at:.2f} (<1.0 = hidden while behind wall) OK")

    # association without GT identity: two parallel walkers must stay two tracks
    pf2 = PerceptionFrontEnd(PerceptCfg(p_miss0=0.0, p_miss_k=0.0, occlusion=False), seed=2)
    for k in range(40):
        cyls = [((5.0 + 0.1 * k, 1.5), 0.3, 1.8, "pedestrian"), ((5.0 + 0.1 * k, -1.5), 0.3, 1.8, "pedestrian")]
        tracks = pf2.step((0.0, 0.0), (1.0, 0.0), cyls)
    assert len(tracks) == 2, f"expected 2 stable tracks, got {len(tracks)}"
    v = tracks[0].trk.state()[1]
    assert abs(v[0] - 1.0) < 0.4, f"track velocity off after 40 ticks: {v}"  # two-point sigma_v~0.28 + CA transient
    print(f"[percept] association: 2 walkers -> 2 tracks, v_x={v[0]:.2f} (truth 1.0) OK")

    # miss + TTL: a mover that leaves the cone coasts then dies
    pf3 = PerceptionFrontEnd(PerceptCfg(p_miss0=0.0, p_miss_k=0.0, occlusion=False, ttl_ticks=5), seed=3)
    for k in range(10):
        pf3.step((0.0, 0.0), (1.0, 0.0), [((4.0, 0.0), 0.3, 1.8, "pedestrian")])
    assert len(pf3.tracks) == 1
    for k in range(4):
        pf3.step((0.0, 0.0), (1.0, 0.0), [])                # gone
    assert len(pf3.tracks) == 1 and pf3.tracks[0].miss == 4, "should be coasting"
    for k in range(3):
        pf3.step((0.0, 0.0), (1.0, 0.0), [])
    assert len(pf3.tracks) == 0, "track should be killed after TTL"
    print("[percept] coast/TTL lifecycle OK")
    print("[percept] ALL PASS")
