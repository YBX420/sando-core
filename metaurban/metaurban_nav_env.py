"""metaurban_nav_env — the SAME learned-planner benchmark arm as drone_nav_env, but the world is a
LIVE MetaUrban SidewalkDynamicMetaUrbanEnv instead of a frozen scenario_lib replay.

Design decision (塔菲大人 拍板 2026-07-05, Option A "忠于研究"):
  * obs / action / dynamics / reward / terminals are IDENTICAL to DroneNavEnv, so the same
    ShieldedEnv + CertShield + conformal tube apply UNCHANGED and results compare directly to v1/v2;
  * the only change is the mover source: each tick we read MetaUrban GROUND-TRUTH movers (pedestrians
    / vehicles / static) and run them through the SAME perception front-end (cone/occlusion/miss/KF);
  * MetaUrban renders NOTHING (use_render=False) — physics is CPU, GPU stays idle by design. The
    crash-mitigation for the flaky 5800X is n_envs=1 + CPU freq cap, NOT the GPU.

Interface parity (so train's ShieldedEnv wraps this with zero changes): exposes
  max_vel, max_acc, p, v, goal, _tracks() -> (ready_tracks, idx), and gym step/reset.

Run: metaurban conda env, PYTHONPATH + CWD = the metaurban repo (assets are CWD-relative), e.g.
  cd /media/boxuan/Data2/projects/metaurban
  PYTHONPATH=/media/boxuan/Data2/projects/metaurban \
    ~/miniconda3/envs/metaurban/bin/python <this dir>/metaurban_nav_env.py     # self-smoke
"""
import os
import numpy as np

from perception import PerceptionFrontEnd, PerceptCfg

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:
    import gym
    from gym import spaces

# --- parity constants with drone_nav_env -------------------------------------------------------
DT = 0.30
K_TRACKS = 6
CRUISE_Z = 1.5
SENSE_R = 16.0                     # cull movers beyond this (matches yaml sense_cull_r spirit)
MIN_GOAL = 50.0
CORRIDOR_LEN = MIN_GOAL + 6.0      # ~56 m corridor, comfortably >= 50 m
MU_SUBSTEPS = 3                    # MetaUrban advances ~0.1 s/step; 3x ~= DT=0.3 s crowd motion
REUSE_SCENE_EPISODES = 4           # drone episodes per MetaUrban asset load (amortise ~8 s reset)

# MetaUrban env config: copied verbatim from run_demo.py (the proven headless closed-loop config).
ENV_CFG = dict(
    crswalk_density=1, object_density=0.4, walk_on_all_regions=False,
    use_render=False, image_observation=False, manual_control=False,
    map=os.environ.get("MU_MAP", "X"),
    default_expert=False, drivable_area_extension=55, height_scale=1,
    show_mid_block_map=False, show_ego_navigation=False, debug=False, horizon=10000,
    on_continuous_line_done=False, out_of_route_done=False,
    vehicle_config=dict(show_lidar=False, show_navi_mark=False,
                        show_line_to_navi_mark=False, show_dest_mark=False, enable_reverse=True),
    show_sidewalk=True, show_crosswalk=True, random_spawn_lane_index=False,
    num_scenarios=20, accident_prob=0, relax_out_of_road_done=True, max_lateral_dist=1e3,
    crash_vehicle_done=False, crash_object_done=False, crash_human_done=False,
    traffic_density=0.25,
    spawn_human_num=18, spawn_wheelchairman_num=1, spawn_edog_num=2,
    spawn_erobot_num=1, spawn_drobot_num=1, max_actor_num=40,
)


def _classify(o):
    """MetaUrban object -> perception class string. Mirrors run_demo.classify (wheeled -> vehicle)."""
    from metaurban.component.agents.pedestrian.base_pedestrian import BasePedestrian
    from metaurban.component.delivery_robot.base_deliveryrobot import BaseDeliveryRobot
    from metaurban.component.robotdog.base_robotdog import BaseRobotDog
    from metaurban.component.vehicle.base_vehicle import BaseVehicle
    if isinstance(o, BasePedestrian):
        return "pedestrian"
    if isinstance(o, (BaseVehicle, BaseDeliveryRobot, BaseRobotDog)):
        return "vehicle"
    return "static"


def _obj_size(o):
    if all(hasattr(o, a) for a in ("WIDTH", "LENGTH", "HEIGHT")):
        try:
            return np.array([float(o.WIDTH), float(o.LENGTH), float(o.HEIGHT)], float)
        except Exception:
            pass
    return np.array([0.6, 0.6, 1.7], float)     # ped-ish default


_NAV_MOVER_ERR = {}   # 2026-07-16 sweep: fallbacks must be loud


class MetaUrbanNavEnv(gym.Env):
    metadata = {}

    def __init__(self, seed=0, max_vel=3.0, max_acc=6.0):
        from metaurban import SidewalkDynamicMetaUrbanEnv
        self.rng = np.random.default_rng(seed)
        self.max_vel, self.max_acc = float(max_vel), float(max_acc)
        self.observation_space = spaces.Box(-np.inf, np.inf, (4 + 5 * K_TRACKS,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (2,), np.float32)
        self._ep_len_max = 240
        self._mu = SidewalkDynamicMetaUrbanEnv(ENV_CFG)
        self._n_scen = int(ENV_CFG["num_scenarios"])       # valid MetaUrban seed range is [0, n_scen)
        self._needs_mu_reset = True                        # first ep must load a scene
        self._scene_uses = 0

    # ---- MetaUrban GT readout ---------------------------------------------
    def _native_movers(self):
        """[(xy_world(2,), r, h, cls)] for every non-ego MetaUrban object with a finite pose."""
        eng, ego = self._mu.engine, self._mu.agent
        out = []
        for _, o in eng.get_objects().items():
            if o is ego:
                continue
            try:
                pos = np.asarray(o.position, float)
                vel = np.asarray(o.velocity, float)
            except Exception as e:
                k = type(e).__name__
                _NAV_MOVER_ERR[k] = _NAV_MOVER_ERR.get(k, 0) + 1
                if _NAV_MOVER_ERR[k] == 1:
                    print(f"[nav] WARNING: dropping native mover, position/velocity read failed: {k}: {e} (first occurrence; counted silently after)", flush=True)
                continue
            if pos.shape[0] < 2 or not np.all(np.isfinite(pos)):
                continue
            size = _obj_size(o)
            r = 0.5 * float(max(size[0], size[1]))       # footprint radius
            out.append((pos[:2], r, float(size[2]), _classify(o),
                        vel[:2] if vel.shape[0] >= 2 else np.zeros(2)))
        return out

    def _cylinders(self):
        """org-relative (xy, r, h, cls) within SENSE_R of the drone, for the perception front-end."""
        cyl = []
        for xy_w, r, h, cls, _v in self._movers:
            xy = xy_w - self.org
            if np.linalg.norm(xy - self.p) <= SENSE_R:
                cyl.append((xy, r, h, cls))
        return cyl

    # ---- episode plumbing -------------------------------------------------
    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        # A full MetaUrban reset reloads assets (~8 s), so REUSE one loaded scene across several drone
        # episodes: only hard-reset when the scene ended or we've used it REUSE_SCENE_EPISODES times.
        # Between reuses the crowd keeps moving (a few warm-up steps) — cheap and keeps diversity live.
        if self._needs_mu_reset or self._scene_uses >= REUSE_SCENE_EPISODES:
            self._mu.reset(seed=int(self.rng.integers(self._n_scen)))
            self._needs_mu_reset = False
            self._scene_uses = 0
        self._scene_uses += 1
        for _ in range(10):                                # warm up so movers acquire velocity
            _o, _r, term, trunc, _i = self._mu.step([0.0, 0.0])
            if term or trunc:
                self._mu.reset(seed=int(self.rng.integers(self._n_scen)))
                self._scene_uses = 1
        self._movers = self._native_movers()

        # pick a >=50 m corridor centred on the pedestrian cluster, along its principal axis
        peds = np.array([xy for (xy, _r, _h, c, _v) in self._movers if c == "pedestrian"])
        if len(peds) < 3:
            peds = np.array([xy for (xy, _r, _h, c, _v) in self._movers
                             if c in ("pedestrian", "vehicle")]
                            or [np.asarray(self._mu.agent.position[:2], float)])
        ctr = peds.mean(axis=0)
        if len(peds) >= 3:
            _, _, vt = np.linalg.svd(peds - ctr)
            axis = vt[0] / (np.linalg.norm(vt[0]) + 1e-9)
        else:
            axis = np.array([1.0, 0.0])
        half = CORRIDOR_LEN / 2.0
        start_w = ctr - axis * half
        goal_w = ctr + axis * half

        # SPAWN LEGALITY (2026-07-08): the corridor endpoints can land INSIDE a static prop/building
        # (clearance<0 at t=0 -> insta-collision before any decision) -- walk each endpoint inward
        # along the corridor axis until it has breathing room. Benchmark-grade worlds must not
        # execute the drone at birth.
        def _clr_at(q):
            best = 1e18
            for xy_w, r, _h, _c, _v in self._movers:
                best = min(best, float(np.linalg.norm(xy_w - q)) - r - 0.25)
            return best
        for _sgn, _name in ((+1.0, "start"), (-1.0, "goal")):
            q = start_w if _name == "start" else goal_w
            step = axis * _sgn * 1.0
            for _ in range(12):
                if _clr_at(q) >= 0.6:
                    break
                q = q + step
            if _name == "start":
                start_w = q
            else:
                goal_w = q

        self.org = ctr.copy()
        self.p = start_w - self.org
        self.goal = goal_w - self.org
        self.v = np.zeros(2)
        self.tick = 0
        self.min_clr = 1e18
        self.pfe = PerceptionFrontEnd(PerceptCfg.from_env(dt=DT),
                                      seed=int(self.rng.integers(1 << 30)))
        self._hd = self.goal - self.p
        self._prev_d = float(np.linalg.norm(self.goal - self.p))
        self._mu_done = False
        return self._obs(), {}

    # ---- perception (parity with drone_nav_env) ---------------------------
    def _tracks(self):
        # ONE perception step per WORLD tick (FS3C-R prereq #12): the shield wrapper and _obs both
        # call _tracks; stepping the KF twice per DT fed it same-world-state update pairs booked
        # 0.3 s apart -> velocity estimates dragged toward HALF. Memoised on self.tick.
        if getattr(self, "_trk_tick", None) == self.tick:
            return self._trk_memo
        cyl = self._cylinders()
        hd = self._hd if np.linalg.norm(self._hd) > 0.2 else (self.goal - self.p)
        trs = self.pfe.step(self.p, hd, cyl)
        self._trk_tick, self._trk_memo = self.tick, ([t for t in trs if t.trk.ready], None)
        return self._trk_memo

    def _obs(self):
        trs, _ = self._tracks()
        d = self.goal - self.p
        dist = float(np.linalg.norm(d))
        o = [d[0] / max(dist, 1e-6), d[1] / max(dist, 1e-6), min(dist / 20.0, 2.0),
             float(np.linalg.norm(self.v)) / self.max_vel]
        trs = sorted(trs, key=lambda tr: np.linalg.norm(tr.xy - self.p))[:K_TRACKS]
        for tr in trs:
            c0, v0, _ = tr.trk.state()
            o += [(tr.xy[0] - self.p[0]) / 10.0, (tr.xy[1] - self.p[1]) / 10.0,
                  (v0[0] - self.v[0]) / 8.0, (v0[1] - self.v[1]) / 8.0, (tr.r + 0.3) / 2.0]
        o += [0.0] * (5 * (K_TRACKS - len(trs)))
        return np.asarray(o, np.float32)

    def _clearance(self):
        best, _ = self._clearance_culprit()
        return best

    def _clearance_culprit(self):
        """(clearance, culprit) where culprit = (xy_rel, r, cls) of the nearest-surface mover —
        the object actually hit on a collision tick (for attribution: coverage hole vs policy)."""
        best, culprit = 1e18, None
        for xy_w, r, _h, c, v in self._movers:
            cxy = xy_w - self.org
            clr = float(np.linalg.norm(cxy - self.p)) - r - 0.25
            if clr < best:
                best, culprit = clr, (cxy, r, c, v)
        return best, culprit

    def step(self, action):
        # advance the MetaUrban crowd by ~DT (physics is CPU; a single env ~= one core)
        for _ in range(MU_SUBSTEPS):
            _o, _r, term, trunc, _i = self._mu.step([0.0, 0.0])
            if term or trunc:
                self._mu_done = True
                self._needs_mu_reset = True                # scene ended -> next reset reloads
                break
        self._movers = self._native_movers()

        a = np.clip(np.asarray(action, float), -1, 1) * self.max_acc
        self.v = self.v + a * DT
        sp = float(np.linalg.norm(self.v))
        if sp > self.max_vel:
            self.v *= self.max_vel / sp
        if sp > 0.3:
            self._hd = self.v.copy()
        self.p = self.p + self.v * DT
        self.tick += 1

        clr, culprit = self._clearance_culprit()
        self.min_clr = min(self.min_clr, clr)
        d = float(np.linalg.norm(self.goal - self.p))
        r = 1.5 * (self._prev_d - d) - 0.02
        r -= 4.0 * float(np.log1p(np.exp(1.2 - clr)) - np.log1p(np.exp(1.2 - 3.0))) * 0.25
        self._prev_d = d
        terminated = truncated = False
        if clr < 0.0:
            r -= 60.0; terminated = True
        elif d < 0.6:
            r += 40.0; terminated = True
        elif self.tick >= self._ep_len_max or self._mu_done:
            truncated = True
        info = dict(min_clr=self.min_clr, reached=bool(d < 0.6), collided=bool(clr < 0.0))
        if clr < 0.0 and culprit is not None:
            # attribution: was the object we hit KNOWN to perception (ready track nearby) or a
            # coverage hole (never tracked / track dead)? Read live tracks WITHOUT advancing the KF.
            # bearing/in_cone split the holes further: "detected but not ready yet" (in cone) vs
            # "structurally blind" (outside the 45-deg half-angle cone = came from the side/behind).
            cxy, cr, ccls, cv = culprit
            ready = [t for t in self.pfe.tracks if t.trk.ready]
            dists = [float(np.linalg.norm(np.asarray(t.xy, float) - cxy)) for t in ready]
            dmin = min(dists) if dists else float("inf")
            rel = cxy - self.p
            hd = self._hd / max(float(np.linalg.norm(self._hd)), 1e-9)
            ang = float(np.degrees(np.arctan2(rel[1], rel[0]) - np.arctan2(hd[1], hd[0])))
            ang = (ang + 180.0) % 360.0 - 180.0
            info["culprit"] = dict(cls=ccls, r=round(float(cr), 2),
                                   in_track=bool(dmin < float(cr) + 0.75),
                                   track_dist=(round(dmin, 2) if np.isfinite(dmin) else None),
                                   n_ready=len(ready), n_live=len(self.pfe.tracks),
                                   bearing_deg=round(ang, 1), in_cone=bool(abs(ang) <= 45.0),
                                   speed=round(float(np.linalg.norm(np.asarray(cv, float))), 2))
        return self._obs(), float(r), terminated, truncated, info

    def close(self):
        try:
            self._mu.close()
        except Exception:
            pass


if __name__ == "__main__":
    import time
    t0 = time.time()
    env = MetaUrbanNavEnv(seed=7)
    print(f"[smoke] env built +{time.time()-t0:.1f}s", flush=True)
    for epi in range(2):
        obs, _ = env.reset()
        assert obs.shape == env.observation_space.shape, obs.shape
        R = 0.0
        for k in range(env._ep_len_max):
            d = env.goal - env.p
            a = d / max(np.linalg.norm(d), 1e-6)          # naive go-to-goal smoke policy
            obs, rew, term, trunc, info = env.step(a)
            R += rew
            if term or trunc:
                break
        print(f"[smoke] ep{epi} end@{env.tick} reached={info['reached']} collided={info['collided']} "
              f"min_clr={info['min_clr']:.2f} return={R:.1f} movers={len(env._movers)} "
              f"+{time.time()-t0:.1f}s", flush=True)
    env.close()
    print(f"[smoke] OK +{time.time()-t0:.1f}s", flush=True)
