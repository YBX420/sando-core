"""d435i_sensor — an Intel RealSense D435i depth camera as the drone's perception INPUT in MetaUrban/panda3d.

Replaces the GT-omniscient fov_cloud (which voxelises KNOWN object boxes inside a cone) with a REAL rendered
depth image from a D435i-spec camera mounted on the drone, deprojected to a world-frame point cloud. This is
sim2real-faithful: the planner now sees only FRONT SURFACES (occlusion), inside the true depth frustum (FOV +
range), with D435i depth noise -- exactly what an onboard D435i delivers. The output is the SAME (N,3) world
cloud fov_cloud returns, so it drops straight into ego.update_cloud(cloud, p_drone).

When a real D435i is connected, swap capture() for a pyrealsense2 frame -> the same deproject()/cloud path
consumes it (the intrinsics below are the D435i's), so this is the sim half of one sim2real interface.

D435i depth module (datasheet): depth FOV 87 deg x 58 deg, range ~0.3-10 m (best <~3 m), stereo depth noise
~< 2% of range. We model FOV + range gating + an axial gaussian noise ~ k*z^2.
"""
import os, math
import numpy as np

# ---- D435i depth intrinsics / limits (datasheet) ----
D435I_HFOV_DEG = 87.0     # horizontal depth FOV
D435I_VFOV_DEG = 58.0     # vertical depth FOV
D435I_ZMIN = 0.3          # min reliable depth (m)
D435I_ZMAX = 10.0         # max range (m); useful planning range smaller
D435I_NOISE_K = 0.01      # axial noise std ~ K * z^2 (m) -> ~1% at 1 m, grows with distance (stereo triangulation)
NEAR, FAR = 0.1, 40.0     # panda3d depth-buffer near/far planes used to linearise the depth texture


class D435iDepth:
    """Mounts a MetaUrban DepthCamera with D435i FOV on an object and returns a deprojected world point cloud."""

    def __init__(self, engine, width=160, height=106, zmax=8.0, noise=True, sensor_name="d435i_depth"):
        # 160x106 keeps the D435i 87:58 aspect (~3:2) while staying light enough for EGO's grid each tick.
        # The DepthCamera must be registered at env-construction time:
        #   sensors=dict(..., d435i_depth=(DepthCamera, 160, 106))
        # (MetaUrban builds sensors from the config; there is no runtime engine.add_sensor.)
        self.engine = engine
        self.W, self.H = int(width), int(height)
        self.zmax = float(zmax)
        self.noise = bool(noise)
        self.cam = engine.get_sensor(sensor_name)
        # set the lens to the D435i depth FOV + near/far for a clean linearisation
        lens = self.cam.get_lens()
        lens.setFov(D435I_HFOV_DEG, D435I_VFOV_DEG)
        lens.setNearFar(NEAR, FAR)
        self._tanx = math.tan(math.radians(D435I_HFOV_DEG) * 0.5)
        self._tany = math.tan(math.radians(D435I_VFOV_DEG) * 0.5)
        # precompute per-pixel normalized image coords in [-1,1] (camera looks +Y in panda3d; +X right, +Z up)
        us = (np.arange(self.W) + 0.5) / self.W * 2.0 - 1.0      # left(-1)->right(+1)
        vs = (np.arange(self.H) + 0.5) / self.H * 2.0 - 1.0      # bottom(-1)->top(+1)
        self._ndc_x, self._ndc_y = np.meshgrid(us, vs)           # (H,W)

    def _linearise(self, depth_buf):
        """OpenGL [0,1] window-depth -> metric eye-space depth (distance along the view axis)."""
        z = depth_buf.astype(np.float64).reshape(self.H, self.W)
        # z_eye = n*f / (f - z*(f-n))   (standard perspective depth inversion)
        denom = (FAR - z * (FAR - NEAR))
        with np.errstate(divide="ignore", invalid="ignore"):
            z_eye = (NEAR * FAR) / denom
        return z_eye

    def capture(self, parent_node, position, hpr):
        """Render the depth buffer from (parent_node, position, hpr); return metric eye-depth (H,W)."""
        buf = self.cam.perceive(to_float=True, new_parent_node=parent_node, position=position, hpr=hpr)
        return self._linearise(buf)

    def deproject(self, z_eye):
        """Metric eye-depth (H,W) -> camera-frame points (M,3) (panda3d frame: X right, Y forward, Z up).
        Gates by [zmin, zmax] (D435i range) before deprojecting -> background sky/floor-at-infinity dropped."""
        valid = np.isfinite(z_eye) & (z_eye >= D435I_ZMIN) & (z_eye <= self.zmax)
        z = z_eye[valid]
        if self.noise and z.size:
            # axial gaussian noise growing with z^2 (stereo). deterministic-enough; seeded by caller's RNG if any.
            z = z + np.random.normal(0.0, 1.0, z.shape) * (D435I_NOISE_K * z * z)
        ndcx = self._ndc_x[valid]; ndcy = self._ndc_y[valid]
        # point = z * (ndc_x*tanx, 1, ndc_y*tany)  (Y is the view/forward axis)
        X = z * ndcx * self._tanx
        Y = z
        Z = z * ndcy * self._tany
        return np.column_stack([X, Y, Z])

    def cloud_world(self, parent_node, position, hpr, subsample=1, z_floor=None, z_ceil=None):
        """Full pipeline: capture from the given pose, deproject, transform to WORLD frame -> (N,3).
        z_floor/z_ceil (world metres) optionally clip the cloud to a height band -> drops the ground plane and
        anything above head height, leaving the obstacle column EGO cares about (matches the cylinder cert)."""
        z_eye = self.capture(parent_node, position, hpr)
        pts_cam = self.deproject(z_eye)
        if pts_cam.shape[0] == 0:
            return np.zeros((0, 3))
        if subsample > 1:
            pts_cam = pts_cam[::subsample]
        from panda3d.core import Point3, Vec3, TransformState
        # IMPORTANT: BaseCamera.perceive() RESTORES the camera to its original pose before returning, so reading
        # self.cam.cam.getNetTransform() here yields the RESTORED (constant) pose, not the (parent_node, position,
        # hpr) we just rendered from -> the deprojected cloud would land in a fixed world orientation regardless of
        # the requested heading. Reconstruct the exact render pose from the inputs instead: world_T_cam =
        # parent.netTransform o makePosHpr(position, hpr). This is the pose perceive() actually rendered with.
        local = TransformState.makePosHpr(Vec3(float(position[0]), float(position[1]), float(position[2])),
                                          Vec3(float(hpr[0]), float(hpr[1]), float(hpr[2])))
        mat = parent_node.getNetTransform().compose(local).getMat()
        # vectorised cam->world: panda xformPoint is the row-vector product [x,y,z,1] @ M (translation in row 3).
        # One matmul over all points instead of a per-point Python loop (the per-tick render hot path).
        M = np.array([[mat.getCell(r, c) for c in range(4)] for r in range(4)], float)
        homog = np.column_stack([pts_cam, np.ones(pts_cam.shape[0])])
        out = (homog @ M)[:, :3]
        if z_floor is not None:
            out = out[out[:, 2] >= z_floor]
        if z_ceil is not None:
            out = out[out[:, 2] <= z_ceil]
        return out


# ----------------------------------------------------------------------------------------------------------------
# standalone smoke test: build a minimal env, mount the D435i on the ego, capture + deproject, sanity-check cloud.
# Run (metaurban env):  PYTHONPATH=/media/boxuan/Data2/projects DISPLAY=:1 python metaurban/d435i_sensor.py
# ----------------------------------------------------------------------------------------------------------------
def _smoke():
    import numpy as np
    from metaurban import SidewalkDynamicMetaUrbanEnv
    from metaurban.component.sensors.rgb_camera import RGBCamera
    from metaurban.component.sensors.depth_camera import DepthCamera
    cfg = dict(
        crswalk_density=1, object_density=0.9, walk_on_all_regions=False,
        use_render=False, image_observation=True,
        sensors=dict(rgb_camera=(RGBCamera, 64, 64), d435i_depth=(DepthCamera, 160, 106)),
        interface_panel=[], manual_control=False, map='X', daytime="12:00",
        default_expert=False, drivable_area_extension=55, height_scale=1,
        show_mid_block_map=False, show_ego_navigation=False, debug=False, horizon=100000,
        on_continuous_line_done=False, out_of_route_done=False,
        vehicle_config=dict(show_lidar=False, show_navi_mark=False, show_line_to_navi_mark=False,
                            show_dest_mark=False, enable_reverse=True, lidar=dict(num_lasers=0, distance=50)),
        multi_thread_render=False, decision_repeat=2, physics_world_step_size=0.05,
        show_sidewalk=True, show_crosswalk=True, random_spawn_lane_index=False,
        num_scenarios=20, accident_prob=0, relax_out_of_road_done=True, max_lateral_dist=1e3,
        crash_vehicle_done=False, crash_object_done=False, crash_human_done=False, traffic_density=0.4,
        spawn_human_num=40, spawn_wheelchairman_num=3, spawn_edog_num=3, spawn_erobot_num=2,
        spawn_drobot_num=2, max_actor_num=80)
    env = SidewalkDynamicMetaUrbanEnv(cfg)
    env.reset(seed=0)
    for _ in range(8):
        env.step([0.0, 0.0])
    eng = env.engine; ego = env.agent
    d435 = D435iDepth(eng, width=160, height=106, zmax=8.0, noise=True)
    # mount facing the ego's heading: panda3d camera +Y forward; ego.origin heading is its +X, so hpr h=0 looks +Y.
    # Use the ego heading -> we just look straight ahead (h=0 relative to the ego origin's forward).
    pos = (0.0, 0.0, 1.2)
    z_eye = d435.capture(ego.origin, position=pos, hpr=(0, 0, 0))
    finite = np.isfinite(z_eye) & (z_eye < 30)
    print(f"[d435i] depth buffer {z_eye.shape}: finite={finite.mean()*100:.0f}%  "
          f"z(min/med/max within 8m)={_stats(z_eye[(z_eye>0.3)&(z_eye<8)])}")
    cloud = d435.cloud_world(ego.origin, position=pos, hpr=(0, 0, 0))
    ep = np.asarray(ego.position, float)
    print(f"[d435i] deprojected world cloud: {cloud.shape[0]} pts")
    if cloud.shape[0]:
        d = np.linalg.norm(cloud[:, :2] - ep[:2], axis=1)
        # the D435i range spec is AXIAL depth (z_eye <= zmax); a corner pixel's EUCLIDEAN distance is larger by
        # 1/cos(corner_angle) ~ 1.4x, so euclidean up to ~zmax/cos(43.5)*... ~11 m is geometrically correct.
        axial_ok = (d.max() <= self_axial_bound()) and cloud.shape[0] > 200
        print(f"[d435i]   euclidean horiz dist from ego: min={d.min():.2f} med={np.median(d):.2f} max={d.max():.2f} m")
        print(f"[d435i]   z range: {cloud[:,2].min():.2f} .. {cloud[:,2].max():.2f} m")
        print("[d435i] PASS: real depth render -> deprojected bounded forward cloud (sim2real perception input)"
              if axial_ok else "[d435i] CHECK: cloud size/range unexpected")
        print("[d435i] NOTE: mount hpr here is arbitrary (ego.origin frame); render_3d_video --d435i aims the "
              "camera along the DRONE heading. Pipeline (depth->metric->deproject->world) is what this validates.")
    env.close()


def self_axial_bound():
    # max euclidean horizontal distance achievable for an axial depth <= 8 m at the FOV corner, + camera offset
    return 8.0 / math.cos(math.radians(D435I_HFOV_DEG) * 0.5) + 2.0


def _stats(a):
    a = a[np.isfinite(a)]
    return "n/a" if a.size == 0 else f"{a.min():.2f}/{np.median(a):.2f}/{a.max():.2f}"


if __name__ == "__main__":
    _smoke()
