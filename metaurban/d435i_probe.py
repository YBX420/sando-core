"""d435i_probe — pin down WHY the d435i world-frame cloud drifts off the requested heading.

For a set of requested headings, mount the D435i at the world origin aimed along that heading (the exact mount
render_3d_video.d435i_cloud now uses), capture the cloud TWICE in a row (to expose any 1-frame render lag), and
report: requested heading, camera net-transform heading, cloud mean bearing, frac of points inside the +/-43.5deg
forward cone. If frac jumps between the two captures of the SAME pose, the depth render lags one frame.

Run: PYTHONPATH=/media/boxuan/Data2/projects DISPLAY=:1 LD_PRELOAD=...sando/lib/libstdc++.so.6 \
     python metaurban/d435i_probe.py
"""
import os, sys, math
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from metaurban import SidewalkDynamicMetaUrbanEnv
from metaurban.component.sensors.rgb_camera import RGBCamera
from metaurban.component.sensors.depth_camera import DepthCamera
from d435i_sensor import D435iDepth

FPV_POS, FPV_HPR = (0.35, 0.0, 0.20), (-90.0, -12.0, 0.0)

cfg = dict(crswalk_density=1, object_density=0.9, walk_on_all_regions=False, map='X', daytime="12:00",
           use_render=False, image_observation=True,
           sensors=dict(rgb_camera=(RGBCamera, 64, 64), d435i_depth=(DepthCamera, 160, 106)),
           interface_panel=[], drivable_area_extension=55, height_scale=1, show_mid_block_map=False,
           show_ego_navigation=False, horizon=100000, on_continuous_line_done=False, out_of_route_done=False,
           vehicle_config=dict(show_lidar=False, show_navi_mark=False, lidar=dict(num_lasers=0, distance=50)),
           decision_repeat=2, physics_world_step_size=0.05, show_sidewalk=True, show_crosswalk=True,
           random_spawn_lane_index=False, num_scenarios=20, accident_prob=0, relax_out_of_road_done=True,
           max_lateral_dist=1e3, crash_vehicle_done=False, crash_object_done=False, crash_human_done=False,
           traffic_density=0.5, spawn_human_num=85, spawn_wheelchairman_num=5, spawn_edog_num=6,
           spawn_erobot_num=3, spawn_drobot_num=3, max_actor_num=170)


def world_cloud(d435, eng, p_d, heading):
    """EXACT mount render_3d_video.d435i_cloud uses: world origin, nose offset rotated by heading, hpr h=deg-90."""
    hd = float(heading); ch, sh = math.cos(hd), math.sin(hd)
    wpos = (p_d[0] + FPV_POS[0] * ch, p_d[1] + FPV_POS[0] * sh, p_d[2] + FPV_POS[2])
    whpr = (math.degrees(hd) + FPV_HPR[0], FPV_HPR[1], FPV_HPR[2])
    c = d435.cloud_world(eng.origin, position=wpos, hpr=whpr, subsample=2, z_floor=0.2, z_ceil=3.0)
    net_h = float(d435.cam.cam.getNetTransform().getMat().getRow3(1)[0])  # +Y col x-component (cheap heading proxy)
    return c, whpr[0], net_h


def stats(c, p_d, heading):
    if not len(c):
        return "n=0"
    d = c[:, :2] - np.asarray(p_d[:2])
    dxy = np.linalg.norm(d, axis=1)
    fwd = np.array([math.cos(heading), math.sin(heading)])
    cosang = (d @ fwd) / np.maximum(dxy, 1e-6)
    mean_brg = math.degrees(math.atan2(np.mean(d[:, 1]), np.mean(d[:, 0])))
    return (f"n={len(c)} frac_ahead={np.mean(cosang>0):.2f} frac_in_cone={np.mean(cosang>math.cos(math.radians(43.5))):.2f} "
            f"cloud_mean_bearing={mean_brg:+.0f}deg")


def main():
    env = SidewalkDynamicMetaUrbanEnv(cfg)
    env.reset(seed=3)
    for _ in range(8):
        env.step([0.0, 0.0])
    eng = env.engine
    d435 = D435iDepth(eng, width=160, height=106, zmax=8.0, noise=True)
    # use the ego start position as a representative drone pose at cruise height
    p0 = np.asarray(env.agent.position, float)
    p_d = np.array([p0[0], p0[1], 1.5])
    print(f"[probe] drone at {np.round(p_d,1)}; testing requested headings, double-capture to expose render lag\n")
    for deg in [0, 45, 90, 135, 180, -135, -90, -45]:
        hd = math.radians(deg)
        c1, reqH, netH = world_cloud(d435, eng, p_d, hd)   # 1st capture
        s1 = stats(c1, p_d, hd)
        c2, _, _ = world_cloud(d435, eng, p_d, hd)         # 2nd capture, SAME pose
        s2 = stats(c2, p_d, hd)
        lag = "  <-- LAG (1st != 2nd)" if (len(c1) and len(c2) and abs(
            float(s1.split('frac_ahead=')[1][:4]) - float(s2.split('frac_ahead=')[1][:4])) > 0.15) else ""
        print(f"  req={deg:+4d}deg (camH={reqH:+.0f}) | 1st: {s1}")
        print(f"  {'':17} | 2nd: {s2}{lag}")
    env.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
