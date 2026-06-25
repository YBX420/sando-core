"""inspect_env — freely INSPECT a MetaUrban scene + report the physics/realism guarantees.

MetaUrban (MetaDrive-based) has a real physics + render stack but the inspection UI is not exposed by default and
this box renders to a virtual display (:1). This gives you three ways to look inside:
  --report      : print the physics engine + settings + scene census (no render).
  --flythrough  : render a camera ORBIT + fly-THROUGH the crowd to an mp4 (headless; just watch the file).
  --interactive : open the real MetaUrban window with the FREE bird-view camera + dashboard + manual control
                  (needs a viewable display: run on the machine's monitor or VNC into :1).

Physics: Panda3D **Bullet** rigid-body world, gravity -9.81 m/s^2, fixed step 0.05 s. Agents: ORCA social crowd
(pedestrians / wheelchairs / delivery robots / robot dogs / vehicles). Scenes: procedural sidewalk blocks.

Run:  PYTHONPATH=/media/boxuan/Data2/projects/metaurban DISPLAY=:1 \
      LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6 python metaurban/inspect_env.py --flythrough --seed 3
"""
import os, sys, argparse, math
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--seed", type=int, default=3)
ap.add_argument("--report", action="store_true")
ap.add_argument("--flythrough", action="store_true")
ap.add_argument("--interactive", action="store_true")
ap.add_argument("--w", type=int, default=640)
ap.add_argument("--h", type=int, default=400)
ap.add_argument("--frames", type=int, default=240)
args = ap.parse_args()

from metaurban import SidewalkDynamicMetaUrbanEnv
from metaurban.component.sensors.rgb_camera import RGBCamera
from metaurban.component.agents.pedestrian.base_pedestrian import BasePedestrian
from metaurban.component.vehicle.base_vehicle import BaseVehicle

BASE = dict(
    crswalk_density=1, object_density=0.9, walk_on_all_regions=False, map='X', daytime="12:00",
    drivable_area_extension=55, height_scale=1, show_mid_block_map=False, show_ego_navigation=False,
    on_continuous_line_done=False, out_of_route_done=False, horizon=100000,
    vehicle_config=dict(show_lidar=False, show_navi_mark=False, lidar=dict(num_lasers=0, distance=50)),
    decision_repeat=2, physics_world_step_size=0.05, show_sidewalk=True, show_crosswalk=True,
    random_spawn_lane_index=False, num_scenarios=20, accident_prob=0, relax_out_of_road_done=True,
    max_lateral_dist=1e3, crash_vehicle_done=False, crash_object_done=False, crash_human_done=False,
    traffic_density=0.5, spawn_human_num=85, spawn_wheelchairman_num=5, spawn_edog_num=6,
    spawn_erobot_num=3, spawn_drobot_num=3, max_actor_num=170)


def census(env):
    eng = env.engine; nped = nveh = nstatic = 0
    for oid, o in eng.get_objects().items():
        if isinstance(o, BasePedestrian): nped += 1
        elif isinstance(o, BaseVehicle): nveh += 1
        else: nstatic += 1
    return nped, nveh, nstatic


def report(env):
    eng = env.engine
    pw = eng.physics_world.dynamic_world
    g = pw.getGravity()
    nped, nveh, nstatic = census(env)
    print("\n=== MetaUrban physics + realism ===")
    print(f"  engine:        Panda3D Bullet rigid-body (panda3d.bullet.BulletWorld)")
    print(f"  gravity:       ({g[0]:.2f}, {g[1]:.2f}, {g[2]:.2f}) m/s^2")
    print(f"  step:          physics_world_step_size=0.05 s, decision_repeat=2 -> 0.1 s control step")
    print(f"  agents:        ORCA social crowd — pedestrians={nped} vehicles/robots={nveh} static={nstatic}")
    print(f"  scene:         procedural sidewalk blocks (map 'X'), {BASE['num_scenarios']} scenarios")
    print(f"  realism:       research-grade behavioural sim (realistic micromobility motion); not photoreal graphics")


def main():
    if args.interactive:
        env = SidewalkDynamicMetaUrbanEnv(dict(BASE, use_render=True, manual_control=True,
                                               interface_panel=["dashboard"], image_observation=False))
        env.reset(seed=args.seed % 20)
        report(env)
        print("\n[inspect] INTERACTIVE: free bird-view camera = mouse + wheel(height); 'b' top-down; WASD drive the "
              "agent. View on the machine monitor or VNC into :1. Ctrl-C to quit.", flush=True)
        try:
            while True:
                env.step([0.0, 0.0]); env.render()
        except KeyboardInterrupt:
            pass
        env.close(); return 0

    # offscreen for --report / --flythrough; register the D435i depth camera too so the inspection shows the
    # planner's REAL perception (the depth the onboard D435i would see), side-by-side with the RGB scene.
    from metaurban.component.sensors.depth_camera import DepthCamera
    env = SidewalkDynamicMetaUrbanEnv(dict(BASE, use_render=False, image_observation=True,
                                           sensors=dict(rgb_camera=(RGBCamera, args.w, args.h),
                                                        d435i_depth=(DepthCamera, 160, 106)), interface_panel=[]))
    env.reset(seed=args.seed % 20)
    for _ in range(8):
        env.step([0.0, 0.0])
    report(env)
    if not args.flythrough:
        env.close(); return 0

    import imageio.v2 as imageio
    from panda3d.core import Vec3
    from d435i_sensor import D435iDepth
    eng = env.engine; cam = eng.get_sensor("rgb_camera")
    d435 = D435iDepth(eng, width=160, height=106, zmax=8.0, noise=True)   # the onboard D435i perception

    def _colorize_depth(z, zmax=8.0):
        H, W = z.shape
        valid = np.isfinite(z) & (z > 0.3) & (z < zmax)
        norm = np.clip((z - 0.3) / (zmax - 0.3), 0, 1)
        g = (255 * (1 - norm)).astype(np.uint8)              # near = bright, far = dark
        out = np.stack([g, g, (0.6 * g).astype(np.uint8)], -1)
        out[~valid] = 0                                      # sky / beyond range = black (no return)
        return out

    def _resize_nn(im, H, W):
        h, w = im.shape[:2]
        return im[(np.arange(H) * h // H)][:, (np.arange(W) * w // W)]
    # crowd centroid (xy) to orbit around
    pts = []
    for oid, o in eng.get_objects().items():
        if isinstance(o, (BasePedestrian, BaseVehicle)):
            try:
                p = np.asarray(o.position, float)
                if np.all(np.isfinite(p[:2])): pts.append(p[:2])
            except Exception:
                pass
    c = np.mean(pts, axis=0) if pts else np.array([0.0, 0.0])
    print(f"[inspect] flythrough around crowd centroid {np.round(c, 1)} ({len(pts)} agents) -> out/env_flythrough.mp4", flush=True)
    os.makedirs("out", exist_ok=True)
    writer = imageio.get_writer("out/env_flythrough.mp4", fps=30, macro_block_size=16)
    N = args.frames
    for i in range(N):
        env.step([0.0, 0.0])                                  # keep the crowd moving
        ph = i / N
        if ph < 0.6:                                          # ORBIT: circle the crowd, looking inward
            ang = ph / 0.6 * 2 * math.pi
            rad = 16.0; hgt = 8.0 - 4.0 * math.sin(ph / 0.6 * math.pi)
            pos = np.array([c[0] + rad * math.cos(ang), c[1] + rad * math.sin(ang), hgt])
            look = np.array([c[0], c[1], 1.2])
        else:                                                 # FLY-THROUGH: descend + cross the crowd
            t = (ph - 0.6) / 0.4
            pos = np.array([c[0] - 18 + 36 * t, c[1] + 4 * math.sin(t * math.pi), 2.5])
            look = np.array([c[0] + 6, c[1], 1.5])
        fwd = look - pos
        yaw = math.degrees(math.atan2(fwd[1], fwd[0]))
        pitch = math.degrees(math.atan2(fwd[2], math.hypot(fwd[0], fwd[1])))
        campos = Vec3(float(pos[0]), float(pos[1]), float(pos[2]))
        camhpr = Vec3(yaw - 90, pitch, 0)                     # panda cam looks +Y; -90 aligns to +X heading
        img = cam.perceive(to_float=False, new_parent_node=eng.origin, position=campos, hpr=camhpr)[..., :3]
        # the D435i depth from the SAME pose -> colorized + upscaled to the RGB height, stitched RGB | D435i-depth
        z = d435.capture(eng.origin, position=(float(pos[0]), float(pos[1]), float(pos[2])),
                         hpr=(yaw - 90, pitch, 0))
        dep = _resize_nn(_colorize_depth(z, zmax=8.0), img.shape[0], img.shape[1])
        frame = np.concatenate([np.ascontiguousarray(img), dep], axis=1)   # side by side
        writer.append_data(np.ascontiguousarray(frame))
    writer.close()
    env.close()
    print("[inspect] wrote out/env_flythrough.mp4", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
