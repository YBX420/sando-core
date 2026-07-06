"""Algorithm-kernel latency: (A) C++ bernstein cert via ego_bridge ctypes; (B) PPO policy + CertShield."""
import os, sys, time
MU = "/media/boxuan/Data2/projects/sando_py/sando-core/metaurban"
sys.path.insert(0, MU); os.chdir(MU)
import numpy as np

def bench(fn, n=2000, warm=50):
    for _ in range(warm): fn()
    t0 = time.perf_counter()
    for _ in range(n): fn()
    us = (time.perf_counter() - t0) / n * 1e6
    return us

print("=== A) C++ 证书核(ctypes 单调用)===", flush=True)
from ego_bridge import EGOPlanner
ego = EGOPlanner(map_origin=(-40, -40, -1), map_size=(80, 80, 8), res=0.2, inflation=0.45)
ego.update_cloud(np.zeros((0, 3)), np.array([0., 0., 1.5]))
ok = ego.replan(np.array([0., 0., 1.5]), np.array([1., 0., 0.]), np.zeros(3), np.array([10., 0., 1.5]))
print(f"  replan ok={ok} dur={ego.duration():.2f}s", flush=True)
c3, vel = (5.0, 0.5, 1.5), (0.0, -1.0)
us_h = bench(lambda: ego.certify_horizontal(obs_c0=c3, R=1.4, obs_vel=vel, t_hi=0.75, v_eff=0.996, delta=0.3))
us_v = bench(lambda: ego.certify_above(z_clear=3.4, t_hi=0.75, delta=0.3))
us_rp = bench(lambda: ego.replan(np.array([0., 0., 1.5]), np.array([1., 0., 0.]), np.zeros(3),
                                 np.array([10., 0., 1.5])), n=200, warm=5)
print(f"  certify_horizontal: {us_h:8.1f} us/call", flush=True)
print(f"  certify_above     : {us_v:8.1f} us/call", flush=True)
print(f"  ego.replan (free) : {us_rp:8.1f} us/call", flush=True)

print("=== B) PPO 臂(策略网 + 证书门)===", flush=True)
from stable_baselines3 import PPO
from shield import CertShield, action_safe
model = PPO.load("out/ppo_planner_mu", device="cpu")
obs = np.random.randn(102).astype(np.float32)
us_pol = bench(lambda: model.predict(obs, deterministic=True))
sh = CertShield(3.0, 6.0, 0.3)
p, v, gd = np.zeros(2), np.array([2.0, 0.0]), np.array([1.0, 0.0])
trk3 = [(np.array([4.0, 1.0]), np.array([0.0, -1.0]), 0.4),
        (np.array([6.0, -2.0]), np.array([-1.0, 0.5]), 0.5),
        (np.array([3.0, 3.0]), np.array([0.5, -0.5]), 0.3)]
trk6 = trk3 + [(np.array([8.0, 0.0]), np.array([-2.0, 0.0]), 2.0),
               (np.array([5.0, -4.0]), np.array([0.0, 1.0]), 0.35),
               (np.array([2.0, -2.0]), np.array([0.3, 0.3]), 0.3)]
us_as3 = bench(lambda: action_safe(p, v, trk3))
us_sh3 = bench(lambda: sh.filter(p, v, np.array([0.5, 0.1]), trk3, gd))
us_sh6 = bench(lambda: sh.filter(p, v, np.array([0.5, 0.1]), trk6, gd))
# 最坏情形:强迫走投影分支(动作直冲障碍)
trk_block = [(np.array([0.9, 0.0]), np.array([0.0, 0.0]), 0.5)]
us_proj = bench(lambda: sh.filter(p, np.array([3.0, 0.0]), np.array([1.0, 0.0]), trk_block, gd))
print(f"  policy MLP forward      : {us_pol:8.1f} us", flush=True)
print(f"  action_safe (3 tracks)  : {us_as3:8.1f} us", flush=True)
print(f"  shield.filter pass(3trk): {us_sh3:8.1f} us", flush=True)
print(f"  shield.filter pass(6trk): {us_sh6:8.1f} us", flush=True)
print(f"  shield.filter 投影最坏  : {us_proj:8.1f} us", flush=True)
print(f"\n  RL 臂整链(policy+shield,6trk) ~= {us_pol+us_sh6:.0f} us -> {1e6/(us_pol+us_sh6):.0f} Hz", flush=True)
