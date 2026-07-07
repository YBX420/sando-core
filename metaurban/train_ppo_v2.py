"""train_ppo_v2 — shield-in-training (learn to propose certifiable actions) + frame-stack + 3M."""
import time
import numpy as np
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor, VecFrameStack
from stable_baselines3.common.logger import configure
from drone_nav_env import DroneNavEnv, DT
from shield import CertShield

class ShieldedEnv(gym.Wrapper):
    """Executes the SHIELDED action; small penalty when the shield must intervene."""
    def __init__(self, env):
        super().__init__(env)
        self.sh = CertShield(env.max_vel, env.max_acc, DT)
    def step(self, a):
        env = self.env
        trs, _ = env._tracks()
        tracks = []
        for tr in trs:
            c0, v0, _ = tr.trk.state()
            tracks.append((np.asarray(c0[:2], float), np.asarray(v0[:2], float), float(tr.r), str(tr.cls), int(tr.trk.n)))  # KF anchor c0, not raw det xy (FS3C-R #12: same anchor as EGO cert + harvest scoring)
        gd = env.goal - env.p
        gd = gd / max(np.linalg.norm(gd), 1e-6)
        v_next, certified, intervened = self.sh.filter(env.p, env.v, np.asarray(a, float), tracks, gd)
        a_exec = np.clip((v_next - env.v) / (DT * env.max_acc), -1, 1)
        obs, r, term, trunc, info = env.step(a_exec)
        if intervened:
            r -= 0.5                                  # teach the policy to PROPOSE certifiable actions
        if not certified:
            r -= 1.0
        return obs, r, term, trunc, info

def mk(rank):
    def _f():
        return ShieldedEnv(DroneNavEnv(seed=2000 + rank))
    return _f

if __name__ == "__main__":
    run = time.strftime("out/ppo_logs/ppo_v2_%Y%m%d_%H%M%S")   # log.txt + progress.csv + TB events land here
    logger = configure(run, ["stdout", "log", "csv", "tensorboard"])
    venv = VecFrameStack(VecMonitor(SubprocVecEnv([mk(i) for i in range(8)])), 3)
    model = PPO("MlpPolicy", venv, n_steps=512, batch_size=1024, learning_rate=3e-4,
                gamma=0.995, gae_lambda=0.95, ent_coef=0.003, verbose=1, device="cpu",
                policy_kwargs=dict(net_arch=[256, 256]))
    model.set_logger(logger)
    t0 = time.time()
    model.learn(total_timesteps=3_000_000, log_interval=20)
    model.save("out/ppo_planner_v2")
    print(f"[ppo-v2] trained in {(time.time()-t0)/60:.1f} min  (logs: {run})")
