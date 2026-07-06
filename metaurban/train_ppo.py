"""train_ppo — end-to-end learned planner arm for the MetaDrone benchmark."""
import time
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor
from stable_baselines3.common.logger import configure
from drone_nav_env import DroneNavEnv

def mk(rank):
    def _f():
        return DroneNavEnv(seed=1000 + rank)
    return _f

if __name__ == "__main__":
    run = time.strftime("out/ppo_logs/ppo_%Y%m%d_%H%M%S")     # log.txt + progress.csv + TB events land here
    logger = configure(run, ["stdout", "log", "csv", "tensorboard"])
    venv = VecMonitor(SubprocVecEnv([mk(i) for i in range(8)]))
    model = PPO("MlpPolicy", venv, n_steps=512, batch_size=1024, learning_rate=3e-4,
                gamma=0.995, gae_lambda=0.95, ent_coef=0.003, verbose=1, device="cpu",
                policy_kwargs=dict(net_arch=[128, 128]))
    model.set_logger(logger)
    t0 = time.time()
    model.learn(total_timesteps=600_000, log_interval=10)
    model.save("out/ppo_planner")
    print(f"[ppo] trained in {(time.time()-t0)/60:.1f} min -> out/ppo_planner.zip  (logs: {run})")
