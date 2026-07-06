"""train_ppo_ground — the generalization arm: PPO sidewalk robot (unicycle) on live MetaUrban,
shield-in-training with the arc-adapted certificate gate. Recipe mirrors train_ppo_mu (fixed-lambda,
n_envs=1, frame-stack 3, checkpoints) so the drone-vs-ground comparison isolates the EMBODIMENT.

Run (metaurban env, cwd+PYTHONPATH = the metaurban repo):
  python train_ppo_ground.py --steps 150000 [--no_shield]
"""
import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor, VecFrameStack
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure

from ground_nav_env import GroundNavEnv
from ground_shield import GroundShieldedEnv


def mk(rank, no_shield=False):
    def _f():
        e = GroundNavEnv(seed=1000 + rank)
        return e if no_shield else GroundShieldedEnv(e)
    return _f


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=150_000)
    ap.add_argument("--n_envs", type=int, default=1)
    ap.add_argument("--ckpt_freq", type=int, default=10_000)
    ap.add_argument("--no_shield", action="store_true")
    ap.add_argument("--resume", type=str, default="")
    args = ap.parse_args()

    os.chdir(_HERE)
    variant = "ground_bare" if args.no_shield else "ground"
    tag = f"ppo_{variant}"
    run = time.strftime(f"out/ppo_logs/{tag}_%Y%m%d_%H%M%S")
    logger = configure(run, ["stdout", "log", "csv", "tensorboard"])
    venv = VecFrameStack(VecMonitor(DummyVecEnv(
        [mk(i, no_shield=args.no_shield) for i in range(args.n_envs)])), 3)
    cbs = [CheckpointCallback(save_freq=max(1, args.ckpt_freq // args.n_envs),
                              save_path=f"out/ppo_ckpt_{variant}", name_prefix=tag)]
    if args.resume:
        model = PPO.load(args.resume, env=venv, device="cpu")
    else:
        model = PPO("MlpPolicy", venv, n_steps=512, batch_size=512, learning_rate=3e-4,
                    gamma=0.995, gae_lambda=0.95, ent_coef=0.003, verbose=1, device="cpu",
                    policy_kwargs=dict(net_arch=[256, 256]))
    model.set_logger(logger)
    print(f"[ppo-ground] start: steps={args.steps} no_shield={args.no_shield}  (logs: {run})",
          flush=True)
    t0 = time.time()
    model.learn(total_timesteps=args.steps, log_interval=1, callback=cbs,
                reset_num_timesteps=not bool(args.resume))
    model.save(f"out/ppo_planner_{variant}")
    print(f"[ppo-ground] done in {(time.time()-t0)/60:.1f} min -> out/ppo_planner_{variant}.zip",
          flush=True)
