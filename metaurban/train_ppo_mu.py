"""train_ppo_mu — PPO on the LIVE MetaUrban world (metaurban_nav_env), shield-in-training.

Same recipe as train_ppo_v2 (shielded action + frame-stack + [256,256] MLP), but:
  * world = MetaUrbanNavEnv (real MetaUrban physics/GT) instead of the scenario_lib replay;
  * n_envs=1 by default + CPU-friendly, because this box's Ryzen 5800X throws a Bank5 MCE and
    reboots under all-core load (see memory training-machine-cpu-mce-reboots) — keep it gentle;
  * CheckpointCallback every --ckpt_freq steps so an MCE reboot costs at most that much, not the run.

Run (metaurban conda env; CWD + PYTHONPATH = the metaurban repo so assets/imports resolve):
  cd /media/boxuan/Data2/projects/metaurban
  PYTHONPATH=/media/boxuan/Data2/projects/metaurban \
    ~/miniconda3/envs/metaurban/bin/python \
    /media/boxuan/Data2/projects/sando_py/sando-core/metaurban/train_ppo_mu.py --steps 150000
"""
import argparse
import os
import sys
import time
from collections import deque

# this file's dir holds metaurban_nav_env, shield, train_ppo_v2 (ShieldedEnv), perception
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import numpy as np
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor, VecFrameStack
from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback
from stable_baselines3.common.logger import configure

from metaurban_nav_env import MetaUrbanNavEnv, DT
from shield import CertShield
from train_ppo_v2 import ShieldedEnv          # reuse the EXACT shield wrapper from v2 (zero drift)


class AdaptiveShieldedEnv(gym.Wrapper):
    """ShieldedEnv with an ADAPTIVE Lagrangian multiplier (PPO-Lagrangian / RCPO style dual ascent).

    v2's fixed penalties (intervene -0.5, uncert extra -1.0) are a hardwired multiplier. Here the
    SAME 1:2 penalty shape is scaled by lam, and lam follows the measured intervention rate:
        lam <- clip(lam + lam_lr * (rate - target_rate), 0, 5)     every 64 ticks, over a 2048-step
    sliding window — two-timescale: lam moves much slower than the policy, so PPO sees a slowly
    drifting reward (standard constrained-RL setup). lam0=0.5 makes step 0 IDENTICAL to the fixed
    recipe, so fixed-vs-adaptive is a clean one-knob ablation.
    """

    def __init__(self, env, target_rate=0.05, lam0=0.5, lam_lr=0.05):
        super().__init__(env)
        self.sh = CertShield(env.max_vel, env.max_acc, DT)
        self.lam, self.lam_lr, self.target_rate = float(lam0), float(lam_lr), float(target_rate)
        self._win = deque(maxlen=2048)
        self._n = 0

    def step(self, a):
        env = self.env
        trs, _ = env._tracks()
        tracks = []
        for tr in trs:
            c0, v0, _ = tr.trk.state()
            tracks.append((np.asarray(tr.xy, float), np.asarray(v0[:2], float), float(tr.r), str(tr.cls)))
        gd = env.goal - env.p
        gd = gd / max(np.linalg.norm(gd), 1e-6)
        v_next, certified, intervened = self.sh.filter(env.p, env.v, np.asarray(a, float), tracks, gd)
        a_exec = np.clip((v_next - env.v) / (DT * env.max_acc), -1, 1)
        obs, r, term, trunc, info = env.step(a_exec)
        self._win.append(1.0 if intervened else 0.0)
        if intervened:
            r -= self.lam
        if not certified:
            r -= 2.0 * self.lam
        self._n += 1
        if len(self._win) == self._win.maxlen and self._n % 64 == 0:
            rate = sum(self._win) / len(self._win)
            self.lam = float(np.clip(self.lam + self.lam_lr * (rate - self.target_rate), 0.0, 5.0))
        info["shield_lam"] = self.lam
        info["shield_rate"] = (sum(self._win) / len(self._win)) if self._win else 0.0
        return obs, float(r), term, trunc, info


class ShieldLamCallback(BaseCallback):
    """Log lam + measured intervention rate into the SB3 logger (csv/TB) alongside train stats."""

    def _on_step(self):
        info = self.locals.get("infos", [{}])[0]
        if "shield_lam" in info:
            self.logger.record("shield/lam", info["shield_lam"])
            self.logger.record("shield/intervene_rate", info["shield_rate"])
        return True


def mk(rank, adapt=False, target_rate=0.05, lam0=0.5, lam_lr=0.05, no_shield=False):
    def _f():
        e = MetaUrbanNavEnv(seed=1000 + rank)
        if no_shield:
            return e                            # pure-learning arm: no gate, no shaping
        if adapt:
            return AdaptiveShieldedEnv(e, target_rate=target_rate, lam0=lam0, lam_lr=lam_lr)
        return ShieldedEnv(e)
    return _f


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=150_000, help="total env timesteps (trial default)")
    ap.add_argument("--n_envs", type=int, default=1, help="parallel MetaUrban envs (keep small: CPU MCE)")
    ap.add_argument("--ckpt_freq", type=int, default=10_000, help="save a checkpoint every N steps/env")
    ap.add_argument("--device", type=str, default="cpu", help="cpu (MlpPolicy is tiny; GPU gives ~nothing)")
    ap.add_argument("--resume", type=str, default="", help="path to a .zip checkpoint to continue from")
    ap.add_argument("--adapt_lambda", action="store_true",
                    help="adaptive Lagrangian shield penalty (default: v2's fixed -0.5/-1.0)")
    ap.add_argument("--no_shield", action="store_true",
                    help="train the BARE policy (no shield at all): the pure-learning 2x2 arm")
    ap.add_argument("--target_rate", type=float, default=0.05, help="target shield intervention rate")
    ap.add_argument("--lam0", type=float, default=0.5, help="initial multiplier (0.5 = fixed recipe)")
    ap.add_argument("--lam_lr", type=float, default=0.05, help="dual-ascent step size")
    args = ap.parse_args()
    assert not (args.adapt_lambda and args.no_shield), "--adapt_lambda and --no_shield are exclusive"

    os.chdir(_HERE)                            # so out/ppo_logs, out/ppo_ckpt_mu land beside v1/v2
    variant = "mu_adapt" if args.adapt_lambda else ("mu_bare" if args.no_shield else "mu")
    tag = f"ppo_{variant}"
    run = time.strftime(f"out/ppo_logs/{tag}_%Y%m%d_%H%M%S")
    logger = configure(run, ["stdout", "log", "csv", "tensorboard"])

    venv = VecFrameStack(VecMonitor(DummyVecEnv(
        [mk(i, adapt=args.adapt_lambda, target_rate=args.target_rate,
            lam0=args.lam0, lam_lr=args.lam_lr, no_shield=args.no_shield)
         for i in range(args.n_envs)])), 3)

    cbs = [CheckpointCallback(save_freq=max(1, args.ckpt_freq // args.n_envs),
                              save_path=f"out/ppo_ckpt_{variant}", name_prefix=tag)]
    if args.adapt_lambda:
        cbs.append(ShieldLamCallback())

    if args.resume:
        print(f"[ppo-mu] resuming from {args.resume}", flush=True)
        model = PPO.load(args.resume, env=venv, device=args.device)
    else:
        model = PPO("MlpPolicy", venv, n_steps=512, batch_size=1024, learning_rate=3e-4,
                    gamma=0.995, gae_lambda=0.95, ent_coef=0.003, verbose=1, device=args.device,
                    policy_kwargs=dict(net_arch=[256, 256]))
    model.set_logger(logger)

    print(f"[ppo-mu] start: steps={args.steps} n_envs={args.n_envs} device={args.device} "
          f"adapt_lambda={args.adapt_lambda} ckpt_every={args.ckpt_freq}  (logs: {run})", flush=True)
    t0 = time.time()
    model.learn(total_timesteps=args.steps, log_interval=1, callback=cbs,
                reset_num_timesteps=not bool(args.resume))
    out_zip = f"out/ppo_planner_{variant}"
    model.save(out_zip)
    print(f"[ppo-mu] done in {(time.time()-t0)/60:.1f} min -> {out_zip}.zip", flush=True)
