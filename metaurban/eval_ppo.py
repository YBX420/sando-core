"""eval_ppo — run the learned planner through the SAME protocol as bench_run arms."""
import os
import time
import numpy as np
from stable_baselines3 import PPO
from drone_nav_env import DroneNavEnv

model = PPO.load("out/ppo_planner")
col = nm = reach = n = 0
clrs = []
for sd in range(8):                                     # 8 protocol resamples over the suite
    env = DroneNavEnv(seed=90000 + sd)
    for _ in range(len(env.files)):
        obs, _ = env.reset()
        done = False
        while not done:
            a, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(a)
            done = term or trunc
        n += 1
        col += info["collided"]; reach += info["reached"]
        nm += info["min_clr"] < 0.5; clrs.append(info["min_clr"])
line = (f"| ppo_real | {n} | {col}/{n} ({100*col/n:.1f}%) | {nm}/{n} | "
        f"{np.median(clrs):.2f} | reach {100*reach/n:.1f}% |")
print(line)
os.makedirs("out/ppo_eval", exist_ok=True)                  # persist so results survive the terminal
with open("out/ppo_eval/results.md", "a") as f:
    f.write(time.strftime("%Y-%m-%d %H:%M ") + line + "\n")
