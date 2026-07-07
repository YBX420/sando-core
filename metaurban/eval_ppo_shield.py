"""eval_ppo_shield — the composition arm: LEARNED proposal + CERTIFIED gate."""
import os
import time
import numpy as np
from stable_baselines3 import PPO
from drone_nav_env import DroneNavEnv, DT
from shield import CertShield

model = PPO.load("out/ppo_planner")
col = nm = reach = n = 0
clrs = []
uncert = []
for sd in range(8):
    env = DroneNavEnv(seed=90000 + sd)
    sh = CertShield(env.max_vel, env.max_acc, DT)
    for _ in range(len(env.files)):
        obs, _ = env.reset()
        done = False
        while not done:
            a, _ = model.predict(obs, deterministic=True)
            trs, _ = env._tracks()
            tracks = []
            for tr in trs:
                c0, v0, _acc = tr.trk.state()
                tracks.append((np.asarray(c0[:2], float), np.asarray(v0[:2], float), float(tr.r), str(tr.cls), int(tr.trk.n)))  # KF anchor + cls (FS3C-R #12/#13)
            gd = env.goal - env.p
            gd = gd / max(np.linalg.norm(gd), 1e-6)
            v_next, certified, _iv = sh.filter(env.p, env.v, np.asarray(a, float), tracks, gd)
            a_exec = np.clip((v_next - env.v) / (DT * env.max_acc), -1, 1)
            obs, r, term, trunc, info = env.step(a_exec)
            done = term or trunc
        n += 1
        col += info["collided"]; reach += info["reached"]
        nm += info["min_clr"] < 0.5; clrs.append(info["min_clr"])
    uncert.append(sh.stats()["uncert_frac"])
line = (f"| ppo_shield | {n} | {col}/{n} ({100*col/n:.1f}%) | {nm}/{n} | "
        f"{np.median(clrs):.2f} | reach {100*reach/n:.1f}% | uncert_ticks {np.mean(uncert)*100:.1f}% |")
print(line)
os.makedirs("out/ppo_eval", exist_ok=True)                  # persist so results survive the terminal
with open("out/ppo_eval/results.md", "a") as f:
    f.write(time.strftime("%Y-%m-%d %H:%M ") + line + "\n")
