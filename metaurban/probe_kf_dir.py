"""probe_kf_dir — KF 速度方向 vs GT 真速度方向,按 track age 分桶。
回答"KF 方向对物体运动还要不要优化":成熟 track(age>=4)方向误差若已小,则方向感知机动可信、KF 无需再调。
用法: PYTHONPATH=<metaurban> python probe_kf_dir.py [--episodes 6]"""
import argparse, os, sys
_HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, _HERE)
import numpy as np
from metaurban_nav_env import MetaUrbanNavEnv, DT

ap = argparse.ArgumentParser(); ap.add_argument("--episodes", type=int, default=6); args = ap.parse_args()
env = MetaUrbanNavEnv(seed=0)
SPD_MIN = 0.3          # 方向只对真在动的 mover 有意义
MATCH_MAX = 2.5        # KF track ↔ GT mover 最近匹配上限(m)
recs = []              # (age, angle_deg, kf_speed, gt_speed, cls)

for ep in range(args.episodes):
    env.reset(seed=1000 + ep)
    for _ in range(120):
        trs, _ = env._tracks()
        gt = env._native_movers()                       # (xy_world, r, h, cls, vel_world)
        gt_rel = [(np.asarray(m[0], float) - env.org, np.asarray(m[4], float), str(m[3])) for m in gt]
        for tr in trs:
            if not tr.trk.ready:
                continue
            _, vv, _ = tr.trk.state(); kf_v = np.asarray(vv[:2], float); kf_sp = float(np.linalg.norm(kf_v))
            if kf_sp < SPD_MIN:
                continue
            kf_xy = np.asarray(tr.xy[:2], float)
            if not gt_rel:
                continue
            j = int(np.argmin([np.linalg.norm(g[0] - kf_xy) for g in gt_rel]))
            if np.linalg.norm(gt_rel[j][0] - kf_xy) > MATCH_MAX:
                continue
            gt_v = gt_rel[j][1]; gt_sp = float(np.linalg.norm(gt_v))
            if gt_sp < SPD_MIN:
                continue
            cosang = float(np.dot(kf_v, gt_v) / (kf_sp * gt_sp))
            ang = float(np.degrees(np.arccos(np.clip(cosang, -1, 1))))
            recs.append((int(tr.trk.n), ang, kf_sp, gt_sp, gt_rel[j][2]))
        # 驱动 ego 朝目标(制造真实遭遇),世界照常推进
        gd = (env.goal - env.p); gd = gd / max(np.linalg.norm(gd), 1e-6)
        v_cmd = gd * env.max_vel
        act = np.clip((v_cmd - env.v) / DT / env.max_acc, -1, 1)
        _o, _r, term, trunc, _i = env.step(act)
        if term or trunc:
            break

recs = np.array([(a, ang, ks, gs) for (a, ang, ks, gs, c) in recs], float)
print(f"\n[kf_dir] 样本 n={len(recs)} (KF track↔GT 匹配, 双方速度>{SPD_MIN}m/s)")
print("age 桶      | n    | 方向角误差 中位/均值/p90 (deg) | KF速度中位")
for lo, hi, lab in [(2,2,"2 (young)"),(3,3,"3 (young)"),(4,6,"4-6 (mature)"),(7,999,"7+ (mature)")]:
    m = (recs[:,0] >= lo) & (recs[:,0] <= hi)
    if m.sum() == 0:
        print(f"{lab:11s}| 0"); continue
    a = recs[m,1]
    print(f"{lab:11s}| {int(m.sum()):4d} | {np.median(a):5.1f} / {np.mean(a):5.1f} / {np.percentile(a,90):5.1f}       | {np.median(recs[m,2]):.2f}")
print("\n判读:成熟桶(age>=4)中位角误差 = 方向感知机动能否信 KF 方向的直接证据。")
