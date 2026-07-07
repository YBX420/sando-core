#!/bin/bash
# 平滑固定靶场(塔菲大人 2026-07-08):两个固定 (seed,map) 靶,每次平滑迭代打同两靶。
# 用法: ops/smooth_lab.sh [KEY=VAL ...]   (额外 env 直接传决策栈,如 SPEED_SLEW=1)
# 输出: 每靶 ours 姿态统计 vs native 缓存参照 + 曲线 PNG(out/smoothlab_<靶>.png)
set -e
SC=/media/boxuan/Data2/projects/sando_py/sando-core
PY=$HOME/miniconda3/envs/metaurban/bin/python
RUN="env DISPLAY=:1 PYTHONPATH=/media/boxuan/Data2/projects/metaurban LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6"
cd /media/boxuan/Data2/projects/metaurban
EXTRA="$*"                       # 决策栈旋钮(KEY=VAL),必须在 set -- 覆盖 $@ 之前存下
for TGT in "A 7 X" "B 12 CS"; do
  set -- $TGT; NAME=$1; SEED=$2; MAP=$3
  REF=$SC/metaurban/out/telem_native_${NAME}.json
  if [ ! -s "$REF" ]; then
    echo "[$NAME] 生成 native 参照 (seed=$SEED map=$MAP)..."
    $RUN MU_MAP=$MAP TELEM_OUT=$REF $PY -u $SC/metaurban/render_3d_video.py \
      --seed $SEED --ego --clear_spawn --headless --t_max 25 2>&1 | grep "lap done"
  fi
  OUT=$SC/metaurban/out/telem_ours_${NAME}.json
  $RUN MU_MAP=$MAP SPEEDS_CRAWL=1 TAU_SPEED=1 $EXTRA TELEM_OUT=$OUT $PY -u \
    $SC/metaurban/render_3d_video.py --seed $SEED --maneuver --clear_spawn --headless --t_max 25 \
    2>&1 | grep -E "lap done"
  $PY - <<PYEOF
import json
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
o = np.array(json.load(open("$OUT"))); r = np.array(json.load(open("$REF")))
def stats(T):
    tilt = np.hypot(T[:,1], T[:,2]); rate = np.abs(np.diff(tilt)) / 0.06
    return tilt.mean(), tilt.max(), float(np.sqrt((rate**2).mean()))
so, sr = stats(o), stats(r)
print(f"[$NAME seed=$SEED map=$MAP] ours 倾角均值 {so[0]:.1f}° max {so[1]:.0f}° 速率RMS {so[2]:.0f}°/s"
      f"  || native {sr[0]:.1f}°/{sr[1]:.0f}°/{sr[2]:.0f}°/s  → 差距 {so[2]-sr[2]:+.0f}°/s")
fig, ax = plt.subplots(2, 1, figsize=(10, 5.5), dpi=110, sharex=True)
for T, c, lb in ((r, "#888", "native"), (o, "#15d", "ours")):
    ax[0].plot(T[:,0], np.hypot(T[:,1], T[:,2]), color=c, lw=1.0, label=lb)
    ax[1].plot(T[:,0], T[:,3], color=c, lw=1.0, label=lb)
ax[0].set_ylabel("tilt (deg)"); ax[1].set_ylabel("|v| m/s"); ax[1].set_xlabel("t (s)")
for a in ax: a.legend(fontsize=8); a.grid(alpha=0.3)
ax[0].set_title("smooth-lab $NAME: seed=$SEED map=$MAP")
plt.tight_layout(); plt.savefig("$SC/metaurban/out/smoothlab_$NAME.png")
PYEOF
done
echo "靶场完毕: out/smoothlab_A.png / smoothlab_B.png"
