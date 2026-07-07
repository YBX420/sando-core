#!/bin/bash
# 反过拟合姿态扫描:6 靶(3 seed × 2 图)聚合评分。用法: ops/smooth_sweep.sh <标签> [KEY=VAL ...]
set -e
SC=/media/boxuan/Data2/projects/sando_py/sando-core
PY=$HOME/miniconda3/envs/metaurban/bin/python
RUN="env DISPLAY=:1 PYTHONPATH=/media/boxuan/Data2/projects/metaurban LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6"
TAG=$1; shift; EXTRA="$*"
cd /media/boxuan/Data2/projects/metaurban
RES=$SC/metaurban/out/sweep_${TAG}.txt; : > $RES
for TGT in "7 X" "12 CS" "3 X" "21 CS" "5 S" "15 X"; do
  set -- $TGT; SEED=$1; MAP=$2
  REF=$SC/metaurban/out/telem_native_s${SEED}${MAP}.json
  if [ ! -s "$REF" ]; then
    $RUN MU_MAP=$MAP TELEM_OUT=$REF $PY -u $SC/metaurban/render_3d_video.py \
      --seed $SEED --ego --clear_spawn --headless --t_max 25 >/dev/null 2>&1 || echo "native s$SEED$MAP FAIL" >> $RES
  fi
  OUT=$SC/metaurban/out/telem_ours_${TAG}_s${SEED}${MAP}.json
  LAP=$($RUN MU_MAP=$MAP SPEEDS_CRAWL=1 TAU_SPEED=1 $EXTRA TELEM_OUT=$OUT $PY -u \
    $SC/metaurban/render_3d_video.py --seed $SEED --maneuver --clear_spawn --headless --t_max 25 2>&1 | grep "lap done" | head -1)
  $PY - <<PYEOF >> $RES
import json
import numpy as np
try:
    o = np.array(json.load(open("$OUT"))); r = np.array(json.load(open("$REF")))
    def rms(T):
        tilt = np.hypot(T[:, 1], T[:, 2])
        return float(np.sqrt((np.abs(np.diff(tilt)) / 0.06) ** 2).mean() ** 0.5) if len(T) > 3 else 1e9
    def rr(T):
        tilt = np.hypot(T[:, 1], T[:, 2]); rate = np.abs(np.diff(tilt)) / 0.06
        return float(np.sqrt((rate ** 2).mean()))
    lap = """$LAP"""
    reach = "reached=True" in lap; coll = "collided=True" in lap
    print(f"s$SEED$MAP gap={rr(o)-rr(r):+.0f} reach={int(reach)} coll={int(coll)}")
except Exception as e:
    print(f"s$SEED$MAP ERROR {type(e).__name__}")
PYEOF
done
echo "== $TAG 聚合 =="; cat $RES
$PY - <<PYEOF
vals = []
for l in open("$RES"):
    if "gap=" in l:
        vals.append(float(l.split("gap=")[1].split()[0]))
print(f"[$TAG] 平均姿态差距 {sum(vals)/len(vals):+.0f}°/s over {len(vals)} 靶")
PYEOF
