#!/bin/bash
# M1b 全池裁决(塔菲大人 2026-07-17):render-face headless,seed 0-19 三臂
#   base = THIN 基线 | k10 = ETA_FEED=2 K=1.0(平台版)| k05 = K=0.5(提速候选)
#   + oracle 零伤抽查(seed 3/7/11/15/19,ORACLE_THIN ± ETA_FEED=2)
# 断点续跑:每集独立日志,已有 "lap done" 即跳过(5800X MCE 崩机保险)。
# 用法: ops/m1b_pool.sh <arm>   arm ∈ base|k10|k05|orabase|oraeta
set -u
SC=/media/boxuan/Data2/projects/sando_py/sando-core
POOL=$SC/metaurban/out/m1b_pool; mkdir -p "$POOL"
ARM=${1:?arm required: base|k10|k05|orabase|oraeta}

case "$ARM" in
  base)    ENVS="THIN=1";                              SEEDS=$(seq 0 19) ;;
  k10)     ENVS="THIN=1 ETA_FEED=2 ETA_K=1.0";         SEEDS=$(seq 0 19) ;;
  k05)     ENVS="THIN=1 ETA_FEED=2 ETA_K=0.5";         SEEDS=$(seq 0 19) ;;
  orabase) ENVS="ORACLE_THIN=1";                       SEEDS="3 7 11 15 19" ;;
  oraeta)  ENVS="ORACLE_THIN=1 ETA_FEED=2 ETA_K=1.0";  SEEDS="3 7 11 15 19" ;;
  *) echo "unknown arm $ARM"; exit 1 ;;
esac

for S in $SEEDS; do
  LOG="$POOL/${ARM}_s${S}.log"
  if grep -q "lap done" "$LOG" 2>/dev/null; then echo "[pool] skip $ARM s$S (done)"; continue; fi
  echo "[pool] run $ARM s$S ..."
  cd /media/boxuan/Data2/projects/metaurban && env DISPLAY=:1 \
    PYTHONPATH=/media/boxuan/Data2/projects/metaurban \
    LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6 \
    $ENVS \
    ~/miniconda3/envs/metaurban/bin/python -u $SC/metaurban/render_3d_video.py \
    --seed "$S" --maneuver --clear_spawn --headless --t_max 40 --w 560 --h 350 \
    > "$LOG" 2>&1
  grep -h "lap done" "$LOG" | sed "s/^/[pool $ARM s$S] /"
done
echo "[pool] ARM $ARM COMPLETE"
