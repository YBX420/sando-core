#!/bin/bash
# M2-1 移动横穿者考场(2026-07-17):掐点参数按基线臂实测均速反推,横穿者真相遇。
# 用法: ops/crosser_exam.sh <seed> <N crossers> <arm: base|k10|st|orabase> [extra env "K=V ..."]
# vcru = 路线长 / 基线臂实测 t_goal(读 out/m1b_pool/base_s<seed>.log,没有就先跑一发基线)。
set -u
SC=/media/boxuan/Data2/projects/sando_py/sando-core
POOL=$SC/metaurban/out/m1b_pool${POOL_TAG:-}
EXAM=$SC/metaurban/out/crosser_exam${POOL_TAG:-}; mkdir -p "$EXAM"   # face-versioned exam ledger (M8)
SEED=${1:?seed}; N=${2:?n crossers}; ARM=${3:?arm}; EXTRA=${4:-}

BASELOG=$POOL/base_s${SEED}.log
if ! grep -q "lap done" "$BASELOG" 2>/dev/null; then
  echo "[exam] no pool baseline for s$SEED -- run: ops/m1b_pool.sh base" >&2; exit 1
fi
LEN=$(grep -o "route len~[0-9]*m" "$BASELOG" | grep -o "[0-9]*" | head -1)
TG=$(grep -o "t_goal=[0-9.]*" "$BASELOG" | grep -o "[0-9.]*" | head -1)
VCRU=$(python3 -c "print(f'{$LEN/$TG:.2f}')")
echo "[exam] s$SEED baseline: len=${LEN}m t_goal=${TG}s -> vcru=$VCRU"

case "$ARM" in
  base)    ENVS="THIN=1" ;;
  k10)     ENVS="THIN=1 ETA_FEED=2 ETA_K=1.0" ;;
  st)      ENVS="THIN=1 ST_SPEED=1" ;;
  stk10)   ENVS="THIN=1 ETA_FEED=2 ETA_K=1.0 ST_SPEED=1" ;;
  stc)     ENVS="THIN=1 ST_COMMIT=1" ;;
  stck10)  ENVS="THIN=1 ETA_FEED=2 ETA_K=1.0 ST_COMMIT=1" ;;
  orabase) ENVS="ORACLE_THIN=1" ;;
  orastc)  ENVS="ORACLE_THIN=1 ST_COMMIT=1" ;;
  *) echo "unknown arm $ARM"; exit 1 ;;
esac

LOG=$EXAM/${ARM}_s${SEED}_n${N}.log
cd /media/boxuan/Data2/projects/metaurban && env DISPLAY=:1 \
  PYTHONPATH=/media/boxuan/Data2/projects/metaurban \
  LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6 \
  $ENVS $EXTRA EGO_ADV_N=$N EGO_ADV_CROSS_VCRU=$VCRU \
  ~/miniconda3/envs/metaurban/bin/python -u $SC/metaurban/render_3d_video.py \
  --seed "$SEED" --maneuver --clear_spawn --headless --t_max 40 --w 560 --h 350 \
  > "$LOG" 2>&1
grep -h "lap done" "$LOG" | sed "s/^/[exam $ARM s$SEED n$N] /"
