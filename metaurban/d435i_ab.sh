#!/usr/bin/env bash
# d435i_ab — headless A/B: ours certified go-around with GT perception vs with REAL D435i depth, across seeds.
# Proves the D435i perception path now reaches GT-similar (reached + collision-free + comparable clearance).
# Each run is one render-to-depth closed loop (no mp4). Sequential (single GPU/:1 display).
set -u
cd "$(dirname "$0")/.."
source ~/miniconda3/etc/profile.d/conda.sh && conda activate metaurban
export PYTHONPATH=/media/boxuan/Data2/projects/metaurban DISPLAY=:1
export LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6
export MAN_TRACE=0 D435I_DEBUG=0
SEEDS="${1:-1 5 9}"
TMAX="${2:-22}"
OUT=logs/d435i_ab_summary.txt
: > "$OUT"
for s in $SEEDS; do
  for mode in gt d435i; do
    flag=""; [ "$mode" = "d435i" ] && flag="--d435i"
    log="logs/ab_${mode}_s${s}.log"
    timeout 420 python -u metaurban/render_3d_video.py --ego --maneuver $flag --seed "$s" --clear_spawn --t_max "$TMAX" > "$log" 2>&1
    line=$(grep "lap done" "$log" | tail -1)
    astar=$(grep -c 'a star error' "$log")
    printf "seed=%s mode=%-5s | %s  [astar_fail=%s]\n" "$s" "$mode" "${line#*lap done. }" "$astar" | tee -a "$OUT"
  done
done
echo "=== done -> $OUT ==="
