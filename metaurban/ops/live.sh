#!/bin/bash
# live.sh — 一键实时观看(RViz 式,replay 面部署同款决策栈)。
#   ./ops/live.sh scenarios/bench/street_busy_s0.json          # V11+逃生树,1 倍速,带语义底图
#   ./ops/live.sh scenarios/full/gauntlet.json --speed 2       # 任何 live_view 参数原样透传
#   ./ops/live.sh <场景> --stack none                          # 冻结 v1 基线
# 首次看某张新地图会自动建底片(~40 s,永久缓存)。
set -e
cd "$(dirname "$0")/.."
exec env DISPLAY="${DISPLAY:-:1}" ~/miniconda3/envs/metaurban/bin/python live_view.py --build_film "$@"
