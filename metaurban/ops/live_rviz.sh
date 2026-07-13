#!/bin/bash
# live_rviz.sh — 一键 RViz 实时观看(启动秒级,原生 GPU 60fps,不走浏览器)。
#   ./ops/live_rviz.sh scenarios/full/gauntlet.json          # V11+逃生树,1 倍速
#   ./ops/live_rviz.sh scenarios/full/crossers.json --speed 2
# 结构:replay 栈(conda,live_view --serve 出 /state JSON)⇄ 桥(系统 python+rclpy,
# ros/state_to_rviz.py → /sando/markers)⇄ rviz2。conda 与 ROS 零混装。
set -e
MU="$(cd "$(dirname "$0")/.." && pwd)"
SCN="${1:-scenarios/full/sparse_field.json}"; shift 2>/dev/null || true
source /opt/ros/humble/setup.bash

env DISPLAY="${DISPLAY:-:1}" "$HOME/miniconda3/envs/metaurban/bin/python" \
  "$MU/live_view.py" "$SCN" --dummy --serve --loop --build_film "$@" &
LV=$!
/usr/bin/python3 "$MU/ros/state_to_rviz.py" &
BR=$!
trap 'kill $LV $BR 2>/dev/null' EXIT
until curl -s -m 1 -o /dev/null http://localhost:8090/state; do sleep 1; done
# rviz2 in the foreground: closing the RViz window tears down stack + bridge via the trap
DISPLAY="${DISPLAY:-:1}" rviz2 -d "$MU/ros/sando.rviz"
