#!/bin/bash
# ego_forest.sh — MIGHTY 启动件的逐字副本(场景在本库 ros/worlds/),唯一功能差异:
# 规划节点 mighty_node → 咱们的 ego_node。用法和他们的 run_sim.py 一模一样:
#   ./ops/ego_forest.sh --env easy_forest
#   ./ops/ego_forest.sh --env hard_forest --goal 90 0 3
# 副本三件:ros/run_sim.py(pane1/4 指向本库 launch)/ ros/base_sando.launch.py
# (world 取 ros/worlds/)/ ros/onboard_ego.launch.py(仅换 planner 节点)。
set -e
MU="$(cd "$(dirname "$0")/.." && pwd)"
WS="${MIGHTY_WS:-/media/boxuan/Data2/projects/MIGHTY/mighty_ws}"
export DISPLAY="${DISPLAY:-:1}"
cd "$MU/ros"
exec python3 run_sim.py --mode gazebo -s "$WS/install/setup.bash" "$@"
