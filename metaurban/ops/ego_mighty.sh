#!/bin/bash
# ego_mighty.sh — 在 MIGHTY 的 Gazebo(easy_forest 等)里跑咱们的 EGO-Planner。
# 用法:
#   终端1(原样起 MIGHTY 仿真):
#     cd /media/boxuan/Data2/projects/MIGHTY/mighty_ws/src/mighty && python3 scripts/run_sim.py \
#       --mode gazebo -s /media/boxuan/Data2/projects/MIGHTY/mighty_ws/install/setup.bash --env easy_forest
#   终端2:
#     ./ops/ego_mighty.sh            # 杀掉 /NX01/mighty_node,换上 ego_node(接管 /NX01/goal)
# goal_sender 照发 /NX01/term_goal(105,0),ego_node 会去追。
set -e
MU="$(cd "$(dirname "$0")/.." && pwd)"
WS="${MIGHTY_WS:-/media/boxuan/Data2/projects/MIGHTY/mighty_ws}"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
pkill -f "lib/mighty/might[y]" 2>/dev/null && echo "[ego_mighty] killed mighty_node" || true
exec /usr/bin/python3 "$MU/ros/ego_node.py" "$@"
