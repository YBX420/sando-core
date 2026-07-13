#!/bin/bash
# ego_forest.sh — MIGHTY 启动件的逐字副本(场景在本库 ros/worlds/),唯一功能差异:
# 规划节点 mighty_node → 咱们的 ego_node。用法和他们的 run_sim.py 一模一样:
#   ./ops/ego_forest.sh --env easy_forest
#   ./ops/ego_forest.sh --env hard_forest --goal 90 0 3
# 环境 = ros/worlds/ 里任意 world 文件名(31 张全接入,`ls ros/worlds/`):
#   即用 24:easy/medium/hard/easy_high/dynamic_forest、forest/forest2/forest3(_with_walls)、
#     ACL_office(_with_backpack)、empty(_wo_ground)、flight_space、flight_test_env、
#     simple_tunnel、static_uncertainty_test(2/3/4)、roi_map_test、ground_robot_forest、
#     quadruped_easy_forest、quadruped_forest3、finals_systems_prelim2
#   降级 7(模型缺失,场景加载不全):big_forest(_high_res)/forest0=上游即缺资产(无解);
#     office(willowgarage)/hospital(AWS)/tunnel(SubT)=需下载对应 gazebo 模型库到
#     GAZEBO_MODEL_PATH 后可用
# 注意:goal 默认 (105,0) 是给森林走廊设计的,office/tunnel 类环境记得自带 --goal。
# 副本三件:ros/run_sim.py(pane1/4 指向本库 launch)/ ros/base_sando.launch.py
# (world 取 ros/worlds/,env=world 名通用回落)/ ros/onboard_ego.launch.py(仅换 planner 节点)。
set -e
MU="$(cd "$(dirname "$0")/.." && pwd)"
WS="${MIGHTY_WS:-/media/boxuan/Data2/projects/MIGHTY/mighty_ws}"
export DISPLAY="${DISPLAY:-:1}"
cd "$MU/ros"
exec python3 run_sim.py --mode gazebo -s "$WS/install/setup.bash" "$@"
