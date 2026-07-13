#!/bin/bash
# live3d.sh — 一键 3D 实时观看(唯一合法 3D 产线 render_3d_video.py 的 --serve 模式,MJPEG 推流)。
#   ./ops/live3d.sh                                            # seed 7 街景,浏览器开 http://localhost:8089
#   ./ops/live3d.sh --seed 3 --w 440 --h 280                   # 参数原样透传(重复旗标后者生效)
#   ./ops/live3d.sh --scenario scenarios/full/gauntlet.json    # 场景工作台 JSON 进 3D
# ⚠️ 测量面诚实:渲染面跑渲染器自带锦标赛 + 一代 calib(v2/v3/TAU_SPEED 空挡)≠ replay 面 V11 栈。
#    看行为/出成片用这里;报数字标"渲染面"。软件 GL 机器别用 --live(黑屏),--serve 就是为它设的。
set -e
SC="$(cd "$(dirname "$0")/../.." && pwd)"
cd /media/boxuan/Data2/projects/metaurban
exec env DISPLAY="${DISPLAY:-:1}" \
  PYTHONPATH=/media/boxuan/Data2/projects/metaurban \
  LD_PRELOAD="$HOME/miniconda3/envs/sando/lib/libstdc++.so.6" \
  HUMANOID_NO_IDLE="${HUMANOID_NO_IDLE:-60}" \
  "$HOME/miniconda3/envs/metaurban/bin/python" -u \
  "$SC/metaurban/render_3d_video.py" --maneuver --clear_spawn --serve --loop_scene \
  --seed 7 --w 560 --h 350 "$@"
