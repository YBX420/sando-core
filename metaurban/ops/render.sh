#!/bin/bash
# 3D 渲染(产品渲染器,唯一合法产线)。用法: ops/render.sh [ours|native] [seed] [额外 render_3d_video 参数...]
ARM=${1:-ours}; SEED=${2:-7}; shift 2 2>/dev/null || shift $# 
SC=/media/boxuan/Data2/projects/sando_py/sando-core
FLAG=$([ "$ARM" = native ] && echo "--ego" || echo "--maneuver")
cd /media/boxuan/Data2/projects/metaurban && env DISPLAY=:1 \
  PYTHONPATH=/media/boxuan/Data2/projects/metaurban \
  LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6 \
  ~/miniconda3/envs/metaurban/bin/python -u $SC/metaurban/render_3d_video.py \
  --seed "$SEED" $FLAG --clear_spawn --mp4 --t_max 25 --w 560 --h 350 "$@" \
  && mv $SC/metaurban/out/drone_3d.mp4 $SC/metaurban/out/drone_3d_${ARM}_s${SEED}.mp4 \
  && echo "→ out/drone_3d_${ARM}_s${SEED}.mp4"
