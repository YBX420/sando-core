#!/bin/bash
# 字节回归:改 replay/感知/决策后必跑,默认路径逐字节比对基线
SCRATCH=/tmp/claude-1000/-media-boxuan-Data2-projects-sando-py/329f89a2-3fec-48e7-afb8-0ce7cda0efc2/scratchpad
SC=/media/boxuan/Data2/projects/sando_py/sando-core
cd /media/boxuan/Data2/projects/metaurban
env PYTHONPATH=/media/boxuan/Data2/projects/metaurban \
  ~/miniconda3/envs/metaurban/bin/python $SC/metaurban/regress_frozen_ours.py "$@"
