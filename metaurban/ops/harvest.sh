#!/bin/bash
# 残差收割一折(4 片并行,每片新解释器)。用法: ops/harvest.sh <mode: foldB7|test7|designC|...>
M=$1
SC=/media/boxuan/Data2/projects/sando_py/sando-core
cd /media/boxuan/Data2/projects/metaurban
pids=""
for w in 0 1 2 3; do
  env PYTHONPATH=/media/boxuan/Data2/projects/metaurban \
    ~/miniconda3/envs/metaurban/bin/python $SC/metaurban/harvest_v2.py --mode $M --shard $w --nshard 4 \
    > /tmp/harv_${M}_$w.log 2>&1 &
  pids="$pids $!"
done
wait $pids
ls $SC/metaurban/out/conformal/harvest_${M}_v2_s*.npy | wc -l
