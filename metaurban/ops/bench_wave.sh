#!/bin/bash
# replay 面 4 路并行波(345 集)。用法: ops/bench_wave.sh <标签> [KEY=VAL ...]
TAG=$1; shift
SC=/media/boxuan/Data2/projects/sando_py/sando-core
cd /media/boxuan/Data2/projects/metaurban
pids=""
for w in 0 1 2 3; do
  env PYTHONPATH=/media/boxuan/Data2/projects/metaurban DECIDE=v2 "$@" \
    ~/miniconda3/envs/metaurban/bin/python $SC/metaurban/bench_shard.py --config ours_v2 \
    --shard $w --out $SC/metaurban/out/ppo_eval/ego_bench_v2era/rows_${TAG}_w$w.jsonl >/dev/null 2>&1 &
  pids="$pids $!"
done
wait $pids
~/miniconda3/envs/metaurban/bin/python - <<PY
import glob, json
import numpy as np
rows = []
for f in glob.glob("$SC/metaurban/out/ppo_eval/ego_bench_v2era/rows_${TAG}_w*.jsonl"):
    rows += [json.loads(l) for l in open(f) if "error" not in l]
n = len(rows)
print(f"[$TAG] n={n} 干净={100*sum(r['clean'] for r in rows)/n:.1f}% 到达={100*sum(r['reached'] for r in rows)/n:.1f}% "
      f"撞={sum(r['collided'] for r in rows)} evade={sum(r['emerg'] for r in rows)} t={np.median([r['t'] for r in rows]):.1f}")
PY
