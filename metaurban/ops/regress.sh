#!/bin/bash
# 字节回归:改 replay/感知/决策后必跑。用法:
#   ops/regress.sh - out.json                    # 当前树跑 12 键(场景x种子)哈希
#   ops/regress.sh <shadow_dir> out.json         # shadow 目录里的 kf_tracker/perception 优先(改前对照)
# 金哈希:metaurban/regress_golden/<commit>.json(每个过关的刀提交一份);比对:
#   python3 -c "import json,sys; a,b=[json.load(open(f)) for f in sys.argv[1:3]]; d=[k for k in a if a[k]!=b.get(k)]; print('DIFFS:',len(d),d[:5])" metaurban/regress_golden/<sha>.json out.json
set -e
if [ -z "$2" ]; then echo "usage: regress.sh <shadow_dir|-> <out.json>"; exit 2; fi
SC=/media/boxuan/Data2/projects/sando_py/sando-core
cd /media/boxuan/Data2/projects/metaurban
env PYTHONPATH=/media/boxuan/Data2/projects/metaurban \
  ~/miniconda3/envs/metaurban/bin/python $SC/metaurban/regress_frozen_ours.py "$@"
