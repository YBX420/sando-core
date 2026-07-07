#!/bin/bash
# EGO-on-MetaUrban headless 版本测试器(塔菲大人 2026-07-08)
# 用法:  ./test_ego_mu.sh <版本标签> [KEY=VAL ...]
# 例子:  ./test_ego_mu.sh V1_canonical
#        ./test_ego_mu.sh V4_best DELTA_OVR=0.05 YOUNG_TTL=2 CALIB_FILE=$PWD/../out/conformal/calib_young2.json
# 每次调用跑 10 集(固定场景序列,版本间可比),结果追加进台账并打印全版本对比表。
set -e
SC="$(cd "$(dirname "$0")" && pwd)"
TAG="$1"; shift || true
cd /media/boxuan/Data2/projects/metaurban
env PYTHONPATH=/media/boxuan/Data2/projects/metaurban CALIB_V2=1 CALIB_EPS=0.10 DECIDE=v2 "$@" \
  "$HOME/miniconda3/envs/metaurban/bin/python" "$SC/ego_mu_bench.py" --tag "$TAG" --episodes "${EPISODES:-10}" \
  2>&1 | grep -E "^\[$TAG\]"
echo
echo "================ 版本进化表(MetaUrban headless,EGO 线,10集/版本)================"
"$HOME/miniconda3/envs/metaurban/bin/python" - <<PY
import json, collections
rows = [json.loads(l) for l in open("$SC/out/ppo_eval/ego_mu_ledger.jsonl")]
by = collections.OrderedDict()
for r in rows:
    by.setdefault(r["tag"], []).append(r)
print(f"{'版本':24s} {'干净':>6s} {'到达':>6s} {'碰撞':>4s} {'evade总':>7s} {'clr中位':>7s}")
for tag, rs in by.items():
    n = len(rs)
    import statistics as st
    print(f"{tag:24s} {sum(r['clean'] for r in rs):3d}/{n:<2d} {sum(r['reached'] for r in rs):3d}/{n:<2d} "
          f"{sum(r['collided'] for r in rs):4d} {sum(r['evade'] for r in rs):7d} "
          f"{st.median([r['min_clr'] for r in rs]):7.2f}")
PY
