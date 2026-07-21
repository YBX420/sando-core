#!/bin/bash
# final100 生成驱动(07-21 契约):单 worker,每 5 个 slot 一个新解释器(MCE 最多重做当前场景),
# git 锁在 final100_gen 内部执行。用法: ops/final100.sh <plan.json>
set -u
PLAN=${1:?plan.json}
SC=/media/boxuan/Data2/projects/sando_py/sando-core
cd /media/boxuan/Data2/projects/metaurban
PY=~/miniconda3/envs/metaurban/bin/python
EV="PYTHONPATH=/media/boxuan/Data2/projects/metaurban DISPLAY=:1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1"
LOGD=$(dirname "$PLAN")/final100_logs
mkdir -p "$LOGD"
FAIL=0
for ((i=0; i<100; i+=5)); do
  j=$((i+4))
  if ! env $EV LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6 \
       $PY $SC/metaurban/final100_gen.py --plan "$PLAN" --slots $i-$j \
       > "$LOGD/slots_${i}_${j}.log" 2>&1; then
    FAIL=$((FAIL+1)); echo "[final100] chunk $i-$j FAILED (see log; resume re-runs unfinished slots)" >&2
  fi
done
echo "[final100] chunks failed=$FAIL; receipts under scenarios/final100/"
python3 - <<EOF
import glob, json
ok = fail = 0
for rp in glob.glob("$SC/metaurban/scenarios/final100/*.receipt.json"):
    r = json.load(open(rp))
    ok += r.get("status") == "ok"; fail += r.get("status") == "FAILED"
print(f"[final100] slots ok={ok} failed={fail} (registered 100)")
EOF
