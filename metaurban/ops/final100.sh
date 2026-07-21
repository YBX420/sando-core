#!/bin/bash
# final100 生成驱动(07-21 契约):单 worker,每 5 个 slot 一个新解释器(MCE 最多重做当前场景);
# git/runtime 锁在 final100_gen 内部执行;**100/100 全部 ok 才算成功,否则非零退出**(fail-closed:
# 少于 60 cal / 40 test 的数据集绝不静默放行)。用法: ops/final100.sh <plan.json>
set -u
PLAN=${1:?plan.json}
SC=/media/boxuan/Data2/projects/sando_py/sando-core
cd /media/boxuan/Data2/projects/metaurban
PY=~/miniconda3/envs/metaurban/bin/python
EV="PYTHONPATH=/media/boxuan/Data2/projects/metaurban DISPLAY=:1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1"
LOGD=$(dirname "$PLAN")/final100_logs
mkdir -p "$LOGD"
for ((i=0; i<100; i+=5)); do
  j=$((i+4))
  env $EV LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6 \
      $PY $SC/metaurban/final100_gen.py --plan "$PLAN" --slots $i-$j \
      > "$LOGD/slots_${i}_${j}.log" 2>&1 \
    || echo "[final100] chunk $i-$j FAILED (see log; resume re-runs unfinished slots)" >&2
done
$PY - "$PLAN" "$SC" <<'EOF'
import json, os, sys, hashlib
plan = json.load(open(sys.argv[1])); sc = sys.argv[2]
ns = "final100_draft" if plan.get("draft") else "final100"
ok = failed = missing = 0
for slot in plan["slots"]:
    rp = os.path.join(sc, "metaurban", "scenarios", ns,
                      os.path.basename(slot["out"]) + ".receipt.json")
    if not os.path.exists(rp):
        missing += 1; continue
    r = json.load(open(rp))
    if r.get("status") == "ok" and r.get("plan_sha") == plan["plan_sha"]:
        ok += 1
    else:
        failed += 1
print(f"[final100] slots ok={ok} failed={failed} missing={missing} / registered 100")
if ok != 100:
    print("[final100] FAIL-CLOSED: the campaign is NOT complete -- fewer than 60 cal / 40 test "
          "scenarios exist; fix and resume, never harvest a partial benchmark", flush=True)
    sys.exit(3)
EOF
