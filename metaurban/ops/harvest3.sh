#!/bin/bash
# 原子收割驱动(07-20 契约):单 worker、每 episode 新解释器、BLAS 单线程、版本化 run 目录、
# checksum resume、merge 前作业集全等校验。用法: ops/harvest3.sh <mode: pilot12|design69> <run_tag>
set -u
M=${1:?mode}; TAG=${2:?run_tag}
SC=/media/boxuan/Data2/projects/sando_py/sando-core
RUN=$SC/metaurban/out/harvest_runs/${M}_${TAG}
mkdir -p "$RUN"
cd /media/boxuan/Data2/projects/metaurban
PY=~/miniconda3/envs/metaurban/bin/python
EV="PYTHONPATH=/media/boxuan/Data2/projects/metaurban OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1"
env $EV $PY $SC/metaurban/harvest_v3.py --mode $M --run "$RUN" --plan
N=$(python3 -c "import json;print(len(json.load(open('$RUN/jobs.json'))['jobs']))")
FAIL=0
for ((i=0; i<N; i++)); do
  if ! env $EV $PY $SC/metaurban/harvest_v3.py --mode $M --run "$RUN" --job $i \
       > "$RUN/logs/job_$i.log" 2>&1; then
    FAIL=$((FAIL+1)); echo "[harvest3] job $i FAILED (no receipt; resume will retry)" >&2
  fi
done
echo "[harvest3] jobs=$N failed=$FAIL"
env $EV $PY $SC/metaurban/harvest_v3.py --mode $M --run "$RUN" --merge
