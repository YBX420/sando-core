#!/usr/bin/env bash
# psm_run — run the full planner x safety matrix headless (GT movers, no d435i), 8 cells in PARALLEL fresh
# processes (each its own CSV -> no append race, EGO grid leak isolated), then merge + print the table.
set -u
cd "$(dirname "$0")/.."
source ~/miniconda3/etc/profile.d/conda.sh && conda activate sando
SPEEDS="${1:-2,3,4}"; SEEDS="${2:-0-19}"; NEP="${3:-6}"
mkdir -p out/conformal
rm -f out/conformal/psm_*.csv out/conformal/planner_safety.csv
pids=()
for pl in ego quintic septic bspline; do
  for sf in off on; do
    python -u metaurban/planner_safety_matrix.py --planner "$pl" --safety "$sf" \
      --seeds "$SEEDS" --n_ep "$NEP" --speeds "$SPEEDS" \
      --csv "out/conformal/psm_${pl}_${sf}.csv" > "logs/psm_${pl}_${sf}.log" 2>&1 &
    pids+=($!)
  done
done
echo "[psm_run] launched ${#pids[@]} cells (speeds=$SPEEDS seeds=$SEEDS n_ep=$NEP); waiting ..."
fail=0
for p in "${pids[@]}"; do wait "$p" || fail=$((fail+1)); done
echo "[psm_run] cells done (failures=$fail). merging ..."

# merge all per-cell CSVs into one + print the aggregate table
python - <<'PY'
import csv, glob, collections, os
rows = []
for f in sorted(glob.glob('out/conformal/psm_*.csv')):
    rows += list(csv.DictReader(open(f)))
if not rows:
    print('[psm_run] NO ROWS'); raise SystemExit(1)
fields = ['planner','safety','seed','ep','max_vel','eps','reached','collided','min_clr','time_s','max_z','n_hold','dynamics']
with open('out/conformal/planner_safety.csv','w',newline='') as fh:
    w = csv.DictWriter(fh, fieldnames=fields); w.writeheader()
    for r in rows: w.writerow({k:r.get(k) for k in fields})
import statistics as st
ag = collections.defaultdict(lambda:[0,0,0,[]])
for r in rows:
    k=(r['planner'],r['safety']); ag[k][0]+=1; ag[k][1]+=int(r['collided']); ag[k][2]+=int(r['reached']=='True')
    if r['min_clr'] not in (None,'','None'): ag[k][3].append(float(r['min_clr']))
print(f"\n[psm_run] merged {len(rows)} rows -> out/conformal/planner_safety.csv\n")
print(f"{'planner':8} {'safety':6} {'n':>4} {'collide':>12} {'reach':>11} {'clr_med':>8}")
for k in sorted(ag):
    n,c,re,cl = ag[k]
    print(f"{k[0]:8} {k[1]:6} {n:4} {c:4} ({100*c/n:3.0f}%) {re:5} ({100*re/n:3.0f}%) {st.median(cl) if cl else float('nan'):8.3f}")
bad=[k for k in ag if k[1]=='on' and ag[k][1]>0]
print('\nSANITY:', ('FAIL safety=on collisions: '+str(bad)) if bad else 'PASS — every safety=on cell is 0-collision')
PY
echo "[psm_run] done -> out/conformal/planner_safety.csv"
