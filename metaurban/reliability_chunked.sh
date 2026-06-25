#!/usr/bin/env bash
# reliability_chunked — run the reliability sweep in CHUNKS, one fresh process per chunk, appending every episode
# to a master CSV. A fresh process per chunk sidesteps the EGOPlanner C++ grid memory growth (~8 MB/episode) that
# otherwise swaps + hangs a single long process around ep ~250. Resumable + scales to any episode count.
#
# Usage:  bash metaurban/reliability_chunked.sh [N_EP_PER_CHUNK] [EPS] [SEEDS]
#   default: 50 episodes/chunk, eps=0.01, seeds 0-19 (one chunk per seed -> 1000 episodes)
#   to scale: raise SEEDS coverage (harvest more) or pass more seeds; each chunk stays ~<1 GB.
set -u
cd "$(dirname "$0")/.."
source ~/miniconda3/etc/profile.d/conda.sh && conda activate sando 2>/dev/null
export LD_LIBRARY_PATH=/home/boxuan/gurobi1103/linux64/lib:${LD_LIBRARY_PATH:-}

N_EP=${1:-50}
EPS=${2:-0.01}
SEEDS_SPEC=${3:-0-19}
CSV=out/conformal/reliability_full.csv
rm -f "$CSV"   # fresh master csv (must NOT pre-create it, or the first chunk skips the header)

# expand seed spec a-b or csv list
if [[ "$SEEDS_SPEC" == *-* && "$SEEDS_SPEC" != *,* ]]; then
  SEEDS=$(seq "${SEEDS_SPEC%-*}" "${SEEDS_SPEC#*-}")
else
  SEEDS=$(echo "$SEEDS_SPEC" | tr ',' ' ')
fi

echo "[chunked] N_EP/chunk=$N_EP eps=$EPS seeds=$SEEDS_SPEC -> $CSV"
for sd in $SEEDS; do
  [ -f "out/conformal/traj_seed${sd}.npz" ] || { echo "  seed $sd: no data, skip"; continue; }
  # fresh process per seed/chunk -> memory fully released on exit
  python -u metaurban/reliability.py --eps "$EPS" --seeds "$sd" --n_ep "$N_EP" --csv "$CSV" \
       > "logs/rel_chunk_${sd}.log" 2>&1
  rc=$?
  rows=$(($(wc -l < "$CSV") - 1))
  echo "  seed $sd: rc=$rc  cumulative rows=$rows"
done

echo "[chunked] aggregating $CSV ..."
python3 - "$CSV" <<'PY'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
n = len(rows)
coll = sum(1 for r in rows if r["collided"] == "1")
reach = sum(1 for r in rows if r["reached"] == "1")
clrs = [float(r["min_clr"]) for r in rows if r["min_clr"] not in ("", None)]
rate = coll / n if n else float("nan")
ub = 3.0 / n if coll == 0 and n else None
print(f"\n=== RELIABILITY (chunked, {n} episodes) ===")
print(f"  collisions: {coll}/{n}  collision rate={rate:.6f}  success={ (1-rate)*100:.4f}%")
print(f"  reached: {reach}/{n} ({reach/n*100:.1f}%)   min clearance over all = {min(clrs):.3f} m")
if ub is not None:
    print(f"  0 collisions in {n} -> distribution-free 95%-CI collision rate <= {ub:.6f} (success >= {(1-ub)*100:.4f}%)")
    print(f"  for 99.999% (<=1e-5) need ~3e5 episodes; this driver scales there one fresh chunk at a time.")
PY
