"""Parallel headless benchmark launcher — run eval_batch.py across K processes (near-linear speedup).

The headless eval is pure CPU/RAM (use_render=False, image_observation=False => MetaUrban opens NO GL context),
so disjoint-seed workers scale almost linearly. eval_batch.py shards cleanly: scenario seed = (seed0+ep)%num_scenarios
and the route RNG = seed0*1000+ep, so giving each worker a disjoint seed0 base yields disjoint maps AND routes.
Each worker is pinned to BLAS/OMP=1 thread to avoid oversubscription.

Usage (from anywhere; workers cd into the metaurban repo themselves):
  python eval_parallel.py --n 200 --k 8 --t_max 30
  python eval_parallel.py --n 96 --k 6 --reset_every 2
"""
import os, json, time, argparse, subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
MU_ROOT = "/media/boxuan/Data21/projects/metaurban"
PY = os.path.expanduser("~/miniconda3/envs/metaurban/bin/python")

ap = argparse.ArgumentParser()
ap.add_argument("--n", type=int, default=200, help="total episodes")
ap.add_argument("--k", type=int, default=8, help="parallel workers (= physical cores is the sweet spot)")
ap.add_argument("--t_max", type=float, default=30.0)
ap.add_argument("--reset_every", type=int, default=1)
ap.add_argument("--seed_base", type=int, default=0)
ap.add_argument("--map", type=str, default="X")
ap.add_argument("--density", type=str, default="dense", choices=["sparse", "med", "dense"])
ap.add_argument("--animals", type=int, default=2)
ap.add_argument("--route_len", type=float, default=0.0)
ap.add_argument("--stratify", action="store_true", help="cycle density sparse/med/dense across workers (Mondrian benchmark)")
ap.add_argument("--out", type=str, default="out/eval_parallel.json")
args = ap.parse_args()
TIERS = ["sparse", "med", "dense"]

per = [args.n // args.k + (1 if i < args.n % args.k else 0) for i in range(args.k)]
os.makedirs(os.path.join(HERE, "out"), exist_ok=True)

env = dict(os.environ)
for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
          "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    env[k] = "1"                                   # avoid BLAS/OMP oversubscription across K workers
env.setdefault("DISPLAY", ":1")

procs = []
t0 = time.perf_counter()
for w in range(args.k):
    if per[w] == 0:
        continue
    seed0 = args.seed_base + w * 100000            # disjoint seed bases => disjoint maps AND routes
    density = TIERS[w % 3] if args.stratify else args.density
    outw = os.path.join(HERE, "out", f"_shard_{w}.json")
    cmd = [PY, os.path.join(HERE, "eval_batch.py"), "--n", str(per[w]), "--seed0", str(seed0),
           "--t_max", str(args.t_max), "--reset_every", str(args.reset_every), "--out", outw,
           "--map", args.map, "--density", density, "--animals", str(args.animals),
           "--route_len", str(args.route_len)]
    logf = open(os.path.join(HERE, "out", f"_shard_{w}.log"), "w")
    procs.append((w, outw, subprocess.Popen(cmd, cwd=MU_ROOT, env=env, stdout=logf, stderr=subprocess.STDOUT)))
    print(f"[par] worker {w}: {per[w]} eps, seed0={seed0}, density={density}, map={args.map}", flush=True)

for w, outw, p in procs:
    p.wait()
    print(f"[par] worker {w} done (exit {p.returncode})", flush=True)

# ---- merge shard results ----
allres = []
for w, outw, p in procs:
    try:
        d = json.load(open(outw))
        allres.extend(d["results"])
    except Exception as e:
        print(f"[par] WARN shard {w} unreadable: {e}", flush=True)

n = len(allres)
def rate(key): return sum(1 for r in allres if r.get(key)) / max(n, 1)
collide_by_class = {}
for r in allres:
    if r.get("collided") and r.get("collide_class"):
        collide_by_class[r["collide_class"]] = collide_by_class.get(r["collide_class"], 0) + 1
summary = dict(n=n, k=args.k, wall_s=round(time.perf_counter() - t0, 1),
               reached=rate("reached"), collided=rate("collided"), stuck=rate("stuck"), timeout=rate("timeout"),
               collide_by_class=collide_by_class)
outp = args.out if os.path.isabs(args.out) else os.path.join(HERE, args.out)
json.dump(dict(summary=summary, results=allres), open(outp, "w"), indent=2)
print("\n[par] ===== AGGREGATE =====", flush=True)
print(f"[par] n={n}  k={args.k}  reached={summary['reached']*100:.0f}%  collided={summary['collided']*100:.0f}%  "
      f"stuck={summary['stuck']*100:.0f}%  timeout={summary['timeout']*100:.0f}%  wall={summary['wall_s']}s", flush=True)
print(f"[par] collisions by model class: {collide_by_class}", flush=True)
print(f"[par] wrote {outp}", flush=True)
