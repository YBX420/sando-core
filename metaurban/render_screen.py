"""render_screen — screen MANY 3-D render scenarios for navigability, with a RELIABLE concurrency cap.

For each seed run ours (--maneuver) AND native (--ego) in render_3d_video (headless, no mp4), record
reach/collision/clearance/astar; then select the seeds where OURS succeeds (reach + 0 collision = navigable, not
static-blocked) and summarise the A/B on that fair set. ThreadPoolExecutor caps concurrency (bash wait/jobs/xargs
were unreliable / left orphans in the nohup'd context). seeds: env.reset(seed%20) = scene, seed*1000 = ROUTE, so
seeds>=20 are fresh scenarios.

Run: python metaurban/render_screen.py --seeds 0-59 -P 3 --t_max 20
"""
import os, re, csv, argparse, subprocess, collections
from concurrent.futures import ThreadPoolExecutor, as_completed

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MPY = os.path.expanduser("~/miniconda3/envs/metaurban/bin/python")
LOGDIR = os.path.join(REPO, "logs", "screen")
CSV = os.path.join(REPO, "out", "conformal", "render_screen.csv")
ENV = dict(os.environ, PYTHONPATH="/media/boxuan/Data2/projects/metaurban", DISPLAY=":1",
           LD_PRELOAD=os.path.expanduser("~/miniconda3/envs/sando/lib/libstdc++.so.6"))


def expand(spec):
    out = []
    for t in spec.split(","):
        if "-" in t:
            a, b = t.split("-"); out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(t))
    return out


def run_one(seed, mode, tmax):
    flag = ["--maneuver"] if mode == "ours" else []
    log = os.path.join(LOGDIR, f"{mode}_s{seed}.log")
    # --headless: same MetaUrban sim/planner/safety/real-quad dynamics, NO rendering -> CPU-only, ~34s, identical
    # lap-done numbers to the rendered run, so it parallelises freely and IS the render's scenario (just no pixels).
    cmd = [MPY, "-u", "metaurban/render_3d_video.py", "--ego", *flag, "--headless",
           "--seed", str(seed), "--clear_spawn", "--t_max", str(tmax)]
    try:
        with open(log, "w") as f:
            subprocess.run(cmd, cwd=REPO, env=ENV, stdout=f, stderr=subprocess.STDOUT, timeout=340)
    except subprocess.TimeoutExpired:
        pass
    txt = open(log).read() if os.path.exists(log) else ""
    line = ""
    for ln in txt.splitlines():
        if "lap done" in ln:
            line = ln
    g = lambda pat: (re.search(pat, line).group(1) if re.search(pat, line) else "NA")
    reached = g(r"reached=(\w+)"); coll = g(r"collided=(\w+)")
    clr = g(r"min_clr=(-?[0-9.]+)"); tg = g(r"t_goal=([0-9.]+|infs)")
    af = str(txt.count("a star error"))
    # per-class clearance -> separate the collision type: 撞人/撞车/撞动物 (each mover class) + 坠机/撞静态 (static).
    # The cert protects comparable-speed movers (ped/animal) perfectly; a vehicle 4x the drone's speed is the hard case.
    pc = {c: float(m.group(1)) for c in ("pedestrian", "vehicle", "animal", "static")
          for m in [re.search(rf"{c}:(-?[0-9.]+)", line)] if m}
    hit = lambda c: bool(c in pc and pc[c] < -1e-6)
    # DNF = no lap-done line at all (subprocess timed out / crashed mid-run, e.g. astar-spam stuck). Report it
    # EXPLICITLY as a failure rather than silently folding it into 'non-reach' -> the safe-completion denominator
    # stays the full attempted set and a stuck/timed-out run is never miscounted as a safe outcome.
    dnf = (line == "")
    return dict(seed=seed, mode=mode, reached=reached, collided=coll, min_clr=clr, astar_fail=af, t_goal=tg, dnf=dnf,
                hit_ped=hit("pedestrian"), hit_veh=hit("vehicle"), hit_animal=hit("animal"), hit_static=hit("static"),
                ped_clr=("" if "pedestrian" not in pc else f"{pc['pedestrian']:.3f}"),
                veh_clr=("" if "vehicle" not in pc else f"{pc['vehicle']:.3f}"),
                animal_clr=("" if "animal" not in pc else f"{pc['animal']:.3f}"),
                static_clr=("" if "static" not in pc else f"{pc['static']:.3f}"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="0-99")
    ap.add_argument("-P", type=int, default=10)   # --headless is CPU-only (no GPU), so parallelise hard
    ap.add_argument("--t_max", type=int, default=20)
    args = ap.parse_args()
    os.makedirs(LOGDIR, exist_ok=True); os.makedirs(os.path.dirname(CSV), exist_ok=True)
    jobs = [(s, m) for s in expand(args.seeds) for m in ("ours", "ego")]
    print(f"[screen] {len(jobs)} renders (seeds {args.seeds}, P={args.P}, t_max={args.t_max})", flush=True)
    rows = []
    with ThreadPoolExecutor(max_workers=args.P) as ex:
        futs = {ex.submit(run_one, s, m, args.t_max): (s, m) for s, m in jobs}
        for i, fut in enumerate(as_completed(futs), 1):
            r = fut.result(); rows.append(r)
            print(f"[screen] {i}/{len(jobs)}  seed={r['seed']} {r['mode']:5} "
                  f"reach={r['reached']} coll={r['collided']} clr={r['min_clr']} astar={r['astar_fail']}", flush=True)

    fields = ["seed", "mode", "reached", "collided", "dnf", "hit_ped", "hit_veh", "hit_animal", "hit_static",
              "min_clr", "ped_clr", "veh_clr", "animal_clr", "static_clr", "astar_fail", "t_goal"]
    rows.sort(key=lambda r: (r["seed"], r["mode"]))
    with open(CSV, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader(); w.writerows(rows)
    print(f"\n[screen] wrote {len(rows)} rows -> {CSV}", flush=True)

    by = collections.defaultdict(dict)
    for r in rows:
        by[r["seed"]][r["mode"]] = r
    tf = lambda x: str(x).strip().lower() == "true"
    n = sum(1 for d in by.values() if "ours" in d and "ego" in d)
    print(f"[screen] {n} seeds (crash=stop). 安全到达 = reached & not collided & not DNF; DNF = no lap-done (stuck/timeout):")
    print(f"           {'':6} {'安全到达':>8} {'撞人':>5} {'撞车':>5} {'撞动物':>6} {'坠机':>6} {'DNF':>5}")
    for who in ("ours", "ego"):
        col = lambda k: sum(tf(by[s][who].get(k, "")) for s in by if who in by[s])
        dnf = sum(1 for s in by if who in by[s] and tf(by[s][who].get("dnf", "")))
        # safe-completion: reached AND not collided AND not DNF -> a stuck/timed-out run is never counted safe
        safe = sum(1 for s in by if who in by[s] and tf(by[s][who]["reached"])
                   and not tf(by[s][who]["collided"]) and not tf(by[s][who].get("dnf", "")))
        print(f"           {who:6} {safe:>5}/{n:<3} {col('hit_ped'):>5} {col('hit_veh'):>5} "
              f"{col('hit_animal'):>6} {col('hit_static'):>6} {dnf:>5}")
    print("[screen] done", flush=True)


if __name__ == "__main__":
    main()
