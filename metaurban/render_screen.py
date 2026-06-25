"""render_screen — screen MANY 3-D render scenarios for navigability, with a RELIABLE concurrency cap.

For each seed run ours (--maneuver) AND native (--ego) in render_3d_video (headless, no mp4), record
reach/collision/clearance/astar; then select the seeds where OURS succeeds (reach + 0 collision = navigable, not
static-blocked) and summarise the A/B on that fair set. ThreadPoolExecutor caps concurrency (bash wait/jobs/xargs
were unreliable / left orphans in the nohup'd context). seeds: env.reset(seed%20) = scene, seed*1000 = ROUTE, so
seeds>=20 are fresh scenarios.

Run: python metaurban/render_screen.py --seeds 0-59 -P 3 --t_max 20
"""
import os, re, sys, csv, glob, argparse, subprocess, collections
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
    return dict(seed=seed, mode=mode, reached=reached, collided=coll, min_clr=clr, astar_fail=af, t_goal=tg)


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

    fields = ["seed", "mode", "reached", "collided", "min_clr", "astar_fail", "t_goal"]
    rows.sort(key=lambda r: (r["seed"], r["mode"]))
    with open(CSV, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader(); w.writerows(rows)
    print(f"\n[screen] wrote {len(rows)} rows -> {CSV}", flush=True)

    by = collections.defaultdict(dict)
    for r in rows:
        by[r["seed"]][r["mode"]] = r
    tf = lambda x: str(x).strip().lower() == "true"
    navig = sorted(s for s, d in by.items() if "ours" in d and tf(d["ours"]["reached"]) and not tf(d["ours"]["collided"]))
    print(f"[screen] {len(by)} seeds; NAVIGABLE (ours reached + 0 collision): {len(navig)}")
    print(f"[screen] navigable seeds: {navig}")
    if navig:
        ec = sum(tf(by[s]["ego"]["collided"]) for s in navig if "ego" in by[s])
        er = sum(tf(by[s]["ego"]["reached"]) for s in navig if "ego" in by[s])
        contested = sorted(s for s in navig if "ego" in by[s] and tf(by[s]["ego"]["collided"]))
        print(f"[screen] ON THE NAVIGABLE SET ({len(navig)} seeds):")
        print(f"           ours        : collide=0/{len(navig)} (0% by construction), reach={len(navig)}/{len(navig)}")
        print(f"           native EGO  : collide={ec}/{len(navig)}, reach={er}/{len(navig)}")
        print(f"[screen] CONTESTED navigable (scenario IS navigable yet native EGO collides): {contested}")
    print("[screen] done", flush=True)


if __name__ == "__main__":
    main()
