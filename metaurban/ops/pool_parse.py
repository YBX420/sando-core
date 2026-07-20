#!/usr/bin/env python3
"""m1b pool ledger parser. Usage: pool_parse.py <pool_dir> <arm> [arm...]
Prints per-arm aggregate + per-seed table. Handles t_goal=inf and missing hold: field."""
import re, sys, glob, os, math

def parse_log(path):
    lap = None
    with open(path, errors="replace") as f:
        for line in f:
            if "lap done" in line:
                lap = line.strip()
    if lap is None:
        return None
    d = {}
    m = re.search(r"reached=(\w+)", lap); d["reached"] = (m and m.group(1) == "True")
    m = re.search(r"t_goal=([0-9.]+|inf)s", lap); d["t"] = float(m.group(1)) if m else math.inf
    m = re.search(r"collided=(\w+)", lap); d["coll"] = (m and m.group(1) == "True")
    m = re.search(r"min_clr=([0-9.\-]+)m", lap); d["clr"] = float(m.group(1)) if m else float("nan")
    m = re.search(r"pedestrian:([0-9.\-]+)", lap); d["ped"] = float(m.group(1)) if m else float("nan")
    m = re.search(r"hold:(\d+)", lap); d["hold"] = int(m.group(1)) if m else 0
    m = re.search(r"spchurn=([0-9.]+)", lap); d["spchurn"] = float(m.group(1)) if m else float("nan")
    d["lap"] = lap
    return d

def main():
    pool, arms = sys.argv[1], sys.argv[2:]
    data = {}
    for arm in arms:
        for p in sorted(glob.glob(os.path.join(pool, f"{arm}_s*.log")),
                        key=lambda x: int(re.search(r"_s(\d+)\.log", x).group(1))):
            s = int(re.search(r"_s(\d+)\.log", p).group(1))
            d = parse_log(p)
            if d: data[(arm, s)] = d
    seeds = sorted({s for (_, s) in data})
    hdr = "seed | " + " | ".join(f"{a}: t/clr/ped/hold" for a in arms)
    print(hdr); print("-" * len(hdr))
    for s in seeds:
        cells = []
        for a in arms:
            d = data.get((a, s))
            if not d:
                cells.append("--"); continue
            flag = "COLL!" if d["coll"] else ("" if d["reached"] else "NOREACH!")
            cells.append(f"{d['t']:.1f}/{d['clr']:.2f}/{d['ped']:.2f}/h{d['hold']}{flag}")
        print(f"s{s:<3} | " + " | ".join(cells))
    print()
    for a in arms:
        rows = [d for (arm, _), d in data.items() if arm == a]
        if not rows: continue
        n = len(rows); rch = sum(r["reached"] for r in rows); col = sum(r["coll"] for r in rows)
        ts = [r["t"] for r in rows if r["reached"] and math.isfinite(r["t"])]
        peds = [r["ped"] for r in rows if not math.isnan(r["ped"])]
        holds = sum(r["hold"] for r in rows)
        mt = sum(ts) / len(ts) if ts else float("nan")
        mp = sum(peds) / len(peds) if peds else float("nan")
        print(f"[{a}] n={n} reached={rch}/{n} coll={col} mean_t={mt:.2f}s mean_ped={mp:.2f}m holds={holds}")
    # pairwise identical-lap check (first arm = reference)
    if len(arms) > 1:
        ref = arms[0]
        for a in arms[1:]:
            same = [s for s in seeds if (ref, s) in data and (a, s) in data
                    and data[(ref, s)]["lap"] == data[(a, s)]["lap"]]
            both = [s for s in seeds if (ref, s) in data and (a, s) in data]
            diff = [s for s in both if s not in same]
            print(f"[{a} vs {ref}] identical lap lines: {len(same)}/{len(both)}"
                  + (f"  differs: {['s%d' % s for s in diff]}" if diff else ""))

main()
