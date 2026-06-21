#!/usr/bin/env python3
"""Batch A/B runner: for many seeds, render raw EGO vs EGO+per-class (both --clear_spawn), drop everything
into out/ab_runs/, build a per-seed side-by-side mp4 + poster, and a summary.csv. Resumable (skips done).
Run inside the `metaurban` env with LD_LIBRARY_PATH set (see metaurban-render-demo-recipe).
  python3 ab_runner.py 0 2 4 5 7 8 9 13 23 31
"""
import os, sys, re, subprocess
import numpy as np, cv2, imageio.v2 as imageio

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "ab_runs")
os.makedirs(OUT, exist_ok=True)
SEEDS = [int(s) for s in sys.argv[1:]] or [0, 2, 4, 5, 7, 8, 9, 13, 23, 31]

LAP = re.compile(r"reached=(\w+) collided=(\w+) min_clr=([\-\d.]+)m\s+(.*?)(?:\s+egosafe\[cert=(\d+) hold=(\d+)\])?$")
SPAWN = re.compile(r"spawn clearance ([\-+\d.]+)m")
PER = re.compile(r"(\w+):([\-+\d.]+)")


def run(seed, variant):
    mp4 = os.path.join(OUT, f"seed{seed}_{variant}.mp4")
    log = os.path.join(OUT, f"seed{seed}_{variant}.log")
    if not os.path.isfile(mp4):
        cmd = ["python3", os.path.join(HERE, "render_3d_video.py"), "--seed", str(seed),
               "--ego", "--clear_spawn", "--mp4"]
        if variant == "safe":
            cmd.append("--ego_safe")
        with open(log, "w") as f:
            subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=HERE)
        src = os.path.join(HERE, "out", "drone_3d.mp4")
        if os.path.isfile(src):
            os.replace(src, mp4)
    # parse the log
    txt = open(log).read() if os.path.isfile(log) else ""
    row = {"seed": seed, "variant": variant, "ok": os.path.isfile(mp4)}
    m = SPAWN.search(txt); row["spawn_clr"] = m.group(1) if m else ""
    m = LAP.search(txt)
    if m:
        row.update(reached=m.group(1), collided=m.group(2), min_clr=m.group(3),
                   holds=m.group(6) or "")
        for k, v in PER.findall(m.group(4)):
            row[k] = v
    return row


def build_ab(seed):
    raw = os.path.join(OUT, f"seed{seed}_raw.mp4"); safe = os.path.join(OUT, f"seed{seed}_safe.mp4")
    if not (os.path.isfile(raw) and os.path.isfile(safe)):
        return
    ro = imageio.get_reader(raw); rn = imageio.get_reader(safe)
    Na, Nb = ro.count_frames(), rn.count_frames()
    N = max(Na, Nb)                       # run until BOTH finish; the shorter side FREEZES on its last frame
    fa = [ro.get_data(i) for i in range(Na)]; fb = [rn.get_data(i) for i in range(Nb)]
    H = max(fa[0].shape[0], fb[0].shape[0]); gap = 8
    pad = lambda fr: fr if fr.shape[0] == H else np.vstack([fr, np.zeros((H - fr.shape[0], fr.shape[1], 3), fr.dtype)])

    def lab(fr, txt, col, done=False):
        fr = np.ascontiguousarray(pad(fr)); cv2.rectangle(fr, (0, 0), (340, 24), (0, 0, 0), -1)
        cv2.putText(fr, txt + ("  [reached]" if done else ""), (8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA); return fr

    wr = imageio.get_writer(os.path.join(OUT, f"ab_seed{seed}.mp4"), fps=20, macro_block_size=8)
    for i in range(N):
        a = lab(fa[min(i, Na - 1)], "A  EGO (no safety)", (190, 190, 190), done=i >= Na)
        b = lab(fb[min(i, Nb - 1)], "B  EGO + per-class", (40, 230, 40), done=i >= Nb)
        wr.append_data(np.hstack([a, np.full((H, gap, 3), 30, np.uint8), b]))
    wr.close()
    i = N // 2
    a = lab(fa[min(i, Na - 1)], "A  EGO (no safety)", (190, 190, 190), done=i >= Na)
    b = lab(fb[min(i, Nb - 1)], "B  EGO + per-class", (40, 230, 40), done=i >= Nb)
    cv2.imwrite(os.path.join(OUT, f"ab_seed{seed}_poster.png"),
                np.hstack([a, np.full((H, gap, 3), 30, np.uint8), b])[..., ::-1])
    print(f"[ab] seed {seed}: built ab_seed{seed}.mp4 + poster", flush=True)


cols = ["seed", "variant", "ok", "spawn_clr", "reached", "collided", "min_clr",
        "pedestrian", "animal", "vehicle", "static", "holds"]
rows = []
for s in SEEDS:
    for v in ("raw", "safe"):
        print(f"[ab] === seed {s} {v} ===", flush=True)
        rows.append(run(s, v))
    build_ab(s)

with open(os.path.join(OUT, "summary.csv"), "w") as f:
    f.write(",".join(cols) + "\n")
    for r in rows:
        f.write(",".join(str(r.get(c, "")) for c in cols) + "\n")
print(f"[ab] DONE -> {OUT}/summary.csv  ({len(SEEDS)} seeds)", flush=True)
