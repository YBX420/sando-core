#!/bin/bash
# 真机部署前渲染战役:全场景 × 全传感器栈(D435i+realistic 前端),纯统计无视频
cd /media/boxuan/Data2/projects/sando_py/sando-core/metaurban
L=out/render_campaign.log
R=/media/boxuan/Data2/projects/metaurban
echo "=== CAMPAIGN START $(date) ===" >> $L
for f in scenarios/full/*.json; do
  n=$(basename $f .json)
  for arm in maneuver ego; do
    seeds="11 22 33"; [ "$arm" = "ego" ] && seeds="11"
    for ps in $seeds; do
      tag="$n|$arm|s$ps"
      grep -qF "DONE $tag" $L && continue          # resume
      (cd $R && timeout 900 env DISPLAY=:1 PYTHONPATH=$R \
        LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6 PERCEPT_SEED=$((1234000+ps)) \
        ~/miniconda3/envs/metaurban/bin/python -u \
        /media/boxuan/Data2/projects/sando_py/sando-core/metaurban/render_3d_video.py \
        --scenario /media/boxuan/Data2/projects/sando_py/sando-core/metaurban/$f --no_crowd \
        --$arm --d435i --fov_range 10 --t_max 30 2>&1 \
        | grep -E "lap done|theorem" | sed "s#^#[$tag] #" >> /media/boxuan/Data2/projects/sando_py/sando-core/metaurban/$L)
      echo "DONE $tag $(date +%H:%M)" >> $L
    done
  done
done
python3 - <<'PY' >> $L 2>&1
import re
rows = {}
for l in open("out/render_campaign.log"):
    m = re.match(r"\[(.+?)\|(\w+)\|s(\d+)\] .*lap done\. reached=(\w+) t_goal=([\d.inf]+)s collided=(\w+) min_clr=([-\d.]+)m", l)
    if m:
        rows.setdefault((m[1], m[2]), []).append((m[4]=="True", m[6]=="True", float(m[7])))
import numpy as np
out = ["# 渲染战役统计(传感器诚实,D435i+realistic)", "", "| scene | arm | reach | collide | clr med/worst |", "|---|---|---|---|---|"]
tot = {}
for (sc, arm), v in sorted(rows.items()):
    r = sum(x[0] for x in v); c = sum(x[1] for x in v); cl = [x[2] for x in v]
    tot.setdefault(arm, [0,0,0]); tot[arm][0]+=r; tot[arm][1]+=c; tot[arm][2]+=len(v)
    out.append(f"| {sc} | {arm} | {r}/{len(v)} | {c} | {np.median(cl):.2f}/{min(cl):.2f} |")
out.append("")
for arm, (r, c, n) in tot.items():
    out.append(f"**{arm} 总计: reach {r}/{n}, collide {c}/{n}**")
open("out/scenario_runs/RENDER_CAMPAIGN.md", "w").write("\n".join(out) + "\n")
print("[campaign] wrote RENDER_CAMPAIGN.md")
PY
echo "=== CAMPAIGN DONE $(date) ===" >> $L
