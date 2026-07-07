"""curate_report -- benchmark curation draft for human review (user ruling 2026-07-08: worlds where
even the oracle fails were never hand-audited and measure nothing about the algorithm; flag them
for deletion/review rather than letting them pollute rates)."""
import glob
import json
import os

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BD = os.path.join(HERE, "out", "ppo_eval")


def rows(pat):
    out = {}
    for f in glob.glob(os.path.join(BD, "ego_bench_v2era", pat)):
        for l in open(f):
            r = json.loads(l)
            if "error" not in r:
                out[(r["scn"], r["seed"])] = r
    return out


L = []
L.append("# Benchmark 策展草案(双败世界 -> 人工审查)\n")
L.append("## A. MetaUrban 20 集协议(EGO 面)\n")
led = [json.loads(l) for l in open(os.path.join(BD, "ego_mu_ledger.jsonl"))]
by = {}
for r in led:
    by.setdefault(r["tag"], {})[r["ep"]] = r
v7 = by.get("V7_full", {})
orc = by.get("V7_20ep_oracle", {})
if not orc:
    orc = by.get("V7_oracle180", {})
L.append("| ep | ours | oracle | 判定 |")
L.append("|---|---|---|---|")
for ep in sorted(set(v7) | set(orc)):
    o, k = v7.get(ep, {}), orc.get(ep, {})
    def s(r):
        return ("撞" if r.get("collided") else "达" if r.get("reached") else "超时") if r else "-"
    verdict = ""
    if k.get("collided") and o.get("collided"):
        verdict = "**双败 -> 建议删除(oracle 同死=物理死局,未经人工审查)**"
    elif k.get("collided"):
        verdict = "oracle 死但 ours 活 -> 保留(超额战绩)"
    elif o.get("collided"):
        verdict = "仅 ours 死 -> 保留(真失效,要修)"
    L.append(f"| {ep} | {s(o)} | {s(k)} | {verdict} |")
L.append("\n## B. replay 面 69 场景池(V11 vs 锥内全知)\n")
v11 = rows("rows_waveV11_w*.jsonl")
co = rows("rows_cone_oracle_w*.jsonl")
L.append("| 场景 | seed | ours | 锥内全知 | 判定 |")
L.append("|---|---|---|---|---|")
flagged = 0
for k in sorted(v11):
    a, b = v11[k], co.get(k)
    if b is None:
        continue
    def st(r):
        return "撞" if r["collided"] else ("达" if r["reached"] else "超时")
    if (not a["reached"]) and (not b["reached"]):
        flagged += 1
        L.append(f"| {k[0]} | {k[1]} | {st(a)} | {st(b)} | **双败 -> 审查** |")
L.append(f"\n双败集数:{flagged}(仅列双败;其余 {len(v11)-flagged} 集至少一臂成功,保留)\n")
open(os.path.join(HERE, "out", "curation_report.md"), "w").write("\n".join(L))
print(f"wrote out/curation_report.md ({flagged} replay double-fails flagged)")
