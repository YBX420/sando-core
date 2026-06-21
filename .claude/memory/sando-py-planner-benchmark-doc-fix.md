---
name: sando-py-planner-benchmark-doc-fix
description: docs/planner-vs-benchmark-split.md北极星错(追平EGO=0novelty+违铁律+治不了根因);A2软化走廊必须INVERT;应重写成"评测台=认证安全层demonstration harness"
metadata:
  type: project
---

2026-06-19:`docs/planner-vs-benchmark-split.md` 有大问题,北极星指错方向,需重写。

**三重错**:① 整篇框成"追平 EGO-Planner"(A1 机体/A2 自由变形/A3 yaw/A4 接缝)= **0 novelty**(EGO 是 2020 baseline,且 Wave3/6 确认"拓扑搜索+并行优化≈EGO-v2");② A1-A4 全是 side-paper planner 内部 = **违 9/15 铁律**;③ **治不了根因**。

**最危险 = A2(把硬 SFC/STC 走廊软化成软引导)→ 必须 INVERT 成铁律(不准软化)**:它会删掉硬认证主线和唯一存活的 S3([[sando-py-bernstein-deficit-cert]])。

**50% 擦碰根因诊断错**:它自己的 B2 已证根因是**台子把静障当软 occupancy 喂**(`feed()` update_occupancy),非算法缺陷;正解=静障**硬认证**(主线对齐),且是跨边界联合决策,非单方 A2 软化。

**最要命(新发现)**:occupancy 只在巡航单层 z(B1/B3)→ 垂直爬升 recovery 逃进未建模树冠、FN 遮挡/body-clearance 分支永不触发 → **评测台结构性跳不到主线贡献**。

**重写**:北极星翻成"**评测台唯一目的 = non-reactive 域 demonstrate 认证安全层**";headline 图 = 连续时间 violation-rate vs 采样分辨率(S3 平在 0、离散 CP 上升)+ 三档密度经验覆盖≥1-ε。集成侧必做:多层 z occupancy(getTightBounds)+ `--static_hard`。保留 §0 接口契约 + B4 基建。文档头写明 9/15 铁律防下一个读者再提 A1-A4。

注:A3/A4 平滑本身是真需求(见 [[sando-py-smoothness-real]]),doc 的错是"追平 EGO"的框架 + A2 软化,不是"平滑该不该做"。
