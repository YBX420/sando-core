---
name: sando-core-status-2026-06
description: "2026-06-22 全量代码审计后的 sando-core 统一真相:精确连续时间 Bernstein 几何证书 + 双 planner(MINCO/EGO)适配 + EgoSafe 二元 HOLD 闭环 + 10-seed A/B;conformal 统计半边没建(q=0);重心转工程。这是最新权威,覆盖更早的 framing。"
metadata:
  type: project
---

**2026-06-22 全量审计统一稿(8 子系统读真代码后)。** 权威文档 = `docs/safety-layer-spec.md`(v2)+ `docs/safety-layer-plan.md`(v2)+ `CLAUDE.md`,均已按本条重写。**重心从论文(RA-L 9/15)转为工程实现**(用户决定);论文降下游、未终结。

**仓库真相**:唯一真相 = 分支 `feat/bernstein-gate`(HEAD `46f5ac9`)。`master`(6d380fd)只有 MetaUrban 4 类避障+可视化无证书/EGO;`bcert-wire`(e8350cc)是纯祖先,对应的 `sando-core-bcert/` 旧 worktree **忽略**。

**做出来的(已实现+自检)**:
- **精确连续时间碰撞证书** `cpp/include/sando_cpp/bernstein_cert.hpp`:`w=p−c`、`S=‖w‖²` 写 Bernstein,`max_k(R²−S_k)≤0 ⟹ ‖p−c‖≥R` 全连续时间(零采样)。sound 靠**外向取整区间**(`std::nextafter`,非 fesetround);自适应 de Casteljau 细分(maxdepth16);动障证到 `t_hi`。球心最多二次、只证球、是判官。
- **planner 无关已证**:任意次通用核 `certify_segments_vs_sphere` vs deg-5 专用路径交叉验证**判定(certified)一致**(ctest;margin 打印未断言)。MINCO 五次→亏量 10 次;EGO cubic→6 次,`ego_capi.cpp` 的 `ego_certify` 用 `Mb/6` 转 Bezier 喂同一核。见 [[sando-py-ego-port]] [[sando-py-bernstein-deficit-cert]]。
- **EgoSafe**(`metaurban/ego_safe.py`):二元 certify-or-HOLD 闭环(EGO 规划→证书判→不过 HOLD,不改轨迹)。是**当前正典**;`render --ego_safe` 的分级减速刹车是抛光层(EGO 之后再统一)。见 [[sando-py-layer-judge-not-corrector]]。
- **10-seed A/B**(`out/ab_runs/`,真四旋翼动力学):raw EGO 撞 3/10,EGO+层撞 2/10;招牌 seed8 −0.305(撞)→+0.616(不撞);seed31 静态穿透 −0.793 层没动作(静态不在 mover 证书范围)。
- **ctest = 25**(非旧文档的 19/20/24),上次 Linux 跑 **25/25**(`LastTest.log` 2026-06-20)。
- 证书门 `minco_deficit_cert` 接进 MINCO 核但**默认 OFF**(附加门,没替旧 per-CP 半空间 ALM;默认接收门仍是采样门 K≥200)。

**没建(future work)**:① **conformal 统计半边整条没建,`q_conformal=0` 占位 → P(碰)≤ε 没实现,现在只确定性几何 margin**(用户决定 headline 就停在「精确几何证书」);② 最小修正 QP(只判不修);③ S7-CRET 零代码;④ EGO 没接 `eval_batch.py`;⑤ 三感知分支(统计 tube/遮挡阴影/FN body 门)无,mover 全来自 GT;⑥ committed-traj capi getter 没暴露;⑦ `ego_capi.so` 手编 untracked 可能过期、`cpp/build_v2/` 误提交。

**关键诚实**:对外只能说「确定性几何证书 + planner 无关」,不能说「概率/语义风险保证」——直到补 conformal。
