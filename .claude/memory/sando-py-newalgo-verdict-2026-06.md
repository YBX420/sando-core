---
name: sando-py-newalgo-verdict-2026-06
description: 2026-06-19 八波多智能体"给MINCO/全局planner找新算法"的最终诚实裁决:唯一真·新=主线证书S3,新规划器那条线被先验占满
metadata:
  type: project
---

2026-06-19:跑了 8+ 波多智能体攻坚(发现×5→汇总→撞车检测→形式化→证明→对抗破证→双轨硬化/再猎→证破新定理→平滑性),每条结论经实搜撞车+对抗破证。

**最终 genuinely-new + sound 账:**
- **1 个铁的 = S3**:连续时间 distribution-free conformal-Bernstein deficit 证书(见 [[sando-py-bernstein-deficit-cert]])。它**属于主线认证安全层、不是新规划器**,且修了现役 mover 门的真 soundness bug(见 [[sando-py-mover-gate-bug]])。
- **1 个候补(待证)= S7-CRET**:认证平滑 retiming recovery(见 [[sando-py-cret-recovery]]),兼服务"平滑是真需求"(见 [[sando-py-smoothness-real]])。
- **全死**(novelty 坍塌成先验或假):STRIDE 防火墙=RTA;S2 价格-winding=mp-NLP+T-MPC++;O(M)二阶=Pearlmutter/IPOPT;Risk-Wealth Monitor 的"任务级 εN→ε 避撞界"=信息论反例证伪(1-(1-α)^N→1,Ville 界的是财富越界非碰撞)。详见 [[sando-py-prior-art-map]]。

**结论**:"给规划器发明新 unstuck 算法"的搜索收敛成"给安全层找了个新证书"。新规划器逃逸算法确被先验占满(连续 3 波 0 产出)。最高 ROI 单步=写 ~250 行 deficit 原语 + 换双重 unsound mover 门(一步同时:S3 纸面→demonstrated、修真 bug、当接缝/CRET 平滑根基)。

完整报告 + 工作产物:`.claude/wave_work/FINAL_REPORT.md` + 同目录 w1..w8 + smooth JSON。
