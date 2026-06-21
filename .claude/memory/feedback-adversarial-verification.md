---
name: feedback-adversarial-verification
description: 塔菲大人要"发明新算法"时:用重型多波workflow+实搜撞车+对抗破证,诚实报"什么死了为什么";别把真工程需求当cosmetic;token不计直到找到
metadata:
  type: feedback
---

2026-06-19 大规模"发明新算法"任务中确认的工作风格:

- **要彻底 + 诚实,不要乐观**:塔菲大人 explicitly "可以一直开,直到找到,我完全不在乎 token";"两个都要/继续广撒";"至少 10 个 wave,整体优化创新并验证没撞车"。→ 用重型多智能体 workflow 并行(发现→汇总→撞车→形式化→证明→对抗破证→硬化),**每个新颖 claim 必过实搜 prior-art 撞车检测 + 对抗破证**,扛不住就诚实判死。

**Why**:他要的是"genuinely-new + sound"(经得起 RA-L 审稿/答辩),不是听起来炫。把激动人心但其实是先验/假的东西筛掉,本身就是高价值产物(省他在死路上的月)。

**How to apply**:
- 提出任何"新算法/新点"前,先实搜点名最接近的已有工作 + 讲清 delta;剥掉先验后明说"还剩什么"。
- 区分【致命(claim 必撤)】vs【可降为 honest scope/limitation】,不为保住结论粉饰。
- **别把真工程需求当 cosmetic**——我把"轨迹平滑"当面子工程被当场纠正(它其实在修认证前提,见 [[sando-py-smoothness-real]])。被纠正要干净收回 + 保留批判里对的那半。
- 守 9/15 铁律 + spec(标定 Isaac 非 SDD、FN 三分支永不砍)。

配合既有 [[feedback-rootcause-first]](先穷举 formulation bug)、[[feedback-plain-language]]、[[feedback-test-style]]。
