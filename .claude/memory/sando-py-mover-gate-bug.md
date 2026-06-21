---
name: sando-py-mover-gate-bug
description: 现役 mover 碰撞门是双重 unsound 的真 bug(per-CP halfspace 对移动障碍塌 + K≥200 采样漏穿 + 只hold 0.75s),应换成精确 deficit
metadata:
  type: project
---

2026-06-19 读码确认:shipped 的移动障碍碰撞门**双重不 sound**,是真 bug 不是设计选择。

1. **per-CP 冻结半空间对 mover 塌**(`local_opt_hardalm.hpp:236-262`):每个控制点用不同时刻的中心 `predict(t_pt(i,k))` 算法向 `g=R−aᵀ(P−c)`,凸包-over-time 论证需要单一共享方向,mover 旋转几何使它**不成立** → 闭式证书对移动障碍无效。
2. **打补丁的密采样漏穿**(`plan_minco.hpp:482` hard_clearance_trusted,K≥200):只查有限采样点,**样本缝里漏碰**(0.55m 无人机 5m/s 两点间走 0.1m+ 够擦碰),非连续时间 sound。
3. **只 hold 到 τ_trust=0.75s**(代码注释自承),后面靠 receding-horizon 兜。

**注意对比**:S3 精确 deficit(见 [[sando-py-bernstein-deficit-cert]])对 mover **不塌**——它只读 x̂_i 与 r_i、从不用共享冻结法向线性化范数,mover 当多项式 c(t) 携带。修法=精确 deficit 当 GATE,保留 per-CP halfspace 当优化器梯度 surrogate。这是 9/15 前可做的安全层 soundness 修复(default-OFF 保 golden)。
