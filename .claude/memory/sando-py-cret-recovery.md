---
name: sando-py-cret-recovery
description: S7-CRET=认证平滑recovery,核心洞见"别造新轨迹只改时钟";同时是平滑需求里recovery的解
metadata:
  type: project
---

**S7-CRET(Certified Retiming Escape)= 候补新原子 + 平滑 recovery 的解(2026-06-19)。** 单边完备性待自证,novelty 薄(path-velocity/ST-graph 速度规划是老货),但定位独特。

**核心洞见**:一条轨迹 = 路径 + 时钟(你在哪 = 路径(时钟(t)))。
- **老 recovery**(`planner.hpp:1313` yield 瞬时置零 / `:1346` climb 突兀垂直)= 卡死时**造一条新轨迹** → C1/C2 断裂 + **飞出认证走廊**。
- **CRET** = 不造新路径,对**已认证承诺轨迹**做 C2 单调时钟 warp(放慢):`q(t)=mj.eval(s(t))`,s(0)=0,s'(0)=1,s''(0)=0,s' 用 smoothstep 降到 inv_s∈(0,1]。链式法则给 q'(0)=v_A、q''(0)=a_A(**接缝严格 C2**),几何含 z 不变(**留认证走廊**)。

**为什么仍认证**:s'≤1 只放慢 → 对被退让 mover 的 clearance 单调不降 → 可证仍满足 deficit≥0;闭式 go/no-go 在 warp 后重验 deficit(套 plan_minco re-verify 模板)。连"沿走廊原地停"都违才退 yield/climb。

deficit 解析结构给触发:首违时刻 t* = deficit 首根。落点 `planner.hpp:~1249`,复用 last_minco_traj(已 stash)。属主线 RTA 最小修正。详见 [[sando-py-smoothness-real]]、[[sando-py-bernstein-deficit-cert]]。
