---
name: sando-core-planner-agnostic-plan
description: "2026-06-24 用户拍板的下一步计划:为证明安全层 planner-无关(headline 支撑臂),多接几个『好复现的外部 planner』当运动生成器(不只 EGO),每个都套同一套连续时间 Bernstein 证书+conformal keep-out+KF 预测门控。MINCO 跳过(自研、非外部 baseline)。顺序:先修计算速度瓶颈(竖直证书 76ms,见 latency 记忆),再动手接 planner。接入只需 planner 输出『已承诺轨迹的分段 Bézier 控制点』这一个接口。"
metadata:
  type: project
---

**2026-06-24 用户拍板(下一步,排在『先修速度』之后):** 既然证书 planner-无关已经在 EGO 上跑通,**多接几个『好复现的外部 planner』** 当运动生成器,每个都套同一套安全层(连续时间 Bernstein 证书 + conformal keep-out + KF 预测门控),用来**实证 planner-无关**这条支撑臂(不是 headline,但是通用性卖点)。

**约束/取舍:**
- **MINCO 跳过**(`sando_native`/`cpp` 里有自研 min-jerk 五次,但它是自研、不算外部 baseline,说服力弱)。
- **找『好复现』的**(复线 = 能 vendored / 容易复现的开源 planner)。候选方向(待定,选好复现+有代表性的):GCOPTER/MINCO-family、Fast-Planner、min-snap(Mellinger)、kinodynamic A*/MPCC 等——挑 1-2 个**有现成开源、轨迹是多项式/样条、容易去-ROS vendored** 的。
- **接入接口极小**:planner 只需输出**已承诺轨迹的分段 Bézier 控制点**(EGO 是 B-spline→Bézier;MINCO 是五次→Bézier,见 `minco_to_segments`)。证书/预测/门控代码不动。
- **顺序铁律:先修计算速度**(竖直证书 `certify_above` 76ms 瓶颈 + 整 tick 81% 预算,根因+修法见 [[sando-core-latency-agnostic-2026-06]]),**速度修好再动手接 planner**——用户明确"这个先不动手,先修速度"。

**为什么有价值:** 证明"同一套认证安全层能套任意现成 planner、把它变成认证安全+会预测+敢更快"——这是 [[sando-core-conformal-2026-06]] headline(认证最快+最安全绕行)的通用性背书。相关:[[sando-core-latency-agnostic-2026-06]] [[sando-core-win-ego-2026-06]]。
