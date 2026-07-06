# 相关工作对位表(#8,2026-07-04)——两个 novelty 的边界防御

## Novelty A:端到端组合定理(cadence-aware episode 保证)
| 对手 | 他们有什么 | 我们的差异化 |
|---|---|---|
| Lindemann/Pappas 2511.10586 | conformal 轨迹预测→MPC 约束,自称 first valid interactive | 他们=单点预测集喂规划器;**无端到端 episode 级碰撞概率、无重认证节奏论证、动态反应域**(我们限 non-reactive 明写) |
| 2603.08958(Mondrian robotics) | 按状态分层 conformal 覆盖 | 占了 per-state 分层的 genus(audit 判 age-conditional 单独成文会死)——我们不做分层 headline,分层只是定理内的可选细化;**贡献在组合层**(sup 单元+节奏+三支记账) |
| COPPOL 2510.18485 | 感知 FN 率的 conformal bound | 感知层单点保证;不接规划器、不接证书、无 episode 语义 |
| Kalluraya 2209.06323 | 动态环境 STL 规划+预测不确定性 | 假设已知不确定集;我们的集由部署在环残差标定(条件律匹配部署) |
| 2602.12616 | 泛密度偏移 conformal | 占 distribution-shift genus;我们明写分布内(冻结协议),不碰 shift |
**防御要点**:①保证对象是「认证飞行的整个 episode」不是单点预测;②节奏论证(W=DT+δ)是把 3 倍管子差距变出来的机制,文献无先例;③三支记账(认证/陈旧/evade)把"证了一半"的常见假保证显式拆穿;④残差从部署管线自身收割(含关联/间歇/coast),不是干净仿真残差。

## Novelty B:certified-planner benchmark(MetaDrone)
| 对手 | 差异 |
|---|---|
| 通用无人机 sim(AirSim/Flightmare) | 有物理无"认证安全层评测协议":无间距地板/城建规则/遭遇几何过滤/闭环难度调谐/多臂配对统计 |
| planner benchmark(BARN 等) | 静态或弱动态;无 per-condition 失效包络、无 RTA 失效率指标、无感知诚实分层(gt/realistic/dyn 六臂) |
| EGO/SANDO 原生评测 | 单臂 demo 式;我们抓出上游 UB(ISSUE_UPSTREAM.md)+ 三基线可比 |
**防御要点**:六臂分解把"安全来自哪层"变成可测(感知≈动力学 4 倍);失效包络+攻击阶梯(时序调谐攻不动→物理三重叠加破防)是 methodology 贡献。

## 已知不占优的(诚实写限制)
- 交互/反应式行人(ORCA)未做——保证限 non-reactive scripted movers;
- 分布内保证(冻结生成器+传感器律);真实硬件域差距未触碰;
- evade 支无保证是结构性的(能证的不 evade)。
