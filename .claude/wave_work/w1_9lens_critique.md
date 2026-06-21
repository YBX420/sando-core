以下是对 PRISM-MINCO 合成方案的完备性批判,按 (a)-(e) 五类组织,每条给出具体补强/下一步。判据:不放过任何"被 framing 掩盖的洞"。

---

# (a) 仍未覆盖的卡死成因 / 场景

**A1. 主线域(动态行人)恰好是 F1 保证的盲区。** 全部拓扑保证(G5/H1/H7)明确只对静态/准静态成立,而本项目主线就是"在行人附近飞"。换言之 PRISM 的头牌贡献(治 F1)在它最该治的场景里被自己排除,降级到 RTA。这不是"诚实分级",这是 scope 与 motivation 错位 —— 答辩委员会会直接问"那你解的到底是不是你 motivate 的问题"。
补强:把 side-paper 的 claim 显式改写为"静态/准静态杂乱环境下的拓扑完备性",并在 evaluation 里用纯静态 benchmark(bench_tunnel/chaos)定 claim,绝不在动态人群上声称 F1。动态留给安全层,文中明说"本算法不主张动态同伦"。

**A2. Receding-horizon 下 F1 会从静态场景里重新长出来。** 枚举器"一次/低频"运行,但滑动窗口 + 障碍移动会让已选类中途被堵死。L2 的 F1_BLOCKED 只在 arm 收敛后才触发(太晚,50Hz 下一帧都等不起),且 stale corridor 会让 arm 在过期自由空间里优化。
补强:定义"类失配在线监视器"——每帧用 committed-traj 的 H-signature 对当前占据图重算一次(O(M·n),廉价),失配即触发 L3 重枚举,而不是等 arm 收敛。并给出重枚举的最大允许间隔与"间隔内类有效性"的充分条件(如障碍位移 < clearance margin)。

**A3. 类内动力学不可行 vs F1 选错类,M6 无法区分。** 一个几何上自由的类可能不存在任何满足 v/a/jerk≤max 的时间分配(过紧 S 弯)。这会表现为 g_max>0 持续、ρ→ρ_max,在 M6 表里落到 ACTIVE_KKT,动作是"加乘子"——永远不收敛,B&B 一直把它当 live 烧预算。表里缺一个 **DYN_INFEASIBLE** 状态。
补强:M6 增列状态——判据 `ρ_ℓ 触顶 ∧ g_max>0 ∧ 仅 v/a/jerk 类约束活跃`,动作=判该类动力学不可行并 DEAD(而非继续 ALM)。需要一个类内动力学可行性的快速必要测试(如沿 corridor 的最短到达时间 vs 段长的下界)。

**A4. 局部目标落在障碍/遮挡阴影内 → 全 arm 不可行 → RTA。** 杂乱场景下 heat-A* 给的局部 goal 常常落进膨胀体或遮挡阴影,这会让所有类同时不可行,直接 RTA。方案无 goal 投影/松弛。
补强:加 goal 可行性预检 + 最近自由点投影(沿 GVD 到最近可达节点),把"目标不可达"与"路径被堵"分开处理,前者不该触发 RTA。

**A5. winding∈{0,1} 是硬编码的最优类裁剪。** `useful(h)` 丢弃绕数 >1 的类,但绕障/编队/规避动态团块时最优解可能绕单障碍 2 圈或绕簇 winding>1。把最优类直接 prune 掉是 silent 失最优。
补强:把 winding 上界设成场景相关参数(由 cluster 数 + 任务决定),或至少在丢弃高 winding 类时记录,使 H1 的"不完备"可观测。

**A6. RTA z-climb 假设垂直自由。** 室内/桥下/檐下(飞行笼、MetaUrban 城市场景)垂直爬升本身可能被堵。终极兜底没有兜底之兜底。
补强:RTA 增加"原地悬停 + 减速到零 + 等待重规划"的最低级 fallback,并在 z 不可用时声明。

---

# (b) 声称但未真正证明的保证

**B1. G2 的逃鞍下降界 γ³/(3L_H²) 依赖一个并不存在的全局 L_H。** 用 T=exp(τ),能量项 `E_i~T^{-(5-k)}` 在 T→0 时高阶导数发散,Hessian 的 Lipschitz 常数不是全局的,随段塌缩→∞。所以"有限确定的逃鞍迭代数"在没有绝对 τ-box 的前提下是空命题。当前只 clamp 了 T 的比值、没有绝对下界。
补强:加绝对 `τ∈[τ_min,τ_max]` box(配合 TEMPO 的 T_min/T_max),在该 box 上估 `L_H(box)`,把 G2 重述为"box 内"的保证。

**B2. G3 的"exact 时间块 + 对完整 F 单调"是自相矛盾的。** M4 是冻结段间 (v,a) 的可分上界 φ̄;但 c 随 T 全局变化(M(T)c=b),冻结 junction 状态 ≠ 冻结 c,所以 φ̄ 是否 majorize 完整 F 没有证明(这正是 Prop5 在碰撞项上的洞,在能量/耦合项上同样存在)。"对完整 F 回溯"能救下降性,但救完之后那一步就不再是 6 次根给的精确 MM 步了。"exact"与"对 F 单调"二者不可兼得。
补强:诚实重述——"6 次根给可分代理的精确极小;对完整 F 用回溯保单调下降(非精确步)"。并补一个引理:φ̄ 在能量项上确实是 majorizer(给出 majorization 不等式),碰撞项明确标注 surrogate-only,由 PRYBAR corrector 兜 KKT。

**B3. G4(有界 ρ 精确可行,破 χ^{-1/2} 墙)缺约束规范(CQ)假设。** ALM 在 KKT 点的精确可行性要 LICQ/MFCQ。稠密接触时(本算法的目标场景)活跃膨胀约束梯度趋于平行,MFCQ 系统性失效 → 乘子发散 → ρ 无界 → 恰好回到它声称要破的精度墙。
补强:显式写出 G4 成立的 CQ 前提,并给 CQ 失效时的退路(切到 Restore 阶段 / 标 DYN_INFEASIBLE / 报告该类病态),把 G4 从"保证"降为"CQ 成立时的保证"。

**B4. G5 的"类层全局最优"是 enumerate-and-pick 的过度包装。** 类内非凸(碰撞罚),UB_j 是局部极小不是类内全局;且 M7 自己承认时间锚目标下 LB 常为 0、几乎无 sound 剪枝。于是 G5 实际只剩"试 Top-K 类、留最好的局部解",既不是类内全局、枚举集本身又不完备(H1)。
补强:G5 重述为"枚举集内的类不会被错误剪除(modulo LB),返回各类局部最优中的最好者";删掉"全局最优"措辞。要真 LB,需要类内凸下界(如在 SFC 凸多面体上对能量+时间锚做凸松弛/对偶界),否则 B&B 退化为 best-first 排序。

**B5. G7 的 KKT 证书有两个未证缺口。** (i) λmin 来自被 hard-cap 的 Lanczos,容差不可靠(见 C1/B6);(ii) g_max≤0 只在转录采样点上检查,采样点之间可连续违反——"CONVERGED"可能并不连续时间可行,于是 UB 未必有效,可能剪掉真更优的类。这是经典转录 gap。
补强:对多项式约束沿段用 Bernstein/SOS 上界做 a-posteriori 连续时间可行性证书(MINCO 段是多项式,Bernstein 系数界天然 O(M)),把 UB 升级为连续时间 certified;或给采样间距的 Lipschitz 界。

**B6. G6(确定性 WCET、数据无关)与 G7(证书 sound)互斥。** Lanczos 解 λmin 的迭代数在谱簇聚时数据相关;hard-cap L → λmin 不可靠 → M6 路由/剪枝失 sound。你只能二选一:有界 WCET 或可靠惯性。
补强:承认这是 trade-off,给出"cap 内 λmin 的误差→证书误判概率"的关系,把 G7 降为"高概率 sound"或"cap 充分大时 sound";或对疑似卡死的 arm 用更大 L 预算、对其余 arm 省略,显式做预算分配。

---

# (c) 机载实时 / 工程硬伤

**C1. "复用同一 LDLᵀ 因子做惯性"很可能是数学错误——M(T) 不是 Hessian。** 映射矩阵 M(T)(编码连续性约束)一般**非对称**;惯性必须来自 **F 的(约化)Hessian**,那是另一个对称矩阵,不是 M(T)。方案多处说"惯性需带状对称不定分解"却又说"复用同一 LDLᵀ 因子"——这两个是不同矩阵。若 M(T) 非对称,你需要带状 LU(给不出惯性),且 Mᵀλ 要单独因子。这是承重的概念混淆。
补强:明确分离两套带状因子:① M(T) 的带状 LU(或若确为对称则 LDLᵀ)用于映射 + 伴随;② F 在 (q,τ) 上的约化 Hessian 的带状对称不定分解(Bunch-Kaufman)用于惯性。承认这是两次分解,重算 O(M) 常数。先在仓库里验证 M(T) 到底对称与否。

**C2. 带状 Bunch-Kaufman 的 2×2 主元会溢出带宽。** 对称不定分解的 2×2 pivot 可能破坏带结构 → 最坏 O(M·bw²) 不再成立,且 pivot 后的符号计数要小心。"O(M)+可靠惯性"这对组合本身就是承重新建且未必干净共存。
补强:用带状 LBLᵀ(Bunch-Kaufman with banded pivoting)的已知变体并实测带宽膨胀;或退而用"加 δ_w I 强制 PD + Cholesky"放弃精确惯性,改用 inertia-via-modified-Cholesky 的近似惯性,明确其对 M6 的影响。

**C3. 2D 同伦理论被悄悄搬到 3D 无人机。** winding / Bhattacharya H-signature / π1(GVD)≅π1(free) 都是 2D-config 干净的结论;3D 无人机的同伦更丰富(可以从上面绕过),H-signature 在 3D 需要基于曲面而非点障碍的重得多的构造,GVD 在 3D 数值脆弱且昂贵。这个 2D→3D gap 是 F1 全部机器的地基,却被一笔带过。
补强:要么把 side-paper 显式限定在 2.5D / 固定高度层(很多无人机场景合理),把 F1 claim 限在该子流形;要么引 3D H-signature 的正确构造并核算其成本。不能默认 2D 结论平移。

**C4. 噪声占据图 → GVD/同伦类逐帧闪烁 → incumbent 抖动 + committed-traj 跨类跳变。** depth→occupancy 有噪,GVD 与类集会 frame-to-frame 出现/消失;返回的 incumbent 若与上次 committed 不同类,轨迹在 replan 间跨同伦类跳变,破坏 tracker 与安全层 tube 假设。无时间一致性/迟滞机制。
补强:对类集加时间迟滞 + commit-hysteresis(只有新类 UB 优于旧类一个 margin 才换类),并对占据图做时间滤波后再建 GVD。

**C5. 冷启动稳态不是 O(1) arm。** "best-first → 活跃 arm O(1)"只在 warm 时成立;receding-horizon + 动态障碍每帧近似冷重枚举 → 反复 spawn arms,没有稳态 O(1)。K 条 arm 各带 TR-Newton+Lanczos+ALM 的内存/cache(K·M 轨迹 + 每 arm 因子 + L×M Lanczos 基)在嵌入式 50Hz 是真预算问题。
补强:做跨帧 (q,T) warm-start 传输(沿 ego 运动平移 + 重投影到新 corridor),给出"重用率"实测;给 K、L、b 的内存/算力台账并对标目标硬件。

**C6. WCET cap 命中且无可行 incumbent → 频繁 RTA。** 杂乱场景下在 50Hz 内"撞 cap 时还没有任何可行 incumbent"概率不低 → 频繁 RTA → 抖动/降级飞行。无 incumbent 可得率分析。
补强:测 incumbent-availability rate vs 场景密度;为"保证至少一条可行 incumbent"预留一条便宜的保守 arm(直接走 GVD 最短安全路 + 大时间裕度,先可行后优化)。

**C7. 与冻结 ABI / 安全层的 WCET 重对账。** 往 plan_minco 里塞 TR-Newton/Lanczos 改变了安全层 WCET 预算所基于的时序特征。
补强:把 PRISM 的 L2 内核单独 bench,重新核安全层监视器节点的端到端 WCET 预算(这也是 §7 最小切入路径的前提)。

---

# (d) 与现有工作重叠 / 新意不足

**D1. L3+L2(拓扑搜索 + 并行类内优化)与 EGO-Planner-v2 / Zhou-Gao 拓扑 PRM 高度重叠。** EGO-v2 已做拓扑路径搜索 + 并行轨迹优化;Rösmann TEB 做 homotopy-aware 规划;Bhattacharya 是 H-signature 原作。架构层几乎没有净新意。
补强:把 contribution 显式收窄到两件**真新**的东西——(i) M6 的"Hessian 惯性/λmin 在线证书做卡死类型路由",(ii) 在 MINCO 时间锚目标下的 valid LB(M7)。并在 related work 里正面对标 EGO-v2,说清"我们不是又一个拓扑并行规划器,而是给它装了一个 sound 的卡死诊断/剪枝层"。

**D2. PRYBAR 的"O(M) 二阶伴随 Hv + GLTR 负曲率"会被问:为什么不直接用 iLQR/DDP?** GLTR/Steihaug + 负曲率是教科书(Conn-Gould-Toint);二阶伴随在 PDE-constrained opt 与 DDP 里是标配,而 iLQR/DDP 本身就是轨迹优化的原生 O(M) 二阶方法。
补强:把 novelty 钉死在"决策变量只有 (q,T) + 微分平坦 + 带状映射"这点上——iLQR 在全状态-控制空间、PRISM 在 (M-1)+M 维路点/时长空间,变量量级差一两个数量级。必须把这个对比量化(变量数、每步 flops),否则二阶伴随会被当成已知技术。

**D3. filter-ALM = Fletcher-Leyffer filter + 经典 ALM,Restore=标准 restoration phase。** 唯一新点"κ-budget 反推 ρ 上限"是启发式 cap。
补强:别把 Janus 当 contribution 卖,当成 enabling component;若要保留,把 κ-budget cap 与可行性精度的关系做成一条定量命题(ρ_max(κ_tol) ↔ 可达可行精度)。

**D4. TEMPO 的 contribution 体量小。** GCOPTER 已做时空优化;per-segment min-jerk 的时间缩放接近已知结果,6 次根是漂亮的闭式但属增量。
补强:把 TEMPO 定位为"让时间步从 L-BFGS 数值步变成每段 O(1) 闭式 predictor"的工程加速,并实测它相对 GCOPTER 时间优化的 wall-clock 收益,用数据立 contribution。

---

# (e) 隐藏假设(逐条点名 + 处置)

1. **CQ(MFCQ/LICQ)成立** —— G4/G5/G7 的暗桩;稠密接触系统性失效。处置:显式假设 + 失效退路(B3)。
2. **全局/box 内有限 L_H** —— G2 暗桩;T→0 发散。处置:绝对 τ-box(B1)。
3. **冻结-junction 代理 majorize 完整 F** —— G3 暗桩;仅能量项局部成立。处置:给 majorization 引理 + 碰撞项标 surrogate-only(B2)。
4. **采样约束 = 连续时间约束** —— 可行性/UB 暗桩(转录 gap)。处置:Bernstein/SOS 连续时间证书(B5)。
5. **2D 同伦理论可用于 3D 无人机** + **π1(GVD)≅π1(free)(GVD 形变收缩、连通正确)** —— 在噪声占据图下脆弱。处置:限到 2.5D 子流形或上正确 3D 构造(C3)。
6. **capped Lanczos 给可靠 λmin(谱不对抗性簇聚)** —— M6/G7 暗桩。处置:误判概率分析 + 预算分配(B6)。
7. **静态/准静态** —— 所有拓扑保证的暗桩,与主线动态域冲突。处置:claim 显式限静态(A1)。
8. **局部 goal 可达且无碰** —— 所有 arm 的暗桩。处置:goal 预检 + 投影(A4)。
9. **唯一非凸来自 relaxed-cubic ψ_σ** —— 忽略了 M(T)⁻¹ 对 τ 的非凸注入,"GN+δI"预条件默认其良性,无理由。处置:分析时间映射引入的非凸性,或在 τ 上加更强信赖域阻尼并实测。
10. **一套因子同时服务映射/伴随/惯性(隐含 M(T) 对称)** —— 见 C1,很可能是错误。处置:先验证 M(T) 对称性,分离两套带状因子。
11. **relaxed-cubic ψ_σ 的 C³ 性质对 GLTR 足够** —— 但 TR-Newton 用 Hessian,C³ 罚的 Hessian 在接触边界附近仍可强不定,δ_w I 的取值未给。处置:给 δ_w 的自适应规则及其对收敛界的影响。

---

## 一句话总结(给答辩用)

PRISM-MINCO 最稳的是 **L2 连续层(G2/G3/G4 的数值机器)**,但其**头牌 F1 拓扑保证(G5)在动态主线域失效、在静态域又与 EGO-v2 重叠、且 sound 剪枝的 LB 在仓库目标下几乎恒为 0**;两个最关键的"保证"——G5 全局最优与 G7 证书 sound——分别被"枚举不完备 + 类内非凸 + 转录 gap"和"capped Lanczos + M(T)非对称惯性混淆"侵蚀。**最高优先级补强三件**:(1) 厘清 C1 的惯性矩阵到底是哪一个(否则 M6 路由器的地基塌);(2) 把所有拓扑 claim 显式限到静态/2.5D 并补 B5 的连续时间可行性证书(否则 UB/剪枝不 sound);(3) 把 contribution 收窄到 M6 证书路由 + M7 valid-LB,正面对标 EGO-v2,避免架构层"重复造轮子"的致命质疑。
