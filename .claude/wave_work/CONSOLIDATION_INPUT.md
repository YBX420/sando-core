# 跨工作流收敛输入(Wave2 汇总用) — MINCO/全局planner 卡死创新

> 5 路发现工作流的蒸馏。范围已扩到【整个全局 planner】(不绑 MINCO)。
> 目标:确认一个 genuinely new + sound 的算法。两条主线:整体优化创新 + 验证没撞车。

## 0. 最强信号:三路独立撞到同一处(O(M) 二阶 MINCO)
- 效率 MF-GGN(rank1)、新颖 GeoMINCO、9视角 PRYBAR —— 三个独立 agent 群体都推出:
  **MINCO 缩并 (q,T) Hessian 稠密(故全圈子只用一阶 L-BFGS),但你为求 c 已付的 banded 分解让二阶 Hv / lifted-KKT 变成 O(M)。**
- 文献独立佐证:产出 D(求解器级二阶/牛顿)是 prior-art 近乎空白 = "最大发明空白点"。
- 诚实:二阶机器组件(GGN / Pearlmutter Hvp / 带状预条件 / RTI 热启动 / C-GMRES 延拓)各自经典 → 单卖是"组合+结构洞见",不是新原语。撑不起论文级 novelty 单独成立。

## 1. 真·新颖残差(经各自对抗核查后存活的、最该写进论文的点)
- **【最强】M6 证书路由(9视角 PRISM-MINCO):reduced-Hessian 在线惯性 λ_min 当路由器**,O(M) 在线判别"卡在拓扑层 F1 还是数值层 F2/F3/F5/鞍点/动力学不可行",据此分派恢复机制。agent 评:"全场唯一既新又严的杠杆"。**关键开放问题:能否上升到全局-planner 层 → 在动态行人域(F1 失效区)也成立?**
- M7 valid B&B 下界(time-anchor 目标下)—— 真但弱(LB≈0 退化为 best-first 排序)。
- FLÈCHE(新颖 rank5,nov0.6/sound0.78):平坦消元局部子问题的全临界点数值代数几何枚举 —— 最"既新又稳"的单点,软肋是慢(rt0.42);可作 M6 判定 F1 后的认证逃逸算子。
- FI-MINCO(新颖 rank1,nov0.6/sound0.72):连续时间 HOCBF 前向不变性 Bernstein 证书 —— 可补 B5 转录 gap(连续时间可行性证书)。

## 2. 已被占满的路(撞车清单 — 新算法必须避开/超越/正面对标)
- 多同伦枚举 + 并行择优:TRUST-Planner(UTF-MINCO,已做在 MINCO 上)、Fast-Planner 拓扑 PRM、EGO-Swarm 并行线程。
- EGO-Planner 路径引导 + 锚点 v→−v 隐式拓扑逃逸。
- 安全飞行走廊硬约束:Bubble Planner(已在 MINCO)、Liu 2017 SFC。
- FASTER 双轨安全回退 / DYNUS MIQP 硬约束。
- GCS 全局混合整数凸(对加速度/jerk 高阶支持差)。
- iLQR/DDP(二阶但全状态-控制空间) / RTI / C-GMRES / FATROP(结构化带状 SQP) / 结构化拟牛顿(40 年历史)。
- **D1 重点:L3+L2 拓扑搜索+并行优化 ≈ EGO-Planner-v2。** 收窄到 M6+M7 才能避撞。

## 3. 致命 framing 攻击(必须在最终算法里解决)
- **A1:拓扑完备性保证只对静态/准静态成立;主线动态行人域恰是 F1 盲区。** → 要么显式限静态/2.5D,要么找一个在动态域也 sound 的新机制(全局-planner 层的机会)。
- B5 转录 gap:g_max≤0 仅采样点查,样本间可违反 → 需 Bernstein/SOS 段内连续时间证书(MINCO 段是多项式,O(M) 天然)。
- B1:逃鞍界依赖 box 内 L_H(T→0 发散)→ 必须绝对 T∈[T_min,T_max] box(仓库 L-BFGS-B 直接支持)。
- D2:二阶 vs iLQR/DDP → 钉死"仅 (q,T) 微分平坦 + 带状",量化变量数/每步 flops。

## 4. 仓库已核事实(9视角读源码所得,后续波次直接用,勿重复犯错)
- T 是**直接决策变量** + L-BFGS-**B** box(`minco_cost_grad.hpp:192` T(i)=x(nq+i)),**不是** T=exp(τ) → 二阶数学在直接-T 坐标推(更干净)。
- `M(T)` **非对称**(`minjerk_traj.hpp:258-266` solve_adjoint 用转置 LU)→ **惯性绝不能复用 M(T) 的 LU**,必须 reduced-Hessian 独立对称不定分解(Bunch-Kaufman/LDLᵀ)。这是 M6 的地基。
- 现 dense `PartialPivLU`,M≤12→72×72 是 μs 级;严格 O(M) 需带状 LDLᵀ,规模触发后再做。
- time-anchor 目标主项 = w_time·((T_tot−T_target)/T_target)²(w_time=10)→ 长度型 LB≈0。
- 仓库无 GVD/Yen/H-signature 枚举器(仅 ≤5 detour seeds + 单 H-sig passing-side)。
- 可复用资产:`anytime_feasible.hpp`(<50ms anytime incumbent)、`recovery.hpp`(主动 yield 最大化 clearance,实测优于盲目 z-climb)、`spacetime_corridor.hpp`/`st_graph.hpp`(动态时空)、`graph_search.hpp`(heat-A*)。

## 5. 全局-planner 层发现(w8vmg3m3b,已完成) — 全场 novelty 最高
**Top 候选(nov/sound/rt):**
- REGENT 0.74/0.55/0.46:生成式 GCS branch-and-price,GCS 对偶价格当 pricing oracle 在线 IRIS 长时空凸区域当 column + 粗完备松弛给【连续统有效全局下界】→ 第一个 anytime 全局间隙证书 + 对连续自由空间完备的 GCS 类规划器。撞车:GCS/ST-GCS/GCS*/INSATxGCS/ST-RRT*。
- CRISP-ST 0.72/0.55/0.55:**碰撞概率预算 ε 当可交易资源**(per-agent conformal 可放气管 r_i(α_i),Σα_i≤ε),单 λ 对偶放气扫描【联合】搜拓扑+预算,一次出 (时间 vs 认证风险) Pareto 前沿(λ 断点=同伦切换),anytime 对偶间隙证书。**"卡住"改写为 min-risk 一个数 → 搜索全函数永不输"无解"**。直接对接主线安全层(conformal tube + 遮挡 r_occ+v_max·t + body-floor 融成单一风险场)。撞车:**arXiv 2511.18170(conformal×SIPP×行人,头号)**、IRA/Ono-Williams 风险分配、2302.13115 对偶 RCSP、ST-GCS、T-MPC++。
- TANGLE 0.72:责任博弈 braid-lattice;SKEIN 0.68:时空辫索引运动记忆;CDB★ 0.58:冲突驱动 braid-格 anytime-complete;STRATA/PHALANX/STITCH/FORGE/DUET 0.37-0.5。

**三个跨候选收敛主题(真金,比任一单种子强):**
1. **"构造上永不卡死"= 永有认证安全回退(悬停/爬升/让行到不变安全集)→ planner 是全函数**,绝不"无输出"。几乎所有候选都有(复用 repo recovery.hpp/anytime_feasible.hpp)。这是对"找不到前进解就卡住"最直接的解。
2. **时空辫(space-time braid)/时空同伦签名当前后端统一坐标** = 同伦在【动态行人域】的推广(抢行vs让行=不同 braid 的 winding)。**正面填补 MINCO 内 F1 在动态域失效的 A1 致命洞。**
3. **anytime 对偶间隙/下界证书(全局最优性)** + **风险预算当搜索资源(min-risk 改写)**。

**撞车待 Wave3 实搜核查:** 2511.18170 / ST-GCS(2503.00583) / GCS*(2407.08848) / INSATxGCS(2410.08909) / ST-RRT*(2203.02176) / T-MPC++(de Groot 2024) / IRA(Ono-Williams) / 2302.13115 / Trautman-Krause / "Towards Optimizing a Convex Cover"(2406.09631)。

## 7. 收敛后的"真·新"核心假说(送 Wave2 锤炼)
**一个时空-braid 索引、风险预算化、anytime 的全局 planner,把"卡住"改写出存在性:**
- 全函数:返回 ε-可行轨迹 OR 认证 min-risk 轨迹 + 安全回退,**永不"无解"**;
- 动态域 sound:用 space-time braid(非静态同伦)→ 抢行/让行;填 A1;
- 全局最优:anytime 对偶间隙证书(REGENT 连续统下界 / CRISP-ST 对偶 Pareto);
- 风险预算:ε 当可交易搜索资源,接主线安全层;
- 引擎叶子:O(M) 二阶 MINCO(三路收敛)做 per-class 连续精化;
- 诊断:M6 reduced-Hessian 惯性证书的【推广】= 在线判"真堵(min-risk>ε)还是数值卡(λmin<0)还是错 braid"并路由。
**真·新颖残差候选:** (i) "卡住→min-risk 全函数改写"的 framing;(ii) 风险预算×拓扑【联合】搜索 + 对偶 Pareto;(iii) space-time braid 统一坐标下的 anytime-complete + 连续统间隙证书;(iv) 惯性证书路由从 MINCO 内推广到全局诊断层。**每条都要 Wave3 实搜确认没撞车。**

## 6. Wave2 输出目标
- 2-4 个连贯候选统一规格,至少含:(a) MINCO 内最强 = M6 证书路由 + O(M) 二阶 + 认证逃逸(FLÈCHE)+ 连续时间证书(FI-MINCO);(b) 全局层最强(待 w8 回来);(c) 可能的合体:把 M6 证书路由抽象为"全局 planner 的卡死诊断/分派层"。
- 每候选标注:真·新颖残差、撞车对标、动态域是否 sound、可证保证清单(送 Wave4)。
