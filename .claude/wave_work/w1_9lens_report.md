All load-bearing facts verified in source. One correction to the synthesis surfaced: the repo optimizes `T` as a **direct** decision variable with L-BFGS-**B** box bounds (`minco_cost_grad.hpp:192` `T(i)=x(nq+i)`), not `T=exp(τ)` — so the M2 second-order math must be re-derived in direct-T coordinates (which is cleaner and gives B1's absolute T_min box for free). Here is the report.

---

# MINCO 卡死问题的发明空间与复合算法 PRISM-MINCO — 最终研究报告

> 定位声明(对账 CLAUDE.md):本报告是 **side-paper(per-class MINCO 规划器)** 的目标架构设计备案。它正面冲突于「9/15 前冻结规划器」纪律,**现在不动手**;主线仍是认证语义安全层。下面凡涉及代码改动均为「9/15 后」的施工蓝图,且我把所有关键命题都用仓库源码做了核对(给出 `file:line`,塔菲大人可直接复核)。

---

## 1. 一句话结论

**MINCO 有真实但狭窄的发明空间;它不在「换求解器」更不在「换轨迹表达」,而在两个点上:**(i) 用 **reduced-Hessian 的在线惯性/λ_min 证书** 把「这是哪一类卡死(拓扑 F1 还是数值 F2/F5)」做成 O(M) 的在线判别并据此路由——这是全场唯一既新又严的杠杆;(ii) 在仓库实际使用的 **time-anchor 目标**下构造一个 **valid 的 branch-and-bound 下界**(现有所有候选的 LB 在这个目标下都退化为 0)。其余三件数值机器(O(M) 二阶伴随 Hv、每段闭式时间块、filter-ALM)都**稳健但属增量**,价值是工程加速与可行性兜底,撑不起论文级 novelty。

**最该先做(最低风险、最高回报)= L2 连续层**(二阶 Hv + 精确时间块 + 有界-ρ filter-ALM),它整段落在现有 `alm_solve`/`plan_minco` 上、与冻结 ABI 和安全层正交、可在 `bench_tunnel/chaos/recovery` 上增量验证。**最该谨慎(最易被反驳)= F1 拓扑完备性那一层**:它在主线的动态行人域**自证失效**、在静态域又与 EGO-Planner-v2 架构重叠、且 sound 剪枝的 LB 在仓库目标下几乎恒为 0。

---

## 2. MINCO「卡住」的本质(F1–F5 浓缩成两条根因 + 一句桥)

五类失败其实是**两个不可混淆的层**,这决定了任何 sound 方案的分工:

**根因 I — 离散/拓扑层(F1 同伦陷阱 + F4 前端坏盆地)。** `(q,T)` 的连续梯度流是**类内**优化:它改不了 winding number / H-signature,除非把路点推过障碍——而碰撞罚恰恰禁止这一步,于是优化「顶着障碍推不动」。前端 corridor + 初值一旦钉死同伦类,后端无论多强都逃不出这个类。**这是唯一必须引入离散层才能 sound 解决的根因**,梯度法对它本质无能。

**根因 II — 连续/数值层(F2 罚函数病态 + F3 时间退化 + F5 带状条件数)。** 全是「同一个类内」的 conditioning 与罚景观问题:F2 = 罚权大则病态振荡、罚权小则违约,接触处非凸生伪极小;F3 = 某 `T_i→0` 段塌缩使映射矩阵 `M(T)` 病态、或 `T` 爆大;F5 = `T` 比例极端时 `M(T)` 条件数爆炸。三者同源,可被同一套二阶 + 时间整形 + 有界 ALM 的数值机器治理。

**一句桥(也是唯一真 novelty 的支点):**「我此刻卡在 I 还是 II」本身是**可计算判据**——reduced-Hessian 的 `λ_min`(<0 ⇒ 鞍/F2;≥0 且约束满足且 H-signature 失配 ⇒ 真错类 F1)+ 约束活跃度 + κ(M)。把这张判别表从 offline 直觉变成 online 测试,就是下面 M6 路由器,也是这套方案区别于「又一个拓扑并行规划器」的地方。

---

## 3. 排序后的 Top 方向

评分轴 = 功效 × 数学稳健 × 实时;「接入 C++ 代价」是我读仓库后的实测判断。

| 方向 | 攻克成因 | 核心机制(一句话) | 理论保证强度 | 接入现有 C++ 代价 | 新意 |
|---|---|---|---|---|---|
| **PRYBAR**(二阶 KKT) | F2/F5 + **诊断 I↔II** | O(M) 精确二阶伴随 `Hv` + GLTR 信赖域负曲率逃逸 + reduced-Hessian 在线惯性证书 | **最强(sound)**:收敛到二阶稳定点;`λ_min=−γ<0` 单步降 ≥ `γ³/(3L_H²)`(界依赖 **box 内** `L_H`,见 §6 B1) | **中–高**:二阶伴随可复用 `minjerk_traj` 现成 LU(adjoint 已在,见下);但**惯性需新的对称不定分解**(M(T) 非对称,不能复用),且要把 inner solver 从 `LBFGSpp` 换/并成 TR-Newton | 中(GLTR/二阶伴随是教科书;真新点 = **仅 `(q,T)` 微分平坦空间 + 在线惯性路由**) |
| **TEMPO**(精确时间块) | F3/F5 | 冻结 junction `(v,a)` → 每段可分,min-jerk 时间最优 = 解一元 6 次方程取正根 ∩ box | 中:对**可分代理** φ̄ 精确;对完整 `F` 需回溯=**非精确步**(§6 B2) | **低**:在 `minco_cost_grad` 的 explicit-T 梯度旁加每段闭式 predictor,复用 `energy_grad_time_explicit`,**不引入新线性系统** | 低(GCOPTER 已有时空优化;闭式 6 次根是 elegant 增量) |
| **Janus filter-ALM**(可行性恢复) | F2 | 给现有 ALM 外环加 filter 接受准则 + restoration phase + **κ-budget 反推 ρ 上限** | 中:**CQ(MFCQ)成立时**有界 ρ 精确可行(破 χ^{−1/2} 墙);稠密接触 CQ 系统失效(§6 B3) | **低**:仓库已有 `alm_solve`(`rho0=10, rho_max=1e8, grow=2`),只改接受/恢复逻辑与 ρ cap——而 `rho_max=1e8` 正是病态源 | 低(filter+ALM 经典;κ-cap 是启发式) |
| **HALO/PERSEUS 拓扑层**(枚举 + B&B) | F1/F4 | 枚举极小完备同伦类 + anytime best-first B&B + valid LB 剪枝 | **弱**:LB 在 time-anchor 下恒≈0、B&B 退化为排序;枚举不完备(H1);动态域失效(A1) | **高**:**GVD/Yen/H-signature 枚举器仓库不存在**(仅 ≤5 detour seeds + 单 H-sig passing-side),承重新建;anytime/recovery 已有可复用 | 低(与 EGO-v2 重叠);唯一可留的真新点 = **M7 valid LB** |
| **零件供体**(MOLT ψ_σ / GhostLadder 重播) | — | ψ_σ = 闭式高斯软化 **C³ relaxed-cubic 罚**;「顶层罚权→0 塌成单凸盆」洞见 + 异步重播 | 主 F1 机制已被数学反驳(Λ⊥w 假 / 三次 transcritical vs 绕障的 Z₂ 四次 pitchfork),**仅作组件** | 低(ψ_σ 直接替现 cubic 罚) | 低 |

> 下五名(HEDGE 幽灵续延 / CHAMP 学习暖启动 / GhostLadder & MOLT 整体 / PERSEUS standalone)**已降级**:或被核心机制反驳,或引入非 MINCO 重依赖(PERSEUS 每回合 conic IPM)、或靠未建枚举器(CHAMP T-escape)。它们的可救零件已在上一行收编,不单列以免灌水。

**裁决要点:** 没有任何候选拿 ≥2 票 fatal;但「头牌 contribution 存活」的只有 PRYBAR(correctness=sound)。**真正能写进论文的 novelty 收窄成两条:M6 证书路由 + M7 valid-LB**;其余都当 enabling component 卖。

---

## 4. 推荐的「理论最佳」复合算法:PRISM-MINCO

> **P**ortfolio of **R**educed-hessian, **I**nertia-certified, **S**eparable-time **M**INCO arms,拓扑编排的 anytime B&B。组织铁律:**(q,T) 连续优化治类内(II),离散层治跨类(I);用 PRYBAR 证书当路由器在两层间分派,每条 arm 全程 O(M) 带状 MINCO。**

### 4.1 架构(三层 + 两个窄接口)

```
┌─ L3 外层编排(F1/F4):枚举 + anytime best-first B&B ───────────────────┐
│  GVD→π1 生成元 → Yen K-shortest → H-signature 去重 → Top-K 类          │
│  每类: (SFC 走廊, 暖启动 seed(q0,T0), valid 下界 LB_j[M7])             │
│  + 【新】类失配在线监视器:每帧用 committed-traj 的 H-sig 重算(A2)     │
│  接口↓ {(corridor_j, seed_j, LB_j)}    接口↑ {(UB_j, state_j, λmin_j)} │
├─ L2 每类 arm(F2/F3/F5):filter-ALM 外环 + [TEMPO 预测 / PRYBAR 校正] ──┤
│  时间预测块 TEMPO(6 次根, T∈[T_min,T_max] box)                       │
│  联合校正块 PRYBAR(O(M) 精确 Hv + GLTR 负曲率, 同时覆盖 q 和 T)       │
│  罚 = C³ relaxed-cubic(ψ_σ);出口 = 类型卡死信号(M6,含 DYN_INFEASIBLE)│
├─ L1 MINCO 核(字节不动):(q,T)→ M(T)c=b → 一阶/二阶伴随(复用同一 LU)──┤
└─ 安全:anytime 可行 incumbent(anytime_feasible)+ 硬 WCET cap + 全堵→recovery ┘
```

**接口即 HALO 的接口**:把 PRYBAR/TEMPO/Janus 作为 arm 内脏插进去即成。L2 同时补两个最强组件各自的洞:PRYBAR 不能逃 F1 → 由 L3 逃;TEMPO 碰撞盲 → 由 PRYBAR 的联合 `(q,T)` 校正步补完整 `F` 的 KKT。

### 4.2 伪代码(已折入批判修正)

```
PRISM_MINCO(x0, goal, O_static, A_dynamic, budget):
  # ---- L3 前端(低频):极小完备类枚举(F1/F4) ----
  GVD  = generalized_voronoi(O_static)              # π1(F)≅π1(GVD) —— 限 2.5D 子流形(C3)
  cand = yen_k_shortest(GVD, s, t, len + w_clr/clearance)
  arms = []
  for P in cand:
     h = reduce(h_signature(P))                     # Bhattacharya 完备去重
     if h new and winding(h) ≤ W_max(scene):        # W_max 场景相关, 丢弃高 winding 时记录(A5)
        arms += build_arm(SFC(P), seed(q0,T0), LB_j[M7])
     if len(arms)==K or gvd_cost(P) > (1+η)*f_star: break
  if goal_in_obstacle_or_shadow(goal):              # goal 预检 + 投影(A4)
     goal = project_to_nearest_free_on_GVD(goal)    # 「目标不可达」≠「路径被堵」, 不触发 RTA

  # ---- L3 anytime best-first B&B(F1/F4) ----
  incumbent=∅; f_star=+inf
  while budget_left and any_live(arms):
     if class_mismatch_monitor(committed_traj, occ_map): re-enumerate()   # A2: 不等 arm 收敛
     prune {j: LB_j ≥ f_star or state_j∈{CONVERGED, DYN_INFEASIBLE, DEAD}}
     j = argmin_live  f̂_j = LB_j + (UB_j−LB_j)·ρ_j^b         # 排序启发式(不入剪枝)
     (UB_j,state_j,x_j,λmin_j) = ARM_STEP(arms[j], b)
     if feasible(x_j) and UB_j<f_star: incumbent,f_star = x_j,UB_j    # anytime
     if state_j==F1_BLOCKED:
        if ∃ corridor-certified 相邻自由类: GHOST_BRIDGE(arms[j])  # best-effort(H4)
        else: DEAD(arms[j])
  return incumbent if incumbent≠∅ else recovery_yield()    # recovery.hpp(非盲目 z-climb)

ARM_STEP(arm, b):
  for t in 1..b:
     # 联合校正(PRYBAR):O(M) 精确二阶,直接 (q,T) 空间
     g, Hv(·) = grad_and_2nd_adjoint(q,T; ψ_σ)         # 复用 M(T) 的 LU(M2)
     if ‖g‖ small: λmin,v = lanczos(reducedHessian_BunchKaufman, L)  # 独立对称因子(C1)
     d = GLTR_TR(g, Hv, P=GN+δ_w I+diag(1/T_i), Δ)     # λmin<0 → 含负曲率逃逸方向
     (q,T) = TR_accept((q,T), d)
     # 时间预测(TEMPO):每段独立闭式
     (v_j,a_j)=read_junction_states(c)
     for i in 1..M parallel:
        T_i* = argmin_{T∈posroots(ρT⁶−q4T⁴−2q3T³−3q2T²−4q1T−5q0)∩[T_min,T_max]} ψ_i(T)
     T = backtrack_on_full_F(T, T*); clamp_ratio(T)     # 对完整 F 回溯保单调(B2)
     # 约束/可行性(Janus filter-ALM)
     μ_ℓ=clip(max(0,μ_ℓ+ρ_ℓ g_ℓ),0,μ_max); ρ_ℓ ↑ ≤ ρ_max(κ_tol)   # κ-budget cap(取代 1e8)
     filter_accept((h,J)); if reject: Restore min ½Σ[g_ℓ]_+²
  return classify(‖g‖,α,λmin,g_max,sign(sᵀy),κ̂(M),Hsig(c) vs h*)   # M6
```

### 4.3 关键数学(子问题 + 批判修正已内联)

**(M1) MINCO 映射(L1,字节不动)。** `M(T)c=b(q,T)`,一阶伴随 `Mᵀλ=∂F/∂c` → `∂F/∂q=λᵀ∂b/∂q`,`∂F/∂T_i=∂F/∂T_i|_c+λᵀ(∂b/∂T_i−∂M/∂T_i·c)`。**仓库现状(已核)**:`minjerk_traj.hpp:45,159–160` 用 dense `Eigen::PartialPivLU` 因子化 `M(T)` 一次,`solve_adjoint`(`:258–266`)用**转置同一 LU**解 `Mᵀλ`。

**(M2) 二阶伴随 `Hv`(PRYBAR,O(M),复用 M(T) 的 LU)。** 方向 `v=(v_q,v_T)`:
- 切向 `M ċ = (∂b/∂q)v_q − Σ_k(∂M/∂T_k)c·v_{T,k}`(正向回代);
- `r = F_cc ċ + F_cT v_T`,`F_cc` 块对角 = 能量 Gram `2W(T_i)` + ψ_σ 罚 Hessian;
- 二阶伴随 `Mᵀλ̇ = r − Σ_k(∂M/∂T_k)ᵀλ·v_{T,k}`(转置回代);`Hv_q=λ̇`;`Hv_T` 由 `F_TT v_T + F_Tc ċ` 减去 `Σ_k{λᵀ[(∂²M/∂T_k²)c·v + (∂M/∂T_k)ċ] + λ̇ᵀ(∂M/∂T_k)c}` 组成。
- **修正(对账仓库,合成原稿用 `T=exp(τ)` 是错的)**:`minco_cost_grad.hpp:192` 是 `T(i)=x(nq+i)`——**T 是直接决策变量**,经 L-BFGS-**B** 的 box 约束保正。故 M2 应在 **直接-T 坐标**推导(**没有** exp 的 Jacobian/Hessian 链式项,更干净),而 B1 想要的绝对 `T_min>0` 由 box 直接给出。【保证:精确,O(M)】

**(M3) 联合校正(GLTR 信赖域)。** `min_d gᵀd+½dᵀ∇²F d s.t. ‖d‖_P≤Δ`,预条件 Lanczos 矩阵自由,每步 1 个 (M2)。不定时边界解天然含最左 Ritz = 负曲率逃逸方向。**`P=GN(PSD)+δ_w I(→PD)+diag(1/T_i)`,`δ_w I` 必修**(负曲率下 `‖·‖_P` 信赖域须 PD;δ_w 需自适应,§6 隐藏假设 11)。【保证:收敛到二阶稳定点;逃鞍单步降 ≥ `γ³/(3L_H²)`,**仅在 τ-box 内**(B1)】

**(M4) 时间块(TEMPO)。** 冻结 `(v,a)` → 可分上界 `φ̄(T)=Σ_i[E_i(T_i)+ρT_i]`,`E_i=Σ_{k=0}^4 q_k^{(i)} T_i^{−(5−k)}`。`ψ_i'(T)=0 ⇔ ρT⁶−q₄T⁴−2q₃T³−3q₂T²−4q₁T−5q₀=0`,正实根 ∩ `[T_min,T_max]` 代回取最小。**诚实重述(B2):6 次根是 φ̄ 的精确极小;对完整 `F` 用回溯保单调下降(非精确步)**;能量项可证 φ̄ 是 majorizer,碰撞项**标 surrogate-only**,由 (M3) corrector 兜 KKT。【保证:对完整 `F` 单调 + 无塌缩 + κ(M)≤κ_tol】

**(M5) filter-ALM(Janus)。** `μ_ℓ←clip(max(0,μ_ℓ+ρ_ℓg_ℓ),0,μ_max)`;`ρ_ℓ` 持续违反才增、**上限由 κ-budget 反推**(防 `ρ→1e8`,正是 `plan_minco.hpp:87` 的 `alm_rho_max=1e8`);卡死 → Restore `min ½Σ[g_ℓ]_+²`(= `alm_term(λ=0,ρ=1)`,零新推导)。**诚实(B3):有界 ρ 精确可行需 CQ(MFCQ);稠密接触梯度趋平行 → CQ 系统失效 → 退 Restore / 标 DYN_INFEASIBLE。**【保证:CQ 成立时有界 ρ 精确可行,filter 无循环】

**(M6) 类型卡死证书(路由器)。** 由 `(‖∇F‖, α, λmin, g_max, sign(sᵀy), κ̂(M), H(c) vs h*)` 判:

| state | 判据 | 动作 |
|---|---|---|
| CONVERGED | `‖∇‖≤ε_g ∧ λmin≥−ε_λ ∧ g_max≤0` | 冻结,`UB_j=f_j` |
| SADDLE | `‖∇‖≤ε_g ∧ λmin<−ε_λ` | 负曲率步(M3) |
| ACTIVE_KKT(F2) | `g_max>0 ∧ ‖∇‖≤ε_g ∧ ρ 未触顶` | ALM 乘子(M5) |
| **DYN_INFEASIBLE**【新,A3】 | `ρ 触顶 ∧ g_max>0 ∧ 仅 v/a/jerk 类约束活跃` | 判类内动力学不可行,**DEAD**(不再 ALM) |
| PLATEAU/SECANT(F2/F5) | `‖∇‖>ε_g ∧ α≤ε_α` 或 `sᵀy≤0` 频/κ̂ 大 | 预条件/重标度 + ψ_σ + 时间归一化基 |
| **F1_BLOCKED** | `‖∇‖≤ε_g ∧ λmin≥0 ∧ g_max≤0 ∧ H(c)≠h*` | 弃类/拓扑动作(L3) |

**关键修正(C1,已在代码确认):** 惯性**绝不能**复用 `M(T)` 的 LU——`M(T)` 是编码连续性/边界约束的**非对称**矩阵(`solve_adjoint` 必须转置 LU 才能解 `Mᵀ`,即铁证),其 LU 给不出惯性。`λmin`/惯性必须来自 **`F` 对 `(q,T)` 的 reduced Hessian** 的**独立**对称不定分解(Bunch-Kaufman / LDLᵀ)。**两套因子:① `M(T)` 的 LU 服务映射+一阶/二阶伴随(已存在);② reduced-Hessian 的对称分解服务惯性(新建)。**【保证:CONVERGED⇒真 KKT⇒UB 有效⇒剪枝不丢最优;误判只损算力(modulo capped-Lanczos 容差,B6)】

**(M7) 外层 B&B 的 valid 下界(修 HALO 在仓库目标下的退化)。** 仓库目标主项**已核**为 `w_time·((T_tot−T_target)/T_target)²`(`minco_cost_grad.hpp:12,235–241`,`w_time=10`)。类内可行 ⇒ `T_tot ≥ ℓ_j/v_max`;锚项是以 `T_target` 为顶点的抛物线,故
```
LB_j = ( max(0, ℓ_j/v_max − T_target) / T_target )²  +  jerk_floor_j
```
【保证:valid 全局下界,B&B 决不剪掉枚举集内最优类】**诚实(B4):`ℓ_j/v_max ≤ T_target` 时 LB=0、该区无 sound 剪枝(退回 best-first 排序);jerk_floor 跨 C² 内接点非严格 → 启发式收紧。** 这正是为何 §1 把 G5 从「类层全局最优」降级。

### 4.4 理论保证(诚实分级)

**【GUARANTEED】** G1 结构保持 & O(M)(M2/M4/M5 全带状/对角,复用同一 LU,决策量仍 `(q,T)`);G2 二阶稳定 + 负曲率逃逸(**τ-box 内**);G3 时间无塌缩 + 良条件 + 联合 KKT(M3 补 M4 的 Prop5 洞);G4 **CQ 成立时**有界 ρ 精确可行;G6 anytime + 确定性 WCET(硬 cap,带状 flops 数据无关);G7 证书 soundness ⇒ 剪枝正确(**capped-Lanczos 容差内,高概率**)。

**【HEURISTIC,明确标注】** H1 枚举只到 Top-K;H2 LB 紧致度(time-anchor 下常 0);H3 卡死分类阈值 + Lanczos 容差;H4 GHOST_BRIDGE 跨类桥;H5 学习暖启动;H6 ρ 外推仅排序;H7 动态时空 `2^m`:只取与 ego 走廊相交者 + clearance 剪枝,对抗稠密人群 → recovery。

**被批判明确削弱的两条**:G5「类层全局最优」→ **「枚举集内不误剪 + 各类局部最优取最好」**(类内非凸 + LB≈0 + 枚举不完备);G2 的逃鞍界 → **「τ-box 内」**(否则 `T→0` 时 `L_H→∞`,界空洞)。

### 4.5 复杂度

前端(低频):GVD `O(n log n)` + Yen `O(K·|V|(|E|+|V|log|V|))` + H-sig 去重 `O(K·M·n)`。每 arm 每内步:带状解/伴随/Hv `~3·O(M·bw²)`(bw=6)+ 采样罚 `O(M·K_s·d)` + Lanczos `O(L·M·bw²)` + 时间块 `O(M)`(M 个 6 次根并行)→ **段数线性**。外层 `O(K log K)`/轮。内存 `O(M)` 带状 + `O(K·M)`(K 条 arm)。WCET 数据无关(硬 cap)。

### 4.6 如何保住 MINCO 带状红利

- 决策量仍 `(q,T)`;`c` 仍由 `M(T)c=b` 解;一阶伴随路径**字节不动**(golden 安全)。
- 新增线代全部尊重带宽:二阶伴随 = 2 次额外回代(O(M),复用 `M(T)` 的 LU);Lanczos = `O(L·M)` matvec;时间块只读已算 `c` + 解 M 个 O(1) 标量 6 次根(**不引入任何新线性系统**,可并行);filter-ALM = 对角乘子记账。**无一处引入跨段稠密耦合。**
- **两个工程前提(否则 O(M) 保证打折,§5/§6 承重)**:① 仓库现为 **dense PartialPivLU**(已核),严格 O(M) 需先写带状 LDLᵀ——但**当前 `M≤12`→`72×72` dense 是 μs 级,O(M) 只在 M 增大时承重**,可后置;② **GVD+Yen+H-signature 枚举器仓库不存在**(已核,grep 仅得单 H-sig passing-side seed),是 F1/F4 承重新建。

---

## 5. 落地到现有 C++ 代码库的分阶段路线(最低风险高回报优先)

> 全部「9/15 后」。所有 Phase 1–2 与冻结 ABI / 安全层**正交**,可在现有 bench 上增量验证;Phase 3 与「冻结规划器」纪律冲突,最后再上。

**Phase 0 — 前提核对(部分已完成)。**
- **C1 已 closed**:`M(T)` 经源码确认**非对称**(`minjerk_traj.hpp:258–266` 转置 LU),故惯性必须独立矩阵——**Phase 2 直接按两套因子设计,不要试图复用 LU 做惯性**。
- **带状 LU 暂不写**:`M≤12` dense 是 μs 级,标记为「规模触发」项(M 增大或上多 arm 后再做)。
- 真正要先建的地基 = **reduced-Hessian 的对称不定分解**(Eigen 无现成 banded Bunch-Kaufman;现规模可先 dense `LDLT` + modified-Cholesky 兜底,见 §6 C2)。

**Phase 1 — L2 数值层(最低风险,纯加速 + 兜底,落在 `plan_minco.hpp`/`alm_solve`)。**
- **1a TEMPO 时间块**:在 `minco_cost_grad` 的 explicit-T 梯度旁加每段 6 次根 predictor + 对完整 `f` 回溯。落点 `bench_tunnel`(窄缝时间整形)、`bench_corridor`。复用 `energy_grad_time_explicit`,零新线性系统。
- **1b Janus filter-ALM**:改 `alm_solve` 的接受逻辑 + 用 κ-budget cap 取代 `alm_rho_max=1e8`(`plan_minco.hpp:87`)。落点 `bench_recovery`、`bench_chaos`。复用现成 ALM 记账(`lam<-max(0,lam+rho*g)`)。
- **1c ψ_σ(C³ relaxed-cubic 罚)**:替现 cubic 罚,平滑 Hessian,为 1a/2a 的二阶量铺路。
- 验收:`ctest` 19/19 保持全绿(golden 不破);bench 上 wall-clock 收益数据立 TEMPO/Janus 的 contribution(对标 GCOPTER 时间优化,§6 D4)。

**Phase 2 — PRYBAR 二阶 + 证书(真 novelty 核心,中风险)。**
- **2a 二阶伴随 `Hv`**:复用 `minjerk_traj` 的 LU(adjoint 已在),加 `ċ`/`λ̇` 两次回代;**在直接-T 坐标**(非 exp(τ),按 §4.3 修正)。验证:对 `minco_cost_grad` 的数值 `Hv` 比对。
- **2b reduced-Hessian 惯性**:独立对称不定分解 → `λmin`(现规模 dense)。
- **2c GLTR TR-Newton inner solver**:替/并 `LBFGSpp`(`plan_minco.hpp:45,724,761`),`P=GN+δ_w I+diag(1/T)`。**L2 内核单独 bench,重核安全层监视器节点端到端 WCET**(§6 C7——这是最小切入路径的前提)。
- **2d M6 路由器(含 DYN_INFEASIBLE)+ A2 类失配在线监视器**。

**Phase 3 — L3 拓扑层(高风险,后置)。**
- 枚举器 **GVD+Yen+H-signature 承重新建**;建成前 L3 = 现有 `generate_detour_seeds`(`max_seeds=5`)+ 单 H-sig passing-side(`plan_minco.hpp:159,162`)顶着,F1 完备性降为「≤max_seeds 启发式组合」。
- B&B + M7 valid LB(诚实:LB 在 time-anchor 下常 0,B&B≈best-first 排序)。
- **claim 显式限静态/2.5D**(A1/C3);动态主线继续用现成 `spacetime_corridor.hpp` + `st_graph.hpp` + `recovery.hpp`。

**仓库已有、可直接复用的便宜资产(读码确认):** `anytime_feasible.hpp`(anytime 可行 incumbent,目标 <50ms,deadline-stop 且 computation-invariant 人体安全——部分覆盖 §6 C5/C6);`recovery.hpp`(全堵时**主动 yield 最大化 min-clearance**,`bench_recovery` 实测 freeze `−0.29`→yield `+2.97/+3.70`,**比盲目 z-climb 强**,部分解 §6 A6);`graph_search.hpp`(heat-A* 向导)、`hgp_planner/manager`(全局编排)。**这些让「先可行后优化」的兜底链不必从零造。**

---

## 6. 诚实的风险与未决问题(来自批判)

**A. 已在代码证实的硬伤(从「很可能」升级为「确认」)。**
- **C1(已证实)**:`M(T)` 非对称(`solve_adjoint` 转置 LU)、dense `PartialPivLU`,**复用同一因子做惯性是概念错误**。修法已定:两套因子(M6 的地基由此 closed)。
- **M7/B4(已证实)**:time-anchor 目标 = `((T_tot−T_target)/T_target)²` 是**主项**(`w_time=10`)。长度型 LB ≈ 0,**B&B sound 剪枝几乎失效** → G5 必须降级为「枚举集内不误剪 + 各类局部最优取最好」。要真 LB,需在 SFC 凸多面体上对「能量 + 时间锚」做凸松弛/对偶界,否则 B&B = best-first 排序。
- **工程前提(已核)**:带状 LU 与拓扑枚举器仓库均无;现 `M≤12` dense 是 μs 级,O(M) 仅规模承重。

**B. 命题需弱化(理论缺口)。**
- **B1**:逃鞍界 `γ³/(3L_H²)` 依赖**全局 `L_H`**——`E_i~T^{−(5−k)}` 在 `T→0` 高阶导发散,`L_H` 随段塌缩→∞。**必修绝对 `T∈[T_min,T_max]` box**(仓库 L-BFGS-B 的 bound 直接支持),G2 重述为「box 内」。
- **B2**:「exact 时间块」与「对完整 `F` 单调」二选一。重述:6 次根给**可分代理**精确极小;对 `F` 用回溯保单调(非精确步)。
- **B3**:G4 精确可行需 CQ(MFCQ);稠密接触系统失效 → 降为「CQ 成立时」+ 失效退路(Restore / DYN_INFEASIBLE)。
- **B5(转录 gap)**:`g_max≤0` 只在采样点查,**采样点之间可连续违反** → CONVERGED 未必连续时间可行 → UB 未必有效。MINCO 段是多项式,用 **Bernstein/SOS 段内上界**做 a-posteriori 连续时间证书(O(M) 天然)把 UB 升级为 certified。
- **B6**:capped Lanczos 的 `λmin` 在谱簇聚时数据相关,与确定性 WCET **互斥**。证书降为「高概率 sound」或对疑似卡死 arm 分配更大 `L` 预算。

**C. scope / 动态域(最致命的 framing 攻击面)。**
- **A1**:主线动态行人**恰是 F1 保证的盲区**(全部拓扑保证只对静态/准静态成立)。**claim 必须显式限「静态/准静态杂乱环境」,动态交安全层**;evaluation 只在纯静态 bench(`bench_tunnel/chaos`)立 F1 claim。这是答辩第一攻击点。
- **A2**:receding-horizon 下 F1 会从静态重新长出(已选类中途被堵)。需**类失配在线监视器**(每帧 `O(M·n)` 用 committed-traj 的 H-sig 对当前占据图重算),失配即重枚举,不等 arm 收敛。
- **C3**:2D 同伦理论(winding / Bhattacharya H-sig / π1(GVD)≅π1(free))搬到 3D 不干净(3D 同伦更丰富、H-sig 需基于曲面、GVD 数值脆)。**限 2.5D/固定高度层**,F1 claim 限在该子流形。
- **C4**:噪声占据图 → GVD/类集逐帧闪烁 → incumbent 抖动 + committed-traj 跨类跳变(破 tracker/安全层 tube 假设)。需占据图时间滤波 + 类集迟滞 + commit-hysteresis。

**D. 新意风险(related work 必须正面处理)。**
- **D1**:L3+L2(拓扑搜索 + 并行类内优化)与 **EGO-Planner-v2 / Zhou-Gao 拓扑 PRM / Rösmann TEB** 高度重叠。**contribution 必须收窄到 M6 证书路由 + M7 valid-LB,在 related work 正面对标 EGO-v2**:「不是又一个拓扑并行规划器,而是给它装了一个 sound 的卡死诊断/剪枝层」。
- **D2**:二阶伴随会被问「为何不 iLQR/DDP」。**novelty 钉死在「仅 `(q,T)` 微分平坦 + 带状映射」**:iLQR 在全状态-控制空间、PRISM 在 `(M−1)+M` 维路点/时长空间,变量量级差 1–2 个数量级——**必须量化(变量数 + 每步 flops)**,否则被当已知技术。

**E. 未决问题(需先做的实验/决策)。**
- **C2**:带状 Bunch-Kaufman 的 2×2 pivot 会溢出带宽 → 最坏 O(M·bw²) 不再成立。现规模可先 dense;规模触发后实测带宽膨胀,或退「`+δ_w I` 强制 PD + Cholesky + modified-Cholesky 近似惯性」并评估对 M6 的影响。
- **C6**:测 incumbent-availability rate vs 场景密度;为「保证至少一条可行 incumbent」预留一条便宜保守 arm(GVD 最短安全路 + 大时间裕度)。
- **A3**:DYN_INFEASIBLE 需一个**类内动力学可行性的快速必要测试**(沿 corridor 最短到达时间 vs 段长下界)。
- **A4**:goal 落障碍/遮挡阴影 → 全 arm 不可行 → 误触 RTA。需 goal 预检 + 最近自由点投影,把「目标不可达」与「路径被堵」分开。
- **隐藏假设 11**:ψ_σ 的 C³ 不保证 Hessian 在接触边界 PD,`δ_w` 取值未定 → 需自适应 `δ_w` 规则及其对收敛界的影响。

---

### 给答辩的一句话总结
PRISM-MINCO 最稳的是 **L2 连续层(G2/G3/G4 的数值机器)**,但头牌 F1 拓扑保证(G5)**在动态主线域失效、在静态域与 EGO-v2 重叠、且 sound 剪枝的 LB 在仓库 time-anchor 目标下几乎恒为 0**。**最高优先级三件**:(1) C1 的惯性矩阵已查清是 reduced-Hessian 而非 `M(T)`——按两套因子落地,否则 M6 路由器地基塌;(2) 所有拓扑 claim 显式限静态/2.5D + 补 B5 连续时间可行性证书,否则 UB/剪枝不 sound;(3) contribution 收窄到 **M6 证书路由 + M7 valid-LB**,正面对标 EGO-v2,避开「重复造轮子」的致命质疑。**工程上先落 Phase 1(TEMPO + filter-ALM + ψ_σ)——全场最稳、最 realtime-safe、与安全层正交——再上 Phase 2 的 PRYBAR 二阶 + 证书,L3 枚举器最后。**
