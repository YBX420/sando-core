# MINCO/全局-planner "新算法"攻坚 — 最终交付报告

> 8+ 波多智能体攻坚(发现×5 → 汇总 → 撞车检测 → 形式化 → 证明 → 对抗破证 → 双轨硬化/再猎 → 证破新定理 → 平滑性)。
> 每条结论都经【实搜撞车检测 + 对抗破证】。诚实优先:写清什么是真新、什么死了、为什么。
> 工作产物在 `.claude/wave_work/*.json`(w1..w8 + smooth)。

## 0. 一句话结论
严格验证把"给规划器发明新 unstuck 算法"的搜索,收敛到 **1 个铁的真·新+sound 成果**——它不属于规划器,而是**你 RA-L 主线认证安全层的一个新证书**,且**修了现役代码的一个真 soundness bug**。"新规划器逃逸算法"那条线**确被先验占满**(连续 3 波 0 产出 + STRIDE/S2/二阶/Monitor 全被打成先验或假)。

## 1. genuinely-new + sound 最终账

| 原子 | 判定 | 一句话 |
|---|---|---|
| **S3**:连续时间 distribution-free conformal-Bernstein deficit 证书 | ✅ **1 个铁的** | 单条已提交 quintic 上,split-conformal tube 半径折进**精确连续时间 deg-10 Bernstein 凸包 deficit** → per-track/per-episode/marginal P(撞或误分)≤ε。**新在合取**(vs Jasour=distribution-free;vs Lindemann/Dixit=连续时间精确无 per-step union)。 |
| **S7-CRET**:认证 retiming escape | 🟡 **1 个候补(待证)** | deficit 解析首违时刻→闭式 z-保持时钟 warp + 闭式 go/no-go,单边完备。novelty 薄,但**正是"平滑的认证 recovery"**。 |
| STRIDE 防火墙 / S2 价格-winding / S5-F1+F2 二阶 / Risk-Wealth Monitor | ❌ **全死** | 见 §3。 |

**净:1 铁(S3,主线) + 1 候补(CRET,待证 + 服务平滑需求)。**

## 2. 存活成果 S3 —— 细节
- **是什么**:相对位置 r(t)=p(t)−c(t) 是矢量 quintic,S(t)=‖r‖² 是 deg-10 多项式;deficit D(t)=R²−S(t),R 含 conformal 半径 q。**CERTIFIED ⟺ max_k b_k ≤ 0**(b_k=R²−S_k,deg-10 Bernstein 凸包界)。整段连续时间、无 K 采样、无 per-step union(时间轴被 sup-over-horizon 分数预付)、mover 由 c(t) 当多项式携带(不塌,不同于 shipped per-CP halfspace)。
- **证明状态(Wave5)**:mostly-proven(conf 0.83)。(a)Bernstein 凸包界=定理;(b)control_points/C2B 逐行核对=精确 deg-5 Bernstein;(c)sup-score 收时间 union=Wave6 攻击 MISS(最硬一块)。
- **scope(诚实,写死)**:non-reactive/外生障碍域;marginal(跨标定抽取);per-round(**不复利成 per-flight**);commit<τ_trust=0.75s;Mondrian class×density;标定在 Isaac 机载 tracker 输出(**绝不 SDD**)。
- **学习预测器(Wave7)**:interpolate 网络 waypoint 为低次多项式 x̂_i + 选多项式 ρ_i + 标定真值-vs-插值 sup 残差 → 回到 Case A,soundness 不限 CA(tube 紧度由 W4 q95 gate 而非定理保证)。
- **可落地(Wave7 Track A)**:新 `bernstein_cert.hpp`(~200 LOC:Interval+running-error/fesetround、degree_elevate、Chu-Vandermonde 平方、deg-10 min/max-coeff、deficit_and_margin)+ `minjerk_traj.hpp` 加 `control_points_interval()`(~30 LOC)。双后端:区间(机载,μs 级)+ GMP 有理(离线 ground-truth)。**float-sign soundness**:计算 b̄_k≤0 → 真 b_k≤0。
- **修的真 bug**:shipped mover 门**双重 unsound**——per-CP halfspace 对 mover 连续时间塔(不同时刻法向)+ 密采样 K≥200 漏穿样本间 + 只 hold 到 0.75s(`plan_minco.hpp:482`,代码注释自承)。精确 deficit 当 GATE 取代它(保留 halfspace 当梯度 surrogate)。

## 3. 什么死了 + 为什么(省你在死路上的时间)
- **STRIDE "安全⊥枚举完备性"防火墙** = 教科书 **RTA/Simplex(Sha'01)/shielding(Alshiekh'18)**,形式核 A∩B⊆B。事实对、零定理新颖。降为一句 RTA-instance 定位。
- **S2 价格-winding Pareto** = mp-NLP critical-region(Bemporad/Tøndel)+ T-MPC++(2401.06021)+ Dual-CC-SSP(2302.13115)。且 σ_i 是瞬时点积符号**非拓扑不变量**、per-agent 预算**无代码 substrate**、无强对偶。撤。
- **S5-F1 惯性路由 / F2 O(M) 二阶** = IPOPT inertia + Pearlmutter R-op;部署 M≤12 二阶零收益(现稠密 O(M²))。非 load-bearing。撤定理,降附录。
- **Risk-Wealth Monitor "任务级 εN→ε 避撞界" = FALSE**。信息论反例:每回合 i.i.d. α 地板 ⟹ P(任一回合撞)=1−(1−α)^N→1;Ville 界的是**财富越界(type-I)非碰撞**;union 对每回合新事件本质最优、鞅零改进。唯一可救=非碰撞的 anytime-valid 标定漂移哨兵(partial-novel + gap-blocked G1/G2 + non-reactive)。**撤避撞界 claim**,至多 future remark。
- **动态/反应行人域 per-episode 概率保证** = 反应破可交换性(数据管线就破)+ 同组 **2511.10586**(Lindemann/Pappas,quadcopter)已自称"first valid interactive"。限 non-reactive 域。
- **整条"新规划器算法"**(拓扑枚举+并行优化)= EGO-v2/TRUST/Bubble/GCS 全占。

## 4. 平滑性(真需求)—— 和硬认证和解
**核心洞见**:接缝/recovery 平滑**不是 cosmetic,是修认证前提"飞的=认证的"**(今天被接缝 ~0.2m 乐观滞后 + 突兀 recovery 静默破坏)。

10 类 jerk 来源排序(逐行核对)Top:① 接缝 A_predicted≠A_actual(唯一同时破"飞的=认证的")② recovery_yield 刹车瞬时置零(`planner.hpp:1313`)③ recovery_climb 突兀垂直(`:1346`)④ yaw dyaw 阶跃(`yaw_smooth:1494` 只限 rate 不限 accel)⑤ 低速 atan2 抖动+滤波滞后。

**落地方案(全 default-OFF + golden 20/20 等价 + 不软化走廊 + 不碰证书)**:
1. **接缝 C2-from-exec-state**(最高,~30-40 行):新 MINCO 起始边界用**当前实执行态 A_exec** 而非预测态 A(=障碍侧诚实修正 1050-1063 的无人机侧对偶)。在 A_exec 上重跑同一套硬走廊/S3/FN/human。**强化证书**。
2. **S7-CRET recovery**(~60-90 行 + capi 重编):卡死时对已认证承诺轨迹做 z-保持 C2 时钟 warp(q'(0)=v_A,q''(0)=a_A 消接缝跳变,留认证走廊),闭式 go/no-go 重验 deficit。no-go 才退 yield/climb。
3. recovery 刹车 C2 splice(~15 行):瞬时置零换 min-jerk 减速。
4. yaw 二阶 reference-governor(cert-正交,demo 抛光,最低优先/最先砍):限 yaw-accel、滞回防低速抖、去 freeze/spin。**否决方案 A(yaw 进 MINCO 第4维)**——动 ABI + 与 cert retime warp 冲突。
5. k-value EMA + retiming 相位(低,post-9/15)。

## 5. `planner-vs-benchmark-split.md` 北极星重写
现框架三重错:**0 novelty(追平 2020 EGO)+ 违 9/15 铁律(A1-A4 全 side-paper)+ 治不了根因**。最危险 **A2(软化硬走廊)→ 必须 INVERT(铁律:不准软化)**,它会删掉唯一存活的 S3。
- A 节删 A1-A4 当目标;交付物=【写精确 deficit 原语 + 换双重 unsound mover 门 + (可选)漂移哨兵】。
- B 节扶正:50% 擦碰根因=台子把静障当**软 occupancy**(B2)→ 修法=静障**硬认证**(主线对齐),跨边界联合决策。
- B1/B3 单层 z occupancy 从"nice-to-have"升为"demonstrate 前置必做":不 3D,垂直 recovery 逃进未建模树冠、FN 遮挡/body-clearance 分支永不触发 → **评测台结构性跳不到主线贡献**。
- 新北极星:**评测台唯一目的 = non-reactive 域 demonstrate 认证安全层**;headline 图 = 连续时间 violation-rate vs 采样分辨率(S3 平在 0、离散 CP 上升)+ 三档密度经验覆盖 ≥1−ε。

## 6. 施工路线(pre/post-9/15)
**9/15 前可做(正交/增强主线、default-OFF、不吃 W1/W4/W7 关键路径)**:
- 接缝诚实(§4 步骤1)= cert-fidelity 修复,价值最高最小,首选。
- 精确 deficit 原语 `bernstein_cert.hpp` + 数值审计 golden(区间 vs GMP vs 1e6 采样)+ 换 mover 门。golden 20→21,OFF 字节等价。
- S7-CRET recovery(认证 C2 RTA recovery,强化主线 RTA)。
- training-conditional Beta-tail/DKW (n,δ) 膨胀 + per-round/per-flight 诚实记账。
- 评测台:多层 z occupancy + `--static_hard`(集成侧,不碰 .so)。
- demonstration:non-reactive benchmark + Isaac-tracker split-conformal 标定 + headline 图 vs Jasour/Lindemann。

**post-9/15 / side-paper**:yaw 进 MINCO(方案A)、k-value/retiming 抛光、把平滑当贡献写。

## 7. Boyle 一页纸(弱化定理,送 W1 闸门)
**Theorem(S3,non-reactive)**:对外生障碍 + 多项式/插值运动预测,在单条已提交 quintic 上,split-conformal tube 半径折进精确连续时间 Bernstein 凸包 deficit,逐回合、跨标定边际地认证 P(撞或误分被检测 track i)≤ε_cls+ε_pred。**新在**:distribution-free(vs Jasour moment/SOS)× 连续时间精确(vs Lindemann/Dixit 离散步 CP-MPC),且消 per-step union。**诚实边界**:non-reactive、marginal、per-round(非 per-flight)、Mondrian、Isaac 标定。**够不够当 RA-L 主线 headline 由 Boyle 裁**;不够则按 spec 回退 plan C(per-class MINCO side-paper 当主线)。

## 8. 给塔菲大人的诚实判断
- **找到了真·新的东西**(S3),但它比最初设想窄得多,且是**证书/主线**而非**新规划器**。这是严格验证的功劳——把激动人心但其实是先验的东西(winding-price/firewall/二阶/mission-bound)全部筛掉。
- **CRET 是意外收获**:既是候补新原子,又恰好是你刚要的"平滑认证 recovery"。
- **最高 ROI 单步** = 写那 ~250 行 deficit 原语 + 换掉双重 unsound 的 mover 门:它同时(a)把 S3 从纸面变 demonstrated、(b)修真 bug、(c)是接缝/CRET 平滑的根基。
