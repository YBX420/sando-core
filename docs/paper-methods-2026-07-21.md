# 论文方法部分组织稿(2026-07-21,冻结栈 stack-freeze-m3off-2026-07-21)

> 依据:tag `stack-freeze-m3off-2026-07-21`(bbfdfd6)+ 战役 commit 链至 `85f7cb9`;
> 一切数字出自封卷 artifact `out/conformal/calib_v6_final100.json` 与收据链,可逐字复核。

---

## 1. 整体算法(pipeline)

```
                         ┌──────────────────────────── 离线(冻结前) ────────────────────────────┐
  Scenario Generator ──► Residual Collector ──► Conformal Calibration ──► 管参数 (q̃, v_eff)/类
  (populate/final100)     (harvest_v3, 原子)      (设计面定形状+场景级秩)          │
                         └──────────────────────────────────────────────────────┼───────────────┘
                                                                                ▼ 在线(每拍 0.1s)
  传感锥+遮挡 ─► 感知前端 ─► CA-Kalman track ─► CV 预测多项式 ─► ① 喂 planner(σ-带占据环+时间维代价)
  (±45°,10m)   (NN关联,     (两点初始化;A+     c(t)=c0+v·t      ② 喂证书(逐类保形 keep-out)
                噪声/漏检)   CV-coast;秒 TTL)                          │
                                                                       ▼
  EGO-Planner(B样条,ESDF-free)─► 方向×速度锦标赛 ─► 连续时间 Bernstein 证书 ─► 逐字执行
   围绕"未来占据"规划            (每候选独立认证)      (胶囊珠链∧冻结分量∨飞越;      (flown==certified;
                                                     窗口覆盖契约,fail-closed)     悬停尾窗补全)
                                                            │ 证不过
                                                            ▼
                                        HOLD 兜底(悬停自证或显式记 U 暴露)+ 收据/归档(U/P/Q/D)
```

**四个组件一句话+要点:**

**Planner(认证的最快-最安全绕行)。** vendored EGO-Planner(三次 B 样条、ESDF-free L-BFGS)。喂给它的不是障碍的现在,而是 **KF 预测的未来占据**:每个 mover 按预测位置渲染占据环,环半径带 σ-预算带(`R + min(K_ETA·predict_sigma(t), 1.2)`,全知臂构造性为零);solver 内加 EGO_TDYN 时间索引代价(控制点 i 在自己的时刻 t_i 对 mover 匀速多项式罚,唯一一处 solver 手术)。决策层是统一锦标赛 `maneuver_decide_v2`:候选 = 方向(直行/±25°/±50°/飞越/爬升)× 速度档(1.0…0.3),**每个候选单独过证书**;incumbent 驻留防抖、认证的逐格释放;执行器**逐字执行**当选档(改 s(t) = 改时空轨迹,必须重证——家法)。样条短于信任窗时,终点悬停对余窗 [u_stop/warp, τ] 单独认证(共享停点 u_stop=duration−1ms=执行器冻结点)。全都证不过 → HOLD 兜底:悬停能自证则发 hold_cert 收据,否则显式记 U 暴露。

**证书(连续时间,逐入口 fail-closed)。** 对已承诺 B 样条逐 mover 验:`(预测管 ∧ 冻结分量) ∨ 飞越`。keep-out 是 v6.1 胶囊:段 [mover 背点, KF 预测尖端] ⊕ q̃ 的珍珠链覆盖(s=0 珠=旧冻结分量,s=1=预测管)+ 背面盘。数学是 Bernstein 亏格:‖p(t)−c(t)‖²−ρ(t)² 的系数凸包上界 ≤0 ⟹ 全连续时间安全,区间算术外圆整保证"CERTIFIED 永不是浮点谎言";ρ(t)=R+v_eff·(t+δ) 时间增长管。**窗口覆盖契约**:有限请求窗必须由从 t≤0 起的连续铺砖盖满,否则大声拒绝——核心永不静默缩窗(含 MINCO 入口)。每拍产 **certificate receipt**(plan hash、障碍快照行、标定 SHA、执行 hash、有效窗),碰撞按 **U/P/Q/D 代数**自动归档(U=无证书绑定执行;P=肇事物不在快照=感知账;Q=真轨迹越出标定管=ε 该赔;D=管内仍撞=确定性违约零容忍)。**定理形态:P(Q)≤ε;C_cert,represented ⊆ Q∪D;验证 P(D)=0 后得 P(C_cert,represented)≤ε。U、P 分列实测。**

**Generator(预注册基准生成)。** `populate` 内核:程序化街景(MetaUrban)+ 撒 mover + **contest**(选锚横穿者、走廊垂穿其穿越点、按 t_drone=frac·L/v_nom 对时 spawn_t 使真相遇;锚级走廊轮换;起飞区 clear-start 律=32m 退避+横向探点,找不到即 REJECT;起点移动后**按最终几何精确重对时**+重验间距)。验收 = 时空相遇条件 `|t_mover−t_drone|≤1.0s`(逐交点检查)。每场景 provenance 全录(generator commit/实际 attempt seed/family/params/map seed/scenario SHA);同世界 multistart 归一 cluster。

**Collector(原子残差收割)。** `harvest_v3`:一集=一解释器=一原子文件(tmp→sha256→rename→receipt);作业表预注册,episode id≡注册序号;单 worker、BLAS 单线程;resume 需 data+receipt+checksum+job 四配;merge 要求收据集与注册**全等**;**合法零行集进 universe.json 计 −inf 参与排序**(否则空相遇场景静默消失——退役池曾 20/69 消失)。收据内嵌全部生效 env 旋钮+运行时 .so SHA(git tag 冻不住手编二进制,`STACK_MANIFEST` 下 dlopen 前逐集核验)。

**Conformal calibration(裂脑设计:形状归设计面,秩归 cal 面)。** 见 §3。

---

## 2. 实验设计(final100 基准)

**预注册协议**:100 个 slot 在生成**之前**全部注册(`final100_plan.json`,immutable,plan_sha 自校验,绑定冻结栈全量 git SHA + 三 .so SHA);生成器 git+运行时双锁,任何代码改动自动作废 plan(本战役三次现场修 bug 即三次重注册,审计链保留全部作废件)。

**四家族定义**(标签由冻结 RNG(seed 20260721)**独立等概率**抽取——论文主张是"对该四家族等概率生成分布的 pooled conformal 覆盖",实现抽样 18/38/26/18 合法、不重抽、不得称"经验平衡"):

| family | 生成参数 | 覆盖的失效模式 |
|---|---|---|
| `single_timed_encounter` | 4 行人+2 横穿者、无车;contested 对时 | 单次干净定时交会(基本几何) |
| `dense_multibody` | rush 密度档;contested | 密集多体混行、走廊压力 |
| `occlusion_coast_reacquire` | 6+3+1;hard(遮挡广告牌+3 锚交错) | 遮挡晚现、锥缘 coast/重捕获 |
| `high_dynamics_vehicles` | 3+2+4 车;hard(车辆对时+对头) | 高动态、曲线、车辆交互 |

动物类**只进 stress suite,不进主覆盖定理**(artifact 中标 STRESS_SUITE_ONLY)。

**cal/test 划分与 60/40 的理由**:split 是 slot 的属性,在生成前由冻结洗牌固定(不看任何结果)。60/40 的算术:可交换单元=**场景**(同场景多 episode 共享几何与剧本,不是独立样本——旧产线的教训);n_cal=60 时 ε=0.05 取第 ⌈61×0.95⌉=58 阶,最坏端仍有 **2 个样本余量**;ε=0.10 取第 55 阶,余量 5——两档都不落在 max-rank 悬崖上。n_test=40 给覆盖率 **2.5% 分辨率**。每场景统一 2 个感知 episode(单元同质);场景分=两集取 max。**cal 只算分位数;test 由 `--open-test` 恰好打开一次**,结果封进 artifact(`test_opened=true`),此后任何调参都非法。

**与设计面的关系(裂脑防泄漏)**:管**形状**(b, v_eff 斜率)、rear 律、以及全部算法超参(EGO_MEM_K=2.0 锚缓冲等)在**退役设计面**(旧 69 场景重收,design69,138 集)上确定并冻结;final100 的世界是全新生成 ⟹ 折与设计面**场景不相交 by construction**。K 影响飞行路径,故绝不允许看 cal 后再选。

---

## 3. 评价指标

**Nonconformity score(逐行→逐飞→逐场景)**:

- 行级残差(收割时每个 (track, 视界 d) 一行):**胶囊残差** `esg = dist(真值未来位置, 线段[KF 锚 c0, 预测尖端 c(d)])`——减速/停下的 mover 得 ~0 而不付全程沿轨误差(这正是部署 keep-out 的形状);背面越出 `erb`(rear 律)。
- 形状归一(设计面拟合的仿射包络,Theil-Sen 斜率):成熟 track `s_row = esg / max(b_cls + v_cls·d, b_min)`;young(age<4,冻结盘律)`s_row = (esg − v_c·d)/b_y`;rear `s_row = erb / max(b_r + v_r·d, b_min)`。
- **飞行分 = sup** over 合格行(qual=1, age≥2),capsule 与 rear 取 joint max;**场景分 = max** over 该场景 2 个 episode;**零合格行 = −inf**(仍在 universe 内投票)。

**λ 与部署管**:λ = cal 60 个场景分的第 k 阶次序统计量,k=⌈(n+1)(1−ε)⌉。部署:`q̃_cls = λ·b_cls`,`v_eff_cls = λ·v_cls`;keep-out 半径 `R(t) = r_obs + r_body + d_safe + q̃ + v_eff·(t+δ)`(δ=感知→承诺延迟)。young/rear 同法缩放。本次:**ε=0.10 λ=2.241(k=55/60);ε=0.05 λ=2.525(k=58/60)**。

**Coverage 定义与主张措辞**:在场景可交换性下,新场景的场景分 ≤ λ 的概率 ≥ 1−ε(标准 split-conformal 有限样本保证,单元=场景)。实测:test 40 场景中分 ≤ λ 的占比。**本次封卷:ε=0.05 实测 0.950(38/40,正中名义);ε=0.10 实测 0.875(35/40)——分辨率 0.025 下低名义一格,Binom(40,0.9) 下 P(X≤35)≈0.27,报告为"观察低于名义",非统计判决。** 论文措辞禁"distribution-free",写 **"model-free, valid under scenario-level exchangeability of the registered generator mixture"**。

**行为/安全指标**:头号=**干净抵达**(到达 ∧ 零碰撞 ∧ 零无证紧急暴露);碰撞逐对象归 U/P/Q/D(带完整证据链:接触时刻、快照行、管越出量、track 态);无证暴露拍数(HOLD/evade)分列;时间、机体最小净空(扫掠最早接触口径)、切换/spchurn(舒适度)。

---

## 4. 最终 artifact 与主表

**`calib_v6_final100.json` 结构**:

```
provenance: spec / ruling / plan_sha / stack_sha + stack_runtime(三 .so SHA)
            design_npy(path+sha) / cal_run / family_scheme + histogram
            rank_0.05: {n:60, k:58, lam:2.525, slack:2, observed_test_coverage:0.950, n_test:40}
            rank_0.1:  {n:60, k:55, lam:2.241, slack:5, observed_test_coverage:0.875, n_test:40}
            test_opened: true(开考一次即封的凭证)
capsule: true, n_pearls: 6
groups.{pedestrian|vehicle|static}.levels.{0.05|0.1}: {q_conformal, v_eff, status}
groups.animal.levels.*: STRESS_SUITE_ONLY
young.{cls}.{eps}: {q0y, growth}          # 冻结盘 young 律
rear.{cls}.{eps}:  {q0r, growth}          # v6.1 背面律
```

**计划主表/主图**:
- **T1 校准主表**:ε 档 × (λ, k/n, slack, 实测覆盖)——本文核心承诺;
- **T2 逐类管参数**:q̃/v_eff/young/rear(两档);
- **T3 final100 行为表**:ours(冻结栈)vs baselines × (干净抵达率、碰撞及其 U/P/Q/D 分解、无证暴露拍、时间、净空)——分家族列;
- **T4 消融阶梯**:执行器逐字化(±)、记忆秒语义、k10 σ-带、估计器双刀(coast/TTL)——渲染面 6-seed + 回归面已有全部数字;
- **图**:①pipeline 图(上方 ASCII 的正式版);②四家族示例俯瞰+成片帧;③coverage 散点(40 test 场景分 vs λ 线);④证书几何示意(胶囊珠链+背面盘+飞越);⑤U/P/Q/D 判决树。

---

## 5. Baselines 与对照臂

| 臂 | 角色 | 状态 |
|---|---|---|
| **native MIT-ACL SANDO(锥对齐)** | 主 baseline。同一感知锥(历史教训:native 曾拿 30m 全知 mover vs 我们 8m 噪声锥=不公平,已裁"锥对齐+predict=False 消融双对照") | **待在冻结栈重打**(九月主表首项欠账;视力手术后未重跑) |
| **裸 EGO(--ego)** | planner 同、无安全层/预测 → 隔离"认证预测栈"的净贡献 | 渲染面现成 |
| **ours 冻结栈** | 主臂(M3-off 策略) | 新 6-seed 渲染基线 9.45s 已转正;final100 行为面待跑 T3 |
| ORACLE_THIN(全知) | 分析臂:架构上限/估计器税标尺(非 baseline) | 需在冻结栈重校验 |
| GT-xy(零噪声) | 分析臂:感知噪声税分解 | **⚠️ 07-21 体检异常**(s7:12.8s/净空0.034/hold51,方向反常),KFDBG 诊断 log 已产,病未定前**不进论文** |
| 理论最优(考卷答案,B 档剧本) | 三线主表的第三条线 + "物理包络"判词的终审判官(ICS 审计) | 设计已议,未开工 |

**结果/讨论组织建议**:主线 = T1(承诺)→T3(行为,分家族)→U/P/Q/D 分解(把"哪类失效谁负责"讲成故事:ε 只为 Q 负责,P 是感知税、U 是 RTA 暴露、D 恒零=确定性链完好)→T4 消融(每刀的税与收益都有 AB 数字)→诚实边界段(横穿者让行地板、物理包络重审、ε=0.10 覆盖观察、GT-xy 异常若未解需披露)。

---

## 附:本稿数字的复核路径

`git tag stack-freeze-m3off-2026-07-21` → `out/final100/plan_20260721d.json`(注册)→ `metaurban/scenarios/final100/*.receipt.json`(生成收据)→ `metaurban/out/harvest_runs/final100{cal,test}_f1/`(收割收据+universe)→ `out/conformal/calib_v6_final100.json`(封卷 artifact)。每一步的 SHA 链在收据里闭合。
