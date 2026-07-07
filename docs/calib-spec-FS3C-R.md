数据现状已核验(62ep quick 文件、网格顶 0.85、ep_scn 68≠62、static 15170 行/6ep),与三份审查的实测一致。以下为最终合成规范。

---

# 最终实现规范 FS3C-R:场景级航次上确界 split-conformal 仿射管标定(修订裁决版)

## 0. 裁决摘要

**采纳骨架 = FS3C 的统计核心**(A 折定形 / B 折单秩、类归一化加性充气 q0=b+q̂σ、v_eff 不被尾部充气)——这是三案中唯一被两轮对抗审查明确标注"不要动"的部分。**吸收 GateRSC 两件东西**:可交换单位上移到场景级(经 Dunn 每场景抽一 episode 的精确形式),以及 UNDER_CALIBRATED 降级输出(绝不 +inf/0/静默)。**吸收 Flight-sup 两件东西**:ε 三桶记账结构(A1 管逸出 / A2 感知缺席 / A3 确定性桥梁)与 fail-loud 铁律。**否决**:GateRSC 的乘性 λ̂(斜率被充气,其原型自己触发 REFUSE:tube(1.05)=3.62>3.5)、Flight-sup 的 R_bind=5(几何自相矛盾)、FS3C 的 episode 级 iid 容忍秩(聚簇下无效)。

**一个决定性设计动作解决三分之一的致命伤**(数据窥探、seed 重叠、网格设计非可交换、optional stopping):**全部历史数据与全部历史 seed 整体划为设计域(折A);折B 与 TEST 只用从未出现过的全新 PERCEPT_SEED,收割前预注册落盘**。split conformal 只要求 B ⊥ 分数函数,不要求随机切分——"B=全新seed"是最干净的满足方式。

---

## 1. 致命伤逐条裁决(按主题合并,涵盖四份审查全部条目)

| # | 致命伤(出处) | 裁决 | 处置 |
|---|---|---|---|
| 1 | R_bind=5 与 VCAP_TRUE[veh]=8 自相矛盾,dd∈(5,13] 碰撞无桶可归(FS 两审) | **成立,接受** | ρ_c 按类推导公式(§2),veh≈16.5m;收割按 ρ_c 门,部署证书**不需要** dd 门(见 §2 可达性引理——部署多查远处 mover 只增保守性不伤音性) |
| 2 | 门控→类饥饿→静默退化 pedestrian-only(FS-F2) | **部分驳回** | 是 R_bind=5 的下游:诚实 ρ_veh 下 97.5% 车辆行回归。static 形状不承担有效性,用全部 static 行拟形状(dd 门只作用于打分资格),饥饿消失 |
| 3 | −inf 空 episode 散文/伪码矛盾 + 稀释 gaming(FS-F3) | **成立,接受 X2** | 主保证改 Mondrian-on-binding:秩只在含合格行的 B 场景上取,P(违管\|航次含合格行)≤ε,marginal 版自动成立(违管⊆含合格行);q̂ 非有限一律 raise |
| 4 | 固定网格收割 ≠ 可交换向量、有效 n≈场景数(FS-F4;FS3C-#2#3;GateRSC 单元论证) | **成立,接受** | 单位=场景,Dunn 每场景抽 1 个全新-seed episode(精确可交换);目标总体显式声明为"冻结场景池均匀 × 新seed";(ε,δ) 容忍层在**场景级**算,池<45 前用 100 次重抽 p10 作经验 δ 并标注 |
| 5 | 6 个零行 episode 静默消失=删失偏倚(FS-F5;GateRSC 审) | **成立,接受** | 收割 manifest 逐 launched-episode 落盘(完成态/行数);零行且含交通=收割失败,查根因或记 A2;严禁填 0/−inf |
| 6 | "顶端外推反保守"方向写反 + 边界垫低估(FS-F6) | **成立,接受** | 网格扩为 {0.00,0.05,…,1.05}(含 Δ=0,消灭边界问题);垫 η_c=(VCAP_TRUE_c+VCAP_PRED_c+v_eff_c)·h/2(GateRSC 审的修正式,含包络自身移动项) |
| 7 | ε_miss=0.01 无凭据、CP 置信层级混搭(FS-F7;GateRSC 审) | **成立,接受** | 不预分预算:ε̂_miss 实测 + Clopper–Pearson 95% 上界,headline 写"≤ε_cal+ε̂_miss^CP95"并声明置信层级;CP95>0.03 则感知治理阻塞 headline |
| 8 | 自适应再收割/超参窥探破坏 exact 秩(FS-F8;FS3C-#1;GateRSC 窥探) | **成立,接受** | §0 的设计域动作 + 全部超参(AGE_MIN、ρ_c、q90、σ 定义、网格、VCAP 表)hash 锁定 config,折B 收割前冻结;窥视后改动 ⇒ 折B 作废重收 |
| 9 | age 断言不实(e>1.5 中位 age=4)+ 语义对齐(FS-F9);age<4 冻结支路非确定性/账本漏项(FS3C-#5;GateRSC 两审) | **成立,接受合成修** | young 轨迹(2≤n<4)**并入同一 sup 分数的第二臂**:冻结位置残差、确定性增长 VCAP_TRUE_c·Δ、只标定截距(§2)。碰撞通道回到 A1,零额外 Bonferroni;部署实现同语义冻结盘。age 门不再宣称"最大杠杆",只作臂分离;harvest age ≡ 签发时 trk.n 写断言测试 |
| 10 | 长航次切段假设几乎必假(FS-F10) | **成立,承认为局限** | benchmark 航次与收割 episode 同 runner 同长(实际 moot);更长部署航次的 ⌈K/L⌉·ε 段界作为显式申报假设留在局限清单 |
| 11 | 闭环分布漂移(管→行为→残差律)三案共病(GateRSC/FS3C 各两审) | **成立,部分可治** | 折B 收割在**全部门控+近终管**在环下跑;标定后强制一轮不动点复核(新 calib 在环重收割,按 d_drone 分桶 QQ 比对,漂移超阈值再迭代一次);PPO/shield-in-training 臂不给 per-flight 保证只报经验碰撞率;剩余部分为申报假设 |
| 12 | shield 臂双倍 pfe.step + 锚点(raw xy vs KF c0)不匹配(全部审查) | **成立,接受** | 前置修复:shield 改喂 KF c0(与 EGO/harvest 同锚)、单步进;修完才允许 shield 读表;网格含 Δ=0 盖裸 q0 |
| 13 | static 消费矩阵三格反保守(load_calib 无 static 键、build_cylinders 不置零 vv、warp 证漂移、eval_ppo 3 元组)(GateRSC/FS3C 审) | **成立,接受** | 同一 PR:mlist/build_cylinders 对 cls=='static' 代码级强制 vv=aa=0;删 replay_core.load_calib 只留 SL 版并加 static/animal 键、缺组 raise;eval_ppo_shield 传 4 元组;warp-static 回归测试 |
| 14 | 联合概率 vs 条件概率口径错位(GateRSC 两审、FS3C-#5) | **成立,接受** | headline 定为联合式 P(碰∧全程认证)≤ε_cal+ε̂_miss,同时实测报 P(认证);条件式只经实测认证率换算给出 |
| 15 | 可部署性在错误 Δ 评估、无验收线(FS 二审-#6) | **成立,接受** | 必报 tube(0.3/0.75/1.05)+每类 keep-out+基准场景走廊几何检查+cert-rate+RTA 率;预注册验收线;不达标降级 ε 档并明说,不许 tube(0.3) 包装 |
| 16 | 定向倍采扭曲混合分布(FS 二审-#7) | **成立,设计消解** | 每场景抽 1 episode ⇒ 折B 混合=池均匀,构造性成立;额外 seed 只进设计域/诊断;benchmark 声明同为池均匀 |
| 17 | 场景混叠(speed_sweep≡wall_and_crosser 等 4 对)、族内近重复(GateRSC 两审) | **成立,接受** | 收割逐行写场景名(废除事后 ep_scn 映射)+ 每 episode 残差 multiset 指纹查重,跨场景相同即 fail;先查混叠根因再收割;street/veh_cal 记族,TEST 与 Tier-2 按族切;池计数按去重后 |
| 18 | GateRSC 乘性 λ̂ 斜率充气 | **成立(对 GateRSC)** | 采加性:充气全走截距,v_eff=A 折 q90 斜率 |
| 19 | FS3C Tier-2 形状见过全部场景 ⇒ 新场景保证反保守(FS3C-#4) | **成立,接受** | Tier-2(全新场景保证)要求形状族与校准族不相交且族数≥45,当前**标注无效、只作诊断**,不进 headline |
| 20 | animal 类缺席无人报警(FS3C 二审) | **成立,接受** | animal 显式入 CLASSES;池内无 animal 行 ⇒ calib 写显式 UNCALIBRATED 条目,loader fail-closed |

---

## 2. Score 精确定义

**行资格(harvest 与部署证书逐字同门,来自同一份 hash 锁定 config):**

- 类 c 的绑定半径:`ρ_c = (V_EGO_MAX + VCAP_TRUE_c)·1.05 + R_EGO + R_OBS_MAX_c + Q0_CAP_c`,常数:V_EGO_MAX=3.0(replay_core 实测默认)、R_EGO=0.35、horizon=τ+δ=1.05;VCAP_TRUE 由 scenario_lib GT 轨迹全扫描定(占位:ped 2.5 / veh 8.0 / static 0 / animal 4.0,扫描超限则 config 更新后才准收割)。给出 ρ_ped≈9.0、ρ_veh≈16.5、ρ_static≈5.0、ρ_animal≈11.0。
- **可达性引理(A3 项,逐字验证)**:碰撞发生时刻前的最后一个认证 tick 上,肇事 mover 必在 ρ_c 内(否则速度上限下 1.05s 内够不着)⇒ 其该 tick 行必为打分合格行。**因此部署证书不需要 dd 门**(部署多查远 mover 只是保守),收割按 dd≤ρ_c 过滤即可——这化解了 FS-F1 "部署无 dd 门" 的同构断裂。
- **成熟臂 M**(n≥4,预测器=部署预测器:ped/veh/animal 用 v_cap 钳位后的 KF-CV;static 用 v=0):行分量 `r_i = (e_i − v_c·Δ_i − b_c) / σ_c`。
- **young 臂 Y**(2≤n<4,dd≤ρ_c):对冻结位置打分 `e0_i(Δ)=‖true(t+Δ) − pos_frozen(t)‖`,行分量 `r_i = (e0_i − VCAP_TRUE_c·Δ_i − b_yc) / σ_yc`(增长项是确定性物理界,只有截距被标定)。
- n<2(未 ready)= 无轨迹 ⇒ 归 A2。

**航次分数**:`S_j = max over 该航次全部合格行(两臂并集) r_i`。一个航次一个标量;max 一次跑遍 tick×mover×Δ×类×臂,零 union bound。

**覆盖事件等价**(σ>0 单调仿射,精确):`S_new ≤ q̂` ⟺ 全航次所有成熟行满足 `e ≤ (b_c+q̂σ_c) + v_c·Δ` **且** 所有 young 行满足 `e0 ≤ (b_yc+q̂σ_yc) + VCAP_TRUE_c·Δ`——正是部署消费的两种几何体。

**出厂**:`q0_c = b_c + q̂σ_c + η_c`,`v_eff_c = v_c`;young 盘 `q0y_c = b_yc + q̂σ_yc + η_yc`,增长率=VCAP_TRUE_c(存 calib 的 young 段,不占 v_eff 槽)。`η_c = (VCAP_TRUE_c + VCAP_PRED_c + v_c)·h/2`,h=0.05(static η≈0)。

## 3. 可交换单位

**单位=场景,经 Dunn 子抽样**。目标总体显式声明:**新航次 =(冻结去重场景池均匀抽取)×(全新感知 seed)**。折B 构造:每场景取指定的第一个全新 seed(seed=H(场景名,0),预注册)的 episode,一场景一分数 ⇒ n 个 iid 从目标混合抽取的分数,与新航次精确可交换 ⇒ Lei et al. 定理原样适用。sup-over-fresh-seeds 变体(每场景对其全部新 seed episode 取 max)随机占优单抽 ⇒ 保守有效,作敏感性报告。episode 级 Tier-1(FS3C 原案)因固定网格设计非可交换(裁决#4),**降级为诊断**。族记账:street_busy/rush 变体、veh_cal 各记一族,池计数、(ε,δ) 层与 TEST 切分按族;Tier-1 保证对"含近重复的声明池均匀"成立——这一点写进局限。

## 4. 切分方案

- **折A(形状/设计域)= 全部历史数据 + 全部历史 seed + 任何未来想加的形状收割**。超参已在其上调过,就地承认并冻结(hash 锁定 config:AGE_MIN=4、ρ_c 表、q90、σ 定义、网格、VCAP 表、SIG_FLOOR=0.05、h)。
- **折B = 全新 seed,每场景 1 个指定 episode**,收割前 seed 列表预注册落盘;含合格行的场景数 n 进秩。
- **TEST = 另一批全新 seed**(与折B 断言交集为空;与 run_scenario 评测 seed 网格断言不重叠)+ 2–3 个 held-out 场景族做移位压力行(只报告不进保证)。
- 秩:`k=⌈(n+1)(1−ε)⌉`;k>n ⇒ **不 crash 不 +inf**:q̂=max(S),落盘实际达到的 ε̂=1/(n+1),flag=UNDER_CALIBRATED,消费侧可拒载。k=n(max 支配)打 WARNING。
- 禁止:按行/按 track/按 ep-id 顺序切;b_bucket `_split_eps(seed=17)` 旧路径删除。

## 5. Slope 估计(折A,只关管宽不关有效性)

每类:6→22 个网格点上取**行级 q90(Δ)** 曲线,**Theil–Sen** 成对斜率中位数(对单点异常鲁棒,防 5.868 路径),clamp [0, V_SLOPE_CAP=3.0];static/young 臂斜率结构性钉死(0 / VCAP_TRUE_c)。截距 `b_c = max_Δ(q90(Δ) − v_cΔ)`(包络支配全网格)。尺度 `σ_c = max(0.05, 每episode类内sup超额的 q90 − q50)`(episode-sup 尺度,FS3C 实测行级尺度把 ped q0 推到 2.14)。选 q90 而非中位数(0.457 浅斜率使 sup 堆积 Δ=0.85 重尾)而非 episode-sup 分位点直拟(v_eff 爆炸)——三案一致,保留。形状点<2 有限值 ⇒ raise 拒绝该类。

## 6. 每类处理

- **共同**:形状每类独立,**分位数全类共享单秩**(类是同一 sup 的臂,不是多次检验,零 Bonferroni)。类条件覆盖(尤其 vehicle)作 Mondrian 诊断表**必报**,不作保证;任何"每类都安全"措辞禁用(5% 失败可集中于含车航次,P(逸出|含车)可达 ~0.29 而边际仍 0.95——FS3C 审 #7,承认)。
- **static(30ep)**:v≡0(static-v0 律实测 Δ-平坦);形状用全部 static 行(资格门不作用于形状);分位数借全体 ⇒ 小样本问题消解;部署三处置零/读键修复(裁决#13)。static-v0 重收割是前置(当前只有 quick 2-seed)。
- **vehicle(89ep)**:诚实 ρ_veh=16.5 下行数回归;形状用设计域(可自由加收 veh_cal×22 seeds 进折A);其重尾(CV 对加速车辆的模型误差)由共享 q̂ 诚实买单,中期升 CA/IMM 是管宽优化不是有效性问题。
- **animal**:入 CLASSES;无数据 ⇒ 显式 UNCALIBRATED 条目 + loader fail-closed。
- **young(全类)**:§2 的 Y 臂;部署对 2≤n<4 轨迹用冻结盘认证。代价诚实申报:veh young 盘 @0.75s ≈ q0y+6m ⇒ 新车辆轨迹头 ~0.9s 事实上不可证,计入 cert-rate。
- **未标定/缺组**:loader 一律 fail-closed(该类 mover 不可认证),响亮日志,绝不静默 fallback (0.15,0.6)。

## 7. ε 记账

事件 {碰撞 ∧ 全部被执行 tick 持有效证书} ⊆ A1 ∪ A2 ∪ A3:

- **A1**(某合格行——成熟管或 young 盘——逸出):`P(A1 | 航次含合格行) ≤ ε_cal`,由场景级单秩一步给出(sup 已跑遍全航次,**无 ε/K_t 每 tick 分摊**——备忘录#8 的分摊被 sup-score 定理上支配,此结论三案一致成立)。
- **A2**(某认证 tick,ρ_c 内 GT mover 无任何被打分表示:未检出/遮挡/n<2/歧义门丢行):**实测量**。收割钩子逐航次记二值,`ε̂_miss` + Clopper–Pearson 95% 上界;>0.03 阻塞 headline(先治遮挡/OCC_MEM 再谈标定)。
- **A3**(确定性桥梁,逐条验证清单,非概率项):cert_clear 音性(deg<2 假证已修)、冻结合取/warp 语义(static vv=0 修复后回归测试)、DT=0.3≤τ、延时≤δ=0.3、shield 单步进+KF 锚、门控 harvest≡deploy 断言测试、网格覆盖消费域+η 垫、load_calib 统一。
- **总账(headline,联合式)**:`P(碰撞 ∧ 全程认证) ≤ ε_cal + ε̂_miss^CP95`,ε_cal=0.05,ε̂_miss 标注为 95% 置信实测上界(置信层级显式声明)。条件式仅经实测 P(认证) 换算。未认证 tick(RTA 硬刹、young-veh 窗、UNCALIBRATED 类)在保证外,uncert_frac/cert-rate/任务完成率与 ε 同版面。
- **训练条件层**:场景级 (ε,δ) 容忍秩 `k_tol=min{k: BinomSF(k−1;n,1−ε)≤δ}` 需 n≥45 族(δ=0.1);池不足前,报 100 次重抽的经验 p10 并标注"容忍层为经验性"。**不做** episode 级 iid 容忍秩(裁决#4)。

## 8. 0.789 最终诊断(定案)

原 510ep 文件已于 07-07 02:25 被覆盖,无法验尸;定案为**多因叠加、逐一被复现、逐一被新设计结构性消灭**:

1. **主嫌:per-Δ multiplicity**——若实现沿用"每 Δ 各取 ε=0.05 分位 → 6 点包络 → 按航次联合评测",每 Δ 边际 0.95、联合仅 ≥0.70;62ep 数据 200 次切分实测该构造每航次覆盖 **0.775**,与 0.789 吻合,rank147 签名一致(=审计 multiplicity 洞的 horizon 翻版)。
2. **并列主嫌:场景聚簇下的单次切分方差**——即使配方边际有效(200 次重切实测均值 0.840@ε=0.2 ≥0.80),有效 n≈34 场景而非 153 episode,sd≈6–10%,0.789≈2.7σ 的寻常坏抽签。Beta(147,7) 下 10σ 的算术证明:**要么实现偏离了描述(机制1/3),要么有效 n 是簇限的(机制2)**——两者都被处理。
3. **切分非随机**(ep-id 顺序块 ⇒ TEST=bench street 族):倒序切实测 0.769,同量级。
4. **污染与放大器**:旧文件 static 行 CV 误打分(TEST ~5.9% episode 系统性失败)+ 中位斜率 0.457 使 sup 堆积远 Δ 重尾。

新设计的消灭对应:单一联合 sup(杀1)、场景单位+全新 seed 折B(杀2、3)、static-v0 重打分(杀4a)、q90 斜率+加性充气(杀4b)。判别协议(新数据上跑一次,结果不影响设计):强制随机切分+严格联合 sup 复算应回 0.95±;打印失败航次的场景×类构成(static/vehicle 扎堆→行集不对称,street 扎堆→切分)。

## 9. 补充收割清单(阻塞排序)

1. **管线前置修复(一个 PR,修完才准收割)**:shield 双倍 pfe.step;shield 锚点改 KF c0;build_cylinders/mlist 对 static 强制 vv=aa=0;统一 load_calib(删 replay_core 版,SL 版加 static/animal 键、缺组 raise);eval_ppo_shield 传 cls 4 元组;部署 age 分臂(n≥4 仿射管 / 2≤n<4 冻结盘 / n<2 不可认证);fit_affine_envelope inf 吞噬路径废弃;每类 v_cap 钳 KF 速度;harvest age≡签发 trk.n 断言测试。
2. **收割器改造**:逐行写场景名(废 ep_scn.json);episode manifest(launched/完成态/行数,零行必落盘);残差 multiset 指纹查重(先查明 speed_sweep≡wall_and_crosser 根因);`_HARV_DELTAS={0.00,0.05,…,1.05}`(隔 tick 抽样补偿行数×3.5);young 行冻结预测器打分列;NIS 列(v1 不用,只收);A2 事件钩子(认证 tick 绑定域内 GT mover 无被打分表示);npy 时间戳文件名+provenance 闸(seed 列表、行数下限、网格=消费域,不满足拒跑)。
3. **冻结设计**:VCAP_TRUE 由 GT 轨迹全扫描定(speed_sweep 行人、fast_overtake 车辆重点核);全部超参写 hash 锁定 config;历史 seed 全体声明为设计域。
4. **折A 补形(自由)**:veh_cal_0..4 ×22 seeds(修 VEH_HEAVY 正则)、scenarios/full 的 fast_canyon/canyon_deadlock/pincer_v2(+23 statics)入池、static-v0 全量重打分。
5. **折B 收割**:去重后全池(目标 ≥34 场景,路线图扩到 ≥45–60 族),每场景 ≥3 个全新 seed(第 1 个进秩,其余诊断+ε_miss);seed=H(场景名,i)。
6. **ε̂_miss 先测**:CP95>0.03 即感知治理优先,headline 冻结。
7. **TEST 收割**:另一批全新 seed(~200 航次,CP 半宽 ~3%)+ held-out 族;预注册验收线(逐场景覆盖 CP 区间、cert-rate≥阈值如 80%、走廊几何检查、RTA 率)。
8. **不动点复核**:新 calib 在环重收割一轮,d_drone 分桶 QQ 比对残差律,漂移超阈值再迭代一次并记 provenance。

## 10. numpy 伪代码(可照写)

```python
# calibrate_fs3cr.py — 场景级航次sup split-conformal, 加性Mondrian充气, 双臂(成熟管+young冻结盘)
import numpy as np, json, hashlib
CFG = json.load(open('conf/calib_gates.json'))        # hash锁定: 收割与部署同读这一份
H_CFG = hashlib.sha256(open('conf/calib_gates.json','rb').read()).hexdigest()
DELTAS  = np.round(np.arange(0.0, 1.051, 0.05), 2)
EPS     = 0.05
AGE_MIN = CFG['age_min']            # 4
RHO     = CFG['rho']                # {'pedestrian':9.0,'vehicle':16.5,'static':5.0,'animal':11.0}
VTRUE   = CFG['vcap_true']; VPRED = CFG['vcap_pred']  # GT扫描后冻结
CLASSES = ['pedestrian','vehicle','static','animal']
SIGF, H = 0.05, 0.05

def load(path, expect_fresh_seeds=None):
    man = json.load(open(path.replace('.npy','.manifest.json')))
    assert man['deltas'] == DELTAS.tolist() and man['gates_hash'] == H_CFG, 'provenance gate'
    if expect_fresh_seeds is not None:
        assert set(man['seeds']) == set(expect_fresh_seeds) and \
               not (set(man['seeds']) & set(man['design_domain_seeds'])), 'fold-B seeds must be fresh'
    R = np.load(path)                                  # d,e,age,cls,ep,dd,scn,arm  (arm: 'M'成熟KF/'Y'冻结)
    # 指纹查重: 跨场景episode残差multiset相同 => fail (speed_sweep≡wall混叠杀手)
    fp = {}
    for e in np.unique(R['ep']):
        k = hashlib.sha256(np.sort(R[R['ep']==e]['e']).tobytes()).hexdigest()
        assert not (k in fp and fp[k][1] != R[R['ep']==e]['scn'][0]), f'aliased episodes {fp.get(k)} / {e}'
        fp[k] = (int(e), str(R[R['ep']==e]['scn'][0]))
    return R, man

def eligible(R):                                       # 与部署证书行集逐字同门
    ok = np.zeros(len(R), bool)
    for c in CLASSES:
        m = (R['cls']==c) & (R['dd']<=RHO[c])
        ok |= m & (R['arm']=='M') & (R['age']>=AGE_MIN)
        ok |= m & (R['arm']=='Y') & (R['age']>=2) & (R['age']<AGE_MIN)
    return R[ok]

# ---- 折A: 设计域(全部历史数据), 每类每臂形状 (b, v, sigma) ----
def shape(A, cls, arm):
    rows = A[(A['cls']==cls) & (A['arm']==arm)]
    if arm=='M' and cls=='static': rows = A[(A['cls']==cls)]   # static形状不设资格门
    q90 = np.array([np.quantile(rows['e'][np.round(rows['d'],2)==d], .90)
                    if (np.round(rows['d'],2)==d).any() else np.nan for d in DELTAS])
    ok = np.isfinite(q90)
    if ok.sum() < 2: raise RuntimeError(f'{cls}/{arm}: shape starved — REFUSE, no silent fallback')
    if arm=='Y':            v = VTRUE[cls]                     # 确定性物理增长, 不拟合
    elif cls=='static':     v = 0.0
    else:                                                       # Theil–Sen on q90 curve
        ts = [(q90[j]-q90[i])/(DELTAS[j]-DELTAS[i]) for i in range(len(DELTAS))
              for j in range(i+1,len(DELTAS)) if ok[i] and ok[j]]
        v = float(np.clip(np.median(ts), 0.0, CFG['v_slope_cap']))
    b = float(np.nanmax(q90 - v*DELTAS))
    sups = []
    for e in np.unique(rows['ep']):
        m = rows['ep']==e
        sups.append(np.max(rows['e'][m] - v*rows['d'][m] - b))
    sig = float(max(SIGF, np.quantile(sups,.9)-np.median(sups))) if len(sups)>=5 else 0.2
    return b, v, sig

A_raw,_ = load('out/conformal/residuals_design_domain.npy')
A = eligible(A_raw)
SH = {}
for c in CLASSES:
    for arm in ('M','Y'):
        try: SH[(c,arm)] = shape(A, c, arm)
        except RuntimeError as ex:
            if c=='animal': SH[(c,arm)] = None            # 显式UNCALIBRATED, 不静默
            else: raise

# ---- 折B: 全新seed, 每场景取指定第1个episode(Dunn), 一场景一分数 ----
B_raw, manB = load('out/conformal/residuals_foldB.npy', expect_fresh_seeds=manifest_seeds)
B = eligible(B_raw)
scores = {}
for s in np.unique(B['scn']):
    eps_here = sorted(np.unique(B['ep'][B['scn']==s]))
    rows = B[B['ep'] == eps_here[0]]                     # 预注册: 每场景第1个fresh seed
    vals = []
    for (c,arm),sh in SH.items():
        if sh is None: continue
        b,v,sig = sh; m = (rows['cls']==c)&(rows['arm']==arm)
        if m.any(): vals.append(np.max((rows['e'][m]-v*rows['d'][m]-b)/sig))
    if vals: scores[str(s)] = float(max(vals))           # 无合格行场景不进秩(Mondrian-on-binding)
S = np.sort(list(scores.values())); n = len(S)
k = int(np.ceil((n+1)*(1-EPS)))
if k <= n: qhat, eps_hat, flag = float(S[k-1]), EPS, 'OK'
else:      qhat, eps_hat, flag = float(S[-1]), 1.0/(n+1), 'UNDER_CALIBRATED'   # 永不+inf/0
assert np.isfinite(qhat), 'non-finite qhat — refuse to write calib'
if k >= n: print(f'WARNING q̂=max(B): worst-scenario dominated (n={n}); expand pool')

# ---- 出厂: 充气全走截距 + Lipschitz垫; young盘入独立段 ----
out = {'groups':{}, 'young':{}, 'provenance':{}}
for c in CLASSES:
    if SH[(c,'M')] is None:
        out['groups'][c] = {'levels':{str(EPS):{'flag':'UNCALIBRATED'}}}; continue
    b,v,sig = SH[(c,'M')]; eta = (VTRUE[c]+VPRED[c]+v)*H/2
    out['groups'][c] = {'levels':{str(EPS):{'q_conformal':round(b+qhat*sig+eta,4),
        'v_eff':round(v,4),'eps_hat':eps_hat,'flag':flag,'n_cal_scenarios':n}}}
    if SH[(c,'Y')]:
        by,vy,sgy = SH[(c,'Y')]; etay = (2*VTRUE[c])*H/2
        out['young'][c] = {'q0':round(by+qhat*sgy+etay,4),'v_growth':VTRUE[c]}
cal = [v0['levels'][str(EPS)] for v0 in out['groups'].values() if 'q_conformal' in v0['levels'][str(EPS)]]
out['groups']['all'] = {'levels':{str(EPS):{'q_conformal':max(x['q_conformal'] for x in cal),
    'v_eff':max(x['v_eff'] for x in cal),'flag':flag}}}          # shield旧路径兼容(上包络)
out['provenance'] = {'scheme':'FS3C-R','unit':'scenario (Dunn 1-ep/scn, fresh seeds)',
    'score':'two-arm flight-sup, additive Mondrian','rank':f'{min(k,n)}/{n}','qstar':qhat,
    'eps':EPS,'gates_hash':H_CFG,'foldB_seeds':manB['seeds'],'deltas':DELTAS.tolist(),
    'shape':{f'{c}/{a}':SH[(c,a)] for (c,a) in SH},'scores_by_scenario':scores}
json.dump(out, open('out/conformal/calib.json','w'), indent=1)
# ---- 诊断(必报,无保证): Mondrian每类条件覆盖(vehicle!), 100次重抽p10(经验容忍层),
#      sup-over-seeds敏感性, TEST逐场景覆盖+Clopper–Pearson, cert-rate/uncert_frac/RTA率,
#      ε̂_miss = mean over TEST flights of A2二值 + CP95上界 ----
```

## 11. calib.json 格式

```json
{
 "groups": {
  "pedestrian": {"levels": {"0.05": {"q_conformal": 1.83, "v_eff": 1.31,
      "eps_hat": 0.05, "flag": "OK", "n_cal_scenarios": 38}}},
  "vehicle":    {"levels": {"0.05": {"q_conformal": 2.41, "v_eff": 1.55, "eps_hat": 0.05, "flag": "OK", "n_cal_scenarios": 38}}},
  "static":     {"levels": {"0.05": {"q_conformal": 0.52, "v_eff": 0.0,  "eps_hat": 0.05, "flag": "OK", "n_cal_scenarios": 38}}},
  "animal":     {"levels": {"0.05": {"flag": "UNCALIBRATED"}}},
  "all":        {"levels": {"0.05": {"q_conformal": 2.41, "v_eff": 1.55, "flag": "OK"}}}
 },
 "young": {
  "pedestrian": {"q0": 0.95, "v_growth": 2.5},
  "vehicle":    {"q0": 1.4,  "v_growth": 8.0},
  "static":     {"q0": 0.5,  "v_growth": 0.0}
 },
 "provenance": {
  "scheme": "FS3C-R", "unit": "scenario (Dunn 1-ep/scn, fresh seeds)",
  "score": "two-arm flight-sup, additive Mondrian normalization",
  "rank": "37/38", "qstar": 2.13, "eps": 0.05,
  "eps_miss_cp95": 0.021, "guarantee": "P(collision AND certified-throughout) <= eps + eps_miss_cp95 (joint form; eps_miss at 95% confidence)",
  "gates_hash": "sha256:...", "gates": {"age_min": 4, "rho": {"pedestrian": 9.0, "vehicle": 16.5, "static": 5.0, "animal": 11.0}},
  "deltas": [0.0, 0.05, "...", 1.05], "lipschitz_pad_included": true,
  "foldB_seeds": ["H(scn,0)..."], "design_domain_seeds": ["1234567", "..."],
  "shape": {"pedestrian/M": [0.61, 1.31, 0.42], "...": "..."},
  "scores_by_scenario": {"climb_trap": 1.02, "...": "..."},
  "diagnostics": {"mondrian_class_coverage": {"vehicle": 0.91, "...": "..."},
                  "resplit_p10": 0.93, "tolerance_layer": "EMPIRICAL (pool<45 families)",
                  "cert_rate": null, "uncert_frac": null}
 }
}
```
所有数值为示意格式;数字来自 62ep quick 的方向性外推,**任何 headline 数字必须等折B 全新收割后重算**。loader 契约:缺组/缺级/flag=UNCALIBRATED ⇒ 该类 fail-closed 不可认证 + 响亮日志;`all` 仅为 shield 旧路径的上包络兼容;两个 v(v_eff / v_growth)语义不同,消费点按 age 分臂选择。

## 12. 剩余假设与局限(诚实清单)

1. **保证范围=声明池均匀混合 × 新 seed**(Tier-1)。不覆盖全新场景类型;Tier-2(新场景保证)在形状-校准族不相交且 ≥45 族前只作诊断。池含近重复族(street 变体),保证对"含重复的声明池"成立——不等于对真实世界场景分布成立。
2. **闭环漂移未根除**:残差律条件在收割时策略+管上;不动点一轮复核是经验缓解不是证明;PPO/shield-in-training 无 per-flight 保证。
3. **ε̂_miss 是 95% 置信实测上界不是分布无关保证**,headline 是"conformal 精确项 + 置信估计项"的混合层级,已显式声明;遮挡重场景(occlusion_reveal 族)可能使其成为主导项。
4. **A3 全部是逐条验证的工程断言**,任何一条(延时、重发周期、几何音性、门同构)破则总账作废;断言测试覆盖但非形式化证明。
5. **可达性引理依赖 VCAP_TRUE 扫描的完备性**:GT 中出现超 v_max 的 mover(或 ego 超 3.0 m/s)则 ρ_c 与 young 增长界一起作废;config 里的每个常数都是保证的载荷。
6. **k≈n(max 支配)在池 ~34–38 时成立**:q̂ 由最坏场景独占,方差大;(ε,δ) 容忍层在池 <45 族前是经验性的。扩池是统计质量的第一杠杆。
7. **类条件覆盖无保证**(只有联合):vehicle 的结构性 CV 模型误差可集中吃掉失败预算;Mondrian 诊断表必报,论文措辞禁用"每类"。
8. **可部署性未证**:诚实 ε=0.05 航次管预计 ped tube(0.3)≈1.5–2.5m、tube(1.05)≈3–4.5m,约为旧(无效)pooled 管的 3–5 倍;young-vehicle 冻结盘使新车辆轨迹头 ~0.9s 不可证。若走廊几何检查/cert-rate 验收线不过,诚实结论是"ε=0.05 每航次保证与现走廊几何不兼容",降级到 ε=0.1 或 CRC(语义弱化,另行论证),不许包装。
9. **更长部署航次**用 ⌈K/L⌉·ε 段界,段可交换是未经支持的申报假设;benchmark 声明限于与收割同长航次。
