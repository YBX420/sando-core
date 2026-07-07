# SANDO 项目问题总清册(论文冲刺用)

**编制:2026-07-07|来源:5 路扫描(memory 快照 / docs·spec·plan / 代码注释 / calib·eval 产物 / git log)+ 主线程今日实况|口径:去重合并,状态以最新证据为准,矛盾处明示不裁决**

---

## 〇、十大最危险速览(对论文/安全主张威胁最大)

| # | ID | 问题 | 为什么致命 |
|---|----|------|-----------|
| 1 | P54 | plan §2B 点名的两个证书 sound bug(τ 锚 t_obs+δ / 亏量先组再细分)六文档无闭环记录 | 若未修,证书核本体不 sound,全部安全主张归零(plan §0.1 红线) |
| 2 | P55 | 两份标定文件互相矛盾(q 差 3 倍甚至反号),PPO eval 加载哪份零记录 | 标定≠部署同源 → exchangeability 断 → P(碰)≤ε 前提不成立 |
| 3 | P37 | episode-sup 不等大小集不可交换(test 覆盖实测掉 0.79),sup-score 修正进行中(#6) | headline 统计保证的根基;修不完覆盖 claim 不能写 |
| 4 | P04 | static 组标定全零 bug(n=12092 下 coverage=0.0,代码短路) | 标定产物含明显假数,审稿人一眼假证 |
| 5 | P05 | 部署 calib.json per-class 四类逐字节同数字(假分组) | 复活了已判死的"per-class 死代码",假证打点 |
| 6 | P06 | vehicle 标定仅 116 条残差、eps=0.05 全零,而 shield 臂碰撞 100% 是 vehicle | 最缺标定的类恰是唯一还在撞的类,per-class 保证是空头支票 |
| 7 | P02 | MetaUrban 臂 conformal 前提未满足(calib 是 replay 世界标的) | RL 臂"证书零违约"只是经验观察,无理论支撑 |
| 8 | P03 | RL/MetaUrban 全线零 held-out split(训练=评估同 20 场景) | 一切外推/泛化 claim 被卡死 |
| 9 | P56 | 同模型同 seed 复跑 shield 行为大幅漂移(projected 2619/308/185),summary 无配置指纹 | 结果不可归因、不可复现,整批 A/B 可信度存疑 |
| 10 | P01 | 论文骨架未动笔,白区仍在但 MIT-ACL ~6月/篇节奏竞速 | 数据已全备,被抢发是最大外部风险 |

候补:P07(训练机 MCE 硬件重启,威胁所有长训练)、P89(episode_sup 负 q/v_eff 爆炸,62 集分辨率崩坏)、P62(组合定理 held-out 覆盖 0.908<0.95)。

---

## 一、未修(按优先级排序)

- **P01** 论文骨架未动笔;"sound 连续时间证书 × distribution-free conformal"白区仍在,MIT-ACL ~6月/篇是最大威胁(RA-L 滚动+ICRA2027 ~9/15);CLAUDE.md 仍记"论文降为下游目标,投否未定"。证据:sando-core-audit-2026-07.md、scenario-workbench CP29-33(两次列为下一步未动)、CLAUDE.md L3-6。下一步:骨架最优先(定理+诚实链+帕累托 / benchmark+失效包络+七臂),顺带请用户确认投稿目标。
- **P02** MetaUrban/RL 臂 conformal 前提未满足:calib 是 replay 世界标的,跨分布失效,"证书零违约"仅是经验观察。证据:ppo-mu-shield-ab-2026-07.md 遗留①;主线程今日实况确认未修。下一步:复用 b_bucket 配方对 MetaUrban 臂在环重标定。
- **P03** RL/MetaUrban 全线零 held-out split:训练与评估同 20 场景。证据:ppo-mu-shield-ab、rl-shield-campaign 头部黑体、metaurban/eval_ppo_mu.py:7;主线程确认未修。下一步:补场景级 held-out(bench 侧已有先例);之前禁止任何外推 claim。
- **P04** static 组标定全零 bug:episode_sup 与全部 age 桶在 n=12092 下 coverage=0.0(q=0 时覆盖理应趋近 1,系代码把 static 排除/短路);同文件 pooled 路径正常(0.9105-0.9439)证明数据在。证据:metaurban/out/conformal/calib_realistic.json groups.static。下一步:定位标定脚本 static 分支,修后重跑 recal(参照 recal_static_v0_console.log,07-07 02:25 刚跑过一版)。
- **P05** 部署 calib.json per-class 完全退化:all/pedestrian/vehicle/animal 四组 q_conformal 逐字节相同(eps=0.05→1.054、v_eff 全 0.996),无 static 组——写了分组结构、灌的全是 pooled 值。证据:out/conformal/calib.json(07-04,compose_theorem v3 用)。下一步:把 calib_realistic 里真实分化的 per-class 分位数(ped 0.4008 / veh 0.3915 / static 0.3245 @eps=0.05 pooled)接进部署加载路径,或删掉假分组。
- **P06** vehicle 标定饥饿:calib_realistic n_test=116(占 0.67%),eps=0.05 与全部 age 桶全零,而六个 eval summary 里 shield 臂碰撞 by_cls 100% 是 vehicle。证据:calib_realistic.json groups.vehicle;注意与 CP24-27"veh_cal_0..4 治愈饥饿(36649 残差/240ep,cov=0.998)"记录直接矛盾——07-07 重跑版可能未含 veh_cal 场景。下一步:对账两处记录;定向补采 vehicle 标定 episode。
- **P07** 训练机 5800X Bank5 MCE 满载硬件重启未修(两次逐位相同 MCE,硬件故障)。证据:training-machine-cpu-mce-reboots.md;缓解(限 4.2GHz/低 n_envs/CheckpointCallback/逐集 jsonl)已做;主线程确认 BIOS 待关 PBO。下一步:关 CO/PBO→JEDEC 内存对照→lm-sensors,再谈多 env 放量。
- **P08** v_cap 覆盖债降速未做(任务簿 #4):唯一"检测前改 ego 行为"、oracle 消融后仍可能有效的杠杆。证据:algo-upgrade 提案#10(default-OFF A/B 未执行)、bio-inspiration-sweep;主线程确认未修。下一步:closing-speed 单调判决扫描拆可约/不可约,再上 v_cap 曲线(H1:reach 损失≤2)。
- **P09** 盲区仿生杠杆判决未做:"撞前锥内有无被 gate 丢弃的 orphan 检测"未验证;4 个廉价判决实验(orphan 计数/shadow-lead oracle/锥外 bearing-tau oracle/closing-speed 扫描)未跑;"21/24 hole_blind"转述无存档 summary;与 oracle 消融(检测无用)存在张力需对账。证据:bio-inspiration-sweep-2026-07.md。下一步:先跑 attribution 存档核实+orphan 计数,再定三条 D435i 内杠杆哪条动工。
- **P10** birth-gate 出生延迟:8m/s×0.3s=2.4m > gate 1.2m → endless newborn ghost tracks。证据:perception.py:53-56 注释自认。下一步:判决实验①通过后做 depth-seed birth+各向异性关联门。
- **P11** 定理 v4 final 的 4 撞逐案尸检+roast 二三波(退化传感+14m/s 干道车)未做。证据:scenario-workbench CP31-33(real_dyn 4/304=1.3%≤ε✓,但后续无完成记录)。下一步:尸检确认 4 撞是否全为 FOV 前提被违反(定理免罪线)。
- **P12** evade 暴露压缩(anti-thrash)与 evade 暴露时间进定理记账未做(结构性无保证本身见 P84)。证据:CP31 尸检④、CP28 里程碑2候选;注意 CRET 系替身 A/B 已全负(FLAGS.md)。下一步:anti-thrash 设计+记账落地。
- **P13** shield 的安全-活性净负交易(ground 模型):timeout 升 2-4 倍(20v5/16v6/12v5,mu_bare 臂 38/400=9.5%)、reached 反降,而碰撞收益不显著(见 P75)。证据:三个 ground ab summary。下一步:分析 timeout 是否集中在 brake/projected 高频段(brake 940 vs 306),联动三值判决/认证慢滑。
- **P14** RL 论文措辞对账未做:"paired scenarios"过强(实为同 seed 同分布非严格配对),且 summary 用非配对 Fisher 浪费配对功效。证据:ppo-mu-shield-ab 遗留③;各 summary.md。下一步:改 McNemar 配对检验重算三次 ground run;措辞降级。
- **P15** RL λ 顶到 clip=5(4.85):饱和 vs 收敛未辨,clip 是否掩盖更高均衡 λ 未验证。证据:ppo-mu-shield-ab 遗留⑤。下一步:抬 clip 复跑或如实记录。
- **P16** 里程碑2余项:陈旧支频率实测接入(maneuver 路径结构性为空是观察非实测)、ε 扫描帕累托三点补全(held-out 放量复验见 P62)。证据:CP28/CP29-30。下一步:排期。
- **P17** ID-swap/multiplicity 极端情形不在 ε 覆盖内。证据:composition-theorem 诚实边界3。下一步:作声明限制;审稿人追问需量化 swap 频率。
- **P18** 保证仅分布内:冻结生成器+传感器定律,换环境必须重标;真实硬件域差距未触碰。证据:composition-theorem 诚实边界2、related-work-matrix、FLAGS.md 管道行。下一步:论文照抄声明;硬件域差距长期项。
- **P19** 保证限 non-reactive scripted movers,ORCA 反应式未做(2511.10586 已占反应域)。证据:related-work-matrix、spec §6。下一步:论文明写限制,不抢反应域。
- **P20** 证书表示上限:球心最多二次、只证球体、动障只证到 t_hi;deg-2 拨盘+竖直圆柱待做(圆柱垂直分支同时是无人机 3D 臂前置)。证据:spec §1 诚实边界①②、plan §2B。下一步:按 plan §2B 排期,与 3D 臂合并。
- **P21** 层只判不修:最小修正 QP 没建,RTA 三件套只有判官半边(用户已暂缓)。证据:spec §1③/§3/§5.2、plan §3。下一步:无需动作(保持 future work)。
- **P22** 二元 HOLD 与分级刹车两份实现并存,分级削弱"纯判官"形式叙事。证据:spec §3、plan §2 搁置段。下一步:随 MINCO 回归统一;论文按二元正典写。
- **P23** S7-CRET 零代码,现役逃逸仍是 jerky recovery_climb;CRET 系轻量替身实测全负。证据:spec §5.3、plan §3、FLAGS.md。下一步:评估直接放弃 S7-CRET 押 DECIDE v2。
- **P24** 构建/接口卫生:get_pwp() 未进 ABI、ego_capi.so untracked 手编可能旧于源码、两个 capi 均不在 CMake、summary.csv 与 ab_runner 解析器对不上、build_v2 产物入库应清;.so 加载优先级会静默覆盖重编。证据:spec §4/§5.6、plan §2.5/2.6、CLAUDE.md L38-41/L63。下一步:capi 进 CMake+新旧校验脚本;对齐解析器;清 build_v2。
- **P25** EGO 自身正确性无 golden:reboundReplan 无 ctest 覆盖。证据:spec §7.5。下一步:补最小 golden 进 ctest。
- **P26** 通用核 vs deg-5 专用路径交叉验证只断言 verdict 一致,margin 未断言相等。证据:spec §2、plan §1。下一步:补 margin 数值断言(容差)进 ctest。
- **P27** C++ 移植明示未移植清单(Gurobi 路径/SFC 凸分解/hover avoidance/完整 yaw 机/ROS 位姿/wall-clock 超时被移除)。证据:cpp/include/sando_cpp/planner.hpp:3,35-47,813,1572,1615-1617、hgp_manager.hpp:32-37、hgp_planner.hpp:25-33。下一步:论文明示范围外裁剪;部署若需 SFC 或超时保护单独立项。
- **P28** MINCO 核结果不稳(到达 12.5-54%、卡死 46-88%),与证书正交但拖累 demo——路线已搁置。证据:spec §7.2、plan 风险表。下一步:无需动作(仅回归 MINCO 时查根因,别误归因证书)。
- **P29** 渲染器遮挡模型 v1:只有 mover 圆柱当遮挡体,静态建筑不遮挡。证据:render_3d_video.py:756、CP8。下一步:感知前端加静态几何遮挡(与 --d435i 真深度路径对齐)。
- **P30** 工作台/生成器长尾:T-pose、ORCA 互让、斑马线用车道末端近似、走廊落草地、drone.waypoints 未接 leg-chaining(harness 直飞 start→goal 并 WARN)、harvest 3 个 WARN 起飞点、静态遮挡边界。证据:scenario-workbench CP3/6/12/15/16、scenario_lib.py:8-11、CHECKPOINTS.md:339。下一步:论文优先级低,择机清;leg-chaining 落地后移除 WARN。
- **P31** metadrone 平台线 P1-P3 未动:ghost body 碰撞真值对账、建筑 AABB 误差标定、IMU/GPS/DroneEnv gym。证据:CP11(楼顶碰撞根因已查明但三阶段未开工)。下一步:排期或明示不做。
- **P32** drone_nav_env 只有运动学质点动力学,quad-tracking 变体未加。证据:drone_nav_env.py:6-8。下一步:PPO 臂进 dyn 对比表前补。
- **P33** 场景合法性明示豁免与近似:手工编队免 spacing 地板、urban_rules 带宽是 dials 不是 law、路面 cone 仅警告。证据:verify_spacing.py:14-17、urban_rules.py:10-17。下一步:论文引用合法性时声明豁免规则。
- **P34** 引用卫生:投稿前必须逐条重读被引原文+重扫 arXiv(雷点:Sundarsingh 2509.25124 venue、Lindemann 2210.10254 与 Cleaveland LCP 混引)。证据:spec §6、plan §3/风险表、related-work-matrix。下一步:投稿前执行。
- **P35** 算法侦察残项无执行记录:IMM 前置 20 行 oracle 门、_cyl_cloud 广播+GT 插值预计算(#13);证书-规划半径差 0.575m 归入 P58 裁决;CRET-hold(#9)目标已转入慢滑。证据:algo-upgrade #6/#13。下一步:低优先排期。
- **P36** 仓库唯一真相=分支 feat/bernstein-gate(HEAD 46f5ac9);master 无证书/EGO;bcert-wire 及旧 worktree 一律忽略。证据:CLAUDE.md L26-28。下一步:无需动作(常设约束,写论文引数据时守住)。

## 二、半修

- **P37** conformal 可交换单元/sup-score 链(审计洞#4):pooled per-instance(n≈150万虚高)已换 episode-sup→scenario 单元(管 2.23m,覆盖 0.987✓),但 07-07 确认不等大小集不可交换(test 覆盖实测掉 0.79);episode_sup 报告还静默吞 inf(static 仅 30 集有限样本下限);mover per-class 真数值未并入。证据:audit、CP29-30、ppo-mu-shield-ab 07-07。下一步:任务簿 #6 进行中(设计工作流并行跑)——完成不等大小集修正+inf 修+per-class 真数值;顺带清点 9 项任务簿(仅 #1、#8 记录完成,其余 7 项状态未见)。
- **P38** pedestrian 覆盖未达标:plain pooled 三档全 miss(0.7564/0.8516/0.9176,n=4999),normalized 路径三档达标(0.875/0.9346/0.9632)、episode_sup 达标但见 P89;审计"0.944<0.95"的复核(任务簿 #9)未做。证据:calib_realistic.json groups.pedestrian、audit。下一步:部署默认切 normalized 或明确弃用 plain pooled 并只承诺 normalized 保证;做 #9 复核。
- **P39** native 29 实为 31 舍入 bug/"2x 行人净空"无工件:docs 时效增编已把旧数字带条件作废(对账层面),但 .2f 舍入代码本身与工件补录无记录。证据:audit、CP24-27。下一步:确认代码已修;2x claim 补工件或删除。
- **P40** 复现性基建:已补 PROTOCOL.md 评测协议、FLAGS.md 注册表(删 6 开关)、calib provenance、sando.sh .so 守护、isaac 归档;仍缺 CI、python pytest、headline 配置 manifest。证据:audit、CP24-27、maneuver-smoothing、CHECKPOINTS.md:339。下一步:CI+manifest 落地。
- **P41** A* 内存泄漏根修:~AStar() 3 个 delete[] 已补+重编+LOAD_OK,但 MB/ep 回归未重跑且 8.08→0.02 的 scratchpad 证据已随 session 消失(任务簿 #9 复核未做)。证据:algo-upgrade;主线程确认。下一步:重跑 MB/ep 回归,确认后退役 chunked workaround。
- **P42** proximity-triggered recert:①KF 变步长化②感知 dt 透传已验(regress_frozen_ours 12/12 字节一致,3d2a549+28b6257);③贴近触发子拍(0.3→3×0.1s)④固定 vs 自适应 A/B 未做;理论关(变节奏组合记账/Zeno 最小间隔/按节奏分桶重标)未过。证据:proximity-triggered-recert.md。下一步:做③④;红线=感知+KF+cert 一起提速,只快重认证=假保证。
- **P43** 机动平滑链:sticky v2(c85758b)+统一锦标赛 v2(8e6b463)已入库(evade-37%/switches-53%);待做 a)真持久化治 acchurn(本次仅-1%)b)hold→认证慢滑 c)C2 桥接样条 d)渲染器 SMOOTH 副本 e)52 场景放量;另 spchurn 在 v2 下略升未调(主线程实况)。证据:maneuver-smoothing-2026-07.md。下一步:按 a-e;红线=证书后不许平滑。
- **P44** SHIELD_PERCLASS 三项全胜(超时-40%/到达+2.3pp/碰撞-0.25pp)已接线但默认 OFF 未转正。证据:FLAGS.md、commit 2ec7278/b5b3bb6。下一步:转正默认+用 28b6257 字节回归 harness 验证+复跑受影响基线。
- **P45** OCC_MEM:v2(OCC_CAP+TTL)已成真部署默认(在线建图),但 naive 版教训(mover 轨迹 phantom walls、seed5 围死)的正解 decay+mover-track removal 仍是 future work;任务簿 #5 记未修(主线程实况)。证据:render_3d_video.py:1120-1127、CP26。下一步:完成 decay+mover-track removal。
- **P46** 感知前端自声明局限:分类误差直通、尺寸误差未建模、greedy NN 允许 ID swap 与轨迹分裂(by design);头注"PERCEPT=gt still default"声明已过期(replay_core 默认已 realistic,FROZEN 07-03)。证据:perception.py:18-25、replay_core.py:260-262。下一步:分类/尺寸误差 dial 立项;更新头注。
- **P47** benchmark 感知对等:协议已声明 gt-vs-gt 公平边界、native 已可消费同款 realistic 前端,但感知对等臂未纳入正式表(P4 剩余)。证据:bench_run.py:4-9、replay_core.py:261-262、CHECKPOINTS.md:339。下一步:对等臂入正式表。
- **P48** props_alley 残余:surface-honest 可见性+evade 静态 clear 已修(real_dyn 7→3),但残余 3/8 撞集中单场景,聚类 caveat(曾 5/7)。证据:CHECKPOINTS.md:288-291/344、perception.py:141-146。下一步:场景层 bootstrap 置信区间+残余深挖。
- **P49** spec §5.5"三感知分支一个都没有"已过期(realistic 前端已建成部署默认),但 depth→occupancy FN body 门在全部文档内仍无着落。证据:spec §5.5 vs rl-shield-campaign 头部、composition-theorem 设定段。下一步:更新 spec;FN body 门单独立项或明写不做。
- **P50** 文档滞后群:CLAUDE.md L22"q_conformal 全程 0.0 占位"已被组合定理真 (q₀,v_eff) 取代未同步;CLAUDE.md 方向段未同步 planner-agnostic 升回 headline;spec §5.1/plan §3 陈旧表述;headline 文档对账未完成;render_3d_video.py:1208-1210 过期注释("per-class reverted"与下方已接线代码矛盾)。证据:各文件+composition-theorem.md+CHECKPOINTS.md:339。下一步:统一做一次文档对账 pass;对外措辞升级前提=held-out 覆盖达标(P62)。
- **P51** M1 实测执行净空 0.677<d_safe 0.8(KF 误差吃裕度,仍>0 没撞):真 (q₀,v_eff) 已有,但 M1 场景净空复测未见记录。证据:spec 再聚焦⑤、plan §2A1、CLAUDE.md L22。下一步:用 compose_theorem.json 的 (q₀,v_eff) 复测净空是否回到 ≥0.8。
- **P52** flag 治理:FLAGS.md 全量登记 A/B 判决+孤儿 _committed_warp 已删,但判负 flag 的删除未执行;多个审计修复(SMOOTH/RADIUS_CONSIST/CRET_GLIDE)默认 OFF 保冻结——当前 headline 数字基于冻结行为。证据:commit 8e6b463/8f63e13、replay_core.py:35-45。下一步:v2 转正后执行删除;转正裁决见 P57/P58。
- **P53** 上游 EGO-Planner issue:UB 已根修(见 P108),docs/ISSUE_UPSTREAM.md 草稿已写但一直未发出。证据:CP2/CP24-27、CHECKPOINTS.md:336。下一步:拍板发或不发。

## 三、悬案(需复核)

- **P54** plan §2B 点名的两个证书 sound bug(τ 锚 t_obs+δ / 亏量先组再细分)六份文档无任何"已修"宣告——soundness 红线级。证据:safety-layer-plan.md §2B(06-23)→docs/direction-2026-06.md §3。下一步:对照 direction-2026-06.md 与当前 bernstein_cert.hpp 确认闭环;未修立即修。
- **P55** 两份标定互相矛盾且部署加载来源不明:calib.json q@0.05=1.054 vs calib_realistic pooled 0.3811/episode_sup -0.0851(3 倍差/反号);两份 provenance 管线不同,PPO eval 的 shield 加载哪份、感知配置是否匹配,summary 零记录——"标定≠部署"的翻版。证据:两文件+各 eval summary。下一步:eval summary 头部打印 calib 路径+hash+percept 指纹;确认同源后重跑一轮。
- **P56** 同模型同 seed=7777 三次复跑 shield 内部行为大幅漂移(projected 2619/308/185、brake 940/1012/306、uncert_frac 0.0223/0.0254/0.0078),summary 无任何配置项记录:要么有意迭代未记录,要么管线有未固定随机源。证据:ab_20260706_211916 / ab_20260707_001457 / ab_20260707_023948 各第 9 行。下一步:summary 增记 shield 全部超参+git hash;同配置连跑两次验证可复现。
- **P57** headless DECIDE v1→v2 裁决未定(默认仍 v1,主线程实况):北极星指标全胜,用时+40% 半为假象(P87);SMOOTH 三开关(被 v2 上位替代)的删除同悬于此。证据:FLAGS.md、maneuver-smoothing。下一步:52 场景全台架+#6 真 q 后一并裁决 DECIDE/SMOOTH/RADIUS_CONSIST。
- **P58** RADIUS_CONSIST:full 封廊被否决(evade 64→87),dsafe 档单独赢(evade-14%)但与 SMOOTH 冲突,默认 OFF 悬而未决。证据:FLAGS.md、commit 2ed411b。下一步:#6 落地后 full-bench 裁决。
- **P59** 过夜 ~52 场景 BENCH_FULL_SCALE.md 产物未读/未定(速度扫描 v2 已读入 CP28,全量表无读取结论)。证据:CP24-27(pid 368558)。下一步:读表出裁决(联动 P57)。
- **P60** PPO v1 shield 臂 9/304 碰撞未归因(uncertain 兜底 vs cert margin 两假设未判;用户曾说不深挖;该 env 无归因记录)。证据:ppo-e2e-composition。下一步:若做 conformal 洞对账则顺带归因,否则明示不做。
- **P61** PPO v2 3M(DroneNavEnv,pid 10387,07-05 启动)死活无记录,只在跑完才 save 一次,且机器有 MCE 前科(P07)。证据:ppo-e2e-composition、training-machine md。下一步:查 out/ppo_logs/ppo_v2_20260705_154524/ 与 checkpoint。
- **P62** 组合定理 held-out 覆盖 0.908<0.95(98 eps,≈2σ 低;可交换性近似+有限样本),放量重验"在跑"但结果未见。证据:composition-theorem 诚实边界1、CP29-30。下一步:收放量结果;veh-heavy 场景加采;达标前对外措辞不升级(联动 P50)。
- **P63** headless δ_track 缺失疑云(任务簿 #9 复核未做):0.473 修复证据集中在渲染器路径(render_3d_video.py:1023-1031),headless 是否消费同值未核。证据:主线程实况。下一步:核 headless 路径,缺则接。
- **P64** SLIP 臂对账残留:审计点的"SLIP arm R 不含 q_conformal/δ_track、tube 斜率手设 0.25"未见对账记录(窗口错位本体已修,见 P96)。证据:audit。下一步:对账或修正。
- **P65** deg-10 孪生 certify_moving_sphere(~L141)同源 -inf 理论隐患(恒 deg-10 当前不触发)——低优先。证据:algo-upgrade。下一步:加同款守护。
- **P66** EGO B-spline→Bezier 的 Mb/6 变换用普通 double,非区间-sound(seam)。证据:spec §2。下一步:拍板补不补区间;论文 soundness claim 限定到证书核本体。
- **P67** minco_deficit_cert 默认 OFF,线上接收门=采样门(K≥200,样本间不 sound),证书只是附加门。证据:spec §4/§7.3、plan §3。下一步:MINCO 已搁置,随回归一并裁决。
- **P68** 静态障碍无证书门:spec 时代静态穿透(seed31 −0.793m)是 A/B 碰撞主源;此后 RL 线 shield 臂静物清零、replay 线 OCC_MEM 默认 ON,是否仍复现未复核;ego_safe per-frame FOV 门自认"既过保守又欠保护",正解=基于 OCC_MEM v2 记忆的静态硬门。证据:spec §3/§7.1、plan §2.4、render_3d_video.py:1742-1746、rl-shield-campaign。下一步:当前管线复现检查→决定静态硬门。
- **P69** bench_chaos S2 无退路场景:yield 限 |y|<1.3 自声明可能不足。证据:cpp/test/bench_chaos.cpp:153-154。下一步:跑 S2 统计,决定是否放宽 yield 通道。
- **P70** EGO 未接 eval_batch(无 layer ON/OFF 定量表)——疑已被 bench 六臂体系取代但 spec 未清理。证据:spec §5.4、plan §2.3。下一步:确认 bench_run 覆盖则 spec 划掉。
- **P71** M1 的 7 次 HOLD(整条预测扫掠冻死正前方走廊,first_optimize_step=0)是否已被 M3 机动线实质解决,无闭环记录。证据:plan §2A2(06-23)。下一步:复核后关闭或立项"只喂近期预测位置"。

## 四、负结果(已关死,留判决)

- **P72** age-conditional 单独 headline 死亡:诚实感知下 fresh-hole 消失(age2-3 覆盖 0.998),归一化 baseline 也救不了新洞 age13+(0.909/0.924,长命航迹到转角 CV 预测不了转弯);2603.08958 已占 per-state 分层 genus;ε=0.05 行人 episode-sup 管 q=0.277+3.56Δ 运营不可用(此差距=组合定理数据背书)。证据:audit、CP16、b_bucket_recalibrate.py:18-22、CHECKPOINTS.md:206-209、related-work-matrix。下一步:分层只作定理内细化;turn/maneuver-conditional 新故事未开工(仅修了感知锥)。
- **P73** 93 vs 69 速度匹配后 p≈0.68 不显著——旧 headline 已被 CP28 去混杂速度扫描 v2 取代(native 高速端下降是真曝险效应,ours 全速段 0/1140),旧数字随 docs 时效增编作废。无需动作。
- **P74** YOLO 感知接入判负:微调模型与渲染域不匹配(高清帧 5/28 检出;延时 6.7-8.7ms 本身无虞)。证据:CP24-27、MEMORY.md。下一步:需标注 recall 集或重微调才可重启,否则不接。
- **P75** 强策略(ppo_planner_ground)上 shield 收益三连不显著:6.75/9.25(p=0.24)、7.50/8.25(p=0.79)、6.50/8.75(p=0.29),n=400/臂;对比弱策略 mu(p=0.0007)、adapt(p=0.0000)——策略越强,边际收益缩到 0.75-2.5pp。证据:三个 ab summary、commit 4be6ebe(cert-integrity 零违例是硬结果)。下一步:McNemar 重算(见 P14)或放量 n≥1200,或 headline 如实写"shield 收益随策略变强缩水"+主打 cert-integrity 零违例。
- **P76** adaptive-λ 用裸跑安全换证书覆盖:shield ON 6.50%/覆盖最优,shield OFF 恶化到 19.25%(三臂最差)——策略学会依赖门,脱门即险。证据:rl-shield-campaign 3×2 矩阵。下一步:门常开产品可用 adaptive-λ,否则 fixed-λ;论文作 tradeoff 注脚。
- **P77** CCF 系实证退役(冻 2/5 seed;EGO_CCF/EGO_CCF_WARP/EGO_MAN_RELEASE 07-07 已删,被 v2 吸收)。证据:FLAGS.md。无需动作。
- **P78** CRET-hold 死旋钮:机制成立但可转化 hold≈0(holds 是 static-gated,retime 只松 mover 证),EGO_CRET_HOLD/SMIN 已删。证据:FLAGS.md、commit af8e4d7。下一步:除非 mover-hold 主导场景重验,否则关死。
- **P79** CRET_GLIDE 判负:evade -22% 但用时 +22%、worst_clr 1.24→0.55(headless 同判,只转化 mover 限速冻结的诚实总体)。证据:FLAGS.md、commit 07015b5、replay_core.py:478-482。下一步:留论文 ablation 一行,v2 转正后删代码。
- **P80** SHIELD_EPS=0.1 是坏交易(超时仅-4、碰撞+0.75pp),维持 0.05;放宽路线改走 per-class tube。证据:FLAGS.md、commit b5b3bb6/08e6e95。无需动作。
- **P81** 渲染器回退 keep climb:headless 式 evade 作为静态安全防冻结手段在渲染器路径被否。证据:commit 6bdfc7d。无需动作(判决留档)。
- **P82** MINCO 路线级搁置(用户拍板:只做 EGO,MINCO 代码留对照不投入)。证据:CLAUDE.md L3/L18。无需动作。

## 五、物理边界(不修,写包络)

- **P83** 盲区快车碰撞地板 ~6.5-7.5%:oracle 360°/16m/GT 轨迹下碰撞仍 27/400=6.75%(与部署 shield 臂一字不差)、27/27 全 brake、大量侧后方 bearing(±100~180°)即被追尾——1.9s 预警×3m/s 平台 vs 8.3m/s 车扫掠走廊无逃逸解,地板是物理的非感知的;45° 锥 blind-side 份额只能靠传感器配置收敛;360° 安全环路线否决(收益=0)。证据:rl-shield-campaign 归因段、oracle_20260706_153925/summary.md、commit 999c6a8/4be6ebe/29d2b9b、eval_shield_ab.py:8-9。下一步:论文按失效包络写(条件:16m 视界/3m/s 平台/2D)+2D vs 3D 可爬升注脚;可选 SENSE_R=30 分离实验按诚实边界原则暂不做;归因分母修正见 P90。
- **P84** evade 支结构性无保证(能证的都不 evade),占比与碰撞集中同一批退化场景。证据:composition-theorem 三支记账③/诚实边界4、related-work-matrix。下一步:按 RTA 暴露如实报;降占比走 anti-thrash(P12),不给 evade 发保证。
- **P85** MINCO 端到端 golden 非位精确(LBFGSpp≠scipy 非凸 ALM):误差诚实报告不 gate,确定性 helper 仍逐项 FAIL 保护。证据:cpp/golden/gen_planner_golden.py:18-24、test_plan_minco.cpp:8-14、test_planner.cpp:15-18。无需动作(接受为优化器非确定性边界)。
- **P86** 100-seed 头对头 ours 6 撞全为 both-collide/impossible(6/97 vs native 30/100,head-to-head 24 胜 0 负)。证据:commit 4b63af1。无需动作(写进包络叙事)。

## 六、度量假象(改度量不改算法)

- **P87** 统一锦标赛 v2"用时+40%"约一半是 stall-放弃计时假象(window-progress 打分自述短视);北极星已改 evade 数+近失距离优先。证据:maneuver-smoothing、commit c7f7ea6。下一步:52 场景放量把 stall-放弃与真实用时分开报,再评剩余 +20% 是否真代价(联动 P57)。
- **P88** RL 绝对碰撞率是 env 定义产物(车=r2.87 圆、2D 点质量),只可臂间比较。证据:rl-shield-campaign 头部、ppo-mu-shield-ab 遗留④。下一步:对外只报臂间对比+检验 p 值,不报绝对率为真实世界指标。
- **P89** episode_sup 在 eps≤0.1 量纲崩坏:q_conformal 为负(-0.017/-0.0851)反推 v_eff 10-13 爆炸、覆盖 0.999 过覆盖;normalized 版 q 冲 8.93-10.67 且多处 v_eff=0——n_episodes=62 时分位分辨率 1/63≈0.016 不足,系数值假象。证据:calib_realistic.json groups.all/pedestrian/vehicle+provenance。下一步:episode_sup 限定 eps≥0.2 报告或标定集加到 ≥200 集;v_eff 计算加负 q 断言/夹逼。
- **P90** hole_blind 归因标签失真:oracle 360° 无盲区仍同率、9 条 n_ready=0/n_live=0、多为被追尾——现行归因器把"被撞"误记为盲区洞。证据:oracle_20260706_153925 明细+各 ab summary 归因行(24/27、24/33、21/26)。下一步:拆"主动撞入 vs 被追尾/侧撞"两个分母分开报告,后者从 shield 可归责碰撞剔除。
- **P91** 烟测与正式训练共用输出文件名→看门狗抢跑评了婴儿策略(85% 超时假象);教训已记录,护栏是否固化到脚本无记录。证据:metaurban-ppo-live-env 07-06 追记。下一步:确认"烟测独立输出名/看门狗看进程退出"已进脚本。

## 七、已修(一行带证据)

- **P92** 标定KF≠部署KF(replay 线):B桶部署在环重标(b_bucket_recalibrate.py→calib_realistic,29419残差/290ep)→CP28 晋升 calib.json(q₀=0.632,v_eff=0.967),provenance 明写 must match deployment,三宗罪自述闭环——仅覆盖 replay,MetaUrban 见 P02。
- **P93** vehicle CV≠CA 反转已修:内核本就 CA(_AxisCAKalman)、CV 预测系故意(行人误差减半);veh_cal_0..4 治愈饥饿(eps.05 q=0.345 cov=0.998,36649残差/240ep)——与 P06 当前文件矛盾待对账。
- **P94** δ_track 0.45→0.473(eps0.01 per-flight 合法分位,删静默钳位+UNACHIEVABLE 输出;render_3d_video.py:1023-1031)——headless 疑云见 P63。
- **P95** 证书 deg<2 false-certify:g_elevate 升阶+worst_hi==-inf 守护,3 个 deg-1 ctest、31 用例全过、.so 重编(algo-upgrade)——孪生隐患见 P65。
- **P96** SLIP 重证窗口错位:mover 回溯 c0−v·t_ego/s+证 [0,t_ego+s·TAU] 带验证(CP24-27②)——臂对账残留见 P64。
- **P97** per-class tube 死代码接线:按 d_safe 查每类 (q,v_eff)、缺类回退标量(render_3d_video.py:1212-1215);RL 侧 SHIELD_PERCLASS 也已接(2ec7278)——转正见 P44、calib 内容退化见 P05、1210 行过期注释待清(P50)。
- **P98** 决策分叉+无认证 climb soundness 洞(seed-56):climb 认证门入 safety_layer.maneuver_decide(safety_layer.py:120-143,"NEVER fly uncertified")、统一锦标赛 v2 一份实现两端共用、渲染器默认 v2/v1 字节级保留(8e6b463)、FOVCAP 等被吸收删除——headless 默认裁决见 P57。
- **P99** EGO warm-start 恒 1 死代码:诚实化+capi 空 local_data_ UB 护栏+EGO_WARMSTART=1 opt-in+ego_capi.so 重编(ego_bridge.py:69-73、ego_capi.cpp:75-77)——转正需 A/B。
- **P100** mover 感知 GT 透视:realistic 前端(锥+遮挡+漏检+NN 关联无 GT 身份)建成并两处默认翻 realistic,render_3d_video 接入(CP7/8/16)。
- **P101** EGO_STATIC_MAP GT 泄漏:真部署标准=无先验地图在线建图,OCC_MEM 默认 ON,全 bench 重跑;props_alley 第一版误修已撤、正解 5→1 撞(CP18-26)。
- **P102** 协方差归一化 conformal baseline(审稿人必问项)已做:救不了 age13+ 洞(0.924)——弹药在手,兼作 age 故事死亡第二证据(B桶实测)。
- **P103** isaac/ira 9.5G 退役缓存已归档(maneuver-smoothing 07-07)。
- **P104** naive 全局 sup 管 3.18m 冻死无人机→cadence-aware W=0.4 收到 q₀=0.632/管 1.02m(3 倍紧缩);Δ>W 段过保守但 sound(composition-theorem 关键杠杆节)。
- **P105** 硬约束只 enforce 到 tau_trust=诚实门:执行 commit(~replan_dt)恒<tau_trust,飞过的路径始终有证(plan_minco.hpp:480-487)。
- **P106** retime 绕过软墙碰撞洞:body-penetration floor 先判(plan_minco.hpp:953-961)。
- **P107** C++ 障碍 id 开区间 id≥200 故意分叉(治 stall):文档+测试齐备(types.hpp:390-393、docs/dyntraj-labelset-abi.md、test_dyntraj_labelset.cpp:54)——残留:derived_class 空集合回退路径与 Python [200,300) 语义仍可能错配(spec §4),对拍注意,小项=统一语义或保证 label_set 恒非空。
- **P108** 上游 EGO-Planner bspline 存量 UB 根修:fresh_intersection+原子绑定+空邻居守护,ASan 全程零报告(CHECKPOINTS.md:52-57)——issue 发出见 P53。
- **P109** MetaUrban 时基质疑双证排除:0.02s×5=0.1s/step(实测 0.0976),3 substeps=0.293s≈DT 0.3(比值 0.98)(rl-shield-campaign 附)。
- **P110** 旧口径"撞了还算 reached"已改:碰撞即停止 episode(commit 97264e6),100-seed 结果口径由此修正。
- **P111** statics 蹭 mover tube 的 keep-out 墙:static 类入部署在环 per-class 标定,static=(0.15,0) stationarity(d2abe0d、2ec7278)。
- **P112** planner 占据视野 plan_hi 故意短于证书 TAU(防 phantom WALL 冻走廊):近窗占据+deg-2 tube+频繁重规划补齐其余窗口(ego_goaround.py:33-38)。
- **P113** spec/plan v1 作废头注已加:旧论文叙事不再权威,读时与 07 月文档(composition/rl-shield/FLAGS)对表。
- **P114** 审计洞#5"0/120 零 held-out":held-out 诚实版重挣 ours 1/120 vs native 17/120,旧 0/120(GT 感知产物)作废(CP24-27)——覆盖放量复验见 P62。

---

**对账提示(编制者注,非新问题)**:① P06 与 P93 直接矛盾(vehicle 饥饿"已治愈" vs 当前文件 n=116),须先对账再引用任一数字;② P83(oracle 证明检测无用)与 P09(仿生杠杆赌 orphan 检测有用)存在张力,判决实验就是为此设计;③ 主线程 9 项任务簿仅 #1/#8 确认完成,#4(P08)、#5(P45)、#6(P37)、#9(P38/P41/P63)如上归位,其余项状态未见,清点并入 P37。
