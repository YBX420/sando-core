---
name: sando-core-goaround-m1-2026-06
description: "2026-06-23 里程碑 M1:EGO 上的 KF 预测认证绕行(certified go-around)首次跑通。kf_tracker.py(CA-Kalman)+ ego_goaround.py。行为从基线 ego_safe.py 的'等一等走直线'变成真绕行(横向甩到 y≈4.2)。两发现:q_conformal=0 让预测误差吃掉 d_safe(净空 0.677<0.8);喂整条预测扫掠会短暂冻走廊(7 次 HOLD)。KF 留 Python(微秒级,EGO replan 才是瓶颈),部署全 C++ 闭环时再移植。"
metadata:
  type: project
---

**2026-06-23 M1:认证绕行(certified go-around)首次端到端跑通(EGO,headless 闭环)。** 承接 [[sando-core-direction-2026-06]] 的 6/23 拍板(摒弃 MINCO、只做 EGO、绕行而非 HOLD、KF 预测)。

**做了什么(都是新增,不动 `ego_safe.py` 这个二元-HOLD 对照基线):**
- `metaurban/kf_tracker.py` —— 每 mover 一个 **CA-Kalman**(xy 全滤波、z 常数),吐证书要的 `c(t)=c0+v·t+½a·t²`(`state()`)+ 预测采样器(`predict(ts)`,喂占据用)。是 `conformal/kf_predictor_experiment.py` 的可部署版。a 做了 clamp(KF 长时 a 噪)。
- `metaurban/ego_goaround.py` —— 闭环:噪声检测→喂 KF→**把预测的扫掠足迹渲染成点云喂 EGO**(grid 每次清零重填,见 `ego/src/grid_map.cpp`,所以干净)→EGO 绕开未来→证书检预测移动球(`ego.certify` 带 obs_acc)→**过则飞绕行,不过才 HOLD(兜底)**。

**结果(同 3 横穿人场景,跟基线直接可比):**
- 行为:基线几乎走直线(y 只到 −0.73);新脚本**真绕行**——横向甩到 **y≈4.2** 再绕回。KF 预测生效。
- 指标:replans=29、go-around(认证飞)=22、HOLD=7、到达 @tick28;执行净空 **0.677 m**;都没撞(SAFE,净空>0)。

**两个诚实发现(正好砸在下一步路线上):**
1. **`q_conformal=0` 真咬人**:证书是对 **KF 预测中心**保证 ≥d_safe 的,真人与预测差 ~0.12 m → 执行净空掉到 **0.677<0.8**(仍>0 没撞)。基线净空 1.28 是因为它用**上帝视角精确状态**(无噪、人正好匀速=零预测误差)。**这就是 conformal 层(C2)的存在理由**——给 R 一个覆盖预测残差的裕度就能拉回 ≥0.8。
2. **喂整条预测扫掠会短暂冻走廊**:7 次 HOLD 几乎都在 x≈5、三人预测足迹叠一起时,EGO 反复 `first_optimize_step_success=0` 找不到路,卡~2s 才绕。即 spec §5 的"冻走廊"。修法:只喂近期预测位置 + 之后 deg-2 紧 tube。

**KF 用 C 吗(用户问"要快")= 不用,先 Python。** KF 微秒级,EGO `reboundReplan`(0.5–4ms)才是瓶颈,搬 C 对速度几乎无收益、徒增 ABI 维护。**只有要做全 C++ 机载闭环(循环里零 Python)才移植**——照搬 CA 滤波进 `bernstein_cert.hpp` 旁小 header,几十行,随时可做。

**下一步(用户拍板顺序):A 先做—给 R 加预测裕度占位 q_conformal(~0.15 或 KF 残差分位)拉回净空≥0.8 + 改"只喂近期预测"砍 HOLD(几十行 Python,当天见效);B 后做—证书 deg-2 拨盘(`R²→(r0+v_eff·t)²`+修两 sound bug+圆柱-z),v_eff 接 conformal 分位=真 C2。** 见 [[sando-py-bernstein-deficit-cert]] [[sando-py-ego-port]]。

---

**2026-06-24 跟进(M2):A 两半 + B(cert 核 deg-2 tube)全部落地并验证。**

- **B(证书 deg-2 tube,治本)已进 `bernstein_cert.hpp`:** `certify_segments_vs_sphere` 加 `v_eff/delta`,把常数半径 R 升成时变 tube `ρ(t)=R+v_eff·(t+δ)`。关键 soundness:亏量 `b=ρ²−S`(deg-2n)**必须先组装再 de Casteljau 细分**(对固定 R² 细分才合法,时变 tube 不行)——新加 `g_seg_worst_deficit`。ρ² 全程外向取整区间。`v_eff=0` 字节等价旧常数核(回归测试断言)。capi `ego_certify` + `ego_bridge.certify` 透传 `v_eff/delta`。**ctest 25/25 全过**(新 3 例:v_eff=0 等价 / tube 增长单调收紧并翻转 cert / 短 t_hi risky-fast)。
- **A1(q_conformal 占位)= 复用 B 的 tube:** `EGO_VEFF=0.2` 让 tube 在 [0,TAU] 长 ~0.21m 盖住 KF 残差 → 执行净空 **0.677→0.820 ≥ d_safe 0.8** ✓。(env 可扫 conservative↔risky-fast,无需改码。)
- **A2(砍冻走廊 HOLD)= planner 占据时窗与证书信任窗解耦:** `_predicted_cloud` 新增 `PLAN_HI`(默认 `1.5*DT=0.45`,env `EGO_PLANHI`),只喂**近期**预测占据,别把 [0,TAU] 整条扫掠叠成"假墙"。扫 {0.30..0.75}:**0.45 HOLD 最少=8、净空 0.820、到达最快 @28**(M1 是 7 HOLD 但无紧 tube,不可直比;比本轮 deg-2 默认 9 HOLD/@30 严格更好)。

**第三个诚实发现(砸在"HOLD 是不是好兜底"上,强化绕行论点):** PLAN_HI 越长净空反而**塌**(0.60→0.126,0.75→0.082)。诊断:最差净空那刻是 **HOLD**——无人机冻在横穿人正前方,**人径直走向停着的机**把真净空蹭没(证书全程没放飞,它在尽职)。即 **HOLD 在动态人群里不是安全兜底:证书管 EGO 的提交,管不住有人走进停着的车**。这正是要把 HOLD 换成 certified evasive maneuver(future work 的 S7-CRET / 最小修正 QP)的硬理由,也再证"绕行>HOLD"。

**仍未做(治本路线剩余):** ① 证书的另两个 sound bug(τ 锚 `t_obs+δ`、球改竖直圆柱恢复飞越)还没碰,这轮只做了时变 tube;② `v_eff` 现在是手拍 0.2,真 conformal 分位(C2 统计半边)仍空缺——0.820 是"占位裕度刚好够",不是概率保证。对外仍只说"确定性几何证书"。

---

**2026-06-24 M3:去 HOLD + 圆柱几何 + 飞越/绕行 + 比原生快(端到端跑通)。** 承接 6/24 用户三连拍板:① HOLD 拿掉(冻住不是好解);② 遇人主动机动(能飞越能绕),用预测选最快安全招;③ 人=竖直圆柱(见 [[sando-core-collision-geometry-2026-06]]),证书=横向证 OR 竖直证整窗 disjunction;④ 北极星=**安全(证书硬约束)下越快越好**,目标=比原生 EGO 快("因为我知道人会怎么动,所以敢更快但安全")。丝滑不是独立目标=急刹/抖本身丢速度,所以"最大化朝目标速度"自带丝滑。

**做了什么:** C++ 两道证书(`n_axes=2` 横向 + `certify_segments_above_plane` 竖直,纯复用亏量/细分机器,ctest 28cert/25test 全过,含"3-D 球误判过、2-D 横向证正确拒"的 soundness 对照)。capi `ego_certify_horizontal`/`ego_certify_above`(抽了 `build_segs`)+ bridge 同名方法 + 重编 `ego_capi.so`。`metaurban/scenario.py`(可手编场景:start/goal/humans[pos,vel,r,head])+ `metaurban/ego_maneuver.py`(无 HOLD 候选-锦标赛:直走/左绕/右绕/飞越/紧急爬升偏置子目标→逐个 replan+证书(横 OR 竖,snapshot-before-next-replan)+取片→**提交朝目标速度最大的被证过候选**;紧急爬升=不冻兜底)+ `metaurban/ego_vs_native.py`(A/B 速度扫描)。**先实测验证 EGO 给抬高 z 子目标会真爬升**(max_z 1.62→2.93 翻过墙顶),EGO 一行不改。

**结果(4 场景全 SAFE、零 HOLD、零 cert_miss):** gauntlet 整面墙→飞越 13 拍(max_z 2.85);crossers/head_on→绕;wall_and_crosser 一趟里飞越 8+绕 10。**A/B 同等安全标距(原生 EGO 膨胀设 d_safe 公平对比):crossers 每个 vmax(3/5/8/12)我们都更快(快 0.3–3.3s);head_on vmax=12 原生刹到 1.05m/s 还撞(clr−0.22),我们 6.6s 安全到达。** 诚实点:① 原生裸跑(膨胀 0.3)会贴 0.14m 飞假装快→不公平,故对齐标距;② 高 vmax 我们 mean_v 仍偏保守(v_eff tube+5 粗候选限速),时间已赢但还能再榨快;③ v_eff/conformal 半边仍是占位,对外只说"确定性几何证书"。

**下一步候选:** 把"更快"榨到底(细化候选/置信时降 v_eff/少喂占据);带渲染可视化飞越;A/B 出定量表;真 conformal 分位补 C2。
