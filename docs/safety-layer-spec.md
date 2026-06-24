# 安全层 · 工程 Spec(v2,2026-06-22 全量重写)

> **⚠️ 2026-06-23 方向再聚焦(用户拍板,覆盖本文 §0/§3 的"判官只 HOLD"+"双 planner"framing):**
> ① **产品 = 认证的「最快+最安全绕行」(certified go-around)**,不是判官只 HOLD;HOLD 降为兜底。
> ② **只做 EGO**(实测效果好);MINCO 暂搁置(留作对照/支撑)。
> ③ 机制:**KF(CA 模型)预测障碍未来轨迹** → 把预测的扫掠占据喂 EGO(solver 不动=良性,仍 agnostic)→ EGO 绕开未来 → 证书检预测移动球 `c(t)=c0+v·t+½a·t²` → 过则飞绕行。落地 `metaurban/ego_goaround.py` + `kf_tracker.py`(`ego_safe.py` 二元 HOLD 留作对照基线)。
> ④ **planner 无关 = 支撑性质/通用臂,不是 headline。**
> ⑤ 诚实红线照旧 + M1 新增:`q_conformal=0` 期间是几何证书;且**预测误差会吃 d_safe**(M1 实测净空 0.677<0.8,仍>0 没撞)→ conformal 层(C2)是拉回裕度的关键。
> 详见 `docs/direction-2026-06.md` + `.claude/memory/sando-core-goaround-m1-2026-06.md`。

> v1(2026-06-11)是「冲 RA-L 9/15」的论文蓝图,**已作废**:它把核心押在 conformal 标量化 tube + Isaac 机载标定 + 三感知分支上,而实际 6/19–6/21 做出来的东西**更强也更窄**——一个精确连续时间几何证书 + 双 planner 适配,统计半边还没建。本文按**代码现实**重写,重心是工程,论文是下游。
>
> 旧 v1 的对手地图(原 §7)技术上仍有参考价值,挪到本文 §6;旧 §1-§5 的论文叙事不再权威。

---

## 0. 一句话定位

**认证安全层(EGO-first)**:用 KF 预测障碍未来轨迹→喂预测占据给 EGO 让它绕开未来→对 EGO 输出的**已承诺轨迹**逐动态障碍做一道**精确连续时间碰撞证书**;证过则飞绕行,证不过才 HOLD(兜底)。证书核与规划器解耦——同一套数学能跑 MINCO(五次)和 EGO(三次),这条 planner 无关是**支撑性质/通用臂**(MINCO 现暂搁置)。

**当前 headline = 认证的最快+最安全绕行(KF 预测 + 精确连续时间几何证书,确定性、可证 sound);planner 无关是支撑性质,不是 headline**。
**不是** P(碰)≤ε 的概率/语义风险保证——那需要 conformal 统计半边,现在 `q_conformal=0` 占位,**列为 future work**(§5)。

---

## 1. 证书:精确连续时间 Bernstein 亏量(已实现)

代码:`cpp/include/sando_cpp/bernstein_cert.hpp`(namespace `sando::bcert`,自包含,只依赖 `minjerk_traj.hpp` + Eigen)。

**输入**:一条已承诺轨迹(逐段 Bernstein/Bezier 控制点)+ 一个球障碍,球心是解析多项式 `c(t)=c0+v·t+½a·t²`(**最多二次** = 匀速/匀加速),膨胀半径 `R`。
**输出**:`{certified, margin}`。

**数学**(逐行核对过):
- 相对位置 `w(t)=p(t)−c(t)`(向量多项式),平方距离 `S(t)=‖w‖²` 写成 **Bernstein 多项式**;MINCO 五次 → `S` 是 **10 次**,EGO 三次 → `S` 是 **6 次**(都是 `2n`)。
- 亏量系数 `b_k = R² − S_k`。Bernstein 基**非负且和为 1** ⟹ `D(s) ≤ max_k b_k`,所以 **`max_k b_k ≤ 0` ⟹ `‖p−c‖ ≥ R` 对 `[0,t_hi]` 内全部连续 `t` 成立**(零时间采样)。
- `R = r_obs + r_body + d_safe + q_conformal (+ ε_track)`,调用方传一个标量;**现在 `q_conformal=0`、`ε_track=0`**,所以 `R` 只含确定性几何项。

**soundness(证过永不是浮点谎言)**:全程用**外向取整区间** `Iv{lo,hi}`(`std::nextafter` 各往外一个 ULP,**不是 fesetround/cfenv**);判定用严格上界 `h = rup(R2.hi − S_k.lo)`,`hull = max h`。算出 `hull≤0` ⟹ 真系数 `≤0`。平方用精确 Chu-Vandermonde 比值(`C5[i]·C5[j]/C10[k]`)在区间里做。

**非空(真安全也证得出来)**:对凸包做**自适应中点 de Casteljau 细分**(`seg_worst`,maxdepth=16,`hull≤0` 即早退),把松的凸包界收紧到真多项式。自检里 ≥0.3m 净空即能证。

**动/静**:动障只证到信任窗 `t_hi`(`left_subcurve` de Casteljau 左子曲线裁剪);静障证整条。

**诚实边界(写死)**:① 球心最多二次,更高阶运动不可表示;② 只证**球**(非球硬障碍仍走旧采样判定);③ 是**判官**——只吐 bool+margin,不投影/不裁剪/不重解;④ 1−ε 的统计覆盖本该全靠 `q_conformal`,**但它现在是 0**,所以这是**几何证书,不是概率证书**(见 §5)。

---

## 2. planner 无关核:一套数学,两个规划器(已证等价)

`bernstein_cert.hpp` 有两个入口共享同一核:
- `certify_traj_vs_sphere` —— MINCO 五次专用路径(亏量 10 次)。
- `certify_segments_vs_sphere`(line ~294)—— **任意次通用核**:吃 `BSeg{t0,dur,bern[…]}`,每段 Bernstein 控制点可任意次 `n`;`g_elevate`/`g_square`(n→2n)/`g_subdiv`/`g_left_subcurve`/`g_seg_worst` 是 deg-10 版本的任意次类比。`minco_to_segments`(line ~335)把 MINCO 转成 `BSeg`。

**适配 EGO**(`ego/capi/ego_capi.cpp` 的 `ego_certify`,line ~82-105):取 EGO 已承诺的 cubic 均匀 B-spline 控制点(`get_control_points` 3×N,`getInterval` dt),每段 4 个 B-spline 控制点用固定矩阵 **`Mb/6`** 转成 4 个 Bezier 控制点,组 `BSeg` 喂**同一个** `certify_segments_vs_sphere`。cubic(3 次)→ 亏量 6 次,走的就是 MINCO 五次→10 次那个任意次核。

**已证**:`test_bernstein_cert.cpp` 的 6 个交叉验证例断言通用核 vs deg-5 专用路径在同一条 MINCO 轨迹上**判定(certified)一致**(margin 打印供检视,**未断言相等**);上次 Linux ctest 全过。
**已知 seam**:EGO 那步 B-spline→Bezier 的 `Mb/6` 用的是普通 double,不是下游证书的外向区间——这一步本身不是区间-sound(下游证书 sound)。要不要补成区间待定。

---

## 3. 层的形态:认证绕行(产品,2026-06-23 起) + HOLD(兜底) + 分级刹车(以后)

> **2026-06-23 起产品形态 = 认证绕行(certified go-around)**,见 `metaurban/ego_goaround.py`:KF 预测每 mover → 把预测占据喂 EGO 让它绕开未来 → 证书检预测移动球 → **过则飞绕行**,证不过才 HOLD(兜底)。下面"二元 certify-or-HOLD"(`ego_safe.py`)降为**对照基线**(喂当前位置、无预测),不再是正典。

- **对照基线 = 二元 certify-or-HOLD**:`metaurban/ego_safe.py` 的 `EgoSafe`。每次 replan,EGO 规划 → 逐 mover 算 `R` 调 `ego.certify`(`t_hi=tau_trust=0.75s`)→ 证过执行该 B-spline 一个 DT,**证不过 HOLD**(v=a=0,不改轨迹)。喂的是障碍**当前位置**(无 KF 预测),所以 EGO 看不到未来→只会等,是纯判官,留作 go-around 的对比。
- **下游(EGO 之后再续)= 分级减速刹车**:`render_3d_video.py --ego_safe` 里 `ego_certify_commit` 在分级膨胀半径上算一个速度缩放 `g∈[0,1]`,沿同一条 B-spline 做时间 warp 减速(`EGO_BRAKE_LEVELS`、方向门 `EGO_APPROACH_EPS`、释放限速)。比二元 HOLD 顺滑,但削弱「纯判官」的形式叙事。**两者并存,二元是当前正典,分级是工程抛光层**——以后统一。
- **RTA 三件套只建了判官那半**:最小修正 QP **没建**(用户暂缓);现在是**进程内**的门,不是独立 ROS 节点。兜底是 HOLD/yield + 平滑刹车(OFF)+ `recovery_climb`(>8 次失败后的垂直逃逸,jerky)。

**per-class `d_safe`**(`render_3d_video.py`):行人 0.8 / 车 0.6 / 动物 0.7 / 静态 0.4。**静态障碍故意不进 mover 证书门**(否则会在树冠上永久 HOLD),静态交给 EGO 自己的点云避障。

---

## 4. 接线 / ABI(已实现的真东西)

- **DynTraj 标签集**:`types.hpp:375` `label_set`(Mondrian 码 0=HUMAN/1=VEHICLE_LIKE/2=OTHER,默认空);`derived_class()`(`types.hpp:394-405`)非空集合时 human>animal>vehicle>wall,空集合退回 id 启发式(开区间 `id≥200`→wall,**不是** Python 的 [200,300));硬软在 `planner.hpp:520-524` 落地(非 wall 即硬;wall 速度过阈值重分类为动态硬)。capi `traj_set_label_set`。
- **证书门接线**:`Parameters.minco_deficit_cert`(`types.hpp:674`,**默认 false**)→ `params_set_bool` → `opt.minco_deficit_cert` → `plan_minco.hpp:931-948`:在采样门 `hard_clearance_trusted` 之后,**对已过采样门的球硬障碍再证一道**,不过则 `trajectory_valid=false`(不提交)。OFF 时与现役采样门字节一致。**注意**:① 证书是**附加**门,没替换旧的 per-CP 半空间 ALM 约束(那仍是优化器驱动);② 默认 ON 的接收门仍是采样门(`hard_clearance_trusted`,K≥200,样本间不 sound)。
- **capi 缺口**:已承诺轨迹 `pwp_to_share` 有 C++ getter(`get_pwp()`)但**没暴露到 ABI**;Python 只能 `sando_get_next_goal` 弹 deque 头。其余 getter 已通:`get_seam_bias`/`get_corridor`/`get_obst_class_codes`/`get_obst_ids`/`get_obst_snapshot_time`(后者只在真 replan 时推进 → 消费方能查陈旧)。
- **默认 OFF 的工程开关**(实现了 + 接线了但不开就 inert):`minco_deficit_cert`、`seam_c2_from_state`(+`seam_bias_alpha=0.8`)、`minco_recovery_smooth_brake`、`minco_yaw_c2_smooth`(+`accel_max=6.0`/`lowspeed_lo=0.05`)、`minco_recovery_progress`、ST-corridor/ST-graph/retime-overshoot 等。**默认 ON**:`recovery_enabled`、SFC 走廊(`minco_sfc_radius=0.6`/`w_corridor=50`)。

---

## 5. 没做的(future work,诚实清单)

按重要度:

1. **conformal 统计半边 = 整条没建**。`q_conformal` 全程 0.0,没有学习预测器、没有 Mondrian 分层、没有 tracker 输出标定。**所以 P(碰)≤ε 还没实现**,现在只有确定性几何 margin。要么以后补(预测器→split-CP→分层覆盖审计),要么 headline 就停在「精确几何证书」(**当前选后者**)。
2. **最小修正 QP**(矫正那半):没建。层只判不修。
3. **S7-CRET**(认证 retiming 平滑逃逸):只有 `.claude/memory` + `wave_work` 的设计,**零代码**;现役逃逸还是 jerky 的 `recovery_climb`。
4. **EGO 接进 `eval_batch.py`**:现在只接了 `render_3d_video.py` / `ab_runner.py`;`eval_batch` 还是 SANDO 核、无 layer ON/OFF 轴。
5. **三感知分支**(统计 tube / 遮挡阴影 / depth→occupancy FN body 门):一个都没有,mover 全来自 MetaUrban GT。
6. **committed-traj 的 capi getter**;**ego_capi.so 的构建卫生**(手编 untracked,且比 MINCO 构建旧,可能过期);**summary.csv 与 ab_runner.py 解析器对不上**(csv 有 cert/brake/hold 三列,提交版正则只抓 cert+hold → csv 是更新的脚本生成的);**committed 进仓库的 `cpp/build_v2/` 构建产物**应清掉或加 .gitignore。

---

## 6. 对手地图(技术参考,投稿前重核)

> 仅在以后真要写论文时用;现在不前置阻塞工程。错引会被抓,引用前重读原文。

- **Sundarsingh 2509.25124**:静态、joint(非类条件)保证——动态续作是该组自然下一篇,头号近邻;投稿前确认 venue。
- **Lindemann 2210.10254**(RA-L'23):per-step + δ/T union(非 max-over-horizon),自承保守——我们 trajectory-level / 连续时间是差异点;别与 Cleaveland LCP 续作混引。
- **Dixit 2212.00278**(L4DC'23):ACP=time-average 诚实先例;其 future work 点名 backup/递归可行性。
- **Timans 2403.07263**(ECCV'24):FN explicitly out of scope——若以后补 FN 分支,这是引证基础。
- **Jasour**:distribution-free 但 moment/SOS;我们(若补 conformal)是 distribution-free × 连续时间精确的合取。
- 其余:Kalluraya 2209.06323、COPPOL 2510.18485(FN bound)、2602.12616(泛密度偏移已被占)、2603.08958(Mondrian 进机器人按状态分层)、2511.10586(Lindemann/Pappas,自称 first valid interactive——动态反应域已被占,我们限 non-reactive)。

**差异化锚点(若写论文)**:精确连续时间(无 per-step union、无采样)× planner 无关(MINCO+EGO 同核)× 可证 sound(区间外向取整)。conformal 不是当前 claim。

---

## 7. 开放风险(工程视角)

1. **静态碰撞不在 mover 证书范围**:A/B 里 seed31 静态穿透 −0.793m,层那一圈完全没动作。静态安全目前全靠 EGO 点云避障 + SFC,没证书覆盖——这是评测里碰撞的主来源。要不要给静态也上证书门?
2. **MINCO 核结果不稳**:最新 recovery*/bbox3d* 调参反而退化(到达 12.5–54%、卡死 46–88%)。证书默认 OFF,这些是规划器/recovery 调参问题,与证书正交,但拖累 demo。
3. **证书默认 OFF + 采样门默认 ON**:现在线上跑的接收门样本间不 sound。要不要把 `minco_deficit_cert` 设默认 ON?
4. **`q_conformal=0` 的诚实**:任何对外材料都不能说「概率/语义风险保证」,只能说「确定性几何证书」——除非补了统计半边。
5. **EGO 自身正确性无 golden**:只有手跑的 python 自检,没 ctest 覆盖 reboundReplan 真避障。
