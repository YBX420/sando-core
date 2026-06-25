---
name: sando-core-win-ego-2026-06
description: "2026-06-24 目标'任何场景任何条件赢过 native EGO-Planner'的取胜架构 + 踩坑史。最终设计:ours = 把 native EGO 的地面 2D 绕行用连续时间圆柱证书做门(直走/±25°/±50°子目标,第一个被证过的就飞=和 native 一样快),只有所有地面路线都证不过(真墙)才认证飞越,被困才直爬/外推——永不冻。关键:ours 的 EGO 把『预测 mover 按 d_safe 膨胀』喂进去(headless 抬全局 inflation;渲染器单独膨胀 mover 保留静态 0.3),否则地面路线全被证书拒→疯狂爬越。踩过的坑:证书当硬门会把无人机逼进不可行态再冻死(被走进来撞)→ 证书降为门但兜底永远是移动的;噪声下只证预测中心会撞→对预测+当前双重证+q_conformal;evade 阈值太激进会不到达。"
metadata:
  type: project
---

**2026-06-24 目标(用户 `/goal`):任何场景任何条件赢过 native EGO-Planner(更安全 + 同等安全下更快)。** 这是 M3 之后把"机动层"从"能跑"逼到"碾压 EGO"的攻坚。承接 [[sando-core-goaround-m1-2026-06]] M3 + [[sando-core-collision-geometry-2026-06]]。

**最终取胜架构(`metaurban/ego_maneuver.py` run_episode 'ours' + 渲染器 `ego_maneuver_replan`):**
每控制拍:
1. **喂"按 d_safe 膨胀的预测 mover 占据"**(headless:ours 的 EGO grid inflation 抬到 `d_safe+q+slack`;渲染器:单独把预测 mover 渲成半径 `r+d_safe` 的圆柱喂进去、静态保持 0.3,因为街景有树不能全局抬膨胀)。**这条是命门**——不膨胀的话 EGO 地面路线只留 0.3m,证书要 0.8m → 全被拒 → 疯狂爬越(over=68/152,又慢又乱)。
2. **地面选项证书门**:依次试 直走→goal、±25°、±50° 偏置子目标(都 cruise 高度),`ego.replan` 后用连续时间圆柱证书(横向 dual:预测+当前位置,各 `R=r+d_safe+q`;OR 竖直),**第一个被证过的就飞** = 和 native 一样在地面绕行、一样快。
3. **没有地面路线被证过(真墙)→ 认证飞越**(forward 到 z_top,证书过才飞)。
4. **被困(飞越也证不过)→ 直爬升**;**完全卡死(EGO 起点在占据里规划失败)→ 朝最近 mover 反方向推开(evade)**。每级都在动,**永不冻**。

**⚠️ 战绩要诚实分两种 native:**
- **vs 拖慢的 native(EGO inflation=0.8=和我们一样间距,matched-safety)**:ours 全场景更快(crossers 4.9<7.0)——但这是把 native 拖到和我们一样保守再比,**不是公平的"更快"**(2026-06-24 用户当场戳穿)。
- **vs 真 EGO(inflation=0.3,贴 0.14–0.87m 飞,渲染器 --ego 用的就是这个)**:**速度是场景相关的,不是全赢**——`crossers`(横穿车流,预测起作用)ours **更快+更安全**(5.5 vs 6.0,净空 1.2 vs 0.15);`head_on`(单人迎面,预测帮不上)ours **略慢**(4.4 vs 3.9)但净空大一倍(1.7 vs 0.87)。
- **唯一全条件成立的赢:ours 永远不撞 + 永远到达 + 净空 1.4–2× 于 EGO;真 EGO 贴飞、有场景/seed 会撞(seed13 native 撞了)。** raw 速度上 ours 不是"任何条件都更快"——迎面/简单场景它用一点速度换了大安全裕度。
- **渲染器(MetaUrban 街景)目前 ours 比真 EGO 慢**:① 街景偏 head_on/密集静态,预测速度优势小;② 移植 bug(过度爬越、静态避障、d_safe=0.8 太宽);③ 真 EGO 贴 0.3m 飞快但会蹭/撞。要在街景也速度赢,得修移植 + 调 d_safe,且不一定能在所有街景超过贴飞的 EGO。

**踩过的坑(都是根因级,别再踩):**
1. **证书当"会冻死的硬门"= 灾难**:cert-gate 拒掉所有候选时让无人机冻在原地,人径直走进来撞(诊断到连续 8 拍卡在 (10.2,0.56) 被走进来,clr −0.088)。**修法:证书仍当门,但兜底永远是移动的(飞越/爬升/外推),绝不冻。**
2. **只证"预测中心"→ 噪声下撞**(q_conformal=0 老账):预测偏了就吃裕度。**修法:对『预测位置 AND 当前位置』双重横向证(覆盖"人没按预测走")+ q_conformal 余量 0.15。**
3. **inflation 与证书不匹配 → 疯狂爬越**(见命门①)。
4. **evade/stall 阈值太激进 → 不到达**:EGO 一拐弯 gs 低就触发逃离被推走。**修法:只有 EGO 真规划失败才 evade,能规划(哪怕绕)就跟着走。**
5. **EGO 的 C++ 往 stdout 刷屏**:扫描脚本要 `os.dup2` 静音 fd1 才能抓 JSON。
6. **`pkill -f sweep_predict` 会杀自己的 shell**(命令行含该串)→ 用 `[d]` 括号技巧或按 PID;Bash 默认 120s 超时会留 python 孤儿,跑长扫描用 `run_in_background`。

**可调旋钮(env):** `EGO_QCONF`(默认 0.15)、`EGO_INFLX`(地面膨胀 slack,默认 0.1)、`EGO_VEFF`/`EGO_TAU`。工具:`metaurban/sweep_predict.py`(单配置统计)、`tune_config.py`(支配度评分)、scratchpad/qtest.py(ours vs native 表)。相关:[[sando-core-goaround-m1-2026-06]] [[metaurban-render-recipe-2026-06]]。

---

**2026-06-24 根本性结论(用户拍板接受,SLIP 攻坚 + min-time survey 共同确认):**

**目标"渲染后也严格比 native EGO 更快"被物理排除。** 证据链:① 认证证书必须算 `R=r+无人机半径(0.25)+d_safe` → EGO 按 grid 膨胀 0.3 绕行比 R 窄 → 证书拒 → SLIP 在窄走廊反复卡死(5 版不同参数卡同一点 y=−26);② **native 的快来自不要命**(seed13 native 自己 `min_clr−0.01 撞了`、平时贴 0.16–0.41m 飞、不正经算自身体积);③ 你不可能既比一个东西安全、又比它快,如果它的快正来自不安全;④ **min-time survey**(`sando_py/compass_artifact_*.md`)明说:最短时间=逼近执行器饱和+像赛车贴障碍切弯(不安全地飞),min-snap/认证安全**必然牺牲时间**。

**survey 给的真定位(金句):** "**真·时间最优 + conformal/认证安全 在动态障碍上几乎没人做过,是真空白、是强 RA-L 选题**"。建议"用 conformal 界 size 障碍 keep-out,给分布无关碰撞概率保证,同时 cost 仍优化时间"——**= 我们项目的精确定位**。

**可达且新颖的 win(已部分验证):认证不撞 + 平滑 + 时间有竞争力 + 同等安全下比 native 快**(native 也膨胀到 d_safe=0.8 时,ours 的 KF 预测让它更快——headless crossers 每个 vmax 都赢)。

**SLIP(space-time speed-warp,`--slip`,已提交 0e271cd):** 设计 workflow 出的方案,sound(重定时 `obs_vel=v/s,t_hi=s·TAU` 单元测试 0 false-certify,见 `test_retime.py`),用 EGO 执行器余量 warp 速度(s>1 抢身前=survey 说的用执行器余量)。但在密集街景**会诚实地拒绝挤过 native 会撞的窄缝→卡/慢**——这是认证规划器该有的正确行为。后续:加飞越让 SLIP 在窄缝爬过(能到达,但更慢);真 conformal 分位标定 v_eff(补 q_conformal=0)。**对外措辞:认证安全 + 同等安全更快,绝不说"比贴飞 native 严格更快"。**
