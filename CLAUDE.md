# CLAUDE.md — sando-core(给 Claude Code 自动读的项目入口)

> **⚠️ 2026-07-14 现状速览(覆盖下面 6-23/6-22 的 framing;详情看 `.claude/memory/MEMORY.md` 索引):**
> - **产品方向不变**(认证的最快+最安全绕行,只做 EGO),但已远超 M1:现役 = **M3 无 HOLD 机动锦标赛 + v6 胶囊 conformal keep-out(345 集四臂最优)+ EGO_TDYN 时间维 + CPA 相遇点提前量环**。conformal 半边**早已补齐**(06-24 起真轨迹 split-conformal 标定,P(碰)≤ε 有实测依据)——下面 6-22 正文里"conformal 没建"的诚实点**已过时作废**。
> - **头号指标 = 干净抵达**(到达 ∧ 零碰撞 ∧ 零紧急干预)。**证明纪律:每个算法改进主张必须配 3D 成片对比**(唯一产线 `render_3d_video.py`,日常走 `metaurban/ops/render.sh`;并行渲染用 `OUT_MP4` env,全 3D 上限 2 路,headless 3 路)。
> - **默认生成配置(07-14 钦定)**:全知臂 `ORACLE_THIN=1`、KF 臂 `THIN=1`(薄管尺寸包+TDYN+提前量),两臂唯一差别=估计器。seed7 阶梯:KF 裸基线 14.1s → 全知 7.4s / KF 生产 9.4s。薄管无估计误差预算——只报实测净空,**永不声称 conformal 覆盖**。
> - **当前主刀 = KF 三层病**(`.claude/memory/sando-core-kf-campaign-2026-07-14.md`):①新生两点差分速度垃圾(young 诚实门已下刀 53f8fe3)②锥缘丢检串杀 track 重生循环(未修)③成熟速度 0.4m/s 噪声地板卡提前量(未调)。掉头/急停=模型外诚实边界;横穿者 0.7s 让行=物理地板。
> - **场景池 = 20 个场景(seed 0-19,seed≥20 复用 seed%20 换路线)**,07-14 肃清完毕 21/21 全绿;病 seed 手术表焊在 `plan_route`(`_GOAL_FIX` 治终点口袋 / `_ROUTE_SALT` 治路线穿楼,先看卡点离终点多远再选刀)。
> - **ROS 部署臂已开张**:我们的 EGO 接管 MIGHTY Gazebo 全 31 环境(`ops/ego_mighty.sh`)。

> **⚠️ 2026-06-23 方向再聚焦(用户拍板,覆盖下面 6-22 的 framing):** ① 产品 = 认证的「最快+最安全绕行」(certified go-around),**不是判官只 HOLD**;HOLD 降为兜底。② **只做 EGO**(实测效果好),MINCO 暂搁置(代码留作对照/支撑,不投入)。③ 用 **KF(CA 模型)预测障碍未来轨迹** → 把预测占据喂 EGO(solver 不动=良性,仍 agnostic)→ EGO 绕开未来 → 证书检预测移动球 → 过则飞绕行。④ **planner 无关降为支撑性质/通用臂,不是 headline**。M1 已跑通(见 `metaurban/ego_goaround.py` + `.claude/memory/sando-core-goaround-m1-2026-06.md`)。
>
> **2026-06-22 全量重写。** 重心从「冲 RA-L 9/15 的论文蓝图」转为 **工程实现优先**;论文降为下游目标(投不投、何时投未定)。
> 旧的论文-deadline 叙事(W1-W13 时间线、Gate 0 Boyle 签字、三铁律、双 planner 已砍、Isaac 机载标定铁律)**已作废**,被本文件 + 重写后的 `docs/safety-layer-spec.md` + `docs/safety-layer-plan.md` 取代。

Claude:在本仓库工作前先读:
- **权威现状/方向**(先读这两份):`docs/safety-layer-spec.md`(安全层是什么、证书怎么算、诚实边界)+ `docs/safety-layer-plan.md`(工程路线:做完了什么 / 下一步 / 以后再说)。
- `.claude/CLAUDE.user.md` —— 用户(**塔菲大人**)偏好:称呼「塔菲大人」、默认中文、**说人话**(口语清楚、少术语)、简洁、改 bug 别顺手重构、销毁性操作先确认。
- `.claude/CLAUDE.workspace.md` —— `~/code` 工作区/仿真机细节(注:带 display 的仿真机 + `~/code/sando_ws` 只在另一台机器上,本机没有)。
- `.claude/memory/` —— 记忆快照。**`sando-core-status-2026-06.md` 是最新统一真相**;其余多为更早的技术笔记/工作风格(feedback-* 仍有效),framing 可能过时,以前者为准。

## 这个仓库是什么(工程视角)

`sando-core`:一个**认证安全层**的工程实现——无人机在行人/动态障碍附近飞行时,用 **KF 预测**障碍未来轨迹,把预测占据喂给规划器让它**绕开未来**,再对规划器输出的**已承诺轨迹**做一道**精确连续时间碰撞证书**。**产品形态 = 认证的「最快+最安全绕行」(certified go-around);证不过才 HOLD(兜底)。**

**当前只做 EGO**(vendored 去-ROS 的 ZJU EGO-Planner,cubic B-spline,实测效果好);MINCO(自研 min-jerk 五次)暂搁置,代码留作对照/支撑。证书核**与规划器解耦**(同一套 `bernstein_cert.hpp` 数学能跑 MINCO 和 EGO)——这条 **planner 无关是支撑性质/通用臂,不是当前 headline**。喂 EGO 的是「预测占据点云」(solver 不动),所以仍算良性 agnostic。

仿真/评测在 **MetaUrban**(MetaDrive 系)里跑(2026-06-18 起从 Isaac 切过来)。

**一句话现状(⚠️ 2026-06-23 版,已被顶部 07-14 速览覆盖——其中"conformal 没建"已作废)**:精确证书 + 双 planner 适配 + 10-seed A/B + **M1 认证绕行(KF 预测,`metaurban/ego_goaround.py`)已跑通**——EGO 现在会为避开预测中的人群真绕行(横向甩到 y≈4.2)而非干等。**两个诚实点**:① **统计/conformal 半边还没做**(`q_conformal` 全程 0.0 占位 → 确定性几何 margin,不是 P(碰)≤ε 概率保证);② M1 实测**执行净空掉到 0.677 m < d_safe 0.8**(仍>0 没撞)——KF 预测误差吃了裕度,正是 conformal 层(C2)要补的。证书在 MINCO 核里**默认 OFF**。

## 仓库真相(分支/目录)

- **唯一真相 = 分支 `feat/bernstein-gate`(HEAD `46f5ac9`)。**
- `master`(`6d380fd`)只有 MetaUrban 4 类避障 + 可视化,**没有证书/EGO**。
- `bcert-wire`(`e8350cc`)是 feat 的**纯祖先**,少 3 个前沿提交;对应的 `sando-core-bcert/` 是它的旧 worktree → **忽略**。

目录:`cpp/`(MINCO C++ 核 + 证书 `bernstein_cert.hpp` + ctest)、`ego/`(vendored 去-ROS 的 EGO + `ego_capi`)、`metaurban/`(评测 harness:`ego_safe.py`/`ego_bridge.py`/`ab_runner.py`/`render_3d_video.py`)、`isaac/`(旧 Isaac 闭环,已退居二线)、`sando_native/`(MIT-ACL 原版 SANDO + GUROBI,当对照基线)、`docs/`、`.claude/`。

## 跑 demo / 测试

**算法 + C++ 测试 = Ubuntu + conda env `sando`**(权威 `docs/UBUNTU22_PORT.md`):
```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate sando
cd cpp && cmake --build build -j && (cd build && ctest)   # ctest = 25 例(上次 Linux 跑 25/25 全过, LastTest.log 2026-06-20)
# ★ capi 不在 CMake 构建图里,改 C++ 后必须手动重编(否则 python 桥/闭环全断):
g++ -O2 -shared -fPIC -std=c++17 -o capi/sando_capi.so capi/sando_capi.cpp -Iinclude -Ithird_party/eigen -Ithird_party
# ★ EGO 的 capi 也是手编、独立、不在 CMake(注意 -Wno-narrowing):
cd ego && g++ -O2 -shared -fPIC -std=c++17 -Wno-narrowing -w -o capi/ego_capi.so capi/ego_capi.cpp src/*.cpp -I include -I ../cpp/include -I ../cpp/third_party/eigen -I ../cpp/third_party
```
MetaUrban 评测(需 metaurban conda env;`.so` 是 Linux ELF,Windows 跑不了):
```bash
python metaurban/ego_maneuver.py                          # ★当前主线(M3):无 HOLD 圆柱证书机动(飞越/绕行,fastest-safe);EGO_SCENE=gauntlet/crossers/head_on/wall_and_crosser
python metaurban/ego_vs_native.py                         # ★卖点 A/B:同等安全标距下我们 vs 原生 EGO 的速度扫描(因预测而更快)
python metaurban/scenario.py                              # 可手编场景定义自检(start/goal/humans 圆柱)
python metaurban/ego_goaround.py                          # 旧主线(M1/M2):KF 预测认证绕行(二元过则飞/HOLD 兜底),留作 A/B 对照
python metaurban/kf_tracker.py                            # CA-Kalman 追踪器自检
python metaurban/ego_safe.py                              # 更早基线:二元 certify-or-HOLD(喂当前位置)
```
真 3D 渲染出 mp4(★配方关键,见 `.claude/memory/metaurban-render-recipe-2026-06.md`;三件套环境变量缺一不可,否则 ego.replan 段错误):
```bash
cd /media/boxuan/Data2/projects/metaurban && env DISPLAY=:1 \
  PYTHONPATH=/media/boxuan/Data2/projects/metaurban \
  LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6 \
  ~/miniconda3/envs/metaurban/bin/python -u \
  $SC/metaurban/render_3d_video.py --seed 3 --maneuver --clear_spawn --mp4 --t_max 22 --w 440 --h 280
# 变体 --maneuver(ours M3 飞越/绕行) / --ego(native 基线) / --ego_safe(旧 HOLD 层) / --frame_only(单帧验证)
# A/B 左右对比: scratchpad/make_ab.sh 顺序渲两圈 + metaurban/stitch_ab.py 拼接 -> out/ab_maneuver.mp4
```

> 坑:`.so` 加载优先级 `cpp/capi/` > `cpp/build/` > `python/`;Windows 上 `python/sando_capi.dll` 会静默盖住重编。`ego_capi.so` 现在 untracked,可能比源码旧——改 EGO 后记得重编。

## Git 提交署名(强制)
- **禁止** `Co-Authored-By:` 行,**禁止**把 Claude/任何 AI 列为 co-author。
- **禁止** "Generated with Claude Code"、"🤖" 等 AI 署名。
- **积极主动 commit**(2026-07-06 塔菲大人拍板,取代旧的"只在明确要求时"):每到一个有意义的节点(新功能跑通、实验出结果、修完一个 bug)就分主题 commit,别让工作裸奔在工作区——这台机器 CPU 会硬件崩。**推送(push)仍只在明确要求时**做。
