# CLAUDE.md — sando-core(给 Claude Code 自动读的项目入口)

> **2026-06-22 全量重写。** 重心从「冲 RA-L 9/15 的论文蓝图」转为 **工程实现优先**;论文降为下游目标(投不投、何时投未定)。
> 旧的论文-deadline 叙事(W1-W13 时间线、Gate 0 Boyle 签字、三铁律、双 planner 已砍、Isaac 机载标定铁律)**已作废**,被本文件 + 重写后的 `docs/safety-layer-spec.md` + `docs/safety-layer-plan.md` 取代。

Claude:在本仓库工作前先读:
- **权威现状/方向**(先读这两份):`docs/safety-layer-spec.md`(安全层是什么、证书怎么算、诚实边界)+ `docs/safety-layer-plan.md`(工程路线:做完了什么 / 下一步 / 以后再说)。
- `.claude/CLAUDE.user.md` —— 用户(**塔菲大人**)偏好:称呼「塔菲大人」、默认中文、**说人话**(口语清楚、少术语)、简洁、改 bug 别顺手重构、销毁性操作先确认。
- `.claude/CLAUDE.workspace.md` —— `~/code` 工作区/仿真机细节(注:带 display 的仿真机 + `~/code/sando_ws` 只在另一台机器上,本机没有)。
- `.claude/memory/` —— 记忆快照。**`sando-core-status-2026-06.md` 是最新统一真相**;其余多为更早的技术笔记/工作风格(feedback-* 仍有效),framing 可能过时,以前者为准。

## 这个仓库是什么(工程视角)

`sando-core`:一个 **planner 无关的「认证安全层」** 的工程实现——无人机在行人/动态障碍附近飞行时,给规划器输出的**已承诺轨迹**做一道**精确连续时间碰撞证书**;证不过就 **HOLD**(判官,不矫正)。

证书核**与规划器解耦**:同一套数学(`bernstein_cert.hpp`)既跑自研的 **MINCO**(min-jerk 五次)规划器,也跑 vendored 进来的 **EGO-Planner**(ZJU,cubic B-spline)。后者是「planner 无关」的活体证明,目前算主贡献(未终结)。

仿真/评测在 **MetaUrban**(MetaDrive 系)里跑(2026-06-18 起从 Isaac 切过来)。

**一句话现状**:精确证书 + 双 planner 适配 + EgoSafe 闭环 + 10-seed A/B 已跑通;**统计/conformal 半边还没做**(`q_conformal` 全程是 0.0 占位,所以现在是确定性几何 margin,不是 P(碰)≤ε 概率保证);证书在 MINCO 核里**默认 OFF**。

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
python metaurban/ego_safe.py                              # EgoSafe 无头自检(3 横穿人, 断言执行净空≥0)
python metaurban/ab_runner.py                             # EGO raw vs EGO+层 10-seed A/B -> out/ab_runs/
python metaurban/render_3d_video.py --ego --ego_safe ...  # 带渲染的完整闭环(需 display/GPU)
```

> 坑:`.so` 加载优先级 `cpp/capi/` > `cpp/build/` > `python/`;Windows 上 `python/sando_capi.dll` 会静默盖住重编。`ego_capi.so` 现在 untracked,可能比源码旧——改 EGO 后记得重编。

## Git 提交署名(强制)
- **禁止** `Co-Authored-By:` 行,**禁止**把 Claude/任何 AI 列为 co-author。
- **禁止** "Generated with Claude Code"、"🤖" 等 AI 署名。
- 提交/推送**只在用户明确要求时**做。
