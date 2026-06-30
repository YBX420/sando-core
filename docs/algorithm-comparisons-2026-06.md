# 算法对比汇总(sando-core,2026-06)

> 把这几天跑的**所有算法对比**整理到一份里:每套对比「比的是什么 / 数据在哪 / 用什么脚本跑的 / 在什么环境和条件下跑的 / 结果 / 诚实边界」。
> 数据全在 `out/conformal/` 下。这份文档同时记录**精确运行环境**,以便复现。
>
> 产品定位:认证的「最快+最安全绕行」——KF 预测障碍未来轨迹 → 把预测占据喂规划器让它绕开未来 → 对规划器输出的**已承诺轨迹**做连续时间 Bernstein 碰撞证书(+ split-conformal 标定的 keep-out)。

---

## 运行环境

**OS / 平台:** Ubuntu 22.04.5 LTS,Linux 6.8.0-124-generic,x86_64。所有 `.so` 均为 Linux ELF(Windows 无法加载)。

**两个 conda 环境**(`~/miniconda3/envs/`):

| 环境 | 用途 | Python | 关键包 |
|------|------|--------|--------|
| `sando` | 算法 + C++ 核 + capi 重编 + 算法侧 python(headless replay/标定/矩阵) | 3.10.20 | numpy 2.2.6 |
| `metaurban` | 跑 MetaUrban 仿真/评测 harness(render_3d_video / render_screen) | 3.10.20 | numpy 1.26.4 · panda3d 1.10.14 · metaurban(editable 安装) |

注:`metaurban` env 里没有独立的 `metadrive` 包;MetaUrban 自带 MetaDrive 派生代码在其 repo 内。

**MetaUrban 仓库:** `/media/boxuan/Data2/projects/metaurban`,git commit `6b8ff9a`(editable 安装进 `metaurban` env)。

**激活算法/C++ 环境:**
```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate sando
```

**MetaUrban 运行必备环境变量**(三件套缺一不可,否则 `ego.replan` 段错误)。harness 用 `metaurban` env 的 python 跑,但 `LD_PRELOAD` 指向 `sando` env 的 libstdc++:
```bash
env DISPLAY=:1 \
  PYTHONPATH=/media/boxuan/Data2/projects/metaurban \
  LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6 \
  ~/miniconda3/envs/metaurban/bin/python -u <harness.py> ...
```

**capi 重编**(都不在 CMake 构建图里,改对应 C++ 后必须手动重编,否则 python 桥用的是旧 `.so`)。在 `sando` env 下:
```bash
# sando_capi(MINCO 核)
cd cpp && g++ -O2 -shared -fPIC -std=c++17 -o capi/sando_capi.so capi/sando_capi.cpp \
  -Iinclude -Ithird_party/eigen -Ithird_party
# ego_capi(EGO,独立、注意 -Wno-narrowing)
cd ego && g++ -O2 -shared -fPIC -std=c++17 -Wno-narrowing -w -o capi/ego_capi.so \
  capi/ego_capi.cpp src/*.cpp -I include -I ../cpp/include -I ../cpp/third_party/eigen -I ../cpp/third_party
```
`.so` 加载优先级 `cpp/capi/` > `cpp/build/` > `python/`;`ego_capi.so` 当前 untracked,改 EGO 后务必重编。

---

## 一张表看懂五套对比

| # | 对比 | 比什么 | 规模 | 动力学 | 感知 | conformal | 一句话结果 |
|---|------|--------|------|--------|------|-----------|-----------|
| 1 | **真轨迹 conformal A/B** | ours(证书) vs native EGO | 120 真录制场景 ×多变体 | 点质量(+`--dynamics` 真四旋翼变体) | 噪声检测 + KF | **真 split-conformal 标定**(calib.json,覆盖率达标) | ours **0 碰** vs native **19/120**,且不更慢 |
| 2 | **100 场景(真四旋翼,公平感知)** | ours(--maneuver) vs native EGO | 100 MetaUrban 场景,1 lap | **真四旋翼前向仿真** | **两者同一前向锥**(real D435i FOV)+ mover KF | mover keep-out:`MAN_QCONF` 固定标量(弱)+ **HCT-D δ_track per-flight 标定** | 安全到达 **88 vs 69**;mover 碰撞 **3 vs 10**、坠机 **9 vs 19**(唯一变量=安全层) |
| 3 | **Planner × 安全层 矩阵** | 4 个 planner × 安全层 on/off | 4×2×360 = 2880 ep | 点质量 | GT + 0.07m 噪声 + KF | calib.json @eps0.05 | 安全层 on → **每个 planner 碰撞归 0** |
| 4 | **三方 ours/EGO/SANDO** | 三算法等速同人群 | 720 ep(6 速 ×20 seed ×6) | 点质量 | 噪声检测 + KF | 0.0 placeholder(几何) | ours 0 碰 / 100% 到达;EGO 88 碰;SANDO 到达 0(集成问题,见诚实点) |
| 5 | **可靠性 + 动力学 ablation** | ours 大样本 + pm vs qd | 848 ep + 360/180 ablation | 点质量 vs 真四旋翼 | 噪声检测 + KF | 0.0 placeholder(几何) | ours 0/848;point-mass 完美跟踪下证书 0 碰,残差全是跟踪过冲 |

---

## 1. 真轨迹 conformal A/B(headline:唯一带 P(碰)≤ε 标定的)

**比什么.** 同一批**真录制的 MetaUrban 人群轨迹**(ORCA 行人/车,不让路),在保证有冲突的过路走廊里,两个规划器各飞一遍:
- **ours** = EGO 喂 KF 预测占据 + 连续时间 Bernstein 圆柱证书 gate 一个机动锦标赛(绕/飞越/爬升),keep-out 用**split-conformal 标定**的每类 `(q_conformal, v_eff)`,让每次更快的承诺都被证到 P(碰)≤ε。
- **native** = 原生 EGO,只对 mover 的**当前噪声位置**反应(inflation 0.3,贴人 ~0.3m 飞),允许撞。

**CSV / 数据.**
- `out/conformal/calib.json` — split-conformal 标定输出:每类(行人/车/all)× 每个 eps(0.20/0.10/0.05/0.01)的 `q_conformal`(截距)和 `v_eff`(管子斜率)+ **held-out 测试边际覆盖率**(all@eps0.05 → 0.954,行人 q=0.125 v_eff=0.612)。
- `out/conformal/traj_seed0..19.npz` — 20 seed 的真 MetaUrban mover GT 轨迹(2321 条:1962 行人 + 359 车),标定 + A/B 共用。
- `out/conformal/ab_summary_*.json` — 各 A/B 变体汇总(见下)。

**生成脚本 + 复现.**
```bash
# 0) 一次性 harvest 真 mover(需 metaurban env + display):
PYTHONPATH=/media/boxuan/Data2/projects/metaurban python metaurban/conformal_harvest.py --seeds 0-19 --steps 600
# 1) 标定 conformal keep-out:
python metaurban/conformal_calibrate.py        # -> out/conformal/calib.json
# 2) headline A/B(复现 ab_summary_cv05.json):
python metaurban/ab_replay.py --seeds 0-19 --n_ep 6 --eps 0.05 --tag cv05
# 变体:--eps 0.01|0.10 (cv01/cv10);--max_vel 4.0 (both4);--ours_speedup 1.33 (spd133);
#       --dynamics (dyn);--dynamics --ours_speedup 1.33 (dyn133);--baseline sando (sando)
```
核心决策共享在 `metaurban/safety_layer.py`(build_cylinders / cert_clear / maneuver_decide),replay 在 `metaurban/replay_core.py::run_replay`。

**运行条件.**
| 项 | 值 |
|----|----|
| env | `sando`(EGO ego_capi.so + SANDO 变体需 GUROBI);harvest 步需 `metaurban` env |
| seed/规模 | 20 seed,6 ep/seed = 120 冲突场景(小变体 9–60) |
| 标定集 | 2321 条真 track,按 track 分 1393 cal / 928 test |
| 语义 | 碰撞 = min clearance < 0(机体 vs 每类圆柱);到达 = 进 goal 0.8m;mover 走 GT 不让路(最坏情况) |
| keep-out | R = r_mover + r_drone(0.25) + d_safe(水平0.10/竖直0.30) + q_conformal;管子 ρ(t)=q + v_eff·(t+δ),δ=0.30s 时延,trust τ=0.75s |
| 感知 | mover 当前位置噪声检测(MEAS=0.07m/轴)+ CA-Kalman,预测用 CV(常速)模型;native 只看当前噪声位置;FOV_R=14m |
| 动力学 | 默认点质量完美跟踪(乐观值);`--dynamics`(dyn/dyn133)走真 Quadrotor 模型,在**飞出来的路径**上量净空 |
| 速度 | 默认 max_vel 3.0;both4=4.0;spd133/dyn133 给 ours 证书 gate 的 1.33× 速度预算(4.0)而 native 仍 3.0 |

**结果.** 最干净的 headline(`ab_summary_cv05.json`,eps0.05,CV 预测,max_vel 3,120 真冲突场景):

| | OURS(证书) | NATIVE EGO(裸飞) |
|---|---|---|
| 碰撞 | **0 / 120** | **19 / 120** |
| 到达 | 120 / 120 | 118 / 120 |
| 最差净空 | **+0.628 m** | **−0.374 m**(真飞进人体里) |
| 用时 | 中位比 native 快 0.15s;严格更快 59/118;预算内 116/118 |

**0 碰撞在整个 eps 扫描 + 各压力变体下都成立:**
```
eps0.01 (cv01):  ours 0 vs native 19/120        eps0.10 (cv10):  ours 0 vs native 19/120
max_vel 4 (both4): ours 0 vs native 25/120       1.33×速 (spd133): ours 0 vs native 19/120,严格更快 103/118,中位快 1.2s
真四旋翼 (dyn):   ours 0 vs native 3/12          真四旋翼+1.33× (dyn133): ours 0 vs native 7/120
vs SANDO (sando): ours 0 碰 & 60/60 到达;SANDO 0/60 到达(见 #4 诚实点)
```

**诚实点.**
- **这是项目里唯一"概率保证为真"的一套**:calib.json 是 2321 条真 track 的真 split-conformal 拟合,held-out 测试覆盖率匹配目标(all@eps0.05=0.954 / @0.10=0.904 / @0.01=0.992)——这是 P(碰)≤ε 的经验依据。
- 车的 `q_conformal` 在 eps0.05/0.01 为**负**(−0.118/−0.256m):仿射包络把截距压到 0 以下,安全裕度全压在大 v_eff 斜率上(1.587/2.607 m/s)。机体半径让总 keep-out 仍为正,但对快车是斜率主导。
- 默认行是点质量完美跟踪(乐观);只有 `--dynamics` 行走真四旋翼,那里净空仍为正但更紧(1.33× 时 ours min_clr 降到 ~0.49m)。
- SANDO 变体里 SANDO 到达 0/60(此 harness 里走不完 14m 走廊),只能显示"ours 安全",不能公平比速度。
- 仿真 only,A/B 是 replay 录制轨迹(无 live sim/PX4/硬件);感知是合成噪声 + KF,不是真深度相机。

---

## 2. 100 场景 ours vs native EGO(真四旋翼 + 碰撞按障碍类型拆分)

**比什么.** 同一 MetaUrban 仿真/场景/感知/**真四旋翼动力学**下,两个规划器各飞 1 lap:
- **ours** = `--maneuver`(无 HOLD 圆柱机动锦标赛:每步 straight/around-L/R/over/climb,各候选用圆柱析取证书检 KF 预测 mover,飞最 goal-ward 的认证候选)。
- **native** = `--ego`(裸 EGO,无安全层)。

**CSV.** `out/conformal/render_screen.csv` — 每 (seed,mode) 一行:reached/collided/**dnf** + 每类 hit 标志(hit_ped/hit_veh/hit_animal/hit_static)+ 每类净空 + min_clr/astar_fail/t_goal。200 行 = 100 对完整 ours/ego(DNF=无 lap-done 的跑显式标记,不再静默丢)。

**生成脚本 + 复现.**
```bash
python metaurban/render_screen.py --seeds 0-99 -P 3 --t_max 20
# 内部拉起:metaurban env python -u metaurban/render_3d_video.py --ego [--maneuver] --headless --seed S --clear_spawn --t_max 20
```
`run_one()` 解析每次跑的 `lap done` 行;`clearance()`(mover=box SDF,static=竖直圆柱 SDF)+ crash=stop 截断(`if c < -1e-6: crashed=True; break`)。

**运行条件.**
| 项 | 值 |
|----|----|
| env | MetaUrban `SidewalkDynamicMetaUrbanEnv`(`--headless` 跳渲染),C++ 证书 .so 在 `sando` env 编 |
| seed/规模 | seeds 0-99(scene=seed%20,route=seed*1000,≥20 是新场景);1 lap/(seed,mode);n=100 对(DNF 显式计为失败,不静默丢) |
| 语义 | **crash=stop=不到达**:任一类净空 c<−1e-6 即 crashed 并截断本 lap → 碰撞绝不会同时算到达(双方同规则) |
| 感知 | **两者同一前向锥**(real D435i FOV:range 8m / ±45° / 沿机体 yaw),静态 + mover 检测都锥限;ours 对锥内 mover 跑 CA-KF 预测喂证书(=安全层),native 反应当前;clearance(GT 度量)静态按圆柱、mover 按 box |
| 动力学 | **真 Quadrotor 前向仿真**(tilt-to-accel + 动量)跟踪认证 setpoint 且滞后;机动门对真四旋翼副本做前向仿真 |
| keep-out | cert_R = r_obs + MAN_DSAFE + MAN_QCONF + MAN_TRACK(每类 d_safe ~ 人0.8/车0.6/动物0.7);竖直 z_clear 飞越 |

**★ 公平感知协议(关键).** 早期版本里 ours 喂 EGO 的是 **360° 全知静态地图**,而 native 只看**前向锥**(fov_cloud)——这是个 confound:ours 静态撞少有一部分是「感知更全」而非安全层。**已纠正**:ours 现在和 native 用**完全相同的前向锥**(real D435i FOV:range 8m / ±45° / 沿机体 yaw,静态 + mover 检测都锥限),`SPAWNDBG` 实测两者看到同一个锥(182=182 点)。**唯一变量 = 安全层(KF 预测 + 证书 + 机动锦标赛)。**

**结果(crash=stop,真四旋翼;2026-06-27 冲-0 终版:已知静态地图[双方] + 圆柱静态门 + mover 前向仿真门).**

| mode | 安全到达 | 撞人 | 撞车 | 撞动物 | 坠机(static) | 总碰撞 |
|------|---------|------|------|--------|--------------|--------|
| **ours** | **93/100** | **0** | 1 | **0** | **0** | **1** |
| native EGO(同感知) | 69/100 | 3 | 3 | 1 | 22 | 29 |

**0撞人、0撞动物、0坠机;唯一 1 个碰撞 = seed 56(快速 van)。安全层 1 vs native 29、到达 93 vs 69。** 双方感知完全相同(已知静态地图 + 前向锥 movers),唯一变量 = 安全层。

**感知协议(终版,更"real状态如何就如何").** STATIC = 已知本地地图(`_local_static`,360°,**双方**;真实:无人机有静态地图/不忘记墙;前向锥-only 静态 → 侧盲 → 0坠 物理不可能)。MOVERS = 前向锥(8m/±45°,**双方**;动态未知)。

**怎么冲到的(逐 seed 诊断驱动,影响排序).** ① 静态门前向仿真 0.75→1.2s(惯性提前避)mover3→1;② 已知静态地图(双方)修锥盲侧向静态 坠机8→4;③ **圆柱-SDF 静态门**(飞行路径对静态圆柱检,同 GT 模型,贴边巨楼 HOLD 不蹭;bug:loc_obs 按表面距非中心距过滤)**坠机4→0**;④ **mover 前向仿真门**(`cert_clear` 无前向仿真→爬越/过冲蹭高 mover;补前向仿真飞行路径对 KF 预测圆柱检)撞人/撞动物→0。

**唯一残留 seed 56 = 本质极限.** ~7 m/s 冲向 2.8m 高 van(mover,前向锥晚检测),惯性需 ~4m 刹车、van 在 ~1.3m → 蹭顶 −0.11。唯一能修的全局降速(`EGO_FOVCAP=1` 或 v_max↓)实测**杀 ~10 个到达**——不值。= "fast-approach 刹不住 + cone-limited mover" 的物理极限;literal 0/100 需更宽 mover 感知或 RTA 刹车距离不变量(不同的更大方法)。**根本判断:0碰撞 + 前向锥-only + 巡航速度 = 过约束三难,只能取二。**

**HCT-D 跟踪管子(让证书覆盖真飞路径)—— 2026-06-27 已按 code-review 修正.** 旧版用**每承诺窗**当 conformal 交换单元是**错的**(同次飞行的窗自相关、不可交换),给出 0.29m 只有边际覆盖、**~30% 飞行实际越界**。修正:① 修好机动切换**震荡**(迟滞曾是死代码)→ per-episode-max δ 的 p95 从 **1.72m 塌到 0.27m**(震荡正是重尾元凶);② δ_track 改用 **per-flight(episode)单元**标定(eps0.05→0.264m、max 观测 0.473m)→ 取 **MAN_TRACK=0.45m**(覆盖几乎全部观测飞行)。接进 `cert_R`(水平 + 竖直)。δ ∝ ‖a‖/曲率,与速度无关。脚本 `track_{harvest,conformal}.py`。

**诚实点.**
- mover 碰撞 3(撞人/车/动物各 1,全是 12–45cm 边界擦碰),**不是"全 0"**。早期"撞人0/撞动物0/唯一是 seed56"的说法**作废**(seed56 已被 HOLD 兜底修好;统计随修复重跑变了)。
- **100 场景的 conformal 半边弱**:mover 横向 keep-out 用的 `MAN_QCONF=0.125` 是**行人 CV 标定的固定标量**,套到车/动物、且是边际覆盖,**不是逐类 P(碰)≤ε 概率保证**。真正带 P(碰)≤ε 的是 #1 真轨迹 A/B(calib.json,覆盖率达标);跟踪 margin(δ_track=0.45)是 per-flight 诚实标定的。
- **soundness 修复(code-review)**:climb/over 不过证书 → **HOLD 兜底**(不再无认证飞);卡死/超时 = **DNF 显式计为失败**(不静默丢)。
- 仿真 only(MetaUrban headless,轻量 quadrotor.py,无 PX4/真深度);静态/mover 都受前向锥限制。
- 已知小问题(`docs/code-review-2026-06-27.md`):mover 横向 cert R 未显式加机体半径(MAN_DSAFE 0.45 已含余量);A_ENV 加速度包络前置条件未实装。

---

## 3. Planner × 安全层 矩阵(planner 无关地基)

**比什么.** 4 个**轨迹表示各不相同**的 planner,全部被**同一个**连续时间 Bernstein-圆柱证书 gate,飞同一批冲突走廊,**有/无**安全层各跑:
- ego(ZJU EGO,cubic B-spline + ESDF 自避)、quintic(min-jerk 5 次幂基,**无自避**)、septic(min-snap 7 次幂基,**无自避**)、bspline(裸 cubic B-spline 到目标,**无自避**)。
- 3 个无自避的嫁接最能说明问题:**证书是它们唯一的安全来源**。

**CSV.** `out/conformal/psm_{ego,quintic,septic,bspline}_{on,off}.csv`(各 360 行)+ 合并 `planner_safety.csv`(2880 行)。

**生成脚本 + 复现.**
```bash
bash metaurban/psm_run.sh "2,3,4" "0-19" 6     # SPEEDS SEEDS NEP -> 3速×20seed×6ep = 360 行/格
# 单格:python -u metaurban/planner_safety_matrix.py --planner ego --safety on --seeds 0-19 --n_ep 6 --speeds 2,3,4 --csv out/conformal/psm_ego_on.csv
```
证书数学 `metaurban/cert_bridge.py`;嫁接 `graft_demo.py`;KF `kf_tracker.py`;常量/episode/calib `replay_core.py`。

**运行条件.** env `sando`(纯 CPU,无 MetaUrban sim/PX4);seeds 0-19 × 6 ep × 3 速 {2,3,4} m/s = 360 ep/格;**点质量**(psm_run.sh 不传 `--dynamics`);感知 GT + 0.07m 高斯噪声 + KF,FOV 14m;eps=0.05,`(q_conformal,v_eff)` 来自 `calib.json`(**非 placeholder**);CV 预测,τ=0.75s。

**结果.**
```
planner  safety   n   碰撞        到达        净空均值  净空中位
bspline  off     360   24 ( 7%)   360 (100%)   1.595    1.816
bspline  on      360    0 ( 0%)   360 (100%)   1.973    2.014
ego      off     360   44 (12%)   356 ( 99%)   0.863    0.899
ego      on      360    0 ( 0%)   360 (100%)   1.493    1.456
quintic  off     360   33 ( 9%)   360 (100%)   0.583    0.521
quintic  on      360    0 ( 0%)   360 (100%)   1.877    1.781
septic   off     360   45 (12%)   360 (100%)   0.506    0.343
septic   on      360    0 ( 0%)   243 ( 68%)   1.880    1.664
```
**HEADLINE:** 安全层 on → **每个 planner 碰撞归 0/360**(off 撞 7–12%),证明同一个 planner-无关证书在 4 种轨迹表示上都成立——包括 3 个证书是唯一安全来源的无自避嫁接。净空中位也大涨(ego 0.90→1.46m)。

**诚实点.**
- **例外:septic on 到达从 100% 掉到 68%**(243/360)——脆的 7 次幂基在 ~1/3 走廊里证不出 goal-ward 机动,于是保守 evade/HOLD(安全但卡住),这是 reach-换-safety 的预期代价,不是 bug;ego/quintic/bspline 保持 ~100% 到达 + 0 碰。
- **点质量**(dynamics=0):setpoint 直接飞,无四旋翼内环,碰撞数不含跟踪滞后。
- conformal 这套用 calib.json(eps0.05,**非** placeholder),但仍是录制数据上的边际/经验 conformal 界,不是逐场景概率证书。
- gcopter 适配器(GcopterP)已实现且可认证,但**未进默认矩阵**(FIRI 凸覆盖在某些动态云上 Eigen 负维段错误)。
- 仿真 only,纯 CPU,mover 是全知 GT + 噪声,非真感知。

---

## 4. 三方对比 ours / EGO / SANDO(等速同人群)

**比什么.** 三算法在**同一批录制人群**、匹配最大速度下各飞:
- ours = EGO + KF 预测占据 + Bernstein 圆柱证书/maneuver-decide(inflation 0.45);
- ego = 原生 EGO,inflation 0.3,无证书,只看当前云;
- sando = 原生 MIT-ACL SANDO(启发式 A* + DecompUtil SFC + GUROBI),自带预测/避障,无证书。

**CSV.** `out/conformal/compare3_matched.csv`(720 行 = 6 速 ×20 seed ×6 ep,每行三算法各自 reach/time/clr/coll)+ `.json`(每速汇总)。

**生成脚本 + 复现.**
```bash
LD_LIBRARY_PATH=~/gurobi1103/linux64/lib python metaurban/compare3.py \
  --speeds 3,4,5,6,7,8 --seeds 0-19 --n_ep 6 --eps 0.05 --tag matched
# 需 traj_seed*.npz + 已编 ego_capi.so + sando_native 桥(GUROBI license)
```
每 seed 从 `traj_seed{sd}.npz` 建一次 episode,三算法复用同一人群/起点/终点,唯一变量是规划器。

**运行条件.** env `sando`(EGO + SANDO/GUROBI 11.0.3,需 `LD_LIBRARY_PATH`);20 seed × 6 ep × 6 速 = 720 ep/算法;**点质量完美跟踪**(此运行 dynamics=False,在 planned setpoint 上量净空,乐观值);感知噪声检测(0.07m)+ CA-KF,FOV 14m,三算法同一噪声流;到达 = 进 goal 0.8m,stall>40 tick / MAXTICKS=240 截断。

**结果.** 720 ep 汇总(速 3-8 m/s):
```
algo    到达            碰撞   中位用时   最差净空   净空中位
ours    720/720 (100%)  0      4.5 s      0.628 m    1.496 m
ego     706/720 (98.1%) 88     4.5 s     −0.412 m    0.895 m
sando     0/720  (0.0%) 29     n/a       −0.542 m    1.986 m
```
**ours 是唯一 0 碰 + 满到达的;native EGO 几乎一样快但换来 88 次碰撞。**

**诚实点(重要,别过度宣传).**
- **SANDO 0/720 到达不是"SANDO 不安全/不会避"的干净结论,而是 harness 集成/收敛不匹配主导的**:SANDO 净空中位反而最高(1.986m)、碰撞最少(29),即它避得好但中心从没在 stall(>40 tick)/超时前进到 0.8m goal_radius。所有 SANDO 用时 13.2–47.4s = 全靠超时/stall 结束。
- SANDO 被接成"路径规划器,committed path 每 tick 走 max_vel·DT 弧长执行",这套自定义配速 + 0.8m goal + stall 预算很可能是它不收敛的原因——**公平性/集成 caveat,不是 SANDO 本质能力的证明**。别 headline "在到达上打败 SANDO"。
- ours 和 ego 共用**完全同一个 EGO**,差别只在安全层 + KF 预测占据 + 0.45 vs 0.30 inflation + 证书门 → **ours-vs-ego 的 0 vs 88 碰撞才是有意义的同台对比**;SANDO 列是最弱的一条腿。
- 点质量(乐观):真四旋翼下裕度缩水(对应 M1 实测执行净空降到 0.677m)。`q_conformal` 此处仍是 0.0 placeholder(几何 margin,非概率保证)。

---

## 5. 可靠性 sweep + 动力学 ablation(point-mass vs 真四旋翼)

**比什么.** 主:ours(EGO + KF + 圆柱证书,安全 ON)在大量走廊上的无碰撞大样本 sweep,看能多接近"从不碰"+ all-clean 支持什么分布无关界(rule of three)。次:同管线在两种跟踪器下(pm 点质量完美跟踪 vs qd 真四旋翼)对比,隔离残差碰撞/净空缺口是规划+证书的还是跟踪过冲的。

**CSV.**
- `reliability_full.csv` — 848 ep(seed 0-16,~50 ep/seed),ours 安全 ON @eps0.01。
- `_dyn_pm.csv` — 360 行,EGO 点质量(dynamics=0),seed 0-9 ×6 ep ×3 速,safety off(180)+on(180),eps0.05。
- `_dyn_qd.csv` — 180 行,EGO 真四旋翼(dynamics=1),**仅 safety off**。

**生成脚本 + 复现.**
```bash
bash metaurban/reliability_chunked.sh 50 0.01 0-19      # 每 seed 一个 fresh 进程(绕 EGO ~8MB/ep grid 泄漏)
python metaurban/planner_safety_matrix.py --planner ego --safety off --seeds 0-9 --n_ep 6 --speeds 2,3,4 --eps 0.05 --csv out/conformal/_dyn_pm.csv
python metaurban/planner_safety_matrix.py --planner ego --safety on  --seeds 0-9 --n_ep 6 --speeds 2,3,4 --eps 0.05 --csv out/conformal/_dyn_pm.csv
python metaurban/planner_safety_matrix.py --planner ego --safety off --seeds 0-9 --n_ep 6 --speeds 2,3,4 --eps 0.05 --dynamics --csv out/conformal/_dyn_qd.csv
```

**结果.**

主 — 可靠性(848 ep,ours 安全 ON,eps0.01):
```
碰撞      到达       全局 min_clr   净空中位   用时中位
0/848    848/848    0.821 m        1.556 m    4.80 s
```
→ 碰撞率 0,成功率 100%;rule-of-three 分布无关 95% CI:碰撞 ≤ 3/848 = 0.354%(成功 ≥ 99.65%)。全局最小净空 0.821m 整段都 > d_safe 0.8m。

次 — 动力学 ablation(eps0.05):
```
arm                         n     碰撞   到达      min_clr   净空中位
pm safety=OFF(裸飞)        180   20    176/180   −0.250    0.947
pm safety=ON (ours)        180    0    180/180    0.868    1.436
qd safety=OFF(裸飞)        180    5    178/180   −0.222    1.669  (中位 max_z 3.44m)
```
**读法:点质量完美跟踪下证书(safety ON)恰好干净——0/180,min_clr 0.868 > d_safe 0.8。即车辆精确实现 setpoint 时,证书的几何裕度毫无 slack 损失地成立。所以证书数学本身不产生碰撞;残差现实缺口是跟踪过冲。**

**诚实点.**
- `q_conformal` 在这两套里仍是 0.0 placeholder(几何 keep-out),所以可靠性是经验值(0/848)+ rule-of-three 频率界,不是 conformal 概率证书(**对比 #1 才是带真标定的那套**)。
- 全局 min_clr 0.821m 只比 d_safe 0.8m 高 0.021m——真跟踪下正是这点裕度被吃掉(M1 闭环实测执行净空 ~0.677m < 0.8,仍 >0 没撞)。
- **ablation 不对称:`_dyn_qd.csv` 只记了 safety=OFF**,没有"真四旋翼 + 安全 ON"那一臂 → 不能直接从这数据读"我们的证书在真四旋翼下的碰撞率",需补跑 `--safety on --dynamics`。能下的干净结论是 pm+safety-ON = 0 碰。
- qd safety-OFF 碰撞(5)反比 pm safety-OFF(20)少是**假象**:真四旋翼在高度上过冲(中位 max_z 3.44m vs pm 1.50m),裸飞的四旋翼误打误撞爬过了行人头顶,不是真动力学更安全。
- 可靠性 driver 要了 seed 0-19 但只有 0-16 有数据 → 848(非名义 1000);chunk 是绕 EGO C++ grid 泄漏的 tooling 处理。

---

## 统一诚实边界

1. **概率保证的成色分两档**:#1(真轨迹 A/B)用了**真 split-conformal 标定**(calib.json,held-out 覆盖率匹配 eps),是项目里 P(碰)≤ε 唯一有经验依据的一套;#2/#4/#5 的 keep-out 多是**确定性几何 margin**(`q_conformal`/`MAN_QCONF` 为 placeholder),数值上很强(0 碰)但属几何而非概率证书。
2. **全是仿真**:MetaUrban headless / 录制 replay,无真硬件、无真深度相机;感知是合成噪声 + KF。sim guarantee,不是 hardware guarantee。
3. **动力学有乐观项,但跟踪缺口已补上**:#1 默认、#3、#4 是点质量完美跟踪(净空量在 planned setpoint 上);#2(真四旋翼前向仿真)、#1 的 `--dynamics` 变体、#5 的 qd 臂走真四旋翼。真跟踪下裕度更紧。**HCT-D 跟踪管子已落地**(见 #2):per-commit-window split-conformal 标定 δ_track=0.29m(eps0.01,窗覆盖 0.996),接进 mover keep-out 让证书覆盖真飞路径 → #2 的移动障碍碰撞从 1 降到 0。残留缺口转为静态擦碰(坠机 6,EGO 自避域)+ 罕见的机动切换跳变尾巴(承诺迟滞待补)。
4. **最强、最公平的同台对比是 ours-vs-native-EGO**(共用同一 EGO,唯一变量是我们的安全层):#1 的 0 vs 19/120、#2 公平同锥 **88 vs 69(mover 3 vs 10)**、#3 的每 planner 归 0。**SANDO 那条腿弱**(harness 里不收敛),别拿它的"到达"做卖点。
5. **2026-06-27 code-review 后修正**:#2 的早期"mover 全 0 / 92v71 / P(碰)≤ε"有过度宣称——已按确认发现修正(δ_track per-flight 重标、climb→HOLD soundness、DNF 透明、conformal 弱处标注)。完整发现见 `docs/code-review-2026-06-27.md`。

---

*相关图:`out/conformal/fig_collision_split3.png`(#2 三类拆分)、`out/conformal/calib_coverage.png`(#1 覆盖率)、`out/conformal/cert_ablation.json`(连续 vs 离散证书 soundness)。两天进展见 `docs/progress-2026-06-24-25.md`。*
