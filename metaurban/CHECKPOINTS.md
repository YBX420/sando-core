# 场景工作台 · Checkpoints(2026-07-02 夜间自主推进)

> 给塔菲大人:每个 CP 一节,含**做了什么 / 怎么验证的 / 你怎么复现 / 已知问题**。
> 全部命令在 `sando-core/metaurban/` 下执行;评测用 sando conda env,3D 预览用 metaurban env。

---

## ✅ CP0(白天已完成,supra):A桶修复 + 工作台三件套 + 3D 编辑器

- A桶 6 项 config-无关修复全清(A* 泄漏 / deg<2 假证 / climb 门 / 三值判决 / 两点差分 KF / δ_track 钳位),ctest 25/25,三个 .so 重编。
- `scenario_lib.py`(schema v1 + 编译器)/ `scenario_designer.py`(2D 俯视)/ `scenario_designer3d.py`(3D,Panda3D)/ `run_scenario.py`(评测 CLI)。
- 复现:`python run_scenario.py scenarios/wall_and_crosser.json`
- 3D 预览(metaurban env):
  ```
  env DISPLAY=:1 PYTHONPATH=/media/boxuan/Data2/projects/metaurban \
    ~/miniconda3/envs/metaurban/bin/python scenario_designer3d.py \
    --preview scenarios/vehicle_spawn_accel.json --seed 3
  # -> out/scenario_previews/*.png(4 时刻 × iso/back 两机位)
  ```
- 交互 3D 编辑器(**待你上机实测**):`... scenario_designer3d.py --seed 3`
  键位:1/2/3=start/goal/无人机路点,p/v/a/o=新行人/车/动物/障碍,点击=放置/加路点,
  TAB=选中循环,q/e=换 asset,[/]=速度,c=常速↔加速,9/0=加速度,,/.=出生时刻,
  SPACE=动画预览,F2=导出,ESC=退出。相机=原生 bird cam(WASD 平移,滚轮缩放)。

## ✅ CP1:场景库扩充(10 个场景,9 可用 1 阻塞)

新增 4 个(全部过评测、带 3D 预览图):

| 场景 | 打什么 | 结果 |
|------|--------|------|
| `fast_overtake` | 后方高速超车(9m/s)+ 前方 crosser 同时逼离中线 | reach 8.1s clr2.09,evade 被逼出 |
| `climb_trap` | U 形墙 + 开口巡逻者 → 逼 climb,考高空 strand | reach 12.3s,**climb×5 over×1**(陷阱生效) |
| `crowd_dense` | 10 行人交叉穿行(8 动 2 站),liveness 压力 | reach 11.1s clr1.29,混合机动 |
| `speed_sweep` | wall_and_crosser 布局 × vmax **网格** 1..6 m/s | 6/6 reach,用时单调 20.7→5.4s,零碰撞 |

- `expand_variants` 新增 **grid 支持**(`"grid":[...]` 按变体序循环取值)→ 速度扫描/参数阶梯用这个;
  `--variants 6` 即跑满一轮网格。speed_sweep 是 93-vs-69 对账要的速度匹配夹具(ours/native 同 vmax 成对跑)。
- 3D 预览:static 现在渲成**实体灰柱**(box.egg + setShaderOff;之前是小点看不出墙)。
- 复现:`python run_scenario.py scenarios/speed_sweep.json --variants 6`
- 库存:wall_and_crosser / crossers(⛔) / gauntlet / head_on / vehicle_spawn_accel / occlusion_reveal
  / fast_overtake / climb_trap / crowd_dense / speed_sweep

## 🔴 已知阻塞:crossers 场景 → EGO 存量 segfault(CP2 目标)

- `run_scenario.py scenarios/crossers.json` 确定性崩(exit 139),replan #7。
- **差分实验已证明与今日补丁无关**(stash 全部改动、纯 HEAD 源编 .so → 同样崩)。
- gdb:`ego_planner::BsplineOptimizer::initControlPoints` 内 `vector<Vector3d>::_M_realloc_insert`
  SIGSEGV → 疑更早处越界写破堆。与审查点名的 warm-start 区域(`ego_bridge.py:62` poly_init 恒 1)重合。

## ✅ CP2:crossers segfault 根修(上游 EGO-Planner 存量 UB,已除根)

**根因**(ASan 定位到 `bspline_optimizer.cpp` initControlPoints/check_collision_and_rebound 两处同模式):
1. `if (got_intersection_id >= 0)` 用**跨 j 的陈旧值** → 本轮 j 无交点时读**未初始化** `intersection_point`(垃圾 length/NaN);
2. `flag_temp[j]=true` 在 `length>1e-5` 检查**之前**设置 → 退化交点留下"已标记但 base_point 为空" → step 3 链式复制 `.back()` 打在空 vector 上(即崩溃点 :291/:823)。

**修法**(两处同修):`fresh_intersection` 本轮标志替代陈旧 id;`flag_temp` 只与实际 push 原子绑定;step 3 加空邻居守护(跳过而非 UB)。

**验证**:
- crossers 通过:reach 7.5s clr1.24 零碰撞(修复前 replan#7 必崩);
- **ASan 全程零报告**(UB 除根,非糊住);
- 健康路径零回退:wall_and_crosser 等修复前后数字**逐位一致**。
- 工具沉淀:`ego_bridge.py` 支持 `EGO_CAPI_SO` env 覆盖;ASan 版 `ego/capi/ego_capi_asan.so` 留存,复现:
  ```
  LD_PRELOAD=$(gcc -print-file-name=libasan.so) ASAN_OPTIONS=detect_leaks=0 \
    EGO_CAPI_SO=../ego/capi/ego_capi_asan.so python3 run_scenario.py scenarios/crossers.json
  ```

## ✅ CP4:全场景库回归基线(15/15 全过、零碰撞)

- 9 场景 + speed_sweep×6 变体全部 reach,无碰撞;速度阶梯单调(vmax1→6: 20.7→5.4s)。
- 基线快照:`out/scenario_runs/BASELINE_2026-07-02.md`(环境冻结前的对照参考,以后改动跑同款对表)。

## ✅ CP3(部分):3D 预览打磨

- static 渲成实体灰柱(box.egg + setLightOff/setTextureOff/**setShaderOff**——场景 auto-shader 会吃掉 setColor,三个都得关)。
- 行人从纯 T-pose 改 Actor+pose(baked 'Take 001' 第 10 帧)+ force update——**部分生效**(近景摆姿势,远景仍 T-pose,惰性求值时序问题)。⚠️纯外观,不影响任何功能,后续再磨。
- 车辆朝向:预览帧目测正确(+Y 车头约定);若交互中发现某款车倒开,在 `HPR_FIX` 加 per-asset 修正即可。

## ✅ CP5:realtime 3D 飞行 + mp4 + 回放叠加(`run_scenario_3d.py`)

**控制回路就是 `replay_core.run_replay` 本体**(加了 `tick_cb` 每 tick 钩子,单一真相)——3D 里渲的、
realtime 里看的、headless 评测跑的,是同一份 tick/KF/证书/tournament。验证:mp4 渲完的评测数字与
headless 基线**逐位一致**(vehicle_spawn_accel: 7.5s/1.442/evade×2)。

```bash
# realtime 窗口($DISPLAY 上 1:1 墙钟节奏,ESC/q 退出),可同时录 mp4:
env DISPLAY=:1 PYTHONPATH=/media/boxuan/Data2/projects/metaurban \
    LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6 \
  ~/miniconda3/envs/metaurban/bin/python run_scenario_3d.py scenarios/crowd_dense.json \
    --seed 3 --live --mp4
# 纯 mp4(CLI 工作流): ... --mp4        # -> out/scenario_videos/<name>_<mode>.mp4
# 叠加上一次跑的轨迹:   ... --overlay out/scenario_runs/<name>_ours_hist.json
# 其它: --view chase(跟拍)/iso  --speed 2.0(2x)  --mode native(基线) --dynamics(真四旋翼动力学)
```
- 飞行轨迹按决策 kind 着色:绿=straight 蓝=around 黄=over 橙=climb 红=evade;HUD 显示 t/kind/clearance。
- ⚠️ LD_PRELOAD sando 的 libstdc++ 必需(缺了 ego.replan 段错误,同 render_3d_video 配方)。
- realtime 实测:head_on 7.2s 仿真 ≈ 7.4s 墙钟(env 构建另 ~20s)。
- RGB 已修:get_rgb_array_cpu 本就是 BGR,此前 preview 多翻了一次通道(红蓝互换),已去掉。
- 交互编辑器速度键 [ ] 与 bird 相机旋转冲突 → 移到 u/i。

## ✅ CP6:Web UI 工作台(`scenario_workbench.html`,已发布可远程用)

单文件 HTML(332KB,内嵌 seed3 真地图底图 8px/m),零依赖零服务——浏览器/手机直接开:
- **编辑**:左侧模式栏(S/G/WP 无人机三件 + PED/VEH/ANI/OBS),点击放置、拖动、Delete 删点;右侧检查器改速度(恒速↔加速 v0→v1@a)、出生时刻、hold_end、asset、静态 r/h、params;
- **动画预览**:JS 移植的 compile_mover(**与 python 输出逐位一致**,vehicle_spawn_accel 验证 t/vmax/点数全同),空格播放、时间轴拖动;
- **回放叠加**:导入 `out/scenario_runs/*_hist.json` → 实飞轨迹按决策着色 + 时间轴联动;
- **导入导出**:下载 JSON 直接喂 `run_scenario.py`;导入文件/粘贴文本。
- 构建:`scenario_workbench.template.html` + 注入脚本(见 git blame);改 UI 后重跑注入再发布。
- 在线版:https://claude.ai/code/artifact/c05a5786-5441-41fc-9a13-21e2d725029d

## ✅ CP7:逼真感知前端(`perception.py`,去 GT 透视的第一刀)

**单一共享模块**(replay_core 已接,render_3d_video 待接——不再造第二份拷贝):
- 四层:FOV 锥(半角45°/量程10m)→ 硬遮挡(2D 射线×其它圆柱)→ 距离漏检 P(d)=p0+p1(d/R)² → 距离噪声 σ(d)=s0+s1·d;
- **NN 关联航迹管理,无 GT 身份**(贪心最近邻+门限,出生/coast/TTL 击杀;复用两点差分 KF)——交叉可换 ID、遮挡可断轨,这正是要的;
- 全部 PERCEPT_* env 可调;`PERCEPT=gt` 为显式对照且**仍是默认**(重标定前不静默改 headline 语义);
- 自测三件:遮挡(burster y=0.80 才被跟踪)、关联(2 走者→2 轨,v 收敛)、coast/TTL 生灭。

**全库 A/B(out/scenario_runs/PERCEPT_AB_2026-07-02.md)**:18/18 reach、双模式零碰撞;
多数场景净空收紧,**occlusion_reveal 1.436→0.476m** = fresh-track 欠覆盖的活体试验台
(审计的 age-conditional 观察第一次物理复现;B桶 per-age q / 归一化 conformal 的验收对象)。

复现:`PERCEPT=realistic python3 run_scenario.py scenarios/occlusion_reveal.json`

**下一步(依赖此件)**:①render_3d_video 的 kf_movers 接同一前端(那边的 GT 泄漏还在);
②B桶标定改为对 [realistic 间歇检测] 收割重标(标定端≠部署端的条件律漂移从根上消掉);
③假阳性/分类错误/尺寸估计误差三个 dial 后续加。

## ✅ CP8:地基②收尾——render_3d_video 接前端 + 感知 dial 补全

- **render_3d_video 的 GT 泄漏关闭**:`kf_movers` 真实锥路径(cam_heading 给定的 L993 机动/L1766 slip)
  在 `PERCEPT=realistic` 下改走 `_kf_movers_realistic`(共享前端;输出合同不变:
  `(oid,c3,vel,r_eff,d_safe)`+`_KF_PRED`;coast 航迹按 MAN_MEM_K·pos_sigma 膨胀,与 GT 路径语义一致;
  锥参数自动继承 `--fov_deg/--fov_range`;per-lap 重置含前端 tracks)。全知调试路径(无 heading)保持 GT。
  冒烟:seed3 --maneuver --headless 10s 完整跑通,决策流正常。
  ⚠️ v1 诚实边界:**遮挡只算 mover 圆柱之间;静态建筑还不遮挡**(需静态场景几何接入,下一步)。
- **三个新 dial**(机制就位,值等冻结定):`PERCEPT_FP_RATE`(泊松杂波,默认 0)、
  `PERCEPT_CLS_ERR`(类别翻转,默认 0)、`PERCEPT_SIZE_ERR`(r/h 对数正态尺寸误差,默认 5%)。
  自测:杂波航迹正常生灭、类别翻转流经证书不炸。
- 两条 harness 现在**同一个感知前端**(perception.py 单一真相)——climb 分叉那种"两份拷贝各自漂移"
  在感知层从根上杜绝。

## ❄️ 环境冻结前,明天需要你拍板的
1. **感知 dial 定值**:锥角/量程(replay 10m vs render 8m 要不要统一?)、p_miss、噪声斜率、
   fp_rate/cls_err 开不开;
2. **PERCEPT 默认何时翻 realistic**(翻了就必须立刻进 B桶重标定,headline 全部重挣);
3. 3D 交互编辑器 + realtime 窗口上机验收;
4. 静态遮挡(建筑挡 mover)要不要进 v1 冻结范围。

## ✅ CP9:全面审查(5 视角多 agent)→ 修复 12 项确认缺陷

**修复的硬伤**(全部带回归验证:ctest 25/25、crossers/dynamics/realistic 全过):
1. 🔴 `replay_core` 前端变量 `pf` 被 `pf, vf = quad.step(...)` 遮蔽 → `--dynamics` 必崩 → 改名 `percept_fe`;
2. 🔴 证书 -inf 假证守护补齐三胞胎(above_plane + traj_vs_sphere 也会 `{true,+inf}`);
3. 🔴 感知关联门 1.2m 罩不住 dt=0.3 快目标(8m/s=2.4m/tick)→ 加出生门 gate+10·dt(未 ready 航迹);
4. 🔴 `run_scenario_3d` DT 硬编 0.1(真 0.3)→ **--live/mp4 之前快 3 倍** → 从 replay_core import;VideoWriter 改懒建(尺寸从真帧取,--w/--h 不再产出空 mp4);
5. 🟠 render realistic:单瞥航迹 coast 幻影 keep-out(+8.6m)→ ready 门;MAN_COLLDBG 同 tick 双推进 → per-t_sim memo;EGO_MEM/TTL dial 接管 realistic;
6. 🟠 bspline 退化交点语义恢复(排除而非继承邻居,靠 step-3 空邻居守护免 UB)×2 份;
7. 🟠 grid-only 参数单跑 KeyError → grid[0] 兜底;2D 设计器 `{}` 键不可达 → unicode 先判;Scene3D 缺 movers/statics 键崩 → setdefault。

**接受不修**(有意决策,记录在案):kf_tracker n==2 误差律变化(B桶重标定是正解)、climb 认证门的 liveness 代价(6/27 团队决策,CRET-hold 是后续解)、2D 遮挡不看高度(v1 已知限制,冻结拍板)。

**⚠️ 数字勘误 → ✅ 已建多采样协议**:occlusion_reveal realistic 的 0.476m 是单次采样不可引用。
解决:`--resample N`(run_scenario 新参数,PERCEPT_SEED 换 N 个传感器随机流,报 min_clr 分位数)。
**可引用版本:12 次采样 → 12/12 reach、零碰撞、min_clr 中位 1.191 / p05 0.959 / 最差 0.800**
(gt 对照 1.436)——realistic 感知把中位净空压掉 ~17%,重尾拖到 0.80,fresh-track 结论成立且有分布背书。
以后所有 PERCEPT=realistic 的数字一律 --resample ≥12 出。

**清理债**(不阻塞,阶段③统一处理):realistic 模式下 GT trackers 白算、3D trail O(T²) 重画、Scene3D 全量重载、
2D/3D 编辑逻辑双份、env cfg/hist 保存双份、gt/realistic 双轨+kf_movers 孪生(接口化)、cert_verdict3 三胞胎、
scenario_designer3d 写死 METAURBAN_ROOT(应 env 化)。

## ✅ CP10(2026-07-03):编辑器 v3 + 资产目录 + 积木地图 + 启动器 + BaseDrone 计划

- **编辑器 v3(用户实测反馈驱动)**:Fusion360 相机全套(中键拖=平移、Shift+中键=orbit 转角/俯仰、
  滚轮=缩放,原生相机任务已停由编辑器自驱)、左键按住拖点、Ctrl+Z 撤销(60 步)、英文工具栏/HUD、
  屏幕像素基准选中半径(修"放大后拖拽不对")、播放到头钳停+Duration±5s 按钮、俯仰方向已反转。
- **417 官方道具目录导入**:`prop_catalog()` 按 metainfo(精确 scale/hshift/偏移+真实 l/w/h)加载,
  **视觉=证书碰撞**(选道具自动写 r/h);选中障碍后 q/e 换道具、Shift+q/e 换类别(29 类:长椅/垃圾桶/
  广告牌/树/护柱/建筑/交通牌…)。
- **block_str 积木地图**:schema `map.block_str`("O"=环岛,"CXO"=链),designer/preview/fly 全透传。
- **新场景**:`roundabout_rush`(环岛环流车=持续曲率 CV 误差压力源,reach 9.3s/2.65)、
  `props_alley`(真道具窄巷,reach 9.0s/1.32;教训:树冠 r=4.69m 会封走廊,视觉=碰撞的诚实代价)。
- **`./sando.sh` 启动器**:editor/fly/preview/eval/ui/test 六个子命令收编全部环境配方
  (含 .so 新鲜度守护);`./sando.sh test` 全绿(12 场景+全自测+ctest)。
- **楼顶碰撞探针(BaseDrone P0)**:static_world max z=1.50m、20m 水平射线 0/36 命中——
  **MetaUrban 建筑无闭合 3D 碰撞,roofline 以上物理真空**(现有 headline 不受影响:证书走自建体素场)。
  完整平台计划(P1 Bullet BaseDrone→P2 碰撞闭合→P3 传感器+RL gym)见 `BASEDRONE_PLAN.md`。

## 明天优先给你看的三样
1. `CHECKPOINTS.md`(本文件)+ `out/scenario_runs/BASELINE_2026-07-02.md`
2. `out/scenario_previews/` 里的 3D 帧(crowd_dense / climb_trap / vehicle_spawn_accel)
3. 上机试交互:`env DISPLAY=:1 PYTHONPATH=/media/boxuan/Data2/projects/metaurban ~/miniconda3/envs/metaurban/bin/python scenario_designer3d.py --seed 3`

---
*本文件由 Claude 维护;每完成一个 CP 追加一节。memory 同步在
`~/.claude/projects/.../memory/sando-core-scenario-workbench-2026-07.md`。*

## ❄️✅ CP16(2026-07-03):环境冻结 + B桶执行完毕(核心)

**冻结配置**:abr=正式管线(--d435i --fov_range 10 标配)、PERCEPT 默认=realistic(gt 为显式对照)、
安全起飞区(不在路面/车辆8m泡/出生窗3m净空)、--multistart 多起点、--block 复杂地图。生成器 v1 冻结。

**B桶核心产出**(详见 `out/conformal/B_BUCKET_REPORT.md`):
1. **CRITICAL#1 重标定完成**——方法升级为"部署在环收割"(残差采自 realistic 前端自身,间歇/coast/
   关联全含):29,419 残差/290 episodes。episode-sup ε=0.05 行人管 q=0.277+3.56Δ → **运营不可用**,
   pooled-vs-sup 差距首次定量化 = 组合定理(阶段⑤)的数据背书。
2. **age 故事反转**:fresh-hole 消失(age2-3 覆盖 0.998),新洞=age13+ 转角误差(0.909),
   归一化 baseline 救不了 → 原 age-conditional 贡献死亡确认;turn/maneuver-conditional 是新方向。
3. δ_track 0.45→0.473(合法分位落值);calib_realistic **不自动晋升**(显式决策,防冻死)。
4. 未完:headline 文档对账、held-out 0/120 重跑、abr 正式基准表(下一 session)。

## ✅ CP17(2026-07-03):bench 批量 runner + contested 修复 + YOLO 延时
- `bench_run.py`:一键 6 场景 × 3 臂(ours_real/ours_gt/native_gt)× N 重采样 → BENCH_TABLE.md;
  公平对比=gt vs gt(native 无感知模型,realistic 只刻画 ours,表头明示)。
- contested 换 anchor 重试(旧回退致 3/6 无判别力)→ 4/6 有判别,rush_s0 强信号(1.68 vs 0.62)。
- YOLO(weights/yolo{11s,26s}_f10_e30):3 类 {human,animal,mechanical} 恰配感知分类;
  RTX4070 延时 6.7-8.7ms = tick 的 7-9%(delta 参数可吃实测值);⚠️iso 帧检出稀疏,
  接入前须在 abr FPV 帧测 recall。集成三档:cls_err 测量 → 类别注入 → bbox+深度 3D 检测器。

## ✅ CP18(2026-07-03):Benchmark P1 修毕(锥统一 + SANDO 第三臂 + suite 统计)
- FOV_R 14→10(冻结值,headless gt/native 门与传感器声明一致——旧值看得比声明远);
- bench_run 四臂:+sando_gt(MIT-ACL 原版,GUROBI,LD_LIBRARY_PATH=~/gurobi1103/linux64/lib);
- suite 汇总:碰撞率/近失率(<0.5m)Wilson 95%CI + clr 中位 bootstrap CI + 场景级配对(ours−native);
- 首个四臂表:36 ep/臂全零碰撞(CI 上界 0.096——太宽,P2 放量收窄);配对 +0.22m,win/tie/loss 2/3/1;
  sando 臂 clr 中位 6.40(大绕行保守风格,速度代价见逐场景行)。
- 差距清单:out/scenario_runs/BENCHMARK_GAPS.md(P2 放量+过滤+显著性 → P3 abr 跑批 → P4 感知对等+RTA率)。

## ⚙️ CP19(2026-07-03):难度校准攻坚——两个结构性发现
- hard 模式(packed 42 mover + 多 anchor 时序对齐 + 车辆对齐 + 遮挡板)+ `tune_difficulty.py`
  (闭环:实飞测到达时刻→反推修 spawn_t→复验间距,迭代)。
- **发现①"ours 自卫效应"**:以 min_clr(ours) 为调谐目标压不下去——它看见就绕(s0 振荡 1.5-2m
  =安全层在工作)。难度应以"遭遇压力"或 min_clr(native) 为目标,分离体现在弱臂被挤压。
- **发现②几何锁死**:s1/s2 三臂恒等(3.50/5.10)=走廊附近根本没有 mover 路径贴近(最近的是
  平行 3.5m 的散步者),时序怎么调都无遭遇 → 需要**遭遇几何过滤器**(纯静态检查:走廊线段与
  任一 mover 路径最小距 >1m 即废弃重生成,不用飞)。
- 有效分离样本:s0(ours 1.54 vs native 1.23 + ours 慢 0.9s=换余量);手工压力场景
  (occlusion_reveal/vehicle_spawn_accel/climb_trap)判别力天然强于随机街景 → 正式 suite 应
  混合"手工压力库 + 过滤后的生成街景"。
- 下一步(P2 修订):①遭遇几何过滤器进 populate ②tune 目标改 native ③手工库并入 bench
  ④然后才放量 50 场景。

## ✅ CP20(2026-07-03):差异打出来了 + 一个统计自查
- P2 修订版落地:遭遇几何过滤器(修了顶点采样漏交叉点的 bug→零 WARN)、tune 目标改 native
  (弱臂定义遭遇)、混合 suite(11 手工压力 + 6 生成街景 + 3 packed 硬场景 = 20 scenes,
  手工编队豁免间距门——人墙是设计不是失真)。
- **首个有分离度的四臂表**:ours 0/120 碰撞 vs native 12/120、sando 6/120;climb_trap 直接
  杀死 native(6/6 碰撞 min_clr −0.19,ours 认证爬升 0 碰撞、代价 +5.7s);occlusion_reveal
  native 0.75 vs ours 1.44;配对 +0.53m,win/tie/loss 10/8/2。**手工压力库是判别力主力**
  (生成街景 packed 仍平手——大家都躲得开)。
- **⚠️ 统计自查(发表前必须诚实)**:gt 臂检测噪声 rng 写死 → 6 次重采样=同一 run 复制 6 遍,
  n=120 实际 n=20(表里 clr 六次全同暴露)。已修(rng 吃 PERCEPT_SEED),重跑中——
  显著性结论以重跑表为准。

## ✅ CP21(2026-07-03):修正后正式四臂表——分离显著
真独立重采样后(场景内 clr 有方差=修复生效):
- **ours(gt) 0/120 碰撞 [0,3.1%] vs native 12/120 [5.8,16.7%] —— CI 不相交,碰撞率差异显著**;
  近失 0 vs 26(21.7%);配对 clr +0.66m,win/tie/loss 11/8/1。
- ours(realistic) 4/120 [1.3,8.3%]:诚实感知的安全代价被定量(4 次碰撞全是感知间歇/关联导致
  ——组合定理/per-age 的靶子);sando 6/120 且 climb_trap 0/6 到达(17.1s 绕死)=安全换 liveness。
- climb_trap 一幕:native 6/6 撞(−0.19m)、ours 0 撞(worst 1.23)、sando 不到达——三种哲学一图。
- 统计残留 caveat:场景内 6 重采样共享场景(聚类),论文级需场景层 bootstrap(cluster-robust);
  P3(abr 传感器诚实版跑批)与 P4(RTA 失效率指标)仍在清单。

## 🎯 CP22(2026-07-03):ours 破防——失效包络测绘完成
攻击阶梯(noightmare 套件 8 场景:合围/转弯伏击/高速盲侧/双向夹击/峡谷死锁/蜂群/奔跑合围/5m/s窄峡谷):
- ours(gt, 点质量):**攻不动**(时序调谐收敛后 0 碰撞,floor~1.2m)——cert+0.3s replan+evade 满状态防御;
- ours(gt+真动力学):0 碰撞,worst 0.59(pincer)——纯跟踪误差不足以击穿;
- ours(realistic):0 碰撞 worst 0.31;退化传感器(7m/15%漏检)出现**首个 DNF**(fast_canyon 7/8,liveness 裂缝);
- **三重叠加(realistic+动力学+退化传感)= 8/80 碰撞(10%)+13 近失**:
  canyon_deadlock 4/10(−0.51m)、fast_canyon 3/10、dual_blindside 1/10。
**失效模式**:静态遮挡(峡谷墙)×迟现对向者×动力学过冲×无逃逸余量的复合——正是 B桶重标定
说"诚实管子须更大"的物理演示:部署 q 用的旧标定,在诚实条件+对抗几何下漏 10%。
证书故事闭环:破防条件=标定假设被违反的条件,组合定理/重标定=补洞的路。
基准对照:同条件 native 在 fast_canyon 8/8 全灭(−0.21)。

## 🏁 CP23(2026-07-03):全量基准定版——1344 episodes,六臂矩阵
`out/scenario_runs/BENCH_FULL_2026-07-03.md`(28 场景=11手工+6街景+3packed+8噩梦,×6臂×8重采样):
| arm | 碰撞率[95%CI] | 近失率 | clr中位 |
| ours_gt | **0/224 [0,1.7%]** | 0 | 1.60 |
| ours_gt_dyn | 1/224 | 5 | 2.21 |
| ours_real | 4/224 | 10 | 1.62 |
| ours_real_dyn | 7/224 [1.5,6.3%] | 21(9.4%) | 2.13 |
| native | 28/224 [8.8,17.5%] | 69(31%) | 0.75 |
| sando | 10/224 | 18 | 3.41 |
- **风险分解干净**:0→+dyn 1→+real 4→+双重 7:感知贡献≈动力学 4 倍,近似可加。
- ours vs native CI 完全不相交;配对 +0.73m,win/tie/loss 18/9/1。
- **新失效模式(props_alley 5/8,−1.64m)**:evade 逃离 crosser 时**扎进被遮挡的树冠**——
  evade 是无认证运动且不查静态;遮挡(树/广告牌)让静态在 realistic 下也可能未被跟踪。
  修补方向:evade 也过静态 clear 检查 / 记忆场保底。climb_trap 1/8 擦碰(−0.01)。
- 残留 caveat:props_alley 单场景贡献 5/7 碰撞(聚类);论文级需场景层 bootstrap。

## ⚖️ CP24(2026-07-03):真部署标准确立 + evade 撞静态修复(修正版)
**用户定标:没有先验地图,地图是建图获得的,一切以 real 部署为标准。**
- 我第一版修复(_MapStatic 全知静态)是 GT 泄漏,已撤回;
- 正解=**在线建图语义**(perception.py):静态观测=地图知识,**见过才存在、存在即不忘**
  (static 类航迹豁免 TTL、coast 冻结不膨胀);mover 航迹照常衰减;
- safety_layer:mapped static 管 v_eff=0(记住的树不会动,生长管封死走廊);
- **props_alley 三重叠加:5/8 撞(−1.64)→ 1/10(−0.34)**;reach 2/10=退化传感下窄巷的
  诚实难度(树冠 keep-out 5.2m 只留 2m 走廊),liveness 边界非泄漏,开放项;
- ⚠️ 连带:render 的 EGO_STATIC_MAP=1(360°GT 静态图)违反真部署标准,待改为建图路径
  (=OCC_MEM v2 正主);全套 benchmark 数字需在此修复后重跑(静态语义变了)。

## 📋 "全部修复"进度与剩余(下 session 继续)
已修:evade 撞被遮挡静态(真部署版)。
剩余队列:①SLIP 重证窗口 ②per-class tube 接线 ③warm-start 护栏 ④DNF seed 调查(verdict3)
⑤headline 文档对账 ⑥held-out 0/120 重跑 ⑦abr 跑批(P3) ⑧RTA 失效率+基线感知对等(P4)
⑨场景层 bootstrap ⑩vehicle 标定饥饿 ⑪PROTOCOL.md ⑫EGO_STATIC_MAP→建图化(上条连带)
⑬全 bench 重跑(静态语义已变)⑭YOLO FPV recall ⑮清理债/上游 issue/isaac 拍板。

## ✅ CP25(2026-07-03):速度扫描 4-8 m/s(BENCH_SPEED_SWEEP.md,1680ep)
- **ours_gt:全速度段 0/560 碰撞**——证书随速度稳健(8m/s 时 0.75s 承诺=6m vs 10m 量程仍不破);
  clr 中位随速度升(1.63→1.83,门更紧绕更宽),用时 7.2→5.4s。
- **ours_real_dyn:7/560(1.25%),无速度趋势**(1/1/4/1/0)——4-8m/s 内部署风险不随速度增长
  (曝险时间↓抵消反应窗↓),clr 中位 2.58→3.04。
- native:18/24/22/15/14,峰值在 5m/s(21%)后回落——**⚠️ 时序混杂**:contested 对齐按
  v_nom=2.4 定时,高速无人机提前通过=躲过伏击,高速行=更容易的场景。做纯净速度曲线需
  per-speed 重调时序(tune_difficulty 逐速度跑)。分离结论不受影响(每档 ours≪native)。

## 🏁 CP26(2026-07-03):「全部修复」冲刺——12/15 清完
**代码修复(全带验证)**:
1. per-class conformal 管接线(审计死代码复活:PERCLASS_CONF 按 d_safe 查,回退保底);
2. SLIP 陈旧样条窗口错位:mover 回溯 c0−v·(t_ego/s) + 证 [0, t_ego+s·TAU](sound,未来窗精确覆盖);
3. warm-start:bridge 恒 1 谎言修成诚实(默认多项式 init=行为不变),EGO_WARMSTART=1 显式 opt-in,
   capi 加空 local_data_ 回落护栏,ego_capi.so 重编;
4. occ_remember v2(密度硬帽 OCC_CAP=2e4 + 无条件 TTL=30s)+ OCC_MEM 默认 ON(真部署建图);
5. RTA 失效率计数器(replay 结果 rta={certified_ticks, violations});
6. 场景层 bootstrap CI 入 bench 汇总(治 props_alley 聚类);
7. 车辆标定饥饿治愈:veh_cal_0..4 场景包 → vehicle eps=0.05 q=0.345/v_eff=3.2/cov0.998(36649 残差/240ep)。
**重跑与调查**:
8. held-out 诚实版:**ours 1/120 vs native 17/120**(clr 1.40 vs 0.49)——旧「0/120」正式作废
   (GT 感知产物),新 headline 是这个;
9. DNF 调查结案(props_alley DNF):kind 流健康+RTA violations=0 → 非证书死锁,是退化传感下
   慢进度超时(liveness/anti-thrash,阶段⑤);
10. 文档:conformal-results 增编(旧数字全部时效声明)、PROTOCOL.md(评测协议 v1)、
    ISSUE_UPSTREAM.md(bspline UB 上游 issue 草稿);
11. YOLO FPV 有界测试:旧低清 AB 帧检出 5/12 帧——接入门槛=今晚 abr 高清帧重测;
12. abr 过夜批已挂(6 场景 ours/native,out/abr_batch.log)。
**未竟(明示)**:基线感知对等(P4 剩余,协议已声明 gt-vs-gt 边界)、T-pose/ORCA/斑马线(外观)、
CI 基建、isaac 拍板、全 bench 重跑结果待读(后台)。

## 🏁 CP27(2026-07-03):修复后全量基准(BENCH_FULL_POSTFIX.md,1344ep)——修复红利落袋
| arm | 修复前 → 修复后 |
| ours_real | 4/224 → **0/224**(诚实感知点质量:碰撞清零) |
| ours_real_dyn | 7/224 → **3/224**(近失 21→13;残余全在 props_alley 3/8) |
| ours_gt / gt_dyn | 0 / 1(不变) | sando 10→8 | native 28→28(对照稳定) |
- 场景层 bootstrap CI 上线(native scene-boot [0.024,0.254] vs per-ep [0.088,0.175]=聚类真实存在);
- props_alley 仍是部署真实态最后堡垒(reach 3/8):树冠 keep-out+退化传感的 liveness/安全权衡,
  归阶段⑤(anti-thrash/CRET-hold);native 在此 8/8 全灭(−2.26)。

## 🏆 CP28(2026-07-04):组合定理落地+晋升+验收全绿(阶段⑤里程碑 1)
- phase5_compose.py:episode 绑定域上确界定理,**节奏感知分支(W=DT+δ=0.4s)把 ε=0.05 管从
  naive sup 3.18m 压到 1.02m(3 倍)**——核心论证:每 DT 重认证 ⇒ 碰撞只能发生在最近认证后
  ≤W 内 ⇒ sup 只跑短水平行。绑定域单刀无效(D_bind≥量程)诚实记录。
- docs/composition-theorem.md:定理+三支记账(认证ε/陈旧按频/evade 无保证按 RTA 报)+诚实边界
  (held-out ep 覆盖 0.908 vs 0.95 差 2σ)。
- **晋升**:calib.json ← (q₀=0.632, v_eff=0.967)(备份 calib_pre_theorem_backup.json)。
- **验收(BENCH_THEOREM.md,38 场景×6 臂×8 采样)**:
  ours_real_dyn **3→0/304**(近失 13→2)、gt_dyn 1→0、配对 21/16/1→**28/10/0**(零败),
  clr 中位 1.80→2.45;**代价:reach −4.3~4.6%**(定理的价格标签,明码)。
  ours_real 出 1 撞(0.3%≪ε=0.05,在保证内;或属 evade 支)。
- 当前 headline(可写摘要):**"定理规定的管子下,部署真实态 0/304 碰撞、2 近失,
  P(碰|认证)≤0.05 有证明,代价 4.6% 到达率。"**

## ✅ CP29(2026-07-04):完善清单 1-8 冲刺
1.render 回归:abr 冒烟过(修复后全链),abr 全批重渲后台中;2.放量复验(111816残差/580ep)
→暴露 episode 聚类泄漏→**切分单元改 scenario→覆盖 0.890→0.987✓**;3.三分切分(shape/cal/test);
4.stale 计数器已装(maneuver 路径每 tick 重规划=陈旧支结构性为空;slip 路径待一发验证);
5.abr 数字表待批完提取;6.per-class 管:ped≈全局 2.21m、vehicle ε_c=0.025 饥饿(等权分配不划算,
需曝险加权);7.**native 感知对等**接线(前端管 native,云=航迹)+native_real_dyn 臂入 bench;
8.related-work-matrix.md(两 novelty 边界防御)。
**诚实链(核心产出)**:定理管 1.02m(双重使用)→1.57m(三分)→**2.23m(scenario 可交换,
覆盖 0.987)**——每一步诚实化都涨价,链条本身就是论文的 methodology 卖点。
已晋升 scenario-honest calib(per-class 择优);验收 bench v2(7 臂)后台,读数看 reach 代价。

## 🏆 CP30(2026-07-04):定理 v2(scenario-honest)验收——全臂零碰撞零近失
BENCH_THEOREM_v2.md(38 场景×7 臂×8 采样):
- **ours 全四臂 0 碰撞 + 0 近失(1216 episodes)**,clr 中位 3.5-4.0;配对 31/7/0 零败 +1.81m;
- **native_real_dyn(感知对等新臂)14/304**——比 native_gt(28)少:真动力学到达更晚错过时序遭遇
  (时序按点质量 native 调的,诚实混杂已注明)+建图航迹云更稳;对等对比 ours 14:0 完胜;
- **价格标签**:reach ours_gt 280→254(83.6%)、real_dyn 249→238(78.3%);
- **安全-liveness 帕累托三点成型**(论文图):旧管 0.37m(3撞/96% reach)→定理 v1 1.02m
  (0撞2近失/88%)→定理 v2 2.23m(0撞0近失/84%)——ε 扫描可补全整条曲线。
摘要句 v2:"scenario 级可交换 ε=0.05 管下,认证层 1216 episodes 零碰撞零近失
(覆盖实证 0.987),代价 reach 84%;同条件感知对等 native 碰撞 4.6%。"

## 🔬 CP31(2026-07-04):极限压测尸检——三个真极限找到,三刀修复,seed0 击毙
**压测**:诚实管×4-8m/s×per-speed 重调(1520ep)→154 失败(138 DNF/11 近失/5 撞),
其中 4 撞带 RTA violations=1(认证 tick 上的碰撞=潜在定理反例)→逐案尸检。
**极限地图(算法的边界,论文§讨论)**:
① **大脚印障碍的感知原子性**:树冠 r=4.7m 以"中心射线"判可见 → 中心出锥/被挡=整棵隐形,
   冠沿贴脸也看不见 → 修:表面点判距 + 表面射线遮挡 + **轮廓 5 点弧采样**(对深度相机诚实);
② **出生延时 vs 接近速度**(crossers v5 案):航迹 ready 需 2 detections,漏检抽签拖到太晚
   → 表面判距修复后该案痊愈;临界速度可解析:(v_d+v_m)·(t_birth+t_stop) vs 有效量程;
③ **高曲率机动盲入**(seed0 案,三修不动的真凶):锥跟速度方向,急绕时传感器看别处、
   身体切进没看过的空域(15 tick 零检测)→ 修:**yaw-to-path**(锥跟上一 tick 指令方向,
   真机标准做法)→ 四种子全零撞。evade 期碰撞(v7 案)=无保证支,结构性,记账不修。
**定理免罪确认**:4 起"认证碰撞"全部是"绑定障碍未被跟踪"= FOV 覆盖前提被违反,
不是管子/定理错;RTA 计数器全部正确报警(仪表工作)。
**待办**:yaw-to-path 是全局感知变化 → 全 bench 需复验(后台);压测第二三波(退化传感
+14m/s 干道车)待 bench 复验后打。

## 🔄 CP32(2026-07-04):管线纪律实战——感知一动,标定重挣
- yaw-to-path 复验(BENCH_THEOREM_v3_y2p.md):reach 238→247 涨,但 real_dyn 冒 5 撞
  ——**旧标定×新感知=条件律漂移复发**,bench 当场报警(体系在自我纠错);
- 重收割(137014 残差/580ep,新感知)→ 定理 v3:**q0=1.054/v_eff=0.996,管 2.23→1.45m**
  ——感知改良(轮廓可见+yaw-to-path)让残差真变小,诚实管随之收缩:**看得清,管就细**;
- 已晋升;终验 v4 后台(BENCH_THEOREM_v4_final.md 待读)。
- **教训入册**:凡动 perception.py/锥朝向/可见性,必须走 recal→compose→promote→bench 全链,
  bench 碰撞尖峰就是漂移警报器。

## 🏁 CP33(2026-07-04):终验 v4——帕累托双运营点定版(论文 final numbers)
BENCH_THEOREM_v4_final.md(38×7×8):新感知(轮廓+yaw-to-path)×新标定(1.45m 管):
- reach 回升:gt 254→272(89.5%)、real 259→276(90.8%)、real_dyn 238→250(82.2%);
- real_dyn 4/304 撞(1.3%)——**在 ε=5% 保证之内**(定理承诺≤5%,交付 1.3%);gt_dyn 1/304;
- 配对 29/9/0 零败 +1.28m;native_real_dyn 18/304(对等对比 ours 4:18)。
**帕累托双运营点(论文就写这两行)**:
  A(保守):2.23m 管 → 0/304 撞、0 近失、reach 78-84%
  B(均衡):1.45m 管(感知改良兑现)→ 4/304=1.3%≤ε、reach 82-91%
两点都带同一条定理保证;选哪个是运营策略,不是安全性之争。
待办:4 撞逐案尸检(下 session 第一件事,沿用 CP31 方法);速度/复杂度 roast 第二三波。

## 🤖 CP34(2026-07-05 凌晨):RL 端到端臂——PPO 基线 + 证书盾组合(通宵自主)
- `drone_nav_env.py`:Gym 环境=同一冻结世界(同 realistic 前端观测/同 DT/同 38 场景/同记账),
  策略与 ours_real 同等视力(不喂 GT);env 极快(600k 步 2.1 分钟)。
- **基线 PPO(600k)**:22/304=7.2% 撞、reach 92.8%、clr 2.97——教科书"学习型快而无保证",
  恰落 native_real_dyn(5.9%)与 native_gt(9.2%)之间。
- **`shield.py` 证书盾**(今晚主菜):PPO 动作过同一 conformal 管(calib.json q0/v_eff)+
  同节奏语义(每 tick 重检),不安全投影到 16 方向×2 速安全集(fastest-safe),兜底刹车=
  evade 支如实计数。**ppo_shield:9/304=3.0%≤ε、reach 96.7%(反超裸 PPO!投影引导优于撞停)、
  无认证暴露 1.0%**——"学习给速度、定理给保证"的组合论点有了数字。
- 剩余 9 撞与极限地图一致(未跟踪/出生延时+刹车支)。
- v2 训练中:盾内训练(Alshiekh'18 思路,干预惩罚教策略提出可认证动作)+ 帧堆叠×3 + 3M 步。
- 文献基座(论文 RL 段引用):Safety Gym/Lagrangian-PPO(Ray'19)、CPO(Achiam'17)、
  shielded RL(Alshiekh'18)、potential shaping(Ng'99)、SB3。
