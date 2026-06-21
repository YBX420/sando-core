# SANDO × MetaUrban — 两侧剥离 (Planner 算法侧 ↔ Benchmark/集成侧)

> 目的:两个 workstream 并行不互踩。**算法侧**(C++ 规划核心)与 **benchmark/集成侧**(MetaUrban 喂数据 + 渲染 + 评测)
> 通过一个**固定接口契约**解耦。下面把已诊断的问题按归属拆开,并标注**只改本侧、不碰对侧**的边界。
> 诊断证据均为 `file:line`(读源码得到,非记忆)。

---

## 0. 接口契约(THE BOUNDARY —— 两侧唯一的耦合点)

唯一耦合 = `isaac/sando_cpp_bridge.py`(ctypes → `cpp/capi/sando_capi.so`)。两侧只能通过它交互:

**集成 → 规划器(每拍喂入)**
- `update_state(pos, vel, accel, yaw)` —— 无人机真值状态
- `update_occupancy_map_ptr(cloud Nx3)` —— 静态占据点云(**当前只在巡航 z 采样,见 C 节**)
- `add_traj(DynTraj{id, bbox, traj_x/y/z(t), vel, label_set})` —— 动态障碍 + conformal 标签
- `set_terminal_goal(pos)`
- 参数:`Parameters`(`drone_bbox`, `drone_radius`, `v_max`, d_safe, STC… 见 `metaurban_sando.yaml`)

**规划器 → 集成(每拍读出)**
- `replan() / get_next_goal() -> (pos, vel, accel)` —— 设定点
- `get_global_path()`, `get_corridor()`, `get_drone_status()`, `get_obst_class_codes()`

> **剥离规则**:① 算法侧只改 `cpp/include/sando_cpp/*.hpp` + 重编 `.so`,不碰 `metaurban/*.py`。
> ② 集成侧只改 `metaurban/*.py`(feed/render/eval),不碰 `cpp/`。
> ③ 任何需要**改接口**的(如要把无人机当实体传机体/姿态),必须在本节先约定再各自实现。
> ④ **共享测试床 = 集成侧的 `eval_batch.py` / `eval_parallel.py`**(headless 成功率/碰撞/卡死),算法侧改完用它回归。
> ⑤ 算法侧另有 `cpp/ ctest` 18-20 golden,改完必须全绿(已验证当前 20/20)。

---

## A. 算法侧问题(C++ 核心 —— 交给做新算法的 Claude)

### A1. 无人机被当**质点**,连半径都没进局部优化器(对标 EGO-Planner 的核心差距之一)
- `hgp_manager.hpp:84` `drone_radius = max(drone_bbox)*0.5` —— 机体压成**外接球**,无 bbox/椭球/姿态。
- `drone_bbox`(`types.hpp:634`)**只在全局 heat-A\* 用**;局部 MINCO 完全忽略机体:`local_opt_hardalm.hpp:237`
  的 margin `R = obstacle_radius + d_safe + q_conformal + eps` **无机体项**;`reach_avoid.hpp:17` 是点查询。
- 软墙机体膨胀 `inflate_walls_by_body` **默认 false**(`types.hpp:537`) → 默认对墙=纯质点。
- **无 SE(3)/姿态**:`RobotState`(`types.hpp:468`)只有 pos/vel/accel/jerk/yaw(标量),无法侧身钻缝。
- 影响:扁平 1.1m 四轴被当 0.55m 胖球 → 既过保守(绕太宽/显胆小)、又对垂直体积无概念。
- **方向**:EGO-Planner 默认 C-space 机体膨胀;若要"当实体"需 ellipsoid/SE(3)(超出 EGO-Planner)。最低成本=把
  机体半径真正接进局部 ALM/软项(各向异性更好:扁盘用 z 薄、xy 宽)。

### A2. 局部轨迹被全局路径**硬牵引**(= 你说的"严格 follow 建图 path";对标 EGO-Planner 的最大结构差距)
- `plan_minco.hpp:696-697`:局部 MINCO 内部路点被锁在全局引导路点的 **SFC 管道**里,注释:
  "*the optimiser **cannot drift off-guide** into a pocket*"。STC 还给每个 seed 路点 carve cuboid(`:874`)。
- **EGO-Planner 相反**:全局路径只做拓扑引导 + 生成 `{p,v}` 排斥锚点,局部 B-spline/MINCO **自由变形**抄近道。
- 影响:轨迹贴栅格折线 → 僵硬、不自然、找不到平滑捷径;也是 benchmark 里"贴着障碍/静物擦碰"的诱因之一。
- **方向**:把 SFC/STC 从"硬管道"放松为"软引导"(大半径或仅作初值),让碰撞梯度主导局部形状。

### A3. yaw 与轨迹**解耦、事后算**(最大的"不自然"直接来源)
- `planner.hpp:1474` `yaw=atan2(vel.y,vel.x)` + `:1494` 一阶滤波 + 限速;
  `:1447` plan<5 点时 **yaw 冻结**;`:1441` replan 连续失败时 **原地自旋**;`:1468` 低速 `atan2` 抖动。
- 影响:急转时航向滞后/跳变;状态机切换(TRAVELING→depleted→SPIN)非平滑。
- **方向**:yaw 进 MINCO 联合优化(或至少 C2 平滑的 yaw 轨迹),去掉 freeze/spin 状态机。

### A4. 重规划**接缝不连续** + recovery 直接插轨迹
- `planner.hpp:1223` 新设定点从 `t=dc` 起(非 0)→ 与承诺状态 A 有 C1/C2 跳变;`:1050` 记录了 A_time vs 实际
  执行态的"systematic optimistic error"(预测滞后 ~0.2m/0.15s)。
- `planner.hpp:825 / 1250` recovery_yield / **recovery_climb**(向上逃逸)直接构造新 MinjerkTraj 写入 deque,
  **不检查与上一段的速度/加速度连续性** → 突然刹车/垂直爬升。
- 影响:每次 replan/恢复都可能 stutter。
- **方向**:接缝强制 C2 续接(用当前执行态而非 A 作初值);recovery 也走平滑续接。

---

## B. 集成 / Benchmark 侧问题(MetaUrban 喂数据 + 渲染 + 评测 —— 我的域)

### ✅ B1 已修复(2026-06-19)——见 E 节结果:碰撞 11/67/58% → **全部 0%**

### B1. 树 / 景观能"随意穿梭"(`feed()` 严重低估几何)【已修复】
- 树**确实**在 `eng.get_objects()` 里(`sidewalk_manager.py:715/1053/1394/1690` 用 `spawn_object` 生成),但
  `render_3d_video.py`/`eval_batch.py` 的 `feed()`:
  1. 当**软静障**喂;
  2. tree 对象多半不暴露真实 `WIDTH/LENGTH/HEIGHT` → `obj_size()` 退化成**默认 0.6×0.6×1.6** 小盒(树实际 3–8m 高、冠 3–6m 宽);
  3. occupancy **只在巡航 z=1.5 采点** → 树干弱软点,**树冠/枝(z 2–6m)对规划器不存在**;
  4. A4 的**向上逃逸**爬到 3–4m 正好进入**零碰撞树冠层** → 穿枝。
- **本侧修法**:用每个 static 对象的真实包围盒(从 panda3d `getTightBounds`/asset 元数据)+ **多层 z occupancy**
  (按对象真实高度铺点),或把高大静物作为竖直柱体喂。**不需要碰算法。**

### B2. 静障喂法 = 软 occupancy → benchmark 里 50% 擦碰(主导失败模式)
- `feed()` 把 static 走 `update_occupancy`(软场),movers 走硬 DynTraj。benchmark(`eval_parallel`,48 集三档)显示:
  **movers 避得极好(仅 3 次碰撞),失败 100% 是静物擦碰,随密度 11%→67%**。
- **本侧可做**:加 `--static_hard` 开关把街道静物也喂成硬 DynTraj(小 d_safe),给算法侧一个可对照的难度档;
  但"软静障 vs 硬静障"哪个对,需和算法侧约定(A2 放松路径牵引后软场可能就够)。

### B3. occupancy 单层 z 切片(同 B1 根因)—— 整套占据只在 cruise z,无垂直结构。

### B4(已完成,记录)benchmark/提速基建
- 渲染 1.9×(MSAA16→4 + 单次 renderFrame + DummyObservation);评测 ~5× 并行(`eval_parallel.py`)+ 去 240 射线 lidar。
- 真实性:地图拓扑 `--map`、密度三档 `--stratify`、动物类注入、路径长度随机、按模型 bbox 记碰撞。

---

## C. 为什么对标 EGO-Planner "缺组件"(一句话归纳)
- **同**:MINCO/min-jerk、平滑软排斥场(`local_opt_terms.hpp:159` 的 `(d_safe-d)^3`,和 EGO 一致)、时间分配(段时长是优化变量,EGO v1 反而弱)。
- **SANDO 多**:硬-ALM 证书层 + conformal 标签(安全层卖点)、STC 时空走廊。
- **SANDO 缺/做反**:① 局部对全局路径**自由变形**(EGO 有,SANDO 硬牵引 A2);② **机体感知**默认接入(EGO 默认 C-space,SANDO 局部丢机体 A1);③ **平滑 yaw**(A3);④ **接缝 C2 连续**(A4)。

---

## D. 两侧独立验证回路
- **算法侧**:改 `cpp/*.hpp` → 重编 `.so`(`g++ -O2 -shared -fPIC -std=c++17 -o capi/sando_capi.so capi/sando_capi.cpp -Iinclude -Ithird_party/eigen -Ithird_party`)→ `ctest`(golden 全绿)→ 跑共享 `eval_parallel.py` 看成功率/碰撞/卡死有没有变好。
- **集成侧**:改 `metaurban/*.py` → 直接跑 `eval_parallel.py` / `render_3d_video.py --serve`,不碰 `.so`。
- **共享指标**:`out/eval_*.json` 的 `reached / collided(by class) / stuck`,三档密度分层。

---

## E. 进展更新 (2026-06-19) —— 集成侧 B1/B3 已修复,把问题干净踢给算法侧

**做了什么(集成侧,不碰算法)**:`feed()` 改成对每个静物用真实 WLH 铺**完整 3D occupancy**(footprint W×L × 真实高度,封顶到 z_max+0.5=6.5m;每场景预计算一次 503 物体≈90K 体素,每帧按 SENSE_R 向量化裁剪——非每帧热点)。原来只在 z=1.5 单层铺 0.6m 小块,树冠对全局规划器完全空白 → 穿枝。`_dt_into` 加签名缓存(静物只编译一次)。

**三档密度 benchmark(每档 8 集)before → after**:

| 档 | reach% (前→后) | **碰撞% (前→后)** | stuck% (前→后) |
|---|---|---|---|
| sparse | 94 → 100 | 11 → **0** | 6 → 0 |
| med | 83 → 62 | 67 → **0** | 17 → 38 |
| dense | 92 → 25 | 58 → **0** | 8 → 75 |

**结论 / 球踢给算法侧**:
- **碰撞 = 集成侧几何不全造成的,补全后归零**(24 集 0 碰撞)。benchmark 现在测真避障。
- **卡死成为唯一短板,且随密度爆炸**(dense 75%)。根因 = A1(质点/外接胖球过保守)+ A2(严格贴全局栅格路径,`cannot drift off-guide`)+ freeze:实心密集环境里找不到可行缝隙就停。
- **下一步全在算法侧(A1/A2/A3/A4)**:放松全局路径牵引为软引导、把机体感知接进局部 ALM(各向异性更好)、freeze→更聪明恢复。集成侧 occupancy/几何已完备,可作为算法回归的固定测试床。
