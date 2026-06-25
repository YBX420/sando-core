---
name: sando-core-planner-agnostic-plan
description: "2026-06-24 用户拍板的下一步计划:为证明安全层 planner-无关(headline 支撑臂),多接几个『好复现的外部 planner』当运动生成器(不只 EGO),每个都套同一套连续时间 Bernstein 证书+conformal keep-out+KF 预测门控。MINCO 跳过(自研、非外部 baseline)。顺序:先修计算速度瓶颈(竖直证书 76ms,见 latency 记忆),再动手接 planner。接入只需 planner 输出『已承诺轨迹的分段 Bézier 控制点』这一个接口。"
metadata:
  type: project
---

**2026-06-24 用户拍板(下一步,排在『先修速度』之后):** 既然证书 planner-无关已经在 EGO 上跑通,**多接几个『好复现的外部 planner』** 当运动生成器,每个都套同一套安全层(连续时间 Bernstein 证书 + conformal keep-out + KF 预测门控),用来**实证 planner-无关**这条支撑臂(不是 headline,但是通用性卖点)。

**约束/取舍:**
- **MINCO 跳过**(`sando_native`/`cpp` 里有自研 min-jerk 五次,但它是自研、不算外部 baseline,说服力弱)。
- **找『好复现』的**(复线 = 能 vendored / 容易复现的开源 planner)。候选方向(待定,选好复现+有代表性的):GCOPTER/MINCO-family、Fast-Planner、min-snap(Mellinger)、kinodynamic A*/MPCC 等——挑 1-2 个**有现成开源、轨迹是多项式/样条、容易去-ROS vendored** 的。
- **接入接口极小**:planner 只需输出**已承诺轨迹的分段 Bézier 控制点**(EGO 是 B-spline→Bézier;MINCO 是五次→Bézier,见 `minco_to_segments`)。证书/预测/门控代码不动。
- **顺序铁律:先修计算速度**(竖直证书 `certify_above` 76ms 瓶颈 + 整 tick 81% 预算,根因+修法见 [[sando-core-latency-agnostic-2026-06]]),**速度修好再动手接 planner**——用户明确"这个先不动手,先修速度"。

**为什么有价值:** 证明"同一套认证安全层能套任意现成 planner、把它变成认证安全+会预测+敢更快"——这是 [[sando-core-conformal-2026-06]] headline(认证最快+最安全绕行)的通用性背书。

---

**2026-06-25 进展:嫁接地基已建 + 10 个 graft spec 已出(workflow wha2thze3/portable-planner-survey)。**

**地基(已建+验证,这是所有嫁接的共用底座):**
- `cpp/capi/cert_capi.so`(`cert_capi.cpp`):**通用 BSeg 证书**——`cert_horizontal_bseg`/`cert_above_bseg` 吃任意分段 Bézier 控制点(n_seg,deg,t0s,durs)+ 障碍,彻底脱离 EGO。手编:`g++ -O2 -shared -fPIC -std=c++17 -o cert_capi.so cert_capi.cpp -I ../include -I ../third_party/eigen`。
- `metaurban/cert_bridge.py`:ctypes `Certifier` + 适配器 `monomial_to_bseg`(power→Bernstein 基变换,min-snap/MINCO power 用)、`native_bezier_to_bseg`(TGK/Btraj 透传)。自检过:cubic Bézier vs 障碍,证书判定 = 暴力真值,sound 无假证。
- **关键洞察:10 个 planner 只用 5 种表示**(cubic B-spline、MINCO/power 五/七次、min-snap monomial、native Bézier、MINVO)→ 证书套这 5 种 = 套全 10 个。

**10 个 graft spec(`out/conformal/portable_planners.json` + `planner_graft_specs.json`,含 repo/license/适配器/工作量):**
执行顺序(易→难):**RapidQuadrocopterTrajectories(~3h,零依赖五次,纯 C++ 闭式)→ am_traj(~4h,header-only,但 GPL-3 别静态链)→ GCOPTER/MINCO(~6h,MIT,header-only)→ mav_trajectory_generation(~7h,Apache,vendor solveLinear 闭式核)→ MPL/Fast-Planner/EGO-v2/FASTER/TGK/Btraj(output-only-cert,只证它们 emit 的轨迹)**。
**GCOPTER 适配器精确坑(workflow 给的):** `Trajectory<5>` 的 `getCoeffMat()` 是 3×6 **降幂** monomial(col0=τ^5)、**实时 τ∈[0,dur]**。转 BSeg:① 反转列 `a_j=C.col(5-j)*dur^j`(降→升 + 实时→[0,1] 的 dur^j 重标);② 用现成 deg-5 power→Bernstein C2B(`minjerk_traj.hpp` 的 /60 下三角矩阵)。**别漏列反转(会证镜像多项式)和 dur^j(Bernstein 凸包错 dur 次幂)。** 适配器要读 D=traj 阶不能硬编 5(min-snap 是 Trajectory<7>)。

**2026-06-24/25 踩坑(记下):reliability 跑到 ep ~250 卡死 = EGOPlanner 内存泄漏。** run_replay 每 episode 新建 EGOPlanner,其 C++ grid(80×80×8 @0.2 = 640万体素 ~8MB)不释放(Python gc 在紧循环里不跑 __del__;gc.collect() 也救不回 C++ 侧)→ RSS 每 ep 涨 ~8MB、ep250 到 ~2GB 换页卡死。**修法(用户拍板"分集跑存csv"):`reliability_chunked.sh` 每 seed/chunk 一个 fresh 进程(退出即释放全部内存)+ `reliability.py --csv` 增量追加每 episode 行 → `reliability_full.csv`,任意规模都能完成(冲 30 万 episode 验 99.999%)。** 根因修法(备选):run_replay 复用单个 EGOPlanner(按 inflation 缓存)而非每次新建。

**下一个大目标(用户 2026-06-25):做完这批 planner 移植后 → 完善 MetaUrban**(仿真环境本身:场景/感知/真实度,具体待定)。

相关:[[sando-core-latency-agnostic-2026-06]] [[sando-core-win-ego-2026-06]] [[sando-core-conformal-2026-06]]。
