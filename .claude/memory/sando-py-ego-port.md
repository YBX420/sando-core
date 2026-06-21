---
name: sando-py-ego-port
description: 2026-06-20 把S3认证安全层移植到EGO-Planner上(planner-无关核);EgoSafe整合EGO丝滑+我们的层判官,headless闭环验证过
metadata:
  type: project
---

**2026-06-20:把"我们的安全包围"移植到 EGO-Planner —— planner-无关认证安全层在第二个 planner 上的活体证明(主论文 §3 核心)。** 用户原话纠正:"移植"=把我们的层套到 EGO 上让 EGO 又丝滑又认证,不是把 EGO 搬进来。

**已 vendor + de-ROS'd 的 EGO**:`ego/`(include/src/capi),经典 ZJU EGO(uniform cubic B-spline + bspline_optimizer + planner_manager + lbfgs + dyn_a_star + grid_map)。`ros_shim.h` stub 掉 ROS。`ego/capi/ego_capi.cpp` 镜像 sando capi(create/set_gridmap/set_params/update_cloud/**replan→reboundReplan**/traj_eval→evaluateDeBoorT)。build:`g++ -O2 -shared -fPIC -std=c++17 -Wno-narrowing -w -o capi/ego_capi.so capi/ego_capi.cpp src/*.cpp -I include -I ../cpp/include -I ../cpp/third_party/eigen -I ../cpp/third_party`(EGO 有 1e10→int narrowing,需 -Wno-narrowing)。python bridge:`metaurban/ego_bridge.py`(EGOPlanner)。

**关键技术(为何几乎免费)**:EGO 输出**均匀 cubic B-spline = 分段多项式**;每段转 Bezier(M₃·控制点列,EGO 控制点是 **3×Ncols**、`evaluateDeBoorT(t)` t∈[0,dur])→ 我们的连续时间 conformal-deficit 证书原样跑(cubic→deg6,像 quintic MINCO→deg10)。执行 EGO 轨迹=白拿丝滑,套层不替换。

**planner-无关核**(在 `bernstein_cert.hpp`):`certify_segments_vs_sphere`(任意次分段 Bernstein 控制点)+ g_elevate/g_square/g_subdiv/g_left_subcurve/g_seg_worst + `minco_to_segments`。MINCO(deg5) 与 EGO(cubic) 喂同一个核。交叉验证:同一 MINCO 轨迹,deg-5 专用路径 vs 通用核**判定+margin 完全一致**。见 [[sando-py-bernstein-deficit-cert]]。

**整合产物**:`ego_capi.ego_certify`(B-spline→Bezier→核)+ `ego_bridge.certify()` + `metaurban/ego_safety_smoketest.py`(5/5 sound,拒静态/head-on 碰撞、认证安全、连续时间正确处理 mover 时序)+ **`metaurban/ego_safe.py` 的 `EgoSafe`**(EGO 规划 + 我们的层当 RTA 判官:每次 replan 认证 B-spline vs 各动障 conformal 管,不过就 hold)。headless 闭环 demo:3 横穿人,replans=25/certified=22/held=3,**到达 + 执行最小净空 1.28m 全程不撞 + 丝滑**。提交在 feat/bernstein-gate(a5e80d1 等)。

**还没做**:接进 `render_3d_video.py --ego`(可视化,需显示器,现 EGO 裸跑没挂层)+ 接进 `eval_batch.py`(headless benchmark 出 EGO+层 vs SANDO 成功率/碰撞/卡死数字)。EGO 喂动障点云就会避障(self-test 直穿是退化场景)。层=判官非矫正见 [[sando-py-layer-judge-not-corrector]]。
