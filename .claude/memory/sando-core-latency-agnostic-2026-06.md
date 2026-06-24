---
name: sando-core-latency-agnostic-2026-06
description: "2026-06-24 安全层每层计算延时 profiling + planner-无关确认 + 竖直证书 76ms 性能 bug 的根因。每层(metaurban/latency.py monkey-patch 真实调用):KF 预测~0.03ms(预测层几乎免费)、EGO replan~1ms、横向证书 0.038ms,但**竖直证书 certify_above=76ms(隔离微基准确认,非抢核)= 头号瓶颈**,整 ours tick~250ms=300ms 控制周期的 81%。根因:bernstein_cert.hpp 的 g_seg_worst_deficit 只在 hull(上界)≤0 时早退;对'飞越'候选在 ground 高度(deficit=z_clear-p_z≈1.0 恒正、必然证不过),hull≈1.0>0 永不早退 → 死磕细分到 maxdepth=16=2^16 次。修法:加 lo_min(min 系数下界)>0 的早退(deficit 整体恒正→sup>0→必不过,直接返回)——sound(只在能证 sup>0 时早退)。planner-无关:同一 bernstein_cert.hpp 证 EGO(三次)和 MINCO(五次),只需轨迹的 Bézier 控制点接口;native SANDO replan~15ms(GUROBI)vs EGO~1ms。"
metadata:
  type: project
---

**2026-06-24:安全层计算延时 + planner-无关。** 用户问"每层计算延时"+"是不是能接别的 planner 不一定 EGO"。承接 [[sando-core-conformal-2026-06]]。

**planner-无关(确认):** 证书 `cpp/include/sando_cpp/bernstein_cert.hpp` 只需要**已承诺轨迹的(分段)Bézier 控制点**这一个接口。同一套数学**既证 EGO(三次 B-spline→Bézier)又证 MINCO(自研 min-jerk 五次,`minco_to_segments`)**。能接任何输出多项式轨迹的 planner(GCOPTER/min-snap/MPCC…)。"换 planner"只换运动生成器,证书+预测+门控不动。**用户计划:多接几个好复现的外部 planner 验证 agnostic(MINCO 跳过,因为是自研非外部 baseline);但"先不动手,先修速度(延时)"。**

**每层延时(`metaurban/latency.py`,monkey-patch EGOPlanner/MoverTracker 真实调用计时;注意测时有 3 个后台任务抢核→绝对值偏高):**
| 层 | mean | median | 说明 |
|---|---|---|---|
| KF 预测/mover | 0.03ms | 0.03 | **预测层几乎免费**——"知道未来"不花钱 |
| EGO replan | ~1ms | 0.29 | 规划器快(SANDO replan~15ms GUROBI,慢一量级) |
| occupancy 喂入 | 0.49ms | 0.47 | |
| 证书 横向 | 0.038ms | 0.05 | 快(隔离微基准 0.038ms) |
| **证书 竖直** | **76ms** | 76 | **隔离微基准确认=真 bug,头号瓶颈** |
| 整 ours tick | ~250ms | 256 | =300ms 控制周期 81%,满载下勉强实时 |

**竖直证书 76ms 根因(确诊):** `certify_segments_above_plane` → `g_seg_worst_deficit(b,0,maxdepth=16)` 找精确 sup。该函数只有 `hull=max(系数.hi)≤0` 的早退(已能证安全时)。但**飞越候选在巡航高度 z=1.5 < z_clear=2.5,deficit b=floor−p_z≈1.0 恒正、本就证不过**,hull≈1.0>0 → **永不早退 → 死磕细分到 2^16=65536 次**。横向快是因为清空轨迹 hull 很快掉≤0 早退。

**修法(sound):** g_seg_worst_deficit 加对称早退——`lo_min=min(系数.lo); if(lo_min>0) return lo_min;`。Bernstein 凸包性质:多项式值 ∈[min系数,max系数],所以 min系数.lo>0 ⟹ 整段>0 ⟹ sup>0 ⟹ certified=(sup≤0)=false,**直接返回不细分**。只在"能证 sup>0=必不过"时早退,sound(不会假证)。预期:竖直证书 76ms→μs 级,整 tick→几十 ms,余量 5-10x。改后必须 `cd cpp && cmake --build build && ctest` + 手编 ego_capi(`-Wno-narrowing`)。

**(测量被 overnight 任务污染,修后要重测干净数字。)** 相关:[[sando-core-conformal-2026-06]] [[sando-core-collision-geometry-2026-06]]。
