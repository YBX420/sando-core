---
name: sando-core-collision-geometry-2026-06
description: "2026-06-24 设计决策(覆盖更早的椭球提案):人的安全碰撞体 = 竖直圆柱,用「横向证 OR 竖直证」的整窗 disjunction 当证书。绕行=证 2-D 横向分离 dx²+dy²≥r²(忽略 z);飞越=证竖直 p_z≥z_clear;每人 OR(各自整 [0,TAU])、跨人 AND。理由=最紧最快(正中『飞得越快越好』),且这个 OR 有干净 sound 形式(跑两道 Bernstein 证、过一个即过,不是逐时刻分支)。关键 soundness 坑:around 必须用 2-D 横向证,不能用 3-D 球(球会把『低空靠近=插在人下半身』误判为过)。椭球被否(对站立的人 ~20-40% 保守)。Bernstein 连续时间核保留=创新点。证书 C++ 实现进行中。"
metadata:
  type: project
---

**2026-06-24 设计决策:人的安全碰撞体 = 竖直圆柱,证书 = 「横向 OR 竖直」整窗 disjunction。** 用户在 sphere / cylinder-disjunction / ellipsoid 三选一里**拍了圆柱(最紧最快)**,覆盖更早记的椭球提案。承接 [[sando-core-goaround-m1-2026-06]] 的 6/24 去-HOLD 预测驱动机动转向。**证书 + Bernstein 连续时间核保留——Bernstein 是创新点。**

**证书结构(每控制拍、每候选机动、对每个 mover):**
- **绕行 disjunct = 2-D 横向证**:证 `dx(t)²+dy(t)² ≥ ρ(t)²` 全 [0,TAU](ρ=r_cyl+d_safe,带 v_eff 时变 tube)。忽略 z。
- **飞越 disjunct = 竖直证**:证 `p_z(t) ≥ z_clear` 全 [0,TAU](z_clear=头顶+reach_pad+d_safe_v;带可选 v_eff_z floor,默认 0 因 KF 钉死 vz=az=0)。
- **每人 = 横向 OR 竖直**(两个 disjunct 各自是全窗充分证明 → OR sound);**跨人 AND**。爬升时横向 disjunct 罩着,过了 z_clear 后竖直 disjunct 接管。

**Why 圆柱(而不是球/椭球):**
1. **最紧 = 飞得最快最低**,正中用户目标②"对会动的几何体飞得越快越好"。
2. **想避的"非凸 OR"有干净 sound 形式**:不是逐时刻分支/猜近窗(那种 unsound,对抗 agent 确认),而是"**整窗证横向 OR 整窗证竖直,跑两道 Bernstein 证、过一个即过**"。两 disjunct 各自全 [0,TAU] 充分 → OR sound。
3. **椭球被否**:单个凸二次曲面贴不住又高又细的站立人,横向要宽 ~20-40% 或爬升要高一截(最小外接椭球:a=1.22r 但 c=1.73H,或 √2/√2)→ 比圆柱慢。椭球的优点(单个优雅移动二次曲面证书、可推广旋转/任意二次曲面)留作论文叙事的备选;工程取紧。

**关键 soundness 坑(必须守):**
- **around 必须用 2-D 横向证,不能用 3-D 球证书**。球对高圆柱 **unsound**:无人机飞到低空(z≈0)且横向近时,球因 z 方向距离大而判"过",但其实正插在人下半身圆柱里。横向证(`dx²+dy²≥r²`,忽略 z)才对。spec 原稿用球当 around 是这条的隐患,已纠正。
- 竖直证只验 `p_z≥z_clear` 全 [0,TAU](`g_left_subcurve` 裁到 t_hi),**绝不**猜一个"近人时间窗"只在窗内验。
- z_clear 必须是人**可达头顶的保证上界**(头高+姿态/手臂/跳 reach_pad+d_safe_v),不是瞬时检测头高。
- B-spline→Bezier 的 Mb/6 是裸 double,有 ULP 缝;竖直证传 bez_pad(外向取整)闭缝,横向证沿用现状(缝是 around 既有,先 disclose)。
- 改 C++ 必重编 `ego_capi.so` + 重核 `test_bernstein_cert` + golden(soundness 不变量 #1)。

**How to apply(实现路线,进行中):**
- C++(`bernstein_cert.hpp`):① `certify_segments_vs_sphere` 加 `n_axes`(默认 3=球,字节等价;2=横向圆柱);② 新增 `certify_segments_above_plane`(竖直证,~30 行,纯复用 g_elevate/g_left_subcurve/g_seg_worst_deficit)。
- capi(`ego_capi.cpp`):抽 `build_segs`,加 `ego_certify_horizontal`(n_axes=2)+ `ego_certify_above`;重编 .so。bridge 加 `certify_horizontal`/`certify_above`。
- python:`scenario.py`(可手编场景:start/goal/humans[pos,vel,r,height])+ `ego_maneuver.py`(无 HOLD 候选-锦标赛:直走/左绕/右绕/飞越/紧急爬升 偏置子目标 → VO 预排序 → 逐个 replan+证书(横 OR 竖)+取片 → 提交最快被证过的;紧急爬升=不冻兜底)。**EGO 给抬高 z 子目标会爬升已实测验证**(max_z 1.62→2.93 翻过墙顶),EGO 一行不改。
- 相关:[[sando-py-bernstein-deficit-cert]] [[sando-py-ego-port]] [[sando-core-direction-2026-06]] [[sando-core-goaround-m1-2026-06]]。

**2026-06-24 已落地并验证(M3):** 证书核 `certify_segments_vs_sphere` 加 `n_axes`(2=横向圆柱)+ 新增 `certify_segments_above_plane`(竖直),capi `ego_certify_horizontal`/`ego_certify_above`,bridge 同名方法,**ctest 28 cert 例 / 25 test 全过**(含"球误判过、2-D 横向证拒"那条 soundness 对照)。`metaurban/ego_maneuver.py`(无 HOLD 候选-锦标赛,fastest-safe)+ `scenario.py`(可手编场景)+ `ego_vs_native.py`(A/B)。结果:整面墙→飞越(max_z 2.85)、横穿/迎面→绕、零 HOLD/零 cert_miss、净空恒≥d_safe;**同等安全标距下 crossers 每个速度都比原生 EGO 快、head_on vmax=12 原生撞我们安全**。详见 [[sando-core-goaround-m1-2026-06]] M3 段。
