---
name: sando-py-bernstein-deficit-cert
description: 核心创新S3=连续时间conformal-Bernstein deficit证书(3个move)+ ~250行bernstein_cert.hpp落地;经8波验证的唯一真·新成果
metadata:
  type: project
---

> **2026-06-22 勘误**(见 [[sando-core-status-2026-06]]):① soundness 是 `std::nextafter` 外向取整,**非 fesetround**;② **`control_points_interval()` 没建**(证书内部用 `C2B_int` 自己重算区间控制点);③ ctest 现 **25/25**(下文 golden 20/20、24/24 已过时);④ 交叉验证只断言**判定(certified)一致**,margin 打印未断言。

**S3 = 唯一存活的真·新+sound 成果(2026-06-19,mostly-proven conf 0.83)。** 单条已提交 quintic 上认证 P(撞或误分 track i)≤ε,连续时间、分布无关。

**三个 move:**
1. **几何(精确,不采样)**:距离² `D(t)=R²−‖p(t)−c(t)‖²` 是 deg-10 多项式;写成 Bernstein 基,系数 `b_k` 是"非负、和=1"权重 → 多项式值落在 [min b_k, max b_k]。故 **max_k b_k≤0 ⟺ 整段连续时间安全**。11 个系数检查覆盖无穷时间区间。
2. **概率(分布无关 + 时间 union 预付)**:每 track 一个标量分数 = 整段 sup 归一化误差,对它做 split-conformal 标定;"分数≤分位"已蕴含"每个 t 误差≤tube" → sup 在标定前免费做掉时间 union。conformal 半径只经 R 进入。
3. **合取(新点)**:分布无关 conformal 半径塞进精确连续时间 Bernstein 壳。vs Jasour=分布无关(他要矩);vs Lindemann/Dixit=连续时间精确无 per-step union(他离散步)。

**scope(写死)**:non-reactive 域、marginal(跨标定抽取)、per-round(**不复利成 per-flight**)、commit<τ_trust=0.75s、Mondrian class×density、标定在 Isaac 机载 tracker 输出(**绝不 SDD**)。学习预测器:interpolate waypoint 为低次多项式 x̂_i + 选多项式 ρ_i + 标定真值-vs-插值残差 → 回 Case A,不限 CA。

**落地**:新 `bernstein_cert.hpp`(~200 行:Interval+running-error/fesetround、degree_elevate、Chu-Vandermonde 平方、deg-10 min/max-coeff、deficit_and_margin)+ `minjerk_traj.hpp` 加 `control_points_interval()`(~30 行,复用 C2B_matrix:18/control_points:197)。双后端:区间(机载 μs)+ GMP 有理(离线 ground-truth)。**float-sign soundness**:b̄_k≤0→真 b_k≤0。default-OFF 门保 golden 20/20。

它替掉双重 unsound 的 shipped mover 门(见 [[sando-py-mover-gate-bug]])。属主线见 [[sando-py-newalgo-verdict-2026-06]]。

**STATUS 2026-06-20:已实现 + 验证(不再是纸面)。** `bernstein_cert.hpp` 落地:deg-5 路径(certify_traj_vs_sphere)+ 自适应 de Casteljau 细分 + 区间外向取整 float-sign soundness;test_bernstein_cert 14 例对密采样零假阳性。**加了 planner-无关任意次核 `certify_segments_vs_sphere`**(MINCO deg5 + EGO cubic 共用,交叉验证一致)→ 用于把层移植到 EGO,见 [[sando-py-ego-port]]。已接成 default-OFF mover GATE(`minco_deficit_cert`),golden 24/24。当前是判官非矫正,见 [[sando-py-layer-judge-not-corrector]]。(GMP 双后端是设计,实测用区间;未单独建 GMP ground-truth,用密采样交叉验。)
