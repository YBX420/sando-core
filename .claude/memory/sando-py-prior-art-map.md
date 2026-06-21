---
name: sando-py-prior-art-map
description: 2026-06 实搜撞车检测画出的"什么已被占/什么是死路"地图,别再重复猎这些方向
metadata:
  type: project
---

2026-06-19 经 5 轮实搜(deep-research + Wave3/4/6/7/8 刺客)确认的 prior-art 边界。**别再把这些当新点猎:**

- **拓扑枚举+并行类内优化** = T-MPC++(arXiv 2401.06021, de Groot T-RO 2024,动态行人 quadcopter)/ Mavrogiannis-Knepper space-time braids / TRUST-Planner(2508.14610, UTF-MINCO)/ Fast-Planner / EGO-Swarm。
- **"安全独立于枚举完备性"防火墙** = 教科书 RTA/Simplex(Sha 2001)/ shielding(Alshiekh-Bloem 2018);形式核 A∩B⊆B。
- **对偶价格断点=结构切换=Pareto** = mp-NLP critical-region(Bemporad/Tøndel)+ Eaves-Zangwill/Geoffrion + Dual CC-SSP(2302.13115)。
- **风险/conformal 预算当可交易资源** = IRA(Ono-Williams)/ Fb-CP-IRA(2510.16376)/ H-PRAP(2507.11920)。
- **conformal × 时空/SIPP × 行人** = 2511.18170(Time-aware MP w/ CP);**反应行人 interactive 安全** = 2511.10586(Lindemann/Pappas,quadcopter,自称 first valid interactive)→ 反应域已被占 + 可交换性破,我们必须限 non-reactive。
- **连续时间风险证书** = Jasour(moment/SOS,IJRR 2023, 2305.17291/2106.05489,需障碍矩);**离散步 conformal-MPC** = Lindemann 2210.10254/Dixit 2212.00278/IA-CP 2502.06221。
- **e-process/test-martingale 当运行时安全监视** = 2506.16416 / WATCH 2505.04608 / 2602.04364 / Vovk conformal test martingale。
- **O(M)二阶/Hvp** = Pearlmutter R-op(1994)/ IPOPT inertia(Wächter-Biegler);**GCS 全局** = Marcucci/Tedrake + ST-GCS 2503.00583/GCS* 2407.08848。
- **"追平 EGO-Planner"** = 0 novelty(EGO 是 2020 baseline)。

**唯一未被占的合取** = distribution-free conformal × 连续时间精确 Bernstein 凸包(无 per-step union)= S3(见 [[sando-py-bernstein-deficit-cert]])。详见 [[sando-py-newalgo-verdict-2026-06]]。
