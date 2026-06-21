---
name: sando-py-layer-judge-not-corrector
description: 我们的安全层现在是判官(裁决+拒绝)非线路矫正;最小修正QP设计有但没建(用户2026-06-20说先不做);5个default-OFF flag清单
metadata:
  type: project
---

**2026-06-20 澄清(用户问"层是判官还是有线路修正"):我们的安全层现在是【判官】,不是线路矫正。**

- **做的事**:S3 证书([[sando-py-bernstein-deficit-cert]])对一条已提交轨迹(MINCO 或 EGO B-spline)+ 障碍/conformal 管 → 输出**裁决**(认证 P(撞)≤ε 通过/不通过 + margin)。只说"安不安全",不产出"该怎么飞"。
- **不通过谁修正**:由**现有 planner 兜底**接手——gatekeeper 保持上一条已认证承诺轨迹 / recovery(yield/刹车/爬升) / 下一拍重规划 / (EgoSafe 里)hold。粗粒度,不是层算的。
- **最小修正 QP(真正的"线路修正")= 设计里有、还没建。** 对照 CLAUDE.md `RTA = 监视器独立节点 + 最小修正 QP + 垂直爬升 backup`:我们有**监视器/判官**那半,**最小修正 QP 那半没做**。**用户 2026-06-20 明确:先不做线路修正,先整合 EGO+套件。** 平滑刹车/CRET 是 recovery 级修正(失败兜底),不是证书的一部分、也不是最小修正 QP。

**这套代码当前形态(feat/bernstein-gate 分支,golden 24/24,全 default-OFF):**
- `minco_deficit_cert`:S3 连续时间 deficit 当 mover GATE(替双重 unsound 密采样门,见 [[sando-py-mover-gate-bug]])。
- `minco_recovery_smooth_brake`:recovery 平滑刹车(诚实发现:头对头急停 jerk 物理固有,平滑会前冲撞威胁→正确回退;只在前方安全时平滑。闭环测试 head-on 回退/侧面 2.5→0.008)。
- `minco_yaw_c2_smooth`(+`minco_yaw_accel_max`/`minco_yaw_lowspeed_lo`):jerk-限制 C2 yaw governor(OFF 1.0→ON 0.06)。
- 全透过 capi(types.hpp Parameters → planner.hpp opt → sando_capi `B(...)`/`D(...)`),YAML 可开;OFF=字节等价。
- **接缝 seam_bias 没做**(诚实降级):deque 本就 C2(A=plan[commit_ahead]),"飞的≠认证的"安全已被 `eps_track` 兜(gate R 加 epsilon_track),seam_bias 是低价值高风险 refinement。

要把层升级成"判官+矫正",下一步=建最小修正 QP(不过时在认证走廊内最小微调使重认证,MINCO/EGO 通用)。
