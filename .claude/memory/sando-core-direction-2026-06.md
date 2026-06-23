---
name: sando-core-direction-2026-06
description: "2026-06-22 三工作流+SANDO先验核实后的研究定位:博弈论/可达集/extend-planner当卖点被抢;活下来的headline=解耦planner无关Bernstein判官+max-speed↔conformal统一拨盘(RA-L级,非flagship)。SANDO arXiv:2604.07599是头号baseline+命门。权威细节=docs/direction-2026-06.md。"
metadata:
  type: project
---

**2026-06-22:三个深度工作流(planner选型+baseline / 博弈论-可达新颖度 / C1-C4形式化)+ 联网核实 SANDO 先验后的研究定位。** 权威细节 = `docs/direction-2026-06.md`。承接 go-around(绕而非HOLD)讨论,见 [[sando-core-status-2026-06]] [[sando-py-bernstein-deficit-cert]] [[sando-py-ego-port]]。

**命门 — SANDO `arXiv:2604.07599`(Kondo/Tordesillas/How,MIT-ACL,2026-04,带硬件)= 本repo `sando_native/` vendored 的那个 planner。** 已做 max-speed 可达膨胀 + 时空走廊 + 连续时间无碰撞保证 → **「max-speed可达+连续时间+动态避障」被它占了**。但它 solver-内嵌(MIQP/Gurobi、通用AABB、无conformal)。**你的 delta(已读全文确认)= 解耦外部判官,认证未修改/黑盒/学习型 planner 的已承诺轨迹 + 免license + Bernstein区间sound + conformal层 + planner无关(一核跑MINCO+EGO)。SANDO=头号baseline。投稿前精读它坐实解耦delta。**

**活下来的 headline(三工作流独立收敛)= 解耦、planner无关、免license的精确连续时间Bernstein判官 + 把【max-speed硬可达↔conformal概率】合成进一个R(t)的「保证拨盘」。RA-L级composition/packaging创新,非flagship first。**

**防雷(全被先验占,别claim):** 博弈论当贡献(Fisac/Hu/FaSTrack/iLQGames)、max-speed硬可达planner(SANDO/FaSTrack)、first conformal+reachability(2602.03799/2304.00432/CROWS/Muthali)、planner无关「端到端确定性安全」(判官只给per-window分离,递归可行性是planner的活)。

**技术核心 C1(成立+sound):** max-speed球 `ρ(τ)=r0+v_max·τ`,`ρ²` 是t的二次式 → 证书标量`R²`升成逐段deg-2n区间多项式即可证;硬保证coverage1、`v_max=0`退回常数-R。**两个必修bug**:① τ锚到`t_obs`+延迟δ(否则少充气不sound);② 亏量`b=ρ²−S`必须**先组再de Casteljau细分**(现seg_worst把R²固定,只对常数成立)。**统一拨盘**:`v_eff=v_max`→C1硬floor;`v_eff=conformal分位`→C2(紧3-5×,实际飞这);`v_eff=(1-λ)v_max`→C3(不sound,丢)。**铁律:确定性端点要静态中心,漂移只能概率吸收或用`v_max+‖v_obs‖`。** 各向同性球挡飞越→改竖直圆柱(z受重力界)恢复overflight。

**planner无关 = 保持(用户「general+best-on-mine」):** 通用臂=agnostic判官(EGO顺滑当证明);最佳臂=planner原生吃tube(MINCO原生已有/EGO anchor注入)。extend-EGO=可选flag化ablation(降HOLD率,标non-agnostic)。良性「改EGO」=喂时变tube当障碍(solver不动,仍agnostic)。

**落地现实:** 认证go-around现在两planner都没端到端(MINCO能绕但证书OFF;EGO认证但HOLD);HOLD内禀→需认证刹停兜底;`q_conformal=0`占位→P(碰)≤ε未实现;EGO的`Mb/6`非区间=soundness seam。下一步ROI:①改证书(deg-2拨盘)②建认证go-around闭环+刹停③建conformal层(预测器先CV/CA-KF测0.75s残差,见`conformal/kf_predictor_experiment.py`)。
