# 研究方向 + go-around 决策(2026-06-22,三工作流 + SANDO 先验核实后)

> 配套 `safety-layer-spec.md`(v2,工程现状)/`safety-layer-plan.md`(v2,工程路线)。本文记录 2026-06-22 三个深度工作流(planner 选型+baseline / 博弈论-可达性新颖度 / C1-C4 形式化推导)+ 联网核实 SANDO 先验后的**研究定位、go-around 方案、baseline 清单、下一步**。

## 0. 一句话裁决

> **⚠️ 2026-06-23 再聚焦(用户拍板,覆盖本节"planner 无关判官"的 framing):产品 = 认证的「最快+最安全绕行」(certified go-around),不是判官只 HOLD(HOLD 降兜底);只做 EGO(MINCO 暂搁置);用 KF(CA 模型)预测障碍未来轨迹→喂预测占据给 EGO 绕开未来→证书检预测移动球→过则飞;planner 无关降为支撑性质/通用臂,不是 headline。** M1 已跑通(`metaurban/ego_goaround.py`,见 §4-§5-§7 与 `.claude/memory/sando-core-goaround-m1-2026-06.md`):EGO 真绕行(y≈4.2),但 q_conformal=0 让预测误差吃 d_safe(净空 0.677<0.8)、喂整条扫掠短暂冻走廊(7 HOLD)→ §7 的 ROI 1-3 正是修这两点。

博弈论 / max-speed 可达集 / extend-planner **当卖点 = 被抢**。活下来的、三个工作流独立指向同一个的 headline = **解耦的、planner 无关、免 license 的精确连续时间 Bernstein 判官** + 把【max-speed 硬可达 ↔ conformal 概率】合成进一个 `R(t)` 的「保证拨盘」。**RA-L 量级的 composition/packaging 创新,不是 flagship first。**

## 1. ⚠️ SANDO 先验(命门,已核实)
`arXiv:2604.07599` **SANDO**(Kondo / Tordesillas / How,**MIT-ACL**,2026-04,带无人机硬件)——**就是本 repo `sando_native/` vendored 的那个 planner**。已做:max-speed 可达膨胀(`r_n=v_max·(n+1)·dt+ε`)+ 时空时间分层走廊 + 连续时间无碰撞保证(Bézier 凸包包含)+ 动态障碍。

→ **「max-speed 可达 + 连续时间保证 + 动态避障」被它占了。** 但它把安全**焊进自己的 MIQP**(solver-internal,要 Gurobi,通用 AABB 非行人,纯确定性无 conformal)。

**你的 delta(窄但真,已读全文确认)**:
- **解耦外部判官**,认证**未修改 / 黑盒 / 学习型** planner 的已承诺轨迹(SANDO 只能保证自家 MIQP 输出);
- **免 license**(区间算术 vs Gurobi);
- **Bernstein 亏量 + 外向区间** soundness(vs 走廊凸包);
- **+ conformal 概率层**;**行人**;**planner 无关**(一个核 `certify_segments_vs_sphere` 跑 MINCO + EGO)。

**SANDO = 头号 baseline**:同问题、同可达数学,但 solver 内嵌 + Gurobi;你 = 解耦 + 免费 + 能审计任意 planner。**投稿前精读 SANDO 全文坐实「解耦 vs solver-内嵌」这条 delta。**

## 2. 防雷:不能 claim 的(全被先验占)
- **博弈论当贡献** → Fisac/Bajcsy/Hu(intent-gating)、FaSTrack、iLQGames/ALGAMES、HJ reach-avoid games 占了。降级为「一个可选的离线 tube 生成器」。
- **「我们 plan against r0+v_max·t」/ max-speed 硬可达 planner** → SANDO、FaSTrack、reachable-inflation planners 占了。
- **「first conformal + reachability」** → Conformal Reachability(2602.03799)、MA Reachability Calibration w/ CP(2304.00432)、CROWS/Muthali/Lindemann 占了。
- **「planner 无关 DETERMINISTIC 端到端安全」= 假**:判官只给**已承诺路径的 per-window 分离**,不是端到端 invariance;递归可行性(刹停 standoff)是 planner/监督器的活。

## 3. 技术核心:C1 + 统一拨盘(形式化工作流)
**C1(赢):** max-speed 球 `ρ(τ)=r0+v_max·τ` → `ρ²` 是 t 的二次式 → 把证书标量 `R²` 升成**逐段 deg-2n 区间多项式**即可证。反向三角 + 逐时刻包含 → **对 v_max 内任意人类运动的硬保证(coverage 1,无概率,连对抗追击者都防,需 v_max≪v_drone)**;`v_max=0` 精确退回现有常数-R 证书。

**候选定理(per-window 确定性分离):** 人在 `t_obs` 被观测于 `B(c0, r_human⁺)`(`r_human⁺=r_human+e_perc`),a.e. 速度 `≤v_max`,感知→commit 延迟 `δ`;`r0=r_human⁺+r_body+d_safe+ε_track`,`ρ(t_traj)=r0+v_max·(t_traj+δ)`(deg-2)。若对亏量 `b=ρ²−S`(逐段 deg-2n,外向区间)做 de Casteljau 细分 + `t_hi` 左子曲线裁剪后 `max_k b_k.hi≤0`,则对**一切**与 `(c0,r_human⁺,v_max)` 相容的人类轨迹,无人机与人保持 `≥d_safe`,于 `[t_obs+δ, t_obs+δ+t_hi]`。

**两个必修 sound bug(实现/锚定,非数学):**
1. **时间锚定**:`τ` 必须 = `t−t_obs`(+ 延迟 `δ`);用轨迹起点会少充气 `~v_max·δ` → **不 sound**。
2. **细分**:现 `seg_worst` 把 `R²` 在 de Casteljau 中**固定**(只对常数成立);deg-2 的 `ρ²` 必须**先组 `b=ρ²−S` 再细分**(左子曲线裁剪也作用于 `b`)。

**统一拨盘(真正的贡献形状):** `ρ²=(r0+v_eff·t)²` 一个旋钮——
- `v_eff=v_max` → **C1**(硬 floor,coverage 1,校准不可信时的兜底);
- `v_eff=conformal 分位` → **C2**(1−ε,比 C1 紧 **3-5×**,**实际飞的工作点**,且 `q_conformal` 终于有原则的非零值);
- `v_eff=(1-λ)v_max` → **C3**(reciprocal,**几何不 sound,丢弃**:走着突然停的人会被错误认证;且 reactivity 破坏 conformal 的 exchangeability,与 C2 互斥)→ 只留作 negative result。
- **铁律**:确定性端点要求**静态中心**;漂移只能概率地(吸进 conformal Q)或用更大增长 `v_max+‖v_obs‖`——C2/C4 攻击各自独立重导出这条,正是 C3 漂移+收缩破的原因。

**新增改进**:各向同性球在 z 也膨胀 → 挡飞越;改**竖直圆柱**(z 受重力界、水平增长)恢复「从上方绕过地面行人」。

**证书改法(行级,形式化工作流给的):** `bernstein_cert.hpp` 里标量 `const Iv R2` → `std::vector<Iv>` 尺寸 2n+1;按 `(A=v_eff², B'=2r0·v_eff+2Aδ, C'=r0²+Bδ+Aδ²)` 用 `τ=t_traj+δ` 建 deg-2 三元组(断言 `tube_t0==seg.t0`),`g_elevate` 到 2n;**先 `b_k=iv_sub(Q_k,S_k)` 再 `g_subdiv`/`g_left_subcurve`**;`g_seg_worst` 改吃 `b`。`v_eff=0` 精确退回常数-R。加两条回归:变半径 + t_hi 裁窗。

## 4. planner 无关 + 「general + best-on-mine」
**保持 agnostic**:判官消费任意 planner 的已承诺曲线;人只暴露为时间参数化原语 `(c(t)=c0+v_obs·t, ρ(t)=r0+v_eff·t)`,任何 planner 经自己障碍接口吃进去。
- **通用臂** = agnostic 判官(EGO 顺滑,当 planner 无关证明)。
- **最佳臂** = planner 原生吃 tube(MINCO 原生 hard-ALM 已有;EGO 经 anchor 注入)。
- **extend-EGO(C4)= 可选 flag 化 ablation**,诚实 claim「焊进 EGO 在中低密度降 HOLD 率 X%,同等认证安全」,显式标 non-agnostic。**良性「改 EGO 适应动态障碍」= 喂时变 tube 当障碍(solver 不动)= 仍 agnostic;改优化器/实时博弈才是被否的。**

## 5. 落地现实
- **认证 go-around 现在两个 planner 都没端到端跑通**:MINCO 能绕但证书门默认 OFF(`run_demo.py` 从不认证);EGO 认证但只 HOLD(没喂 mover 给 replan + grid 无时间轴)。**不是翻 flag,是建「认证+滚动重规划」闭环。**
- **HOLD 删不掉**(认证后门控内禀)→ 配**认证刹停**兜底(刹停轨迹也是多项式,同证书能认),两个 planner 都要。
- **冻走廊**:各向同性球长 t_hi 会封死(v_max=1.5、t_hi=2s → ~3.6m 半径);只在短 t_hi(0.75s→~1.7m,可绕)+ 频繁重锚 + `v_max≪v_drone` 可行。C2(conformal)紧 3-5× 才是真抗冻。
- `q_conformal=0`(占位)→ 现在只有确定性几何 margin,**P(碰)≤ε 还没实现**。
- EGO 的 B-spline→Bezier `Mb/6` 用普通 double 非区间 → soundness seam,要修或披露。

## 6. safety-core baseline 清单
| baseline | 性质 | license / 可跑 |
|---|---|---|
| **SANDO** `2604.07599` | 可达走廊+连续时间,MIQP solver-内嵌 | 要 Gurobi;同 lab → **头号 baseline**,引论文数字 |
| **MADER / FASTER / DYNUS** (MIT-ACL) | 动态障碍 deconfliction,solver-内嵌 | 要 Gurobi → 引用/对纸面 |
| **FaSTrack** | planner 无关、博弈派生 tube | 最接近「agnostic tube」先验 |
| **Fisac/Bajcsy/Hu confidence-aware** | intent-gated 可达,概率 | 行人+无人机硬件 |
| **Conformal Reachability** `2602.03799` / **MA Reach Calib+CP** `2304.00432` / CROWS/Muthali | conformal+可达 | 占了组合 → 别 claim first |
| **EGO / EGO-Swarm,Fast-Planner** | 顺滑、无保证 | 开源能跑(EGO 已有)→ 顺滑 baseline |

可实跑开源(优先):EGO(已有)、conformal-MPC(acados,免费)、CBF-QP;Gurobi 那几个用论文数字对比。

## 7. 下一步(按 ROI)
1. **改证书**:标量 R²→deg-2n 区间多项式 + 亏量先于细分 + 修两 bug + 圆柱 z(§3 行级方案)。C1/C2 共用地基,~几天。
2. **建认证 go-around 闭环**(MINCO 原生 or EGO anchor 注入)+ **认证刹停兜底**;跑 10-seed A/B 出 go-around 率 / 残余 HOLD / 净空。
3. **建 conformal 层**(`q_conformal>0`,见 `safety-layer-spec.md §5`)——差异化锚 + 抗冻 3-5×;预测器先 CV/CA-KF 测 0.75s 残差(`conformal/kf_predictor_experiment.py`),不够再上学习。
4. **投稿前**:精读 SANDO 全文坐实解耦 delta。

**Sources:** SANDO `arXiv:2604.07599` · Conformal Reachability `arXiv:2602.03799` · MA Reachability Calibration `arXiv:2304.00432`

> **P54 闭环(2026-07-07 验尸终审)**:§3 两个 sound bug **均已修@405825b(2026-06-24)**。
> bug①(τ 锚 t_obs+δ):certify_segments_vs_sphere 按三元组 τ=t_traj+δ 落地(bernstein_cert.hpp L337-342/L373-377),数值 witness 验证翻转边界=r0+v_eff(t_hi+δ);δ≠0 回归已并入 ctest(test_delta_anchor)。遗留:MINCO 专用核无 δ 参数,该路径 δ 须折进 R(已搁置、默认 OFF)。
> bug②(亏量先组再细分):亏量 b=ρ²−S 于细分前装配、裁剪/细分均作用于 b(L379-387);ctest Block3 + 新增 test_deficit_order 直接覆盖(冻结序实现会当场挂掉:400 例中假证 42、shipped 0)。
