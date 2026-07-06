# 端到端组合定理(阶段⑤,2026-07-03/04)

## 一句话
把「单点预测误差的 conformal 保证」和「确定性 Bernstein 证书」合成一条可写进摘要的话:
**在可交换条件下,认证飞行的单集碰撞概率 ≤ ε**——并且给出的管子是运营可用的(不是 naive sup 的 3.2m)。

## 设定
- 部署管线每 DT 重新认证:对已承诺 B-spline 和每个被跟踪 mover 的预测圆柱(半径含管
  T(Δ)=q₀+v_eff·Δ)运行连续时间证书;全部通过才飞,否则 evade(无认证,单独记账)。
- 残差从**部署管线自身**收割(realistic 前端:锥/遮挡/漏检/NN 关联/coast 全含):
  每个 ready 航迹、每个水平 Δ∈{0.10..0.85},e = ‖GT 未来 − 预测‖,并记录离机距离 dd。
- 可交换单元 = **episode**(冻结的生成器 + 传感器定律;calibration 与 test 按 episode 切分)。

## 定理(episode 绑定域上确界)
固定管形 T(Δ)=q₀+v_eff·Δ(v_eff 由 pooled 90% 包络给出,**合法性完全由 q₀ 承担**)。定义每
episode 的分数
    S_ep = sup{ e_i − v_eff·Δ_i : dd_i ≤ D_bind(cls_i, W), Δ_i ≤ W }
q₀ = calibration episodes 上 S 的 (1−ε) split-conformal 分位。则对新的可交换 episode:
    P( 所有绑定行都被管覆盖 ) ≥ 1 − ε。
由证书的确定性部分:认证 tick 后 W 秒内发生碰撞 ⇒ 存在绑定行越管
(距离 > D_bind = (v_drone + v_cls)·W + R_keepout 的 mover 在窗内物理上够不着),于是
    **P( 全程任意时刻碰撞 | 全程认证飞行, 可交换 ) ≤ ε。**

## 关键杠杆:重认证节奏(cadence)
naive 全局 sup 在 ε=0.05 要 q₀=2.35(管@0.85s=3.18m,冻死无人机)。但正常运行时每 DT 就重新
认证:碰撞永远落在「最近一次认证后 ≤ W = DT+δ ≈ 0.4s」内 → sup 只需跑 Δ≤0.4 的行 + 相应收紧
的 D_bind:
| 分支 | q₀(ε=0.05) | 管@窗口 | held-out episode 覆盖 |
|---|---|---|---|
| naive 全局 sup | 2.354 | 3.18m @0.85s | 0.936 |
| 绑定域 sup | 2.354 | 3.18m(D_bind≥感知量程,不剪) | 0.936 |
| **节奏感知 W=0.4** | **0.632** | **1.02m @0.4s** | 0.908 |
**3 倍紧缩,运营可用。** 部署时以 (q₀, v_eff) 喂证书对全 [0,TAU] 窗生效:Δ≤W 段合法,
Δ>W 段过保守(sound,由下一次重认证接管)。

## 三支记账(全事件分解,防"证了一半"的假保证)
1. **认证-新鲜**(每 tick 重规划成功):上面的 ε 保证;
2. **陈旧样条**(重规划失败仍飞旧样条):频率可测(render 的 t_ego>DT 时段;SLIP 窗口修复已
   保证此分支的证书窗对齐),此分支需全 TAU 窗管——ε 分摊按测得频率;headless 每 tick 重规划,
   此分支为空;
3. **evade**(无认证逃逸):**无保证**,按 RTA 暴露单独报告(replay 已内置
   rta={certified_ticks, violations};BENCH 里 evade tick 占比即暴露量)。

## 诚实边界(写死)
- held-out 覆盖 0.908 vs 目标 0.95(98 eps,≈2σ 低):可交换性近似(veh-heavy 场景加采)
  + 有限样本;放量重验在跑;
- 分布内保证:换生成器/传感器定律要重标定(冻结协议 PROTOCOL.md);
- ID-swap/multiplicity 由无歧义关联的残差律承担一部分,极端 swap 属感知层失效,不在 ε 内;
- evade 分支无保证是**结构性**的(能证的都不 evade)——评测显示 evade 占比与碰撞集中在同一
  批退化场景(props_alley),改进归 anti-thrash/CRET-hold。

## 复现
```bash
cd sando-core/metaurban
python3 b_bucket_recalibrate.py     # 收割(45354 残差/410 eps,含 dd)
python3 phase5_compose.py           # 定理实例化 → out/conformal/compose_theorem.json
# 晋升即 calib.json(备份 calib_pre_theorem_backup.json);验收= bench_run 全套
```
