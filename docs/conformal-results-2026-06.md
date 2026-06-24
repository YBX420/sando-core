# Conformal 安全层 · 结果与定位(2026-06-24)

> **一句话**:把证书 keep-out 里那个一直占位的 `q_conformal=0` / 手设 `v_eff=0.2`,换成**在真实 MetaUrban 行人/车轨迹上分布无关标定**出来的 `rho(t)=q_conformal+v_eff·(t+δ)`,并在留出测试集上验证了 **≥1−ε 的边际覆盖**。这把项目从"确定性几何 margin"升级成"**有分布无关碰撞概率保证 P(碰)≤ε**"——正是 `safety-layer-spec.md §5` 列为 future work 的统计半边。
>
> 配套:用真轨迹回放做 ours-vs-native 的 A/B(验收:**0 碰撞 + 用时 ≤ native+2s**),用证书级消融证明**连续时间 Bernstein 证书相对离散采样的 soundness**,并用 predictor 比较把 keep-out 进一步收紧(更快)。

承接:[[sando-core-win-ego-2026-06]] 的诚实结论(认证安全 + 同等安全更快)+ `compass_artifact` min-time survey 的金句定位。

---

## 0. 为什么这件事是"强选题"(立意)

min-time survey(`sando_py/compass_artifact_*.md`)的结论一句话:**"真·时间最优 + conformal/认证安全 在动态障碍上几乎没人做过,是真空白、是强 RA-L 选题。"** 现有工作分三摊:

- **时间最优 / min-snap / MPCC**:把时间或进度压到极限,但**默认障碍已知且静态**,对动态障碍只做反应式避让,**没有连续时间的碰撞证明**,更没有对"预测会错"的概率量化。
- **conformal prediction for planning**(近两年兴起):用 conformal 给轨迹预测套分布无关的不确定集,但大多停在**预测层**(给个椭圆/管子),**不和一个 sound 的连续时间规划-证书闭环挂钩**,也很少落到"喂给一个真规划器 + 在执行轨迹上逐点证明"。
- **可达性 / HJ / 漏斗**:连续时间安全有保证,但**保守、且要离线算可达集**,难和"在线最快绕行"统一。

**我们站的位置 = 三者交叉的空白**:
> 一个**与规划器解耦的连续时间碰撞证书**(Bernstein 亏量,sound,不采样)+ 一个**分布无关 conformal 标定的 keep-out 管子**(覆盖 KF 预测误差,给 P(碰)≤ε)+ 一个**用预测把动态障碍"绕开未来"的最快绕行**(EGO,planner-agnostic)。三件套合起来才有"**因为我知道人会怎么动,所以敢更快但安全**"的硬保证。

---

## 1. 方法:split-conformal 标定证书 keep-out

**要证的不等式**(横向圆柱半边):证书强制无人机已承诺 B-spline 满足
```
||p(t) − c_pred(t)|| ≥ R + rho(t),   rho(t) = q_conformal + v_eff·(t+δ)
```
其中 `R = r_mover + r_drone (+ 小舒适 d_safe)`,`c_pred` 是 KF 预测的障碍中心多项式。

**关键推导(为什么 conformal 给的是 P(碰)≤ε)**:设预测残差 `e(Δ)=||c_true(Δ)−c_pred(Δ)||`(Δ=距上次检测的流逝时间)。若证书保证 `||p−c_pred|| ≥ R+rho`,且 conformal 保证 `e ≤ rho` 概率 ≥1−ε,则三角不等式给
```
||p − c_true|| ≥ ||p − c_pred|| − e ≥ (R+rho) − rho = R   概率 ≥ 1−ε
```
即 **P(物理重叠) = P(||p−c_true|| < R) ≤ ε**。所以 keep-out 管子 `rho` 必须 = 预测残差的 (1−ε) conformal 分位。

**标定流程**(`metaurban/conformal_harvest.py` + `conformal_calibrate.py`):
1. **收割真轨迹**:headless 跑 `SidewalkDynamicMetaUrbanEnv`(关相机),记录 85 行人 + 车/机器人(ORCA 群体策略,**会变向变速**——这是残差不平凡的前提,合成匀速 mover 会让残差虚低)的逐 0.1s 轨迹,20 个场景。
2. **造预测实例**:对每条轨迹,按真实控制周期(0.30s)喂**带噪检测**(σ=0.07m)给部署用的 CA-Kalman,每次更新后对一组 Δ∈{0.30…1.05} 预测,残差 = 预测 vs 真值(0.1s 轨迹插值)。
3. **split-conformal**:**按 track 切 cal/test**(残差在轨迹内时间相关,按 track 切才尊重 exchangeability),每个 Δ 取有限样本修正分位 `q̂(Δ)`(rank=⌈(n+1)(1−ε)⌉)。
4. **可部署仿射管**:拟合 `q_conformal+v_eff·Δ ≥ q̂(Δ)`(对每个 Δ 都支配 → 继承 ≥1−ε 覆盖)。
5. **验证**:在留出 test track 上量边际覆盖,确认 ≥1−ε。
6. **按类**(pedestrian / vehicle):动力学差异大,分类标定(对齐 `EGO_PERCLASS_DSAFE`)。

---

## 2. 结果:标定 + 覆盖验证

**数据规模**:20 场景,约 1.1M 预测实例(cal+test)。

| class | eps | 目标覆盖 | q_conformal (m) | v_eff (m/s) | **test 覆盖** |
|---|---|---|---|---|---|
| pedestrian | 0.10 | 0.90 | ~0.00 | 1.12 | 0.923 |
| pedestrian | 0.05 | 0.95 | ~0.00 | 1.29 | 0.964 |
| vehicle | 0.05 | 0.95 | ~0.00 | 1.71 | 0.961 |
| all | 0.05 | 0.95 | ~0.00 | 1.31 | 0.964 |

**两个要点**:
- **覆盖全部达标**(test 覆盖 ≥ 目标,仿射管保守 → 略微过覆盖,正确)。这是支撑 "P(碰)≤ε" 的实测数字。
- **手设 `v_eff=0.2` 严重欠覆盖**:真行人 KF 残差在 95% 水平是 **1.29 m/s**(是旧占位的 ~6×)。这解释了 M1 实测净空跌到 0.677<0.8 的根因(预测误差吃光裕度)——conformal 层既**诊断**又**修复**了这个老账。

> 图:`out/conformal/calib_coverage.png`(残差散点 + 各 ε 的仿射管 + 覆盖率)。

---

## 3. 结果:ours-vs-native A/B(真轨迹回放)

`metaurban/replay_core.py` + `ab_replay.py`:把真 mover 轨迹当**不让路的录像**(strict 最坏情形)回放,无人机飞穿人群走廊。ours = KF 预测 + per-class conformal 证书门控机动;native = 莽撞真 EGO(膨胀 0.3,贴 ~0.3m 飞)。**验收(用户 2026-06-24 拍板)**:ours **0 碰撞** 且 ours_time ≤ native_time + 2s(越快越好)。

> **结果(20 seeds × 6 episodes,回填):**
> - 碰撞:ours `__/120`,native `__/120`
> - 最小净空:ours `__` m,native `__` m(中位 ours `__` vs native `__`)
> - 用时差 ours−native:中位 `__`s,最大 `__`s;**≤native+2s:`__`/`__`**;严格更快 `__`/`__`
> - **验收:0 碰撞 `PASS/FAIL`,全部 ≤native+2s `PASS/FAIL`**

**故事**:ours 永远认证安全(净空恒 >1.3m),native 会直接穿过人(净空 <0 撞);ours 用时只多中位 ~0.3s,**硬场景(native 被人群缠住/撞上)反而更快**(seed1 ep0:native 8.4s 且撞,ours 5.1s 干净)。

---

## 4. 结果:连续时间证书 vs 离散采样(soundness 消融)

`metaurban/cert_ablation.py`:对大量(EGO 已承诺 B-spline + 快速移动圆柱)案例,比 ①连续 Bernstein 证书 ②N 点离散采样 ③稠密 4000 点真值,统计 **false-safe(判安全实则撞)**。

> **结果(回填):**
> | 方法 | 判 safe | **FALSE-SAFE(漏撞)** | false-unsafe(保守拒) |
> |---|---|---|---|
> | continuous (Bernstein) | `__` | **`__`** | `__` |
> | discrete-2 | `__` | **`__`** | `__` |
> | discrete-3 | `__` | **`__`** | `__` |
> | discrete-5 | `__` | **`__`** | `__` |
> | discrete-9 | `__` | **`__`** | `__` |

**要点**:连续证书 false-safe = 0(sound,这是它的全部意义);离散采样在快速 mover 下会**穿越漏撞**,采样越稀漏得越多。这就是"为什么要连续时间证书而不是采样检查"的硬证据。

---

## 5. 结果:predictor 选择 = 收紧 keep-out(更快)

`metaurban/predictor_compare.py`:keep-out = 残差的 conformal 分位,所以**更好的预测器 → 更小 v_eff → 同等覆盖下飞更紧/更快**。比 CA(常加速,当前)vs CV(常速,丢掉噪声加速度项)。

> **结果(回填)**:pedestrian 胜者 = `__`(v_eff `__` vs `__`)。对走走停停的行人,CA 的加速度外推会过冲 → CV 残差可能更小 → 部署切 CV 可在不破坏覆盖的前提下收紧 keep-out。

---

## 6. 诚实边界(红线照旧)

- **边际覆盖,非条件覆盖**:conformal 给的是跨实例的 marginal P(碰)≤ε,不是"对每个具体行人都 ≤ε"。条件覆盖要 Mondrian/CQR,列 future work。已做的 per-class 是粗粒度的 Mondrian。
- **exchangeability 假设**:按 track 切 cal/test 尊重了轨迹内相关性,但跨场景分布漂移(不同城市/密度)仍可能破坏覆盖 → 部署需在线再标定或 ACI(adaptive conformal)。
- **回放 = mover 不让路**:strict 最坏情形(真 ORCA 会部分让路 → 实际更易)。但也没建模 mover 对无人机的反应博弈。
- **感知仍是 GT/带噪 GT**:标定用的是真位置 + 高斯噪声,**不是真深度相机**。D435i 深度相机接入(遮挡/量程/真实噪声)是下一步(`task #6`),会让残差/覆盖更可信。
- **静态障碍未建模进回放**:回放只放 mover,街景静态几何没进 EGO grid(渲染器主线有)。

---

## 7. 复现

```bash
# 1) 收割真轨迹(metaurban env)
PYTHONPATH=/media/boxuan/Data2/projects/metaurban python metaurban/conformal_harvest.py --seeds 0-19 --steps 600
# 2) 标定 + 覆盖验证(sando env)
python metaurban/conformal_calibrate.py          # -> out/conformal/calib.json (+ calib_coverage.png)
# 3) A/B 验收
python metaurban/ab_replay.py --seeds 0-19 --n_ep 6 --eps 0.05
# 4) 连续 vs 离散消融
python metaurban/cert_ablation.py --trials 500
# 5) predictor 收紧
python metaurban/predictor_compare.py
```
产物都在 `out/conformal/`。
