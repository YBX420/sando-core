# MetaDrone Benchmark 评测协议 v1(2026-07-03 冻结)

## 环境(全部默认,改动必须显式注明)
- 感知:PERCEPT=realistic(锥半角45°/量程10m、σ=0.05+0.01d、p_miss=0.05+0.15(d/R)²、遮挡开、
  NN 关联无 GT 身份);静态=在线建图语义(见过才存在、存在即不忘);FOV_R=10 全栈统一。
- 证书:TAU=0.75s、δ=REPLAN_DT、per-class conformal 管(calib.json)、δ_track=0.473。
- 场景:scenarios/full(11 手工压力 + 6 生成街景 + 3 packed + 8 噩梦);生成场景过间距门
  (0.8/2.0/4.0m)+ 城建规则 + 遭遇几何过滤(走廊压力 ≤1m)+ 安全起飞区;手工编队豁免间距门。
- 臂:ours_gt / ours_gt_dyn / ours_real / ours_real_dyn(部署真实态)/ native_gt / sando_gt。
  ⚠️ native/sando 无感知模型:公平对比仅 gt vs gt;realistic 臂只刻画 ours。
- 重采样:≥8 传感器种子(PERCEPT_SEED=1234567+7919k);gt 臂检测噪声同随种子(独立采样)。
- 指标:碰撞率(Wilson CI + 场景层 bootstrap)、近失率(<0.5m)、clr 中位(bootstrap CI)、
  配对 clr 差、RTA 失效率(rta.violations/certified_ticks,replay 结果内置)。

## 一键复现
```bash
cd sando-core/metaurban
./metadrone.sh test                                # 全门(编译/间距/自测/.so 新鲜)
LD_LIBRARY_PATH=~/gurobi1103/linux64/lib python3 bench_run.py --resample 8 --dir scenarios/full
python3 bench_speed.py                             # 速度扫描 4-8 m/s
python3 b_bucket_recalibrate.py                    # 标定重挣(部署在环)
```
已知混杂:速度扫描的 contested 时序按 v_nom=2.4 对齐(高速=变相简单);发表级需 per-speed 重调。
