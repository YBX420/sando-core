---
name: sando-core-m1-2026-07-17
description: "M1 KF 全脸接入收官:σ喇叭管+护栏版σ余量(KF臂 8.2s/净空1.96=追平全知净空,全知零伤逐字节)+ radar 观测口(车辆 -6~7%)+ 拒绝死锁兜底(NET NEUTRAL 留树)+ bayes 负判决;27-agent 评审护栏全数执行"
metadata:
  type: project
---

# M1 · KF 全脸接入(2026-07-17,a513353..159ba48;总路线=docs/timespace-plan-2026-07.md)

**台账约定:** 每行 = 判决 @证据所在干净 commit;渲染面=THIN 薄壁声明面(EGO_PERCLASS=thin,calib 不加载);字节回归金哈希=metaurban/regress_golden/5150b71.json。

## 判决表

| 刀 | 判决 | 证据 |
|---|---|---|
| **M1a σ(t) 喇叭管** | `predict_sigma(ts)`:F P Fᵀ+Q(t) 逐视界位置σ,含过程噪声(收敛行人 0.09/0.25/0.77/2.5m @0/0.3/0.75/1.5s,coast 整体抬升)。纯新增。 | @0cdbebc 自检 3 连 PASS(注意:当时树上有 M1c 未提交码——树卫生违规已被评审抓获,此后全部先 commit 后取证) |
| **M1b σ 余量(护栏版 v3)★M1 最大战果** | ETA_FEED=2:余量=min(ETA_K·predict_sigma(lead+dt), 1.2),仅成熟(n≥4)非 coast track(coast 已在 r_mem 付过σ,双计已除),GT_ORACLE 硬零。**KF 臂 8.2s/净空 1.96(=追平全知臂 1.97!)/hold 7 vs 基线 10.1/0.62/14;K=1.0..2.0 宽平台逐字同(地形健康);K=0.5→7.8s(提速候选,离全知 0.6s,平台外待池裁);全知臂开着模式2 = 7.2 逐字节零伤**。v1(带 q+veff 项)全知 9.6 判负;v2(标量带)数据评审作废。ETA_FEED 仍 opt-in(转默认=塔菲大人拍板,345 集欠) | 码@52de418,默认+证据@159ba48;成片 out/drone_3d_m1b_before_s7.mp4(基线10.1)/ _sigma_s7(v2 8.5,历史)/ _v3horn_s7(v3) |
| **M1c radar 观测口** | `update_velocity(v_xy,r_vel)`:H=[0,1,0] 同拍融合(时钟归 update/coast),速度创新门 3√S+0.5,**评审抓获的 _z0 两点闩 bug 已修**(radar 后下一个检测曾把融合速度整个砸掉;三明治回归:det1→radar→det2,σ_v 保持 0.28)。nuScenes:RadarVel(手解 43B PCD,POINTS 精确,invalid/ambig/dyn_prop/rms 筛点,位置门+pos_sigma 加宽)。**考卷(lidar+seg 基线=07-14 终榜逐字复现):车辆均值 1.16/1.74/2.96/4.39 → 1.08/1.62/2.79/4.12(-6~7%,中位 3s 1.53→1.22);行人 1.24→1.22(微,雷达对行人回波弱=物理)** | 码@131bdd1(初版@5150b71 数字作废——门前跑的);干净考卷 log=scratchpad/radar_clean.log |
| **M1d 拒绝死锁兜底** | PERCEPT_REACCEPT=N:拒绝**事件**计数(ready 移动 track 未匹配∧σ加宽宽门内有未认领检测),N 连发→强制收编最近者(封顶 birth_vmax·dt+3σ),证据消失即清零;不碰出生/静态/YOUNG_TTL。**90 集 replay AB:全指标持平(NET NEUTRAL)→ 留树默认关**(此池拒绝死锁罕见;nuScenes 面是实锤病,兜底保留)。课1+2=ASSOC_KGATE 一体,**尊重 b757c86 旧判决(NET NEUTRAL 不采纳)未重打** | 码@377cd0b;AB=rows_m1dbase_w0 / m1dreacc_w0.jsonl(90 集) |
| **M1e bayes 初始化** | **负判决**:seed7 KF 臂 13.1s/hold 29/行人净空 0.79(基线 10.1/14/1.44)——宽先验 ~3 拍才收敛,σ_v 长悬年轻门上→冻结片翻倍。默认维持两点差分。**条件性复活:radar 在场时 σ_v 被直接砸下,bayes 或翻身(nuScenes 面待验,挂 M2 后)**。与 M1c 交互:_z0 闩清除对 bayes 路径无碍(bayes 不读 _z0) | @377cd0b 树跑(bayes 码在 kf_tracker 未动,env 门) |

## 全知锚点史(每改必测)
7.2s 逐字节 @a513353→0cdbebc→(4666f23 期间三次)→377cd0b→52de418(ETA_FEED=2 开着也 7.2)——全天零翻转(死加载器修复后布局病未再现)。

## 护栏执行情况(27-agent 评审,全数照办)
1. **树卫生**:违规一次被抓(M1a 证据混 M1c 脏码)→ 此后 5 刀全部"先 commit 本刀→跑证据→台账记干净 sha";
2. **M1c _z0 闩 bug**:确认真实,已修+三明治回归+创新门回归(131bdd1);时钟契约按评审写死;rms 用作相对筛(devkit LUT 未 vendor,妥协已注明);
3. **M1b 重做**:五护栏全落(成熟门/coast跳过/cap1.2/硬零/编码钉死+喇叭σ(lead));v2 数据作废重扫;规则⑥(估计器刀落地后重扫 K)以顺序满足(M1c/d/e 全部先落,K 扫在最后);
4. **M1d**:先读 b757c86 才动手;三课收窄为一课(1+2 同体已判);拒绝事件按评审定义;行为面=90 集 replay AB(关联级 ID交换/幽灵指标未建——欠账записано);
5. **可回溯**:calib_norm.json 已提交+诚实标注"无加载器读它"(M1a 提交语的乘子说法已纠);calibrate_norm 防覆盖(FORCE=1);regress.sh 死路径修+双参用法+金哈希入库(regress_golden/5150b71.json,377cd0b 树 12/12 全同);本轮无 .so 重编。

## 给 M2 的移交清单
1. **345 集全池裁决**:ETA_FEED=2 转默认与否 + K=0.5 vs 1.0(单 seed 工作点,池上定);
2. **移动横穿者考场**(M2 第一件事):修 EGO_ADV_CROSSER 掐点(vcru 按臂实测均速),σ 余量/radar/ST 图全要这考场;
3. 关联级行为指标(ID 交换/幽灵出生/寿命直方图)harness 未建——M1d 若要翻案先建它;
4. bayes×radar 组合验证(nuScenes 面);
5. σ 喇叭管的保形乘子:接 calibrate_norm 线时**必须真写加载代码+大声打印**(死加载器法);
6. 渲染面 radar 不存在(MetaUrban 无雷达传感器)——σ 收益全靠 M1b 通道;ST 图(M2)吃 predict_sigma 现成。

相关 [[sando-core-raceline-2026-07-16]] [[sando-core-sweep-kf-audit-2026-07-16]] [[sando-core-kf-campaign-2026-07-14]]

## 全池裁决补记(当日晚,46ce91e,塔菲大人令:只跑全池不动 M2)
render 面 seed 0-19 三臂 + 全知抽查(ops/m1b_pool.sh,断点续跑):
- **K=0.5 死刑**:s5 真碰撞(撞牛 −0.27m)——seed7 上它是 7.8s 明星,全池抓出会撞人。**单 seed 永远不足以转正**。
- **K=1.0 红利属实、带病缓议**:行人净空 +30%(1.20→1.56)、最小净空 +38%、时间持平、s14 救命案例(0.13→1.24);病:s5 堵死(346 hold 超时)、s10/s12/s16 hold 风暴——**σ余量→路线拓扑翻进口袋**,与 oraeta/K=1.5 同病族。维持 opt-in。
- **全知零伤门 5/5 逐字节**(s3/7/11/15/19)。
- 欠账:①k05_s5 碰撞验尸(撞时是否有证?模型外还是证书洞?)②s5 口袋病灶诊断(法医工具现成)③诊完才准复审 K=1.0;可能的药=hold 连续 N 拍自动收缩 K 的自愈阀。
