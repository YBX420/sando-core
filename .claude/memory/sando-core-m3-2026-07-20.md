---
name: sando-core-m3-2026-07-20
description: "M3 承诺机制 v1 落地(ST_COMMIT,33cc578):持有样条+dp_commit/贪婪采纳+成对执行器;字节三面零伤;探针分裂(s19 -0.7s/+54% vs s5/n3收紧负,n3碰撞=包络非无证);病根定量=承诺半衰期 KF 0.45s vs 全知 1.95s(4/5 是估计器税);待塔菲大人裁三路"
metadata:
  type: project
---

# 2026-07-20 M3 承诺机制战役(8d687a5..33cc578)

## 设计与评审(动刀前)
27-agent 级评审复刻:5-lens workflow 攻击设计书 → sound-with-fixes 5/5,1 blocker(快路径漏
extra_gate)+ 8 must-fix 全数吸收。命门预言应验:原始"进度差采纳"在主战区结构性不开火;
三分离架构(优化面 T_c=2.4s / 担保面逐拍裁 tau 重证 / 持久面 state)是评审背书的骨架。

## 实现(33cc578,decide+executor 成对=评审绑定护栏)
- st_speed.dp_commit(向量化,今天动力学:刹 4 格/行、放 1 格;字典序 (reached,-t_arr,s_end))
  + greedy_profile(短视策略忠实模拟,属 DP 可行集 ⇒ DP≥贪婪是定理);量化律 du=dt/20;
  自测:自由格等速、盒场景 DP 3.4s 到 vs 贪婪卡死、200 随机格支配不变量。
- safety_layer:承诺快路径(**持有采纳拍样条不重规划**,certify_profile 带 u_start 滑动
  ——首版每拍重规划把挡序列时间基全毁,COMMIT→DROP(cert) 1 拍死,持有样条后治愈)、
  事件表 carrot/exhaust/goal_switch/betray/spline_out/gate/cert+policy_flip 全大声、
  背叛=ID优先+NN兜底、采纳后置 pass(incumbent+胜者+evade 救援)、refractory、
  SPEED_SLEW 包装器直通、ST_COMMIT 压制 ST_SPEED in-rank。
- renderer:承诺挡逐字飞(绕过 EGO_G_RELEASE 钳)、held 时 t_ego 连续、flown 门带 stc_u0、
  [wait] stand_ticks 标签盲记账(两臂都印,治"hold 靠记账下降"审计洞)、g_flown/raw_mv 管道。
- ops:stc/stck10/orastc 具名臂进池+考场;考场账本面版本化 crosser_exam_sight30。

## 零伤面(全绿,三重证明)
金哈希 12/12 vs 5150b71;render 面 base s7 与 base n3 考卷均与 _sight30 账本**逐字节相同**;
校准后 s7 开臂 0 承诺=逐字节同基线(采纳门不开火时臂完全透明)。

## 探针成绩(ST_CADOPT_S=0.3 校准版;分界:赚钱承诺 dp-gr≥1.0 或 r1-vs-死,交税的 ≤0.10)
| 探针 | base | stc | 判 |
|---|---|---|---|
| s7 素 | 8.8 | 8.8 逐字节(0 承诺) | 零税 ✓ |
| s19 | 8.7/clr1.10 | **8.0/clr1.69**(1 承诺:dp1.68 vs 贪婪0.60 卡死) | 最佳单发 ✓✓ |
| s5 口袋 | 12.1/0.87 | 12.8/0.59(4 承诺全夭折) | 负 ✗ |
| n3 宽松版(S=0.1+tie) | 10.1/ped0.93/hold17/站17 | 10.2/ped**1.51**/hold**4**/站7 | 时间平质量碾压 ✓ |
| n3 收紧版 | 同上 | **碰撞**(animal -0.14) | 反转 ✗ |

**n3 碰撞验尸(KFDBG 确定性复现)**:撞击 t≈6.4,最后承诺 t≈2.4 已掉——撞时素机器认证
HOLD 原地悬停 v=0.00 被 6.25m/s 横穿者碾过=物理包络类(认证悬停跑不过冲脸快车),
外加前期承诺挪站位的运气税。**不是无证飞行,不是 M3 soundness 洞。**

## 病根定量(战役核心发现)
**承诺半衰期:KF 臂 ~0.45s vs 全知臂 ~1.95s(且一次自然寿终=飞到 carrot)。**
⇒ 4/5 死亡率=估计器税:①cert 抖振=KF 速度噪声地板(0.4m/s×tau=±0.3m 管摆)>0.15 采纳余量;
②betray 把"世界变了"与"估计器错了"混为一谈(dev0.88 案例:kfv(1.18,1.15) vs 真值(0.27,0.96),
mover 根本没拐)。架构本身喂真值能撑 2s。全知 n3:orastc 9.1 vs orabase 8.9(+0.2,肉未证,
但 switches 7→3、spchurn -10%)。

## 未决(塔菲大人拍板)
A(我荐)存活优先:betray 阈值速度尺度化(∝|v|·t)+采纳余量再抬,目标 KF 臂半衰期≥1.5s,
  达标再全池+成片;B 冻结 v1 为 opt-in 研究臂,全池先画地图(n3 碰撞提示有险);
C 判"架构已证、等估计器":并回 KF 病②③/radar 线,M3 挂起。
另:宽松 vs 收紧两套采纳参数是真 tradeoff(质量碾压 vs 零税),默认怎么选属同一裁决。
k10 转默认仍在桌上(证据链早已闭合)。

## ❄️ 冻结令(07-20 当日,塔菲大人:"存住所有档。冻结这个算法")
**M3 按 C 路收档:架构已证、等估计器。** ST_COMMIT 留树默认关(=字节不变),不再调参不再动刀。
解冻条件(按当日分析):估计器改善落地后(radar 速度观测/KF 病②③),先重测承诺半衰期,
KF 臂 ≥1.5s 再谈转正;届时剩余设计件=①背叛判据换 KF 自身 σ(分辨"世界变了"vs"估计器错了")
②采纳时按 σ 加宽盒规划(在不确定性下承诺)③全池判决门。
冻结时未决(不随冻结自动裁决):**k10 转默认**(证据链闭合待拍板)、**vs native 锥对齐 AB**
(九月 arXiv 主表,视力手术后未重打)、CAP_MEET 345 集、TDYN replay 面、λ 门重算。

## 成片档(证明纪律,全素颜 HUMANOID_NO_IDLE=0,存 metaurban/out/,Data2 常驻 gitignored)
**写实面**(数字=账本逐字):ab_m3_s19(基线8.7/1.10 vs 承诺8.0/1.69)/ ab_m3_n3(10.1/ped0.93/h17
vs 宽松承诺 10.2/1.51/h4)/ ab_sight_s7(近视8m 10.1/0.62 vs 30m 8.8/1.41)/ ab_k10_s12(14.5 vs 9.9)。
**GT-xy 面**(PERCEPT_SIGMA0=0 SIGMA_K=0 PMISS0=0 PMISS_K=0 SIZE_ERR=0;遮挡+视锥保留;
新测量面不覆盖写实账):ab_gt_m3_s19(7.4 vs 7.4 **零承诺逐字节=M3 在无噪声面正确隐身;
写实面 s19 胜利本质=治噪声犹豫;噪声本身让基线亏 1.3s**)/ ab_gt_m3_n3(10.5/h11/sw14 vs 宽松
13.3/h3/sw4——**首例承诺自然寿终 t_since=2.4 exhaust,存活 2.4/1.7/0.6s=估计器税第三次独立实证;
betray dev1.50=真背叛;宽松配置两面皆过度承诺**)/ ab_gt_sight_s7(11.4 vs 8.4,视力税与噪声无关)
/ ab_gt_k10_s12(11.7 vs 9.0,**σ余量红利不只是抗观测噪声,差分速度固有不确定性本身值 2.7s**)。
各配 _poster.png。感知面真相(07-20 核实):写实面检测=真值中心+σ(d)=0.05+0.01d 高斯+漏检
p=0.05+0.15(d/R)²+硬遮挡+无 GT 身份 NN 关联(perception.py);"KF 吃纯真值 xy"仅 GT 面/全知臂成立。

## 工具移仓
pool_parse.py 已从易失 scratchpad 迁 **metaurban/ops/pool_parse.py**(池账本解析,hold 可选/inf 兜底)。

相关 [[sando-core-m2-2026-07-17]] [[sando-core-m1-2026-07-17]] [[sando-core-kf-campaign-2026-07-14]]
