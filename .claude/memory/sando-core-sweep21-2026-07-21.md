---
name: sando-core-sweep21-2026-07-21
description: sweep21 战役(07-21/22):全代码重审+20-seed 双臂渲染压测;钦定尺干净=0 hold 到达;KF 5/20 vs GT 15/20;全知臂脏=纯活性病(击杀 644 全 replan 证书 0 拒);s17 终身监禁 AB;异常 3/3 复现
metadata:
  type: project
---

# sweep21 战役(2026-07-21/22,塔菲大人令:"从头审查所有代码和 log,不断 render 测试出问题;render 和 log,和对比")

**产物全在 `metaurban/out/sweep21/`**:CAMPAIGN.md(总档)/ final_triage.txt(40 行全表+分型)/
audit_code_findings.md(=docs/code-audit-2026-07-21.md, commit 6d9c6a3)/ 每趟五件套(mp4+log+explain+attrib+账本)
/ ab_s17_kf_vs_gt.mp4(核心 AB 成片)。驱动器与重触诊器在 scratchpad(sweep21.sh/wave2.sh/retriage.py,
断点续跑,killR/killG/killC 分型判别子)。

## ★ 战役中钦定(07-21):干净收严 = 0 hold 到达
"没有任何额外机动,仅凭飞行和预判到达";带 hold 的到达不得再称干净;gear-0 停拍同罪;
"等待-然后-走"(M3/ST)在新尺下天然不干净。详见 home memory [[feedback-honest-boundary]]。

## 头条(钦定尺,20 seed × KF/GT)
- 干净率 **KF 5/20(s3,8,9,11,18) vs GT 15/20**;零碰撞、零 Traceback、40/40 rc=0。
- **全知臂的一切不干净 = 纯活性病**:GT 644 次候选击杀全部 replan,**证书对全知臂 0 次拒绝**。
  含 **s17 终身监禁**(t=5 被吞 201 拍到超时,astar 错 3.1 万/墙钟超时 8899,悬停自证 133 拍通过
  ——"安全地冻死";同 seed KF 肥管早拐 32 绕 8.3s 通过 = 精确导致贪婪的终极 AB)与 s5(ncyl=18)。
- KF 臂脏三分:**活性口袋×7**(s0,4,6,7,10,14,16;killR 主导)/ **诚实等待×5**(s1,5,12,15,19;
  killC 主导)/ gate×1(s13)。s0=3cm 动物擦身在 41 拍无证 hold 窗内(悬停自证 0/41)=s7 碰撞幸运孪生。
- 估计器税(双到 19 seed):中位 **+2.0s**,range [−3.7,+4.6](负端=s11/s17 反转)。
- U 暴露:KF 265 拍 / GT 92 拍(s17 一家 68)。额外机动画像入账本(s8 kf "干净但 72 绕"=新尺下
  0 hold 却机动繁忙的代表)。

## 波二确定性(6 复跑)
gt_s17 监禁 3/3 逐毫米复现;kf_s0 擦身 3/3 复现(t_goal ±0.4s 墙钟摆动);s5 复现。
**墙钟不确定性只在边缘口袋翻结局**(gtxy_s7 四跑三结局),深口袋=结构监狱。

## 代码重审册(无现役 soundness 洞;详见 docs/code-audit-2026-07-21.md)
P3 休眠雷=slew 复证 delta 默认 0.0(safety_layer.py:1572,现役调用全显式传参不触发);
P2=执行器爬升脱困无证飞行角落(render_3d_video.py:2920);化石=vel_smooth 死代码、PERCLASS_CONF
拿 d_safe 当类键、verdict3 缺尾流旁证、_GTXY_TRK 跨圈不清零等;存量 log 普查=死亡链系统性背景病
(7 月以来所有 log 成百上千条);overnight.log 56 个 TB=退役工作台老病。

## 修复菜单含义(待拍板,全属解冻级)
活性四刀(喂云挖无人机气泡[planner-only 云,soundness 好论证]/ free-start / **A* 预算墙钟改迭代数
[~2 行根除不确定性]** / 逃生树+撤退沿历史走廊原语)下完 ⟹ 全知天花板 15/20 应逼近 20/20(那 5 个
脏 seed 无一是证书/估计器的错);之后估计器追赶(10 个 seed 差距)是第二战场。

## ⚠️ 北极星勘误(07-22,外部诊断逐条对码确认)
真目标原话:"没有任何额外机动,仅凭飞行和 KF 预判到达" → **KF 0/20,GT-xy 0/20(结构性)**;
五列总量 KF 停272/绕443/越73/爬35/切267,GT 停229/绕368/越79/20/197。
**本战役"GT 臂"=GT_XY(精确 xy+KF 速度),不是全知;ORACLE_THIN 矩阵未跑,天花板结论作废重测。**
结构根因三件:①候选集只有直/绕/越/爬(safety_layer.py:1201)②排序明文"全速绕行胜过任何降速直行"
(:1290)③时间信息被压成空间墙(CPA 环永久墙 1684 / TDYN 只是软代价 bspline_optimizer.cpp:396 /
全速证书冻结合取 :263 封死"等人走开从原位穿")。ledger.tsv holds 列 bug(全 0),权威=final_triage.txt。
straight 标签内 EGO 仍弯绕 → 账本低估机动,真验收需轨迹级指标(横偏/航向/曲率/挡位)。
一句话:现系统="KF 辅助的认证绕行锦标赛",非"KF 驱动的原航线时空穿缝器"——0/20 的根因。
外部建议的修复顺序(直线时空臂/占用时间窗/最快非零速度曲线/物理无解诚实判定/再做 KF-vs-ORACLE 税)
待塔菲大人拍板;⚠️ 我方补充的物理墙:诚实管子随视野膨胀(q+v_eff·t),长窗穿缝的可行性本身
被估计器质量定价——"提前量须匹配估计质量"定律的正题,ORACLE 臂穿缝免费,生产臂要挣。
