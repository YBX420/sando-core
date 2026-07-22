---
name: sando-core-guide-arm-2026-07-22
description: 北极星 guide 臂战役(07-22 塔菲大人图纸钦定):预测→指导线→EGO 自然飞;架构/文件/酸试判决/gtxy 0-hold 战役现状/迭代协议/下一步=clear 后全量代码检测找病根
metadata:
  type: project
---

# 北极星 guide 臂(2026-07-22 塔菲大人按外部诊断图纸下令,当前主战役)

## 钦定图纸(一句话)
把"预测→造墙→枚举机动→控制执行器"改成 **"预测→一条稳定指导线 Pguide→EGO 自己规划并抵达"**。
验收:到达 ∧ exec_src 全程 plan ∧ 无 warp ∧ 无 hold ∧ 无机动命令 ∧ 无恢复覆盖 ∧ 零碰撞;
**EGO 沿 Pguide 的自然弯曲不算额外机动,Layer 下达绕/越/爬/降挡/悬停才算**。
当前钦定焦点(07-22 晚):**只调 GT_XY 臂,目标 gtxy 0 hold**。
北极星原话(07-22 更早):"没有任何额外机动,仅凭飞行和 KF 预判到达"(基线 0/20 双臂,见 [[sando-core-sweep21-2026-07-21]] 勘误节)。

## 已建成(commit 6ab60f8 → 04a37c4 → 9324f31)
1. **EGO 指导线入口(C++)**:`ego/include/planner_manager.h` setGuidePath+guide_path_ 成员;
   `ego/src/planner_manager.cpp` reboundReplan STEP1 新增 guide 分支(指导线按 ctrl_pt_dist 弧长重采样
   成初始点列,首尾钉到 start/target,rebound 只在占据要求处变形;空=遗留初始化字节不变);
   `ego/capi/ego_capi.cpp` ego_set_guide_path;`metaurban/ego_bridge.py` set_guide_path(stale-so 大声守卫)。
   **改 C++ 后必须手动重编 ego_capi.so**(配方在 CLAUDE.md)。冒烟:无 guide 直线 0.000 / +1.8m 拱 EGO 飞 1.755 / 清空复原。
2. **`metaurban/predictive_guide.py`(独立算法,5/5 自检)**:最小横向变形余弦拱;粘性侧承诺+
   **封顶侧换边逃生舱**(可行性压过粘性=唯一合法换边);每拍限幅 GUIDE_SLEW=0.45m;自然 ETA
   (从当前承诺轨迹弧长-时间映射,不用巡航假设);尾流侧偏好;**半径=滑动窗最大膨胀律
   R0+veff·(min(τ(s),τw)+δ)+GUIDE_BAND(0.25)**(M3 网格同律;酸试一没带生长项被证书连环枪毙=大教训);
   **冻结孪生盘**(τ(s)≤1.4τw 弧段内加 c0 静盘=证书冻结合取的镜像);GUIDE_OMAX=4.0。
3. **渲染器 `EGO_DECIDE=guide` 新臂**(render_3d_video.py):mover 永不进占据云(拆墙——**四环死亡链
   在 guide 臂灭绝实证:astar/inobs/墙钟全 0**);TDYN 关;无锦标赛/无挡位/无爬升恢复(逐出);
   决策=build_guide→set_guide_path→replan→static_clear/mover_clear_flown/cert_clear(why-sink);
   失败两级回滚(roll1=重发 last_ok 指导线;roll2=held-spline 经 st_cert.certify_profile(u_start=t_ego)
   精确复证,executor 靠 _MAN_V2["guide_held"] 保 t_ego 不清零);hold=fail-closed 记败(悬停自证→hold_cert 收据);
   **explain jsonl 每拍**:win/s/certified/cert_id/ncyl/guide{conflicts 有名有姓,off_max,infeasible 旗}/
   fail 死因链{at∈guide|roll1|roll2 × leg∈replan|static|flown|cert(+why)}。

## 酸试判决(全在 out/sweep21/)
- 酸一(guideOR_s17/guideKF_s0):双败——**病根=我漏了管子生长项**(修于 04a37c4)。
- 酸二:**s0 KF×guide 首胜** reached 12.9s/净空 0.892m(锦标赛同 seed 3cm+41 无证 hold),残余 hold×30;
  **s17 ORACLE×guide 诚实败**:44 拍 infeasible(横向 2.6 封顶解不开收敛人群,冲突压到 s≈0)→hold 暴露→t6.2 U 碰;
  抽搐灭绝(switches 37→1)。杀手榜:mover_clear_flown 门 + roll2 certify_profile(严格对策略)+ roll1。
- 中途单针:**org_s1 = 第一趟真 0-hold 到达**(14.0s,guide×140,hold=0);kfg_s1 hold×8。

## 正在跑(clear 时可能已完)
**gxg 矩阵 = GT_XY=1 THIN=1 EGO_DECIDE=guide × 20 seed**(gxg_s0..19,双路)。产物:
out/sweep21/gxg_sN.{log,mp4} + explain_gxg_sN.jsonl + attrib_gxg_sN.json。判卷=多少 seed hold=0。
驱动器:scratchpad/sweep_gtxy.sh(断点续跑,重跑=直接再执行)。kf/oracle 矩阵(kfg/org)被令中止(部分 seed 已有)。

## 迭代协议(钦定焦点)
读 explain fail 链→按最大杀手下刀→重跑 hold seed→全 20 验证,直到 gtxy 0 hold 或诚实物理墙
(infeasible+成片留证)。已知待查嫌疑:①mover_clear_flown 门与 guide 半径律不对齐/四旋翼跟踪暂态;
②roll2 certify_profile 严格对(predicted∧frozen 全 mover)偏杀;③hold 后 ETA 陈旧(v≈0 时 vref=1.2 地板,
committed 轨迹 stale 时弧-时映射错位);④s≈0 冲突(路形改不了"已站在那")=撤退原语缺口(另轴待批);
⑤jerk 44/revs 34 = 逐拍重规划的 EGO 原生 retiming 速度跳变(北极星臂无挡位平滑,观察项)。

## ★ 下一步(塔菲大人 07-22 令,clear 后第一件事)
**全量代码检测,思考问题出在哪**——带着 gxg 矩阵结果,从头细读 guide 臂全链
(predictive_guide.py / render_3d_video.py guide 分支+执行器 / planner_manager.cpp guide 初始化 /
mover_clear_flown / st_cert.certify_profile / cert_clear)找 hold 残余的真病根;渲染验证走
sweep_gtxy.sh,证明纪律=成片。旧账本注意:sweep21 的锦标赛矩阵/勘误/北极星 0/20 基线全在
[[sando-core-sweep21-2026-07-21]];诚实墙(管子随视野膨胀=估计器定价)已多次确认,别推翻。
