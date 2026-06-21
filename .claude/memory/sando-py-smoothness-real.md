---
name: sando-py-smoothness-real
description: 塔菲大人确认轨迹平滑是真需求(非cosmetic);关键:接缝/recovery平滑在修证书前提"飞的=认证的",强化硬认证;5步落地
metadata:
  type: project
---

**2026-06-19 塔菲大人明确:轨迹平滑是真需求,不是 cosmetic(我之前判错被纠正)。** 关键和解洞见:

**接缝/recovery 平滑 = 修认证证书自己的隐含前提"飞的 = 认证的"**(今天被两处静默破坏:接缝处无人机实飞在认证线偏障碍远侧 ~0.2m 乐观滞后;突兀 recovery 离开认证走廊)。所以平滑**强化**硬认证、不矛盾、不是装饰。

**10 类 jerk 来源(逐行核对,planner.hpp)Top**:① 接缝 A_predicted≠A_actual(唯一同时破"飞的=认证的")② recovery_yield 瞬时置零 `:1313` ③ recovery_climb 突兀垂直 `:1346`(failure>8 触发)④ yaw dyaw 阶跃(`yaw_smooth:1494` 只限 rate 不限 accel)⑤ 低速 atan2 抖动+滤波滞后 `:1469`。

**5 步落地(全 default-OFF + golden 20/20 等价 + 不软化走廊 + 不碰证书):**
1. **接缝 C2-from-exec-state**(最高,~30-40 行):新 MINCO 起始边界用真实执行态 A_exec 而非预测态 A(=障碍侧诚实修正 1050-1063 的无人机侧对偶)。强化证书。落 `planner.hpp:864-867/1071-1077/1394`。
2. **S7-CRET recovery**(~60-90 行+capi 重编):见 [[sando-py-cret-recovery]]。
3. recovery 刹车 C2 splice(~15 行):瞬时置零换 min-jerk 减速。
4. yaw 二阶 reference-governor(cert-正交,demo 抛光,**最低优先/最先砍**):限 yaw-accel+滞回+去 freeze/spin。**否决方案 A(yaw 进 MINCO 第4维)**:动 ABI + 与 cert retime warp 冲突。
5. k-value EMA + retiming 相位(低,post-9/15)。

步骤1-3 属安全层邻接、可 9/15 前做;方案A/写平滑当贡献=post-9/15 side-paper。详见 [[sando-py-planner-benchmark-doc-fix]]。
