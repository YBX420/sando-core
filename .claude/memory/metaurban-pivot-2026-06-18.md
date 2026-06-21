---
name: metaurban-pivot-2026-06-18
description: "2026-06-18 决定:MetaUrban 转为算法迭代+完整场景搭建的主力 sim;不再追求 Isaac RTX 保真度(高效验证>照片级),实质放宽 spec「Isaac 机载渲染标定」铁律。Isaac IRA warehouse 管线保留作回退。"
metadata:
  type: project
---

**2026-06-18 决定(塔菲大人)**:**MetaUrban**(https://metadriverse.github.io/metaurban/,MetaDrive 系,轻量城市行人 sim)转为**现在的主力**,用于:① 迭代算法(conformal 证书 + RTA + freeze-yield)② 搭建完整场景 ③ 验证 sim-to-real gap。**明确:不考虑 Isaac RTX 保真度——"能高效验证"才是重点。**

**为什么转**:当天在 Isaac Sim 4.5 + IRA 上**跑通了 warehouse 5/15/40 三档行人 GT**(`isaac/ira/`,坑与用法见该目录 `README.md`),但做**室外复杂场景 Rivermark** 时在 12G RTX4070 上反复受阻:整城 56k prim 太重 → recast 烤 navmesh 卡死段错误;flat-plane 高度猜不准 → 行人埋地里;IRA 相机绝对高 z=2/3(为 warehouse 地面 z=0 写死)在 Rivermark 地面 z≈4.5 下方 → 全黑。结论:Isaac 这套对"验证 gap+算法"的目标**过重**。

**对铁律的影响**:spec(`docs/safety-layer-spec.md`)的「证书用 **Isaac 机载渲染**标定,绝不用别的域」铁律被**实质放宽**(效率优先)。若日后真要照片级感知标定那一锤,再用保留的 Isaac warehouse 管线补。**注意:当前未改 spec/plan 原文**,下次正式确认时要写清。

**现有资产(别重复造)**:Isaac 4.5 装于 `/media/boxuan/Data21/isaacsim`(软链 `~/isaacsim`);资产包 104G 于 `/media/boxuan/Data21/isaacsim_assets`;IRA 三档 GT + `ira_gt_extract.py` + `_check_motion.py` + 所有坑 都在 `isaac/ira/`(及其 `README.md`)。Rivermark 包装层 `isaac/ira/rivermark_ira.usda`(裁 foliage/grass + flat plane z=5.8 + 相机抬高)做到一半、**已终止未验证完**——转 MetaUrban 后搁置。

**下一步(明天)**:装 MetaUrban → 跑通行人场景 → 确认它的 GT(每帧行人 pos/vel/class)、传感器、相机机位能否支撑算法 + gap 验证。当天的 MetaUrban 评估 workflow 被中止未出结论,明天可重跑或直接上手。承接 [[sando-py-pivot-2026-06]](方案 B 安全层主线不变,只换验证 sim)。
