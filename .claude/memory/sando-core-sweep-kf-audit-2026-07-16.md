---
name: sando-core-sweep-kf-audit-2026-07-16
description: "07-16 下半场:全仓吞错扫描(101 agent,20 项坐实全修,ego_bridge TDYN 静默失效为最险)+ 瘦身 + KF 利用审计(判决:位置/速度用得扎实,协方差半张脸浪费,NIS 仪表生来是死的已修)"
metadata:
  type: project
---

# 2026-07-16 下半场:全仓扫 + 瘦身 + KF 利用审计(8dae89e..5b0af5f)

## 全仓吞错扫描(ultracode 工作流:15 路分诊 + 每项 1-2 名对抗反驳者,101 agent)
**56 候选 → 20 坐实(8 HIGH-CONFIRMED)→ 全部修复;36 被反驳**(反驳记录有价值:如 native_objects 的裸 except 其实常规性过滤无物理体道具——但"常规丢弃也必须可见一次"仍成立,修后首跑就抓到 Drone 道具丢弃标本)。
- **最险(已修)**:`ego_bridge.set_moving_obstacles` 在 .so 过期时静默空操作——TDYN 项无声消失,而 THIN 默认开 TDYN 且规划云把 lead-0 圈让给它管 ⇒ 双重裸奔;四个姊妹出口都 raise 只有它哑。现在同样 raise。
- 同族修复:感知摄取丢弃(render/nav/eval/live/demo 五胞胎)、env.step 失败(人群冻结不再无声)、B 桶收割跳文件、px4 offboard 流失败、bench 基线臂被禁、designer 道具表/重编译、TF 丢弃、legacy ROS 参数;C++ 三处(**障碍轨迹表达式编译失败→鬼影原点**、sando_replan catch-all、plan_minco catch)。
- **哑弹**:sticky 里 GAP_CARROT 拷贝引用未定义 `cyl`(replay 生产链上,开开关即炸;pyflakes 抓获,已拆)。pyflakes 装进 metaurban env 了——**"未定义名字"检查应进常规流程**。
- 验证:ctest 27/27、replay 字节回归 12/12 全同、render THIN 锚点 10.1 逐字节。瘦身:24 个文件未用顶层 import 手术(try 探针/项目模块不碰)+ 拆案结取证块(保留 GTDUMP/_fpx 指纹钩子/EGO_SWATH/TIE_KEEP)。

## ★★★ KF 利用审计(7 面并行,判决:**半张脸浪费**)
**用得扎实(生产链真吃的)**:滤波位置(占位圈/证书圆柱心)、速度(CPA 相遇点/TDYN 行/胶囊顶点/SLIP 重定时/证书 obs_vel)、update-or-coast 契约+变步长 Van Loan、coast 时 pos_sigma 膨胀记忆圈(MAN_MEM_K)、sigma_v 年轻门(仅二值)。
**浪费/死件清单(按疼痛排序)**:
1. **NIS 生来是死的(已修)**:replay/bench 喂的是不存在的 `nis_ewma` 属性 ⇒ NIS 门从不可能触发、遥测 NIS 列恒 0;修成读 `.nis`(None 兜底;门默认关=行为不变)。修复当天就被回归抓过一次 float(None)——活仪表和死仪表的区别。
2. **协方差在"被检测中"完全不参与**:占位圈/胶囊/提前量/veff 全是常数;pos_sigma 只在 coast 用;sigma_v 只做二值门。"提前量须匹配估计质量"定律在代码里只落成**每臂常数**(0.7/1.5),不是每 track 动态。ETA_FEED 不确定度缩放版=第一刀(做一半)。
3. **predict() 不带协方差传播**(F P Fᵀ 没暴露)⇒ 下游拿不到"视界 t 处的不确定度",只能用人口分位数常数顶——行车线安全窗的原料缺口。
4. **radar 速度观测口根本不存在**:H=[1,0,0] 位置-only,update 无速度路径;07-14 首选刀要先给 MoverTracker 开 H=[0,1,0] 的口。
5. **默认关的活体**:KF_INIT=bayes(young 病已写好的修复,无人开)、SIGV_GATE、PLATES(年龄桶细分标定)、ASSOC_KGATE(nuScenes 教训#1 半回迁且默认关)、YOUNG_TTL;**nuScenes 三课(门含 pos_sigma/连拒自愈/强制收编)在 MetaUrban 感知前端 0.5/3 回迁**——关联还是欧氏距离+固定 1.2m 门。
6. 杂病:R 冻结在 5m 工作点(距离相关噪声没进滤波)、q_jerk 全类共用且 MoverTracker 传不进去、感知 Track.miss 与 trk.miss 双账本分叉、SLIP 臂 360° 全知走 kf_movers 无 cam_heading(面不对称)、_man_cloud 的 tcpa 用常数巡航速度而 TDYN/CAP_MEET 用真实速度(同一相遇点两套钟)、vel_smooth 死方法+活副本、加速度估了全被扔(有 nuScenes CV 判决背书=合理弃用,但 cert_clear 与 cert_clear_warp 一个传 aa 一个清零=不一致)。
**总判决:"KF 无罪"的 07-14 结论补注——不是 KF 不行,是只用了它半张脸;行车线/余量缩放/radar 三条线全都要那半张没用的脸(P 与 NIS)。**

## 下一刀候选(待塔菲大人排序)
① ETA_FEED 不确定度缩放(用起 P 的第一刀,同时是薄面 8.5s 战果转正手续);② MoverTracker 开 radar 速度观测口(07-14 首选);③ predict 带 P 传播(行车线原料);④ nuScenes 三课全量回迁感知前端;⑤ KF_INIT=bayes 开关验证转正。

相关 [[sando-core-raceline-2026-07-16]] [[sando-core-kf-campaign-2026-07-14]]
