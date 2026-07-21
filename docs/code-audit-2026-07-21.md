# 全量代码重审病单(2026-07-21,sweep21 战役第一册;渲染压测另册 ledger.tsv)

范围:飞行关键路径全文重读 = safety_layer(1605)/render_3d_video(3222 关键区)/kf_tracker(434)/
perception(370)/collision_attribution(172)/replay_core(1027 后段)/ego_bridge(252)/C++ 证书核定点。
结论:**没有发现现役的 soundness 洞**;抓到 1 个休眠雷、1 个边角无证飞行支路、若干死代码/化石。

## P2(边角,建议解冻时一并处理)
1. **执行器"爬升脱困"支路是无证飞行** — render_3d_video.py:2920-2928
   `ego_stuck>=3` 时无条件 `quad.step(rec…)` 爬 0.6m/tick。maneuver 臂在"从未产出过样条"
   的角落(开局即被围,ego_dur=0)也能触发。账面诚实(exec_src=hold → 碰撞记 U),但违反
   flown==certified 字面律。gtxy 案里 ego_ok 恒真所以没走到它;属旧 raw 臂时代遗产。

## P3(休眠雷,一行修,解冻时改)
2. **slew 复证的 delta 默认错** — safety_layer.py:1572
   `d = (delta if delta is not None else 0.0)`;core 的同名默认是 tau。现役三个调用点全部显式
   传 delta(DELTA/_sub/REPLAN_DT)→ 不触发;未来任何 delta=None + SPEED_SLEW=1 的调用,
   换挡复证会用 0 延迟垫(乐观方向=不安全侧)。修法:`else tau`。

## P4(设计债/化石,记档不动)
3. cert_verdict3 无 v6.1 尾流旁证(safety_layer.py:331-355)→ 诊断孪生对 wake-certified 候选比
   闸门更悲观;只影响 replay 诊断线(replay_core:973),不伤 U/P/Q/D(判决走收据)。
4. `_cap_behind` 在 cert_clear 里用未裁剪 tau(safety_layer.py:290)而珍珠用 _tw;C++ 窗口
   fail-closed 兜住 → 短样条上尾流旁证永不通过 = 只保守。
5. winner-restore 只在 s_ok 复证(safety_layer.py:1362),复证失败直接 evade 而不降挡重试
   —— 保守,可能造成可避免的 evade。
6. 渲染面 v2 调用不传 cyl_ids(render_3d_video.py:2137)→ 收据快照 track_ids=行索引;
   归因靠 1.5m 空间匹配兜底。文档化的已知债。
7. 收据快照 xy 圆整 2 位小数(safety_layer.py:1523)→ tube_excess 的 Q/D 边界带 ±1.4cm
   圆整余隙;只影响归因分类,不影响飞行。
8. `MoverTracker.vel_smooth` 零消费者=死代码(kf_tracker.py:244;活的 EMA 是 _man_cloud 的
   _VF_STATE)。死加载器家族,建议清除。
9. perception.py:205 `d_eff` 算完没人用(内含 `r*0.0` 编辑化石);Track.update `_pre=None` 死变量。
10. `_GTXY_TRK` 跨圈不清零(render_3d_video.py:880)→ 单圈默认无害;--serve/loop 多圈模式下
    陈旧追踪器越积越多。
11. PERCLASS_CONF 以 d_safe 浮点值当类别键(render_3d_video.py:1963/2105)→ 两类共享同一
    d_safe 时会串班。现役类别 d_safe 互异所以不触发;换标定表时是暗雷。

## 归档级(非现役代码)
12. overnight.log 的 56 个 Traceback 全部定性:39×tune_difficulty.py:102 断言(07-03 场景工作台,
    已退役,仅自引用)+ 14×批脚本单引号 `$var` 未展开的 ValueError。非飞行代码。

## 存量 log 签名普查(死亡链无处不在的证据)
astar/in-obstacle/timeout/t3cp 四签名在 abr_batch(616/26/330/14)、afterlog_s12(292/240/51/131)、
afterlog_s23(321/2/320/15)、overnight(984/1325/2/4982)全部大量存在 —— 四环死亡链是
**系统性背景病**,不是 gtxy_s7 特例;07-08 前的 oldlog_* 全零(人群 idle 配置差异,非代码回归)。
