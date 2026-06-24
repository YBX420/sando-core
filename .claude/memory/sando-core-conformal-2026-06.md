---
name: sando-core-conformal-2026-06
description: "2026-06-24 overnight:把一直占位的 q_conformal=0 / 手设 v_eff=0.2 换成在真 MetaUrban 行人/车轨迹上分布无关 split-conformal 标定的 keep-out rho(t)=q_conformal+v_eff·(t+delta),test 覆盖率验证 ≥1-eps → 项目首次有 P(碰)≤ε 实测依据。新管线 metaurban/conformal_{harvest,calibrate}.py + replay_core.py + ab_replay.py + cert_ablation.py + predictor_compare.py + d435i_sensor.py。三大结果:① CV 预测器把行人 keep-out 砍半(v_eff 1.29→0.61 @95%,CA 加速度外推过冲);② A/B 真轨迹回放 ours 0/120 碰撞 vs native 19,用时中位持平/均值更快、全部 ≤native+3s(用户 6/24 把 bar 从 2s 放宽到 3s);③ 连续时间 Bernstein 证书 0 false-safe,离散采样 2 点漏 202/367 真撞=soundness 硬证据。D435i 深度相机接入(render --d435i,真深度图反投影喂 EGO)。证书 C++ 未改,ctest 25/25 + test_retime sound。"
metadata:
  type: project
---

**2026-06-24 overnight(用户 leave 前指令:先把 conformal v_eff + 那些都做了,最严标准,不做取舍,0 碰撞,用时 ≤ native+2s 后放宽到 +3s,不问选择题)。** 这一夜把 [[sando-core-win-ego-2026-06]] / [[sando-core-status-2026-06]] 一直挂的诚实漏洞 **q_conformal=0(几何 margin,非概率保证)** 真正补上。承接 [[sando-py-conformal-cert]] 的理论。

**做了什么(全 headless、可复现,产物在 out/conformal/):**
1. **收割真轨迹** `metaurban/conformal_harvest.py`:headless 跑 SidewalkDynamicMetaUrbanEnv(关 image_observation→纯物理),记 20 场景里 85 行人+车/机器人(ORCA 会变向)的逐 0.1s 轨迹。**关键:合成 scenario.py 的 mover 是匀速→CA-Kalman 残差虚低→标定不可信;必须用真机动轨迹。**
2. **split-conformal 标定** `conformal_calibrate.py`:残差 e(Δ)=‖真−预测‖按"距上次检测流逝时间 Δ"取每-Δ 有限样本分位 q̂(Δ)(rank=⌈(n+1)(1−ε)⌉),拟合仿射上包络 q_conformal+v_eff·Δ ≥ q̂(Δ),**按 track 切 cal/test**(尊重轨迹内相关性),test 集验证边际覆盖。per-class(行人/车)。**推导:cert 保 ‖p−c_pred‖≥R+rho 且 conformal 保 ‖c_true−c_pred‖≤rho 概率1−ε ⟹ P(‖p−c_true‖<R)≤ε。**
3. **终值(CV、eps=0.05):行人 q=+0.125,v_eff=0.612,覆盖 0.952;车 v_eff=1.587,覆盖 0.964;all v_eff=0.669。** 旧手设 0.2 欠覆盖近 3×。
4. **预测器反选** `predictor_compare.py`:keep-out=残差分位 → 更好预测器=更小 v_eff。**CV(常速,丢噪声加速度项)把行人 keep-out 砍半(0.61 vs CA 1.29 @≥0.95 覆盖)**,因 CA 对走停行人加速度外推过冲。**部署切 CV**(kf_tracker.predict(model="cv");replay/scenario 默认 EGO_VEFF=0.61、EGO_QCONF=0.125)。
5. **A/B 真轨迹回放** `replay_core.py`+`ab_replay.py`:真 mover 当**不让路录像**(strict),无人机飞穿走廊,ours=KF(CV)+per-class conformal 证书门控机动 vs native=贴 0.3 莽撞 EGO。**关键数字:native@3.0 自己就撞 19/120(16%)。** 三重对照(120 ep,eps=0.05,CV):**① ours@3.0 vs native@3.0(同速):ours 0 碰撞、native 19,用时中位 +0.00(持平)→同样时间 ours 安全 native 不安全;② ours@4.0 vs native@3.0(主,`--ours_speedup 1.33`):ours 0 碰撞、用时中位 −1.20s、均值 −1.47s、103/118(87%)严格更快、全部 ≤nat+3s(max +1.2)→"敢飞更快但安全";③ native@4.0 对照:提速后撞更多(确认 ours 的速度是认证安全独有的)。** 用户 6/24 追加要求"用时中位<0"→ 用 `--ours_speedup`(ours 用更高速度上限,证书门控每个 commit 保安全,native 用不起)达成。验收三项全 PASS:0 碰撞 + 中位<0 + ≤nat+3s。
6. **soundness 消融** `cert_ablation.py`:500 案例(367 真撞),连续 Bernstein 证书 **false-safe=0**;离散采样 2 点漏 202、3 点漏 107、5 点漏 25、9 点漏 5=**采样会穿越漏撞,连续证书不会**(核心创新硬证据)。
7. **D435i 感知输入** `d435i_sensor.py`+`render_3d_video.py --d435i`:挂 D435i 规格 DepthCamera(FOV 87×58/量程/轴向噪声)于无人机鼻锥(FPV 位姿 −90/−12),渲真深度图→度量化(near/far 反演)→反投影世界点云→喂 EGO 替代 GT fov_cloud(遮挡/量程/噪声=sim2real)。单帧验证 6575 点有界前向云;全渲染闭环+用 D435i 云重标定待跑。同接口可经 pyrealsense2 接真硬件。

**坑/教训:**
- **双重后台(nohup & + run_in_background)→ 子进程被提前收掉**,只用 run_in_background、别再 `&`;查进程死活**用 `ps aux` 直接看 PID**,pgrep 有时序误判("DONE"假象)。
- map 用 MetaUrban 世界坐标(几百米)→ EGO grid 上亿体素卡死;**回放平移到走廊中点局部帧 + 小地图**。
- 回放 episode 必须**保证对抗**(无人机和 anchor crosser 同时到路口中心、走廊垂直 crosser 速度),否则全 straight 不区分。加 FOV 过滤(14m)+ 早停(40 tick 不接近目标)防退化空转。
- 文档 `docs/conformal-results-2026-06.md`(含 0.5 相关工作:CP-SIPP/Safe-Interval/conformal-set/intent-MPC/time-optimal 的精确差异 + 参考)。证书 C++ **没动**,ctest 25/25、test_retime 0 unsound。

**对外措辞(诚实):认证安全(0 碰撞、净空中位 ~1.5m)+ 时间有竞争力(中位持平、均值更快)+ 分布无关 P(碰)≤ε**;边际覆盖非条件覆盖、exchangeability 假设、回放 mover 不让路、感知标定仍带噪 GT(D435i 闭环重标定是下一步)。相关:[[sando-core-win-ego-2026-06]] [[metaurban-render-recipe-2026-06]] [[sando-py-conformal-cert]] [[sando-py-sim2real-fakes]]。
