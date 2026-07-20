---
name: sando-core-audit-fixes-2026-07-20
description: "外部评审 10 条验尸+修复战役(f89ceba..a93ab2a):9/10 属实;修①④⑥⑦⑧⑩(4 commits);①6-seed AB 混合判按家法转正 verbatim(A/B 案待塔菲大人复核);④账本口径变更(4/700 追溯改判);⑨反转=现役 v6 产线统计单元病+三折场景污染(THIN 不受影响);②不修=设计,③⑤=MINCO 停车场清单"
metadata:
  type: project
---

# 2026-07-20 外部评审验尸 + 修复战役(M3 冻结令当日,四 commit:213a19d/d32b63b/d4e66eb/a93ab2a)

## 评审 10 条判定(9 属实 / 1 定性错 / 2 条作用面标错)
①slew 低通产生无证格间档=**属实真 soundness 缝**(slip 分支早有 "RE-CERT THE FLOWN SCALE" 正解,唯 maneuver 释放坡道漏);②evade/HOLD 绕过安全门=**事实对定性错**(是设计的 RTA 兜底,rta.certified_ticks/counts 早已分账,干净抵达指标本就计罚;逃向次近障碍=已知局限,逃生树是正解);③recovery_climb 无检查=属实但作用面=**搁置 MINCO 臂**非现役;④测量口径=属实最重(中心 vs 机体 + 拍首 mover vs 拍末机);⑤MINCO 默认无连续证书=属实已知已文档;⑥ELLIPSE×ST 小圆误证=**属实**(椭圆沿轨半轴 κ·r_geom+q > 行 R,连 _ell_of "回退 sound" 注释都错);⑦NaN fail-open/长整型溢出/静默缩窗=属实(NaN→hull 停 -inf→假认证,机理实锤);⑧CALIB_EPS 不贯通=属实(bench_shard/probe_v3 设 0.10 实飞 0.05,仅 shield.py 接通过);⑨phase5 统计单元=属实且**反转升级**(见下);⑩单段空数组=属实。

## 已修(全部验证过)
- **213a19d 证书入口硬化**(⑥⑦⑧⑩):st_cert 单段守卫;certify_profile 椭圆行 fail-loud+safety_layer 启动断言禁 ELLIPSE×ST;calib loader 五处 eps=None→CALIB_EPS env→0.05(键名 '0.1' 与 str(float) 匹配已验);bernstein_cert.hpp g_segs_guard(NaN/阶>30 大声 fail-closed)接进两个通用入口+cert_bridge python 同款。**ctest 27/27,回归 12/12 逐字节同金哈希 5150b71,四守卫冒烟全开火**。
- **d32b63b 测量口径**(④):replay_core/_clearance 减 R_DRONE+t+DT 测量;ego_maneuver 自带复制品同修(mover 先推进再量);渲染面 step_env 后 feed(None,..) 纯读刷新(渲染面 clearance() 本来就减机身——**三面口径原本劈叉**,评审只看到 replay)。**铁证:crossers s11 全 tick 历史 pre/post 轨迹字段 0 差异/clr 字段 31 差异;12 键 reach/coll 零翻转,Δclr≈−0.25±时序**。场景生成 clr0 门刻意不动(场景池不漂移)。
- **a93ab2a 释放坡道**(①):EGO_DECIDE=v2 时执行器逐字飞锦标赛档(v2 内部本有认证的 one-grid-step 释放,渲染器 EGO_G_RELEASE 是双重平滑且产无证格间档);v1 遗留臂保旧;STC/GAPSPEED 分支原样。**6-seed AB(THIN,headless):均值 t 10.37→9.78(−0.6s),4 胜(s7 −0.8/s12 −4.0/s19/s23)2 负(s3 +1.4/s5 +2.1),12/12 干净抵达,净空普遍变贴(被删的是无证余量)**。成片:out/ab_audit1_release_s12.mp4(14.5s 爬直线 122 tick vs 10.5s)。**B 案留档:对 slew 档逐拍复证(slip 模式)——若按 seed 方差改判,一行换回**。
- d4e66eb:cert_capi.so 重编入仓。

## ⑨ 反转(本战役最重发现,超出评审指控)
**现役 v6(calibrate_capsule)产线同病且更重**:foldE 98 flights 挤 49 scenarios(每场景 2 sensor seed,分位单元=flight 错)+ **designCR/foldE/testE 三折共享同一 49 场景**——形状拟合/秩标定/覆盖测试全见过同批几何,testE 覆盖率只证"同场景新噪声"不证新场景;phase5 明文写过的按场景切分教训(07-03)没被 v6(07-13)继承。**THIN 包不受影响**(手设薄壁,明确不主张 conformal 覆盖);中招=calib_v6.json 的 P(碰∧certified)≤ε 主张与 345 集判决面。修复=场景级 sup+场景不相交折(n≈49,q̃ 变肥)或分层 conformal——**需重标定+重跑基准,单独战役,待塔菲大人拍板**。

## 账本口径变更(全体后续读账者必知)
min_clr 自 d32b63b 起=**机体净空**(旧=中心净空,差 0.25+时序);渲染/replay/ego_maneuver 三面统一。追溯:最近 700 集账本 4 集(全在 ell_cap6/cap61 波,0.6%)旧口径 min_clr∈(0,0.25) 应改判碰撞;345 集全量追溯审计未做。渲染面行为账本(exam/_sight30)自 a93ab2a 起不再逐字节复现(①为行为面修复,预期)。

## 不修与停车场
②按设计不修(真待办=逃生树转默认判决,原有);③⑤=MINCO 复活前置清单(recovery_climb 过证书门+deficit cert 转默认);⑦缩窗=文档化+桥注释(改判定会动现役行为,未动);v1 臂释放坡道保留旧无证行为(遗留复现臂,已注明)。

## 待塔菲大人拍板
①A 案(verbatim,已转正)vs B 案(slew+复证)——s3/s5 两退化 seed 是否值得换;②345 集追溯改判审计范围;③v6 重标定战役(场景级 conformal)立项与排期;④渲染面新基线数字(6-seed 表)是否替换 exam 账本。

相关 [[sando-core-m3-2026-07-20]] [[sando-core-raceline-2026-07-16]]
