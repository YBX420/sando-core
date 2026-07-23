---
name: sando-core-audit-fixes-2026-07-20
description: "外部评审 10 条验尸+修复战役(f89ceba..8388066):9/10 属实;修①④⑥⑦⑧⑩(4 commits);①6-seed AB 混合判按家法转正 verbatim(A/B 案待塔菲大人复核);④账本口径变更(4/700 追溯改判);⑨反转=现役 v6 产线统计单元病+三折场景污染(THIN 不受影响);②不修=设计,③⑤=MINCO 停车场清单"
metadata:
  type: project
---

# 2026-07-20 外部评审验尸 + 修复战役(M3 冻结令当日,四 commit:13e9269/9d974de/87ed0f4/8388066)

## 评审 10 条判定(9 属实 / 1 定性错 / 2 条作用面标错)
①slew 低通产生无证格间档=**属实真 soundness 缝**(slip 分支早有 "RE-CERT THE FLOWN SCALE" 正解,唯 maneuver 释放坡道漏);②evade/HOLD 绕过安全门=**事实对定性错**(是设计的 RTA 兜底,rta.certified_ticks/counts 早已分账,干净抵达指标本就计罚;逃向次近障碍=已知局限,逃生树是正解);③recovery_climb 无检查=属实但作用面=**搁置 MINCO 臂**非现役;④测量口径=属实最重(中心 vs 机体 + 拍首 mover vs 拍末机);⑤MINCO 默认无连续证书=属实已知已文档;⑥ELLIPSE×ST 小圆误证=**属实**(椭圆沿轨半轴 κ·r_geom+q > 行 R,连 _ell_of "回退 sound" 注释都错);⑦NaN fail-open/长整型溢出/静默缩窗=属实(NaN→hull 停 -inf→假认证,机理实锤);⑧CALIB_EPS 不贯通=属实(bench_shard/probe_v3 设 0.10 实飞 0.05,仅 shield.py 接通过);⑨phase5 统计单元=属实且**反转升级**(见下);⑩单段空数组=属实。

## 已修(全部验证过)
- **13e9269 证书入口硬化**(⑥⑦⑧⑩):st_cert 单段守卫;certify_profile 椭圆行 fail-loud+safety_layer 启动断言禁 ELLIPSE×ST;calib loader 五处 eps=None→CALIB_EPS env→0.05(键名 '0.1' 与 str(float) 匹配已验);bernstein_cert.hpp g_segs_guard(NaN/阶>30 大声 fail-closed)接进两个通用入口+cert_bridge python 同款。**ctest 27/27,回归 12/12 逐字节同金哈希 5150b71,四守卫冒烟全开火**。
- **9d974de 测量口径**(④):replay_core/_clearance 减 R_DRONE+t+DT 测量;ego_maneuver 自带复制品同修(mover 先推进再量);渲染面 step_env 后 feed(None,..) 纯读刷新(渲染面 clearance() 本来就减机身——**三面口径原本劈叉**,评审只看到 replay)。**铁证:crossers s11 全 tick 历史 pre/post 轨迹字段 0 差异/clr 字段 31 差异;12 键 reach/coll 零翻转,Δclr≈−0.25±时序**。场景生成 clr0 门刻意不动(场景池不漂移)。
- **8388066 释放坡道**(①):EGO_DECIDE=v2 时执行器逐字飞锦标赛档(v2 内部本有认证的 one-grid-step 释放,渲染器 EGO_G_RELEASE 是双重平滑且产无证格间档);v1 遗留臂保旧;STC/GAPSPEED 分支原样。**6-seed AB(THIN,headless):均值 t 10.37→9.78(−0.6s),4 胜(s7 −0.8/s12 −4.0/s19/s23)2 负(s3 +1.4/s5 +2.1),12/12 干净抵达,净空普遍变贴(被删的是无证余量)**。成片:out/ab_audit1_release_s12.mp4(14.5s 爬直线 122 tick vs 10.5s)。**B 案留档:对 slew 档逐拍复证(slip 模式)——若按 seed 方差改判,一行换回**。
- 87ed0f4:cert_capi.so 重编入仓。

## ⑨ 反转(本战役最重发现,超出评审指控)
**现役 v6(calibrate_capsule)产线同病且更重**:foldE 98 flights 挤 49 scenarios(每场景 2 sensor seed,分位单元=flight 错)+ **designCR/foldE/testE 三折共享同一 49 场景**——形状拟合/秩标定/覆盖测试全见过同批几何,testE 覆盖率只证"同场景新噪声"不证新场景;phase5 明文写过的按场景切分教训(07-03)没被 v6(07-13)继承。**THIN 包不受影响**(手设薄壁,明确不主张 conformal 覆盖);中招=calib_v6.json 的 P(碰∧certified)≤ε 主张与 345 集判决面。修复=场景级 sup+场景不相交折(n≈49,q̃ 变肥)或分层 conformal——**需重标定+重跑基准,单独战役,待塔菲大人拍板**。

## 账本口径变更(全体后续读账者必知)
min_clr 自 9d974de 起=**机体净空**(旧=中心净空,差 0.25+时序);渲染/replay/ego_maneuver 三面统一。追溯:最近 700 集账本 4 集(全在 ell_cap6/cap61 波,0.6%)旧口径 min_clr∈(0,0.25) 应改判碰撞;345 集全量追溯审计未做。渲染面行为账本(exam/_sight30)自 8388066 起不再逐字节复现(①为行为面修复,预期)。

## 不修与停车场
②按设计不修(真待办=逃生树转默认判决,原有);③⑤=MINCO 复活前置清单(recovery_climb 过证书门+deficit cert 转默认);⑦缩窗=文档化+桥注释(改判定会动现役行为,未动);v1 臂释放坡道保留旧无证行为(遗留复现臂,已注明)。

## 裁决落地(07-20 晚,塔菲大人)
**①A 案定案**(钦定理由:释放坡道改 s(t)=改时空轨迹,不对完整 schedule 重证就不许飞;safety_layer:543 M3 配对法同源)——B 案废;**②k10 转默认**(ETA_FEED 默认 "0"→"2",ETA_K=1.0;oracle 臂内部硬零不受毒;ETA_FEED=0 逐字节复现旧路径已验);**③新 6-seed 基线转正**=out/baseline_render6_2026-07-20.json(新默认+新口径:均值 9.45s,6/6 干净;阶梯 旧低通 10.37→verbatim 9.78→+k10 9.45;k10 救回 s3 10.5→8.8/s5 14.2→9.8、s7/s19 双 7.5 贴全知地板 7.4,税 s23 7.5→11.5/s12 +1.1)。
**新钦定优先级:先解决"有限样本"(v6 场景级一期)与"漏检事件的定义"(覆盖主张的事件代数)**,再进大重跑。
仍待拍板:345 集追溯审计范围(建议折进大重跑);v6 重标定二期(一期数据出来后)。

## 有限样本一期结果(07-20 深夜,工具=metaurban/v6_scenario_ladder.py,纯重排零新飞行)
三档诚实阶梯(ped-M q̃@0.3s 为量规;production 复现=RUNG-1 λ=1.841 与账面逐字,**λ 门重算旧欠账就此结案**):
- **RUNG-1 现状**(episode 秩+同场景折):ε=0.05 λ=1.841 q̃=0.766,**test 覆盖率实测 0.938<0.95、ε=0.10 面 0.847<0.90——现状面连自家考卷都不及格(新发现)**
- **RUNG-2 只修单元**(场景秩 n=49,折仍污染):ε=0.05 λ=2.084(+13%)q̃=0.867;**ε=0.10 的 λ=1.841=旧 ε=0.05 的 λ——旧 5% 管其实只配叫 10% 管**
- **RUNG-3 全修**(场景秩+场景不相交折,fit16/cal21/test12 分层抽样):ε=0.05 λ=4.336 **MAX-RANK 零松弛**,ped q̃ 1.232(+61%)、veh 2.552(×2.4);单切分方差大,数字是"这一刀"的价不是终价。**结论:49 场景撑不起场景级 ε=0.05**
- phase-2 三选项待拍板:①收场景(产线现成,最干净)②主张降到 ε=0.10 ③场景级 jackknife+/CV+ 吃满 49 个单元(理论保证打折到 1−2ε)
- 漏检事件代数草案已提交塔菲大人(E1 认证覆盖违约=ε 赔/E2 无证暴露=RTA 账/E3 感知盲=漏检锥外遮挡/E4 仅限声明分布漂移;判决序 E2→E3→E1;**"物理包络"不是合法类别,必归 E1 或 E3,n3 案=第一个重验尸对象**);批准后落地=账本 per-collision attribution 字段

## 深夜裁决落地(07-20 深夜,塔菲大人细则,029836a..188028a)
**phase-2=①+②批准,③降附录敏感性;事件代数改判 U/P/Q/D + 正交 domain 标签(ID|OOD|UNKNOWN 按 episode 冻结)**:U(无有效证书绑定执行)=RTA 账/P(证书有效但肇事物不在 snapshot)=感知账/Q(在 snapshot 内但真轨迹出管)=ε 赔/**D(在管内仍撞=确定性证明链违约,零容忍,防代码 bug 混进 ε)**。正式定理形态:P(Q)≤ε;C_cert,represented⊆Q∪D;**须验证 P(D)=0 后才能推 P(C_cert,represented)≤ε**。论文措辞禁"distribution-free",写"model-free、场景可交换性下成立"。
**已落地(先决件,attribution 之前)**:
- **certificate receipt v1**(029836a):make_receipt 每决策拍一张(plan_hash=持有样条+档/schedule、障碍 snapshot 行 (tid,x,y,R)+sha、calib 文件 sha+eps、window/delta、gate 标签);公开 maneuver_decide_v2=receipt 壳(不变量:非 evade 返回=已认证),SPEED_SLEW 壳改名 _decide_v2_slew 垫底(**SPEED_SLEW=1 就是现成 B 案机制,默认关**);replay certified_ticks 改 receipt 驱动(kind 字符串只做显示;v0/v1/v3 遗留路径保留 kind 记账明示二等;V2_ESC 逃生沿用 evade receipt=保守记无证);CRET 自发 receipt。**验证:DECIDE=v2 臂 41/41 拍带据(40 认证+1 evade)、轨迹逐字节不变;⚠️ 字节回归面默认 DECIDE=v1(冻结锦标赛),receipt 在回归面只覆盖 sub-cadence**
- **swept-contact**(同 commit):_swept_clearance=[t,t+DT] 双弦最小机体净空(xy 二次极小+柱顶 z 穿越+端点);12 键 reach/coll 零翻转、新金哈希 84ae78d(029836a.json)
- **基线 artifact 修订**:钉死 5343faa+calib sha b8671ed818c67764+manifest(metaurban efbc6ad);zero_evade 替换错误措辞,增列 uncertified_exposure_ticks(s3:1/s12:6/s23:6=U 类暴露)
- **阶梯 v2**(188028a,四硬条件全落):RUNG-2 降级为管宽等价诊断;strata=名族+manifest 世界组成预注册(**全 49 场景只有 6 个含静态物**,纯抽签 ~9% 饿死 static 拟合,rng17 踩中;数据窥探强塞已删);cal/test 统一 2 episode/场景;有限样本措辞修正。**修正版 RUNG-3:ε=0.10 λ=1.522(k=19/20 带一格尾松弛)ped q̃@0.3=0.838=比污染版 production 仅 +21%(旧 λ=4.3 是窥探切分的假吓);ε=0.05 max-rank λ=2.822 实证需 ≥39 cal 场景(荐 ≥59,camera-ready 档);全 episode 敏感性 λ=1.755**;interim artifact=out/conformal/calib_v6_scn010_interim.json(切分全注册,INTERIM 待新场景+估计器冻结)
**下一步(按裁决序)**:attribution 落账(U/P/Q/D+domain 标签+漏检子码[锥外/遮挡/随机miss/关联失败/TTL串杀/太年轻]+object_scope+全接触物列表;P 判据=肇事物 spatial 匹配"最后一张因果有效 receipt 的 tracks 行",反应窗由感知/计算延迟+最小制动导出,不手填);渲染面 receipt;新场景收割开工

## 工序 1-4 已落地(07-20 深夜二批,268a3f8+6df07d0,塔菲大人最终工序)
- **collision_attribution.py 共享模块**:replay+renderer 同一套 U→P→Q→D(U=无证书绑定实际执行/窗外/执行覆盖;P=不在 snapshot+子码;Q=真轨迹出管=ε 类;D=管内仍撞=确定性违约零容忍);辅助原因永不丢;domain 按 episode 冻结只注不改判;tube 检验=CV 点律+胶囊段律双模
- **receipt 补齐**:executed_segment_hash+exec_src(执行器真飞的东西:plan vs evade/brake/escape 覆盖);snapshot 行升级 (tid,x,y,vx,vy,R,veff,cap);渲染面 maneuver 臂 receipt 已接(决策时 fed 快照配对+静态碰撞=认证拍 D/无证拍 U)
- **验证链**:四类合成单测全对+中拍穿越(端点全清 s=0.5 抓到 −0.55);**强制碰撞探针 scenarios/diag/forensic_cross.json 端到端全对**(tick0=P/snapshot_gap:FE ready 门,证书真没看见新生 track;tick1=U/no_certificate+exec_override:evade;全 collider/track 态/cert_id 链/扫掠时刻齐);200k fuzz 新旧扫掠 0 失配;12 键 flags+clr 与 029836a 逐位同;渲染面 seed7 7.5s/2.041 逐字复现(全透明)
- **⚠️ n3 验尸结果=历史碰撞不复现**(ETA_FEED=0 钉回历史面,stc 臂 11.5s 0撞 clr1.55):旧碰撞是 flown≠certified 时代执行器的产物,三刀裁决修复(verbatim/扫掠/口径)之间行为合法变化;"认证 HOLD 被碾→U"模式在代数里就位待真实案例;渲染面碰撞路径=冒烟级验证(共享代数已由 replay 探针背书)
- **工序 5 已落地(同夜三批,commit 后续)**:5a winner-restore 重跑 replan+duration+extra_gate 全 gate(失败大声转 evade);5b 窗口补全=cert_clear/warp 终点悬停对余窗 [dur/warp,τ] 认证(hover_clear:Lipschitz 声采样+胶囊珠+冻结分量+椭圆外接圆超集;C++ 静默裁短洞关闭);5c HOLD 有证或明示 U(渲染 evade→hold 拍试 hover_clear,过则发 hold_cert 收据=s=0 珠飞行形态,attribute 认 hold+hold_cert 为匹配计划);5d v1 loader 静默手值兜底(0.15,0.6)改大声 1e6 fail-closed;5e executed==certified 逐拍验证(exec_verified,teleport 精确/dynamics 按 DYN_TRACK,违反大声+碰撞挂 exec_envelope_violation 证据)。**验证:12 键与金 268a3f8 逐字节同(冻结面纯保险)、探针判决不变、s12 逐字复现、hover_clear 五态单测全对**。MINCO recovery=停车场不动
- **工序 6 前两刀已落(d2fb9c0)**:**KF 病②=coast 状态传播改按认证假设 CV 走**(部署证书 PRED_MODEL=cv 而估计器 coast 用 CA 积垃圾加速度=内部自相矛盾;协方差保留全 CA 白 jerk 增长=保守超集;KF_COAST=ca 回旧;注入病理单测 v 钉 0.60 vs 旧漂 2.10,九项自测全过含精确合成律);**病③=PERCEPT_TTL 8→16**(确诊丢检串 7-8 拍正卡 TTL=8,confirmed 死在串尾重生进病①;tentative 的 YOUNG_TTL 短绳不动)。**行为:回归 12 键零碰撞+fast_canyon:42 从到不了翻干净抵达(串杀场景类痊愈);6-seed 均值 9.45→9.68,4 胜(s19 6.6s 新纪录/s7 7.3 穿旧全知地板 7.4/s5 −1.0/s23 −1.5)2 税(s3 +3.7/s12 +1.3)。★拆解反转:s3 税全是②的(③单刀=逐字节零差)——旧速度部分建立在 CA 漂移把 coast 幽灵以垃圾速度甩出走廊的不 sound 免费午餐上;诚实 coast 留住幽灵、σ 带(k10)负责任绕行=σ margin 重定阶段的正题**。承诺一瞥(stc s7 n3):两次 drop 全 dev=0.00(估计器噪声死法消失),一段活 1.4s(旧半衰期 0.45s)死于 policy_flip=架构侧——**M3 解冻协议(正式半衰期重测)就绪待跑**
- **遗留**:σ margin 再定(待塔菲大人:A 接受现状收/B coast 高龄 track 降级冻结小盘/C coast track 单独 ETA 系数);M3 半衰期正式重测(解冻门 ≥1.5s);**估计器未冻结,终版标定继续等**;P 子码细分需感知面事件挂钩;渲染面 exec_verified 待接

## A+ 闭合 + M3 正式判决(07-20 终批,d54675f=估计器候选冻结)
塔菲大人裁决=A+(四接口闭合)/B 不转默认(证书双查 predicted∧frozen,B 等于删一半)/C 名不副实(ETA_K 跳过 coasting,coast 真旋钮=MAN_MEM_K×pos_sigma→终版标定改叫 COAST_MEM_K)/interim ε=.10 永远叫审计产物。
**A+ 四件全落(d54675f)**:①hybrid coast=均值协方差同走 CV 传播,coast Q=白 jerk→a 通道精确积分+白加速度项 q_a=|a_held|²·τ_a(τ_a=2.0>记忆窗)⟹ **σ_v(T)≥|a_held|·T 被删漂移显式进二阶矩=保守性证出**;复检首拍 CV 预测(旧 CA predict 会把陈旧加速度回积一次);合成律逐位保持;KF 自测 11/11。②③TTL 改秒语义两面统一(confirmed 1.6s/tentative 0.3s 显式字段;legacy PERCEPT_TTL/YOUNG_TTL/EGO_MEM_TICKS 大声换算)——**渲染面病③首次真落地**(此前被 EGO_MEM_TICKS=8 压 0.8s)+**撤回:knife-1 的 fast_canyon 救活=replay 面意外 4.8s 记忆的产物**,诚实 1.6s 后回 not-reached(仍零碰撞)。④tentative 显式短绳+自测。
**s19 验尸(渲染面首例真实碰撞归档)**:A_FLOOR=1.0 的 Singer 热度把 coast keep-out 撑爆→走廊封死→无证 HOLD 冻成活靶→动物碾过,**判决 U[no_certificate, exec_override:hold]**(非 D,定理无恙);dominance 论证本不需要地板→**默认 KF_COAST_AFLOOR=0**(热度归终版标定);顺手修渲染归档兜底选肇事者 bug(按最负类不按最近中心——否则静默漏记)。
**候选冻结面数字**:回归 12 键零碰撞(fast_canyon:42 诚实 not-reached);6-seed 6/6 干净均值 11.73s(统一 1.6s 记忆在渲染面的实价,s5 hold:65 冻挡丑但净;定价=COAST_MEM_K 归终版标定,不 6 seed 手调)。
**★M3 正式半衰期套件(6 seed stc,12 commits)**:**median 0.50s,12/12 失败降(gate×7 全 dev=0.00/betray×3 dev 0.82-1.31/cert×2),零自然完成——<1.5s 门,不过**。机制读数:估计器噪声死法确实没了(gate/cert 死全 dev=0),新主死因=滑动窗重证 gate 撞上诚实增长的 coast 管(架构×诚实不确定度的相互作用,不是估计器 bug)。**按裁决:M3 保持关闭;估计器冻结在 d54675f;终版标定按 M3-off 策略走**(重产 residual→场景不相交统一 episode→新 ε=.10→收场景→camera-ready ε=.05)。早前 1.4s 一瞥=2 样本茶渣,正式 12 样本说话。

## 收割前缺口关闭(07-20 末批,a84e0f5+2c10526;tag=estimator-freeze-2026-07-20 已钉 d54675f,244M bundle 备份 ~/sando-core-backup-2026-07-20.bundle)
塔菲大人裁决细则全落:**harvest 契约重建**(harvest_v3+ops/harvest3.sh:一集=一解释器=一原子文件 tmp→sha256→rename→receipt;预注册 jobs.json 永不截断;ep==job index;U64 场景名;单 worker+BLAS=1;版本化 run 目录;resume 需 data+receipt+checksum+job 四配;merge 作业集全等否则拒;**合法零行集留宇宙计 -inf**;receipt 钉全部 env 旋钮+generator commit+schema 版);**残差 schema +psig/coast_s/a_held/mem_m**(EGO_MEM_K 分析前提);**生成器修**(--contested 真门控;pushback 后 retimed movers spawn_t 平移 back/v_nom+重验起飞窗;无安全起点=REJECT;时空 encounter 验收 _encounter_ok;场景 provenance 全录;multistart 同世界=一 cluster);**fail-loud TTL≤τ_A 两面断言**;**肇事者身份闭环**(fed 行带稳定 oid;clearance 返回逐对象列;扫掠按 oid 配对;记录名对象+全接触物各自净空;补修第二个 clearance 调用点)。
**pilot12 验收(工序3)**:acc1 当场抓获 **episode-id 偏一真身**(run_replay 开局 _HARV_EP+=1,历史所有 pool 行 ep=manifest+1,池内统计幸存但 manifest 交叉引用一直错;merge 全等校验以"16 幻影零行集"报警)→根修(replay 不得篡改调用者身份)→acc2:24/24 零失败、行数两轮逐字节同 39338、**9 个真实零行集全 gen_pool 家族**(=真"消失场景"人口,现可见计 -inf)、resume 删收据→23 SKIP+1 重跑→merge 依旧全等、新列活。
**待办(design69 开机前)**:9 个 gen_pool 零行集的 eligibility 诊断(为何全程零合格行——生成器 eligibility 验收范畴);然后工序4=重收旧 69 定 EGO_MEM_K→stack-freeze-m3off tag→工序6=一次性生成预注册新 100 场景(60cal/40test,四家族等权冻结 RNG,动物留 stress suite)→cal 只算分位/test 只开一次。

## 工序 4 完成(07-20 终批二,a84e0f5..此后):design69 + EGO_MEM_K 冻结候选
- **零行诊断实锤**:gen_pool_00 全程离 mover 最近 25.8m、零拍进感知圈=**空相遇场景**(老生成器 stale-timing 遗孤;新 _encounter_ok 验收正为杀此类)。
- **design69(v3 契约)**:138/138 零失败、98040 行、**44 零行集=32% 舰队空相遇全现形**(老宇宙实际只 2/3 在参与排序)。
- **mem_k_fit(23k coast 行/49 场景,e/psig 设计面 q90)**:行人 q90=1.62/q95=2.28→**历史 K=2.0 盖 q93 保留**;**★vehicle q90=13.3/q95=18.0**——收敛车 track coast 时 a_held≈0→A+ 白加速度项消失→psig 严重低估("KF 协方差不是概率保证"的测量版);psig 小所以 13.5×psig≈2m 合理。**冻结档:MAN_MEM_K=2.0 默认+MAN_MEM_K_CLS[vehicle]=13.5(EGO_MEM_K_VEH)**,在任何新场景 cal/test 存在之前定死。coast-age 结构(0.3-0.8s q90=2.05 vs 0.8-1.6s q90=0.67)=单 K 是粗包络,taper 留下代。
- **验证**:6-seed 5 个逐字节不变,s5(车重)18.3→23.5s=堵车辆真洞的代价,6/6 零碰撞;replay 回归面不涉(renderer 独有档)。
- **待塔菲大人**:①K 表冻结批准(接受 s5 代价 vs 车辆律改形);②stack-freeze-m3off tag;③工序 6 开机(新 100 场景一次性预注册生成)。

## 07-21 三否决裁决 + 最短路径执行完毕(b9b9078..fea940b)
塔菲大人三个都不批,病灶全中:**K=13.5 撤回**(拟合把全视界 e(d)/psig(0) 混池,而运行时 K 只买 d=0 锚——量纲错;分层真相:锚 d=0 q90=0.873全体/0.845行人/**1.225车辆**,13.3=长视界(d=1.05 阶梯到 2.79)+young 冻结预测器(young 4.6 vs mature 1.41);外加 0.30s 拟合 vs 0.10s 应用、只接 realistic 路径漏 GT-keyed、与 conformal q+v_eff·t 重复收费)→**统一 K=2.0 作工程锚 buffer 保留,"车辆洞已关闭"撤回**,重拟前提+可追溯账本=out/baselines/mem_k_evidence_2026-07-21.json;零行集"可见≠参赛"(flight_sups 只枚举有行 episode);44/138 是 episode 比例,场景级=**20/69 全空+4 半空**(正合原判"约20个消失")。
**最短路径三步全落**:①K 恢复 2.0(s5 逐字节复现 18.3=撤回精确);②**证书窗口覆盖契约**(C++ g_window_covered:有限窗必须 t≤0 起连续铺满否则 fail-closed 大声;五个门函数显式裁剪 _tw/_tws,尾窗由 hover 律背书——**12 键与候选金逐字节同=零行为变化关掉 E1 毒洞**);③**零行 calibrator**(flight_sups(universe=):design69 94→138 flights、参赛场景 49→69)+**final100 契约建成未开机**(final100_plan 预注册:100 slot/家族冻结 RNG iid 或 --balanced 25×4 两注册选项/60-40 按 slot 冻结/root seed+重试律 root+1000k<50/注册 map-seed 集/stack SHA 必填离树拒发;final100_gen git 锁+原子场景+收据(实际 seed/attempt/全 reject 记录/scenario+stack sha)+resume 三配+耗尽写 FAILED 收据不补位+压力帽 1.5;ops/final100.sh 单 worker 每进程 5 slot)+**生成器内核四闭**(encounter 查全部交点+弧长记账;shift 后重验 min_pair 间距;provenance 记实际 attempt seed;bench 防覆盖+MANIFEST 合并不截断)。
**现状**:栈在 fea940b,证书窗/K 口径两个 tag 阻塞项已关;**待塔菲大人:①iid vs balanced 家族方案二选一;②stack-freeze-m3off tag 批准(钉后由冻结树发真 plan);③工序 6 开机**。设计面重收(design69)+锚律证据+零行参赛全部就绪;M3-off 终版标定路径畅通。

## 07-21 第二轮三否决 + 六步全落(f783ea3+7cab276)
裁决:家族=**iid_equal_prob 定案**(主张=四家族等概率生成分布的 pooled conformal;18/38/26/18 合法不得重抽、不得称"经验平衡";25×4 只配"有限 cohort"更窄口径);tag/开机再否(三硬伤+四链伤)。K=2.0 验收通过,撤回账本保留,age-taper=下代。
**六步执行**:①**u_stop 统一**(唯一停点=max(dur−1e-3,0)=执行器冻结点;样条证 [0,u_stop]、终点 eval(u_stop)、hover 从 u_stop/warp 起——1ms/warp 无证缝合上;12 键与候选金零差=逐字节不变);②**verdict/margin 接尾窗**(verdict3 尾不过 certified→unknown 防 E1/U 污染;两 margin 的 ok 也 AND 尾窗——"五接口背书"的虚标改为真);③**MINCO 入口关窗**(certify_traj_vs_sphere 有限窗超轨迹端 fail-closed;"核心永不缩窗"现在无例外);④**stack_manifest**(--emit 记三 .so SHA-256+全量源 git SHA+MetaUrban 环境 SHA;verify fail-closed;ego_bridge 在 STACK_MANIFEST 下 dlopen 前核 .so;harvest 收据永远内嵌 runtime_shas;发→验→带核验渲染 s7 逐字复现全链通);⑤**final100 硬化**(plan 不可覆盖+全 SHA+plan_sha 自校验+内嵌 stack_runtime;gen 全 git 锁+运行时二进制锁+draft 隔离 namespace+resume 五配 plan-bound+FAILED 收据带 plan_sha;driver 100/100 否则退出 3);⑥**链路接通**(harvest_v3 final100cal/test 模式读注册 plan、缺/异/败一票拒收、路径从 plan namespace 解析;calibrate_v3 主程序 --universe-cal/--universe-test 让零行集在真 calibrator 投票)。bundle 已刷新(07-21,244M)。
**下一步(待塔菲大人静态复核)**:在新 HEAD(7cab276)钉 stack-freeze-m3off tag → 冻结树 emit stack manifest + 发 IID 正式 plan(immutable)→ 单 worker 每进程 5 slot 开机 → final100cal/test 收割 → calibrator 带双 universe。

## ★★★ 07-21 发射日:final100 战役全程收官(塔菲大人"走"令,..85f7cb9)
**tag=stack-freeze-m3off-2026-07-21 钉 bbfdfd6**;官方 stack manifest(三 .so SHA+双 git)发出。**三次现场抓病三次锁之舞**(每次:停机→修→commit→新 plan→旧产物由 plan-bound resume 自动作废):①MetaUrban 引擎单例(同进程二次 build_env 断言)→close-on-switch;②clear-start 饿死(165/300 拒;**旧生成器只 WARN 就在车流里起飞=退役池诚实性注脚**)→退避 32m+横向 ±2/±4 探点;③contested 家族 0/6 全灭(单锚无第二走廊+横向探点破坏平移近似对时)→**锚级走廊轮换 + 精确重对时**(v4 后 12/12 全 ok 四家族复活)。
**战果链(全部收据可溯)**:100/100 slot 生成(plan_sha 52bd39b5,iid 18/38/26/18,60/40 slot 级冻结);cal 120/120(43901 行,2 零行)、test 80/80(26487 行,5 零行)——**相遇合格率 98%+ vs 退役池 32% 空相遇**;逐集 ego_capi SHA 核验全过(⚠️ 外层 verify 被 `|tail` 管道旁路=流程 bug 记档;git_sha 失配=tag 后纯工具链 commit,飞行源码+三 .so 自 tag 未动,收据链可证)。
**★论文校准(设计面形状+场景级秩+宇宙全员投票+场景不相交 by construction)**:**ε=0.10:λ=2.241(k=55/60,松弛 5);ε=0.05:λ=2.525(k=58/60,松弛 2)。test 开考一次即封:ε=0.05 实测覆盖 0.950(38/40)正中名义;ε=0.10 实测 0.875(35/40)=分辨率 0.025 下低名义一格,Binom(40,.9) P(X≤35)≈0.27 属正常观察非判决**。artifact=out/conformal/calib_v6_final100.json(全 provenance:plan_sha/stack sha+runtime/design npy sha/run 目录/test_opened=true;动物 STRESS_SUITE_ONLY)。基准 100 场景+收据已入仓;四份 plan 注册(a/b/c 作废件留审计链)。**camera-ready 备注:cal60 已满足 ε=0.05 秩余量(旧"需≥59"算术针对 49 池)——两档均为活论文档位,框架措辞归塔菲大人**。
**遗留**:九月主表余项(vs native 锥对齐 AB/新基线成片);ε=0.10 覆盖观察的解读措辞;verify 管道 bug 与 manifest git 严格度设计(逐集核验为承重件);M3 保持关闭;radar/估计器下代周期。

## ★★ 07-21 GT-xy 异常全案 + 决策解释链落地(6d193e3)
**起因**:塔菲大人要看 GT-xy 成片→冻结栈下首跑反常(12.8s vs 写实 8.6s,贴 3cm,hold 51)。**解释链**(塔菲大人令"log 里也存这些,验证可解释性"):EGO_EXPLAIN=1 把每次拒绝的 (方向,档位,击杀腿,肇事 mover) 穿进 receipt.explain+EXPLAIN_LOG jsonl(hover/_tail/cert_clear/warp 全 why 管道+锦标赛全候选矩阵);默认关=12 键逐字节零差。
**实证判决(v1 叙事撤回)**:冻结窗击杀 **replan×131 vs 证书×16(冻结盘 0、悬停尾 0)**——不是证书保守,是 **EGO 规划器在预测环+记忆管封死的占据格上产不出样条=规划器活性病**(正中旧债"空间规划器冻结占据,TDYN 只治半边")。**分岔 t=2.5 双臂仅差 0.9m:写实肥管→直行证不过→被迫 over→顺畅;GT 细管→直行@1.0 证过→speed-first early-exit 贪快→入袋**("噪声当战略保险"升实证)。冻结 20 拍中 19 拍=诚实 U 暴露(1 拍 hold_cert)。
**★复现性矩阵(第二发现)**:GT 面同配置四跑三结局(12.8/0.034 干净×2、−0.022 碰撞、12.7/0.228 干净),写实臂三次逐字节稳。**头号假说=EGO A* 墙钟计算预算**(重 replan 口袋里 131 次失败部分是时间预算死,仪器/负载微扰即改写轨迹;写实臂不进重 replan 区所以稳)——待验,属飞行代码不动冻结栈。
**结论**:GT-xy 臂论文禁入直至①口袋活性方向拍板(逃生树转默认/时空线解冻/考卷答案判口袋真伪)②墙钟预算钉死(确定性预算 or 记入 explain)。诊断三件套+解释 log 已交塔菲大人(gtxy_s7_diagnosis_v2.md/explain_*.jsonl)。**可解释性主张有了实证形态:每一次不飞都有名有姓、机器可查、跨面可 diff。**
**★代码级四环死亡链(成片 rollout 同源钉死,file:line+log 计数)**:环1 喂入=渲染器 mover 预测环(render_3d_video ~1755)**无"挖无人机气泡"机制**(clear_spawn 只管出生点静态),3-4 mover 收敛≈1.1m 环半径内→悬停点被占据格吞;环2 初始化=bspline_optimizer.cpp:754"drone is in obstacle"(×239)+ **:995 前 3 控制点在障碍 return false**(×87=replan 假第一来源);环3 前端=dyn_a_star.cpp:245 **0.2s 墙钟(ros::Time 实钟)预算**烧完 25 万迭代 return false(×66;:114/:795"a star error"×284=上层回声)——**墙钟=四跑三结局的不确定性源,岔口本身两跑一致(结构性)**;环4 决策=七方向共享中毒起点→7/7 每拍全灭(119)→无逃生树→无证悬停→碾。写实臂免疫因 t=2.5 肥管逼上 over 不进收敛区。修复靶点各环一个:喂云挖气泡(planner-only,cert 不吃此云=soundness 好论证)/free-start 语义/A* 预算改迭代数(2 行,确定性)/逃生树转默认——全属飞行代码=解冻级待批。

相关 [[sando-core-m3-2026-07-20]] [[sando-core-raceline-2026-07-16]]
