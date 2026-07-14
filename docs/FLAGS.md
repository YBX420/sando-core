# 环境开关注册表(2026-07-07 stage-2 建立)

> 规则:新开关必须登记在此,带 A/B 判决;判决为"负/被吸收"的开关在下个大版本删除。

## 决策/平滑
| 开关 | 默认 | 判决 |
|---|---|---|
| `EGO_DECIDE`(渲染器) | **v2** | v2=统一(方向×速度)锦标赛,seed7/12 追平或反超 CCF;v1=冻结复现档 |
| `DECIDE`(headless) | v1 | v2 待 52 场景全台架 + #6 真 q 值后裁决(北极星指标全胜,用时 +40% 半为度量假象) |
| `SMOOTH` / `SMOOTH_DWELL` / `SMOOTH_MARGIN` | off | headless sticky:switches -10% spchurn -13% evade -17%;被 DECIDE=v2 上位替代,v2 转正后删 |
| ~~EGO_CCF / EGO_CCF_WARP / EGO_MAN_RELEASE~~ | — | **已删**(07-07):被 v2 吸收;CCF 本身 06-30 已实证退役(冻 2/5 seed) |
| ~~EGO_CRET_HOLD / EGO_CRET_SMIN~~ | — | **已删**:可转化 hold≈0(静物不减速) |
| ~~EGO_FOVCAP~~ | — | **已删**:注释自认等待的"anti-thrash 未来"= v2 |
| `CRET_GLIDE` / `CRET_GLIDE_SMIN`(headless) | off | **负结果**(evade -22% 但时 +22%、clr 0.55):留作 ablation,v2 转正后删 |

## 管道/标定
| 开关 | 默认 | 判决 |
|---|---|---|
| `SHIELD_PERCLASS`(RL) | off | **三项全胜**(超时-40%/到达+2.3/碰撞持平)→ 候选转正默认 |
| `SHIELD_EPS` | 0.05 | 0.1 档:坏交易(超时只-4,碰撞+0.75pp) |
| `RADIUS_CONSIST` | 0 | full=封廊否决;dsafe 单独赢(evade-14%),与 SMOOTH 冲突,#6 后裁决 |
| `VERDICT3_LOG` | off | 纯诊断(evade 拍三值判决入 hist) |
| `PERCEPT*` / `PRED_MODEL` / `EGO_TRACK` 等 | 冻结值 | 部署在环标定绑定,动=重标 |

## v_cap(task#4,2026-07-07)
| 开关 | 默认 | 判决 |
|---|---|---|
| `V_CAP` | off | 刹车包线限速 v·t+v²/2a≤d_F−m(SL.v_cap);v≤7 惰性(两侧全等),**v=9 净赢**(fast_canyon reach 2/3→3/3、t −27%);高速世界建议开 |

## CPL-v3 逃生树(V3_ESC,2026-07-13)
| 开关 | 默认 | 判决 |
|---|---|---|
| `V3_ESC`(replay+渲染) | off | **预认证应急树三件套**:①plan_local 两阶段(直刹被否的 top-K 候选按意图进度开 dodge-to-rest 逃生网格,前缀先证);②L1.5 新鲜逃生;③L2 飞在位复合体预认证分支(上拍证书绝对窗覆盖本拍)取代裸 evade/冻结 hold。replay probe 8 场景:evade 104→46(−56%)、reach 1/8→2/8、0 撞、crossers evade 27→0 快 12.6s。配 V11 共享旋钮后 evade 38=V11 全栈 38 打平(净空更肥)但 reach 3/8 vs 7/8(残余=慢爬/振荡病,非紧急崩溃);现役仍=V11。渲染面 seed23 两臂全同(该 seed 病=振荡不进展,非拒证,树正确不介入)。新生 track 甄别 TODO。 |
| `V3_ESC_ANG` | 45,90 | 逃生方向:背离否决者 ±ang(否决者侧决定先序);静止态先 0°(正背离) |
| `V3_ESC_HOP` | 1.5,3.0 | 逃生距离网格(m);3.0 才打得过 head-on 中距(quintic 时长 vs 管增速的赛跑) |
| `V3_ESC_TOPK` | 12 | 逃生树每拍候选预算(按意图进度取 top-K,hover 永远重试) |
| `V2_ESC`(replay,V11 可用) | off | **逃生树接 v2/V11**:提交拍预证下一拍应急分支(障碍前推+DELTA+DT 时间系平移),evade 拍飞 L1.5 新鲜逃生/L2 预证分支。probe:evade 38→37、gauntlet 快 2.1s、14 拍无证→有证、0 撞;行为净收益小(v2 evade 集中在深口袋物理区),**主要收益=组合定理记账(evade 支瘦身)**;345 集待判。 |

## FOV-retention 锦标赛打分(FOV_RET,2026-07-13)
| 开关 | 默认 | 判决 |
|---|---|---|
| `FOV_RET`(replay+渲染,v2 锦标赛) | off | **感知感知 tiebreak**(Mueller 谱系,paper review 灵感#1):全网格打分 key 变 (速度, 进度/0.15 量化, FOV 保持率[, −excess if SAFETY_BAND])——同速同进度候选间选"移动威胁留在 45°×10m 视锥内"的绕行方向。保持率=信任窗 5 采样 × 移动 keep-out(\|v\|>0.3 或 veff>0;静物不计;超量程不计;无威胁=1.0 中性),朝向=计划速度方向(yaw-to-path 代理)。只动偏好:速度优先字典序、证书语义、incumbent 粘性全不动。因果链主张:coast 占比↓→λ-SHAPE-H 半径紧→干净抵达↑(用 COAST_LOG 测)。345 集 AB 判决(07-13,rows_fov*0713 配对):**不采纳为默认**——干净 68.4→68.1%(翻好 0/翻坏 1)、coast 占比 34.33→34.55%(因果链第一环没兑现:tiebreak 只在速度+进度打平时咬合,345 集仅 45 集行为有变)、唯一真收益 evade −33 且 props_alley 一场 −34(遮挡巷保视野=少晚发现);零星变坏 climb_trap +4/roundabout_rush +3。与 GAP_CARROT 同命;复活路径=进 CPL-v3 候选连续评分(方向感知基元待办同批)。 |
| `COAST_LOG` | off | 纯诊断:counts.trk_ticks/coast_ticks(ready 非 static track 的 coast 拍账),bench_shard 行加 coast/trk 字段。门控原因:counts 在字节回归哈希内。 |

## 椭圆管(v5 λ-SHAPE-HE 运动系各向异性 conformal,2026-07-13;注意 v4=LCP-alpha/v4b 另一条线)
| 开关 | 默认 | 判决 |
|---|---|---|
| `ELLIPSE`(replay,需 `CALIB_V2=1`) | off | **运动系椭圆 keep-out**:成熟(age≥4)、非 zombie、\|v\|≥v_min_dir(0.5)的 ped/veh track,管从圆改椭圆——沿 KF 速度方向半轴 A(t)=R+v_eff·(t+Δ),横向 A(t)/κ(κ=标定冻结长短轴比,calib_v5.json)。实现=白化代换:横轴拉伸 κ 的常数线性映射同时作用于承诺 B-spline 段与障碍多项式(`ego_certify_horizontal_aniso`,Bernstein 结构保持),圆内核原判交照跑;体半径按 R_w=κ·r_geom+q 折入(横向精确、沿向保守 sound)。接入 cert_clear/warp/margin 四门;verdict3 诊断与 v2 逃生树保持各向同性(圆⊇椭圆,保守 sound)。young/static/慢速一律圆(标定镜像同一判定)。收割新列 ea/ec/spd(harvest_v2 designCR/designE/E2/foldE/testE),标定 calibrate_ellipse.py(设计域=designCR,κ ped 2.07 / veh 1.17,λ=2.363@ε.05,testE 覆盖 95.8%)。κ=1 与各向同性字节同判(冒烟已证);字节回归 12/12(关)。**345 集三臂 AB(07-13,rows_ell_v11ref/ell_iso5/ell_ell5,V11esc 全旋钮)**:V11(v3 圆) 干净 69.3% / evade 435 → v5 椭圆 **70.7% / 362(−17%)**,同 0 撞、reach 持平、t 13.2→12.6;单变量对照(v5-iso 双胞胎同折 κ=1,68.1%)→ 椭圆范数本身 +2.6pp,配对翻好/翻坏 +23/−14(符号检验单侧 p=0.094,趋势正未达显著);复发坏例 street_rush_s3(沿向变肥税)。**判决:正向候选,未采默认**;渲染面/MU 面接线与成片欠账。 |
| `CALIB_FILE_V5` | out/conformal/calib_v5.json | v5 标定文件路径覆盖(冒烟/v5-iso 消融对照用) |
| `CAPSULE`(replay+渲染,需 `CALIB_V2=1`,与 ELLIPSE 互斥) | off | **v6 胶囊 keep-out(塔菲大人钦定形状)**:长轴端点=mover 背面与 KF 预测顶点(人身后不立墙),= 线段 [c₀, c₀+v·t]⊕q̃;残差改"到线段距离"(行人 q90@0.9s 塌缩 1.19→0.56,ped-M 形状 (0.308,0.36) vs 点法 (0.407,1.274))。cert=珍珠链覆盖:s∈{0..1} K=4 颗 obs_vel=s·v 圆 cert 取 AND,珠隙 \|v\|t/(2(K−1)) 折进 v_eff,s=0 珠即旧"冻结在当前位"合取——**零 C++ 改动**;q̃ 当普通圆心圆用会偏乐观,cap 标签在 verdict3/逃生树里必须保留(已接)。λ=1.841@ε.05,testE 覆盖 93.8%(ε.1 面 84.7% 低 2σ,入账)。**345 集四臂**:V11 69.3%/435 → **胶囊 70.1%/261(evade −40%,战役最大管红利)、reach 96.2% 最高、t 11.7s 最快**;vs iso 双胞胎 +2.0pp(p=0.17);vs v5 椭圆干净 −0.6pp 但 evade −101/更快/reach +1.1pp。渲染面椭圆四种子全慢于 iso(+1.9~4.6s,对称沿向税实锤)→ 成片走胶囊。复发坏例 crossers(横穿=物理税)。**判决:v6=形状线最优候选,未采默认待拍板。** **v6.1 尾流升级(07-13 晚,e4d4d55)**:实测背后仅需常数分位(ped rear-overrun q90=0.147 不随视界长,veh=0.000)→ ①rear 律入联合覆盖(单 λ 不变);②圆珠削不了背(截断盘外接圆=原圆),改加『整窗待在尾流』析取(`ego_certify_behind` 锚定半平面证,投影复用 above 内核);③K 4→6。**345 集五臂:干净 72.5% / reach 98.3% / evade 268 / t 11.7s 全面最优;形状单变量 iso→v6.1 配对 +27/−12 p=0.0119 达显著**;尾流增量本身 +14/−6(reach +7)。crossers min_clr 3.08→2.04=尾流放行真被使用。解锁认证跟飞。 |

## 时间维 EGO(EGO_TDYN,2026-07-13;MIGHTY/EGO-Swarm 公式,solver 首次开刀)
| 开关 | 默认 | 判决 |
|---|---|---|
| `EGO_TDYN`(渲染面 --maneuver) | off | **solver 级时间对齐移动障碍项**:bspline_optimizer 新增 calcMovingObstacleCost——控制点 i 时刻 t_i=((order−1)/2+(i−order+1))·interval(EGO-Swarm glb_time 惯例),对 mover 匀速多项式 c₀+v·t_i 的 xy 距离做静态同款三次铰链罚(z>顶+0.3 免罚,认证飞越保留)。**开启时 _man_cloud 不再喂 mover 占据环**(环会把扫掠走廊重新冻回空间,吃掉时间维)——planner 靠时间项塑形,珍珠链证书照旧把关(s=0 珠"人可能急停"诚实边界不动)。管线:ego_set_moving_obstacles capi(n×8 行 [c₀ v r_clear z_top])→ ego_bridge.set_moving_obstacles → ego_maneuver_replan 每拍喂 kf_movers。**关=字节回归**(seed7 25s 逐字节复现基线,已验)。seed7 单种子(KF 感知,CAPSULE=1):t 14.1→13.1s、min_clr 1.34→1.66m、switches 13→8、hold 13→5、jerk 35.9→30.3 全指标改善;345 集判决欠账。 |
| `EGO_TDYN_W` | 10.0 | 时间项权重 λ_moving |
| `EGO_TDYN_PAD` | 0.6 | 铰链半径 pad:r_clear=r_obs+d_safe(per-class)+PAD。**必须证书尺度**——r+0.45 的环尺度铰链在锦标赛保持的 ~1.5m 间距外永不激活(728/728 replan 字节同,已验教训) |
| `GT_ORACLE`(渲染面) | off | **全场全知诊断臂**:绕过感知前端(无锥/限距/遮挡/丢检/成熟度门),SENSE_R=30m 内所有 mover 从第 0 拍喂精确 GT 位置+速度(humanoid 引擎 o.velocity 恒 0,真速度=GT 位置逐拍差分 _GT_VELFD)。诊断链条阶梯:KF 14.1s → 传感器门内真值 9.9s → 全场全知 8.5s(seed7)=感知侧占总时长 40%。**非部署臂,纯上限标尺** |
| `KFDBG` | off | 逐拍 GT-vs-KF 行人对拍打印(gt/gtv/det/kf/kfv/miss),诊断 KF 滞后/coast 漂移/两点差分垃圾初速用 |
| `ORACLE_THIN`(渲染面) | off | **一键全知薄管臂(gt_thin 渲染面双胞胎)**:强制 GT_ORACLE=1 + 默认 EGO_TDYN=1/PAD=0.2/EGO_MANDSAFE=0.15 + CALIB_FILE_V6=out/conformal/calib_v6_thin.json(全类 q̃=0.05, v_eff=0.1, young/rear 同薄)。对人需求距离 ≈0.7m(默认标定 ≈2.0m)。**薄管只在零估计误差前提下 sound,故开关强绑全知——严禁手动把 thin 标定配 KF 臂**。seed7:8.4s / min_clr 1.90m / 0 撞(管大小在全知+时间维下已非瓶颈,剩余时间=路线物理)。 |
| `EGO_TAU_DYN` + `EGO_TAU_MAX`(渲染面) | off / 2.5 | **相遇时间视界(实验,判决:负面)**:cert 窗口与胶囊 tip 从固定 τ=0.75s 拉到各 mover 相对运动 CPA 时刻的最大值(clip [0.75, MAX])——"长度=两者何时相遇"。**seed7 ORACLE_THIN 单调变坏:0.75s 固定 8.4s / cap 1.25s 8.8s / cap 2.5s 10.6s+3 秒尾流跟爬(min_clr 0.60)**。机制:s=0"急停"珠随窗口等长冻结 mover 当前位,窗口越长前方封越死,反而杀光绕行候选——相遇视界利 planner(EGO_TDYN 已全程时间对齐,不需要它)、害证书。留档不入 ORACLE_THIN 默认;复活路径=cert 窗与胶囊段长解耦(需 _SL 手术)或 s=0 珠时长上限单独治理。 |
| `KF_SIGV_YOUNG`(渲染面 realistic 路径) | 0(关) | **新生 track 诚实门(KF 第一刀,07-14)**:σ_v 超阈的 ready track 冻结在**最后检测位**(_LASTDET 锚,非 coast 漂移位)、速度置零。物理依据:0.1s 内两个 7cm 噪声位置观测的速度下限 σ≈1 m/s(两点差分=漫先验贝叶斯精确解,Monte-Carlo 实证 bayes 初始化零改善,KF_INIT=bayes 留档 kf_tracker.py)——垃圾初速(病①)与 coast 漂移(病②)一门双治。**甜点 0.5**(≈4 观测放行):seed7 KF 臂 10.6→**9.4s**、ped min_clr 1.50→**2.00m**、0 撞;0.3/0.7/1.0 全不如。附带验证:门后成熟速度仍有 0.4m/s 过程噪声地板,EGO_PLANHI=1.5 在 KF 臂依旧变差(9.9)→ 提前量-估计质量定律成立。345 集判决欠。 |
