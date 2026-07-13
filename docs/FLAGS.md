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
| `ELLIPSE`(replay,需 `CALIB_V2=1`) | off | **运动系椭圆 keep-out**:成熟(age≥4)、非 zombie、\|v\|≥v_min_dir(0.5)的 ped/veh track,管从圆改椭圆——沿 KF 速度方向半轴 A(t)=R+v_eff·(t+Δ),横向 A(t)/κ(κ=标定冻结长短轴比,calib_v5.json)。实现=白化代换:横轴拉伸 κ 的常数线性映射同时作用于承诺 B-spline 段与障碍多项式(`ego_certify_horizontal_aniso`,Bernstein 结构保持),圆内核原判交照跑;体半径按 R_w=κ·r_geom+q 折入(横向精确、沿向保守 sound)。接入 cert_clear/warp/margin 四门;verdict3 诊断与 v2 逃生树保持各向同性(圆⊇椭圆,保守 sound)。young/static/慢速一律圆(标定镜像同一判定)。收割新列 ea/ec/spd(harvest_v2 designCR/designE/E2/foldE/testE),标定 calibrate_ellipse.py(设计域=designCR,κ ped 2.07 / veh 1.17,λ=2.363@ε.05,testE 覆盖 95.8%)。κ=1 与各向同性字节同判(冒烟已证);字节回归 12/12(关)。345 集 AB 待跑。 |
| `CALIB_FILE_V5` | out/conformal/calib_v5.json | v5 标定文件路径覆盖(冒烟/v5-iso 消融对照用) |
