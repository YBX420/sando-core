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
