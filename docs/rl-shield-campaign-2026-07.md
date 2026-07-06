# RL × 证书门 campaign 汇总(2026-07-05/06)

PPO 臂跑在 **live MetaUrban**(`metaurban_nav_env.py`:2D 平面质点抽象,同 v1/v2 协议,障碍源=真 MetaUrban GT→感知 KF→conformal shield)。所有评估:训练同 20 场景(**无 held-out split,外推前必须补**),每格 400 集,seed 7777,确定性策略。绝对碰撞率是 env 定义的(车=圆形 footprint、2D),**只做臂间比较**。

## 3×2 矩阵(碰撞% / 到达% / 未认证暴露)

| 训练 \ 评估 | shield ON | shield OFF |
|---|---|---|
| fixed-λ(shield-in-training) | **6.75** / 89.5 / 1.12% | 14.25 / 81.0 |
| bare(纯学习) | **8.25** / 82.2 / 0.58% | 14.50 / 76.8 |
| adaptive-λ(target 1%) | **6.50** / 87.5 / **0.22%** | **19.25** / 78.2 |

Fisher(ON vs OFF):p=0.0007 / 0.0073 / <1e-4。数据:`metaurban/out/ppo_eval/ab_20260706_{014118,125020,125022}/`。

结论:①门对任意策略有效(bare 训的也腰斩);②shield-in-training 有门外价值(in-track 死 2 vs 9、超时 15 vs 38);③adaptive-λ 用裸跑安全换证书覆盖(uncert 5 倍压缩→99.78% 拍在保证伞下,λ 0.5→4.85 近 clip);④静物碰撞所有 shield 臂清零。

## 碰撞归因 + oracle 消融(boundary 定稿)

Shield 臂残余碰撞 **100% vehicle**(8.3 m/s),~89% 从 45° 视锥外(侧后方)来。
**Oracle(16m 全向 GT 喂 shield)碰撞 6.75%=一次未减**,但致命拍从"25 passed(看不见)"变为 **27/27 全 brake(看见、投影逃生失败、诚实宣布未认证)** → 地板是**物理的**(1.9s 预警 × 3m/s 机动 vs 8.3m/s 车扫掠走廊无逃逸解),非感知的。360° 安全环路线否决(收益=0)。数据:`out/ppo_eval/oracle_20260706_153925/`。

**失效包络写法**:传感器包络内证书零违约(无一次"认证 track 内障碍后碰撞";in-track 死拍均为 brake=已计入未认证暴露);包络外 ~6.75% 地板经 oracle 证实物理不可避(条件:16m 视界、3 m/s 平台、2D)。

## 泛化臂(进行中)

`ground_nav_env.py` + `ground_shield.py`:同世界同证书,载体换 unicycle 人行道机器人(2 m/s、感知锥焊车头、证书检查直线→弧线族)。地面机器人 2D=真身;无人机臂后续升 3D(z 轴+爬升+圆柱证书垂直分支)。

## 附:时基验证

MetaUrban `0.02s×5=0.1s/step`(配置+实测 0.0976s 双证),3 substeps=0.293s ≈ DT 0.3s(比值 0.98)。

视频:`out/videos/ab_ep13.mp4`(shield 救命)、`ab_ep11_boundary.mp4`(物理边界双撞)、`ab_ep14_detour.mp4`(绕行代价)。模型:`out/ppo_planner_mu{,_bare,_adapt}.zip`。
