# 2026-07-14:KF 战役开刀 + 全知臂成片波 + 场景池肃清

## KF 战役进度(核心主线,接 tdyn-oracle-ladder 的"KF 三层病")
- **病①第一刀已下**:`53f8fe3` KF cut 1:young-track 诚实门(`KF_SIGV_YOUNG`,n 小时不信差分速度),**seed7 KF 臂 10.6 → 9.4s,行人净空 2.00m**;`c3c18eb` 补 THIN sizing pack(全知同款胶囊挂任意臂,唯一差别=估计器,塔菲大人 07-14 钦定方法)+ young 门迟滞防抖。
- **病② coast 漂移加速未修**:CA 模型 coast 持续积分,新生 track 0.6s 内 |v| 0.6→2.1 垃圾加速度。
- **病③ 开局锥缘丢检串未修**:7-8 拍丢检杀 track→重生循环,病①反复发作。
- 修完 KF 后的顺势红利:KF 臂相遇点提前量 0.7→1.5s(全知臂已证 1.5s 值 1.1s;定律=提前量长度须匹配估计质量)。
- 其余欠账不变:λ=1.841 门重算(潜在 −0.5m)、TDYN 接 replay 面、345 集全量判决。

## 全知臂成片波(ORACLE_THIN=1,产线 render_3d_video,均 0 撞全到达)
`out/drone_3d_oracle_s{3,5,7,13,23}.mp4`:s7 **7.2s**/clr1.97/零机动(比记档 7.4 又快 0.2=THIN 收尾红利)、s13 8.5/0.83、s3 8.7/0.93、s5 9.1/0.97、s23 修后 8.7/0.59。
**并行渲染纪律(差点又忘)**:`OUT_MP4` env 每任务独立输出可并行,但**全 3D 上限 2 路**(3 路 OOM 实锤过)+ 5800X MCE 满载崩;headless 探针可 3 路。

## 场景池肃清(塔菲大人 07-14 下令)
- 场景池底数 = **20 个场景(seed 0-19)**;seed≥20 复用 `seed%20` 场景+不同路线 RNG。
- **per-seed 终点手术表 `_GOAL_FIX` 已焊进 `plan_route`**(render_3d_video.py,无 env、全臂全面生效,焊在静态清障回缩之前保 bbox 兜底):`{23: +5m}`。seed23 病根=终点摆进静态口袋:18.5s/68 hold → **8.7s/1 hold**(路还长 5m),0.53→0.59m。**"seed 病"先查终点摆位再怪算法**。
- **肃清完毕(21/21 全绿,全知臂 headless,全 0 撞)**:健康 18 个 6.5-11.8s;重病 3 个全治愈——**hold 刷屏有两种病源,选刀前先看卡点离终点多远**:
  - **终点口袋病(轻)**:s23 → `_GOAL_FIX={23:+5m}`(18.5s/68hold→8.7s/1hold);
  - **路线结构病(重,+5m 无效实测)**:s1(冻在离终点 17m 的 z=3.7 高空)、s11(路线起点和出生点脱节 150m)→ `_ROUTE_SALT={1:1, 11:2}` 重摇路线(换锚点人群+方向):s1→7.9s/hold1,s11→13.0s/hold0(s11 场景静态极密,salt1 也堵,salt2 才通)。
- 两张手术表都在 `plan_route`(render_3d_video.py),无 env、全臂生效;死场景宁可除名不硬塞假路线(这次没用上)。

下一刀:KF 病②(丢检串→track 缝合)/③(噪声地板),刀序待塔菲大人拍板。

## nuScenes 真实数据面开张(07-14 下午,8faafdb)
塔菲大人转向目标检测/真实数据:`data/v1.0-mini` = **nuScenes mini(10 scene × ~40 关键帧 @2Hz 抖动 0.4-0.6s,场内连续场间不拼)**。`nuscenes/kf_nusc.py`:生产 `MoverTracker` 原样 import(变 dt 路径吃真实时间戳),GT 关联(instance_token,=真实数据版 GT_ORACLE 臂,隔离估计器),世界系 3D 框中心喂 KF,预测投回 CAM_FRONT 出 jpg(`nuscenes/out/`,昼/夜目检过)。**46k 评分点判决**:①**CV 全面赢 CA**(行人 MOVING 3s 0.71 vs 1.57 均值)= MetaUrban 病④(CA 过冲)真实数据实锤;②KF 大胜 still 基线(车 MOVING 3s 3.4 vs 21.3m);③**病①(young 两点差分)在 GT 喂入下基本消失**(n=2 与 n≥4 同量级)→ 病①=噪声×节奏的乘积而非结构病,young 诚实门(治标)是对的药。下一步:接 visibility 低档当真实丢检串(病②测试床)/换掉 GT 关联上检测器。

### YOLO 无标注臂(同日,`nuscenes/yolo_nusc.py`)
塔菲大人的 YOLO(`cvmusecore/yolo26s.pt`,COCO 预训练)全管线不碰标注:检测→框底反投影地面(z=0)→自家 NN 关联→生产 KF→CAM_FRONT 叠加 mp4(out/yolo_scene-{0103,1094}.mp4)。**三轮判决**:①裸跑=比 still 还差(单目深度噪声 0.5s 节奏下变幻影速度,行人 1s 2.8 vs still 2.1);②R 按距离(σ≈0.3+d²/950)+ young 门 0.5 → 全冻结=止损地板(误差=still,不再瞎说);③**per-class 放宽门(车 3.0)负面判决**:车 3s 15.4 vs 冻结 6.0——**车速误差=系统偏差(框底=车头近边随视角滑动)非方差,σ_v 门拦不住,修法在观测端不在门上**。同 KF GT 喂 1s 行人 0.19m vs YOLO 喂 2.14m = 检测器+单目测距税;young 门(cut 1)在真实数据上判对。观测升级候选:12Hz sweeps 提节奏 / 框中心+类尺寸先验测深 / 更大权重 yolo26m。
