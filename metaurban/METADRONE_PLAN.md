# metadrone 计划:MetaUrban → 无人机原生仿真平台(对标 AirSim)

> 2026-07-03 v2。合并用户两轮方案 + 两处关键修正(用户全仓 grep 实证,推翻本文 v1 的 P1/探针结论)。
> 平台名:**metadrone**(不再叫 sando——sando 是安全层研究线的名字)。
> AirSim 已被微软归档(2022,社区分支 Colosseum)——市场有空位,我们带着别人没有的东西入场:
> **认证安全层 + 场景工作台 + 评测协议**。

## 0. 两处修正(v1 的错误,以此为准)

### 修正①:不做 Bullet 施力驱动的 BaseDrone
全仓 grep 无 `apply_central_force/apply_torque` 真实调用先例——所有非轮式 agent(行人/机器狗/
送货机器人)全走 `setLinearVelocity` 运动学驱动(base_object.py:312-330)。
**"刚体+施力"在此 codebase 是荒路;现行路线(Quadrotor 自积分动力学 + set position 贴渲染网格)
与仓库惯例一致,是正确选择,不改。** v1 的 P1(Bullet 刚体)作废。

### 修正②:建筑楼顶有碰撞(v1 探针是掩码盲)
建筑碰撞体 = BulletBoxShape 实心盒 0→HEIGHT(test_new_object.py:88-91,碰撞组 InvisibleWall)。
v1 探针 rayTestClosest 未传掩码 → 默认掩码滤掉了 InvisibleWall 组 + seed3 出生区恰无建筑。
**全开掩码复测(±150m):>2m 命中 60 处,z 中位 10.9m,max 12.8m——楼顶是实心可靠碰撞面。**
真正的问题反转:碰撞盒是**粗糙 AABB**,可能比可见几何胖/瘦;只有人行道/地形是真三角网。
→ 贴楼飞行的安全距离必须吃进包围盒误差;**roofline 以上是安全空域,可进规划假设**
(metaurban_sando.yaml 的操作盒 z 范围据此定)。

## 1. 修订后的三个行动点(用户定)

1. **kinematic ghost body**:无人机对世界是"幽灵"(Quadrotor 不在 Bullet 里,撞楼判定全靠
   C++ 规划层保证)。要独立碰撞真值(如统计 RTA 失效率),给 drone 挂一个 kinematic ghost body
   **只做 overlap 查询、不参与动力学**——比施力驱动重构便宜一个量级。
2. **感知升级绕开原生 lidar**(单线、固定 0.6m 高):走 point_cloud_lidar.py 或深度相机管线;
   现行 GT 分类喂 C++ 的路线已绕开,保持。
3. **建筑 z 语义进规划**:建筑盒顶=HEIGHT 是可靠碰撞面 → "roofline 以上安全空域"作为规划假设;
   AABB 误差进安全距离预算。

## 2. 现状资产(metaurban-sando-drone 仓已有,勿重做)
完整 Quadrotor plant(位置 PD 外环 → 限倾角推力矢量 → 一阶姿态内环,接口对齐 PX4 set-point)、
ctypes 桥到 sando C++ 安全层(heat-A* + per-class MINCO + RTA)、离屏 MJPEG 渲染 demo;
README 已记录 heading=+X、cv2 import 顺序等坑。本仓(sando-core/metaurban)另有:
场景工作台全套、逼真感知前端、417 道具目录(metainfo 驱动,视觉=证书碰撞)、
城建规则预言机(urban_rules.py,规则表=官方 AssetManager 摆放标准)、block_str 积木地图。

## 3. 修订执行序
- **P1(改):ghost body 碰撞真值** —— kinematic ghost + overlap 查询,输出独立 collided 信号,
  与 C++ 层判定对账(RTA 失效率统计的地基)。
- **P2:建筑 z 语义 + AABB 误差标定** —— 全开掩码射线扫描量化"碰撞盒 vs 可见几何"偏差分布,
  偏差分位数进安全距离;操作盒 z 范围按楼高分布定。
- **P3:传感器 + RL 接口** —— IMU/气压/GPS 噪声(自机状态去上帝视角)、云台相机、
  深度管线(绕开单线 lidar);DroneEnv(gym) 包装(动作=姿态层 set-point,Quadrotor plant 消化)。

## 4. 定位不变
不比渲染,比评测科学:①唯一带 sound 连续时间证书的仿真平台;②场景 schema+设计器+重采样协议;
③逼真感知前端开箱即用;④headless 并行评测。**AirSim 教无人机飞,metadrone 证明无人机安全。**
