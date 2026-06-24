---
name: sando-core-px4-sitl-2026-06
description: "2026-06-24 把 PX4 SITL 闭环跑通的血泪教训。ours 在真 PX4 上飞通走廊、到达、0 碰撞(clr 1.43m,t 54.6s 慢因 position-offboard)——之前所有'在 PX4 上输'全是基础设施 bug 不是算法:① stale lockstep jmavsim 占 :4560 污染一切;② SITL 电池 SIM_BAT_DRAIN 默认 60s 耗光→紧急电池强制降落解锁;③ px4_bridge.set_setpoint 高度轴 bug(巡航点 down 算成 0=地面→无人机降落);④ 'px4 alive=0' 是假阴性(-d 守护进程名叫 px4 不叫 bin/px4);⑤ EKF/GPS 没收敛就 arm→'Arming denied: health failures'。修复全在 px4_sitl.sh(启停脚本)+ px4_bridge(等满 health+arm 重试+SIM_BAT_DRAIN=0+frame 修)。"
metadata:
  type: project
---

**2026-06-24:把 PX4 SITL 在本机(`/media/boxuan/Data2/projects/PX4-Autopilot`,二进制 Jun19 已编译)闭环跑通。** 用户拍板"px4 最后 sim2real 躲不掉"——必须啃。承接 [[sando-core-conformal-2026-06]] 的 dynamics 工作;[[dev-env-data2-data21]] 的盘改名也坑了 PX4 build cache(`make px4_sitl jmavsim` 报 CMakeCache 目录不符 + 'ninja: unknown target jmavsim',所以**绕过 make 直接起预编译二进制**)。

**最终成功证据:** `px4_diag.py` ours@4.0 seed1 ep0 —— `armed=True` → motion check 移动 1.56m(OK flying)→ **reached=True t=54.6s clr=1.43m**(到达+0 碰撞)。**所以 ours 在真 PX4 上可行;之前所有失败全是基础设施。**

**5 个真因(每个都耗了我很久,别再踩):**
1. **stale lockstep jmavsim 占 :4560**:`jmavsim_run.sh` 默认带 `-lockstep`;一个没杀干净的旧 java(jmavsim)占着 TCP 4560,新 sim 启动报 "Address already in use",PX4 连上**幽灵旧 sim** → 各种诡异死法。**根治:杀 `jmavsim_run.sh` 的整个进程组(它 own java 子进程)`kill -9 -$pgid`。** jmavsim_run.sh **没有重生循环**(只 getopts+跑一次 java),所以杀进程组就死透。
2. **SITL 电池 `SIM_BAT_DRAIN` 默认 60s** 从满耗到空(armed 时)→ arm 后 ~12s 触发 emergency-battery → **RTL/Land/Disarmed by landing**,无人机停在原地。`COM_LOW_BAT_ACT=0` 拦不住 emergency。**根治:`SIM_BAT_DRAIN=0`(文档:设 0 完全禁用电池模拟器)+ BAT_*_THR=0**,bridge 连接后立刻设(timeout 包裹防挂)。
3. **px4_bridge.set_setpoint 高度轴 bug**:原来 `down=-(z-origin_z)`,巡航高度点(z=1.5,origin_z=1.5)→ down=0=NED 地面 → **无人机被命令降到地面**→落地/卡死,从不飞走廊。**修:down=-(绝对高度 z);get_pose_world 同步改 x=E+ox,y=N+oy,z=-down。**
4. **`px4 alive` 假阴性**:`./bin/px4 -d` fork 守护进程,名字是 **`px4`** 不是 `bin/px4`;`pgrep -f 'bin/px4'` 查不到→我以为死了→反复重启搞乱。**查活:`pgrep -ax px4 | grep bin/px4`。** 也别用 `pgrep -f` 含模式串自身(自匹配),用 bracket `[p]x4`。
5. **EKF/GPS 没收敛就 arm** → `Arming denied: Resolve system health failures first`。**修:bridge 等满 `is_local/global/home_position_ok + cal_ok`(120s budget)再 arm,且 arm/offboard 各重试若干次。** 外加 `px4_sitl.sh wait` 起飞后再等 ~55s EKF。

**工具(都已提交):**
- `metaurban/px4_sitl.sh {start|stop|restart|status|wait}` —— 标准化启停;`restart` 干净起一套,`wait` 额外等 EKF。
- `metaurban/px4_bridge.py` —— MAVSDK offboard;`__init__` 后台 asyncio 线程,`set_setpoint(world_xyz,yaw)` / `get_pose_world()` sync 门面;现含 failsafe 关闭+满 health 等待+arm 重试+frame 修。
- `metaurban/px4_diag.py` —— 单 episode 过真 PX4,**起飞前 motion check(命令 +3m 验证真在动 >1.5m)**——防"没飞却报 stall"的假结果。
- `metaurban/px4_replay.py` —— 多 episode PX4 spot-check(real-time)。
- 跑法:`bash metaurban/px4_sitl.sh restart` → 等 EKF → metaurban env + `LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6`(ego_capi+mavsdk 共存)+ `LD_LIBRARY_PATH=~/gurobi1103/linux64/lib`(若同时跑 SANDO)。mavsdk 只装在 metaurban env。

**诚实 caveat:PX4 上很慢**(54.6s vs stand-in 5.4s)——position-only offboard 跟踪保守。相对比较(ours/EGO/SANDO 都过同一 PX4)仍公平;绝对提速要 velocity-feedforward offboard(future)。相关:[[sando-core-conformal-2026-06]] [[dev-env-data2-data21]] [[metaurban-render-recipe-2026-06]]。
