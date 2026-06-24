---
name: metaurban-render-recipe-2026-06
description: "2026-06-24 跑通 MetaUrban 真 3D 渲染(render_3d_video.py)出 mp4 的可用配方 + 三个非显然环境坑。渲染必须在 metaurban env(只它有 metaurban+panda3d+GPU),但 ego_capi.so 在 sando env 编 → 两个 libstdc++ 混进同进程会在 ego.replan 段错误(exit139)。修法=非静态重编 ego_capi.so + 渲染时 LD_PRELOAD sando 的 libstdc++ 让 panda3d 与 ego_capi 共用一个新版。还修了资产路径硬编旧盘名 Data21。新增 --maneuver 变体渲新机动 + stitch_ab.py 出左右对比。"
metadata:
  type: reference
---

**MetaUrban 真 3D 渲染配方(2026-06-24 在本机跑通,RTX 4070 + DISPLAY :1)。** `render_3d_video.py` 出 `out/drone_3d.mp4`(FPV+第三人称双视角真 3D)。

**完整命令(三件套环境变量缺一不可):**
```bash
cd /media/boxuan/Data2/projects/metaurban
env DISPLAY=:1 \
    PYTHONPATH=/media/boxuan/Data2/projects/metaurban \
    LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6 \
    ~/miniconda3/envs/metaurban/bin/python -u \
    /media/boxuan/Data2/projects/sando_py/sando-core/metaurban/render_3d_video.py \
    --seed 3 --maneuver --clear_spawn --mp4 --t_max 22 --w 440 --h 280
# 变体: --maneuver(ours M3 无 HOLD 圆柱机动) / --ego(native 基线) / --ego_safe(旧二元 HOLD 层) / --frame_only(只出一帧验证)
```

**三个非显然坑(都踩过、都修了):**
1. **`metaurban` 包 import 不到** → Python 用脚本所在目录(sando-core/metaurban)而非 cwd,metaurban 包没 pip 装 → 必须 `PYTHONPATH=/media/boxuan/Data2/projects/metaurban`。
2. **`ego.replan` 段错误(exit 139)** = 致命坑。渲染只能在 metaurban env 跑(只它有 metaurban+panda3d+GPU),但 `ego_capi.so` 在 sando env 编(新 libstdc++,GLIBCXX_3.4.32)。metaurban env 旧 libstdc++ 加载不了 → 先想静态链接(`-static-libstdc++`)能加载但**和 panda3d 的 libstdc++ 混进一个进程会在 replan 段错误**(两个 C++ 运行时)。**正解:ego_capi.so 用非静态原配方编 + 渲染时 `LD_PRELOAD=$HOME/miniconda3/envs/sando/lib/libstdc++.so.6`**,让 panda3d 和 ego_capi 共用同一个新版 libstdc++(新版向后兼容,panda3d 照常)。单独跑(不带 panda3d)静态/非静态都不崩——崩只在共存时。
3. **资产路径硬编旧盘名** → `render_3d_video.py` 的 `ASSETS` 曾写死 `/media/boxuan/Data21/...`(盘改名成 Data2,见 [[dev-env-data2-data21]]),`cow_quaternius.glb` 加载失败。已改成自动选存在的 `Data2`/`Data21`。

**A/B 出片(native vs ours):** `scratchpad/make_ab.sh` 模式 = 顺序渲两圈(`--ego` 与 `--maneuver`,同 seed/`--clear_spawn` 同路线)→ `metaurban/stitch_ab.py LEFT.mp4 "A标" RIGHT.mp4 "B标" OUT.mp4` 左右拼接 + poster。**两渲染别并行**(同 GPU 抢占)。产出 `out/ab_maneuver.mp4` + `_poster.png`。

**M3 demo 实测(seed 3 街景):** ours `status=OVER` 真从行人上方飞越(alt 2.03m,行人净空 1.31m);native 低空贴行人挤过(净空 0.25m);都到达不撞。见 [[sando-core-goaround-m1-2026-06]] M3。`--maneuver` 接入点:`ego_maneuver_replan()`(证书门地面绕行+飞越兜底,见 [[sando-core-win-ego-2026-06]])。

**2026-06-24 KF 接进渲染器 + 可视化(用户要"证明 KF 在用"):** 之前渲染器用 GT 速度,没真用 KF。现在 `--maneuver` 走 `kf_movers()`:对**每个非静态 mover(行人/车/动物,不只人)**喂噪声检测(`EGO_MEASNOISE` 默认 0.10)→ per-mover `MoverTracker`(CA-Kalman)→ 滤波中心+速度喂证书/占据 + 预测未来轨迹存 `_KF_PRED`。`draw_predictions()` 在 `pred_root`(置顶橙色层)画:白点=噪声检测、橙=KF 滤波当前、**橙线+橙标=KF 预测未来位置(无人机绕开"将来在哪")**。HUD 图例 maneuver 模式改成 "ORANGE=Kalman forecast"。demo:`out/kf_demo_seed3.mp4`。**注意:渲染器原 native(--ego)用 GT 无 KF,仍是反应式对照。** d_safe 街景速度:ours 比贴飞的真 EGO 慢(用户拍板"别纠结 d_safe,核心不撞+快",可调 `EGO_MANDSAFE`)。
