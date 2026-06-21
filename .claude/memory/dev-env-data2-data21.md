---
name: dev-env-data2-data21
description: 环境坑:Data2和Data21是同一块盘改了挂载名;仓库只在Data21一份;改名断了缓存绝对路径(CMake/.so)=MetaUrban"突然不行"的根因;含重编命令
metadata:
  type: reference
---

**2026-06-20 环境坑(会反复困惑,记牢):`/media/boxuan/Data2` 和 `/media/boxuan/Data21` 是同一块物理盘,只是挂载点改了名。**
- 真盘 `/dev/nvme0n1p2`(954G)**现在挂在 `/media/boxuan/Data21`**;`/media/boxuan/Data2` 现在是空目录(root 所有、没挂盘)。
- sando-core **只有一份**(在 `/media/boxuan/Data21/projects/sando_py/sando-core`),没分裂没搬丢。

**后果(= 各种"之前能跑现在不行"的根因)**:挂载名 Data2→Data21 把**写死的绝对路径缓存**全打断:
- `cpp/build/CMakeCache.txt` 缓存了 `/media/boxuan/Data2/...` → 报 "CMakeCache directory different" → 构建失败。**修法**:`rm -rf cpp/build && cmake -S cpp -B cpp/build`(已修)。
- `.so` 旧、或 python 找不到 → 重编(见下)。bridge `sando_cpp_bridge.py` 在 **`isaac/`**(不是 metaurban/),`eval_batch.py:21` 把 isaac/ 加进 path 再 import。
- 仓库内**没有残留写死 Data2** 的路径(grep 干净);但**很多文件写死了 `/media/boxuan/Data21/...`**(eval_parallel.py / render_live.py / run_demo.py / isaac/ira/*.yaml)→ **下次盘再改名又会断**(latent 脆弱,未根治;可改相对路径)。

**重编命令(改 C++ 后必做,capi 不在 CMake 图里)**:
- sando capi:`cd cpp && g++ -O2 -shared -fPIC -std=c++17 -o capi/sando_capi.so capi/sando_capi.cpp -Iinclude -Ithird_party/eigen -Ithird_party`
- ego capi:`cd ego && g++ -O2 -shared -fPIC -std=c++17 -Wno-narrowing -w -o capi/ego_capi.so capi/ego_capi.cpp src/*.cpp -I include -I ../cpp/include -I ../cpp/third_party/eigen -I ../cpp/third_party`
- `*.so` 被 gitignore(不提交)。conda env = `sando`。

验证核心通了:`python -c "import sys;sys.path.insert(0,'isaac');from sando_cpp_bridge import SANDO,Parameters;SANDO(Parameters())"` → OK。
