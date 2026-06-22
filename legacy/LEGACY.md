# LEGACY — 退役/前身代码记录

> 2026-06-22 仓库统一。**唯一活跃 git = `sando-core`**(本仓库,GitHub `YBX420/sando-core`,分支 `feat/bernstein-gate`,HEAD `46f5ac9`)。
> 此前 `D:/projects/sando_py/` 下并存 3 个 sando 相关 repo,现已统一:把前身的独有代码折进 `legacy/`,其余退役并在此记录(是什么 / 为何退役 / 现在去哪找)。

---

## 1. `legacy/sando-py/` —— 原始 python repo 的独有代码(已折入本仓库)

- **来源**:GitHub `YBX420/sando-py`,分支 `cpp-port`(HEAD `e3e531a`,74 commits;另有 `main` HEAD `4bd936c`、`chinese-comments` 等)。
- **是什么**:sando 项目最早的 python+cpp+ros2 实现。**sando-core 当初就是从它克隆、去掉 python 来的**——所以它的 `cpp/`、`docs/` 已被 sando-core 取代,不折入。
- **折进来的(独有、仍有价值)**:
  - `legacy/sando-py/python/` —— 原始 python 算法 + 测试套件(含 **conformal 证书数学验证** `test/stage4_conformal_*.py`、per-class MINCO python 实现 `sando_py/`、Isaac 闭环脚本、stage3/4 全套测试)。这是「per-class MINCO 规划器 / conformal 证书」的 python 前身,数学结论可查。
  - `legacy/sando-py/ros2_bridge/` —— depth→occupancy、状态桥、可视化等 ROS2 胶水。
- **没折的**:`python/media/`(23M 演示 gif/图)、`_speed_bench.log`、`__pycache__`、以及 `cpp/`+`docs/`(被 sando-core 取代)。**完整副本仍在 GitHub `YBX420/sando-py` 归档**。
- **为何退役**:活跃开发已全部迁到 sando-core(C++ 核 + EGO + metaurban)。python 侧留作前身/参考,不再主线开发。

## 2. `sando-core-bcert/` —— 断掉的 git worktree(已移除,内容无丢失)

- **是什么**:sando-core 的一个 git worktree,分支 `bcert-wire`(HEAD `e8350cc`)。
- **关键事实**:`bcert-wire` 是 `feat/bernstein-gate` 的**纯祖先**(0 ahead / 3 behind)——它的全部已提交内容**已包含在 sando-core 历史里**(`git checkout e8350cc` 可还原)。
- **为何退役**:① 内容是 sando-core 的子集,纯冗余;② 它的 `.git` 指向 Linux 路径 `/media/boxuan/Data21/...`,在本 Windows 机器上**已是断的 worktree**,git 无法操作。唯一独有的是可重建的构建产物 `cpp/capi/sando_capi.so`(无需保留)。
- **现在去哪找**:sando-core 历史 `e8350cc`;或 GitHub `YBX420/sando-core` 分支 `bcert-wire`。

## 3. 无关的 `D:/projects` 大仓库(不在本次范围)

- `D:/projects/.git` 是另一个无关 repo(含 `cvmusecore` OMR 项目 + 杂项 "first push")。它**不跟踪** `sando_py/` 下的内容。与本安全证书工作无关,未触碰。

---

## 备注:仓库体积

- sando-core 的 `.git` 约 9.5G、`isaac/` 约 9.5G——历史里塞了大二进制(Isaac 数据集 / `cpp/build_v2/` 构建产物 / metaurban 视频)。
- 彻底瘦身需 `git filter-repo` 重写历史(会改所有 SHA + 必须 force-push),**属单独高风险一步,本次未做**。本次只做了 .gitignore 卫生(停止未来再塞构建产物)。
