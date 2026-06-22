# 安全层 · 工程路线(v2,2026-06-22 全量重写)

> v1 是「13 周冲 RA-L 9/15」的论文时间线(W1-W13 / Gate 0 Boyle 签字 / 三铁律 / 砍单),**已作废**——重心改成工程实现,论文降为下游(投不投、何时投未定)。配套 `docs/safety-layer-spec.md`(先读它)。
> 本文 = 做完了什么 / 现在做什么 / 以后再说,按工程优先级排,不绑 deadline。

## 0. 工程不变量(技术真,无论投不投论文都守)

1. **证书 soundness 不可破**:外向取整区间,「证过」必须真过。改证书数学 → 必重核 `test_bernstein_cert` + golden。
2. **不软化硬走廊**:SFC/硬约束是证书赖以成立的基础,不准为了追平 benchmark 把它软化(那会删掉唯一存活的精确证书价值)。
3. **改 C++ 必重编 capi `.so`**(`sando_capi.so` + `ego_capi.so`),否则 python 桥/闭环全断;Windows `.dll` 旧库陷阱。
4. **对外措辞诚实**:`q_conformal=0` 期间只说「确定性几何证书」,不说「概率/语义风险保证」。

## 1. 做完了(2026-06-17 → 06-21)

- **C++ MINCO 核** 去-ROS 起底 + Isaac ctypes 桥 + 闭环烟雾(Ubuntu)。
- **DynTraj 标签集 ABI** 端到端(`label_set` + `derived_class` + `traj_set_label_set` + 桥),per-class 硬软默认生效。
- **精确连续时间 Bernstein 亏量证书**(`bernstein_cert.hpp`):MINCO 五次专用路径 + **任意次通用核**;sound(区间外向取整)、非空、自检无误证/无漏证。
- **planner 无关已证**:通用核 vs deg-5 专用路径在同轨迹上**判定(verdict)一致**(ctest 交叉验证;margin 未断言)。
- **EGO vendored + 适配**:去-ROS 的 ZJU EGO 进 `ego/`,`ego_certify` 把 cubic B-spline→Bezier 喂同一核(亏量 6 次);`ego_capi.so` 编出来了。
- **EgoSafe 闭环**(`ego_safe.py`):EGO 规划 → 证书判 → 不过 HOLD;80-tick 自检断言执行净空≥0。
- **10-seed A/B**(`ab_runner.py` + `render_3d_video.py --ego/--ego_safe`):真四旋翼动力学,raw EGO vs EGO+层。
- **证书门接进 MINCO 核**(`minco_deficit_cert`,默认 OFF,OFF 时字节一致)。
- **三个 cert-fidelity 开关**(默认 OFF + 各自闭环 ctest):seam C2-from-exec-state、平滑刹车、C2 yaw governor。
- **ctest 涨到 25**,上次 Linux 跑 **25/25 全过**(`LastTest.log` 2026-06-20)。
- **对照基线** sando_native(MIT-ACL SANDO + GUROBI)vendored。

## 2. 现在做什么(下一波工程,按优先级)

> 都不依赖 conformal/论文,纯把现有东西做扎实、做完整。

1. **统一「层」的形态**:正典 = 二元 certify-or-HOLD(`ego_safe.py`);分级减速刹车(`render --ego_safe`)是抛光层。把两者关系理清,别再有两份打架的实现 + 两个对不上的 A/B 脚本(`ab_runner.py` 解析器要对齐 `summary.csv` 的 cert/brake/hold)。
2. **EGO 接进 `eval_batch.py`**:出 EGO raw vs EGO+层 的成功率/碰撞/卡死/净空定量表(现在只有 per-seed mp4)。
3. **静态碰撞**:A/B 碰撞主来源是静态穿透(seed31 −0.793m,层没动作)。决定静态要不要也上证书门 / 或修 SFC 覆盖。
4. **MINCO 核稳定性回归**:最新调参退化到到达 12.5–54%、卡死 46–88%;查 recovery/regrasp 根因(与证书正交,但拖累 demo)。
5. **构建卫生**:`ego_capi.so` 变成 tracked/CMake 目标或明确 .gitignore(现手编 untracked、可能比源码旧);清掉误提交的 `cpp/build_v2/` 构建产物。
6. **committed-traj 的 capi getter**(`get_pwp` 暴露到 ABI),监视器要整条已承诺曲线时能拿到。

## 3. 以后再说(future work,需要时才动)

- **conformal 统计半边**:学习预测器 → split-CP 标定 → `q_conformal` 真值 → Mondrian 分层覆盖审计。这是把「几何证书」升级成「P(碰)≤ε 概率证书」的唯一路径。**当前 headline 不依赖它**;补了才能对外说概率保证。
- **最小修正 QP**(矫正那半 RTA):层从「只判」升级到「判 + 最小修正」。
- **S7-CRET**:对已认证轨迹做 C2 时钟 warp 的平滑认证逃逸,替 jerky 的 `recovery_climb`(现零代码)。
- **三感知分支**:统计 tube / 遮挡阴影 / depth→occupancy FN body 门(现 mover 全来自 GT)。
- **`minco_deficit_cert` 默认 ON** + cubic-EGO 路由从 demo 升为带测试的正式目标。
- **论文**(若决定投):差异化锚点见 spec §6(精确连续时间 × planner 无关 × 可证 sound);投稿前重扫 arXiv + 确认对手 venue。

## 4. 风险(每隔一阵过一遍)

| 信号 | 动作 |
|---|---|
| 改了证书 R 公式/数学 | 立即重核 `test_bernstein_cert` + golden 重基线,一次做完 |
| `ego_capi.so` 与源码不同步 | 改 EGO 后强制重编;长期改成 CMake 目标 |
| MINCO 核到达率持续掉 | 查 recovery/调参根因,别误归因到证书(它默认 OFF) |
| 对外材料出现「概率/语义风险保证」字样 | 立即纠正为「确定性几何证书」,直到补了 conformal |
| 同组动态+conformal preprint 出现 | 若以后要投论文,评估重叠;现在不阻塞 |
