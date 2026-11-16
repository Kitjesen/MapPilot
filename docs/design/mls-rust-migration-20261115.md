# MLS Rust 迁移

更新日期为 2026-11-16。

生产链由 OctoMap 快照、Rust MLS prepared artifact 和 OctoPlanner3D 请求适配组成。
Rust 准备 observed/inferred 多层表面及稀疏路网，C++ 将候选表面格转换为机身位置，
检查端点连接及路线几何。底层仍使用固定版本 OctoPlanner3D 和 DimOS 的算法来源，
本地适配范围见 `src/nav/cpp/planning/global/octoplanner/SURFACE_NOTICE`。

## 当前职责

| 数据或计算 | 拥有者 |
| --- | --- |
| 原始占据、已知 free 与未知证据 | 原生 OctoMap 快照 |
| observed/inferred 多层表面 | Rust `lingtu_mls_core` |
| 四邻域、wall distance、稀疏节点与 Voronoi corridor | Rust `lingtu_mls_core` |
| 端点吸附、图路线、一次受阻绕行重试 | Rust graph query |
| 原始地面支撑、机身净空、步高、坡度与连续扫掠 | C++ OctoMap 几何查询 |
| 推断补面支撑与路线高度拟合 | Rust prepared surface query |
| 地图身份、请求取消、缓存文件与 DDS endpoint | C++ `PlannerSession` 与 native endpoint |
| 实时避障、局部轨迹和速度跟踪 | 现有 SCAN 与执行链 |

`PlannerArtifact` 持有一个 Rust graph handle。原来的 C++ `SurfaceIndex` 已退出生产
准备、规划查询和 prepared cache。独立 C++ `SurfaceGraph` 用于算法回归测试。
生产请求查询 Rust 候选，逐段复核几何。几何拒绝时，最多携带一个拒绝格向 Rust
请求绕行，再进入原有 XYZ 网格 A* 及其细分辨率重试。

表面候选按列净空筛选，因此某个低机身姿态仍可通过的地面可能没有进入图候选。
原始支撑查询直接读取 OctoMap 中暴露的占据表面，XYZ 回退可以继续使用这些地面。
Rust 推断的补面单独提供支撑，明确观测为 free 的格子不会被补成地面。

机器人半径、上下净空与支撑高度来自所选 RobotConfig，搜索和补面参数来自解析后
的 `native_nav.global`。Rust 表面准备使用 closing 半径和列净空。
`config/system/settings/maps.yaml` 的 `maps.surface.max_hole_area_m2` 与 `max_slope`
为 C++ 地图清理的构建默认值，修改后重新构建原生程序。
本地几何复核包含 LingTu 的机身和运动约束，参数不能解释成
DimOS 或原版 OctoPlanner3D 的默认值。

## 准备与查询

准备阶段将 OctoMap 的 occupied 叶节点展开，调用 Rust 提取表面，并只对拟补格
查询原生 free 证据。空闲体积由 OctoMap 持有。Rust 构图后，导航保留表面和图。

地图或 `PlannerConfig::preparation()` 中的参数变化会重新准备。目标 XYZ/XY/Z
容差、端点吸附半径、迭代预算和同层偏好更新本次查询，复用已有 artifact。
当前 regional update 算法为首次实际更新延迟创建其增量图状态，避免初次构图与
缓存恢复重复构图。

取消回调在 C++ 准备循环、Rust 调用前后、候选验证及平滑后查询。取消结果会清空
路线并报告 `Cancelled`。当前 Rust FFI 内部尚无取消回调，单次调用耗时仍影响取消
响应时间。

## 持久化

`PlannerSession` 在地图目录 `.nav/profile-N.bin` 保存可重建的派生 artifact。
外层格式为 `lingtu.navigation.prepared.7`，内容包含地图身份、源文件信息、准备参数、
Rust observed/inferred cells、nodes、edges、corridors 及 ABI 版本。旧格式失效后
重新准备。保存不会改写原始点云或 OctoMap。

恢复路径通过 `createFromGraph()` 安装已保存的表面和图。图更新状态按需创建，
缓存加载无需再次构图。地图源信息、准备参数或 ABI 不匹配时重建该 profile。

原始 occupied/free 更新只适用于持有对应源数据的 Rust handle。导航创建及缓存
恢复的 surface/graph handle 拒绝该入口，避免把 known-free 更新误读成 inferred
surface。mapd 已有 `RegionUpdate` 数据契约，生产导航尚未接入这个增量输入，地图
变化目前仍重新准备完整 artifact。

## 历史同图对照

以下数值来自 2026-11-15 的 arrival-box 离线对照，供说明迁移过程。当前版本需按
相同地图、参数和端点重新测量，不能沿用这些数字作为本次构建或现场验收结果。

| 指标 | DimOS | LingTu Rust | 历史 LingTu C++ |
| --- | ---: | ---: | ---: |
| surface cells | 2,241 | 2,240 | 2,240 |
| sparse nodes | 29 | 29 | 36 |
| graph edges | 47 | 45 | 43 |
| graph components | 2 | 2 | 9 |
| graph build | 6.86 ms 完整冷构图 | 8.6–14.1 ms | 11.4–22.3 ms |

逐次试验记录保存在 `docs/research/mls-surface-compare-arrival-box-20261115.md`。
曾用于比较 C++ `SurfaceIndex` 与 Rust 的 `mls_surface_compare.cpp` 已退役，历史版本
可从提交 `79f9562a` 获取。当前全局请求和路线几何验证使用
`octoplanner3d_headless`、`octoplanner3d_verify_path` 及相应 smoke。

## 本轮验证与剩余范围

2026-11-16 在独立集成工作树验证：Rust 25 项、Windows Release 全局规划 CTest
19 项、配置与 RunPlan 19 项全部通过。地图模块编译成功，两个迁移头文件相关测试
通过；`test_active_octomap_gate` 也已独立构建并验证新快照恢复同一份地面路网。
GitHub 的 Linux global job 同时通过 Rust 和全局规划构建与测试。

`prepared_cache_smoke` 对同一地图、参数及端点比较首次准备、内存复用和磁盘恢复，
验证路线一致、补面保留、查询参数不重建、机身变化切换 profile、原图不变，以及
截断缓存重新准备。`surface_smoke` 保留已知 free、原始支撑、上下层、机身净空与
扫掠回归。取消回归验证空路线及规划器可重用，尚不证明 Rust 调用可被中途打断。

- 接入 mapd regional delta 时再验证增量更新与整图重建的一致性。
- 分别记录离线、MuJoCo 与实际机器人验证，现场执行仍需独立验收。
