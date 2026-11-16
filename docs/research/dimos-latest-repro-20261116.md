# 最新 DimOS 与 LingTu 同图回归

本文下表保留性能优化前的原始测量。最终 Release 已完成四图、每图 20 次 warm query，完整查询中位数依次为 9.047、9.780、8.869 和 121.160 ms，四图均使用 Rust graph、到达目标并通过同一几何复核。最新改动、入口语义、最终路线长度和复测方法见后续性能报告。

[Rust MLS 查询与 prepared cache 性能实测](mls-query-performance-20261116.md)

2026-11-16，Windows 原生 Release。LingTu 使用 `503b5b72` 加当前工作树，源码差异与编译命令保存在 `build/mls-regression-20261116/`。

DimOS 固定官方 main `83d807bb91bf6436a1a0e9c2a3260d96478636aa`，原 Rust crate、Cargo.lock、dimos-module 和 worker pool 按上游构建。新增 example 直接调用官方 `update_global_map`、`plan`、`update_region`。

[固定上游源码](https://github.com/dimensionalOS/dimos/tree/83d807bb91bf6436a1a0e9c2a3260d96478636aa/dimos/navigation/global_planner/mls_planner)

官方 Navigation Deep Dive 说明 Go2 的激光输入、体素地图更新、地形代价图、持续 A*、速度输出，以及探索和巡逻模块。该页展示的 costmap/A* 链路与 MLS 源码需要分别阅读。MLS 的 `region_tests` 展示局部表面、节点、边和 ownership 的维护。

[Navigation Deep Dive](https://github.com/dimensionalOS/dimos/blob/83d807bb91bf6436a1a0e9c2a3260d96478636aa/docs/capabilities/navigation/deep_dive.md)

本轮统一使用正式 `compile_run_plan(nav, real, doso/thunder_v4, scan)` 解析出的参数。机身半径 0.32 m，离地高度 0.435 m，上下净空 0.35/0.05 m，高度容差 0.10 m，最大步高 0.45 m，最大坡度 tan(30°)。四个案例使用同一参数集。upper_floor、stairs、multifloor 的端点保留原地面高度，机身高度换成 Thunder 的 0.435 m。这些资产是静态合成地图。

DimOS 接收相同 OctoMap 的完整占据体素。旧 dump 工具只输出压缩 leaf 的中心，会丢掉大 leaf 内的分辨率级体素。本轮加入 `--expand` 后，stairs 恢复为 3,040 格、multifloor 恢复为 14,208 格，与原地图构建报告一致。

两边共享占据体素、端点和可对应的构图参数。DimOS 原生接口缺少 LingTu 的 known-free 输入、最大坡度和机身扫掠合同。DimOS surface 高度使用体素顶面，LingTu 使用 cell center。DimOS 返回地面路线后仅加回 0.435 m，再交给与 LingTu 相同的 GeometryGate 复核。

| 场景 | 占据格 | LingTu 路线 / 复核 | DimOS 路线 / 同一复核 | 路线长度 LingTu / DimOS |
|---|---:|---|---|---:|
| arrival_box | 7,329 | Rust 成功，通过 | 返回路线，通过 | 3.600 / 3.677 m |
| upper_floor | 9,600 | Rust 成功，通过 | 返回路线，通过 | 2.400 / 2.402 m |
| stairs | 3,040 | Rust 成功，通过 | 返回路线，point_occupied_body，索引 4 | 6.563 / 6.943 m |
| multifloor | 14,208 | Rust 成功，通过 | 返回路线，point_occupied_body，索引 19 | 31.162 / 27.629 m |

失败索引从 0 开始。stairs 是第 5 点 `(1.910717, 0.08191028, 0.635)`，multifloor 是第 20 点 `(1.8064878, 3.88028, 0.635)`。两条 DimOS 路线在逐点机身检查阶段被拒绝，后续扫掠检查尚未执行。

LingTu 的 warm 计时覆盖完整 `PlannerSession::run`，包含端点接入、搜索、GeometryGate 和平滑。DimOS 的 warm 计时覆盖官方 `Planner::plan`，包含其端点接入和官方平滑，LingTu GeometryGate 在计时后单独执行。两列计量范围有这项差异，因此此表用于定位工作量与耗时，不计算算法速度倍数。

LingTu 每图 1 次预热后测 5 次查询，DimOS 每图 1 次预热后测 30 次查询。两套正式采样期间本任务无并行编译。机器未隔离，保留全部原始样本。5 个样本的 P95 实际为最大值。

| 场景 | LingTu warm 中位数 / P95 | DimOS warm 中位数 / P95 |
|---|---:|---:|
| arrival_box | 1490.641 / 1682.083 ms | 0.748 / 1.153 ms |
| upper_floor | 20294.231 / 21189.411 ms | 1.211 / 1.535 ms |
| stairs | 47.853 / 52.109 ms | 0.433 / 0.671 ms |
| multifloor | 546.010 / 551.921 ms | 6.641 / 7.712 ms |

LingTu 冷准备包含地图读取、静态准备、缓存写入，恢复包含地图读取与 prepared cache 读取。DimOS 冷构图仅计 `update_global_map`，地图读取、Planner 创建和线程池启动在计时外。

| 场景 | LingTu 冷准备 | LingTu 缓存恢复 | DimOS 冷构图中位数 | DimOS 区域添加 / 恢复 |
|---|---:|---:|---:|---:|
| arrival_box | 1041.670 ms | 509.460 ms | 20.592 ms | 0.991 / 0.386 ms |
| upper_floor | 4656.095 ms | 238.311 ms | 17.800 ms | 0.732 / 0.579 ms |
| stairs | 280.686 ms | 123.029 ms | 6.002 ms | 0.525 / 0.364 ms |
| multifloor | 13153.248 ms | 6428.374 ms | 64.168 ms | 0.610 / 0.224 ms |

四个 LingTu 案例的 prepare_count=1、map_load_count=1、cache_load_count=0，重复查询期间复用准备图。新 session 的 restored_cache_load_count=1，same_result=true。返回状态、路线坐标、Rust 使用与 fallback 状态全部一致。

慢查询已经定位到 `query.rs::snap_candidates()`。临时插桩副本中，arrival_box 每次检查 1,124 个起点连接和 969 个终点连接，耗时约 1.299 s，占 Rust query 的 97.5%。upper_floor 各检查 1,606 个起点和终点连接，耗时约 17.201 s，占 96.9%。这些候选逐个调用完整机身扫掠，缓存中的 surface 索引已复用。插桩数据用于热点定位，正式计时采用上表。

本轮性能修改已采用最近可执行入口优先、失败后扩展其他入口，并为 surface search 增加目标代价下界停止条件。修改还包括准备图复用、节点 spacing=1 快路径、OctoMap 导出扩容、缓存批量 I/O、地图占据查询复用及 occupied leaf 机身遍历。最近入口优先会影响入口及路线选择，最终结果按任务、楼层、几何、路线长度和 fallback 复核。新数值与原始采样保存在性能报告和 `build/mls-performance-20261116/release/report.json`。

| 请求中的工作 | 当前状态 |
|---|---|
| 最新 main 同图 benchmark | 已完成四图原始占据输入与官方 crate 运行 |
| warm query 统计 | 已实现正式 PlannerSession 计时、原始样本和重复结果检查；纯搜索/几何/平滑分段性能仍需补 |
| mapd → PlannerState | 持久区域生产者和正式应用入口待接 |
| GeometryGate 拒绝后的 Rust detour | 当前工作树 checkedQuery 已将几何回调用于 Rust 搜索；四图均 used_rust_graph=true、rust_fallback=false |
| stable EdgeId | ABI 7、artifact 格式 3 已保存和恢复；跨重启可更新 source 状态仍缺 |
| upper_floor / stairs / multifloor | 已使用统一正式 profile 完成静态路线和几何回归 |
| Rust global route → MuJoCo → 实机 | 本轮未覆盖动力学与硬件执行 |

mapd 接线有三个具体工作。首先由持久 OctoMap 事务提交完整 region occupied/free，使用固定世界网格和相同分辨率。其次给 cache 恢复补齐 occupied/free 与 SurfaceConfig，或从同一 OctoMap 恢复 source。最后在 PlannerSession 互斥边界内同时更新 OcTree 和 Rust、清查询缓存，再继续规划。

当前 `RegionUpdate` 导出的是 rolling collision 整窗快照。未标记的格子可能只是暂未观测，而 Rust replacement 会删除区域内旧证据。rolling occupancy 默认 0.05 m，保存 OctoMap 默认 0.10 m，且两边 cell 原点不同。直接接线会删除旧地面或误判空闲。当前 cache 的 createFromGraph 还将输入视作 surface，后续 occupied/free 更新会得到错误语义。

本轮验证包括 headless/verify/dump 的 Windows 与 WSL 构建、3 个 Python 测试及 4 个子案例、11 个 Rust query 测试、四图正式回归。官方最新 region_tests 为 30 passed，覆盖 stale removal、node relocation、seed/full equivalence、wall buffer、wipe/regrow 与连续 ownership 修复。

新增入口和证据保存在以下文件。

- `tools/bench/mls_regression.py` 运行 LingTu 计时和同参数几何复核。
- `tools/bench/dimos_benchmark.py` 与 `dimos_same_map.rs` 运行冻结上游。
- `build/mls-regression-20261116/cases.json` 保存本次正式解析参数及全部端点。
- `build/mls-regression-20261116/windows-final/report.json` 保存 LingTu 结果，各案例目录保留请求、日志和路线。
- `build/dimos-latest-20261116/windows-production/report.json` 保存 DimOS 原始样本、命令与几何结果。
- `build/query-profile-20261116/` 保存回调计数、插桩副本和直接连接诊断。

复现 LingTu 时使用新的输出目录，避免将已存在的缓存当成冷准备。

```powershell
.venv/Scripts/python.exe tools/bench/mls_regression.py --cases build/mls-regression-20261116/cases.json --planner build/artifact-check/Release/octoplanner3d_headless.exe --verifier build/artifact-check/Release/octoplanner3d_verify_path.exe --output build/mls-regression-repeat --samples 5 --timeout 240
```

复现已冻结的 DimOS 源码构建与测试。首次获取源码时加 `--fetch`。

```powershell
.venv/Scripts/python.exe tools/bench/dimos_benchmark.py --cases build/mls-regression-20261116/cases.json --source-root build/dimos-latest-20261116/rust --output build/dimos-regression-repeat --dump build/artifact-check/Release/octoplanner3d_dump_octomap.exe --verify build/artifact-check/Release/octoplanner3d_verify_path.exe
```

旧的 6.86 ms / 0.665 ms 来自 `26144f7` 的另一组配置，保留在历史报告中。

[历史同图报告](mls-same-map-benchmark-20261115.md)
