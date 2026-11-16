# Rust MLS 查询与 prepared cache 性能实测

最新复核补充。本文的历史四图通过结论对应原采样 checker。历史 before 可执行文件经后续解析扫掠 verifier 重查，stairs 有 5 段、multifloor 有 1 段连续扫掠失败，原始记录保存在 `build/mls-deep-20261116/paired-before/report.json`。当前已构建共享最新 C++/Rust 几何规则的隔离 before 与最终 after，四图 ABBA 对照的 16 批全部通过、路径逐点相同。最新证据及本轮独立性能收益见后续 deep-dive 报告，本文原始历史数据继续保留。

[最新几何复核与隔离 ABBA 性能对照](mls-plan-deep-dive-20261116.md)

2026-11-16。本文记录同图回归发现的热点、本轮修改及最终 Release 复测。四个案例每图预热 1 次后采样 20 次，全部使用 Rust graph、到达目标并通过同一三维 GeometryGate。

## 输入和计时范围

DimOS 固定官方 main 提交 `83d807bb91bf6436a1a0e9c2a3260d96478636aa`。基准直接调用该提交 Rust crate 的 `update_global_map`、`plan` 和 `update_region`，保留 Cargo.lock 和所需上游依赖。

[固定 DimOS MLS 源码](https://github.com/dimensionalOS/dimos/tree/83d807bb91bf6436a1a0e9c2a3260d96478636aa/dimos/navigation/global_planner/mls_planner)

LingTu 的前后对照使用 Windows 原生 Release 和同一组保存地图、端点、Thunder 机身参数。基线二进制与源码副本保存在 `build/mls-performance-20261116/baseline/`，最终结果位于 `build/mls-performance-20261116/release/report.json`，案例清单位于 `build/mls-performance-20261116/cases.json`。最终源码和构建复现元数据归档在 `release/source/`。这些参数来自正式 `compile_run_plan(nav, real, doso/thunder_v4, scan)`。半径为 0.32 m，离地高度为 0.435 m，上下净空为 0.35/0.05 m，高度容差为 0.10 m，最大步高为 0.45 m，最大坡度为 tan(30°)。

| 案例 | 占据体素 | 起点 XYZ / m | 终点 XYZ / m |
|---|---:|---|---|
| arrival_box | 7,329 | (1.5, 0, 0.444792) | (5.1, 0, 0.435) |
| upper_floor | 9,600 | (-1.2, 0, 0.935) | (1.2, 0, 0.935) |
| stairs | 3,040 | (0.75, 0, 0.435) | (7.1, 0, 1.635) |
| multifloor | 14,208 | (7, -0.8, 0.435) | (7.4, 2.8, 4.035) |

DimOS 使用同一 OctoMap 展开后的完整分辨率级 occupied voxels。输入导出使用 `octoplanner3d_dump_octomap --expand`，包括压缩 leaf 内的全部体素。DimOS ground route 加回 0.435 m 后，交给 LingTu 同一 GeometryGate 复核。

| 字段 | LingTu | DimOS |
|---|---|---|
| cold_prepare_ms | 地图读取、静态准备、cache 写入 | update_global_map，地图读取和线程池创建在计时外 |
| warm query | PlannerSession::run，包含端点接入、搜索、GeometryGate、平滑 | 官方 Planner::plan，包含官方端点接入和官方平滑 |
| cache restore | 新 session 读取 OctoMap 和 prepared artifact | 当前同图 runner 未测对应完整恢复 |
| geometry verification | warm query 内执行，另复核最终路线 | plan 计时后执行同一 LingTu verifier |
| local update | Rust 核心局部更新已有，正式 mapd 输入待接 | 官方 update_region |

因此两者 warm 查询包含的机身检查工作量不同。本文保留原始数值，算法速度倍数应使用后续分段计时。LingTu 修改前后使用相同完整入口，适合评估本轮产品查询的延迟变化。

## 修改前的正式采样

原始报告位于 `build/mls-regression-20261116/windows-final/report.json`。每个案例预热 1 次后查询 5 次，P95 取样本最大值。DimOS 原始报告位于 `build/dimos-latest-20261116/windows-production/report.json`，每图预热 1 次后查询 30 次。采样期间本任务无并行编译，机器未隔离。

| 案例 | LingTu cold / ms | LingTu restore / ms | LingTu warm 中位数 / P95 / ms | DimOS warm 中位数 / P95 / ms |
|---|---:|---:|---:|---:|
| arrival_box | 1041.670 | 509.460 | 1490.641 / 1682.083 | 0.748 / 1.153 |
| upper_floor | 4656.095 | 238.311 | 20294.231 / 21189.411 | 1.211 / 1.535 |
| stairs | 280.686 | 123.029 | 47.853 / 52.109 | 0.433 / 0.671 |
| multifloor | 13153.248 | 6428.374 | 546.010 / 551.921 | 6.641 / 7.712 |

四个基线案例均返回 Rust 路线并通过最终三维几何复核，`used_rust_graph=true`、`rust_fallback=false`。重复查询与 cache 恢复后的结果一致。

端点连接的诊断副本显示，arrival_box 每次检查 1,124 个起点连接和 969 个终点连接，约 1.299 s，占该次 Rust query 的 97.5%。upper_floor 分别检查 1,606 个起点和终点连接，约 17.201 s，占 96.9%。连接候选逐一调用完整机身扫掠。诊断文件在 `build/query-profile-20261116/`。

## 已完成的修改

### 端点先选择最近的可执行入口

`SnapCandidates` 先按三维距离排列候选，逐个执行几何连接检查，找到起点和终点最近的可执行 surface 后开始搜索。只有这组入口无法连通目标时，才继续加入其余允许范围内的候选。替代入口、surface detour、取消和共享 iteration budget 仍进入同一 query。

这改变了原来的入口选择语义。原实现先检查所有候选，再在所有入口组合间寻找路线。现在优先固定最近的可执行入口，优先保留用户选中的表面和楼层。路线坐标或长度允许随入口选择变化，最终仍必须通过相同机身几何检查。完整比较应记录路线长度、楼层、成功状态、fallback 及几何结果。

`surface_search` 使用到目标的代价下界，在开放队列已经无法改进当前目标代价时结束搜索。短距离请求先在起终点附近的 surface 范围寻找路径，再使用稀疏 graph 和必要的 surface detour。已通过的连接检查由单次 query 复用。

同一次 query 还按机身姿态复用 stance admission。同一 surface 格被相邻 corridor 多次访问时，其相同机身检查结果只求一次。surface A* 在相同 f 值时优先扩展更大的 g，以减少到达目标前的等价扩展。稀疏 graph 保留原先较小 g 的顺序。试验将该 tie-break 一并用于 sparse search 后，multifloor 路线增加约 1.49 m，因而该 sparse 修改已撤回。20×20 等价多路径小测试曾从 399 次几何回调减少到 74 次；这个 sparse 排序已撤回，该数字不计入最终保留实现的收益。

相关测试覆盖最近入口、较远可达入口、同 XY 多层、预算耗尽、取消和 geometry rejection 后的 detour。实现在 `src/kernels/nav/mls_core/src/query.rs`。

### 减少冷准备的重复构图和复制

`PlannerState::new`、`from_surface` 和 `from_graph` 根据已经生成或恢复的 graph 初始化 `IncrementalGraph`。原路径会再次调用 build_graph。新路径保留同一 node ID、edge ID 和 corridor，以支持后续区域更新。

`nodes.rs` 对 `node_spacing_cells=1` 使用整数网格的距离性质。不同 XYZ 整数格之间距离至少为 1，因而可直接收集通过墙距筛选的格子，省去候选逐个扫描已选节点。其他 spacing 使用原有 NMS 逻辑。测试将新旧节点集合逐项对照，覆盖不同 spacing 和多高度层。

`rebuildRustGraph` 导出 OctoMap leaf 时取消每个 leaf 的精确 reserve。原代码不断按 `size + leaf_cells` 扩容，会反复复制已累积体素。vector 继续使用正常容量增长，导出的 occupied/free 内容由同输入实验核对。

逐 leaf 导出的独立微基准从 248.781 ms 降到 2.101 ms，新旧 occupied/free 数组逐项相等。该耗时只覆盖输入体素展开，完整冷准备数据见最终复测表。

### 复用地图占据查询并减少机身碰撞遍历

`occupied_blocks_` 缓存的是固定地图的占据、空闲、未知和 leaf center 状态，按 setOctomap/setPreparedRustGraph 等地图准备边界失效。makePlan 和 preparePoseQueries 继续重置各自路线或姿态查询状态，同时复用原地图占据查询。新 no-go 对象只清除受策略影响的 body_cache。现有 grid smoke 验证不同 anchor、新 no-go、下一次 route 查询的缓存复用，以及同一 OcTree 指针经 setter 重新提交后的失效。

`queryWorld` 按 occupied leaf 的包围盒遍历相交范围，跳过没有占据证据的区域。相交判断继续使用原 finest-cell cylinder 规则，并逐格应用 start exemption。混合深度叶、细栅格圆柱、unknown、外部禁区与起点豁免的既有几何合同由回归验证。

### 减少 prepared cache 导入导出成本

缓存恢复将 nodes 和 edges move 到临时 RustGraph，保存时 move FFI 已导出的 surface 和 graph，省去中间深拷贝。FFI 导入 corridor 前一次性 reserve 总容量。Rust `read_graph` 直接从 FFI slice 生成各 edge corridor，省去整份 corridor 的中转 Vec。

BinaryReader/Writer 复用现有 vector 批量读写 observed、inferred、nodes 和 corridor。编译期约束与 size/offsetof 断言确保这些 record 具有原逐字段字节布局。原 count、remaining、截断异常及 artifact 内容检查继续执行。

独立微基准使用项目 BinaryReader/Writer，WSL `g++ -O2`，1,000,000 个 cell，共 12,000,008 字节，5 轮。每轮新旧编码逐字节相等。写入中位数从 64.791 ms 降到 13.234 ms，读取从 61.287 ms 降到 11.899 ms。该结果只描述批量序列化，完整恢复收益以最终 prepared benchmark 为准。临时源文件为 `build/cache-record-bench.cpp`。

### 修复跨 ownership 边的软代价

`edges.rs` 原先在 Voronoi ownership 交界的最后一段只加入基础距离，漏掉该段 step penalty 和 wall penalty。新实现使用与 ownership 内部一致的 edge_penalty。非零软代价下，边成本及最终选择可能变化。

外层 prepared 格式已更新为 `lingtu.navigation.prepared.7`，使旧成本缓存失效并自动重建一次。内部 MLS artifact 格式仍为 3。保存的原始 OctoMap 内容保持原值，重新生成的是导航 prepared artifact。

## 最终 Release 复测

原始数据位于 `build/mls-performance-20261116/release/report.json`。每图 20 次 warm query，P95 使用第 19 个有序样本。cold 和 restore 各测 1 次，未将单次值解释为稳态分布。完整原始 query_ms 保存在报告中，采样期间本任务无并行编译。

| 案例 | 修改后 cold / ms | 修改后 restore / ms | 修改后 warm 中位数 / P95 / ms | 路线长度 / m | 几何 / Rust fallback |
|---|---:|---:|---:|---:|---|
| arrival_box | 48.820 | 13.890 | 9.047 / 12.696 | 3.613871 | 通过 / false |
| upper_floor | 101.752 | 13.530 | 9.780 / 14.589 | 2.400000 | 通过 / false |
| stairs | 21.801 | 6.022 | 8.869 / 11.654 | 6.686159 | 通过 / false |
| multifloor | 111.762 | 32.909 | 121.160 / 129.303 | 30.915221 | 通过 / false |

四图均为 `used_rust_graph=true`、`rust_fallback=false`，目标 XYZ 误差为 0。每图准备次数为 1，重复查询没有加载或重建地图。新 session 命中磁盘 cache，恢复后的任务结果与各自最终首次结果一致，`same_result=true`。这里的 same_result 比较同一最终实现的重复查询与恢复。

本次同时包含独立的路线平滑修改，平滑保留高度过渡点。相对 `after/` 中间结果，arrival_box 增加约 1.4 cm，stairs 增加约 12.2 cm。相对修改前完整基线，multifloor 从 31.162104 m 变为 30.915221 m。前后路径长度和路点可能变化，表中给出最终实际长度。`after/`、`final/`、tie-break 实验均保留为过程证据，最终交付使用 `release/`。

## 验证和复现

Rust 测试 57 项通过，覆盖 query、恢复 ID、restore 后区域更新、节点放置等价、ownership 交界成本和取消语义。最终 C++ native smoke 7/7 通过，覆盖 route check、MLS backend、route smoothing、grid、prepared cache、no-go 和 no-air-climb。日志位于 `build/mls-performance-20261116/tests-release.log`。缓存测试包含原字段编码字节等价和截断缓存处理。

```powershell
cargo test --manifest-path src/kernels/nav/mls_core/Cargo.toml

cmake --build build/artifact-check --config Release --target octoplanner3d_headless octoplanner3d_verify_path octoplanner3d_mls_backend_smoke octoplanner3d_prepared_cache_smoke octoplanner3d_route_check_smoke octoplanner3d_route_smoothing_smoke octoplanner3d_grid_smoke octoplanner3d_no_go_smoke octoplanner3d_no_air_climb_smoke

ctest --test-dir build/artifact-check -C Release --output-on-failure -R "octoplanner3d_(mls_backend|prepared_cache|route_check|route_smoothing|grid|no_go|no_air_climb)_smoke"

.venv/Scripts/python.exe tools/bench/mls_regression.py --cases build/mls-performance-20261116/release/cases.json --planner build/artifact-check/Release/octoplanner3d_headless.exe --verifier build/artifact-check/Release/octoplanner3d_verify_path.exe --output build/mls-performance-repeat --samples 20 --timeout 240

.venv/Scripts/python.exe tools/bench/dimos_benchmark.py --cases build/mls-performance-20261116/cases.json --source-root build/dimos-latest-20261116/rust --output build/dimos-performance-repeat --dump build/artifact-check/Release/octoplanner3d_dump_octomap.exe --verify build/artifact-check/Release/octoplanner3d_verify_path.exe
```

每次冷准备实验使用新的 output/cache 目录。复测期间停止本任务的并行编译，保留全部 query_ms 原始样本。修改前后的比较同时检查到达、Rust 使用、fallback、目标误差、路线长度、GeometryGate 和重复结果。入口语义变化后，用几何和任务结果核对路径，记录坐标差异。

## 剩余耗时的实际分布

最终源码的独立插桩副本保存在 `build/query-opt-20261116/release-profile/`。它没有替换正式二进制，插桩时间不进入上表中位数。在一个 multifloor 热查询样本中，Rust 搜索约 80.5 ms、完整路线复核约 3.4 ms、平滑约 33.0 ms。该次搜索的起终点各只检查 1 个连接；查询内检查 5,504 条不同有向边，没有同方向重复回调。

其中，起终点附近的 surface 搜索约 37 ms 后没有连通，再进入两端接入树和稀疏 graph；平滑还需要约 33 ms。这是目前可继续研究的主要工作量。要进一步减少，需要用同图证据改进局部搜索的选择和保留地形高度的平滑策略；不能直接删除这个搜索分支或跳过它负责的几何检查。正式生产的分段指标尚未加入 API，当前只是离线诊断。

## 正式区域更新仍需完成

mapd 当前 RegionUpdate 来自 rolling collision 整窗快照。未标记格可能暂未观测，保存地图与 rolling grid 的分辨率和原点也不同。正式生产接线需要持久 OctoMap 提交完整 occupied/free 区域，在固定世界网格内调用 PlannerState 更新，并在同一 PlannerSession 互斥边界更新 OcTree、Rust artifact 和查询缓存。

prepared cache 恢复目前导入 surface/graph。跨重启后继续做 source 区域替换，还需要从相同 OctoMap 恢复 occupied/free 和 SurfaceConfig。本文的性能优化已覆盖静态 saved-map 查询、恢复及 native 几何回归，MuJoCo 动力学和实机运动的完整 Rust 路线执行验证仍待完成。

[同图 DimOS 回归及生产缺口](dimos-latest-repro-20261116.md)
