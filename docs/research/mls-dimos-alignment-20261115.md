# LingTu MLS 与 DimOS MLS 对齐审计

日期：2026-11-15

本轮把当前 Rust `mls_core`、C++ `SurfaceIndex/SurfaceGraph` 与本机保存的 DimOS
源码快照逐项对照。DimOS 参考快照为：

```text
build/dimos-review-20261025/dimos/navigation/global_planner/mls_planner/rust/src
commit: 26144f7fd8424bdc33ab97dcd58e9497f2d413ee
```

DimOS 的相关实现文件是 `surfaces.rs`、`adjacency.rs`、`nodes.rs`、`edges.rs` 和
`mls_planner.rs`。本轮没有复制 DimOS 源码到产品目录，比较只使用源代码和已有
离线 runner 结果。

## 最新结论（2026-11-15 复测）

现在已经把 Rust 原生 `build_graph()` 接入离线摘要 ABI，并和同一批 Rust
surface cells 生成的 C++ `SurfaceGraph` 做了实测。Rust graph 已经从旧的
“节点之间找最近邻”改成四邻域表面边、wall-distance 节点筛选、多源 Voronoi
ownership 和 predecessor corridor。

Rust core 现在已经按职责拆成：

```text
surfaces.rs   surface extraction and inferred cells
adjacency.rs  four-neighbor surface adjacency
nodes.rs      wall distance and sparse node placement
edges.rs      Voronoi edge construction and edge cost
corridor.rs   predecessor corridor recovery
lib.rs        public types, artifact entry point and C ABI
```

| 同一份地图 | C++ SurfaceGraph nodes / edges / components | Rust 原生 nodes / edges / components | Rust corridor |
| --- | ---: | ---: | ---: |
| arrival-box | 36 / 43 / 9 | 30 / 50 / 1 | 711 cells，最大 22 |
| body-depth/align | 11 / 13 / 3 | 10 / 10 / 3 | 295 cells，最大 41 |
| body-depth/upper_floor | 28 / 48 / 2 | 17 / 26 / 2 | 905 cells，最大 53 |

现在 Rust 原生图的规模已经与 C++ 图处于同一数量级，corridor 也保存了中间
表面单元。完整 node、edge、corridor 已经通过 C ABI 导出，并由 C++ smoke 验证
了 corridor 链。离线对照工具已经完成最近节点接入、Dijkstra 路线恢复、逐 edge
几何过滤和同一 C++ 几何复核。arrival-box 原始 Rust route 在第 41 段被
`metric_limits` 拒绝，support、body 和 sweep 检查已经通过；过滤后保留 34 条边、
修复 6 条局部可执行连接、拒绝 16 条边，最终路线通过同一套 C++ 几何复核。

## 剩余差异的处理方式

不要为了让三个数字相等而继续压缩 Rust 图。当前剩余差异来自两个不同层次：
Rust/DimOS 风格图先生成拓扑候选，LingTu C++ 再根据真实机器人几何决定候选
是否可执行。下一步按下面的边界处理：

1. Rust 输出完整 `nodes / edges / corridor`，不再只输出摘要。
2. C++ 适配层对每个节点做机身和支撑查询，对每条 corridor 做步高、坡度和
   连续扫掠查询；被拒绝的边从候选图中删除，并重新计算组件。
3. 用过滤后的 Rust 图再做起终点接入、路线搜索和同一条 C++ 几何复核。
4. 只有过滤后的 Rust 图在固定地图上与 C++ 的连通性、路线长度和几何结果稳定，
   才进入 `PlannerArtifact` A/B；地图局部更新和正式运行切换放在这之后。

这能保留 Rust/DimOS 图的拓扑表达，同时不把 Rust 没有的机身占据和支撑证据
硬塞进算法核心，也不会让未经复核的连接直接进入 SCAN。

## 当前仿真门结果

使用当前 Release `octoplanner3d_headless` 和仓库已有的
`saved_map_plan_gate.py`，每个场景重新生成 OctoMap，再执行全局规划和 C++ 三维
路线复核。没有下发硬件速度命令。

| 场景 | 地图构建 | 全局路线 | 到达 | 路点 | 查询耗时 |
| --- | --- | --- | --- | ---: | ---: |
| `corridor` | 通过 | 通过 | true | 14 | 245.96 ms |
| `stairs3d` | 通过 | 通过 | true | 8 | 16.55 ms |
| `multifloor_stack_3` | 通过 | 通过 | true | 75 | 280.15 ms |
| `nav_3d_course` | 通过 | 通过 | true | 152 | 889.43 ms |

证据目录：

```text
build/mls-alignment-sim-corridor-20261115/
build/mls-alignment-sim-stairs-20261115/
build/mls-alignment-sim-multifloor-20261115/
build/mls-alignment-sim-course-20261115/
```

这些结果证明当前地图构建、全局路线生成和 C++ 几何复核在这四个仿真地图上可用。
它们没有证明 SCAN 局部跟踪、MuJoCo 动力学、策略步态或实机行走。

## 三维课程的配置修正

历史版本的 `nav_3d_course` 验收配置使用 `support_height_m = 0`。当时
`rebuildSurfaceGraph()` 会直接跳过 SurfaceGraph，`startPlan()` 随后进入 26 邻域
的 XYZ A*。这不是正式三维导航应有的运行方式。

现在课程配置已统一为：

```text
ground_support = true
direct_support_only = true
support_height_m = 0.45
support_height_tolerance_m = 0.11
max_step_height_m = 0.25
max_slope = 0.65
lowest_traversable_only = false
```

同一条课程路线重新运行后的原生日志为：

```text
SurfaceIndex: 16,313 observed + 0 inferred, prepare_ms = 52.132
SurfaceGraph: 620 nodes, 1,262 edges, prepare_ms = 255.240
SurfaceGraph query: graph_ms = 5.842, smooth_ms = 1.589, 59 waypoints
reached_goal = true
```

现在这条正式三维课程已经确认运行在 SurfaceGraph 上，没有再走 XYZ A*。saved-map
gate 也通过，目标三维误差为 0。生产 `PlannerSession` 已有地图和准备图缓存，单次
`runPlan()` 是否承担冷启动成本仍取决于调用入口，不能把这个离线 gate 的准备时间
当成 warm query 时间。

原生 headless 结果现在直接输出 `diagnostics.used_surface_graph=true`，验收可以按
结果字段判断路线是否使用 SurfaceGraph，不再需要解析日志中的提示文本。

## 源码语义对照

| 层 | DimOS MLS | 当前 LingTu | 对齐结论 |
| --- | --- | --- | --- |
| 输入地图 | 占据体素集合，free/unknown 不进入 MLS | Rust 对照入口接收 occupied 和 known-free，生产地图由 C++ OctoMap 持有 | 输入边界不同，不能直接称等价 |
| 表面提取 | 列内 `robot_height` 净空，得到 standable cells | observed surface 使用相同的列内净空语义 | 观测表面已对齐 |
| 补面 | 每个 Z 层做 L1 `dilate` 再 `erode`，并检查邻域支撑 `has_support` | 当前 Rust 对照实现使用 C++ 风格的 bounded-hole、边界平面、坡度、残差、known-free 和 headroom 规则 | 已与 C++ 对齐；仍不是 DimOS 原始 closing |
| 邻接 | 四邻域 XY，允许配置的垂直步高 | Rust graph 已使用四邻域 XY 和配置的垂直步高 | 基本对齐 |
| 节点 | wall distance、硬墙距、局部极大值 NMS、组件至少一个节点 | Rust 已使用 wall distance、四格 hole-span wall boundary、spacing NMS 和组件兜底节点 | 基本对齐，节点 relocation 仍未实现 |
| 边 | 多源 Dijkstra Voronoi 边界，保存完整 corridor chain | Rust graph 已使用多源 Dijkstra、ownership boundary 和 predecessor corridor | 基本对齐，代价细节仍需对齐 |
| 代价 | 墙距硬门槛、墙距软惩罚、步高惩罚 | Rust 已保留 wall buffer、wall penalty 和 step penalty 参数，但 C ABI 摘要暂时使用默认值 | 部分对齐 |
| 地图更新 | `update_region` 局部替换表面、节点、边，保留稳定 CellId | Rust `PlannerState.update_region` 已接入稳定图的局部节点、边和 corridor 修复；没有足够局部节点时回退整图 | Rust 生命周期已接入 C ABI，mapd delta 仍缺 |
| 执行复核 | DimOS planner 返回表面路径 | LingTu 继续由 C++ 负责 body、support、step、slope、continuous sweep | 这是有意保留的系统边界 |

## 同一份 OctoMap 的现有 A/B 证据（修正前历史快照）

当前对照工具使用同一份
`build/arrival-box-20261027/occlusion-map-final/same_source_map/octomap.ot`：

| 指标 | C++ SurfaceIndex | Rust MLS |
| --- | ---: | ---: |
| occupied cells | 7329 | 7329 |
| observed surface | 2240 | 2240 |
| inferred surface | 0 | 4 |
| XY columns | 2240 | 2240 |
| 多 Z 层 columns | 0 | 4 |
| 图 nodes | 36 | 39 |
| 图 edges | 43 | 53 |
| 图 components | 9 | 9 |
| 起点接入 | true | true |
| 终点接入 | true | true |

observed surface 逐格集合一致。Rust 的 4 个 inferred cell 改变了图节点和边数量。
Rust 图路线送入同一 C++ 几何复核后，第 0 段失败为 `ground_support`。C++ 生产路线
通过 body、support、step、slope 和 continuous sweep。

因此修正前状态是：

```text
仿真全局规划门       4/4 通过
observed surface     已对齐
DimOS surface closing 未对齐
DimOS node/edge       未对齐
Rust 路线几何复核     未通过
生产 PlannerArtifact 仍使用现有 C++ SurfaceIndex/SurfaceGraph
```

这段保留用于解释本轮修复前的差异。修正后真实结果见上面的“最新结论”，
Rust surface inferred 已与 C++ 对齐，Rust 原生 graph 的差距则仍然存在。

## 对齐顺序

1. 保留现有 C++ 生产路径，新增一个离线 DimOS-compatible surface 实现，只复现
   `surfaces.rs` 的 L1 closing 和 `has_support`，先在同一 occupied columns 上比较
   observed、inferred、同 XY 多层数量。
2. Rust graph 的四邻域、step threshold、wall distance NMS、Voronoi Dijkstra 和
   corridor chain 已完成第一版。下一步是把 wall boundary 判定、软代价和组件接入
   规则逐项和 DimOS 对照，再比较 nodes、edges、components、端点接入和路线长度。
3. 使用已导出的完整 node、edge、corridor 完成起终点接入，并让 Rust route 继续通过同一个 C++ body/support/step/slope/swept verifier。只有
   表面、图和几何三层同时稳定，才讨论替换 `PlannerArtifact` 的生产准备阶段。
4. 增量 `update_region` 放在全量对照通过之后，避免同时引入算法差异和生命周期差异。

SCAN、Follower、Driver 不参与这轮 MLS 对齐，也不改变现有运行链路。

## 局部图生命周期复测（2026-11-15）

Rust `PlannerState` 现在持有 source map、surface artifact 和 `IncrementalGraph`。
一次区域更新会在 surface halo 内重提取表面，按稳定 surface cell 复用 node ID，删除
失效节点，失效 corridor 关联的 edge，并在窗口内重新计算 wall-distance、Voronoi
ownership 和 predecessor corridor。区域外 node 和 edge 保留，稳定 edge ID 使用
端点 node ID 对生成，edge 自己持有 corridor。

如果更新后局部窗口没有可用节点而旧图仍有受影响边，Rust 会用完整图替换并在
`graph_rebuilt_globally` 标志中报告回退。这个回退保证查询不会拿到断边图，同时保留
局部 repair 的正常路径。

当前验证结果：

```text
Rust unit tests: 13 passed
prepared_surface / surface_graph / surface / prepared_cache / mls_backend smoke: 5 passed
```

这已经实现了 Rust 离线图生命周期的核心模块。`mapd` 的真实 region delta、正式
`PlannerArtifact` 选择、SCAN/Follower/Driver 接入和实机证据仍未完成，因此不能写成
DimOS 那样的完整生产 MLS runtime。

## Corridor geometry repair复测（2026-11-15）

Rust graph 的 corridor 是拓扑候选，部分相邻表面格的高度变化超过当前 C++
metric slope 限制。直接逐格送入执行复核会把整张 Rust 图误判为不可用。对照工具
现在在 C++ 适配层对每条被拒 corridor 做最小子序列选择，只保留已经通过同一条
`metricConnectionWithinLimits`、support、body 和 continuous sweep 合同的跳转。
没有调大步高、坡度或净空阈值，也没有改变 Rust 图的节点和候选边。

arrival-box 的复测结果如下：

| 项目 | 修复前 | 修复后 |
| --- | ---: | ---: |
| Rust native edges | 50 | 50 |
| 几何可执行边 | 28 | 34 |
| 子序列修复边 | 0 | 6 |
| 仍拒绝的边 | 22 | 16 |
| 起终点路线 | 断开 | 连通 |
| 过滤后路线几何 | 无路线 | 34 段，通过 |

同一工具在 `body-depth/align` 上得到 10 个节点、10 条边、3 个组件，1 条边
经过子序列修复，路线通过全部几何复核。`body-depth/upper_floor` 仍为两个
组件且端点未接入，保持失败，说明修复没有跨越真实断层制造路线。大地图
`batch-room-20261031/occupied-10cm.ot` 本轮没有在比较工具时间窗口内产出完整
结果，不能计入通过样本。

这一步只完成离线 C++ 适配层验证，Rust graph 仍未接入生产 `PlannerArtifact`。
下一阶段需要在同一批固定地图上完成 warm query 时间和 route length 对照，再决定
是否把这层适配提取到正式的 PlannerArtifact 查询入口。

## 生产接入补充（2026-11-15）

上述结论对应本报告生成时点。当前 `PlannerArtifact` 已持有 Rust graph handle，
生产 `startPlan()` 已先执行 Rust endpoint query，并把候选路线交给 C++ geometry
adapter。Rust 候选失败时回退 C++ 图。Rust surface 查询、mapd region delta 和
prepared cache 仍未切入，历史离线结果不应被改写成完整 Rust 后端验收。

## 同图性能复测与队列修复（2026-11-15）

同一份 `arrival-box` occupied-cell 输入的实测见
[`mls-same-map-benchmark-20261115.md`](mls-same-map-benchmark-20261115.md)。DimOS
单线程冷构图中位数为 6.86 ms，LingTu Rust 修复后为 27.32 ms。LingTu 之前的
675 ms 来自 `OrderedF32` 与 `BinaryHeap<Reverse<...>>` 的双重反转，导致 Voronoi
Dijkstra 约 525 万次重复松弛。现在队列顺序已修正，7 个 Rust 单元测试通过，
arrival-box 的 surface、filtered route 和 C++ 几何复核结果保持不变。

此前的独立 Rust benchmark 以整张 artifact 构建计时，约为 DimOS 的 3.98 倍，且
LingTu 还没有与 DimOS 等价的 warm query 和增量更新 API。因此队列修复本身不是
“已经超过 DimOS”的证明。

随后又修正了对照工具的步高体素换算。DimOS 的 `step_cells()` 使用向下取整，
`0.45 / 0.10` 应为 4。此前 LingTu 对照使用向上取整 5，导致图多连了一层。
改为 floor 后，arrival-box 的 Rust native graph 为 `29 nodes / 45 edges / 2
components`，DimOS 为 `29 / 47 / 2`。过滤后的 Rust 图保留 35 条可执行边，10 条
边经过 corridor 子序列修复，10 条边被真实几何合同拒绝，最终路线仍通过
metric、support、body 和 continuous sweep 复核。

同一 C++ 对照工具的三轮计时中，Rust C ABI graph build 为 8.61–14.11 ms，C++
SurfaceGraph 为 11.39–22.28 ms。这个结果说明 Rust graph 的单独构建已经和当前
C++ 图处于同一数量级，但整套 surface 加 graph 准备仍慢于 DimOS 的单次冷构图。
