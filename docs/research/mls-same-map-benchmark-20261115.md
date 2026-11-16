# LingTu 与 DimOS MLS 同图实测

日期：2026-11-15

本轮使用同一份 OctoMap：

```text
build/arrival-box-20261027/occlusion-map-final/same_source_map/octomap.ot
```

由同一个 `octoplanner3d_dump_octomap` 导出 7,329 个 occupied voxel center。
DimOS runner 和 LingTu 的独立 Rust benchmark 读取这份 XYZ 文件。更严格的 Rust C ABI
节点、边、组件和计时对照则直接使用同一个 C++ 工具从 OctoMap 提取 occupied 与
known-free，避免把 free-space 输入差异混进结果。输入体素间距为 0.10 m。两边均使用
单线程冷构图，避免线程数成为主要变量。

固定参数如下：

| 参数 | 值 |
| --- | ---: |
| voxel size | 0.10 m |
| robot height | 0.35 m |
| surface closing | 0.20 m |
| node spacing | 1.0 m |
| wall clearance | 0 m |
| wall buffer | 0.20 m |
| wall penalty | 20 |
| step threshold | 0.45 m |
| step penalty | 4 |

## 结果

DimOS 和 LingTu 各运行 5 次。表中的构图时间使用 5 次结果的中位数，warm query
和局部更新使用 DimOS runner 的 5 次结果中位数。

| 指标 | DimOS MLS | LingTu Rust MLS |
| --- | ---: | ---: |
| occupied points | 7,329 | 7,329 |
| observed + inferred surface | 2,241 | 2,240 |
| sparse nodes | 29 | 29 |
| graph edges | 47 | 45 |
| graph components | 2 | 2 |
| cold build median | 6.86 ms | 18.97 ms surface + 13.82 ms graph |
| graph build range | not separated in runner | 8.61–14.11 ms |
| warm query median | 0.665 ms | 未提供完整 planner query API |
| warm query P95 | 0.910 ms | 未提供完整 planner query API |
| local update median | 0.432 ms | 未提供增量更新 API |

DimOS runner 的单次结果示例：

```json
{"points":7329,"surface":2241,"nodes":29,"edges":47,"components":2,
 "route_points":21,"build_ms":6.612,"warm_query_median_ms":0.665,
 "warm_query_p95_ms":0.943,"update_ms":0.432,"update_changed":true}
```

LingTu C++ 对照工具中的 Rust C ABI 单次结果示例：

```json
{"observed_surface_count":2240,"inferred_surface_count":0,
 "nodes":29,"edges":45,"components":2,
 "rust_surface_ms":18.971,"rust_graph_ms":14.115}
```

LingTu 另做过 graph-only Dijkstra 测试，缓存图查询中位数约 `0.0009 ms`。它不包含
起终点吸附、路径字符串拉直、平滑和执行几何复核，不能与 DimOS 的 `Planner::plan`
warm query 直接排名，只能说明缓存 graph 的 Dijkstra 本身不是当前瓶颈。

## 性能结论

当前 LingTu Rust C ABI 的表面和 graph 准备合计约 32–33 ms，约为 DimOS 冷构图的
4.7 倍。Rust graph 单独构建约 8.6–14.1 ms，与当前 C++ SurfaceGraph 的同图准备
约 11.4–22.3 ms 处于同一数量级。之前的 675 ms 是 `OrderedF32` 排序方向错误
造成的 Dijkstra 反复松弛，修正后已经消除。

本轮还把 `0.45 / 0.10` 的最大步高体素数从向上取整改为和 DimOS `step_cells()`
一致的向下取整。修正后 Rust graph 从 `30 / 50 / 1` 变为 `29 / 45 / 2`，与
DimOS 的节点数和组件数对齐，边只差 2 条。

本轮修复位于 `src/kernels/nav/mls_core/src/corridor.rs`。Rust core 的队列现在与
`BinaryHeap<Reverse<...>>` 的最小代价语义一致，并增加了 `ordered_cost_is_a_min_heap_key`
回归测试。修复后 7 个 Rust 单元测试全部通过，arrival-box 的 Rust corridor 仍能
经过同一 C++ body、support、metric 和 sweep 复核。

## 结构和执行结论

- 表面数量相差 1 格，尚未完全一致。
- 节点和边数量接近，但组件数仍不同。
- DimOS 已有稳定的 warm query 和 `update_region` 增量更新。
- LingTu Rust 当前只有全图 `build_artifact`，还没有正式的 warm query 和增量图更新入口。
- LingTu 的 C++ 适配层已经能把 Rust corridor 过滤为可执行路线，但这层还没有接入生产
  `PlannerArtifact`。

因此当前不能宣称 LingTu 超过 DimOS。可以确认的是，队列修复后 LingTu 已从“实现有
明显性能错误”进入“结构接近、冷构图仍慢、增量生命周期尚未补齐”的状态。

## 生产接入补充（2026-11-15）

本报告前文记录的是离线对照时点。随后 `PlannerArtifact` 已持有 Rust graph
handle，生产 `startPlan()` 会先执行 Rust limited endpoint query，再经过 C++
机身、支撑、步高、坡度和 continuous sweep 复核。Rust 候选被拒绝时仍回退 C++
图，因此本报告的历史性能表不能解读为 Rust 已完全替换 C++，也不能解读为已超过
DimOS。Rust surface 查询和 prepared cache 仍未迁移。
