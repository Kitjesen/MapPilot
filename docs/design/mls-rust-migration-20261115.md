# MLS Rust 迁移路线

日期：2026-11-15

目标是让 Rust 接管多层表面和全局图算法，让 C++ 保留 OctoMap 输入、机器人几何
复核和现有导航运行入口。迁移完成后，生产规划器仍然使用同一套 body、support、
step、slope 和 continuous sweep 合同。

## 当前对比

同一份 arrival-box OctoMap 的现有结果：

| 阶段 | DimOS | LingTu Rust | LingTu C++ |
| --- | ---: | ---: | ---: |
| surface cells | 2,241 | 2,240 | 2,240 |
| sparse nodes | 29 | 29 | 36 |
| graph edges | 47 | 45 | 43 |
| graph components | 2 | 2 | 9 |
| graph build | 6.86 ms 完整冷构图 | 8.6–14.1 ms | 11.4–22.3 ms |
| geometry contract | 未提供 LingTu body 合同 | C++ 复核通过 | 原生通过 |

Rust graph 已经接入生产 `PlannerArtifact` 的候选查询。端点使用 Rust 图查询和
配置的吸附半径，C++ 负责把候选点转换为机身姿态并做几何复核。当前仍缺少 Rust
surface 查询替换、prepared cache 的 Rust 序列化，以及把几何拒绝重新交给 Rust
继续选路的完整 detour 查询。

## 目标边界

Rust 最终拥有：

- occupied/free 输入转换后的 surface extraction；
- observed/inferred 多层 surface artifact；
- 四邻域 adjacency；
- wall distance 和 sparse nodes；
- Voronoi ownership 和 predecessor corridor；
- graph edge cost；
- endpoint snapping 和 graph query；
- stable CellId；
- `update_region` 和缓存恢复；
- surface、graph、route 的离线诊断数据。

C++ 继续拥有：

- OctoMap 读取和地图身份；
- 机器人机身模型；
- ground support、body、step、slope 和 continuous sweep 查询；
- no-go 和临时占据接入；
- DDS、PlannerSession 和现有 nav endpoint；
- 最终可执行路线复核。

这不是保留两套表面图长期并行。C++ 的 SurfaceIndex/SurfaceGraph 只在迁移期间作为
对照和几何适配边界，Rust 通过固定接口后再删除 C++ 算法实现。

## 三个替换接口

持有式接口已经落地在 `lingtu_mls_core` C ABI 和
`global_planner::mls::PreparedArtifactHandle`。C++ smoke 已验证 create、summary、
surface export、graph export、graph query、region update 和析构。生产
`PlannerArtifact` 现在同时持有 Rust graph handle，`startPlan()` 先查询 Rust
候选路线，失败时才回退到 C++ 图并记录原因。

### SurfaceArtifact

Rust C ABI 现在返回可持有的 surface handle，并提供：

```text
observed cells
inferred cells
below(x, y, z)
inferred(x, y, z-range)
height(x, y, z, tolerance)
surface iteration
```

生产支撑查询已经使用 Rust handle 的 `below` 和 `inferred`，路线平滑和图路线高度
修正也已经使用 Rust `height`。原始占据、body 净空和 continuous sweep 仍直接读取
OctoMap。当前没有让两份表面结果在同一个判断中混用。

### GraphArtifact

Rust C ABI 返回可持有的 graph handle，提供：

```text
nodes
edges
corridor
component ids
endpoint candidates
graph query
```

每条候选 corridor 交给 C++ geometry adapter 复核。Rust graph 查询本身只负责一次
静态路线搜索，几何不通过时交给现有 fallback 或外层重新规划。动态执行期间采用
DimOS `truncate_to_safe` 的职责边界，不在同一次 global query 中循环修改图。

### PlannerArtifact

Rust handle 持有 graph、surface cell 和 revision。生产 `PlannerArtifact` 仍同时持有
C++ `SurfaceIndex`、C++ `SurfaceGraph` 回退和 Rust graph handle。C++ `PlannerSession`
负责：

```text
map identity
artifact lifetime
geometry verifier
query cancellation
route result
```

地图没有变化时复用同一个 Rust graph handle。Rust `update_region` 已可用并通过
smoke，但当前 mapd 仍以新 OctoMap 重新准备整张生产 artifact，尚未把区域 delta 接入。

## 迁移顺序

1. 持有式 Rust artifact、稳定节点 ID、区域更新和 Dijkstra 已完成。
2. Rust graph 查询已接入生产 `PlannerArtifact`，并保留配置的端点吸附半径。
3. C++ geometry adapter 负责机身姿态转换、endpoint 连接和最终路线复核。
4. 在 arrival-box、body-depth/align、stairs、multifloor 和 nav_3d_course 上做同图
   surface、graph、route 和 geometry 对照。
5. 将 Rust surface 查询和 prepared cache 接到同一生产 artifact。
6. 删除 C++ SurfaceGraph，再删除 C++ SurfaceIndex 的算法实现。

## 当前不切换生产的原因

当前 Rust graph 已经在 arrival-box 的生产 `startPlan()` 路径通过 C++ 几何合同。
C++ SurfaceGraph 仍作为回退和对照，Rust surface 查询与 prepared cache 也仍未替换。
这两个边界完成后，才能删除 C++ 图算法。

迁移完成的最低证据是：

- Rust surface 和 C++ surface 逐格对齐；
- Rust graph 在固定参数下组件和端点接入稳定；
- Rust route 通过 C++ body、support、step、slope、continuous sweep；
- warm query 和 `update_region` 可复用；
- `used_surface_graph` 和 `used_rust_graph` 已出现在正式诊断；
- 五个正式三维场景回归通过。
