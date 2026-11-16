# MLS Rust 迁移路线

日期：2026-11-15

目标是让 Rust 接管多层表面和全局图算法，让 C++ 保留 OctoMap 输入、机器人几何
复核和现有导航运行入口。迁移完成后，生产规划器仍然使用同一套 body、support、
step、slope 和 continuous sweep 合同。

## 当前对比

## GeometryGate 的有效配置

Rust graph 只产生候选 surface route。正式 `navd` 会在每一段候选和起终点连接上
复用 C++ GeometryGate。参数的有效来源按优先级落在 RobotConfig、选定 Product 的
`native_nav.global` 和运行时环境覆盖，不由 Rust graph 自己决定。

| 检查 | 有效配置来源 | Go2 当前值 | Thunder V4 当前值 |
| --- | --- | ---: | ---: |
| 机器人水平半径 | RobotConfig `geometry.collision_cylinder_radius`，SCAN 后与 `octoplanner3d_robot_radius` 取较大值 | 0.25 m | 0.32 m |
| 机身下方净空 | RobotConfig `geometry.collision_clearance_below`，按 SCAN 体素分辨率向上取整 | 0.10 m | 0.05 m |
| 机身上方净空 | RobotConfig `geometry.collision_clearance_above`，按 SCAN 体素分辨率向上取整 | 0.24 m | 0.35 m |
| 支撑高度 | RobotConfig `geometry.support_height` | 0.35 m | 0.435 m |
| 支撑高度容差 | RobotConfig `geometry.support_height_tolerance` | 0.05 m | 0.10 m |
| 地面支撑 | `config/system/settings/global.yaml` 或 Product 覆盖 | true | true |
| 最大步高 | `native_nav.global.max_step_height_m` | 0.45 m 默认 | 0.45 m 默认 |
| 最大坡度 | Product 根据 RobotConfig 倾角限制解析为 `tan(tilt_limit_deg)` | 约 0.577 | 约 0.577 |
| 终点误差 | Product `native_nav.global.terminal_*` | 0.15 m | 0.15 m |
| no-go | mapd/map store 的 `NoGoRegion`，按请求注入 | 地图数据 | 地图数据 |

`PlannerConfig` 头文件中的 `robot_radius=0.20` 只是 C++ standalone 默认值。正式
Product 会在 `build.cpp` 的 SCAN 配置阶段用 RobotConfig 碰撞 profile 覆盖它，不能
把这个 standalone 默认值当作 Go2 或 Thunder 的现场半径。

GeometryGate 的具体调用顺序是 `routeConnectionFree()` → `queryWorld()`、
`isStepWithinLimits()`、`sweptRouteFree()`。起点和终点还会经过
`isStartConnectionAllowed()`、`resolveGoal()` 和 `resolvePlanEndpoints()`。

## 本轮已落地

- Rust 补面接收精确物理半径，非整数地图分辨率不会放宽洞宽或边缘距离。
- OctoMap 提供原始占据证据，Rust artifact 独占表面、支撑查询和图路线。
- prepared cache 升到 `.6`，只保存 Rust observed/inferred surface、node、edge、corridor 和 ABI 版本。
命中 `.6` 时直接安装 prepared surface 和 Rust graph，跳过重复构图和 C++ SurfaceGraph。
  graph 的重复构建。旧 `.4` cache 按格式失效并重建。
- Rust 图查询接收全局 `max_iterations`，预算耗尽会进入原有回退链。
- 全局诊断区分 Rust 查询失败、C++ 几何拒绝和回退原因。
- Rust 回退结果现在同时输出 `rust_query_failures`、
  `rust_geometry_rejections` 和 `rust_other_failures`，统计范围是一条路线请求。
- 区域图修复会保留仍存在的 surface cell 对应的稳定 node ID，并在失效窗口外留出
  一圈连接带，避免只重建窗口内部而丢失边界连接候选。
- 局部 Voronoi 修复会把修复窗口内仍有效的旧 node 作为固定 ownership seed，
  再重算窗口内的多源 Dijkstra、predecessor corridor 和受影响边。
- Rust 已增加首个 `check_route()` ABI，并在表面对照工具中与 C++ verifier 做同一路线 A/B 输出；它当前覆盖表面、occupied、step、slope 和单个 no-go，连续 swept-path 仍待迁移。
- 默认 surface extraction 已收敛到 DimOS 规则，Rust 表面使用列净空、逐 Z closing 和 nearby support。原始 occupied / known-free 仍由 OctoMap 几何层保留。

本轮验证为 Rust 18 个单元测试、MLS backend smoke、prepared surface smoke、prepared
cache smoke 和 mapd region contract test 全部通过。C++ SurfaceIndex 已从生产
PlannerArtifact、prepared cache 和 OctoPlanner 构建目标移除，旧 cache 会按格式失效并重建。

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
配置的吸附半径，C++ 负责把候选点转换为机身姿态并做几何复核。`.6` prepared cache
已经保存并恢复 Rust graph。当前仍缺少把几何拒绝重新交给 Rust 继续选路的完整
detour 查询。

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

生产链和 OctoPlanner 构建目标只使用 Rust 表面和图。原始占据和机身几何仍由
OctoMap 查询，避免把表面准备和执行碰撞重复维护。

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

生产支撑查询从 Rust artifact 读取，原始占据、body 净空和 continuous sweep 直接读取
OctoMap。Rust artifact 负责表面图、节点、corridor 和路线查询。

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

Rust handle 持有 graph、surface cell 和 revision。生产 `PlannerArtifact` 只持有 Rust
graph handle；旧 prepared artifact 按 `.6` 版本失效并重建。C++ `PlannerSession`
负责：

```text
map identity
artifact lifetime
geometry verifier
query cancellation
route result
```

地图没有变化时复用同一个 Rust graph handle。`.6` cache 命中时直接恢复 prepared surface
和这个 handle，不重新构图。
mapd 已提供 `RegionUpdate` 转换契约，包含 resolution、边界、occupied、known-free、
content epoch、reset epoch、sequence 和 generation。nav 还没有把这个契约接到持有
`PlannerState` 的生产适配层，因此当前地图变化仍会重新准备整张生产 artifact。

## 迁移顺序

1. 持有式 Rust artifact、稳定节点 ID、区域更新和 Dijkstra 已完成。
2. Rust graph 查询已接入生产 `PlannerArtifact`，并保留配置的端点吸附半径。
3. C++ geometry adapter 负责机身姿态转换、endpoint 连接和最终路线复核。
4. 在 arrival-box、body-depth/align、stairs、multifloor 和 nav_3d_course 上做同图
   surface、graph、route 和 geometry 对照。
5. 将 mapd `RegionUpdate` 接到持有 Rust `PlannerState` 的 nav 适配层。
6. 把几何拒绝后的 detour 查询接回 Rust graph。
7. 迁移完成。后续只需在固定地图上持续做 Rust 表面与执行几何回归。

## 当前不切换生产的原因

当前 Rust graph 已经在 arrival-box 的生产 `startPlan()` 路径通过 C++ 几何合同，
并且 `.6` cache 已能直接恢复 Rust graph。mapd 区域 delta 和几何拒绝后的 Rust detour
仍未接入，但它们不再要求 C++ SurfaceIndex 参与准备阶段。

迁移完成的最低证据是：

- Rust surface 和 C++ surface 逐格对齐；
- Rust graph 在固定参数下组件和端点接入稳定；
- Rust route 通过 C++ body、support、step、slope、continuous sweep；
- warm query 和 `update_region` 可复用；
- `used_surface_graph` 和 `used_rust_graph` 已出现在正式诊断；
- 五个正式三维场景回归通过。
