# 三方 MLS 与全局规划能力对照

日期：2026-11-15

本页保留迁移过程中的能力对照。2026-11-16 的最新 main 四图实测、正式热查询、
缓存恢复、几何失败点与 mapd 接线状态见 [最新同图回归](dimos-latest-repro-20261116.md)。

这份矩阵把三套东西分开比较：

- **原 LingTu C++**：当前正式 `PlannerArtifact`、GlobalRoute、SCAN、Follower、Driver 链路。
- **新 LingTu Rust MLS**：当前已接入 C ABI、离线 graph、C++ 几何过滤和路线复核的候选后端。
- **DimOS MLS**：DimOS Rust MLS 源码和已有 matched runner。当前 `main` 已冻结为
  `83d807bb91bf6436a1a0e9c2a3260d96478636aa`；已有同图性能和路线数字主要来自固定的
  `26144f7` matched 构建，不能混写。

## 完整能力矩阵

| 能力 | 原 LingTu C++ | 新 LingTu Rust MLS | DimOS MLS | 当前证据和缺口 |
|---|---|---|---|---|
| OctoMap / 点云输入 | 原生 OctoMap，地图边界由 Maps/OctoMap 负责 | C++ 展开 occupied/free cells 后通过 C ABI 输入 | 主要消费占据体素/点云表面输入 | 三方输入格式和 free/unknown 语义尚未统一同图复跑 |
| 地图分辨率 | 生产配置支持 3D 体素分辨率 | 以整数 cell 运行，参数由 C++ 按分辨率换算 | `voxel_size` 参与表面和图构建 | 参数单位已部分对齐，需统一记录每个参数的米/格单位 |
| 观测表面 | `SurfaceIndex` 取列内可站立表面 | 当前 Rust observed surface 已和 C++ 对齐 | DimOS 自己的 `surfaces.rs` 规则 | arrival-box、body-depth 已对齐；DimOS 最新 main 同图未复跑 |
| 补面 / inferred surface | 边界平面、坡度、残差、known-free、headroom | 已复用对齐后的 C++ 语义 | L1 closing、邻域支撑、局部表面更新 | Rust 与 LingTu 已对齐，但不是 DimOS 原始补面语义 |
| 多高度表面 | 支持同 XY 多个 Z | 支持 | 支持 | 结构已对照，楼梯/平台的统一统计仍需补更多地图 |
| 四邻域表面连接 | C++ `SurfaceGraph` 使用带步高的邻接 | Rust 已使用四邻域和垂直步高 | DimOS 使用四邻域和步高 | 基本对齐；细节和参数仍需同图核对 |
| 节点墙距 | 有墙距和稀疏节点 | Rust 已加入 wall-distance、NMS 和组件兜底节点 | DimOS 有墙距、NMS 和节点放置 | 节点统计已做离线对照，墙边界判定还不是逐行等价 |
| 节点稳定 ID | C++ 图内部有索引，但没有 Rust regional lifecycle | Rust `PlannerState` 已保留未受影响 node ID | DimOS CellId 可在区域更新中保留 | 还缺同一地图的三方区域更新报告 |
| Voronoi ownership | C++ 已有多源图结构 | Rust 已有多源 Dijkstra ownership | DimOS 已有多源 Dijkstra | Rust 结构已接通，代价和 tie-break 尚未逐项对齐 |
| corridor | C++ 保存可查询的通道链 | Rust 已通过完整 graph ABI 输出 corridor | DimOS 保存完整 corridor chain | corridor 链已 smoke 验证；还缺过滤后 corridor 的组件统计 |
| 边代价 | 距离、楼层变化、墙距和本地配置 | 距离、墙距、step penalty 参数已接入 | 墙距、步高、表面图代价 | 参数数值和权重尚未三方统一 |
| 机身占据 | 生产路径在节点和边上做 3D 机身查询 | Rust 不持有机器人几何，由 C++ 过滤 | DimOS MLS 本身不等同于 LingTu 机身合同 | Rust 候选已接 C++ 逐 corridor 过滤 |
| 地面支撑 | C++ 支撑查询和支撑高度带 | C++ 过滤和最终复核负责 | DimOS 自己的 standable 表面规则 | arrival-box Rust 过滤后路线已通过 |
| 步高 / 坡度 | C++ 统一执行合同 | C++ 统一执行合同 | DimOS 使用自己的 step/slope 参数 | 三方参数和失败出口未完全同义 |
| 连续扫掠 | C++ `sweptRouteFree` | C++ 过滤每条 corridor | DimOS 不是同一套 LingTu swept verifier | Rust route 已共享 C++ 几何复核 |
| 起点吸附 | C++ 有 attachment、局部连接器和终点处理 | 新 Rust graph 已有节点接入和局部修复第一版 | DimOS 有 `select_entry` 和 planner 端点处理 | arrival-box 已修复通过；复杂楼梯/多层还需重复验证 |
| 终点吸附 | C++ 统一处理终点连接器 | Rust 使用离线 C++ 接入/复核 | DimOS 有自己的 goal entry | 三方连接器代价和失败原因未统一 |
| 路线搜索 | C++ SurfaceGraph 搜索 + XYZ fallback | Rust graph 已作为正式 PlannerArtifact 的首选候选查询 | DimOS planner 搜索表面图 | 仍保留 C++/XYZ fallback，尚未证明全链路替换 |
| 路线平滑 | C++ `smoothRoute`，每次保留完整检查 | Rust 当前 corridor 输出和离线 route，尚未复用正式 smooth contract | DimOS 有自己的 smoother | Rust 与 C++ 平滑后路线长度尚未对照 |
| yaw / 终端朝向 | 任务和执行层处理 | Rust graph 不负责 yaw，沿用 C++/任务层 | DimOS planner 有自己的 path/goal 表达 | 三方 yaw 语义和时间参数未统一 |
| 动态障碍 | SCAN 和最终控制负责实时障碍 | Rust 候选图不接实时传感器，C++ query/filter 负责静态合同 | DimOS local/global runtime 分层 | Rust 与 SCAN 的动态障碍端到端未测 |
| 禁行区 / no-go | LingTu C++ 有独立 no-go policy | Rust 图不拥有 no-go，C++ 负责 | DimOS 有自己的地图/规划约束 | 三方 no-go 输入和代价未统一 |
| 地图保存 | C++ prepared artifact 和 `.nav/profile-*` | Rust surface/node/edge/corridor 已写入 prepared artifact | DimOS 有 planner/map 资产和自身缓存 | 旧 cache 兼容与新格式已 smoke，缺跨版本完整报告 |
| prepared cache | C++ 保存/恢复 Rust artifact，C++ SurfaceGraph 退兼容 | Rust graph ABI、稳定 EdgeId、格式 3 已接入 | DimOS 有构图和运行时缓存 | 仍缺同一地图三方磁盘 roundtrip 对照 |
| 地图区域更新 | 当前 mapd 主要整图快照 | Rust 已有局部 surface、node relocation、edge/Voronoi/corridor repair；mapd 契约已定义 | `update_region` 局部表面/节点/边修复 | 生产 `mapd → PlannerState` adapter 仍缺 |
| 取消 / 超时 | C++ 有 cancel callback 和 planner budget | Rust 离线函数没有正式 Product cancellation contract | DimOS 有自己的 worker/模块取消 | 三方取消时图和缓存状态未做同图对照 |
| 失败诊断 | C++ 有 traversability failure、search info | Rust ABI 当前主要返回结构和错误码 | DimOS 有 planner/runtime 日志 | Rust 失败原因映射到 LingTu API 尚未完成 |
| 并发 / 线程 | C++ PlannerSession 和 Product 生命周期 | Rust crate 纯算法，正式 worker 尚未接入 | DimOS 使用 Tokio/Rayon/module worker | 三方线程数、锁、取消、内存峰值未统一测 |
| 内存 | C++ OctoMap、SurfaceIndex、Graph、cache | Rust graph buffers + C ABI，尚无正式内存预算 | DimOS 点云、surface、graph 和 worker | 未做同图 peak RSS/分配统计 |
| 冷准备时间 | 公开 matched 中 LingTu 冷准备约 21.9–98.2 s，视地图而变 | 当前 Rust 对照工具总墙钟不能单独代表 Rust 速度 | DimOS matched 构图约 4.02–19.84 s | 三方仍缺同一机器、同一输入、分段计时 |
| 热查询 | 已有 C++ prepared/session 记录 | Rust route/filter 目前是离线对照 | DimOS matched 广场热查询约 16.4 ms | Rust-only warm query 未测 |
| 正式导航接入 | 已接 nav / SCAN / Follower / Driver | Rust 已进入 GlobalRoute/PlannerArtifact 候选查询，SCAN/Follower/Driver 仍沿用原链路 | DimOS 接入自己的 runtime | Rust 到 MuJoCo/实机仍未验收 |
| MuJoCo | C++ 全局路线有场景门禁 | Rust MLS 还没有完整 MuJoCo 建图/定位/局部执行门禁 | DimOS 旧 runner 不是 LingTu MuJoCo 链路 | 三方动力学执行未统一 |
| 实机 | C++ 正式 field chain，可单独做 no-motion/实机验证 | 尚未切换实机 | DimOS 有自己的 Go2/设备组合 | Rust 实机证据为空 |
| 可视化 | LingTu Web/离线路线和点云工具 | Rust graph 可导出 corridor，但尚未接 Web artifact view | DimOS 有 surface/nodes/edges/path 可视化 | 三方可视化格式未统一 |
| 许可证和运行时 | LingTu/C++/OctoMap 边界 | Rust 核心保留 Apache-2.0 notice，不搬 DimOS runtime | DimOS Apache-2.0，但 runtime/消息体系不能直接搬 | 许可证边界已记录，未缺功能对照 |

## 当前还没有完成的三方实验

下面这些不是“文档没写”，而是确实没有完成同输入实测：

1. **最新 DimOS `main` 同一地图、同一起终点、同一参数的 Node/Edge/Corridor 输出。**
   现有成功率和速度主要来自旧 `26144f7` matched runner，不能标成最新 main。
2. **Rust MLS 与原 C++ 的分段性能。** 目前 Rust 对照工具的墙钟包含 OctoMap 读取、C++
   planner 构建和逐边几何过滤，还不是 Rust-only surface/graph/query 时间。
3. **过滤后 Rust graph 的组件数、端点吸附代价和路线长度。** arrival-box 已经有过滤后
   路线通过，但工具还没有把过滤后组件和路线长度全部输出到统一报告。
4. **三方 cache restore。** LingTu Rust graph 的 nodes/edges/corridor、稳定 EdgeId
   和格式 3 已有 C++ smoke，仍缺与最新 DimOS 在同一输入上的磁盘 roundtrip 对照。
5. **三方局部地图更新。** LingTu Rust core 已有局部修复和 mapd `RegionUpdate` 契约，
   但生产 `mapd → PlannerState` 调用以及与 DimOS 的同图耗时对照仍缺。
6. **三方动态障碍和 SCAN 执行。** MLS 只负责全局候选，SCAN、Follower、Driver 的
   动态避障和停车尚未放进同一份对照。
7. **三方内存、并发、取消和失败诊断。** 目前没有同机 peak RSS、线程数、超时取消、
   worker 恢复和错误码映射的统一证据。
8. **三方 MuJoCo 和实机。** 现有 C++ MuJoCo 门禁不能自动证明 Rust MLS 的完整运行链，
   更不能证明 Go2/Thunder 实机已经切换。

## 接入支持矩阵

Rust MLS 只有完成下面的支持项，才从“离线候选”变成“可选后端”：

| 接入阶段 | 必须完成 |
|---|---|
| `SurfaceProvider` | Rust surface 与 C++ surface 同图一致，known-free/headroom 语义固定 |
| `GraphProvider` | 完整 nodes/edges/corridor ABI、版本、释放、错误码、取消 |
| `GeometryGate` | C++ 节点/边/端点过滤，过滤后组件和路线可复核 |
| `PlannerArtifact` | Rust graph 和 C++ cache 统一保存/恢复，查询参数不重建静态图 |
| `GlobalRoute` | 起点/终点吸附、路线搜索、平滑、终端朝向和失败诊断接入 |
| `ProductControl` | Product/RunPlan/Host 只选择后端，不另起一套生命周期 |
| `SCAN` | 真实全局路线交给 SCAN，动态障碍和停止合同保持原链路 |
| `MuJoCo` | 建图、保存、加载、定位、全局、局部、到达停车全链路门禁 |
| `Field` | no-motion 连接、监督下实机路线、回滚到 C++ 后端 |

当前 Rust graph 已完成 `PlannerArtifact` 和 `GlobalRoute` 的首选候选查询，正式诊断
增加了 `used_rust_graph`。C++ GeometryGate 仍是权威准入，C++ SurfaceGraph 只保留
兼容、离线对照和 fallback 用途。`ProductControl` 不另起一套 planner 生命周期，
SCAN/Follower/Driver 仍接收原有全局路线。缓存和静态全局查询已经进入正式链路，
动态障碍、MuJoCo 和实机条目仍未完成 Rust 后端验收。

2026-11-15 已开始补增量生命周期第一阶段：Rust core 新增 `PlannerState`、
`RegionBounds`、`update_region()` 和 `RegionUpdateReport`。`PlannerState` 把
occupied/free 证据、surface、graph 和配置放在同一状态中。更新时使用区域加 surface
halo 只重提取受影响表面，合并区域外旧表面，并按相同 surface cell 复用 node ID；
测试已经证明区域更新结果与整图 surface rebuild 一致、区域外 node ID 保持不变。

当前 Rust `PlannerState::update_region()` 已包含 node relocation、edge invalidation
和 rebuild、局部 wall-distance/Voronoi/corridor 修复，并对区域外节点和边做保留。
`graph_rebuilt_globally` 只在局部修复无法保持图连通时作为明确 fallback，不再是默认路径。

## 增量图生命周期与 MLS runtime：DimOS 为什么更成熟

当前这部分的判断应明确写成：

```text
增量图生命周期和成熟 MLS runtime：DimOS 的正式 runtime 仍领先，LingTu Rust 的局部修复算法已补齐核心路径
```

这里比较的不是“哪个算法一定更好”，而是地图变化以后，表面、节点、边、缓存和
规划 worker 能不能继续复用已有状态。DimOS 的优势来自完整生命周期，而不是单个
`build_graph()` 函数。

### DimOS 已经具备的生命周期能力

DimOS MLS 的公开 Rust 结构包含以下连续状态：

```text
occupied / surface cells
  → stable CellId
  → columns and adjacency
  → nodes with wall-distance state
  → Voronoi ownership / Dijkstra predecessor
  → NodeEdge corridor chains
  → regional update repair
  → planner query
```

它的关键点是：

| 能力 | DimOS 的处理方式 | 当前 LingTu Rust MLS |
|---|---|---|
| Cell identity | CellId 在区域更新中保持，删除和复用有明确生命周期 | Rust `PlannerState` 已按 surface cell 复用稳定 node/edge ID |
| 区域更新 | `update_region` 只重提取受影响表面 | Rust 已有 surface halo 和区域外表面合并，mapd 生产调用仍缺 |
| 节点变化 | 死节点 relocation，必要时删除或移动到邻近有效 cell | 已有 node relocation 与删除回退 |
| 边变化 | 只修复经过更新窗口、删除节点或受影响 corridor 的边 | 已有 edge invalidation/rebuild，局部无法保持连通时才全局 fallback |
| Dijkstra 状态 | 维护 predecessor、ownership、repair window 和局部重算 | 已有局部 wall-distance、Voronoi ownership 和 predecessor corridor 修复 |
| corridor | 通过 predecessor chain 保存完整通道，并验证端点仍然匹配 | 已有完整 corridor 输出和局部重建 |
| 地图输入 | planner runtime 可以收到区域地图和边界更新 | mapd `RegionUpdate` 契约已定义，正式 nav adapter 仍缺 |
| worker 生命周期 | 模块 worker、规划请求、地图更新和取消有明确并发边界 | Rust crate 目前是同步纯算法调用，没有正式 runtime worker |
| 查询复用 | 已准备的图、节点和边被多个目标查询复用 | Rust graph 已接 `PlannerArtifact` cache 和 GlobalRoute |
| 可观测性 | surface、nodes、node edges、path 有独立输出 | 当前主要输出 C ABI artifact 和比较 JSON |

### 当前 LingTu Rust MLS 的实际边界

当前实现已经完成了 Rust 图、缓存和局部修复的第一版，正常静态查询路径是：

```text
一次完整输入
  → Rust surface extraction
  → Rust node/edge/corridor artifact
  → PlannerArtifact/cache
  → GlobalRoute query
  → C++ GeometryGate
```

它还没有完成：

- `mapd` 的生产区域更新调用和 cell 坐标转换；
- 最新 DimOS main 的同图 runner 和同口径 warm query；
- Rust graph worker 的取消、超时、查询排队和恢复；
- 更新期间旧图和新图的并发读写边界；
- region update 前后与整图重建的逐项等价验证。

因此不能把当前的 `nodes / edges / corridor` C ABI 叫作“已经具备 DimOS 的增量
MLS runtime”。它现在是一个可以被 C++ 几何边界消费的离线算法后端。

### LingTu 应该怎样补，而不是直接搬 DimOS runtime

不把 DimOS 的 Tokio worker、消息体系、LCM transport 或可视化 runtime 搬进产品。
LingTu 需要复用算法思想，并沿自己的所有权边界补生命周期：

```text
mapd
  → MapRegionSnapshot / MapRegionDelta
  → Rust SurfaceProvider
  → Rust NodeEdgeProvider
  → C++ GeometryGate
  → PlannerArtifact
  → GlobalRoute
```

建议按四个阶段实现：

1. **离线区域快照阶段**：先为 Rust core 增加带 bounds 和 source revision 的区域
   输入，在同一张固定地图上比较“整图重建”和“区域重建”的 surface、nodes、edges、
   components、corridor 和路线。当前 mapd 没有可靠 delta 输入前，不宣称在线增量。
2. **稳定图状态阶段**：为 Rust graph 增加稳定的 cell/node/edge identity、删除和
   relocation 规则，明确 corridor 经过更新区域时何时失效。此阶段仍不接正式导航。
3. **C++ artifact/cache 阶段**：把稳定 Rust graph 接入 LingTu `PlannerArtifact`，
   使用同一套 prepared cache、地图身份、查询参数和 C++ GeometryGate。旧图必须在
   新图准备完成前保持可读，更新失败要保留旧 artifact。
4. **运行时接入阶段**：等 mapd 有受支持的 region snapshot/delta 后，再把更新接入
   ProductControl/Host 的生命周期；查询取消、更新取消、旧图切换和 SCAN 路线交接
   需要单独做门禁。

### 增量生命周期的验收门槛

Rust MLS 只有在下面证据齐全后，才可以说“达到 DimOS 生命周期的可用子集”：

| 验收项 | 必须证明 |
|---|---|
| 局部表面更新 | 更新区域外的 surface 集合和 ID 不变 |
| 节点更新 | 受影响节点可移动/删除，区域外节点 ID 不变 |
| 边更新 | 只有受影响 corridor 被重建，区域外边保持一致 |
| 整图等价 | 区域更新结果与同输入整图重建在受影响邻域内一致 |
| 失败恢复 | 更新失败时旧 `PlannerArtifact` 仍能查询和停止任务 |
| 查询并发 | 查询期间更新不会读取半成品图，切换点可观测 |
| 缓存恢复 | 保存、重启、恢复后的 graph 与内存构建结果一致 |
| 几何合同 | 更新后的所有 Rust corridor 继续通过 C++ body/support/sweep |
| 性能 | 区域更新耗时随受影响区域增长，而不是随全图线性增长 |
| 运行接线 | ProductControl、Host、GlobalRoute、SCAN 的所有权不重复实现 |

在这些门槛通过前，产品说明应保持准确：DimOS 在增量图生命周期和 MLS runtime
成熟度上领先；LingTu 当前的优势是 C++ 三维执行合同、产品生命周期边界和已有
SCAN/Follower/Driver 链路，而 Rust MLS 仍处于离线候选后端阶段。
