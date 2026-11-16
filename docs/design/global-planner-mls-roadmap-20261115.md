# LingTu 三维全局规划路线

## 目标

LingTu 的全局规划要先把三维地图转换成稳定的多层可行空间，再在这个空间里搜索路线。楼梯、平台、坡面和过顶结构都属于同一个三维路线问题。局部动态障碍由 SCAN 处理，跟踪器只执行局部轨迹，底层驱动只执行期望速度。

最终主链为：

```text
mapd / OctoMap snapshot
    ↓
SurfaceProvider
    ↓
NodeProvider
    ↓
EdgeProvider / corridor
    ↓
PlannerArtifact
    ↓
GlobalRoute
    ↓
SCAN local trajectory
    ↓
Follower desired velocity
    ↓
Driver platform execution
```

## 对比结论

| 维度 | PCT Planner | DimOS MLS | LingTu |
| --- | --- | --- | --- |
| 多层空间表达 | tomography，多层结构先于搜索 | 多高度 surface cells | OctoMap + SurfaceIndex |
| 稀疏图 | tomography 上的全局搜索结构 | Nodes、Voronoi corridor edges | SurfaceGraph、稀疏节点和通道 |
| 在线地图更新 | 公开流程偏场景级重建 | 支持 update_region 和区域修复 | 当前以整张不可变图资产替换 |
| 局部动态障碍 | 与全局规划分开 | 与 MLS 全局规划分开 | SCAN 负责 |
| 机身执行约束 | 需要另配执行验证 | planner 内部约束依赖配置 | C++ 原生机身、支撑、步高、坡度和扫掠检查 |
| 工程依赖 | ROS、CUDA、CuPy、Open3D、GTSAM | DimOS Rust runtime、消息 crate、Tokio、Rayon 等 | 原生 C++、CycloneDDS、OctoMap、SCAN |
| 许可证 | GPLv2 | Apache-2.0 | 现有产品许可和上游许可边界 |

结论按目标区分。PCT 最适合作为多层空间表达和论文方法参考。DimOS 最适合作为表面、节点、通道和增量生命周期的实现参考。LingTu 继续负责真实机器人可执行性和现场集成。

PCT Planner 的公开仓库标明 GPLv2，不能直接复制到产品。DimOS MLS 文件和仓库使用 Apache-2.0，可以在保留版权和许可证声明的条件下做适配。DimOS 的完整 crate 依赖自己的 runtime 和消息体系，不能把整个 crate 直接塞进 navd。

## 当前实现基线

LingTu 已有：

- `SurfaceIndex`，从 OctoMap 里提取同一 XY 下的多个高度表面，并保存观测表面与推断支撑。
- `SurfaceGraph`，构建稀疏节点、邻接和 corridor chain。
- `PlannerArtifact`，把表面和图作为一个地图派生资产共同保存、恢复和失效。
- `PlannerSession`，按 `MapIdentity` 和 preparation 参数复用准备结果。
- OctoPlanner3D 的三维机身占据、地面支撑、步高、坡度、连续扫掠和终点检查。
- SCAN、Follower、Driver 的现有局部执行链路。

当前仍是整图重建。mapd 已有地图 generation、sequence、occupied/free 证据和完整快照，但还没有向全局规划器提供区域体素增量或区域快照接口。没有这个输入时，区域增量更新无法保持正确。

## 最终模块边界

### SurfaceProvider

输入是地图快照和机器人准备参数。输出是带地图 revision 的多层 `SurfaceArtifact`。观测表面、推断支撑和高度层必须能区分。原始 OctoMap 不被改写。

### NodeProvider

输入是 `SurfaceArtifact`。输出是稀疏节点、节点位置和节点 revision。节点放置负责空间稀疏化、局部墙距偏好和节点迁移，不负责最终机身碰撞授权。

### EdgeProvider

输入是表面和节点。输出是节点边、corridor chain、边代价和边 revision。边构建可以使用多源 Dijkstra、Voronoi boundary 和 corridor chain。每条准备边仍要经过 LingTu 的三维几何检查。

### PlannerArtifact

`PlannerArtifact` 是表面、节点、边和序列化 metadata 的唯一地图派生资产。它记录 `map_identity`、`map_generation`、`surface_revision`、`graph_revision`、`backend` 和 preparation 参数。

### GlobalRoute

全局规划只输出起点到终点的三维路线和诊断信息。动态障碍不重建全局资产。局部实时占据交给 SCAN。

## Rust 与 C++ 边界

Rust 承担纯算法数据结构和图计算：

```text
Voxel columns
  → multi-level surface extraction
  → bounded per-Z closing
  → sparse node placement
  → corridor edge construction
  → serializable planner artifact
```

C++ 继续承担：

- OctoMap 读取、地图身份、generation 和 snapshot 生命周期。
- 机器人机身模型和标定。
- body occupancy、ground support、step、slope 和 swept route 检查。
- no-go、临时请求 overlay、取消和错误传播。
- GlobalPlanRequest/Result、DDS、PlannerSession 和 SCAN 接口。

Rust 的最小适配接口只需要输出 `SurfaceCell`、`Node`、`Edge` 和 `PlannerArtifact`。不把 DimOS module runtime、generated LCM messages、Tokio worker loop 或 DimOS visualization message 带入 navd。

## 地图生命周期

### 第一阶段，整图资产

```text
地图保存或加载
  → 读取 OctoMap snapshot
  → Rust SurfaceProvider
  → Rust NodeProvider
  → Rust EdgeProvider
  → C++ 三维几何复核
  → 写入 PlannerArtifact
```

地图内容、机器人机身、支撑高度、步高、坡度和表面闭合参数变化时重建。目标点、目标容差、吸附半径、搜索预算和同层偏好只更新查询。

### 第二阶段，区域增量

```text
mapd region snapshot / cell delta
  → 局部表面重建
  → 局部节点迁移
  → 局部 corridor 边修复
  → 与旧资产合并
  → 发布新的不可变 PlannerArtifact
```

区域更新必须经过完整重建对照。相同输入下，区域结果和整图结果的表面集合、节点连通性、边 corridor、路线和几何复核需要一致。

## 实施阶段

### Phase 0，冻结基线

固定 903 地图、公开 OctoPlanner3D 地图、楼梯、平台、过顶结构、窄门和动态障碍场景。记录表面数量、节点数量、边数量、组件数量、路线长度、冷准备、缓存恢复、热查询、机身复核和 SCAN 接入结果。

### Phase 1，资产边界

保留当前 C++ SurfaceIndex 和 SurfaceGraph，实现 `PlannerArtifact` 的统一序列化和查询复用。目标是让静态图只由 preparation 参数触发重建。当前这一阶段已经完成并通过相关 smoke tests。

### Phase 2，Rust MLS A/B 后端

导入 Apache-2.0 的 DimOS MLS 算法片段，保留版权和许可证。先做离线 Rust backend，与 C++ 当前 backend 并行输出 SurfaceArtifact 和 PlannerArtifact。两套结果在相同地图、参数和目标下进行对照。

当前进度：Rust surface extraction 已按 C++ `SurfaceIndex` 的 bounded-hole、平面、坡度、已知 free 和头部净空语义对齐。arrival-box 和 body-depth/align 的 observed、inferred、节点、边、组件、图路线和共享 C++ 几何复核均已一致；upper-floor 和 batch-room 也已在无可接入端点时给出相同的无路线结果。Rust 仍只通过离线 C ABI 输出 surface，尚未成为生产 `PlannerArtifact` 的 provider。

### Phase 3，C++ 几何接管

Rust 只构建表面、节点和边。C++ 对节点、边和最终路线执行统一的机身、支撑、步高、坡度和连续扫掠检查。任何 Rust 路线必须经过同一条 LingTu 执行合同。

### Phase 4，mapd 增量输入

增加 mapd 区域快照或体素增量接口。先做区域 SurfaceProvider，再做节点迁移和 corridor 边修复。增量结果需要和整图重建逐项对照。

### Phase 5，生产切换

只有当 Rust backend 在固定场景中达到路线连通、几何复核、缓存恢复和性能门槛后，才允许成为默认 backend。PCT 保留为离线研究对照，不接入产品运行时。

## 验收指标

验收分四层，不把离线规划结果写成实机导航成功。

| 层级 | 必须通过的内容 |
| --- | --- |
| Surface/Graph | 多高度表面、楼梯和平台连接、图组件、起终点吸附、重复构建和缓存恢复 |
| Route geometry | 每个路点机身占据、连续扫掠、地面支撑、步高、坡度、终点 XYZ 误差 |
| Runtime performance | 冷准备、缓存恢复、热查询分别记录，不能混成单次规划时间 |
| Execution | SCAN 接入、动态障碍、Follower 速度、Driver 命令、到达、停车和碰撞 |

现有 matched replay 使用较早的 DimOS `26144f` 构建。LingTu 在建筑跨区、建筑楼梯和广场三例中返回了路线，DimOS matched 在建筑两例中出现入口组件断开，在广场返回的路线未通过 LingTu 支撑复核。LingTu 广场热查询中位数为 74.3 ms，DimOS matched 为 16.4 ms。最新 DimOS `main` 为 `5e0ebfd422a6`，尚未重新跑 matched benchmark，因此最新性能仍待验证。

这些结果属于离线全局规划和路线几何证据，不能代表 SCAN 动力学执行、MuJoCo 完整导航或 Go2 实机通过。

## 决策

采用 PCT 的多层空间问题定义，采用 DimOS MLS 的表面、节点、边和增量生命周期，保留 LingTu 的原生三维几何和 SCAN 执行链路。Rust 只进入纯算法核心，C++ 保留地图身份、机器人几何、DDS、任务取消和执行边界。

下一步是把 Rust node/edge 结果接到离线 `PlannerArtifact` 对照，并复用现有缓存和 C++ 几何合同；完成同条件缓存、路线和性能证据后，再评估是否进入生产准备。没有完成这些证据前，不切换现场默认规划器。

## 参考资料

- DimOS MLS source and license  https://github.com/dimensionalOS/dimos
- DimOS `surfaces.rs`  https://github.com/dimensionalOS/dimos/blob/5e0ebfd422a6002d09f6f956b75a064805f61f86/dimos/navigation/global_planner/mls_planner/rust/src/surfaces.rs
- PCT Planner  https://github.com/byangw/PCT_planner
- PCT Planner paper  https://ieeexplore.ieee.org/document/10531813
- OctoPlanner3D  https://github.com/JackJu-HIT/OctoPlanner3D
- LingTu matched comparison  `docs/research/planner-public-dataset-comparison-20261114.md`
- LingTu surface graph evidence  `docs/research/surface-graph-20261031.md`
