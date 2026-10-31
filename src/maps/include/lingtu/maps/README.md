# Maps C++ 头文件目录

这里是 `lingtu_maps` 的公开 C++ 接口目录，例如
`#include <lingtu/maps/layers/voxel.hpp>`。头文件说明数据和接口，主要实现位于
[`src/maps/cpp/`](../../../cpp/)；小型、模板或内联算法直接定义在头文件中。
理解 Maps 先区分两件事：**实时地图**处理当前扫描，供局部碰撞检测和建图查看；
**保存地图**从 SLAM 冻结的关键帧、位姿和射线构建，供重载定位和全局规划。
完整的运行、保存和激活约定见 [Maps 总览](../../../README.md)。
## 从哪里开始
| 想了解的问题 | 入口 |
| --- | --- |
| 点云怎样进入实时地图？ | [`cpp/mapd/dds.cpp`](../../../cpp/mapd/dds.cpp) → [`mapd/engine.hpp`](mapd/engine.hpp) → [`cpp/mapd/engine.cpp`](../../../cpp/mapd/engine.cpp) |
| 三维碰撞、点云和地面由谁维护？ | [`layers/`](layers/)；点云数据类型见 [`cloud.hpp`](cloud.hpp) |
| 地图怎样保存？ | [`mapd/save.hpp`](mapd/save.hpp) → [`save.hpp`](save.hpp) |
| 保存制品怎样生成？ | [`io/`](io/) → [`build/pipeline.hpp`](build/pipeline.hpp) → [`octomap/`](octomap/) |
| 保存地图在哪里管理？ | [`storage/store.hpp`](storage/store.hpp) |
| 本机服务怎样响应请求？ | [`mapd/protocol.hpp`](mapd/protocol.hpp) → [`cpp/mapd/`](../../../cpp/mapd/) → [`service.hpp`](service.hpp) |

## 两条地图链路
```text
LiDAR + IMU（Maps 之外）
  -> SLAM：去畸变、估计位姿
  -> /slam/map_observation：本帧点云 + 同一时刻的位姿、射线原点
  -> mapd DDS 解码 -> LiveMapEngine -> 实时地图层 -> /maps/*

保存请求
  -> mapd/save.hpp：向 SLAM 请求冻结快照
  -> save.hpp：接收关键帧、位姿和射线，组织优化、清理与构建
  -> build/pipeline.hpp -> octomap/rays.hpp -> storage/store.hpp
  -> 保存的 map.pcd、octomap.ot 等制品
```

mapd 不直接订阅原始 LiDAR 包，也不重新估计机器人位姿。保存地图使用 SLAM 冻结的
`map.pcd`、`poses.txt`、`scan_origin.txt` 和 `patches/`，不把实时 voxel 或 accumulated
快照直接当作整张保存地图。

实时入口的 `Observation` 定义在 [`mapd/engine.hpp`](mapd/engine.hpp)：

| 字段 | 含义 |
| --- | --- |
| `scan`、`sensor_frame` | 本帧去畸变点云及其坐标系；当前 SLAM 发布的是 `body` 坐标系点云。 |
| `map_sensor` | 从点云坐标系到 `map` 的变换；当前实际是 `map ← body`，不能按字段名当作物理雷达位姿。 |
| `sensor_origin_*` | 同一扫描时刻的物理雷达原点，已经位于 `map` 坐标系，射线更新从这里出发。 |
| `stamp_ns`、`reset_epoch`、`sequence` | 扫描时间、SLAM 重置代次与观测序号，用于保持点云和位姿对应、识别新观测。 |
| `reference_map_id`、`reference_map_content_epoch` | 定位模式所引用的保存地图身份；建图观测不继承旧地图身份。 |
| `pose_state`、`pose_quality` | 上游声明的姿态状态和质量；引擎只接收 `MAPPING`、`TRACKING`。 |

引擎把本帧变换到 `map` 后先更新滚动 3D 占据，再按配置更新额外的点云和累计层。
引擎只保留一个最新待处理观测；`replaced_observations` 统计这个位置的替换，
不包含此前 DDS 接收批次中合并掉的观测，不能作为全链路丢帧总数。

当前 SLAM 的 `MapObservation` 仍携带局部估计器状态，保存地图匹配的降级另行反映在
SLAM 状态和导航输入门控中。因此，mapd 的 `TRACKING`、`live` 不能单独证明全局定位
健康或机器人可以运动。对应发布代码见
[`toDdsMapObservation`](../../../../localization/slam/cpp/cyclone_runtime.cpp)。

保存源只有 PCD、没有完整扫描射线时，`octomap/sampled.hpp` 生成的是预览地图：
点云可用于查看和定位，推断的自由包络不等于实测自由空间，制品标记为
`sampled_points`、`navigation_ready: false`。只要 OctoMap 制品本身是有效的完整
`octomap.ot`，它可以被激活用于定位和规划；`saved_rays`、`full_log_odds` 等来源字段
保留为诊断信息，不再作为运行时运动门槛。是否能生成路径由 OctoPlanner3D 判断，
局部运动仍由 SCAN 的实时三维碰撞检查决定。保存成功本身不代表定位和导航验收通过。

## 根目录：公共入口

| 头文件 | 用途 |
| --- | --- |
| `model.hpp` | 保存地图、制品、健康状态和 POI 的公共值类型，以及能力到制品类型的映射。 |
| `cloud.hpp` | 点云数组视图、拥有型点云，以及带扫描时间和射线原点的 `MapCloudFrame`；带位姿的入口 `Observation` 定义在 `mapd/engine.hpp`。 |
| `service.hpp` | `MapsServiceCore` 门面，汇集查询、生命周期、构建、保存、维护、POI 与图操作。 |
| `save.hpp` | 异步保存流程：接收快照、处理/构建、验证并事务提交。 |
| `graph.hpp` | 跨地图 POI/门户图、边管理、持久化和最短路径。 |

## `layers/`：实时地图

核心是滚动 3D 占据、体素点云和累计占据三种数据，各自承担不同用途。
`map` Product 维护完整查看层；`nav` 等控制 Product 由 RunPlan 关闭扩展层，保留
实时局部碰撞所需的滚动 3D 占据。保存制品构建不受这个开关影响。

| 头文件 | 用途 |
| --- | --- |
| `layers/rolling_occupancy.hpp` | 有界滚动 3D 占据、射线清除、膨胀碰撞快照和已观测/未知证据；输出 `/maps/local_collision`。 |
| `layers/voxel.hpp` | 保留观测表面的体素点云、局部窗口快照和容量/更新统计。 |
| `layers/block_grid.hpp` | 稀疏分块的累计 3D 占据网格，支持射线更新、ROI 快照和二进制持久化。 |
| `layers/ground_surface.hpp` | 在同一 XY 保留不同高度证据的局部地面聚类、平面拟合与坡度/粗糙度估计。 |
| `layers/surface_projection.hpp` | 把观测地面与局部 3D 占据投影成展示用支撑/障碍图。 |
| `layers/grid.hpp` | `Grid2D` 及网格创建、几何比较等基础操作。 |
| `layers/elevation.hpp` | 高程投影及其结果类型。 |
| `layers/esdf.hpp` | 二维距离场和梯度计算，用于地图投影。 |
| `layers/cost.hpp` | 离线通行性制品使用的代价融合；不判定速度命令。 |
| `layers/terrain.hpp` | 从高程计算坡度、步高和粗糙度风险。 |
| `layers/exploration.hpp` | 把探索代价网格与观测掩码编码为 free/occupied/unknown 占据值。 |
| `layers/exploration_projection.hpp` | 从滚动 3D 占据按机器人高度带与膨胀半径生成探索网格快照。 |

同一 XY 上的新命中不能证明其他高度为空。实时体素层按射线已观测自由空间清除，
累计层按射线命中/未命中更新，并保留配置的衰减行为。

局部碰撞消息分别保留膨胀碰撞、实测占据和已知自由位图。SCAN 的碰撞查询通过只
表示没有命中膨胀障碍，不等于已观测自由或有地面支撑。二维投影用于查看、候选或
派生制品，最终运动仍由原生 3D 导航和安全链判断。

## `mapd/`：实时进程与本机服务

协议及共享运行时接口留在公开头文件中；UDS 服务、查询和分派实现的头文件位于
`src/maps/cpp/mapd/`，只对 Maps 内部和对应测试开放。

| 头文件 | 用途 |
| --- | --- |
| `mapd/engine.hpp` | `Observation`、`State`、`Snapshot` 与 `LiveMapEngine`；接收最新观测，更新地图并按 generation 缓存实时/完整快照。 |
| `mapd/activation.hpp` | 在固定地图身份上执行 stage/commit/rollback 激活事务并编码回滚令牌。 |
| `mapd/save.hpp` | 协调 mapd 与 SLAM 的 DDS 快照请求/确认，再把快照交给保存引擎。 |
| `mapd/protocol.hpp` | UDS 帧 magic、版本、操作码、状态、大小限制和默认 socket 路径。 |
| `cpp/mapd/server.hpp`（内部） | UDS 监听线程、客户端握手、请求处理和停止生命周期。 |
| `cpp/mapd/query.hpp`（内部） | ping、服务 JSON 请求和制品文件描述符打开的核心处理。 |
| `cpp/mapd/dispatch.hpp`（内部） | 将规范 action JSON 分派到 `MapsServiceCore` 或保存协调器。 |

`SnapshotDetail::kRealtime` 提供当前点云、体素点云与局部碰撞；`kComplete` 在扩展层
开启时增加累计点云、占据投影、地面、高程和 ESDF。扩展层关闭时，点云等扩展输出
为空，局部碰撞继续更新。快照的 ROI/点数上限限制导出内容，不代表删除了底层地图。
配置的发布频率是调度上限，实机频率需测量 DDS 输出。

## `io/`：输入与文件格式

| 头文件 | 用途 |
| --- | --- |
| `io/json.hpp` | 完整 JSON 文档解析，以及字符串转义和序列化辅助。 |
| `io/pcd.hpp` | XYZ PCD 读取、二进制写出、边界裁剪和体素过滤。 |
| `io/scans.hpp` | 遍历保存的关键帧点与其真实传感器射线原点。 |
| `io/patch_bundle.hpp` | 校验 patch 清单、扫描集合和保存射线是否完整。 |

## `build/`：保存制品构建

| 头文件 | 用途 |
| --- | --- |
| `build/pipeline.hpp` | `MapPipelineCore` 编排源提交、制品构建和事务；保存导航包默认构建 3D OctoMap，2D 占据、ESDF 与可通行性按请求生成。 |
| `build/process.hpp` | 运行有超时、取消和输出上限的外部构建命令。 |
| `build/occupancy_snapshot.hpp` | 从保存点云拟合局部地面并生成 2D 占据快照、PGM 和 YAML。 |
| `build/grids.hpp` | 读取原生占据制品，并构建 ESDF 与可通行性网格制品。 |

## `octomap/`：保存地图的全局 3D 树

| 头文件 | 用途 |
| --- | --- |
| `octomap/sampled.hpp` | 无完整射线时，从采样点构造预览占据体素、支撑膨胀和有限自由包络。 |
| `octomap/rays.hpp` | 从完整保存扫描重放命中/未命中证据，保留实测端点并抑制掠地射线误清除。 |
| `octomap/io.hpp` | 读取 `.ot`/`.bt`，并按全 log-odds 或显式二值编码写出 OctoMap。 |

射线构建以清理后的 `map.pcd` 判断哪些端点保留为 hit；被清掉的端点仍贡献端点前
实际经过的 free 射线，不生成 hit，也不推断端点之后的空间。同一帧中 hit 优先于 miss。
保留原来的近端擦地保护；对保存扫描已经命中的保留体素，另用该高度附近的实测点拟合
局部表面。只有射线在体素内的整段都位于表面容差带外同一侧，才跳过 miss；真实穿面和
拟合不确定时继续更新。不会把同一 XY 的不同楼层平均，也不会将仅有 PCD 的点补成占据。
构建器版本为 `0.3.2`，旧缓存按现有规则重建。

关键帧清单完整只表示选中帧均保存，不等于每帧观测完整覆盖。融合 PCD 可能比保存射线
覆盖更广，查看器能显示表面不代表规划器已得到支撑和自由空间证据。
[数据集诊断记录](../../../../../build/dataset-planning-fix-20261021/README.md) 包含实测断口和
修正结果；该直接 replay-save 实验未经过 Product SaveService 的动态清理。

## `storage/`：保存地图状态

| 头文件 | 用途 |
| --- | --- |
| `storage/store.hpp` | `MapStore` 管理地图目录、内容 epoch、活动地图、生命周期和制品/导航证据验证。 |
| `storage/lock.hpp` | 单地图持久锁及失效持有者检查，保护变更、构建和恢复。 |
| `storage/health.hpp` | 汇总定位、规划、碰撞、时效和制品验证样本的地图健康模型。 |
| `storage/maintenance.hpp` | 地图审计、损坏隔离、包导入/导出及其报告。 |

## `semantic/`：语义地图

| 头文件 | 用途 |
| --- | --- |
| `semantic/taxonomy.hpp` | 从 JSON 载入语义类别，规范化标签并解析别名。 |
| `semantic/occupancy.hpp` | 带占据概率、统计量和主标签的语义体素层，支持更新、替换和范围查询。 |
| `semantic/store.hpp` | `semantic_map.bin` 的原子写入、读取和格式验证。 |
| `semantic/unity.hpp` | 将 Unity 场景类别与对象几何导入语义体素地图。 |
