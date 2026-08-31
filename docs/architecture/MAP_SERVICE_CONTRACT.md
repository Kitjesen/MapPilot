# 地图域合同

状态：当前产品合同。`src/maps` 是 LingTu 唯一的持久地图域。
读者：地图、定位、规划、Gateway 和部署维护者。
替代文档：无。

## 1. 职责

`src/maps` 负责：

- 地图创建、保存、导入、裁剪、删除、改名、退役和激活；
- `MapRecord`、artifact、健康度、POI、地图图关系和 active map；
- 点云、2D occupancy、OctoMap、ESDF、traversability 和 semantic artifact；
- SaveMap 的幂等任务、失败恢复和目录替换；
- 向定位、规划、Gateway 和诊断提供 capability/bundle 查询。

`src/maps` 不负责：

- LiDAR 驱动、SLAM、全局重定位和 `map -> odom` 估计；
- 回环候选检索、图因子测量或 PGO 算法；SaveMap 只负责把冻结的 source 工作副本交给 native 优化器并处理其执行或跳过结果；
- mission FSM、全局路径搜索、局部避障、PathFollower 或 `cmd_vel`；
- HTTP、WebSocket、前端渲染和设备进程生命周期。

## 2. 实现边界

字段运行时使用两条边界，不能混用。这里采用成熟机器人地图系统常见的结构：常驻
native 地图 owner、typed 命令/事件、薄客户端。该所有权模式可见于
[Nav2 Map Server](https://github.com/ros-navigation/navigation2/blob/main/nav2_map_server/README.md)、
[Autoware Map Loader](https://autowarefoundation.github.io/autoware_core/latest/map/autoware_map_loader/)
和 [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)。LingTu 只借鉴边界，
不引入 ROS2 节点、service 或 message 依赖。

```text
实时状态与场景
  mapd -> typed DDS -> HostBus / native consumers

本机低频地图管理、artifact 查询与大文件打开
  Gateway stateless UDS transport
    -> /run/lingtu-mapd/mapd.sock -> mapd
  mapd -> bounded JSON metadata + read-only fd
```

UDS 仅是 Sunrise 同机的私有请求/文件句柄通道，不取代 DDS，也不对局域网开放。生产 opcode 是 `PING`、`SERVICE` 和 `OPEN_ARTIFACT`；active map 等运行状态仍以 `/maps/state` 为准。PCD 等大文件不分块塞入 DDS；Gateway 不解析地图目录或 artifact 路径，只把 `mapd` 返回的文件描述符流式转成 HTTP 响应。

地图管理命令统一走：

```text
SDK client -> Gateway HTTP adapter
  -> stateless same-host UDS request
  -> private mapd service endpoint
  -> the single C++ MapsServiceCore inside mapd
       -> MapStore
       -> MapPipelineCore
       -> SaveMapEngine
       -> map layers
```

Gateway 的 Python `MapClient` 是普通无状态 UDS transport，不是 Module，不加入
Blueprint，不协调 SaveMap，也不管理进程生命周期。仓库已删除 `MapsModule`、
`MapdServiceClient`、Python save pipeline 和 Maps Python C ABI；Host 不得创建第二个
`MapsServiceCore`。

以下内容只能由 C++ 实现：

- 地图 id 和生命周期状态；
- 当前内容的数值 `content_epoch`；
- active map；
- artifact 扫描、bundle 和 health；
- PCD 修改、地图构建、保存和失败恢复；
- voxel、occupancy、ESDF、traversability、semantic occupancy 热路径。

Python 只允许在 Gateway API 边界校验 HTTP 参数、发送同机 UDS 请求并转发 mapd
响应或只读 artifact fd。SLAM snapshot 请求、SaveMap 协调、地图业务状态和文件处理
全部由 native runtime 负责。mapd endpoint 不可用时操作必须明确失败，不允许回落到
Python 文件实现或 Python live map layers。

## 3. 对外服务

查询：

```text
ListMaps
GetMapTypes
GetRecord
GetActiveMap
GetMapHealth
GetMapBundle(capability)
GetMapPoints
GetVoxelEdits
ListPOI
ListMapGraph
ListSaveMapJobs / GetSaveMapStatus
```

控制：

```text
CreateMap
SaveMap
ImportPcd
CropPcd
BuildArtifact
EditOctomapVoxels
RenameMap / RetireMap / DeleteMap
SetPOI / DeletePOI
SetMapEdge / DeleteMapEdge
CancelSaveMap / RetrySaveMap
AuditMaps / QuarantineCorruptMaps
ImportMapPackage / ExportMapPackage
```

`SetActiveMap` / `ClearActiveMap` 不属于 UDS `SERVICE` action。ProductControl
通过独立的 typed activation transaction 驱动 `mapd`，普通 `MapClient` 不能改变
active map。

地图服务没有激活历史回滚、地图版本列表、内容版本回滚或版本发布 API。

Gateway、Planner 和脚本不得在服务不可用时扫描目录、读取私有状态文件或自行修改地图文件。

## 4. 标准地图目录

每个 map-id 只有一个可变的 canonical 目录：

```text
<map-root>/<map-id>/
  map.pcd
  poses.txt                      # 可选
  patches/                       # 可选
  patch_bundle.manifest          # 可选；有 patch bundle 时记录完整性证据
  map_optimization.json          # SaveMap 生成的优化/跳过结果
  occupancy.npz                 # 按产品需要生成
  map.pgm                       # 按产品需要生成
  map.yaml                      # 按产品需要生成
  octomap.ot                    # 当前导航需要
  esdf.npz                      # 可选
  traversability.npz            # 可选
  semantic_map.bin              # 可选，不单独赋予 activation_ready
  metadata.json
  voxel_edits.jsonl             # 可选
  lifecycle_state.txt           # RETIRED 等生命周期状态
```

不存在 `.versions/`、`current_version.txt`、不可变版本发布、内容版本列表、内容版本回滚或 `save_manifest.json`。调用者读取 `<map-root>/<map-id>/` 中的当前内容，不自行维护兼容视图。`map_record.json` 和 `active` symlink 不属于当前主合同。

`MapStore` 为当前 canonical 内容生成正整数 `content_epoch`。唯一身份表达是两个独立
字段：稳定逻辑名 `map_id` 和当前内容代次 `content_epoch`。DDS 和导航合同使用
`map_content_epoch`；Python/Gateway 只运输这个数值。`map:v123` 与 `map:e123` 都是
非法 map-id，也不是可解析的组合身份；其中 `map:e...` 从未成为本仓库协议。

## 5. SaveMap 主路径

SaveMap 只有一条产品路径：

```text
SDK client -> Gateway -> mapd: save_map(map_id, request_id)
  -> native SaveCoordinator 创建 SaveMap job
  -> /slam/map_command: SlamMapSnapshotRequest
  -> slamd 冻结同一 Product session 的 source/patch bundle
  -> /slam/map_event: SlamMapSnapshotAck
  -> mapd 把已验证 snapshot 提交给 SaveMapEngine
  -> 获取该 map-id 的 MapLock
  -> 在 staging 目录构建候选地图
  -> 在 OPTIMIZE_SOURCE 阶段处理 PGO：
     显式约束文件存在 -> lt_pgo --constraints
     否则 patch bundle 完整 -> lt_pgo --auto-constraints
     否则以 patch_bundle_incomplete 保留 raw source
  -> 做最小可用性校验
  -> 暂存现有 canonical 目录为一次性 backup
  -> 用候选目录替换 <map-root>/<map-id>/
  -> 替换失败时恢复 backup
  -> 成功后删除 backup
```

`save_map` 是唯一公开保存动作。`BeginSaveMap` 和 `ProvideSaveMapSnapshot` 只是
mapd 内部 C++ 步骤，不是 Gateway、Host 或 Product API。两个既有 topic path 保持不变，
但 payload 已是 typed `SlamMapSnapshotRequest` / `SlamMapSnapshotAck`，不再运输
Text/JSON 命令。

替换 canonical 目录前只要求以下校验：

- `map.pcd` 能打开且非空；
- `metadata.json` 能读取，且坐标帧为 `map`；
- 当前导航使用的 `octomap.ot` 能打开且非空。

这三项用于阻止明显不可用的导航地图覆盖现有目录，不形成版本清单、全目录 hash 或第二套保存协议。SaveMap 失败不得暴露 staging 半成品；若替换失败，恢复本次操作临时保留的旧目录。backup 不是历史版本，成功保存后不保留。

`OPTIMIZE_SOURCE` 是同一 SaveMap 事务内的数据驱动阶段，不是功能开关或另一条产品
路径。Fast-LIO2 在线前端只冻结 `map.pcd`、`poses.txt`、body-local patches 与
manifest；它不生成图因子文件。自动模式在 SaveMap 工作副本上对每对相邻单 patch
做正反向 trimmed 4DoF point-to-plane 配准，得到完整 `N-1` measured chain；信息矩阵
来自实际 `J^T J / sigma^2`，并转换到 body-right tangent。随后合并通过几何验证和
共识门限的 loop factors。

完整 patch bundle 会自动进入 native PGO 分析；只有相邻链完整且至少存在一个可信
loop 时才生成优化后的自包含 source bundle。
否则 SaveMap 保留 raw source，并把 `insufficient_keyframes`、
`sequential_chain_incomplete`、`no_verified_loops`、`patch_bundle_incomplete` 或
`pgo_timeout` 等准确原因写入 `map_optimization.json`。优化器执行失败或声称执行却
输出不完整时，SaveMap 必须在替换 canonical 目录前失败。

`pose_graph.constraints` 只是在 `lt_pgo` 内部原子写入、严格回读并立即删除的私有
临时输入，不是 Fast-LIO2/Product API 输出，也不发布到最终地图目录。旧 ROS2
PGO/HBA runtime 仍已删除；`lt_pgo` 是 native release 内的短时工具，不是常驻地图
服务。当前没有现场回放、误回环率、轨迹质量或 S100P 性能证据，不能把代码接线写成
已通过现场验收。

## 6. 状态与激活

核心状态：

```text
CREATED -> STALE -> READY -> ACTIVE
                     |        |
                     +-> RETIRED
```

- `active_map.txt` 只是 mapd 私有存储中的 active map-id 状态文件，不属于 Gateway、Product 或 Python API；
- mapd 内部修改 `active_map.txt` 时必须持有跨进程 `__active_map__` 锁；恢复先前 active map 时还必须在锁内校验预期值，过期操作不得覆盖新选择；
- ProductControl 是唯一公开地图激活 owner；Gateway 拒绝 `set_active_map` 和 `clear_active_map`；
- ProductControl 只回退当前切换事务，不维护地图激活历史，也不提供“回到上一个地图”的服务命令；
- pipeline 的持续 `.build_lock` 与 `MapLock` 是互斥的同一写入域：构建锁只能在短暂持有 MapLock 时创建，MapLock 在构建结束前必须拒绝获取；
- rename 必须同步更新 active map；delete/retire 必须清除相关 active map；
- 被 `map_graph.ltg` 引用的地图不得 rename/delete/retire；调用方必须先显式删除相关 edge，避免留下无法读取的悬空地图图；
- `SetMapEdge(from,to,...)` 是幂等替换语义：重复调用必须更新 edge 类型和方向；
- `DeleteMapEdge` 必须物理删除 edge，并清理不再被引用的服务生成 portal node；
- `RETIRED` 地图不能再次激活；
- 仅有 `map.pcd` 的地图是 `STALE`，不能报告 `activation_ready=true`；
- `activation_ready` 只表示当前 canonical 目录中的静态规划 artifact 可用；
- `runtime_health_state` 独立表示 `unknown`、`ready` 或 `blocked`，未知样本不会制造首次激活死锁；运动前仍必须由 Product readiness 验证实时定位和控制链。

## 7. 写入不变量

1. SaveMap 先在 staging 构建，通过最小校验后才替换 canonical 目录。
2. 同一 map-id 的写操作必须持有 native `MapLock`。
3. import、crop、save-source 和 voxel edit 失败时不得暴露半成品。
4. 修改 source PCD 后必须让旧派生 artifact 失效，并在必要时取消 active。
5. `map_graph.ltg` 的查询和 read-modify-write 必须持有同一个 graph lock，避免并发边更新相互覆盖。
6. 地图生命周期写操作只能经过 mapd 内唯一的 `MapsServiceCore`；不得恢复 Maps C ABI 或 Python 文件写路径。

## 8. 文件交换安全

- PCD 导入只接受 `LINGTU_MAP_IMPORT_DIR` 下现存的 `.pcd`；
- package 导入只接受该 import root 下的目录；
- package 导出只允许写入 `LINGTU_MAP_EXPORT_DIR`；
- 默认目录为 `<map-root>/.exchange/import` 和 `<map-root>/.exchange/export`；
- 禁止绝对路径逃逸、`..`、symlink 越界和 Gateway 任意主机文件读取；
- 下载时由 `mapd` 在 map lock 下解析 artifact 并打开普通文件；Gateway 只接收只读 fd 和对应元数据。

## 9. 数据流

```text
SLAM accepted scan + pose
  -> native mapd voxel / occupancy / elevation / ESDF engine
  -> /maps/scene + typed layer outputs
  -> Gateway

SLAM registered cloud + odometry
  -> standalone native traversability
  -> /nav/traversability
  -> local planner / final safety gate
```

短 TTL 的实时障碍层可以复用 maps C++ layer 算法，但不能冒充持久地图产品。

## 10. 验收门槛

```powershell
cmake -S src\maps -B build\maps -DLINGTU_MAPS_BUILD_TESTS=ON
cmake --build build\maps --config Release
ctest --test-dir build\maps -C Release --output-on-failure
python -m pytest src\maps\tests -q
```

还必须通过 Gateway/runtime 合同测试和旧路径静态扫描。Windows CI 使用进程唯一的 pytest basetemp，不能依赖 pytest 的编号临时目录清理。

## 11. 尚需设备验证

以下结论必须在设备上取证，不能用桌面测试代替：

- Ubuntu aarch64 编译和 OctoMap 原生链接；
- MID-360 长时 replay、动态残影和 scene throughput；
- S100P 保存过程中断电后的旧目录恢复与下次启动行为；
- 走廊退化、无语义标签和定位漂移修正；
- 真机 Gateway、Planner、Localization 对同一 active `map_id` 及其已解析 artifacts 的一致消费。
