# 地图域合同

状态：当前产品合同。`src/maps` 是 LingTu 唯一的持久地图域。
读者：地图、定位、规划、Gateway 和部署维护者。
替代文档：无。

## 1. 职责

`src/maps` 负责：

- 地图创建、保存、导入、裁剪、删除、改名、退役、激活和回滚；
- `MapRecord`、artifact、版本、健康度、POI、地图图关系和 active slot；
- 点云、2D occupancy、OctoMap、ESDF、traversability 和 semantic artifact；
- SaveMap 的幂等任务、崩溃恢复、构建队列、校验和事务发布；
- 向定位、规划、Gateway 和诊断提供 capability/bundle 查询。

`src/maps` 不负责：

- LiDAR 驱动、SLAM、全局重定位和 `map -> odom` 估计；
- mission FSM、全局路径搜索、局部避障、PathFollower 或 `cmd_vel`；
- HTTP、WebSocket、前端渲染和设备进程生命周期。

## 2. 实现边界

字段运行时使用两条边界，不能混用：

```text
实时状态与场景
  mapd -> typed DDS -> HostBus / native consumers

本机低频 artifact 查询与大文件打开
  Gateway -> plain MapClient -> /run/lingtu-mapd/mapd.sock -> mapd
  mapd -> bounded JSON metadata + verified read-only fd
```

UDS 仅是 Sunrise 同机的私有请求/文件句柄通道，不取代 DDS，也不对局域网开放。当前生产 opcode 只有 `PING` 和 `OPEN_ARTIFACT`；active map 等运行状态仍以 `/maps/state` 为准。PCD 等大文件不分块塞入 DDS；Gateway 不解析地图目录或 artifact 路径，只把 `mapd` 返回的文件描述符流式转成 HTTP 响应。

当前迁移期间，尚未迁走的地图管理命令仍走以下旧适配链：

```text
Gateway / CLI / Planner / Localization
  -> typed MapsModule contract
  -> thin Python runtime adapter
  -> stable C ABI
  -> C++ MapsServiceCore
       -> MapStore
       -> MapPipelineCore
       -> SaveMapEngine
       -> ArtifactJobWorker
       -> map layers
```

每迁完一组调用者，就删除该组 Python 字符串路由和 C ABI 生命周期；不得让 UDS 与旧链同时拥有同一写命令。`MapClient` 是普通无状态对象，不是 Module，不加入 Blueprint，也不管理进程生命周期。

以下内容只能由 C++ 实现：

- 地图 id 和生命周期状态；
- active map、active slots、rollback history；
- artifact 扫描、hash、bundle、health；
- PCD 修改、地图构建、版本发布和失败回滚；
- voxel、occupancy、ESDF、traversability、semantic occupancy 热路径。

Python 只允许承担现有 Blueprint/Module 端口适配、命令解包、SLAM snapshot 请求和 C ABI 响应转发。缺少 `lingtu_maps` 时必须启动失败，不允许回落到 Python 文件实现。

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
ListActiveSlots
GetBuildStatus / GetArtifactJob
```

控制：

```text
CreateMap
SaveMap
ImportPcd
CropPcd
RestoreSourceBackup
BuildArtifact
EditOctomapVoxels
SetActiveMap / ClearActiveMap / RollbackActiveMap
SetActiveSlot / ClearActiveSlot
RenameMap / RetireMap / DeleteMap
SetPOI / DeletePOI
SetMapEdge / DeleteMapEdge
CancelArtifactJob / RetryArtifactJob
```

Gateway、Planner 和脚本不得在服务不可用时扫描目录、读取 active symlink 或自行修改地图文件。

## 4. 标准地图产品

```text
<map-root>/<map-id>/
  current_version.txt
  lifecycle_state.txt             # RETIRED 等生命周期状态
  .versions/<version>/
    map.pcd
    poses.txt                      # 可选
    patches/                       # 可选
    occupancy.npz
    map.pgm
    map.yaml
    octomap.ot                     # 或显式兼容 octomap.bt
    esdf.npz                       # 可选
    traversability.npz             # 可选
    semantic_map.bin               # 可选，不单独赋予 navigation-ready
    metadata.json
    voxel_edits.jsonl              # 可选
    save_manifest.json
    save_manifest.sha256
    artifact_checksums.sha256
```

根目录中的兼容视图由 C++ SaveMap 发布器生成，不是真相源。`map_record.json` 和 `active` symlink 不属于当前主合同。`tomogram.pickle` 仅属于退役的 PCT 实验，不参与产品 readiness。

## 5. 状态与激活

核心状态：

```text
CREATED -> STALE -> READY -> ACTIVE
                     |        |
                     +-> RETIRED
任何构建失败 -> FAILED
```

- `active_map.txt` 是主 active map 的原生状态文件；
- `active_slots.lts` 管理 `navigation`、`mapping`、`reference` 等命名 slot；
- rename 必须同步替换所有 slot；delete/retire 必须清除相关 slot；
- `RETIRED` 地图不能再次激活，也不能进入任意 active slot；
- 仅有 `map.pcd` 的地图是 `STALE`，不能报告为 navigation-ready；
- 产品 SaveMap 版本必须通过 manifest 和所有 artifact SHA256 校验后才能激活。

## 6. 事务不变量

1. 所有修改先写 staging，成功后再原子发布。
2. occupancy、OctoMap、ESDF、traversability、metadata 和 record 必须作为同一导航包提交。
3. import、crop、save-source、source restore 和 voxel edit 失败时不得暴露半成品。
4. 修改 source PCD 后必须让旧派生 artifact 失效，并在必要时自动取消 active。
5. C ABI 命令探测不得执行副作用；响应缓冲区不足时命令只执行一次，并缓存结果供重试读取。
6. 同一 map 的写操作必须持有 native map lock。

## 7. 文件交换安全

- PCD 导入只接受 `LINGTU_MAP_IMPORT_DIR` 下现存的 `.pcd`；
- package 导入只接受该 import root 下的目录；
- version/package 导出只允许写入 `LINGTU_MAP_EXPORT_DIR`；
- 默认目录为 `<map-root>/.exchange/import` 和 `<map-root>/.exchange/export`；
- 禁止绝对路径逃逸、`..`、symlink 越界和 Gateway 任意主机文件读取。
- 下载时由 `mapd` 在 map lock 下解析声明身份，使用 `O_NOFOLLOW` 打开普通文件并核对 SHA256；Gateway 只接收只读 fd 和对应元数据。

## 8. 数据流

```text
SLAM accepted scan + pose
  -> maps.voxel / maps.occupancy / maps.elevation
  -> maps.esdf / maps.semantic
  -> /maps/scene + typed layer outputs
  -> Gateway

SLAM registered cloud + odometry
  -> standalone native traversability
  -> /nav/traversability
  -> local planner / final safety gate
```

短 TTL 的实时障碍层可以复用 maps C++ layer 算法，但不能冒充持久地图产品。

## 9. 验收门槛

```powershell
cmake -S src\maps -B build\maps -DLINGTU_MAPS_BUILD_TESTS=ON
cmake --build build\maps --config Release
ctest --test-dir build\maps -C Release --output-on-failure
$env:LINGTU_MAPS_LIB=(Resolve-Path build\maps\Release\lingtu_maps.dll).Path
python -m pytest src\maps\tests -q
```

还必须通过 Gateway/runtime 合同测试和旧路径静态扫描。Windows CI 使用进程唯一的 pytest basetemp，不能依赖 pytest 的编号临时目录清理。

## 10. 尚需设备验证

代码合同已经闭合，但以下结论必须在设备上取证，不能用桌面测试代替：

- Ubuntu aarch64 编译和 OctoMap 原生链接；
- MID-360 长时 replay、动态残影和 scene throughput；
- S100P 断电/重启后的 SaveMap 恢复；
- 走廊退化、无语义标签和定位漂移修正；
- 真机 Gateway、Planner、Localization 对同一 committed version 的一致消费。
