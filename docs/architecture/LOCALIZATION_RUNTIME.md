# 定位运行时、重定位与 DDS 合同

状态：current

本文只描述当前 Product 会实际启动和接线的能力。定位源码树已经移除 ROS2
message/service 包；未进入 RunPlan 的算法资产会明确标成“未接入”，不能据此
声称现场已经融合或闭环。

## 1. 先给结论

- 现场权威位姿由一个 `lt-slam.service` / `slamd` 实例产生；Host 通过
  `cpp_slam_status` 适配器消费它。这是“一位权威 owner”，不是“只有一种算法”。
- 当前现场里程计是 Fast-LIO2，实际测量输入是 LiDAR + IMU。
- 保存地图不是第三种传感器。保存地图在导航时提供 `map` 坐标系的几何约束，
  启动重定位和后续 continuous map-based localization 据此更新 `map -> odom`。
- GNSS 设备配置、驱动代码和 DDS topic 都存在，但当前 real RunPlan 不启动
  GNSS 角色，`slamd` 也没有 GNSS DDS reader，所以 GNSS 尚未参与现场融合。
- `/slam/odom_prior` 是可选外部里程计先验，当前主要供仿真/实验；现场配置未启用。
- DVO 和 Scan Context 都不是当前 Product 的现场定位链；旧 PointLIO 以及 ROS2
  PGO/HBA 源码已删除。ROS-free `lt_pgo` 是保存地图阶段的短时工具，不是在线定位 backend。
- 当前 Product 没有已接收的在线回环闭环。`ScanPlanner` 是导航局部避障规划器，
  不是 Scan Context，也不是 SLAM 回环检测器。

不要再用“1 路、2 类、3 类”混合描述 owner、算法和输入。应分别写成：

| 维度 | 当前事实 |
| --- | --- |
| 权威 owner | 1 个：native `slamd` |
| 实时估计算法 | Fast-LIO2 |
| 现场测量输入 | LiDAR、IMU |
| 保存地图定位能力 | seeded/global 启动重定位；随后自动持续 scan-to-map 校正 |
| 当前未融合输入 | GNSS、`odom_prior`（现场）、DVO |
| 未接入当前 Product 的算法资产 | Scan Context 回环 |

这里的三个名字必须分开使用：

| 名字 | 含义 |
| --- | --- |
| `fastlio2` | C++ SLAM 算法 backend |
| `mapping` / `localization` | `slamd` 运行 mode，由 Product YAML 的 `slam_mode` 决定 |
| `cpp_slam_status` | Python Host 消费 native 状态/点云快照的 adapter，不执行 SLAM 算法 |

旧 Python 路径曾把 `localizer` 同时当作 profile 和 backend 参数，再在 binding
内部还原成 `fastlio2 + localization`。这是历史建模债务，不代表存在一个名为
`localizer` 的第二套 Product backend。当前 Product 只启动外部 `slamd`；Python
不再 feed/tick C++ backend，也不再构建 `_native.SlamRunner`。保存地图定位应写成：

```text
backend = fastlio2
mode = localization
map = ProductControl 绑定的精确 MapIdentity
```

## 2. ProductControl、Gateway、slamctl 和 DDS 各自做什么

```text
scripts/lingtu switch nav ...
  -> python -m lingtu.control
  -> ProductControl 解析 Product + env 为唯一 RunPlan
  -> real: systemd 启动 lt-lidar / lt-slam / lt-maps / lt-terrain / lt-nav / ...
  -> slamd 启动时加载 ProductControl 已绑定的地图目录
  -> 首次定位成功后持续执行 scan-to-map；readiness 只判断能否接收导航目标

Gateway HTTP
  -> localization facade
  -> NativeSlamRelocalizationService
  -> slamctl（DDS 客户端）
  -> rt/slam/relocalization/request
  -> 已经运行的 slamd
  -> rt/slam/relocalization/response
```

| 组件 | 负责 | 不负责 |
| --- | --- | --- |
| ProductControl | Product 生命周期、锁、session、精确地图身份、systemd/subprocess、顺序、readiness、失败回滚 | 算法内部计算 |
| Gateway | HTTP schema、权限/参数校验、当前 Product 地图守卫、状态观察、把请求转给定位 facade | 启动 systemd unit、切 Product、任意选择 SLAM 地图 |
| `slamctl` | 创建临时 DDS participant，发送请求并等待匹配响应 | 启动 `slamd`、持有 Product 策略 |
| DDS | 已运行进程之间的 typed data plane 与 request/reply transport | 进程或 systemd 生命周期 |
| `slamd` | Fast-LIO2、地图加载、重定位、定位输出与健康 | Product 选择和跨进程启动 |

因此“DDS 能否唤起服务”的答案是不能。DDS 请求只有在 `slamd` 已由
ProductControl 启动并进入相同 domain 后才会得到响应。现场优先这样检查：

```bash
systemctl --no-pager --full status 'lt-*.service'
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/health"
PYTHONPATH=src python -m diagnostics.field.dds_readiness --seconds 5 --topics \
  /slam/odometry /slam/state_at_scan \
  /slam/localization_quality /slam/localization_health
```

Gateway 只报告当前 Product 和定位状态，不暴露 Product 生命周期切换入口。

## 3. nav 重定位启动

```bash
bash scripts/lingtu --env real switch nav --map MAP_NAME --initial-pose X Y YAW
bash scripts/lingtu --env real switch nav --map MAP_NAME --relocalize
```

两种启动方式的语义都不是“给当前进程发一个热 DDS 命令”：

- 两者都会执行完整的 `nav` ProductControl 冷切换；
- seeded 形式必须显式给出 `X Y YAW`；
- global 形式不需要初值，启动后执行全局粗搜索与局部精配准；
- ProductControl 按所选 `env` 解析并执行对应 RunPlan；
- 如果只想在已经健康运行的同一 Product、同一地图上重试，可使用 Gateway
  的 in-place API；它仍不会启动 `slamd`。

Gateway in-place API：

| API | 产品语义 |
| --- | --- |
| `POST /api/v1/localization/relocalizations`，`mode=seeded` | 使用调用方提供的 `initial_pose` 在 active map 上重定位 |
| `POST /api/v1/localization/relocalizations`，`mode=global` | 不提供初值，在 active map 上执行全局重定位 |
| `POST /api/v1/localization/map-tracking` | 请求当前 `slamd` 重新确认/恢复 active map tracking；正常启动定位成功后 tracking 已自动持续运行 |
| `GET /api/v1/slam/status` | 查看 SLAM 运行状态，不改变生命周期 |

公开合同使用定位领域语义，不把当前内部 matcher 名称暴露为 API。此次切换不保留
`/api/v1/slam/relocalize`、`/api/v1/slam/auto_relocalize` 或
`/api/v1/slam/track_against_map` 的长期兼容路由；客户端必须直接迁移到上述新接口。

### 为什么 `map_name` 必须与当前 Product 地图一致

ProductControl 从 mapd 取得的是当前地图身份：
`MapIdentity(map_id, frame_id, artifacts)`。同一次 RunPlan 中的 SLAM target、
mapd live map 和规划器必须绑定同一个 `map_id` 及其已解析 artifact，Gateway 只接受
与这个 active map 一致的请求。

Gateway 请求中的 `map_name` 是一致性守卫，不是把另一个 PCD 路径交给
`slamd` 的第二选择器。若当前 Product 使用 `warehouse`，请求
`map_name=factory` 会返回 409，并要求先通过 `scripts/lingtu` 或
`python -m lingtu.control` 做 ProductControl 切换。Product 管理
运行时也会拒绝 DDS 请求携带 `map_path` 或 `load_map`，避免出现：SLAM 在 A
地图定位、mapd/规划器却在 B 地图规划的分裂状态。

## 4. 建图、重定位和持续校正流程

### 4.1 建图

```text
rt/lidar/raw_frame + rt/imu/raw
  -> slamd / Fast-LIO2
  -> odom -> body 实时状态
  -> rt/slam/odometry + rt/slam/state_at_scan
  -> 注册点云、增量 MapObservation、实时 map cloud
  -> mapd 维护现场地图层
```

`state_at_scan` 是“被接受的那帧 LiDAR 扫描时间戳上的状态”，与普通高频
`/slam/odometry` 的时间语义不同。现场 mapd 的同步输入已经由
`/slam/map_observation` 原子携带同一扫描、位姿和原点；独立的
`/slam/state_at_scan` 现在仅作为外部诊断/兼容输出，不应虚构一个产品内消费者。

### 4.2 保存地图导航的启动重定位

```text
Fast-LIO frontend 接收 fresh scan/IMU 并进入 TRACKING
  -> 有初值：执行一次 seeded relocalization verify/refine
     无初值：执行一次 global relocalization coarse search -> refine
  -> native gate 检查质量、重叠率、协方差和边界
  -> 成功才提交 map -> odom
  -> 同一个 slamd 立即开始/继续周期 scan-to-map 修正
  -> navigation can_accept_goal 根据地图、持续跟踪、定位和位姿新鲜度派生判断
```

这是 saved-map localization Product 的自动启动事务，不是所有 SLAM 模式都执行：

1. 本次请求带显式初值时做一次 seeded localization；
2. `--no-relocalize` 只允许复用同一地图上仍 fresh 的有效定位；
3. 其余冷启动自动做一次 global localization；
4. 首次定位失败时不提交 `map -> odom`，导航不能接收目标；
5. 首次定位成功后，同一 `slamd` 立即进入持续地图校正，不等待导航 readiness。

real/systemd Product 先等待 Fast-LIO 前端有 fresh scan/odometry，再发起首次定位；
sim/subprocess 把显式 seed 直接绑定进本次 RunPlan，无 seed 时同样做 global 初始化。
两者成功后都由同一 runtime 持续对齐保存地图。Gateway 旧的 last-pose 后台线程及其
无人消费的缓存文件已经删除，避免第三个启动 owner。

### 4.3 持续基于地图定位

正式术语为 **continuous map-based localization**；具体算法动作是
**scan-to-map registration**。首次 seeded/global relocalization 提交 `map -> odom`
后，同一 `slamd` 周期性地用新注册点云对已加载地图做几何配准，并对修正幅度、
质量、重叠、协方差和新鲜度设门槛。通过时更新 `map -> odom`；拒绝时保留上一次
有效变换，连续失败则报告 degraded/lost 并允许重新执行 global relocalization。

`can_accept_goal` 是导航侧从地图可用、有效 `map -> odom`、持续地图跟踪健康、定位健康和位姿新鲜度
派生出的布尔判断。它不执行配准，不启动 tracking，也不是定位算法阶段。持续定位在
首次定位成功后已经运行；readiness 只决定导航目标是否可以进入执行链。

持续定位不会构建历史关键帧图、发现任意历史回环或优化整条轨迹，因此不等同于
loop closure 或 PGO。

## 5. 回环与 PGO 的当前边界

先把 FAST-LIO2 和回环后端分开：FAST-LIO2 是实时 LiDAR–IMU 前端，负责 IMU
传播、点云去畸变、扫描匹配、里程计和局部增量地图。它不负责历史地点检索、回环
几何验证或全局位姿图优化。一个完整闭环系统通常把 FAST-LIO2 的关键帧/patch 作为
后续回环模块的输入，而不是把回环功能归到 FAST-LIO2 本身。

旧 ROS2 PGO/HBA 包已经删除，不会作为节点、service 或兼容 runtime 回到 Product。
ROS-free `lt_pgo` 与 Rust `pose_graph_opt` kernel 已恢复并进入 native 构建；`lt_pgo`
也随 native release 发布。它是 SaveMap 的短时子进程，不是常驻服务，也不修改在线
`slamd` 的位姿或地图。

Fast-LIO2 在线前端只冻结 `map.pcd`、`poses.txt`、body-local patches 和 manifest；
它不在线发现历史回环，也不生成或发布 `pose_graph.constraints`。SaveMap 在
`OPTIMIZE_SOURCE` 阶段把完整 patch bundle 的工作副本交给
`lt_pgo --auto-constraints`：每对相邻单 patch 经过正反向 trimmed 4DoF
point-to-plane 配准，形成完整 `N-1` measured chain；信息来自实际
`J^T J / sigma^2` 并转换到 body-right tangent，再合并通过几何验证与共识门限的
loop factors。

只有相邻链完整且至少一个 loop 可信时才优化。否则 SaveMap 保留 raw source，并在
`map_optimization.json` 中记录 `insufficient_keyframes`、
`sequential_chain_incomplete`、`no_verified_loops`、`patch_bundle_incomplete` 或
`pgo_timeout` 等具体 skip code。`pose_graph.constraints` 仅由优化器内部临时原子写入、
严格回读并删除，不是 Fast-LIO2 或 Product API 输出，也不会进入最终地图目录。
当前仍缺现场回放、误回环率、轨迹质量和 S100P 性能证据。

`ScanPlanner` 只负责导航局部避障与轨迹规划，名称中的 Scan 与 Scan Context、回环
检测没有关系。continuous map-based localization 也只把当前扫描对齐到保存地图，
不生成历史回环约束。

## 6. `semantic_map.bin` 到底是什么

当前所有现场 Product 都把 `SemanticMapModule` 列为 forbidden module，因而不会
在线生成语义体素地图。`semantic_map.bin` 只可能来自开发、导入或显式离线流程，
并且在地图包里是可选 artifact。

`NativeRelocalizer` 加载 `map.pcd` 时会检查同目录是否存在
`semantic_map.bin`：

- 不存在：MapIcp 使用 `map.pcd`；
- 存在且有效：通过 maps semantic C ABI 读取体素，MapIcp 使用体素中心几何；
- 存在但无效：加载失败，不静默退回 PCD。

当前 MapIcp 没有使用语义类别或置信度加权残差。所以准确说法是“可从语义地图
artifact 提取体素几何作为 ICP target”，不是“已经进行语义定位”。

## 7. 定位相关 DDS Topic 全表

逻辑名 `/slam/...` 在 native CycloneDDS wire 上映射为 `rt/slam/...`。

### `slamd` 输入与控制

| 逻辑 / wire | 类型 | QoS | 当前用途 |
| --- | --- | --- | --- |
| `/lidar/raw_frame` / `rt/lidar/raw_frame` | `lingtu.dds.LivoxFrame` | best-effort, depth 2, lifespan 350 ms | 现场 Fast-LIO2 扫描输入 |
| `/imu/raw` / `rt/imu/raw` | `lingtu.dds.Imu` | best-effort, depth 256 | 现场 Fast-LIO2 IMU 输入 |
| `/slam/odom_prior` / `rt/slam/odom_prior` | `lingtu.dds.Odometry` | best-effort, depth 256 | 可选外部里程计/视觉先验；现场关闭，仿真/实验用 |
| `/slam/map_command` / `rt/slam/map_command` | `lingtu.dds.SlamMapSnapshotRequest` | reliable, volatile, depth 32 | mapd 请求 native SLAM 冻结指定 map/Product session 的保存快照 |
| `/slam/relocalization/request` / `rt/slam/relocalization/request` | `lingtu.dds.RelocalizationRequest` | reliable, volatile, depth 32 | typed seeded/global/status/track；Product 运行时禁止路径选图和 `load_map` |

### `slamd` 输出

| 逻辑 / wire | 类型 | QoS | 语义 |
| --- | --- | --- | --- |
| `/tf` / `rt/tf` | `lingtu.dds.TFMessage` | best-effort, depth 100 | 动态 `odom/body` 与 `map/odom` 关系 |
| `/slam/odometry` / `rt/slam/odometry` | `lingtu.dds.Odometry` | best-effort, depth 5, deadline 20 ms | 高频权威里程计 |
| `/slam/state_at_scan` / `rt/slam/state_at_scan` | `lingtu.dds.Odometry` | 同上 | 精确到已接受扫描时间戳的状态 |
| `/slam/registered_cloud` / `rt/slam/registered_cloud` | `lingtu.dds.PointCloud2` | best-effort, depth 2, lifespan 200 ms | body frame 注册扫描 |
| `/slam/map_observation` / `rt/slam/map_observation` | `lingtu.dds.MapObservation` | best-effort, depth 2 | 同一接受扫描的点、位姿、原点与序号 |
| `/slam/map_cloud` / `rt/slam/map_cloud` | `lingtu.dds.PointCloud2` | best-effort, depth 2, lifespan 200 ms | 实时地图点云 |
| `/slam/saved_map_cloud` / `rt/slam/saved_map_cloud` | `lingtu.dds.PointCloud2` | 同上 | 一次成功 snapshot 对应的点云 |
| `/slam/map_event` / `rt/slam/map_event` | `lingtu.dds.SlamMapSnapshotAck` | reliable, transient-local, depth 64 | native SLAM 返回 request-correlated snapshot 身份、路径、epoch、序号和健康证据；mapd 消费并继续 SaveMap |
| `/slam/relocalization/response` / `rt/slam/relocalization/response` | `lingtu.dds.RelocalizationResponse` | reliable, transient-local, depth 64 | 与 typed request 对应的结果/质量/重叠证据 |
| `/slam/localization_quality` / `rt/slam/localization_quality` | `lingtu.dds.Float32` | best-effort, depth 5, deadline 20 ms | 高频定位质量 |
| `/slam/localization_health` / `rt/slam/localization_health` | `lingtu.dds.Text` JSON | reliable, volatile, depth 10 | 低频状态、freshness、退化和重定位健康 |

### 容易误读的声明

| Topic | 当前解释 |
| --- | --- |
| `/tf_static` / `rt/tf_static` | producer 是 calibration，不是 `slamd`；已移除 `slamd` 的重复 writer |
| `/slam/cumulative_map_cloud` | 兼容常量和 QoS 映射仍存在，但当前 `slamd` 没有 writer，不能当作运行输出 |
| `/gnss/fix`, `/gnss/status`, `/gnss/odom` | GNSS endpoint 合同；当前 real RunPlan 不启动该角色，`slamd` 不订阅 |

## 8. SaveMap 的直接合同

每张地图只有一个 canonical 目录：`<map_root>/<map_id>/`。SaveMap 的路径是：

```text
1. SDK client 经 Gateway 调用 mapd 的唯一公开动作 `save_map`
2. mapd 发布 typed `SlamMapSnapshotRequest`
3. slamd 从同一 Product session / SLAM epoch 冻结快照并返回 `SlamMapSnapshotAck`
4. mapd 校验回执身份并把 snapshot 提交给 SaveMapEngine
5. Optimize source：完整 patch bundle 自动走 `lt_pgo --auto-constraints`；显式约束走 `lt_pgo --constraints`；图不满足条件时保留 raw source 并记录具体 skip code
6. Build：在 staging 中清理点云并生成当前规划器需要的 artifact
7. Check：确认 `map.pcd`、`metadata.json` 和当前规划器使用的 `octomap.ot` 可打开
8. Replace：持有 MapLock，把旧目录临时移到一次性 backup，再用 staging 替换 canonical 目录
```

`BeginSaveMap` / `ProvideSaveMapSnapshot` 只存在于 mapd 内部 C++ 调用链，不是
Python 或 Product API。保存路径没有 ROS2 node、service 或 message 依赖。

MapLock 防止两个保存事务同时替换同一地图。staging 防止构建中的半成品暴露给
运行时；临时 backup 只用于本次替换失败后的恢复，成功后立即删除，不形成历史版本。
检查只回答运行时马上要读取的最小 artifact 是否可用，不扫描或证明整个目录。

这里没有历史版本、版本指针、版本列表或回滚 API。PGO 是同一 SaveMap 事务中的
可选数据驱动阶段，不创建第二条保存路径；优化失败时不得替换 canonical 目录。

地图文件按用途分组：

| 分组 | 文件 | 处理意见 |
| --- | --- | --- |
| 运行必需 | `map.pcd`, `metadata.json`, `octomap.ot` | 保存前做最小可打开检查 |
| 兼容显示 | `map.pgm`, `map.yaml` | 保留兼容视图；不是 3D 运行真相源 |
| 可追溯/清理 | `poses.txt`, `patches/*.pcd` | 提供给真实的动态点清理；不应成为调用方算法开关 |
| 保存优化证据 | `map_optimization.json`, `patch_bundle.manifest` | 记录本次是否执行优化及 patch bundle 完整性；不是地图身份或版本协议 |
| 规划器可选 | `esdf.npz`, `traversability.npz` | 仅在声明该能力的流程构建 |
| 语义可选 | `semantic_map.bin` | 不单独赋予 `activation_ready` |

更详细的地图目录与 artifact 合同见
[`MAP_SERVICE_CONTRACT.md`](./MAP_SERVICE_CONTRACT.md)。

## 9. 代码入口

| 问题 | 实现入口 |
| --- | --- |
| Product 生命周期 | `src/lingtu/control.py`, `src/lingtu/real/switch.py`, `src/lingtu/real/systemd.py` |
| Gateway 定位 API | `src/gateway/routes/operations.py` |
| Gateway -> slamctl 适配 | `src/localization/adapters/relocalization.py` |
| DDS client | `src/localization/slam/cpp/slam_control.cpp` |
| `slamd` DDS runtime | `src/localization/slam/cpp/cyclone_runtime.cpp` |
| Fast-LIO2 | `src/localization/slam/cpp/fastlio.cpp` |
| BBS3D + MapIcp | `src/localization/slam/cpp/native_relocalizer.cpp`, `map_icp.cpp` |
| Topic/QoS | `src/message/cpp/topics.hpp`, `qos.hpp`, `config/runtime_graph/topics.yaml` |
| SaveMap transaction | `src/maps/cpp/mapd/save_coordinator.cpp`, `src/maps/cpp/save.cpp` |
