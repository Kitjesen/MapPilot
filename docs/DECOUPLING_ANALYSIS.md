# 3D NAV — ROS 2 架构审计与优化路径

> 审计日期: 2026-02-13
> 目标: 全面审计 ROS 2 耦合度，确定务实的优化路径

## 0. 决策结论 (先说结果)

**不建议全量解耦到裸 CycloneDDS。** 原因:

1. ROS 2 底层已经运行在 CycloneDDS 之上 (rmw_cyclonedds)
2. "解耦"意味着丢掉 tf2、rosbag2、rviz2、launch、消息生成、诊断等整套生态
3. 重建这些能力的成本远高于 ROS 2 带来的开销
4. 本系统 92% 代码已是纯 C++，ROS 2 只是通信胶水层

**正确路径: ROS 2 内部优化 → CycloneDDS 配置调优 → 局部硬实时旁路**

以下审计数据仍然有价值 — 它精确标定了 ROS 2 耦合边界，
有助于做 QoS 调优、模块化测试、和未来可能的局部旁路设计。

---

---

## 1. 整体架构现状

```
                    ┌─────────────────────────────┐
                    │      Flutter App (gRPC)      │  ← 已完全独立
                    └──────────────┬───────────────┘
                                   │ gRPC
┌──────────────────────────────────┼──────────────────────────────────┐
│                    ROS 2 Node Process                               │
│                                  │                                  │
│  ┌───────────────────────────────┼───────────────────────┐         │
│  │           remote_monitoring (grpc_gateway)             │         │
│  │  ┌─────────────┐ ┌───────────┐ ┌──────────────┐      │         │
│  │  │ StatusAggr  │ │SafetyGate │ │ TaskManager  │ ...  │         │
│  │  │ (5 subs)    │ │ (2 sub    │ │ (3 sub       │      │         │
│  │  │ (3 timer)   │ │  2 pub)   │ │  2 pub, TF)  │      │         │
│  │  │ (TF)        │ │           │ │              │      │         │
│  │  └──────┬──────┘ └─────┬─────┘ └──────┬───────┘      │         │
│  │         │ ROS topics   │              │              │         │
│  └─────────┼──────────────┼──────────────┼──────────────┘         │
│            │              │              │                         │
│  ┌─────────┼──────────────┼──────────────┼─────────────────┐      │
│  │         ▼              ▼              ▼                  │      │
│  │  /nav/odometry  /nav/terrain_map  /nav/way_point        │      │
│  │  /nav/path      /nav/cmd_vel      /nav/stop             │      │
│  │  /nav/slam/cloud /localization_quality  ...              │      │
│  └─────────────────────────────────────────────────────────┘      │
│                              │                                     │
│  ┌───────────────────────────┼───────────────────────────┐        │
│  │            SLAM / Planning / Autonomy Nodes            │        │
│  │  fastlio2 │ localizer │ pgo │ local_planner │ ...     │        │
│  └───────────────────────────────────────────────────────┘        │
│                              │                                     │
│  ┌───────────────────────────┼───────────────────────────┐        │
│  │              Drivers (livox, robot_driver)              │        │
│  └───────────────────────────────────────────────────────┘        │
└────────────────────────────────────────────────────────────────────┘
```

---

## 2. 逐文件 ROS 2 依赖清单

### 2.1 remote_monitoring (22 源文件)

#### 完全无 ROS 依赖 (纯 C++/gRPC/Protobuf) — 7 个文件

| 文件 | 行数 | 说明 |
|------|------|------|
| `core/flight_recorder.hpp/cpp` | ~295 | 环形缓冲黑盒 |
| `core/event_buffer.hpp/cpp` | ~266 | 事件环形缓冲 |
| `core/lease_manager.hpp/cpp` | ~119 | 租约管理 |
| `core/idempotency_cache.hpp` | ~50 | 幂等缓存 |
| `services/control_service.hpp/cpp` | ~532 | gRPC 控制服务 |
| `services/telemetry_service.hpp/cpp` | ~154 | gRPC 遥测服务 |

#### ROS 轻耦合 (仅 Node 指针 + 日志) — 2 个文件

| 文件 | ROS 用途 | 耦合度 |
|------|----------|--------|
| `core/service_orchestrator.cpp` | `node_->get_logger()` (16 处日志) | ~15% |
| `grpc_gateway.cpp` | `declare_parameter` + 1 个 publisher + 5 处日志 | ~35% |

#### ROS 中度耦合 (Pub/Sub + Param) — 4 个文件

| 文件 | 订阅 | 发布 | 定时器 | TF | 参数 | 日志 |
|------|------|------|--------|-----|------|------|
| `core/mode_manager.cpp` | 0 | 3 (Int8/Float32/PoseStamped) | 0 | 0 | 3 | 7 |
| `core/geofence_monitor.cpp` | 2 (Odom/Polygon) | 2 (Int8/String) | 1 | 0 | 8 | 4 |
| `services/system_service.cpp` | 0 | 0 | 0 | 0 | 3 | 6 |
| `webrtc_bridge.cpp` | 2 (CompressedImage/Image) | 0 | 0 | 0 | 4 | 40+ |

#### ROS 重耦合 (Pub/Sub + TF + 消息类型) — 5 个文件

| 文件 | 订阅 | 发布 | 定时器 | TF | 参数 | 消息类型 |
|------|------|------|--------|-----|------|---------|
| `status_aggregator.cpp` | 5 | 0 | 3 | Buffer+Listener+canTransform+RPY | 9 | Odom/Path/PC2/Int8/RobotState |
| `core/safety_gate.cpp` | 2 | 2 | 0 | Quaternion+Matrix3x3+RPY | 8 | Odom/PC2/TwistStamped/Int8 |
| `core/health_monitor.cpp` | 4 | 2 | 2 | Buffer+Listener+canTransform | 7 | Odom/Path/PC2/Float32/Int8/String |
| `core/task_manager.cpp` | 3 | 2 | 1 | Buffer+Listener+lookupTransform+doTransform | 5 | Odom/PointStamped/Path/Int8 |
| `core/localization_scorer.cpp` | 1 | 0 | 1 | Buffer+Listener+3x lookupTransform | 4 | Float32 |
| `services/data_service.cpp` | 模板订阅 | 0 | 0 | 0 | 10+ | PC2/Path/CompressedImage |

### 2.2 SLAM (5 个包, ~20 源文件)

#### 纯算法文件 (无任何 ROS 依赖)

| 包 | 文件 | 说明 |
|-----|------|------|
| fastlio2 | `lidar_processor.cpp/h` | LiDAR 处理 |
| fastlio2 | `ieskf.cpp/h` | 迭代误差状态卡尔曼 |
| fastlio2 | `commons.cpp/h` | 数学公共函数 |
| fastlio2 | `map_builder.cpp/h` | 地图构建器 |
| fastlio2 | `imu_processor.cpp/h` | IMU 处理 |
| fastlio2 | `ikd_Tree.cpp/h` | 增量 KD 树 |
| localizer | `icp_localizer.cpp/h` | ICP 定位 |
| localizer | `commons.cpp/h` | 公共函数 |
| pgo | `simple_pgo.cpp/h` | 位姿图优化 (GTSAM) |
| pgo | `commons.cpp/h` | 公共函数 |
| hba | `hba.cpp/h` | 层次化 BA |
| hba | `blam.cpp/h` | 批量 BA |
| hba | `commons.cpp/h` | 公共函数 |

#### ROS Node 壳 (薄封装)

| 文件 | 订阅 | 发布 | 服务 | TF | message_filters |
|------|------|------|------|-----|----------------|
| `lio_node.cpp` | 2 (IMU/CustomMsg) | 4 (PC2/Odom/Path) | 1 (SaveMaps) | TransformBroadcaster | 无 |
| `localizer_node.cpp` | 0 (通过 msg_filter) | 2 (PC2/Float32) | 2 (Relocalize/IsValid) | TransformBroadcaster | ApproximateTime<PC2, Odom> |
| `pgo_node.cpp` | 0 (通过 msg_filter) | 1 (MarkerArray) | 1 (SaveMaps) | TransformBroadcaster | ApproximateTime<PC2, Odom> |
| `hba_node.cpp` | 0 | 1 (PC2) | 2 (RefineMap/SavePoses) | 无 | 无 |

#### 特殊: `utils.h` (fastlio2)

- 包含 `std_msgs/msg/header.hpp` 和 `livox_ros_driver2/msg/custom_msg.hpp`
- 用于时间戳转换和 Livox 自定义消息解析
- **需要抽象**: 定义一个平台无关的 TimestampedPointCloud 结构体

### 2.3 base_autonomy (6 个源文件, 3674 行)

| 文件 | 行数 | 订阅 | 发布 | TF | PCL-ROS | 参数 | 算法占比 |
|------|------|------|------|-----|---------|------|---------|
| `localPlanner.cpp` | 1197 | 9 | 4 | RPY 转换 | fromROSMsg x4, toROSMsg x2 | 44 | **65%** |
| `pathFollower.cpp` | 513 | 6 | 1 | RPY 转换 | (未用) | 25 | **60%** |
| `terrainAnalysis.cpp` | 782 | 4 | 1 | RPY 转换 | fromROSMsg x1, toROSMsg x1 | 20 | **75%** |
| `terrainAnalysisExt.cpp` | 598 | 5 | 1 | RPY 转换 | fromROSMsg x2, toROSMsg x1 | 14 | **70%** |
| `sensorScanGeneration.cpp` | 142 | 0 (msg_filter) | 2 | Broadcaster | fromROSMsg/toROSMsg | 0 | **15%** |
| `visualizationTools.cpp` | 442 | 3 | 6 | RPY 转换 | fromROSMsg/toROSMsg | 12 | **40%** |

**特点**: 所有代码在单文件中，无 .h 头文件分离。算法和 I/O 混合在同一个 `processLoop()` 中。

### 2.4 Drivers (2 个包)

| 包 | 类型 | ROS 耦合 | 独立 SDK |
|----|------|----------|---------|
| `livox_ros_driver2` | C++ | 高 — 但内含 Livox-SDK2 独立库 | **Livox-SDK2 完全独立**, `LivoxLidarSdkInit()` 等 C API |
| `robot_driver` | Python | 高 — `han_dog_bridge.py` 是 ROS node | 底层是 gRPC → 电机控制器，可以改 |

### 2.5 global_planning (2 个包)

| 包 | ROS 耦合 | 纯算法 |
|----|----------|--------|
| `PCT_planner` | 脚本层 (Python rclpy) | **`lib/` 完全独立** — A*, GPMP, ElevationPlanner, MapManager |
| `pct_adapters` | 100% ROS | 无 (纯适配器) |

---

## 3. ROS 2 依赖分类汇总

### 3.1 使用的 ROS 2 功能

| 功能 | 使用模块数 | 替代方案 |
|------|-----------|---------|
| **Pub/Sub** | 15+ 个节点 | CycloneDDS / ZeroMQ / 自定义 |
| **Parameter Server** | 12 个文件, ~200 个参数 | YAML 加载 (**已部分完成**: `robot_config.yaml`) |
| **TF2 Transform Tree** | 6 个核心模块 | 自建变换管理器 / standalone tf2 |
| **RCLCPP Logging** | 12 个文件, ~120 处调用 | spdlog / 自定义 logger |
| **ROS Services** | 3 个节点 (Relocalize/SaveMaps/RefineMap) | gRPC 替代 (**remote_monitoring 已是 gRPC**) |
| **message_filters** | 3 个节点 (localizer/pgo/sensor_scan) | 自建时间同步器 |
| **PCL-ROS Bridge** | 8 个文件 | PCL 直接操作 (去掉 fromROSMsg/toROSMsg) |
| **TF Broadcaster** | 4 个 SLAM 节点 | 自建 TF 发布或 DDS 直发 |
| **rosidl 消息定义** | 5 个自定义 msg/srv | Protobuf / IDL / FlatBuffers |
| **Launch System** | 8 个 launch.py | supervisord / shell 脚本 (**已完成**: Docker) |

### 3.2 使用的 ROS 消息类型 (14 种)

| 类型 | 使用处 | 大小/频率 |
|------|--------|----------|
| `nav_msgs/Odometry` | 8+ 个文件 | ~200 bytes, 100 Hz |
| `sensor_msgs/PointCloud2` | 8+ 个文件 | **~1 MB**, 10 Hz |
| `nav_msgs/Path` | 5 个文件 | ~10 KB, 1 Hz |
| `geometry_msgs/TwistStamped` | 3 个文件 | ~100 bytes, 50 Hz |
| `geometry_msgs/PoseStamped` | 3 个文件 | ~100 bytes, ~1 Hz |
| `geometry_msgs/PointStamped` | 3 个文件 | ~50 bytes, ~1 Hz |
| `std_msgs/Int8` | 6 个文件 | 1 byte, 事件驱动 |
| `std_msgs/Float32` | 4 个文件 | 4 bytes, 10 Hz |
| `std_msgs/String` | 2 个文件 | ~100 bytes, 事件驱动 |
| `sensor_msgs/Imu` | 2 个文件 | ~200 bytes, 200 Hz |
| `sensor_msgs/CompressedImage` | 1 个文件 | ~50 KB, 30 Hz |
| `geometry_msgs/PolygonStamped` | 2 个文件 | ~1 KB, 事件驱动 |
| `visualization_msgs/MarkerArray` | 1 个文件 | 可变, 0.5 Hz |
| `interface/RobotState` (自定义) | 2 个文件 | ~500 bytes, 10 Hz |

### 3.3 自定义消息和服务 (interface 包)

| 类型 | 名称 | 字段 |
|------|------|------|
| msg | `RobotState` | Header, BatteryState, joints, IMU |
| msg | `BatteryState` | voltage, current, percentage, temperature |
| srv | `Relocalize` | `pcd_path, x/y/z/yaw/pitch/roll` → `success, message` |
| srv | `SaveMaps` | `file_path, save_patches` → `success, message` |
| srv | `RefineMap` | `maps_path` → `success, message` |
| srv | `SavePoses` | `file_path` → `success, message` |
| srv | `IsValid` | `code` → `valid` |

---

## 4. 务实优化路径 (推荐)

**不做全量裸 DDS 迁移。** 保留 ROS 2 生态，在其框架内优化。

### 4.1 决策顺序

```
Step 1: RMW 切换到 CycloneDDS + XML 配置固化
        ↓ (零代码改动)
Step 2: QoS 策略场景化 — 按链路特性配置 reliability/history/deadline
        ↓ (配置文件改动)
Step 3: 链路级 profiling — 找到真正的瓶颈
        ↓ (测量驱动)
Step 4: 局部旁路 — 仅对硬实时链路 (cmd_vel) 做零拷贝/共享内存
        ↓ (仅改 1-2 条链路)
Step 5 (如确有必要): 对最少数模块做 Transport 抽象化
```

### 4.2 各步详情

见以下独立配置文件:
- `config/cyclonedds.xml` — CycloneDDS 运行时配置
- `config/qos_profiles.yaml` — ROS 2 QoS 场景化参数
- `docs/COMM_OPTIMIZATION.md` — 完整优化指南

---

## 5. 架构审计数据 (保留参考)

以下数据仍然有价值——它精确标定了代码中的 ROS 2 边界，
有助于判断 QoS 应该在哪些链路上做调优，以及未来如果
确需局部旁路时的影响面。

### 代码行数统计

| 层 | 纯算法行数 | ROS 耦合行数 | 纯算法占比 |
|----|-----------|-------------|-----------|
| SLAM 算法 (`map_builder`, `icp`, `pgo`, `hba`) | ~8,000 | 0 | **100%** |
| PCT Planner `lib/` | ~12,000 | 0 | **100%** |
| remote_monitoring 纯 C++ 模块 | ~1,400 | 0 | **100%** |
| remote_monitoring ROS 耦合模块 | ~2,600 | ~1,400 | **65%** |
| base_autonomy 算法部分 | ~2,200 | 0 | **100%** |
| base_autonomy ROS I/O | ~1,470 | ~1,470 | **0%** |
| Drivers | ~660 | ~660 | **0%** |
| Livox-SDK2 | ~15,000 | 0 | **100%** |
| **总计** | **~43,330** | **~3,530** | **~92%** |

### ROS 2 耦合热力图 (按模块)

```
耦合度: ██ 重  ▓▓ 中  ░░ 轻  ·· 无

remote_monitoring:
  flight_recorder      ··  (0%)
  event_buffer         ··  (0%)
  lease_manager        ··  (0%)
  idempotency_cache    ··  (0%)
  control_service      ··  (0%)
  telemetry_service    ··  (0%)
  service_orchestrator ░░  (15%, 仅日志)
  mode_manager         ▓▓  (25%, 3 pub)
  system_service       ▓▓  (25%, 2 srv client)
  grpc_gateway         ▓▓  (35%, 1 pub + param)
  geofence_monitor     ▓▓  (50%, 2 sub + 2 pub)
  task_manager         ██  (35%, 3 sub + 2 pub + TF)
  health_monitor       ██  (40%, 4 sub + 2 pub + TF)
  safety_gate          ██  (45%, 2 sub + 2 pub + TF)
  localization_scorer  ██  (45%, 1 sub + 3 TF lookups)
  status_aggregator    ██  (55%, 5 sub + 3 timer + TF)
  data_service         ██  (30%, 模板订阅 + 流)
  webrtc_bridge        ▓▓  (25%, 2 sub)

SLAM:
  算法库 (13 个文件)   ··  (0%, 纯 C++/Eigen/GTSAM/PCL)
  Node 壳 (4 个文件)   ██  (100%, pub/sub/TF/srv)

base_autonomy:
  terrainAnalysis      ▓▓  (25%, 算法 75%)
  terrainAnalysisExt   ▓▓  (30%, 算法 70%)
  localPlanner         ▓▓  (35%, 算法 65%)
  pathFollower         ▓▓  (40%, 算法 60%)
  sensorScanGeneration ██  (85%, 几乎全是 I/O)
  visualizationTools   ██  (60%, 可视化重)
```

---

## 6. 未来可选: Transport 抽象层 (仅供参考)

如果未来确实需要把某些模块脱离 ROS 2 独立测试或部署,
以下接口设计可作为参考。**当前不建议实施。**

> 详见附录 A (本文档原始版本存档)

---

*文档生成: 2026-02-13 | 基于 src/ 全量代码审计*
*修订: 2026-02-13 | 从"全量解耦方案"修正为"务实优化路径"*
