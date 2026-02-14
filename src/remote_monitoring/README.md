# Remote Monitoring - gRPC Gateway

机器人远程监控与控制系统 | ROS 2 + gRPC

**版本**: 1.2.2  
**日期**: 2026-02-12  
**状态**: Phase 2 完成（航点管理 + Flutter 集成 + 重连恢复 + 质量收口）

---

## 目录

- [1. 系统概述](#1-系统概述)
- [2. 架构设计](#2-架构设计)
- [3. 功能模块](#3-功能模块)
- [4. Proto 接口契约](#4-proto-接口契约)
- [5. 配置参数](#5-配置参数)
- [6. 构建与运行](#6-构建与运行)
- [7. 客户端接入](#7-客户端接入)
- [8. 代码审查与已知问题](#8-代码审查与已知问题)
- [9. 实施进度](#9-实施进度)
- [10. 文件清单](#10-文件清单)

---

## 1. 系统概述

### 1.1 定位

本包（`remote_monitoring`）是 3D 导航机器人的**远程监控与控制网关**。它将 ROS 2 内部的话题、TF、服务等桥接为标准 gRPC 接口，使 App 端（Flutter/Android/iOS）**无需依赖 ROS 2 DDS** 即可完成：

| 能力 | 说明 | 优先级 |
|------|------|--------|
| **See（看到）** | 低延迟视频、实时状态（姿态/电量/温度/模式）、SLAM 地图、点云可视化 | P0 |
| **Control（控制）** | 模式切换、任务指令、遥操作（速度/角速度）、急停、Safety Gate 本地最高优先级 | P0 |
| **Manage（管理）** | 事件告警、地图/日志下载、OTA、用户权限、审计 | P1 |

### 1.2 设计原则

- **App 永远不直接访问 ROS 2 DDS**：仅通过 gRPC 网关通信，减少攻击面、降低耦合
- **控制面/状态面/数据面严格隔离**：大数据（点云/地图/视频）可降级，不拖垮控制
- **安全门控在机器人本地闭环**：网络异常不可突破 Safety Gate

---

## 2. 架构设计

### 2.1 系统拓扑

```
┌─────────────────────────────────────┐
│  Flutter App（Android/iOS/Pad）      │
│  ├─ gRPC Client（4 services）        │
│  ├─ WebRTC Client（视频，可选）       │
│  └─ UI（状态/控制/地图/告警）         │
└─────────────┬───────────────────────┘
              │ Wi-Fi / LAN (TCP :50051)
              ▼
┌─────────────────────────────────────┐
│  Robot（grpc_gateway 进程）          │
│  ┌─ GrpcGateway ──────────────────┐ │
│  │  ├─ SystemService              │ │
│  │  ├─ ControlService             │ │
│  │  ├─ TelemetryService           │ │
│  │  └─ DataService                │ │
│  ├─ StatusAggregator              │ │
│  ├─ LeaseManager                  │ │
│  ├─ EventBuffer                   │ │
│  ├─ SafetyGate → /cmd_vel_safe    │ │
│  ├─ IdempotencyCache              │ │
│  └─ WebRTCBridge（可选）            │ │
└─────────────┬───────────────────────┘
              │ ROS 2 DDS
              ▼
┌─────────────────────────────────────┐
│  ROS 2 Navigation Stack            │
│  /Odometry /terrain_map /path      │
│  /camera/* /cloud_registered       │
│  /cmd_vel_safe (Safety Gate 输出)   │
└─────────────────────────────────────┘
```

### 2.2 技术选型

| 组件 | 协议/技术 | 说明 |
|------|-----------|------|
| 控制/状态/数据主干 | gRPC (HTTP/2 + Protobuf) | 强类型、易版本化、支持 streaming |
| 视频 | WebRTC (SRTP)，可选 | 低延迟、弱网自适应 |
| WebRTC 信令 | gRPC 双向流 | 统一走网关 |
| 发现 | mDNS + 备用固定 IP | `robot.local` / `192.168.4.1` |

### 2.3 端口规划

| 端口 | 用途 | 协议 |
|------|------|------|
| **50051** | gRPC Gateway（所有 RPC + WebRTC 信令） | TCP |
| WebRTC 媒体 | 动态 UDP | WebRTC 内部协商 |

---

## 3. 功能模块

### 3.1 GrpcGateway（入口层）

**文件**: `src/grpc_gateway.cpp` | `include/remote_monitoring/grpc_gateway.hpp`

统一入口，负责：
- 初始化所有核心组件和服务实例
- 在独立线程启动 gRPC Server（`0.0.0.0:50051`）
- 后台循环（100ms）检查 Lease 超时和 Deadman 状态
- 注册 4 个 gRPC Service

### 3.2 StatusAggregator（状态聚合器）

**文件**: `src/status_aggregator.cpp` | `include/remote_monitoring/status_aggregator.hpp`

ROS 2 话题订阅 → Protobuf 状态消息转换：

| 订阅话题 | 用途 |
|----------|------|
| `/Odometry` | 位姿、速度、RPY 角 |
| `/terrain_map` | 地形点云频率统计 |
| `/path` | 路径频率统计 |
| `/slow_down` | 减速信号 |
| TF (`map→odom→body`) | 坐标系可用性 |

输出两种状态：
- **FastState** (默认 30Hz)：位姿、速度、IMU、RPY、TF 状态
- **SlowState** (默认 1Hz)：当前模式、话题频率统计

### 3.3 LeaseManager（租约管理器）

**文件**: `src/core/lease_manager.cpp` | `include/remote_monitoring/core/lease_manager.hpp`

- **同一时间仅允许一个 operator 持有 Lease**
- 操作：`AcquireLease` / `RenewLease` / `ReleaseLease` / `ValidateLease`
- Token 生成：`lease_` + 16 位随机十六进制
- TTL 超时自动释放（由 GrpcGateway 每秒检查）
- 线程安全（mutex 保护）

### 3.4 SafetyGate（安全门控）

**文件**: `src/core/safety_gate.cpp` | `include/remote_monitoring/core/safety_gate.hpp`

**本地强制执行，不依赖网络**：

| 保护机制 | 默认参数 | 说明 |
|----------|----------|------|
| Deadman Switch | 300ms | 遥操作输入超时 → 速度归零 |
| 速度限幅 | 1.0 m/s | 线速度上限 |
| 角速度限幅 | 1.0 rad/s | 角速度上限 |
| 倾斜保护 | 30° | Roll/Pitch 超限 → 速度归零 |
| 急停 | - | 所有速度归零 |

- 订阅 `/Odometry` 获取实时姿态角
- 输出安全速度到 `/cmd_vel_safe`
- 每次处理返回限幅原因列表

### 3.5 EventBuffer（事件缓冲器）

**文件**: `src/core/event_buffer.cpp` | `include/remote_monitoring/core/event_buffer.hpp`

- 环形缓冲区，容量 1000 条
- 事件 ID：时间戳(12位hex) + 序列号(6位hex)，严格递增
- **回放能力**：客户端携带 `last_event_id` 连接，服务端补发缺失事件
- **实时推送**：`WaitForEventAfter` 使用条件变量阻塞等待，避免空转
- 事件类型覆盖：系统启停、模式切换、导航、安全、传感器、任务、网络

### 3.6 IdempotencyCache（幂等缓存）

**文件**: `include/remote_monitoring/core/idempotency_cache.hpp`（Header-only）

- 基于 `request_id` 的请求去重
- TTL 默认 600 秒（10 分钟）
- 惰性清理：在 `TryGet` 时检查过期
- 用于所有事务型 RPC（AcquireLease, SetMode, EmergencyStop, StartTask 等）

### 3.7 SystemService（系统服务）

**文件**: `src/services/system_service.cpp`

| RPC | 类型 | 说明 |
|-----|------|------|
| `Login` | Unary | 登录（当前简化实现，接受任何用户名） |
| `Logout` | Unary | 登出，可选释放 Lease（`release_lease=true` 调用 `LeaseManager::ForceRelease`） |
| `Heartbeat` | Unary | 心跳，返回 RTT 和活跃会话数 |
| `GetRobotInfo` | Unary | 机器人 ID、固件版本、软件版本 |
| `GetCapabilities` | Unary | 支持的资源（camera/front, pointcloud/lidar 等）和任务类型 |
| `Relocalize` | Unary | 重定位：加载指定地图并设置初始位姿（调用 ROS `/relocalize` 服务） |
| `SaveMap` | Unary | 保存当前地图到指定路径（调用 ROS `/save_map` 服务，支持 `save_patches`） |
| `ListMaps` | Unary | 列出 `maps_directory` 下所有地图文件，返回名称/大小/创建时间 |
| `DeleteMap` | Unary | 删除指定地图文件 |
| `RenameMap` | Unary | 重命名地图文件 |

### 3.8 ControlService（控制服务）

**文件**: `src/services/control_service.cpp`

| RPC | 类型 | 说明 |
|-----|------|------|
| `AcquireLease` | Unary | 获取独占控制租约 |
| `RenewLease` | Unary | 续约（推荐 2Hz 调用） |
| `ReleaseLease` | Unary | 释放租约 |
| `SetMode` | Unary | 切换模式（idle/manual/teleop/autonomous/mapping/estop） |
| `EmergencyStop` | Unary | 急停（软/硬），触发 Safety Gate + 挂起 TaskManager |
| `StreamTeleop` | **双向流** | 遥操作：客户端发 TeleopCommand ← → 服务端返回 TeleopFeedback |
| `StartTask` | Unary | 启动任务（需 Lease + AUTONOMOUS 模式） |
| `CancelTask` | Unary | 取消任务（需 Lease） |
| `PauseTask` | Unary | 暂停任务 |
| `ResumeTask` | Unary | 恢复任务 |
| `GetTaskStatus` | Unary | 查询任务进度 |
| `GetActiveWaypoints` | Unary | 查询当前活跃航点（来源/列表/进度） |
| `ClearWaypoints` | Unary | 清除所有航点并立即停车（需 Lease） |

**航点管理流程**（App 端推荐调用顺序）：
1. `GetActiveWaypoints` → 查看当前航点（来源: APP/PLANNER/NONE）
2. `ClearWaypoints` → 清除旧航点（可选，发布 /stop=2 立即停车）
3. `AcquireLease` → 获取操作租约（如果还没有）
4. `SetMode(AUTONOMOUS)` → 确保自主模式
5. `StartTask` → 发送航点任务

**航点来源说明**：
- `WAYPOINT_SOURCE_APP` — 由 App 通过 StartTask 下发的航点列表
- `WAYPOINT_SOURCE_PLANNER` — 全局规划器（pct_path_adapter）发布的航点
- `WAYPOINT_SOURCE_NONE` — 无活跃航点

**安全联动**：
- E-stop / 切 TELEOP / 切 IDLE → TaskManager 自动挂起任务
- 回 AUTONOMOUS → 自动恢复系统挂起的任务（用户手动暂停的不受影响）
- 单航点超时（默认 300s）→ 任务自动 FAILED

**遥操作关键约束**：
- 必须携带有效 `lease_token`，否则拒绝
- 服务端执行"新覆盖旧"：只取最新命令
- `TeleopFeedback` 返回：实际执行速度、限幅原因、安全状态、控制延迟

### 3.9 TelemetryService（遥测服务）

**文件**: `src/services/telemetry_service.cpp`

| RPC | 类型 | 说明 |
|-----|------|------|
| `StreamFastState` | Server Stream | 高频状态（最高 60Hz，默认 30Hz） |
| `StreamSlowState` | Server Stream | 低频状态（默认 1Hz） |
| `StreamEvents` | Server Stream | 事件流（先回放历史，再实时推送） |
| `AckEvent` | Unary | 确认事件（客户端已查看） |

**事件流特性**：
- 连接时携带 `last_event_id` 实现回放
- 支持按 `EventType` 和 `EventSeverity` 过滤
- 实时阶段使用条件变量阻塞等待，不空转

### 3.10 DataService（数据服务）

**文件**: `src/services/data_service.cpp`（1066 行，最大文件）

| RPC | 类型 | 说明 |
|-----|------|------|
| `ListResources` | Unary | 列出可用资源（camera/front, map/global_map, pointcloud/local_cloud） |
| `Subscribe` | Server Stream | 订阅资源流，支持频率限制 |
| `Unsubscribe` | Unary | 取消订阅（通过 cancel flag） |
| `DownloadFile` | Server Stream | 分块文件下载（默认 64KB/块） |
| `StartCamera` | Unary | 启动 WebRTC 相机（外部命令式） |
| `StopCamera` | Unary | 停止 WebRTC 相机 |
| `WebRTCSignaling` | **双向流** | WebRTC 信令（Offer/Answer/ICE/Hangup） |

**Subscribe 资源流**：

| 资源类型 | 默认话题 | 默认频率 | QoS |
|----------|----------|----------|-----|
| Camera | `/camera/color/image_raw/compressed` | 15 Hz | SensorDataQoS |
| Map | `/overall_map` | 1 Hz | Transient Local |
| PointCloud | `/cloud_registered` | 10 Hz | SensorDataQoS |

特性：
- `resource_id.name` 以 `/` 开头时直接作为 ROS topic 覆盖
- 相机话题有 fallback 机制（主话题无发布者时自动切换）
- 订阅管理：`request_id` 作为 `subscription_id`，防止重复订阅
- 频率限制：0.1~60 Hz 范围内裁剪

### 3.11 WebRTCBridge（WebRTC 媒体桥接，可选）

**文件**: `src/webrtc_bridge.cpp` | `include/remote_monitoring/webrtc_bridge.hpp`

- 依赖 `libdatachannel`，编译时通过 `WEBRTC_ENABLED` 宏控制
- 管理多个 `WebRTCPeer` 连接（按 session_id）
- H.264 视频轨道（SendOnly）
- STUN/TURN 服务器可配置
- 订阅 ROS 压缩图像话题，广播给所有连接的 Peer

---

## 4. Proto 接口契约

5 个 proto 文件，包命名空间 `robot.v1`：

| 文件 | 内容 |
|------|------|
| `common.proto` | Vector3, Quaternion, Pose, Twist, Header, RequestBase/ResponseBase, ErrorCode, OperatorLease, Event, Resource, Task, ConnectionQuality |
| `system.proto` | SystemService (Login, Logout, Heartbeat, GetRobotInfo, GetCapabilities, Relocalize, SaveMap, ListMaps, DeleteMap, RenameMap) |
| `control.proto` | ControlService (Lease, SetMode, EmergencyStop, StreamTeleop, Task), RobotMode, SafetyStatus |
| `telemetry.proto` | TelemetryService (StreamFastState, StreamSlowState, StreamEvents, AckEvent), FastState, SlowState |
| `data.proto` | DataService (ListResources, Subscribe, Unsubscribe, DownloadFile, StartCamera, StopCamera, WebRTCSignaling), DataChunk, FileChunk, WebRTCSignal |

**关键契约约束**：
- 幂等控制：所有事务型 RPC 必须携带 `request_id`（UUID v4）
- 租约机制：`lease_token` + TTL
- 事件回放：`event_id` 严格递增 + `last_event_id` 回放
- 遥操作：`timestamp` + `sequence` 用于延迟计算和丢包检测

---

## 5. 配置参数

配置文件：`config/grpc_gateway.yaml`

### 5.1 核心参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `grpc_port` | 50051 | gRPC 服务器端口 |
| `fast_state_hz` | 30.0 | FastState 流频率 |
| `slow_state_hz` | 1.0 | SlowState 流频率 |
| `rate_window_sec` | 2.0 | 话题频率统计窗口 |

### 5.2 ROS 话题映射

| 参数 | 默认值 |
|------|--------|
| `odom_topic` | `/Odometry` |
| `terrain_map_topic` | `/terrain_map` |
| `path_topic` | `/path` |
| `slow_down_topic` | `/slow_down` |
| `data_camera_topic` | `/camera/color/image_raw/compressed` |
| `data_camera_fallback_topic` | `/camera/color/compressed` |
| `data_map_topic` | `/overall_map` |
| `data_pointcloud_topic` | `/cloud_registered` |

### 5.3 TF 坐标系

| 参数 | 默认值 |
|------|--------|
| `tf_map_frame` | `map` |
| `tf_odom_frame` | `odom` |
| `tf_body_frame` | `body` |

### 5.4 Safety Gate

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `deadman_timeout_ms` | 300.0 | 遥操作失联超时 |
| `max_speed` | 1.0 | 最大线速度 (m/s) |
| `max_angular` | 1.0 | 最大角速度 (rad/s) |
| `tilt_limit_deg` | 30.0 | 倾斜保护阈值 (°) |

### 5.5 WebRTC（可选）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `webrtc_enabled` | false | 是否启用 WebRTC |
| `webrtc_offer_timeout_ms` | 3000 | 等待 Offer 超时 |
| `webrtc_start_command` | "" | 启动外部 WebRTC 进程的命令模板 |
| `webrtc_stop_command` | "" | 停止外部 WebRTC 进程的命令模板 |

---

## 6. 构建与运行

### 6.1 依赖安装

```bash
# 方式 1: 从包管理器安装（推荐）
sudo apt-get update
sudo apt-get install -y libgrpc++-dev protobuf-compiler-grpc libprotobuf-dev

# 方式 2: 运行自动安装脚本（从源码编译，需 30+ 分钟）
cd src/remote_monitoring/scripts
chmod +x install_grpc.sh
./install_grpc.sh
```

可选 WebRTC 依赖：
```bash
# 安装 libdatachannel（编译时自动检测）
sudo apt-get install -y libdatachannel-dev
```

### 6.2 构建

```bash
cd /home/sunrise/data/SLAM/navigation
colcon build --packages-select remote_monitoring
source install/setup.bash
```

### 6.3 运行

```bash
# 使用 launch 文件（推荐）
ros2 launch remote_monitoring grpc_gateway.launch.py

# 或直接运行
ros2 run remote_monitoring grpc_gateway
```

### 6.4 WebRTC + H.264 编码部署

机器人端需要安装 `libx264-dev` 并以 `-DX264_ENABLED=ON` 重新编译来启用 H.264 软编码器。

```bash
# 1. 安装 x264 开发库
sudo apt-get update
sudo apt-get install -y libx264-dev

# 2. 安装 libdatachannel (WebRTC)
sudo apt-get install -y libdatachannel-dev
# 如果包管理器没有，从源码编译:
# git clone https://github.com/nickhub/libdatachannel.git
# cd libdatachannel && cmake -B build && cmake --build build && sudo cmake --install build

# 3. 重新编译 remote_monitoring（启用 x264 + WebRTC）
cd /home/sunrise/data/SLAM/navigation
colcon build --packages-select remote_monitoring \
  --cmake-args -DX264_ENABLED=ON

# 4. 验证编译结果
source install/setup.bash
ros2 run remote_monitoring grpc_gateway --ros-args -p webrtc_enabled:=true
# 查看日志确认:
#   [INFO] Found x264: ...
#   [INFO] Found libdatachannel ...
#   [INFO] WebRTC Bridge initialized successfully
```

**编码参数说明** (在 `webrtc_bridge.cpp` 中配置):

| 参数 | 默认值 | 说明 |
|------|--------|------|
| Preset | ultrafast | x264 编码预设（CPU 友好） |
| Tune | zerolatency | 零延迟调优（适合实时传输） |
| Profile | baseline | H.264 Profile（兼容性最好） |
| Bitrate | 1500 kbps | 目标码率 |
| 分辨率 | 跟随输入 | 跟随 ROS 话题图像分辨率 |

**Jetson 硬件编码**（待实现）：

Jetson 平台可替换 x264 为 NVENC 硬件编码器以大幅降低 CPU 占用。
需修改 `webrtc_bridge.cpp` 中的编码器初始化，使用 `nvv4l2h264enc` 或 GStreamer pipeline。

### 6.5 验证

```bash
# 检查端口
ss -tlnp | grep 50051

# 使用 grpcurl 测试
grpcurl -plaintext localhost:50051 list
grpcurl -plaintext localhost:50051 robot.v1.SystemService/GetRobotInfo
grpcurl -plaintext localhost:50051 robot.v1.TelemetryService/StreamFastState
```

---

## 7. 客户端接入

### 7.1 生成 Dart 代码

```bash
# 安装 protoc Dart 插件
dart pub global activate protoc_plugin

# 生成代码
protoc --dart_out=grpc:lib/generated \
  -I src/remote_monitoring/proto \
  common.proto system.proto control.proto telemetry.proto data.proto
```

### 7.2 Flutter pubspec 依赖

```yaml
dependencies:
  grpc: ^3.2.4
  protobuf: ^3.1.0
```

### 7.3 连接示例

```dart
import 'package:grpc/grpc.dart';

// 连接
final channel = ClientChannel(
  '192.168.4.1',  // 机器人 IP
  port: 50051,
  options: ChannelOptions(credentials: ChannelCredentials.insecure()),
);

// 获取机器人信息
final system = SystemServiceClient(channel);
final info = await system.getRobotInfo(Empty());
print('Robot: ${info.robotId}, Version: ${info.firmwareVersion}');

// 订阅快速状态流
final telemetry = TelemetryServiceClient(channel);
await for (var state in telemetry.streamFastState(FastStateRequest(desiredHz: 10.0))) {
  print('Pose: (${state.pose.position.x}, ${state.pose.position.y})');
  print('Speed: ${state.velocity.linear.x} m/s');
  print('RPY: ${state.rpyDeg}');
}

// 遥操作（需要先获取 Lease）
final control = ControlServiceClient(channel);
final leaseResp = await control.acquireLease(AcquireLeaseRequest());
final leaseToken = leaseResp.lease.leaseToken;

final teleopStream = control.streamTeleop();
teleopStream.stream.listen((feedback) {
  print('Actual speed: ${feedback.actualVelocity}');
  print('Safety: ${feedback.safetyStatus.safetyMessage}');
});

teleopStream.sink.add(TeleopCommand()
  ..leaseToken = leaseToken
  ..sequence = 1
  ..targetVelocity = (Twist()
    ..linear = (Vector3()..x = 0.5)
    ..angular = (Vector3()..z = 0.0))
);
```

---

## 8. 代码审查与已知问题

### 8.1 安全性问题

| 级别 | 问题 | 位置 | 建议 |
|------|------|------|------|
| **HIGH** | Login 无真实鉴权，接受任何用户名，密码未校验 | `system_service.cpp:18` | 实现密码校验或 Token 鉴权 |
| **HIGH** | Session Token 可预测（`session_` + 用户名） | `system_service.cpp:25` | 使用加密随机生成 |
| **HIGH** | gRPC 使用 InsecureServerCredentials | `grpc_gateway.cpp:53` | 生产环境需启用 TLS |
| **MED** | StartCamera 使用 `std::system()` 执行外部命令 | `data_service.cpp:496` | 存在命令注入风险，改用 `fork/exec` |
| **MED** | DownloadFile 未做路径遍历防护 | `data_service.cpp:405` | 需检查路径不超出 `file_root_` |

### 8.2 功能缺陷

| 级别 | 问题 | 位置 | 说明 |
|------|------|------|------|
| **HIGH** | AcquireLease 硬编码 holder_id 为 `"client_001"` | `control_service.cpp:84` | 所有客户端被视为同一持有者，Lease 冲突检测失效 |
| **HIGH** | Deadman 检查间隔（1秒）远大于超时阈值（300ms） | `grpc_gateway.cpp:71` | 实际停车延迟可达 1.3 秒，应改为 100ms 检查 |
| **MED** | SlowState 模式硬编码为 `"autonomous"` | `status_aggregator.cpp:173` | 应从 ControlService 的 `current_mode_` 读取 |
| **MED** | SlowState 缺少系统资源数据（CPU/内存/温度/电量） | `status_aggregator.cpp:163-184` | `SystemResource` 字段全部为零 |
| **MED** | SlowDownCallback 为空实现 | `status_aggregator.cpp:81-83` | 从未缓存 slow_down 值 |
| **MED** | Heartbeat RTT 精度为秒级 | `system_service.cpp:56` | 应使用纳秒级精度计算 |
| **LOW** | IdempotencyCache 无主动清理 | `idempotency_cache.hpp` | `Cleanup()` 方法存在但从未被调用，缓存会无限增长 |
| **LOW** | PointCloud2 流缺少元数据 | `data_service.cpp:343-365` | 只发送原始 bytes，客户端无法解析 point fields |

### 8.3 代码质量

| 级别 | 问题 | 位置 | 说明 |
|------|------|------|------|
| **MED** | 使用 `goto cleanup` | `data_service.cpp:1032` | 建议重构为 RAII 或 break + flag |
| **LOW** | WebRTC EncodeFrame 始终返回空 | `webrtc_bridge.cpp:501-514` | 原始帧编码未实现，仅传压缩图像 |
| **LOW** | EventId 注释为 ULID 但实际格式不是 | `common.proto:145` | timestamp_hex + sequence_hex |
| **LOW** | TaskService 为占位实现 | `control_service.cpp:290` | 始终返回 `task_001`，未接入实际调度 |

### 8.4 需要修复的优先项（建议排序）

1. **Deadman 检查频率**：将 `grpc_gateway.cpp` 中的检查间隔从 1s 改为 100ms（安全关键）
2. **AcquireLease holder_id**：从实际 gRPC context 或 Login session 获取客户端身份
3. **SlowState 模式**：接入 ControlService 的真实模式状态
4. **Heartbeat RTT**：改用纳秒级时间戳计算
5. **IdempotencyCache 清理**：在 GrpcGateway 后台循环中定期调用 `Cleanup()`

---

## 9. 实施进度

### Phase 1 - 主干打通 ✅ 已完成

- [x] Proto 接口契约（5 个文件，~750 行）
- [x] gRPC Gateway：4 服务注册、单端口对外
- [x] SystemService：Login / Logout / Heartbeat / GetRobotInfo / GetCapabilities / Relocalize / SaveMap / ListMaps / DeleteMap / RenameMap
- [x] ControlService：Lease / EmergencyStop / StreamTeleop（含幂等缓存）
- [x] TelemetryService：FastState / SlowState / Events（含回放）
- [x] DataService：图像流（gRPC Subscribe 方式）
- [x] Safety Gate：deadman + 限幅 + 倾斜保护
- [x] 构建系统、Launch 文件、配置文件

### Phase 2 - 控制与任务产品化 ⏳ 待完成

- [x] SetMode / StartTask / CancelTask 接入实际 ROS 2 (通过 ModeManager + TaskManager)
- [x] GetActiveWaypoints / ClearWaypoints 航点管理 RPC
- [x] TaskManager 成为唯一 /way_point 发布者 (map→odom tf2 变换)
- [x] 安全联动: E-stop / 模式切换自动挂起/恢复任务
- [x] 操作守卫: StartTask 需 Lease + AUTONOMOUS 模式
- [ ] Lease 续约/抢占规则完善（多客户端）
- [ ] 事件系统与 `/diagnostics` 集成
- [ ] 弱网断连恢复流程（含 teleop 失联停车验证）
- [ ] 真实登录鉴权

### Phase 3 - 视频产品化 ⏳ 待完成

- [ ] WebRTC 推流（Jetson NVENC 硬件编码）
- [ ] DataService 视频降级策略（分辨率→帧率→码率→暂停）
- [ ] 视频档位反馈到 SlowState

### Phase 4 - 数据面与运维 ⏳ 待完成

- [ ] 点云订阅（带 ROI 裁剪、LZ4/Zstd 压缩、max_bitrate 预算约束）
- [ ] 地图/日志分块下载完善
- [ ] 权限系统（viewer / operator / admin）+ 审计日志
- [ ] OTA 升级支持

---

## 10. 文件清单

### Proto 契约（5 个）

```
proto/common.proto       共享类型：向量、位姿、错误码、租约、事件、任务
proto/system.proto       SystemService：登录/登出/心跳/信息/能力/重定位/保存地图/地图管理
proto/control.proto      ControlService：租约/模式/急停/遥操作/任务
proto/telemetry.proto    TelemetryService：快慢状态流/事件流
proto/data.proto         DataService：资源订阅/文件下载/WebRTC信令
```

### 头文件（9 个）

```
include/remote_monitoring/grpc_gateway.hpp            网关入口
include/remote_monitoring/status_aggregator.hpp        状态聚合器
include/remote_monitoring/webrtc_bridge.hpp            WebRTC 桥接（可选）
include/remote_monitoring/core/lease_manager.hpp       租约管理
include/remote_monitoring/core/event_buffer.hpp        事件缓冲
include/remote_monitoring/core/safety_gate.hpp         安全门控
include/remote_monitoring/core/idempotency_cache.hpp   幂等缓存
include/remote_monitoring/services/system_service.hpp  系统服务
include/remote_monitoring/services/control_service.hpp 控制服务
include/remote_monitoring/services/telemetry_service.hpp 遥测服务
include/remote_monitoring/services/data_service.hpp    数据服务
```

### 实现文件（10 个）

```
src/main.cpp                         程序入口
src/grpc_gateway.cpp                 网关核心逻辑
src/status_aggregator.cpp            ROS2 话题 → Protobuf 转换
src/webrtc_bridge.cpp                WebRTC 媒体桥接（条件编译）
src/core/lease_manager.cpp           租约生命周期管理
src/core/event_buffer.cpp            环形事件缓冲 + 回放
src/core/safety_gate.cpp             速度限幅 / deadman / 倾斜保护
src/services/system_service.cpp      会话管理 RPC
src/services/control_service.cpp     控制 RPC + 遥操作流
src/services/telemetry_service.cpp   状态流 + 事件流
src/services/data_service.cpp        资源订阅 + 文件下载 + WebRTC 信令
```

### 配置与部署

```
CMakeLists.txt                       构建系统（支持多 proto 编译 + 可选 WebRTC）
package.xml                          ROS 2 包描述
config/grpc_gateway.yaml             运行时参数配置
launch/grpc_gateway.launch.py        Launch 文件
scripts/install_grpc.sh              gRPC 依赖安装脚本
```

### 代码统计

| 类型 | 行数 |
|------|------|
| Proto | ~750 |
| C++ Header | ~550 |
| C++ Source | ~2500 |
| **总计** | **~3800**（不含生成代码） |
