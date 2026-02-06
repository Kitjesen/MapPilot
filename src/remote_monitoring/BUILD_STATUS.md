# 构建状态与依赖

## 当前状态

**全部代码已实现并构建成功（含 WebRTC + 导航栈集成）**

最后验证时间：2026-02-06  
验证结果：构建通过，无警告

## 系统集成状态

### 已完成的集成

- [x] **SafetyGate → /cmd_vel**: 遥操作命令直达 robot_driver（可配置话题名）
- [x] **EmergencyStop → /stop**: 急停信号同步发送到 pathFollower
- [x] **ModeManager**: 模式切换协调导航栈（TELEOP/AUTONOMOUS/IDLE/ESTOP/MAPPING）
- [x] **Deadman 模式感知**: 仅在 TELEOP 模式下触发 deadman 超时
- [x] **StartTask → /goal_pose**: 导航任务发布目标点到 PCT 全局规划器
- [x] **CancelTask**: 取消任务并停止 pathFollower
- [x] **Relocalize**: 通过 gRPC 调用 /relocalize ROS2 服务（重定位）
- [x] **SaveMap**: 通过 gRPC 调用 /save_map 或 /pgo/save_maps 服务
- [x] **丰富遥测**: 订阅 /cmd_vel, /pct_planner/status, /way_point, /pct_path, /cloud_registered
- [x] **定位检测**: 周期性调用 /relocalize_check 检查定位状态
- [x] **SlowState 扩展**: 包含 active_task, navigation_status（含规划器状态、航点、路径长度、定位状态）
- [x] **DataService 扩展**: terrain_map_ext 作为可订阅资源
- [x] **配置化**: 所有话题名、robot_id、firmware_version 均可通过 YAML 配置

## 依赖要求

### 必需依赖
- protobuf-compiler (protoc)
- libprotobuf-dev
- libgrpc++-dev
- protobuf-compiler-grpc (grpc_cpp_plugin)
- interface（同 workspace 的 SLAM 接口包）

### 可选依赖（WebRTC）
- libdatachannel 0.24.1（从源码编译安装到 /usr/local）
- 安装后 CMake 自动检测并启用 WEBRTC_ENABLED

## 安装步骤

### 选项 1：从包管理器安装（推荐，快速）

```bash
sudo apt-get update
sudo apt-get install -y libgrpc++-dev protobuf-compiler-grpc libprotobuf-dev
```

如果包管理器中没有 gRPC（旧版 Ubuntu），使用选项 2。

### 安装 libdatachannel（可选，启用 WebRTC）

```bash
cd /tmp
git clone --recurse-submodules https://github.com/paullouisageneau/libdatachannel.git
cd libdatachannel && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DNO_TESTS=ON -DNO_EXAMPLES=ON
make -j$(nproc)
sudo make install && sudo ldconfig
```

### 选项 2：运行自动安装脚本

```bash
cd src/remote_monitoring/scripts
./install_grpc.sh
```

## 构建命令

安装依赖后：

```bash
cd /home/sunrise/data/SLAM/navigation
source /opt/ros/humble/setup.bash
colcon build --packages-select interface remote_monitoring
source install/setup.bash
```

## 运行

```bash
ros2 launch remote_monitoring grpc_gateway.launch.py
```

或直接运行：

```bash
ros2 run remote_monitoring grpc_gateway
```

## 验证

检查 gRPC 服务是否监听：

```bash
# 检查端口
ss -tlnp | grep 50051

# 使用 grpcurl 测试（需要安装 grpcurl）
grpcurl -plaintext localhost:50051 list
```

## 配置参数

所有参数在 `config/grpc_gateway.yaml` 中配置：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `grpc_port` | 50051 | gRPC 端口 |
| `robot_id` | "robot_001" | 机器人 ID |
| `cmd_vel_topic` | "/cmd_vel" | 速度命令输出话题 |
| `stop_topic` | "/stop" | 停止信号话题 |
| `goal_pose_topic` | "/goal_pose" | 导航目标话题 |
| `odom_topic` | "/Odometry" | 里程计话题 |
| `terrain_map_topic` | "/terrain_map" | 地形分析话题 |
| `planner_status_topic` | "/pct_planner/status" | 全局规划器状态话题 |
| `way_point_topic` | "/way_point" | 航点话题 |
| `deadman_timeout_ms` | 300.0 | 遥操作超时 (ms) |
| `max_speed` | 1.0 | 最大速度 (m/s) |

完整参数列表见 `config/grpc_gateway.yaml`。

## 已完成的功能

- Proto 契约定义（5 个文件，含 Relocalize/SaveMap 扩展）
- TelemetryService（fast/slow/event 三流，含导航状态扩展）
- SystemService（Login/Heartbeat/Capabilities/Relocalize/SaveMap）
- ControlService（Lease/Mode/EmergencyStop/StreamTeleop + 任务管理）
- DataService（ListResources/Subscribe/Unsubscribe/DownloadFile/StartCamera/StopCamera + terrain 资源）
- Safety Gate（deadman + 限幅 + 模式感知）
- ModeManager（模式切换协调导航栈）
- 统一 gRPC Gateway
- WebRTC Bridge（libdatachannel 集成，SDP/ICE 信令通道）
- 导航栈完整集成（cmd_vel/stop/goal_pose/relocalize/save_map）

## 待完成（Phase 2-4 剩余）

- [ ] 弱网断连恢复流程
- [ ] WebRTC 实际视频帧推流（需 H.264 编码器）
- [ ] 视频降级策略
- [ ] 点云/地图压缩（LZ4/Zstd）
- [ ] 事件持久化与 /diagnostics 集成
- [ ] 权限与审计日志
