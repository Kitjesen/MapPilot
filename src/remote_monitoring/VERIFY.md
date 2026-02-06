# 通信验证计划：remote_monitoring ↔ flutter_monitor

## 目标

验证机器人端 gRPC Gateway（`remote_monitoring`）与 Flutter 客户端（`flutter_monitor`）的端到端通信。覆盖 4 个 gRPC 服务的核心 RPC，确认数据正确流通。

---

## 前置条件

### 机器人端（ROS 2 环境）

| 项目 | 要求 | 检查命令 |
|------|------|---------|
| ROS 2 | Humble / Iron | `ros2 --version` |
| gRPC C++ | libgrpc++-dev | `dpkg -l \| grep grpc` |
| Protobuf | libprotobuf-dev + protoc | `protoc --version` |
| colcon | 构建工具 | `colcon --version` |

### 客户端（开发机）

| 项目 | 要求 | 检查命令 |
|------|------|---------|
| Flutter | >= 3.0 | `flutter --version` |
| protoc-gen-dart | Dart proto 插件 | `which protoc-gen-dart` |
| 网络 | 与机器人同一网段 | `ping <ROBOT_IP>` |

---

## Phase 0：构建 & 启动

### 0.1 机器人端：编译 remote_monitoring

```bash
# 在工作空间根目录
cd ~/data/SLAM/navigation    # 或你的实际工作空间路径

# 安装依赖（首次）
sudo apt-get install -y libgrpc++-dev protobuf-compiler-grpc libprotobuf-dev

# 编译
colcon build --packages-select remote_monitoring
source install/setup.bash
```

**验证编译成功：**
```bash
# 应该能找到可执行文件
ls install/remote_monitoring/lib/remote_monitoring/grpc_gateway
```

**预期结果：** 文件存在，无编译错误

### 0.2 机器人端：启动 gRPC Gateway

**方式 A：通过 launch 文件（推荐）**
```bash
ros2 launch remote_monitoring grpc_gateway.launch.py
```

**方式 B：直接运行节点**
```bash
ros2 run remote_monitoring grpc_gateway \
  --ros-args --params-file src/remote_monitoring/config/grpc_gateway.yaml
```

**预期输出：**
```
[INFO] [grpc_gateway]: gRPC Gateway listening on :50051
[INFO] [grpc_gateway]: Safety Gate initialized (deadman=300ms, max_speed=1.0)
```

### 0.3 机器人端：验证端口监听

```bash
ss -tlnp | grep 50051
```

**预期结果：** 看到 `LISTEN ... *:50051`

### 0.4 客户端：网络连通性

```bash
ping <ROBOT_IP>                       # 基础连通
nc -zv <ROBOT_IP> 50051               # TCP 端口可达
```

**预期结果：** ping 通，50051 端口可连

---

## Phase 1：SystemService 验证

> 最基础的服务，确认 gRPC 框架层面完全跑通。

### 1.1 grpcurl 测试（机器人端或客户端均可）

```bash
# 安装 grpcurl（如果还没有）
# Ubuntu: sudo snap install grpcurl
# 或: go install github.com/fullstorydev/grpcurl/cmd/grpcurl@latest

# 列出所有服务
grpcurl -plaintext <ROBOT_IP>:50051 list

# 列出 SystemService 方法
grpcurl -plaintext <ROBOT_IP>:50051 list robot.v1.SystemService

# 调用 GetRobotInfo
grpcurl -plaintext <ROBOT_IP>:50051 robot.v1.SystemService/GetRobotInfo

# 调用 GetCapabilities
grpcurl -plaintext <ROBOT_IP>:50051 robot.v1.SystemService/GetCapabilities
```

**验证清单：**
- [ ] `list` 返回 4 个服务：SystemService, ControlService, TelemetryService, DataService
- [ ] `GetRobotInfo` 返回 robot_id, display_name, firmware_version 等字段
- [ ] `GetCapabilities` 返回 supported_resources, teleop_supported 等字段
- [ ] 无 gRPC 错误码（UNAVAILABLE / UNIMPLEMENTED 等）

### 1.2 Flutter 客户端连接

1. 在开发机运行 Flutter 应用：
   ```bash
   cd client/flutter_monitor
   flutter run -d linux    # 或 android / chrome
   ```
2. 在连接页面输入机器人 IP 和端口 50051
3. 点击"连接机器人"

**验证清单：**
- [ ] 连接成功，跳转到 HomeScreen
- [ ] 无连接错误弹窗
- [ ] 状态栏显示 "ONLINE"（绿色）

---

## Phase 2：TelemetryService 验证

> 核心只读数据流，最高频率的通信通道。

### 2.1 FastState 流（10Hz → StatusScreen）

**前置：** 确保机器人端有 `/Odometry` 话题发布。

```bash
# 检查话题是否存在
ros2 topic list | grep -i odom
ros2 topic hz /Odometry
```

**如果还没有真实话题，可用模拟数据：**
```bash
# 发布模拟 Odometry（另开终端）
ros2 topic pub /Odometry nav_msgs/msg/Odometry '{
  header: {frame_id: "odom"},
  pose: {pose: {position: {x: 1.5, y: 2.3, z: 0.1}}},
  twist: {twist: {linear: {x: 0.5}, angular: {z: 0.2}}}
}' -r 10
```

**Flutter 端验证（StatusScreen）：**
- [ ] 位姿卡片显示 X, Y, YAW 数值且持续更新
- [ ] MOTION 卡片显示 Linear / Angular 速度
- [ ] 数值与发布的 Odometry 数据一致
- [ ] 更新频率合理（不卡顿、不闪烁）

### 2.2 SlowState 流（1Hz → StatusScreen）

SlowState 包含系统资源数据（由 gateway 从 `/proc` 读取 CPU/Memory/Temp）。

**Flutter 端验证（StatusScreen）：**
- [ ] BATTERY 卡片显示数值
- [ ] CPU 卡片显示数值（应与机器人实际 CPU 大致一致）
- [ ] TEMP 卡片显示数值
- [ ] VOLTAGE 卡片显示数值
- [ ] TOPIC RATES 卡片显示 Odom / Lidar / Map 频率

### 2.3 SlowState 话题频率验证

要让 Topic Rates 显示非零值，需要有对应话题在发布：

```bash
# 查看相关话题
ros2 topic hz /Odometry        # → odom_hz
ros2 topic hz /terrain_map     # → terrain_map_hz（如果有）
ros2 topic hz /path            # → path_hz（如果有）
```

**Flutter 端验证：**
- [ ] 有发布的话题，对应 Hz 数值 > 0
- [ ] 没有发布的话题，显示 0.0 Hz（而非崩溃）

### 2.4 EventStream（5s 间隔 → EventsScreen）

目前 Gateway 可能没有主动产生事件。可在机器人端手动触发：

```bash
# 查看是否有 diagnostics 话题
ros2 topic list | grep diag
```

**Flutter 端验证（EventsScreen）：**
- [ ] 页面能正常打开（即使无事件也不崩溃）
- [ ] 如有事件：显示时间线 + 严重级别标签 + 时间戳
- [ ] ACK 按钮可点击，不报错

---

## Phase 3：ControlService 验证

> 控制面验证，涉及租约和遥操作。

### 3.1 Lease（租约）获取与释放

1. Flutter → HomeScreen → 点击 "Control" Tab → 进入 ControlScreen
2. 点击右上角 "NO LEASE" 按钮

**验证清单：**
- [ ] 点击后变为 "LEASE ACTIVE"（绿色）
- [ ] 机器人端日志显示 lease acquired
- [ ] 再次点击 → 变回 "NO LEASE"
- [ ] 机器人端日志显示 lease released

### 3.2 遥操作（Teleop Stream）

**前置：** 获取 Lease 成功。

1. 在 ControlScreen 拖动左/右摇杆

**机器人端验证：**
```bash
# 另开终端监听安全门控输出
ros2 topic echo /cmd_vel_safe
```

**验证清单：**
- [ ] 拖动左摇杆 → `/cmd_vel_safe` 收到 linear.x / linear.y 变化
- [ ] 拖动右摇杆 → `/cmd_vel_safe` 收到 angular.z 变化
- [ ] 松开摇杆 → 速度归零
- [ ] 底部状态栏显示 Vx / Vy / Wz 实时数值

### 3.3 模式切换

1. 在 ControlScreen 中间，点击 IDLE / MANUAL / AUTO 按钮

**验证清单：**
- [ ] 点击后弹出 SnackBar："模式已切换: ..."
- [ ] StatusScreen 的 Mode 字段更新
- [ ] 机器人端日志显示模式变更

### 3.4 紧急停止

1. 在 ControlScreen 点击红色 "STOP" 按钮

**验证清单：**
- [ ] 弹出红色 SnackBar："紧急停止已触发"
- [ ] `/cmd_vel_safe` 立即归零
- [ ] 有触觉反馈（真机上）

### 3.5 Deadman 超时验证

1. 获取 Lease，开始遥操作
2. 突然关闭 Flutter App（或断开 Wi-Fi）

**机器人端验证：**
- [ ] 300ms 后 `/cmd_vel_safe` 归零（Safety Gate 接管）
- [ ] 日志显示 deadman timeout

---

## Phase 4：DataService 验证

> 大数据流：相机、点云、地图。

### 4.1 相机流（Camera → ControlScreen FPV）

**前置：** 确保相机话题在发布。

```bash
ros2 topic hz /camera/color/image_raw/compressed
# 如果没有，检查备用话题
ros2 topic hz /camera/color/compressed
```

**Flutter 端验证（ControlScreen）：**
- [ ] 背景显示相机画面（而非黑色占位符）
- [ ] 画面持续更新（不冻结）
- [ ] 如无相机 → 显示 "No Camera Feed" 占位（不崩溃）

### 4.2 地图 & 点云（Map/PointCloud → MapScreen）

```bash
# 检查相关话题
ros2 topic hz /overall_map
ros2 topic hz /cloud_registered
```

**Flutter 端验证（MapScreen 2D 模式）：**
- [ ] 如有 `/overall_map` → 2D 视图显示灰色全局地图点
- [ ] 如有 `/cloud_registered` → 显示红色局部点云
- [ ] 机器人在移动时，轨迹线持续绘制
- [ ] 罗盘方向随机器人朝向旋转

### 4.3 3D 模型视图

**Flutter 端验证（MapScreen 3D 模式）：**
- [ ] 切换到 3D 模式：能看到 URDF 机器人模型加载
- [ ] 机器人位姿随遥测数据更新
- [ ] 加载无崩溃

---

## Phase 5：连接弹性验证

> 验证断线重连、Provider 状态管理。

### 5.1 WiFi 断开 → 自动重连

1. Flutter 已连接且显示正常数据
2. 关闭机器人 WiFi 或拔网线（模拟断网）

**验证清单：**
- [ ] HomeScreen 顶部出现橙色横幅 "正在重新连接..."
- [ ] StatusScreen 状态变为 "RECONNECTING"（橙色）

3. 恢复网络

- [ ] 自动重连成功（无需手动操作）
- [ ] 横幅消失，状态恢复 "ONLINE"
- [ ] 数据流恢复更新

### 5.2 gRPC Gateway 重启

1. Flutter 已连接
2. 在机器人端 Ctrl+C 停止 `grpc_gateway`

**验证清单：**
- [ ] Flutter 显示重连状态
- [ ] 机器人端重新启动 `grpc_gateway`
- [ ] Flutter 自动恢复连接

### 5.3 断开按钮

1. 点击导航栏的电源按钮

**验证清单：**
- [ ] 弹出确认对话框
- [ ] 确认后回到连接页面
- [ ] 机器人端 lease 正确释放

---

## Phase 6：Mock 模式对照

> 不需要机器人，快速验证 Flutter 客户端 UI 完整性。

1. 启动 Flutter → 连接页面 → 点击 "Mock 演示模式"

**验证清单：**
- [ ] StatusScreen：位姿圆周运动，Battery/CPU/Temp 动态变化
- [ ] ControlScreen：摇杆可操作，Lease 可获取
- [ ] MapScreen：轨迹画出圆形路径
- [ ] EventsScreen：每 5s 生成一条模拟事件

---

## 问题排查速查

| 现象 | 可能原因 | 排查命令 |
|------|---------|---------|
| Flutter 连接超时 | 端口未监听 / 防火墙 | `ss -tlnp \| grep 50051` |
| 连接成功但无数据 | ROS 话题未发布 | `ros2 topic list` + `ros2 topic hz <topic>` |
| FastState 有数据但 SlowState 无 | SlowState 1Hz 延迟 | 等几秒；检查 `/proc/stat` 读取权限 |
| 遥操作无响应 | 未获取 Lease | 检查 ControlScreen 右上角状态 |
| `/cmd_vel_safe` 无输出 | SafetyGate 问题 | 检查 gateway 日志中 safety 相关打印 |
| 相机画面黑屏 | 相机话题未发布 / 话题名不匹配 | `ros2 topic info /camera/color/image_raw/compressed` |
| grpcurl 报 UNIMPLEMENTED | proto 服务未注册 | 检查 gateway 启动日志是否有 4 个 service |
| 编译失败 grpc 找不到 | 依赖未安装 | `bash scripts/install_grpc.sh` |

---

## 验证结果记录

| Phase | 测试项 | 通过 | 备注 |
|-------|--------|------|------|
| 0 | 编译成功 | | |
| 0 | 端口监听 | | |
| 0 | 网络连通 | | |
| 1 | grpcurl 服务列表 | | |
| 1 | GetRobotInfo | | |
| 1 | Flutter 连接成功 | | |
| 2 | FastState 数据流 | | |
| 2 | SlowState 数据流 | | |
| 2 | EventStream | | |
| 3 | Lease 获取/释放 | | |
| 3 | 遥操作 → cmd_vel | | |
| 3 | 模式切换 | | |
| 3 | 紧急停止 | | |
| 3 | Deadman 超时 | | |
| 4 | 相机流 | | |
| 4 | 地图/点云 | | |
| 4 | 3D 模型 | | |
| 5 | WiFi 断连重连 | | |
| 5 | Gateway 重启重连 | | |
| 5 | 断开按钮 | | |
| 6 | Mock 模式全流程 | | |
