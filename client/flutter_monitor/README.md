# Robot Monitor — Flutter Client

通过 gRPC 实时监控和遥操作机器人的 Flutter 客户端。

## 功能

- **实时遥测**：位姿、速度、姿态、话题频率、系统资源（10Hz / 1Hz）
- **遥操作控制**：双摇杆、模式切换、紧急停止、租约管理
- **视频流**：WebRTC 实时视频 + JPEG 流回退
- **3D 可视化**：Three.js URDF 机器人模型
- **2D 轨迹**：实时路径 + 全局地图 + 点云叠加
- **事件时间线**：按严重级别着色的事件流 + 确认机制
- **自动重连**：指数退避重连 + 连接健康检查

## 快速开始

### 1. 前置要求

- Flutter SDK >= 3.0.0
- protoc + protoc-gen-dart

```bash
dart pub global activate protoc_plugin
export PATH="$PATH:$HOME/.pub-cache/bin"
```

### 2. 生成 gRPC 代码

```bash
./generate_proto.sh
```

### 3. 安装依赖 & 运行

```bash
flutter pub get
flutter run -d linux     # 桌面端
flutter run -d android   # Android
flutter run -d chrome    # Web
```

### 4. 打包 APK

```bash
flutter build apk --release
# 输出: build/app/outputs/flutter-apk/app-release.apk
```

## 项目结构

```
flutter_monitor/
├── lib/
│   ├── main.dart                           # 入口 + 连接页面
│   ├── screens/
│   │   ├── home_screen.dart                # 主导航（底部Tab + 断连）
│   │   ├── status_screen.dart              # 实时仪表盘
│   │   ├── control_screen.dart             # 遥操作（摇杆 + FPV）
│   │   ├── map_screen.dart                 # 2D/3D 轨迹可视化
│   │   └── events_screen.dart              # 事件时间线
│   ├── services/
│   │   ├── robot_connection_provider.dart   # 集中状态管理 (Provider)
│   │   ├── robot_client_base.dart          # 抽象接口
│   │   ├── robot_client.dart               # gRPC 实现
│   │   ├── mock_robot_client.dart          # Mock 演示
│   │   └── webrtc_client.dart              # WebRTC 信令
│   ├── widgets/
│   │   ├── glass_widgets.dart              # 毛玻璃组件
│   │   ├── camera_stream_widget.dart       # JPEG 相机流
│   │   ├── webrtc_video_widget.dart        # WebRTC 视频
│   │   └── robot_model_widget.dart         # 3D URDF 查看器
│   └── generated/                          # protoc 自动生成 (gitignored)
├── proto/                                  # Protobuf 定义
│   ├── common.proto
│   ├── telemetry.proto
│   ├── control.proto
│   ├── data.proto
│   └── system.proto
├── assets/
│   ├── libs/                               # Three.js + URDFLoader
│   ├── meshes/                             # STL 网格文件
│   ├── urdf/                               # URDF 机器人模型
│   └── urdf_viewer*.html                   # 3D 查看器页面
├── tools/
│   └── test_grpc_client.dart               # gRPC 连接测试工具
├── generate_proto.sh                       # Proto 代码生成脚本
└── pubspec.yaml                            # 依赖配置
```

## 架构

### 状态管理

使用 `Provider` + `ChangeNotifier` 集中管理：

```
RobotConnectionProvider (ChangeNotifier)
├── 连接状态 (connected / reconnecting / error)
├── FastState / SlowState 广播流
├── 租约状态
├── 自动重连 (指数退避: 2s → 4s → 8s → 30s)
└── 连接健康检查 (5s 轮询)
```

### gRPC 服务

| 服务 | 用途 |
|------|------|
| `TelemetryService` | FastState (10Hz), SlowState (1Hz), Events |
| `ControlService` | Lease, SetMode, EmergencyStop, Teleop |
| `DataService` | 资源订阅 (Camera/Map/PointCloud), WebRTC 信令 |
| `SystemService` | GetRobotInfo, GetCapabilities |

### 性能优化

- `setState` 节流：FastState UI 更新限 10FPS，Map 位姿限 5Hz
- `RepaintBoundary`：隔离高频更新组件
- `AutomaticKeepAliveClientMixin`：Tab 切换保持屏幕状态
- `shouldRepaint` 精确判断：数据版本号 + 坐标比较
- Teleop 命令限 20Hz
- 事件列表限 200 条 + 虚拟滚动

## 配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| Robot IP | `192.168.66.190` | 连接页面可修改 |
| gRPC Port | `50051` | 连接页面可修改 |
| FastState Hz | `10.0` | 快速状态流频率 |
| Max Path Points | `5000` | 轨迹最大点数 |
| Max Events | `200` | 事件列表最大条数 |
| Reconnect Max | `10` 次 | 最大重连尝试 |

## 故障排查

```bash
# 检查机器人服务
ss -tlnp | grep 50051

# 测试 gRPC 连接
grpcurl -plaintext <ROBOT_IP>:50051 list

# 运行 gRPC 测试脚本
dart run tools/test_grpc_client.dart <ROBOT_IP> 50051

# Flutter 依赖问题
flutter clean && flutter pub get
```

## 技术栈

Flutter + gRPC + Protobuf + WebRTC + Three.js + Provider
