# MapPilot Monitor — Flutter Client

<p align="center">
  <img src="assets/app_icon.png" alt="MapPilot" width="120"/>
</p>

通过 gRPC 实时监控、遥操作和管理机器人的跨平台 Flutter 客户端。

## 功能概览

| 页面 | 功能 | 说明 |
|------|------|------|
| **Status** | 实时遥测 | 位姿、速度、姿态、话题频率、系统资源 (10Hz / 1Hz) |
| **Control** | 遥操作 | 双摇杆、模式切换、紧急停止、租约管理、FPV 视频 |
| **Map** | 轨迹可视化 | 2D/3D 地图、实时路径、全局地图、点云叠加 |
| **Events** | 事件时间线 | 按严重级别着色的事件流 + 确认机制 |
| **Settings** | 设置与管理 | 连接信息、文件管理 (OTA)、云端更新、关于 |

### Settings 页面详细功能

- **连接信息** — 实时显示 CPU / 内存 / 温度 / 电池 / 运行模式
- **文件管理 (OTA 部署)** — 从手机上传模型/地图/配置/固件到机器人
  - 4 种分类 (模型 · 地图 · 配置 · 固件)，可切换目标目录
  - 浏览机器人远程目录、上传文件、删除文件
  - 分块传输 + 实时进度条
- **云端更新 (Cloud OTA)** — 从 GitHub Releases 获取最新版本并一键部署
  - 自动查询 Release 列表 + 资产文件
  - 按扩展名识别文件类型 (`.pt` → 模型, `.pcd` → 地图...)
  - 两阶段进度: 云端下载 → 部署到机器人
  - 可配置云端源 (GitHub 仓库 / 自定义 URL)

---

## 快速开始

### 前置要求

- Flutter SDK >= 3.0.0
- 机器人端运行 `remote_monitoring` gRPC 服务 (端口 50051)

### 安装 & 运行

```bash
# 安装依赖
flutter pub get

# 运行 (选择平台)
flutter run -d linux     # 桌面端
flutter run -d android   # Android
flutter run -d chrome    # Web
```

### 打包 APK

```bash
flutter build apk --release
# 输出: build/app/outputs/flutter-apk/app-release.apk
```

> APK 编译需要 x86_64 环境。ARM64 设备请使用 GitHub Actions 自动构建。

### 安装 APK

**方式一：GitHub Release (推荐)**

从 [Releases](https://github.com/Kitjesen/MapPilot/releases/latest) 下载最新 `MapPilot-*.apk`。

**方式二：App 内云端更新**

Settings → 云端更新 → 检查 → 下载 APK。

---

## 项目结构

```
flutter_monitor/
├── lib/
│   ├── main.dart                           # 入口 + 连接页面
│   ├── screens/
│   │   ├── home_screen.dart                # 主导航 (底部 Tab)
│   │   ├── status_screen.dart              # 实时仪表盘
│   │   ├── control_screen.dart             # 遥操作 (摇杆 + FPV)
│   │   ├── map_screen.dart                 # 2D/3D 轨迹可视化
│   │   ├── events_screen.dart              # 事件时间线
│   │   └── settings_screen.dart            # 设置 + 文件管理 + 云端 OTA
│   ├── services/
│   │   ├── robot_connection_provider.dart   # 集中状态管理 (Provider)
│   │   ├── robot_client_base.dart          # 抽象接口
│   │   ├── robot_client.dart               # gRPC 实现
│   │   ├── mock_robot_client.dart          # Mock 演示
│   │   ├── cloud_ota_service.dart          # 云端 OTA (GitHub Releases API)
│   │   └── webrtc_client.dart              # WebRTC 信令
│   └── widgets/
│       ├── glass_widgets.dart              # 毛玻璃组件
│       ├── camera_stream_widget.dart       # JPEG 相机流
│       ├── webrtc_video_widget.dart        # WebRTC 视频
│       └── robot_model_widget.dart         # 3D URDF 查看器
├── assets/
│   ├── app_icon.png                        # 应用图标
│   ├── libs/                               # Three.js + URDFLoader
│   ├── meshes/                             # STL 机器人网格
│   └── urdf/                               # URDF 机器人模型
├── android/                                # Android 平台配置
├── linux/                                  # Linux 平台配置
├── web/                                    # Web 平台配置
└── pubspec.yaml                            # 依赖配置
```

---

## 架构

### 状态管理

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
| `ControlService` | Lease, SetMode, EmergencyStop, Teleop, StartTask, CancelTask |
| `DataService` | 资源订阅, 文件下载/上传, 目录管理, WebRTC 信令 |
| `SystemService` | GetRobotInfo, GetCapabilities, Relocalize, SaveMap |

### 云端 OTA 数据流

```
GitHub Releases (云端)
  │  ① HTTP GET /repos/.../releases
  ▼
MapPilot App (手机)
  │  ② HTTP 下载资产到内存
  │  ③ gRPC UploadFile (分块上传)
  ▼
机器人 (/home/sunrise/models/)
```

### 性能优化

- `setState` 节流 — FastState UI 限 10FPS, Map 位姿限 5Hz
- `RepaintBoundary` — 隔离高频更新组件
- `AutomaticKeepAliveClientMixin` — Tab 切换保持屏幕状态
- Teleop 命令限 20Hz
- 事件列表限 200 条

---

## 配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| Robot IP | `192.168.66.190` | 连接页面可修改 |
| gRPC Port | `50051` | 连接页面可修改 |
| FastState Hz | `10.0` | 快速状态流频率 |
| Max Path Points | `5000` | 轨迹最大点数 |
| Max Events | `200` | 事件列表最大条数 |
| Cloud Source | `Kitjesen/MapPilot` | 云端更新 GitHub 仓库 |

---

## 故障排查

```bash
# 检查机器人 gRPC 服务
ss -tlnp | grep 50051

# 测试 gRPC 连接
grpcurl -plaintext <ROBOT_IP>:50051 list

# 运行 gRPC 测试脚本
dart run tools/test_grpc_client.dart <ROBOT_IP> 50051

# 依赖问题
flutter clean && flutter pub get
```

---

## 技术栈

Flutter · gRPC · Protobuf · WebRTC · Three.js · Provider · GitHub Releases API
