# Changelog

所有重要的项目变更都记录在这里。

---

## [Unreleased] — 2026-02-11

### 运行参数在线配置 (Runtime Config Live Tuning)

Flutter App 到导航系统的完整参数配置通道打通，130+ 参数实时可调。

#### 通信链路

- **robot_client_base.dart**: 新增 `getRuntimeConfig()` / `setRuntimeConfig()` 抽象接口
- **robot_client.dart**: 通过 gRPC `SystemServiceClient` 发起真实 RPC 调用 (5s 超时)
- **mock_robot_client.dart**: mock 实现保证离线开发不报错
- **runtime_config_gateway.dart**: `fetchFromServer()` / `pushToServer()` 从 STUB 替换为真实 gRPC 调用，JSON 编解码 + `ErrorCode` 校验 + `SharedPreferences` 本地缓存
- **main.dart**: 新增 `SlowState` → `RuntimeConfigGateway.updateFromHeartbeat()` 监听器，心跳自动回读配置版本

#### Proto 存根

- **system.pb.dart**: +4 个消息类 (`GetRuntimeConfigRequest/Response`, `SetRuntimeConfigRequest/Response`)，使用 JSON 字符串传输
- **system.pbgrpc.dart**: +`getRuntimeConfig()` / `setRuntimeConfig()` 客户端 + 服务端方法
- **telemetry.pb.dart**: `SlowState` +`configJson` (field 20) + `configVersion` (field 21)

#### 配置模型

- **runtime_config.dart**: ~130 个可配字段 (16 个分组: Motion / Safety / Geofence / PathFollow / Navigation / Terrain / LocalPlanner / SLAM / Localizer / GlobalPlanner / Patrol / Telemetry / Features / Geometry / Driver / HealthMonitor)
- **runtime_params_page.dart**: 专用配置 UI — TabBar 分组 + Slider + 数值直接输入对话框 + 即时推送

---

## [Unreleased] — 2026-02-08

### UI 全局极简风格统一 (Minimalist Design Overhaul)

参考 Cursor IDE 设计语言，对全部页面统一应用极简、清爽的视觉风格。

#### 设计原则
- **去除所有 boxShadow**：卡片、按钮、FAB、导航栏全面移除阴影
- **统一 borderRadius: 10**：所有卡片容器使用 `BorderRadius.circular(10)`，按钮用 `8`
- **单色图标**：所有装饰性图标统一使用 `context.subtitleColor`，尺寸缩小 (18-20)
- **细边框**：`Border.all(color: context.borderColor)` 替代阴影和渐变
- **文字层级**：标题用 `context.titleColor`，副标题/标签用 `context.subtitleColor`
- **按钮简化**：`ElevatedButton` 替换为 `TextButton` + 细边框，去除强色彩
- **移除渐变背景**：`LinearGradient`、`BackdropFilter` 毛玻璃等重装饰效果清理
- **subtle 选中态**：选中状态使用极低透明度背景 (`0.04-0.08`) 而非强色块
- **废弃参数清理**：移除不再使用的 `color` / `iconColor` 组件参数

#### 已完成的页面

| 页面文件 | 修改内容 |
|---------|---------|
| `home_screen.dart` | ProfileCard 去阴影+图标简化；快速操作按钮去 color 参数 |
| `main_shell_screen.dart` | 底部导航栏去阴影、图标 `titleColor` 替代 `primary` |
| `robot_detail_screen.dart` | QuickStats 卡片去阴影+加边框；FeatureCard 移除 color 传参；模式/进度条单色化 |
| `map_screen.dart` | MapFab 去阴影+小圆角；设置弹窗缩小精简；ThemeOption/IconButton 去主色调 |
| `control_screen.dart` | FPV 徽章去主色调；E-Stop 去发光阴影；摇杆简化；Dog 按钮去 color 参数 |
| `status_screen.dart` | 连接徽章简化、呼吸灯移除、所有卡片去阴影、RateChip 纯文本化 |
| `events_screen.dart` | 事件数量徽章简化、事件卡片去阴影 |
| `scan_screen.dart` | TabBar 单色化、设备列表去阴影+图标简化、按钮统一细边框、进度条/WiFi 信号单色 |
| `splash_screen.dart` | 移除渐变背景、Logo 简化为单色图标 |
| `ble_control_screen.dart` | 所有卡片去阴影+统一边框；状态磁贴去 color 参数；延迟徽章单色化；移除 `_batteryColor` |
| `robot_select_screen.dart` | 移除毛玻璃效果、实色卡片+细边框、选中指示器单色化 |
| `app_settings_screen.dart` | 连接超时选项选中态 `titleColor` 替代 `primary` |
| `alert_history_page.dart` | 告警条目去阴影、左侧色条改统一边框、筛选图标统一颜色 |
| `firmware_ota_page.dart` | `AppRadius.card` 全局替换为 `10` |
| `log_export_page.dart` | 所有容器去阴影+统一边框、导出按钮简化、快捷芯片去阴影 |
| `version_detail_page.dart` | 信息卡去阴影、Logo 简化、复制按钮改 TextButton |
| `support_page.dart` | 支持卡片去阴影+去 iconColor 参数；复制按钮统一 |
| `saved_devices_page.dart` | 设备卡去阴影、默认徽章文本化、菜单图标改水平 |
| `feature_card.dart` (shared) | FeatureCard 的 `color` 改为可选；SettingsTile 默认色去 primary；图标统一灰色 |
| `robot_card.dart` (shared) | 去除 boxShadow、borderRadius 统一 10 |
| `robot_model_widget.dart` | Loading spinner 改为 `subtitleColor` + 细线宽 |

### 导航系统修复

- **新增 `/map-select-goal` 路由**：修复 TaskPanel 中"从地图选择目标点"无法跳转的 bug
- **新增 `map_goal_picker.dart`**：地图选点页面，支持点击地图选择 NavigationGoal 并返回

### 任务与导航系统 (Task & Navigation System)

#### Proto 定义
- 新增 `NavigationGoal`、`MappingConfig`、`TaskConfig` 等结构化消息
- 新增 `StartTaskRequest/Response`、`PauseTask`、`CancelTask`、`GetTaskStatus` RPC
- 新增 `SaveMap`、`LoadMap`、`DeleteMap`、`ListMaps`、`RenameMap` 地图管理 RPC
- Proto 已推送至 `Robot_Proto.git` 远程仓库

#### Flutter 前端
- 新增 `task_panel.dart`：任务控制面板（导航/建图/巡检任务类型切换、航点管理、任务进度监控）
- 新增 `map_manager_page.dart`：地图管理页面（列表/保存/加载/重命名/删除地图、重定位对话框）
- 新增 `map_goal_picker.dart`：地图选点页面（点击地图选取目标航点）
- `robot_client_base.dart` / `robot_client.dart` / `mock_robot_client.dart`：扩展 gRPC 客户端接口

#### C++ 后端 (remote_monitoring)
- `TaskManager`：航点导航任务管理、建图任务生命周期
- `DataServiceImpl`：地图文件 CRUD 操作（基于 `std::filesystem`）
- `ControlServiceImpl`：`StartTask`、`PauseTask`、`CancelTask`、`GetTaskStatus` RPC 实现

### 其他改进
- BLE 控制页面完善
- 固件 OTA 页面重新设计
- 机器人型号选择页面卡片化重构
- 底部导航栏悬浮设计
- APP 信息从主页移至设置页

---

## [0.1.0] — 2026-01 (Initial)

### 初始版本
- 基础 gRPC 通信框架
- 实时状态监控（CPU、内存、电池、温度）
- 双摇杆遥控控制（支持 Lease 机制）
- WebRTC/gRPC 视频流
- 网络扫描 + 蓝牙连接
- 机器人型号管理
- 事件日志显示
- 地图轨迹可视化
- 3D URDF 模型渲染
