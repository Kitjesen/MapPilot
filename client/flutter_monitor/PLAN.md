# Plan — Flutter Monitor 后续开发计划

> 最后编辑时间：2026-02-08
> 状态：UI 极简风格统一 **已完成**，废弃参数清理 **已完成**，导航路由修复 **已完成**

---

## 1. UI 样式统一 — ✅ 已完成

### 1.1 `AppColors.primary` 残留（全部为合理语义色，已评估确认保留）

| 文件 | 数量 | 用途 | 处理 |
|------|------|------|------|
| `theme.dart` | 3 | 颜色 token 定义 | ✅ 保留 |
| `main.dart` | 1 | 告警 SnackBar INFO 级别 | ✅ 保留（语义色） |
| `firmware_ota_page.dart` | 8 | 固件状态/进度指示 | ✅ 保留（功能语义色） |
| `events_screen.dart` | 1 | 事件严重级别映射 | ✅ 保留（语义色） |
| `status_screen.dart` | 4 | CPU/运动数据可视化 | ✅ 保留（数据语义色） |

### 1.2 `boxShadow` 残留（全部合理保留）

| 文件 | 数量 | 用途 | 处理 |
|------|------|------|------|
| `theme.dart` | 1 | AppShadows token 定义 | ✅ 保留 |
| `glass_widgets.dart` | 4 | GlassCard 组件（FPV 叠加） | ✅ 保留（功能性） |

### 1.3 废弃参数清理 — ✅ 已完成

- [x] `_QuickActionButton.color` 移除（home_screen.dart）
- [x] `_SupportCard.iconColor` 移除（support_page.dart）
- [x] `_buildDogButton.color` 移除（control_screen.dart）
- [x] `_statusTile.color` 移除（ble_control_screen.dart）
- [x] `FeatureCard.color` 改为可选（feature_card.dart + robot_detail_screen.dart）
- [x] `SettingsTile.iconColor` 默认值改为 `Colors.grey`

---

## 2. 导航系统 — ✅ 已修复

### 路由表（完整）

```
/                → SplashScreen        (启动动画)
/main            → MainShellScreen     (底部 Tab 主容器)
/scan            → ScanScreen          (网络/蓝牙扫描)
/robot-detail    → RobotDetailScreen   (已连接机器人详情)
/settings        → AppSettingsScreen   (设置页)
/control         → ControlScreen       (FPV 遥控)
/robot-select    → RobotSelectScreen   (型号选择)
/task-panel      → TaskPanel           (任务控制面板)
/map-manager     → MapManagerPage      (地图管理)
/map-select-goal → MapGoalPicker       (地图选点) ← NEW
```

### 导航流程

```
SplashScreen → MainShellScreen
  ├─ Tab 0: HomeScreen
  │    ├─→ /scan → /robot-detail → ControlScreen / StatusScreen / MapScreen
  │    ├─→ /robot-select
  │    ├─→ /task-panel → /map-select-goal (选点返回)
  │    └─→ /map-manager
  ├─ Tab 1: StatusScreen
  ├─ Tab 2: MapScreen → /task-panel, /map-manager
  ├─ Tab 3: EventsScreen
  └─ Tab 4: (通过 RobotDetailScreen 跳转到 Settings)
AppSettingsScreen → 各子页面 (SavedDevices, AlertSettings, Firmware, Log, Support, Version)
ScanScreen → BleControlScreen (BLE 连接)
```

---

## 3. 功能开发 [下一阶段]

### 3.1 任务与导航系统 — 前后端联调
- [ ] `TaskPanel` gRPC 调用联调测试（StartTask、PauseTask、CancelTask、GetTaskStatus）
- [ ] `MapManagerPage` 地图 CRUD 联调测试（ListMaps、SaveMap、LoadMap、DeleteMap、RenameMap）
- [ ] `MapGoalPicker` 选点交互优化（长按确认、拖拽微调）
- [ ] 航点列表编辑器（拖拽排序、添加/删除航点）
- [ ] 建图任务配置界面（SLAM 算法选择、分辨率参数）
- [ ] 任务进度实时更新（gRPC streaming 替代轮询）

### 3.2 C++ 后端 (remote_monitoring) 补全
- [ ] `TaskManager` 与 ROS 2 Action Server 对接（nav2）
- [ ] `ModeManager` 状态机完善（MAPPING 模式切换守卫）
- [ ] `SafetyGate` 遥控安全检查实现
- [ ] 地图文件格式支持扩展（PCD + PGM + YAML）

### 3.3 BLE 功能完善
- [ ] BLE OTA 固件升级流程
- [ ] BLE 数据通道扩展（IMU 原始数据流）
- [ ] 断线自动重连策略

---

## 4. 接续说明（给下一个聊天会话）

### 上下文
本项目是一个 Flutter 机器人监控 APP，通过 gRPC 与 ROS 2 后端通信，支持：
- 实时遥测（CPU、电池、温度、IMU、关节）
- 双摇杆遥控 + WebRTC FPV
- 地图可视化 + 3D URDF 模型
- 任务管理（导航/建图/巡检）
- BLE 直连狗板控制

### 设计规范 (Design Tokens)
```
borderRadius: 10 (卡片), 8 (按钮/小组件)
boxShadow: 无 (全部移除，仅 GlassCard 保留)
图标: context.subtitleColor, 尺寸 16-20
标题: context.titleColor
副标题: context.subtitleColor
边框: context.borderColor (1px)
选中态背景: isDark ? white@0.07 : black@0.04
按钮: TextButton + BorderSide(color: context.borderColor)
语义色保留: AppColors.primary 仅用于数据可视化/状态指示器
```

### 主题扩展属性 (来自 theme.dart)
```dart
context.isDark        // bool
context.titleColor    // 标题文字色
context.subtitleColor // 副标题/图标色
context.borderColor   // 边框色
context.surfaceColor  // 卡片背景
context.cardColor     // 卡片背景 (RobotCard 使用)
context.dividerColor  // 分割线色
```

### 关键文件
- `lib/app/theme.dart` — 主题定义 + BuildContext 扩展
- `lib/shared/widgets/feature_card.dart` — FeatureCard / SettingsSection / SettingsTile / SettingsActionButton
- `lib/shared/widgets/glass_widgets.dart` — GlassCard (仅 FPV 控制界面使用)
- `lib/shared/widgets/robot_card.dart` — RobotCard (设备卡片)
- `lib/core/grpc/robot_client_base.dart` — gRPC 客户端抽象接口
- `lib/core/providers/robot_connection_provider.dart` — 连接状态管理
- `lib/features/map/map_goal_picker.dart` — 地图选点页面

### 从哪里继续
→ 直接进入 **§3 功能开发阶段**：任务导航联调
