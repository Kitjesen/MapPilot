# Navigation System - Agent Guide

## 1. 系统概述

本项目是一个完整的**野外/越野自主导航系统**，包含从感知到远程监控的完整链路：

- **SLAM**: Fast-LIO2 实时建图 + PGO 回环优化
- **定位**: Localizer 重定位模块（支持预加载地图）
- **感知**: 地形分析（地面估计、障碍物检测、可穿越性分析）
- **规划**: 全局规划（PCT_planner）+ 局部规划（base_autonomy）
- **驱动**: 机器人底盘控制接口
- **远程监控**: gRPC Gateway（`src/remote_monitoring`）— 任务管理、遥操作、安全门限、OTA
- **客户端**: Flutter Monitor（`client/flutter_monitor`）— 地图可视化、任务编排、多模式控制

---

## 2. 系统架构图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              感知层 (Perception)                              │
├─────────────────────────────────────────────────────────────────────────────┤
│  Livox LiDAR  ──▶  livox_ros_driver2  ──▶  /livox/lidar, /livox/imu        │
└─────────────────────────────────────────────────────────────────────────────┘
                                       │
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              SLAM层 (SLAM & Localization)                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐       │
│  │   Fast-LIO2     │────▶│  sensor_scan    │────▶│  local_planner  │       │
│  │   (实时里程计)   │     │  _generation    │     │  (局部规划输入)  │       │
│  │                 │     │  (坐标转换)      │     │                 │       │
│  │ 输出:           │     │                 │     │                 │       │
│  │ /cloud_reg      │     │ 输出:            │     │                 │       │
│  │ /cloud_map      │     │ /sensor_scan    │     │                 │       │
│  │ /Odometry       │     │                 │     │                 │       │
│  └────────┬────────┘     └─────────────────┘     └─────────────────┘       │
│           │                                                                 │
│           │  /cloud_map                                                     │
│           ▼                                                                 │
│  ┌─────────────────┐     优化后位姿    ┌─────────────────┐                 │
│  │      PGO        │◄────────────────▶│    Localizer    │                 │
│  │  (回环检测优化)  │                  │   (重定位模块)   │                 │
│  │                 │                  │                 │                 │
│  │ 服务:           │                  │ 服务:           │                 │
│  │ /pgo/save_maps  │                  │ /relocalize     │                 │
│  └─────────────────┘                  │ /relocalize_check│                │
│                                       └─────────────────┘                 │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                       │
                                       │ TF: map → odom
                                       │
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           地形分析层 (Terrain Analysis)                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐       │
│  │ terrain_analysis│────▶│terrain_analysis_│────▶│  local_planner  │       │
│  │  (地面估计)      │     │     ext         │     │  (局部规划)      │       │
│  │                 │     │  (连通性检查)    │     │                 │       │
│  │ 输入:           │     │                 │     │                 │       │
│  │ /cloud_reg      │     │ 输入:           │     │                 │       │
│  │ /Odometry       │     │ /terrain_map    │     │                 │       │
│  │                 │     │                 │     │                 │       │
│  │ 输出:           │     │ 输出:           │     │                 │       │
│  │ /terrain_map    │     │ /terrain_map_ext│     │                 │       │
│  └─────────────────┘     └─────────────────┘     └─────────────────┘       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                       │
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           规划层 (Planning)                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                        全局规划 (PCT_planner)                        │    │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐             │    │
│  │  │  tomography │───▶│  Tomogram   │───▶│   Global    │             │    │
│  │  │  (层析成像)  │    │  (多层地图)  │    │  Planner    │             │    │
│  │  │             │    │             │    │  (A* + G)   │             │    │
│  │  └─────────────┘    └─────────────┘    └──────┬──────┘             │    │
│  │                                               │                     │    │
│  │         SceneTrav 可穿越性配置                 │                     │    │
│  │                                               ▼                     │    │
│  │                                      ┌─────────────┐                │    │
│  │                                      │ pct_adapters│                │    │
│  │                                      │ (ROS2接口)   │                │    │
│  │                                      └─────────────┘                │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                        局部规划 (base_autonomy)                      │    │
│  │                                                                     │    │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐             │    │
│  │  │local_planner│───▶│pathFollower │───▶│robot_driver │             │    │
│  │  │ (路径选择)   │    │ (路径跟踪)   │    │ (底盘控制)   │             │    │
│  │  │             │    │             │    │             │             │    │
│  │  │ 输入:        │    │             │    │             │             │    │
│  │  │ /terrain_map│    │             │    │             │             │    │
│  │  │ /way_point  │    │             │    │             │             │    │
│  │  │ /joy        │    │             │    │             │             │    │
│  │  └─────────────┘    └─────────────┘    └─────────────┘             │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                       │
                                       │ /way_point, /Odometry
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                     远程监控层 (Remote Monitoring — gRPC Gateway)             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐  ┌────────────┐  │
│  │ControlService │  │ TelemetryServ │  │  DataService  │  │ SystemServ │  │
│  │  (任务/遥操作) │  │  (状态推送)    │  │  (文件传输)    │  │  (系统管理) │  │
│  └──────┬────────┘  └──────┬────────┘  └───────────────┘  └────────────┘  │
│         │                  │                                               │
│  ┌──────▼────────┐  ┌──────▼────────┐  ┌───────────────┐                 │
│  │ TaskManager   │  │  SafetyGate   │  │  EventBuffer  │                 │
│  │ (航点调度+状态) │  │  (限速+避障)   │  │  (事件持久化)  │                 │
│  └───────────────┘  └───────────────┘  └───────────────┘                 │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                       │
                                       │ gRPC (Protobuf)
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                     客户端层 (Flutter Monitor App)                           │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐  ┌────────────┐  │
│  │  MapScreen    │  │  TaskPanel    │  │ StatusScreen  │  │ ControlScr │  │
│  │  (地图+任务)   │  │  (高级任务)    │  │  (遥测监控)    │  │  (遥操作)   │  │
│  └──────┬────────┘  └───────────────┘  └───────────────┘  └────────────┘  │
│         │                                                                   │
│  ┌──────▼────────┐  ┌───────────────┐  ┌───────────────┐                 │
│  │  TaskGateway  │  │  MapGateway   │  │ServiceGateway │                 │
│  │  (任务生命周期) │  │  (地图管理)    │  │  (服务编排)    │                 │
│  └───────────────┘  └───────────────┘  └───────────────┘                 │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. 启动顺序和依赖关系

### 3.1 启动顺序

```
阶段1: 基础驱动 (必须最先启动)
├── livox_ros_driver2          # LiDAR驱动
└── robot_driver               # 底盘驱动

阶段2: SLAM核心 (依赖驱动)
├── fastlio2                   # 实时里程计
└── sensor_scan_generation     # 点云坐标转换

阶段3: 高级SLAM (依赖SLAM核心)
├── pgo                        # 回环检测与优化 (可选，用于建图)
└── localizer                  # 重定位模块 (运行时必需)

阶段4: 感知与规划 (依赖SLAM)
├── terrain_analysis           # 地形分析
├── terrain_analysis_ext       # 扩展地形分析
└── local_planner              # 局部规划器

阶段5: 全局规划 (独立运行)
└── pct_planner                # 全局规划 (需预建地图)
```

### 3.2 依赖关系图

```
livox_ros_driver2
    │
    ├──▶ fastlio2
    │       │
    │       ├──▶ sensor_scan_generation
    │       │       └──▶ local_planner ◀── pct_adapters ◀── pct_planner
    │       │                              (闭环跟踪)
    │       ├──▶ pgo (可选)
    │       │       └──▶ TF: map → odom
    │       │
    │       └──▶ localizer
    │               └──▶ TF: map → odom
    │
    └──▶ terrain_analysis
            └──▶ terrain_analysis_ext
                    └──▶ local_planner

离线处理 (建图后):
PCD地图文件 ──▶ tomography ──▶ Tomogram (.pickle) ──▶ pct_planner (运行时加载)
```

### 3.3 启动命令示例

#### 建图模式 (首次探索)
```bash
# 终端1: 驱动 + SLAM
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fastlio2 lio_launch.py

# 终端2: 回环优化 (可选但推荐)
ros2 launch pgo pgo_launch.py

# 保存地图
ros2 service call /save_map interface/srv/SaveMaps "{file_path: '/path/to/map.pcd'}"
```

#### 离线处理 (生成Tomogram)
```bash
# 将保存的 map.pcd 复制到 PCT_planner/rsc/pcd/
cd src/global_planning/PCT_planner

# 运行层析成像 (生成 .pickle 文件)
python3 tomography/scripts/tomography.py --scene Common
# 输出: rsc/tomogram/Common.pickle
```

#### 运行模式 (定位+规划)
```bash
# 终端1: 驱动 + SLAM
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fastlio2 lio_launch.py

# 终端2: 重定位 (加载预建地图)
ros2 launch localizer localizer_launch.py
ros2 service call /relocalize interface/srv/Relocalize \
  "{pcd_path: '/path/to/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

# 终端3: 地形分析和局部规划
ros2 launch terrain_analysis terrain_analysis.launch
ros2 launch terrain_analysis_ext terrain_analysis_ext.launch
ros2 launch local_planner local_planner.launch

# 终端4: 全局规划 + 路径适配
ros2 launch PCT_planner planner_only_launch.py  # 或 python3 planner/scripts/global_planner.py
ros2 run pct_adapters pct_path_adapter

# 设置目标点 (通过RViz或命令行)
ros2 topic pub /way_point geometry_msgs/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 10.0, y: 0.0, z: 0.0}}"
```

---

## 4. 坐标系定义

### 4.1 坐标系层级

```
map (全局地图坐标系) - 固定不变的世界参考系
 └── odom (里程计坐标系) - 由PGO或Localizer发布 ← 感知和路径适配在这里
     └── body (机器人本体坐标系) - Fast-LIO2输出 ← 局部规划和控制在这里
         └── lidar (激光雷达坐标系) - 传感器原始坐标系
```

**重要**: 
- **感知层** (terrain_analysis) 在 **odom 坐标系**下工作
- **局部规划** (local_planner) 在 **vehicle(body) 坐标系**下工作
- **全局规划** (pct_planner) 在 **map 坐标系**下工作

### 4.2 坐标系详细说明

| 坐标系 | 名称 | 定义 | 发布者 | 备注 |
|--------|------|------|--------|------|
| **map** | 地图坐标系 | 全局固定坐标系 | PGO/Localizer | 经过回环优化或重定位的统一坐标系 |
| **odom** | 里程计坐标系 | 局部平滑坐标系 | PGO/Localizer | 与map的偏移量实时更新，**感知层工作坐标系** |
| **body** | 本体坐标系 | IMU/传感器中心 | Fast-LIO2 | **局部规划和控制坐标系**，机器人永远在原点 |
| **lidar** | 雷达坐标系 | LiDAR传感器中心 | - | 通过外参与body关联 |
| **sensor_at_scan** | 扫描时刻坐标系 | 历史扫描位置 | sensor_scan_generation | 用于点云对齐 |

**重要**: 代码中的 `vehicleX_` 虽然名为 vehicle，但实际是**底盘中心在 odom 系的位置**（已考虑传感器偏移）

### 4.3 TF变换关系

```
map → odom: 由 PGO 或 Localizer 发布
  - PGO: 基于回环检测的偏移量 (m_r_offset, m_t_offset)
  - Localizer: 基于ICP配准的偏移量 (last_offset_r, last_offset_t)

odom → body: 由 Fast-LIO2 发布 (/Odometry)
  - 实时高频更新 (100Hz)
  - 包含位置和姿态

body → lidar: 静态变换 (外参)
  - 通过配置文件 lio.yaml 中的 r_il, t_il 定义
```

### 4.4 关键配置参数

```yaml
# src/slam/fastlio2/config/lio.yaml
body_frame: "body"      # 机器人本体坐标系
world_frame: "odom"     # 里程计坐标系

# src/slam/pgo/config/pgo.yaml
map_frame: "map"        # 全局地图坐标系
local_frame: "odom"     # 局部坐标系

# src/slam/localizer/config/localizer.yaml
map_frame: "map"
local_frame: "odom"
```

---

## 5. 任务编排流水线 (Task Orchestration)

### 5.0 端到端调度链路

从用户在 Flutter 客户端点击"启动任务"到机器人执行航点导航，经历以下层次：

```
┌─ Flutter UI ──────────────────────────────────────────────────────────┐
│  1. GetActiveWaypoints → 查看当前航点 (来源: App/Planner/无)           │
│  2. ClearWaypoints → 清除旧航点 (可选)                                │
│  3. AcquireLease → 获取操作租约 (必须)                                │
│  4. SetMode(AUTONOMOUS) → 切换到自主模式 (必须)                        │
│  5. StartTask → 发送航点任务                                          │
└────────────────────────────┬──────────────────────────────────────────┘
                             │ gRPC
                             ▼
┌─ C++ ControlService ──────────────────────────────────────────────────┐
│  守卫检查: Lease 有效 ∧ Mode == AUTONOMOUS                            │
│  → 组装 core::TaskParams { type, waypoints, loop, max_speed }         │
│  → task_manager_->StartTask(params)                                   │
└────────────────────────────┬──────────────────────────────────────────┘
                             │ 内部调度
                             ▼
┌─ C++ TaskManager (唯一 /way_point 发布者) ────────────────────────────┐
│                                                                       │
│  航点来源 (互斥, APP 优先):                                             │
│    ① App 任务 → 航点列表按序下发                                        │
│    ② 全局规划器 → /planner_waypoint 透传 (App 空闲时)                   │
│                                                                       │
│  坐标变换: map → odom (tf2), 确保 local_planner 收到 odom 坐标         │
│  到达判定: /Odometry 距离 < arrival_radius → 下一航点                    │
│  超时保护: 单航点 > waypoint_timeout_sec (默认300s) → FAILED            │
│  循环巡检: loop=true 时末尾航点后重置                                    │
│  进度回调: → EventBuffer → gRPC → App                                  │
│                                                                       │
│  安全联动 (由 ModeManager 通知):                                        │
│    E-stop / 切 TELEOP / 切 IDLE → 自动挂起任务                         │
│    回 AUTONOMOUS → 自动恢复 (仅系统挂起的, 用户手动暂停不受影响)           │
│                                                                       │
│  ClearWaypoints → /stop=2 (立即停车) + 发布当前位置航点                  │
└────────────────────────────┬──────────────────────────────────────────┘
                             │ /way_point (PointStamped, odom 坐标系)
                             ▼
┌─ ROS2 局部规划 ───────────────────────────────────────────────────────┐
│  local_planner → /path → pathFollower → /cmd_vel → robot_driver       │
└───────────────────────────────────────────────────────────────────────┘
```

### 5.0.0 航点管理数据流

```
┌──────────────────┐      /pct_path        ┌──────────────────┐
│   pct_planner    │ ──────────────────────▶│  pct_path_adapter│
│   (全局规划)      │                       │  (路径→航点)      │
└──────────────────┘                       └────────┬─────────┘
                                                    │ /planner_waypoint
                                                    │ (odom 坐标系)
                                                    ▼
┌──────────────────┐   StartTask (gRPC)   ┌─────────────────────┐
│  Flutter App     │ ────────────────────▶│    TaskManager      │
│  (map 坐标系)    │                      │  (唯一发布者)        │
│                  │◀────────────────────│  map→odom tf2 变换   │
│  GetActiveWaypoints / ClearWaypoints   │  优先级: APP > PLAN  │
└──────────────────┘                      └────────┬────────────┘
                                                   │ /way_point (odom)
                                                   ▼
                                          ┌─────────────────────┐
                                          │   local_planner     │
                                          │  goalX_/goalY_ 直取  │
                                          │  (odom 坐标系)       │
                                          └─────────────────────┘
```

### 5.0.0a 航点坐标系约定

| 来源 | 输入坐标系 | 变换 | 输出坐标系 | 说明 |
|------|-----------|------|-----------|------|
| **App 航点** | `map` | TaskManager 内部 tf2 `map→odom` | `odom` | Flutter 侧统一使用 map 坐标 |
| **Planner 航点** | `odom` | 无 (直通) | `odom` | `pct_path_adapter` 已完成 `map→odom` |
| **`/way_point` 发布** | — | — | `odom` | `local_planner` 消费时总是 odom 坐标 |
| **GetActiveWaypoints 返回** | — | — | `odom` | 后端直接返回当前 odom 坐标的航点 |

> **关键约定**: `local_planner` 直接读取 `/way_point` 的 `x, y` 作为 odom 下目标位置，**不做任何额外变换**。所有坐标变换在 TaskManager 入口处完成。

### 5.0.1 TaskMode 与 TaskType 映射

| Flutter `TaskMode` | Proto `TaskType`           | 说明                    |
|---------------------|----------------------------|------------------------|
| `navigation`        | `TASK_TYPE_NAVIGATION`     | 单点/多点导航            |
| `mapping`           | `TASK_TYPE_MAPPING`        | 建图（无航点，自由移动）  |
| `patrol`            | `TASK_TYPE_INSPECTION`     | 巡检（多航点 + loop=true）|

### 5.0.2 参数透传现状

| UI 参数               | Proto 字段                          | 状态  |
|------------------------|-------------------------------------|-------|
| `_speedLimit`          | `NavigationParams.max_speed`        | ✅ 已接通 |
| `_missionNameCtrl`     | `NavigationGoal.label` / `MappingParams.map_name` | ✅ 已接通 |
| `_priority`            | —                                   | ⏳ TODO(protocol) |
| `_obstacleOverride`    | —                                   | ⏳ TODO(protocol) |

### 5.0.3 关键源文件索引

| 层级 | 文件 | 职责 |
|------|------|------|
| Flutter UI | `client/.../features/map/map_screen.dart` | TaskMode 枚举、任务发起、地图交互 |
| Flutter Gateway | `client/.../core/gateway/task_gateway.dart` | 任务生命周期封装、状态轮询 |
| Proto 定义 | `src/robot_proto/proto/control.proto` | StartTaskRequest/NavigationParams/MappingParams |
| gRPC 服务 | `src/remote_monitoring/src/services/control_service.cpp` | 请求解析、TaskParams 组装 |
| 任务调度 | `src/remote_monitoring/include/.../core/task_manager.hpp` | TaskParams 结构体、航点调度 |
| 任务执行 | `src/remote_monitoring/src/core/task_manager.cpp` | 到达判定、循环巡检、进度回调 |

---

## 6. 话题和服务列表

### 6.1 核心话题 (Topics)

#### 输入话题

| 话题名 | 类型 | 发布者 | 订阅者 | 说明 |
|--------|------|--------|--------|------|
| `/livox/lidar` | CustomMsg | livox_ros_driver2 | fastlio2 | LiDAR原始点云 |
| `/livox/imu` | Imu | livox_ros_driver2 | fastlio2 | IMU数据 |
| `/joy` | Joy | 手柄驱动 | local_planner | 手动控制输入 |
| `/way_point` | PointStamped | **TaskManager** (唯一) | local_planner | 目标航点 (odom 坐标系) |
| `/planner_waypoint` | PointStamped | pct_path_adapter | TaskManager | 全局规划器航点 (odom 坐标系) |
| `/speed` | Float32 | 速度控制 | local_planner | 速度指令 |
| `/pct_path` | Path | pct_planner | pct_path_adapter, TaskManager | 全局路径 (map 坐标系) |

#### 输出话题

| 话题名 | 类型 | 发布者 | 订阅者 | 说明 |
|--------|------|--------|--------|------|
| `/cloud_registered` | PointCloud2 | fastlio2 | sensor_scan_generation, PGO, Localizer | Body系点云 (body坐标系) |
| `/cloud_map` | PointCloud2 | fastlio2 | terrain_analysis, terrain_analysis_ext, local_planner | 世界地图点云 (odom坐标系) ✅ |
| `/Odometry` | Odometry | fastlio2 | 所有模块 | 里程计信息 |
| `/terrain_map` | PointCloud2 | terrain_analysis | terrain_analysis_ext, local_planner | 地形地图 (odom坐标系) ✅ |
| `/terrain_map_ext` | PointCloud2 | terrain_analysis_ext | local_planner | 扩展地形地图 (odom坐标系) ✅ |
| `/path` | Path | local_planner | pathFollower | 规划路径 (body坐标系) ✅ |
| `/free_paths` | PointCloud2 | local_planner | RViz | 可视化可用路径 (body坐标系) ✅ |
| `/pct_path` | Path | pct_planner | pct_path_adapter, TaskManager | 全局规划路径 (map 坐标系) |
| `/planner_waypoint` | PointStamped | pct_path_adapter | TaskManager | 规划器当前航点 (odom 坐标系) |
| `/way_point` | PointStamped | **TaskManager** (唯一) | local_planner | 当前目标航点 (odom 坐标系) |
| `/slow_down` | Int8 | local_planner | pathFollower | 减速指令 (0-3级) |
| `/stop` | Int8 | local_planner, ModeManager, SafetyGate, TaskManager | pathFollower | 紧急停止 (0=clear, 1=lin, 2=full) |

#### TF变换

| 变换 | 发布者 | 频率 | 说明 |
|------|--------|------|------|
| `map → odom` | PGO/Localizer | 10-20Hz | 全局到局部坐标系 |
| `odom → body` | fastlio2 | 100Hz | 实时位姿 |
| `map → sensor_at_scan` | sensor_scan_generation | 与点云同步 | 扫描时刻坐标系 |

### 6.2 服务 (Services)

| 服务名 | 类型 | 提供者 | 说明 |
|--------|------|--------|------|
| `/relocalize` | Relocalize | Localizer | 加载地图并重定位 |
| `/relocalize_check` | IsValid | Localizer | 检查重定位状态 |
| `/pgo/save_maps` | SaveMaps | PGO | 保存优化后的地图和位姿 |
| `/save_map` | SaveMaps | fastlio2 | 保存当前地图 |

### 6.3 gRPC 服务 (Remote Monitoring)

`src/remote_monitoring` 通过 gRPC 将 ROS2 能力暴露给远程客户端。

Proto 定义在 `src/robot_proto/proto/control.proto`:

| RPC | 请求 | 响应 | 说明 |
|-----|------|------|------|
| `StartTask` | `StartTaskRequest` | `StartTaskResponse` | 启动任务（需 Lease + AUTONOMOUS 模式） |
| `CancelTask` | `CancelTaskRequest` | `CancelTaskResponse` | 取消当前任务（需 Lease） |
| `PauseTask` | `PauseTaskRequest` | `PauseTaskResponse` | 暂停任务 |
| `ResumeTask` | `ResumeTaskRequest` | `ResumeTaskResponse` | 恢复任务 |
| `GetTaskStatus` | `GetTaskStatusRequest` | `GetTaskStatusResponse` | 查询任务进度 |
| `GetActiveWaypoints` | `GetActiveWaypointsRequest` | `GetActiveWaypointsResponse` | 查询当前活跃航点 (来源/列表/进度) |
| `ClearWaypoints` | `ClearWaypointsRequest` | `ClearWaypointsResponse` | 清除航点并停车（需 Lease） |
| `SetMode` | `SetModeRequest` | `SetModeResponse` | 切换模式 (IDLE/MANUAL/TELEOP/AUTONOMOUS/MAPPING) |
| `EmergencyStop` | `EmergencyStopRequest` | `EmergencyStopResponse` | 急停 (同时挂起 TaskManager) |
| `StreamTeleop` | `stream TeleopCommand` | `stream TeleopFeedback` | 双向遥操作流 |
| `AcquireLease` | `AcquireLeaseRequest` | `AcquireLeaseResponse` | 获取操作租约 |

**StartTaskRequest 结构化参数**:

```protobuf
message StartTaskRequest {
  RequestBase base = 1;
  TaskType task_type = 2;                    // NAVIGATION / MAPPING / INSPECTION / FOLLOW_PATH
  string params_json = 3;                    // 向后兼容 JSON
  NavigationParams navigation_params = 4;    // 航点列表 + loop + max_speed
  MappingParams mapping_params = 5;          // 地图名 + 自动保存
  FollowPathParams follow_path_params = 6;   // 路径跟随
}
```

**SafetyGate (安全门限)**:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `max_speed` | 1.0 m/s | 全局最大线速度（任务可覆盖） |
| `obstacle_height_thre` | 0.2 m | 障碍物高度阈值 |
| `stop_distance` | 0.8 m | 急停距离 |
| `slow_distance` | 2.0 m | 减速距离 |

### 6.4 局部规划模块详解 (base_autonomy)

#### local_planner (路径选择器)

**功能**: 基于地形分析结果，从预定义路径库中选择最优路径，并发布减速指令

**路径库结构**:
```
36方向 × 343条 = 12,348条候选路径
    │
    ├── 36个方向 (每10°一个，覆盖360°)
    └── 每个方向7组不同曲率路径 (pathList_ 0-6)
```

**碰撞检测机制**:

使用预计算的 `correspondences_` 映射表实现 O(1) 查询：

```cpp
// 预计算：启动时加载
std::vector<int> correspondences_[gridVoxelNum_];
// 含义：网格ind被哪些路径穿过

// 运行时：遍历障碍物
for (each obstacle in terrain_map) {
    int ind = point_to_grid(obstacle);  // 转网格索引
    for (int path_id : correspondences_[ind]) {  // 查表！
        if (h > obstacleHeightThre_) {
            clearPathList_[path_id]++;  // 标记阻塞
        } else {
            pathPenaltyList_[path_id] = max(h);  // 记录惩罚
        }
    }
}
```

**减速分级逻辑**:

| 减速级别 | 触发条件 | 含义 | 速度比例 |
|---------|---------|------|---------|
| 1 | `penaltyScore > 0.15` | 地形代价高(大障碍物/陡坡) | 25% |
| 2 | `penaltyScore > 0.10` | 地形代价中等 | 50% |
| 3 | 可选路径<5且偏离中心 | 路径选择受限 | 75% |
| 0 | 其他 | 地形良好 | 100% |

**penaltyScore计算**:
```cpp
penaltyScore = pathPenaltyPerGroupScore_[group] / clearPathPerGroupNum_[group]
             = 平均障碍物高度 (米)
```

#### pathFollower (路径跟踪器)

**功能**: Pure Pursuit 路径跟踪，接收减速指令调整速度

**订阅话题**:
| 话题 | 类型 | 说明 |
|------|------|------|
| `/path` | Path | 选定的局部路径 |
| `/Odometry` | Odometry | 当前位姿 |
| `/joy` | Joy | 手动控制输入 |
| `/slow_down` | Int8 | 减速指令 (1=25%, 2=50%, 3=75%) |
| `/stop` | Int8 | 紧急停止 |

**核心参数**:
```cpp
lookAheadDis_ = baseLookAheadDis_ + lookAheadRatio_ * currentSpeed;  // 自适应前瞻
slowRate1_ = 0.25;  // 一级减速比例
slowRate2_ = 0.50;  // 二级减速比例
slowRate3_ = 0.75;  // 三级减速比例
```

---

### 6.5 全局规划模块详解

#### tomography (离线地图处理)

**功能**: 将PCD点云地图转换为多层可穿越性地图(Tomogram)

**工作流程**:
```
输入: PCD文件 (如 scans.pcd)
   ↓
处理: 层析成像 + 可穿越性分析 (SceneTrav参数)
   ↓
输出: 
   1. Tomogram文件 (.pickle) - 供pct_planner加载
   2. 可视化话题 (用于RViz查看)
```

**启动命令**:
```bash
# 离线处理 (在PCT_planner目录下)
python3 tomography/scripts/tomography.py --scene Common
# 或
python3 tomography/scripts/tomography.py --scene Stairs

# 可视化 (ROS2节点)
ros2 run pct_planner tomography --scene Common
```

**输出文件**:
- `rsc/tomogram/{scene_name}.pickle` - 多层地图数据

**可视化话题**:
| 话题 | 说明 |
|------|------|
| `/global_points` | 原始点云 (用于对齐) |
| `/layer_G_0` ~ `/layer_G_N` | 各层地面高度 |
| `/layer_C_0` ~ `/layer_C_N` | 各层可穿越性代价 |
| `/tomogram` | 合并的可视化点云 |

**配置参数** (`tomography/config/scene_*.py`):
```python
class SceneTrav():
    kernel_size = 7          # 可穿越性分析核大小
    interval_min = 0.50      # 最小层高间隔
    interval_free = 0.65     # 自由空间层高间隔
    slope_max = 0.36         # 最大坡度
    step_max = 0.20          # 最大台阶高度
    standable_ratio = 0.20   # 可站立比例阈值
    cost_barrier = 50.0      # 障碍代价值
    safe_margin = 0.4        # 安全边界
    inflation = 0.2          # 膨胀系数
```

---

#### pct_planner (全局规划器)

**功能**: 基于Tomogram进行全局路径规划

**输入**: 
- Tomogram文件 (.pickle)
- 起点/终点坐标

**输出**: `/pct_path` (Path消息，map坐标系)

**规划算法**: A* + G-value引导

---

#### pct_adapters (路径适配器)

**功能**: 桥接全局规划和局部规划，实现闭环跟踪

**核心逻辑**:
```
订阅 /pct_path (全局路径, map坐标系)
订阅 /Odometry (当前位姿, odom坐标系)
   ↓
TF变换: map → odom
   ↓
路径下采样 → 航点序列
   ↓
闭环控制: 到达当前航点才发布下一航点
   ↓
发布 /planner_waypoint (当前目标航点, odom坐标系)
   ↓
TaskManager 透传到 /way_point (App 任务空闲时)
```

**注意**: pct_path_adapter 不再直接发布 `/way_point`，而是发布到
`/planner_waypoint`，由 TaskManager 统一管理，避免与 App 下发的航点冲突。

**参数**:
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `waypoint_distance` | 0.5 | 航点间距 (米) |
| `arrival_threshold` | 0.5 | 到达判定距离 (米) |
| `lookahead_dist` | 1.0 | 前瞻距离 (米) |

**启动命令**:
```bash
ros2 run pct_adapters pct_path_adapter
# 或
ros2 launch pct_adapters pct_adapter.launch.py
```

**为什么需要这个模块**:
- 全局规划器发布的是完整路径，局部规划器只需要当前目标点
- 实现"胡萝卜"引导机制，确保机器人沿全局路径 corridor 行驶
- 允许局部规划器在航点间自主避障，同时保持全局方向

---

### 6.6 服务详细定义

#### `/relocalize` (interface/srv/Relocalize)

**Request:**
```
string pcd_path    # 地图文件路径
float32 x          # 初始位置 X (m)
float32 y          # 初始位置 Y (m)
float32 z          # 初始位置 Z (m)
float32 yaw        # 偏航角 (rad)
float32 pitch      # 俯仰角 (rad)
float32 roll       # 横滚角 (rad)
```

**Response:**
```
bool success       # 是否成功
string message     # 返回信息
```

**调用示例:**
```bash
ros2 service call /relocalize interface/srv/Relocalize \
  "{pcd_path: '/path/to/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

---

## 7. 工程化改进建议

### 7.1 已完成

| # | 改进项 | 状态 |
|---|--------|------|
| 1 | 远程监控 gRPC Gateway（遥操作、任务管理、安全门限） | ✅ `src/remote_monitoring` |
| 2 | Flutter 客户端（地图可视化、多模式任务编排） | ✅ `client/flutter_monitor` |
| 3 | 紧急停止机制（SafetyGate + EmergencyStop RPC） | ✅ `safety_gate.cpp` + `control.proto` |
| 4 | 任务调度（航点序列、暂停/恢复/取消、循环巡检） | ✅ `task_manager.cpp` |
| 5 | 系统服务编排（systemd 服务 + App 端模式切换） | ✅ `service_gateway.dart` + systemd units |
| 6 | TaskMode 枚举化（navigation/mapping/patrol） | ✅ `map_screen.dart` |
| 7 | max_speed 端到端透传（UI → proto → TaskParams） | ✅ 全链路已接通 |
| 8 | **航点统一管理** — TaskManager 成为唯一 `/way_point` 发布者 | ✅ `task_manager.cpp` + `pct_path_adapter` |
| 9 | **航点查询/清除 RPC** — GetActiveWaypoints + ClearWaypoints | ✅ `control.proto` + `control_service.cpp` |
| 10 | **坐标系修复** — App 航点 map→odom tf2 变换 | ✅ `task_manager.cpp` |
| 11 | **安全联动** — E-stop/模式切换 自动挂起/恢复任务 | ✅ `mode_manager.cpp` ↔ `task_manager.cpp` |
| 12 | **操作守卫** — StartTask 需 Lease + AUTONOMOUS 模式 | ✅ `control_service.cpp` |
| 13 | **航点超时** — 单航点超时自动 FAILED | ✅ `task_manager.cpp` (waypoint_timeout_sec) |
| 14 | **Planner 抑制恢复** — 订阅 /pct_path 新路径到来时自动恢复 | ✅ `task_manager.cpp` |
| 15 | **Flutter 航点 UI 集成** — MapScreen 状态栏 / 可视化 / 预启动冲突检测 | ✅ `map_screen.dart` |
| 16 | **TaskPanel 后端航点** — 运行时显示后端航点来源/计数，启动前冲突检测 | ✅ `task_panel.dart` |
| 17 | **ControlGateway 模式跟踪** — `_currentMode` + `syncModeFromString()` | ✅ `control_gateway.dart` |
| 18 | **重连状态恢复** — `TaskGateway.recoverState()` + SlowState 模式同步 | ✅ `main.dart` + `task_gateway.dart` |
| 19 | **Lease 幂等获取** — `ensureLease()` 已持有则直接返回 | ✅ `robot_client.dart` |
| 20 | **build 副作用修复** — polling 逻辑移至 `addPostFrameCallback` | ✅ `map_screen.dart` |
| 21 | **移动端状态栏去重** — 从 `_buildMapArea` 移除，桌面端改 Stack 包裹 | ✅ `map_screen.dart` |
| 22 | **死代码清理** — 删除 ServiceGateway + deprecated 桥接文件 | ✅ 已删除 3 文件 |
| 23 | **mapping_params 透传** — map_name/save_on_complete/resolution 从 proto 到 TaskManager | ✅ `control_service.cpp` + `task_manager.hpp` |
| 24 | **tracking_tolerance 透传** — FollowPathParams 解析并存储 | ✅ `control_service.cpp` |
| 25 | **Logout 释放租约** — release_lease 字段 + ForceRelease() | ✅ `system_service.cpp` + `lease_manager.cpp` |
| 26 | **MapInfo.created_at** — ListMaps 填充文件时间戳 | ✅ `system_service.cpp` |
| 27 | **健康状态 UI** — 综合健康/定位质量/围栏/子系统频率条 | ✅ `health_status_page.dart` |
| 28 | **远程日志下载** — FileGateway.downloadFile 接入 + 分享 | ✅ `log_export_page.dart` |
| 29 | **建图任务完成流** — StopMapping() + save_on_complete 联动 | ✅ `task_manager.cpp` + `control_service.cpp` |
| 30 | **定位分数显示修正** — ICP fitness → 质量百分比 + 偏差值 | ✅ `status_screen.dart` |
| 31 | **App 生命周期守卫** — WidgetsBindingObserver + 自动 disconnect/releaseLease | ✅ `main.dart` |
| 32 | **建图保存反馈** — onMappingComplete await + catch + SnackBar 通知 | ✅ `main.dart` |
| 33 | **任务轮询容错** — 连续失败计数 + 日志分级 + 用户提示 | ✅ `task_gateway.dart` |
| 34 | **围栏警告横幅** — MapScreen 订阅 geofence，WARNING/VIOLATION 弹橙/红横幅 | ✅ `map_screen.dart` |
| 35 | **建图按钮 UX** — 建图任务取消按钮改为"停止建图"绿色保存图标 | ✅ `map_screen.dart` |
| 36 | **README 补全** — SystemService 5 个缺失 RPC 文档 + proto 表 + 实施清单 | ✅ `README.md` |

### 7.2 进行中 / TODO

| # | 改进项 | 说明 |
|---|--------|------|
| 1 | **任务优先级（priority）** | 需扩展 `StartTaskRequest` proto，加 `priority` 字段，C++/Dart 透传 |
| 2 | **避障覆盖（obstacle_override）** | 需扩展 proto，控制 SafetyGate 行为，透传到 `TeleopCommand` 或任务级 |
| 3 | **统一参数管理** | 将分散在各 launch/yaml 的参数收敛到 `system_params.yaml` |
| 4 | **状态机（Behavior Tree）** | 建议引入 `BehaviorTree.CPP`，管理 Idle→Mapping→Nav→EStop 切换 |
| 5 | **代码规范化** | `.clang-format` / `.clang-tidy` / GitHub Actions CI |
| 6 | **测试框架** | `test/` 目录：LIO 单元测试、地形分析测试、规划器集成测试 |
| 7 | **Docker 开发环境** | ROS2 + 依赖容器化，简化新开发者搭建 |
| 8 | **诊断工具** | `ros2 doctor` 扩展、`/diagnostics` 话题、RViz 插件 |

---

## 8. 常见问题排查

### 8.1 TF变换问题

**问题**: `map → odom` 未发布
- 检查 PGO 或 Localizer 是否启动
- 检查是否有回环检测成功 (PGO)
- 检查 `/relocalize` 是否成功调用 (Localizer)

### 8.2 规划失败

**问题**: local_planner 不发布路径
- 检查 `/terrain_map` 是否有数据
- 检查 `/way_point` 是否设置
- 检查手柄 `/joy` 是否发送速度指令

### 8.3 定位丢失

**问题**: 机器人位置跳变
- 检查 Fast-LIO2 是否正常运行
- 检查 Localizer 的 ICP 是否收敛
- 检查地图文件是否正确加载

---

## 9. 附录

### 9.1 参数速查

| 模块 | 配置文件 | 关键参数 |
|------|---------|---------|
| fastlio2 | `lio.yaml` | `body_frame`, `world_frame`, `r_il`, `t_il` |
| pgo | `pgo.yaml` | `map_frame`, `local_frame`, `loop_score_tresh` |
| localizer | `localizer.yaml` | `static_map_path`, `update_hz` |
| terrain_analysis | launch参数 | `obstacleHeightThre`, `slope_max` |
| local_planner | launch参数 | `vehicleHeight`, `vehicleWidth`, `adjacentRange` |
| remote_monitoring | `grpc_gateway.yaml` | `max_speed`, `obstacle_height_thre`, `stop_distance` |
| flutter_monitor | `map_screen.dart` | `TaskMode`, `_speedLimit`, `_priority` |

### 9.2 相关文档

- `docs/CHANGELOG.md` - 版本历史与开发记录
- `docs/ARCHITECTURE.md` - 系统架构文档
- `src/slam/interface/README.md` - 接口定义
- `src/base_autonomy/TERRAIN_ANALYSIS_EXPLAINED.md` - 地形分析详解
- `src/remote_monitoring/README.md` - gRPC Gateway 文档
- `src/robot_proto/proto/control.proto` - 任务控制协议定义
- `client/flutter_monitor/CHANGELOG.md` - 客户端版本记录

---

*最后更新: 2026-02-11*
