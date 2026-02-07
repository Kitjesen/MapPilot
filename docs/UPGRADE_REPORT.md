# 导航系统升级报告

> 日期: 2026-02-07
> 基于: [导航系统优化方案](../docs/plan.md) 中的缺口分析

---

## 一、升级背景

从用户视角审阅项目后发现，底层管道（SLAM → 地形分析 → 局部规划 → 路径跟踪 → 底盘控制）已打通，但"开箱即用"体验存在 **7 个关键缺口**。本次升级集中修复这些缺口，涵盖 P0（能用）→ P1（好用）→ P2（可靠）→ P3（安全）四个层级。

---

## 二、本次升级内容

### 2.1 P0: gRPC Relocalize / SaveMap 实现

| 项目 | 内容 |
|------|------|
| **问题** | `SystemServiceImpl::Relocalize()` 和 `SaveMap()` 是空壳，App 上点"重定位"显示成功但机器人实际没有执行 |
| **方案** | 创建 ROS 2 Service Client，调用底层 `/relocalize` 和 `/save_map` 服务 |
| **改动文件** | `system_service.hpp`, `system_service.cpp` |

关键实现:
- 构造函数中创建 `interface::srv::Relocalize` 和 `interface::srv::SaveMaps` 的 Service Client
- 每次调用先 `wait_for_service(2s)` 检测服务可用性，不可用时返回 `ERROR_CODE_SERVICE_UNAVAILABLE`
- 异步发送请求 + 10 秒超时等待，超时返回 `ERROR_CODE_TIMEOUT`
- 透传 ROS 2 服务的 `success` / `message` 字段到 gRPC 响应

### 2.2 P0: 状态机守卫注入

| 项目 | 内容 |
|------|------|
| **问题** | `ModeManager` 定义了 7 个转换守卫（`tf_ok`, `localization_valid` 等），但函数体为空，安全约束形同虚设 |
| **方案** | 在 `grpc_gateway.cpp` 中从各独立模块取真实状态注入守卫 |
| **改动文件** | `grpc_gateway.cpp`, `lease_manager.hpp`, `lease_manager.cpp` |

守卫绑定详情:

| 守卫 | 数据源 | 说明 |
|------|--------|------|
| `tf_ok` | `aggregator_->GetFastState().tf_ok()` | map→odom→body TF 链完整 |
| `localization_valid` | `aggregator_->GetFastState().tf_ok()` | map→odom TF 由 Localizer/PGO 发布 |
| `has_lease` | `lease_mgr_->HasActiveLease()` | 新增方法，检查未过期租约 |
| `slam_running` | `aggregator_->GetSlowState().topic_rates().odom_hz() > 20` | 里程计频率 |
| `estop_clear` | `!safety_gate_->GetSafetyStatus().estop_active()` | 急停已清除 |
| `tilt_safe` | `!safety_gate_->GetSafetyStatus().tilt_limit_active()` | 倾斜在限制内 |
| `fence_safe` | `geofence_monitor_->GetState() != VIOLATION` | 围栏未越界 |

效果: `IDLE → AUTONOMOUS` 需要 `tf_ok ∧ localization_valid`；`ClearEstop()` 需要 `tilt_safe ∧ fence_safe`。

### 2.3 P0: 一键启动 Launch 文件

| 项目 | 内容 |
|------|------|
| **问题** | 用户需要手动开 5+ 终端，按顺序启动驱动、SLAM、地形分析、规划、监控等 |
| **方案** | 创建两个顶层 launch 文件 |
| **新增文件** | `launch/navigation_bringup.launch.py`, `launch/navigation_run.launch.py` |

#### 建图模式 (`navigation_bringup.launch.py`)

```
Livox 驱动 → Fast-LIO2 → PGO → sensor_scan_generation
          → terrain_analysis → terrain_analysis_ext
          → local_planner (手柄遥控+避障)
          → robot_driver → gRPC gateway
```

用法: `ros2 launch navigation_bringup.launch.py maxSpeed:=1.0`

#### 运行模式 (`navigation_run.launch.py`)

```
Livox 驱动 → Fast-LIO2 → Localizer → sensor_scan_generation
          → terrain_analysis → terrain_analysis_ext
          → local_planner (autonomyMode=true)
          → pct_path_adapter → robot_driver → gRPC gateway
```

用法: `ros2 launch navigation_run.launch.py`

注意: 全局规划器 (PCT_planner, Python) 仍需单独启动。

### 2.4 P1: TaskManager 任务管理模块

| 项目 | 内容 |
|------|------|
| **问题** | 系统一次只能处理一个航点，`StartTask` / `CancelTask` 是空壳，无法多点巡检 |
| **方案** | 新建 `TaskManager` 模块，实现航点队列 + 任务状态机 + 进度推送 |
| **新增文件** | `core/task_manager.hpp`, `core/task_manager.cpp` (~250 行) |
| **修改文件** | `control_service.hpp/.cpp`, `grpc_gateway.hpp/.cpp`, `CMakeLists.txt` |

核心功能:
- **航点队列**: 接收 N 个目标，按序下发 `/way_point`
- **到达检测**: 订阅 `/Odometry`，计算与当前航点的欧氏距离，≤ `arrival_radius` 即到达
- **循环巡检**: `INSPECTION` 类型任务自动设置 `loop=true`
- **状态机**: IDLE → RUNNING → PAUSED → COMPLETED / FAILED / CANCELLED
- **进度回调**: 通过 EventBuffer → gRPC StreamEvents → App
- **ControlService 集成**: `StartTask` 解析 `params_json` 格式 `"x1,y1,z1;x2,y2,z2;..."`，`CancelTask` 委托给 TaskManager

### 2.5 P1: 断联自动降级

| 项目 | 内容 |
|------|------|
| **问题** | gRPC 断联后，AUTONOMOUS 模式下机器人继续全速运行 |
| **方案** | 追踪心跳时间戳，按断联时长分级降速/停车 |
| **改动文件** | `system_service.hpp/.cpp`, `grpc_gateway.hpp/.cpp` |

降级策略:

| 断联时间 | 动作 | 机制 |
|----------|------|------|
| < 30s | 正常运行 | — |
| 30s ~ 5min | 减速 50% | 发布 `/slow_down=2` |
| > 5min | 切换 IDLE，原地停车 | `ModeManager::SwitchMode(IDLE)` |
| 重新连接 | 恢复正常速度 | 发布 `/slow_down=0` |

实现细节:
- `SystemServiceImpl::Heartbeat()` 每次被调用时记录 `steady_clock` 时间戳
- `SecondsSinceLastHeartbeat()` 公开方法供 gateway 查询
- 仅在 `AUTONOMOUS` 模式下检查，其他模式不触发
- 从未收到心跳时不触发（避免启动时误降级）

### 2.6 P2: 接入 `/terrain_map_ext`

| 项目 | 内容 |
|------|------|
| **问题** | `terrain_analysis_ext` 发布 `/terrain_map_ext`（含连通性检查 + 时间衰减累积），但没有任何节点订阅 |
| **方案** | `local_planner` 增加订阅并合并到 planner cloud |
| **改动文件** | `localPlanner.cpp` |

改动摘要:
- 新增 `subTerrainCloudExt_` 订阅 `/terrain_map_ext`
- 新增 `terrainCloudExt_`, `terrainCloudExtCrop_` 点云缓存
- `terrainCloudExtHandler()`: 按距离和高度阈值裁剪
- `processLoop()`: 在 `terrainCloudDwz_` 写入 `plannerCloud_` 之后，追加 `terrainCloudExtCrop_` 点

效果: 局部规划器现在能利用连通性信息，避免驶入"看起来可通行但实际是死胡同"的区域。

### 2.7 P2: 定位质量监控

| 项目 | 内容 |
|------|------|
| **问题** | Localizer 的 ICP fitness score 未暴露，HealthMonitor 不检查定位质量 |
| **方案** | Localizer 发布指标 → HealthMonitor 纳入判定 |
| **改动文件** | `icp_localizer.h/.cpp`, `localizer_node.cpp`, `health_monitor.hpp/.cpp` |

链路:

```
ICPLocalizer::align()
  └→ m_last_fitness_score = m_refine_icp.getFitnessScore()
LocalizerNode::timerCB()
  └→ 发布 /localization_quality (std_msgs/Float32)
HealthMonitor
  └→ 订阅 /localization_quality
  └→ score < 0.1 → OK, < 0.3 → DEGRADED, ≥ 0.3 → CRITICAL
```

### 2.8 P3: 近场急停机制

| 项目 | 内容 |
|------|------|
| **问题** | 障碍物突然出现在 0.5m 内时，减速机制来不及生效 |
| **方案** | 在 `local_planner` 的处理循环中增加近场检测，直接发布 `/stop` |
| **改动文件** | `localPlanner.cpp` |

检测逻辑:
- 在 `plannerCloudCrop_`（已转换到 body 坐标系）中扫描
- 条件: `x > 0`（前方）且 `x < 0.5m` 且 `|y| < vehicleWidth/2 + 0.1m` 且高度超过障碍物阈值
- 触发: 发布 `/stop=2`（全停）
- 恢复: 障碍物离开后发布 `/stop=0`
- 带状态记忆 (`nearFieldStopped_`)，避免重复发布

### 2.9 P3: Proto 健康/围栏扩展

| 项目 | 内容 |
|------|------|
| **问题** | 健康和围栏状态只在 ROS 2 话题上，外部 App 无法获取 |
| **方案** | 扩展 `telemetry.proto`，通过 `StreamSlowState` 推送 |
| **改动文件** | `telemetry.proto`, `status_aggregator.hpp/.cpp`, `grpc_gateway.cpp` |

新增 Proto 消息:

```protobuf
message HealthStatus {
  string overall_level = 1;           // "OK" / "DEGRADED" / "CRITICAL" / "FAULT"
  repeated SubsystemHealth subsystems = 2;
  float localization_score = 3;
}

message GeofenceStatus {
  string state = 1;                   // "NO_FENCE" / "SAFE" / "WARNING" / "VIOLATION"
  bool has_fence = 2;
  double margin_distance = 3;         // 到围栏距离 (m)
}
```

在 `SlowState` 中新增 `health` (field 8) 和 `geofence` (field 9) 字段。

StatusAggregator 在 `update_slow_state()` 中从 HealthMonitor 和 GeofenceMonitor 读取最新状态填充。

---

## 三、文件变更清单

### 新增文件

| 文件 | 行数 | 说明 |
|------|------|------|
| `launch/navigation_bringup.launch.py` | ~170 | 建图模式一键启动 |
| `launch/navigation_run.launch.py` | ~190 | 运行模式一键启动 |
| `remote_monitoring/include/.../task_manager.hpp` | ~115 | 任务管理器头文件 |
| `remote_monitoring/src/core/task_manager.cpp` | ~250 | 任务管理器实现 |

### 修改文件

| 文件 | 改动范围 | 说明 |
|------|----------|------|
| `remote_monitoring/include/.../system_service.hpp` | +15 行 | 新增 Service Client、心跳追踪成员 |
| `remote_monitoring/src/services/system_service.cpp` | ~100 行重写 | Relocalize/SaveMap 实际调用 + 心跳记录 |
| `remote_monitoring/include/.../control_service.hpp` | +5 行 | 新增 TaskManager 指针 |
| `remote_monitoring/src/services/control_service.cpp` | ~80 行重写 | StartTask/CancelTask 对接 TaskManager |
| `remote_monitoring/include/.../grpc_gateway.hpp` | +10 行 | 新增 TaskManager、/slow_down 发布、断联状态 |
| `remote_monitoring/src/grpc_gateway.cpp` | +90 行 | 守卫注入 + 断联降级 + 组件接线 |
| `remote_monitoring/include/.../lease_manager.hpp` | +2 行 | 新增 `HasActiveLease()` |
| `remote_monitoring/src/core/lease_manager.cpp` | +8 行 | `HasActiveLease()` 实现 |
| `remote_monitoring/include/.../health_monitor.hpp` | +10 行 | 新增定位质量订阅和存储 |
| `remote_monitoring/src/core/health_monitor.cpp` | +25 行 | 定位质量判定 + 订阅接线 |
| `remote_monitoring/include/.../status_aggregator.hpp` | +10 行 | 新增 HealthMonitor/GeofenceMonitor 注入 |
| `remote_monitoring/src/status_aggregator.cpp` | +40 行 | SlowState 填充健康/围栏状态 |
| `remote_monitoring/CMakeLists.txt` | +1 行 | 新增 task_manager.cpp |
| `robot_proto/proto/telemetry.proto` | +35 行 | HealthStatus, GeofenceStatus, SlowState 扩展 |
| `base_autonomy/local_planner/src/localPlanner.cpp` | +60 行 | /terrain_map_ext 接入 + 近场急停 |
| `slam/localizer/src/localizers/icp_localizer.h` | +3 行 | 暴露 fitness score |
| `slam/localizer/src/localizers/icp_localizer.cpp` | +1 行 | 记录 fitness score |
| `slam/localizer/src/localizer_node.cpp` | +10 行 | 发布 /localization_quality |

---

## 四、待做事项

### 高优先级

- [ ] **colcon 构建验证**: 完整编译整个工作空间，修复可能的编译错误
- [ ] **Proto 重新生成 Dart 代码**: `telemetry.proto` 变更后需要重新运行 `scripts/proto_gen.sh`，更新 Flutter 客户端的 `.pb.dart` 文件
- [ ] **Flutter App 适配**: MapPilot 的 Settings/Status 页面增加 HealthStatus 和 GeofenceStatus 展示（健康绿/黄/红灯 + 围栏距离指示器）
- [ ] **TaskManager 的 App 端 UI**: MapPilot 需要增加"巡检任务"页面，支持在地图上设置多航点、启动/暂停/取消任务、查看进度

### 中优先级

- [ ] **pct_adapters 到达事件**: 当前 TaskManager 通过 `/Odometry` 自行判断到达，若 `pct_path_adapter` 能发布到达事件（如自定义话题），可以更精准地触发航点切换
- [ ] **TaskManager JSON 解析升级**: 当前使用简易分号分隔格式 `"x1,y1,z1;x2,y2,z2"`，后续改为标准 JSON 解析（nlohmann/json 或 RapidJSON），支持 `arrival_radius`、`loop` 等参数
- [ ] **断联降级可配置化**: 将 30s / 5min 阈值提取到 `grpc_gateway.yaml` 配置文件
- [ ] **近场急停距离参数化**: 将 0.5m 硬编码改为 launch 参数，支持不同车型调整
- [ ] **定位质量阈值调参**: 当前 ICP score 阈值 (0.1 / 0.3) 为经验值，需要在实际环境中标定

### 低优先级

- [ ] **rosbag 集成**: 通过 gRPC 命令触发录制，关键事件自动触发
- [ ] **BehaviorTree 替代状态机**: 使用 BehaviorTree.CPP 替代简单的 ModeManager 状态机，支持更复杂的任务编排
- [ ] **localization_valid 守卫增强**: 当前 `localization_valid` 与 `tf_ok` 共用同一判据，后续可结合 ICP fitness score 做更精确的判定（如 `tf_ok && score < 0.3`）
- [ ] **多机器人协调**: 多台机器人共享地图、任务分配、冲突避免
- [ ] **仿真测试框架**: Gazebo/Isaac Sim 集成，支持自动化回归测试

---

## 五、路线图更新

```
2026 Q1 (当前)
├── ✅ 数学优化
├── ✅ 四层安全架构
├── ✅ 远程监控 + Flutter App
├── ✅ gRPC Relocalize / SaveMap 实现 ← 本次
├── ✅ 状态机守卫注入 ← 本次
├── ✅ 一键启动 Launch 文件 ← 本次
├── ✅ TaskManager 任务管理 ← 本次
├── ✅ 断联自动降级 ← 本次
├── ✅ terrain_map_ext 接入 ← 本次
├── ✅ 定位质量监控 ← 本次
├── ✅ 近场急停 ← 本次
├── ✅ Proto 健康/围栏扩展 ← 本次
├── 🔲 colcon 构建验证
├── 🔲 Proto Dart 代码重新生成
└── 🔲 Flutter App 健康/围栏/巡检 UI

2026 Q2
├── 🔲 TaskManager JSON 解析升级
├── 🔲 断联降级可配置化
├── 🔲 pct_adapters 到达事件
├── 🔲 rosbag 集成
└── 🔲 定位质量阈值标定

2026 Q3+
├── 🔲 BehaviorTree 替代状态机
├── 🔲 多机器人协调
└── 🔲 仿真测试框架
```

---

*最后更新: 2026-02-07*
