# 导航系统工程化计划

> 最后更新: 2026-02-06

---

## 一、已完成工作

### 1.1 数学优化 (commit `c16ab4a`)

对 SLAM、地形分析、路径规划的核心数学部分做了 8 处改进：

| 改动 | 文件 | 类型 | 收益 |
|------|------|------|------|
| 平面估计除零保护 | `commons.cpp` | 严谨 | 消除 NaN 传播, 共线点退化安全返回 |
| IESKF `.inverse()` → `.ldlt().solve()` | `ieskf.cpp` | 可靠 | 对称正定矩阵最优分解, 数值稳定 |
| Jacobian Bug (`t_wi` → `t_il`) | `lidar_processor.cpp` | 严谨 | 修正点到面残差对旋转的偏导错误 |
| 删除重复 `transformPointCloud` | `lidar_processor.cpp` | 可靠 | 原代码应用了 T² 变换 |
| 缓存 `R_wi`/`R_wl` 到循环外 | `lidar_processor.cpp` | 效能 | 省 N 次 3×3 矩阵乘法/帧 |
| 三步欧拉旋转 → 预计算矩阵 | `terrainAnalysis.cpp` | 效能 | 每点 9 vs 18 乘法 (障碍物密集场景显著) |
| `sqrt(sqrt())` NaN 防御 | `localPlanner.cpp` | 严谨 | 负参数不再产生 NaN |
| 删除冗余角度归一化 | `pathFollower.cpp` | 效能 | `[-3π,3π]` 一次归一化到 `(-π,π]` |
| `57.3` → `180.0/M_PI` | `lidar_processor.cpp` | 严谨 | 精确常量, 编译器零开销 |

### 1.2 四层解耦安全架构 (本次会话)

从零搭建了四层独立安全保障体系：

```
Layer 4: HealthMonitor     — 子系统健康聚合 + 自动降级
Layer 3: ModeManager       — 形式化状态机 (转换守卫矩阵)
Layer 2: GeofenceMonitor   — 围栏越界检测 (射线法 + 三级预警)
Layer 1: Driver Watchdog   — 底盘自保护 (200ms cmd_vel 超时)
```

**核心原则: 任何一层崩溃不影响其他层。**

#### Layer 1: Driver Watchdog (`driver_node.py`)
- 独立于所有上层, 200ms 无 `/cmd_vel` → 零速
- 发布 `/driver/watchdog_active` 供外部监控
- 这是最后防线, 即使 remote_monitoring 全崩也能停车

#### Layer 2: GeofenceMonitor (`geofence_monitor.cpp`)
- 订阅 `/Odometry` + `/navigation_boundary`, 不依赖 SafetyGate/ModeManager
- 射线法判断点-多边形关系: O(N) per check @ 20Hz
- 三级策略: SAFE → WARNING (3m) → VIOLATION (0.5m) → 直接发 `/stop=2`
- 越界事件推送到 EventBuffer → gRPC → App

#### Layer 3: ModeManager (`mode_manager.cpp`)
- 形式化 Mealy 机, 15 条合法转换 (vs 原来 36 条全连接)
- 转换守卫: `IDLE→AUTONOMOUS` 需 `tf_ok ∧ localization_valid`
- `EmergencyStop()` lock-free 独立路径, 不经过 `SwitchMode()`, 不等锁
- `ClearEstop()` 需要 `tilt_safe ∧ fence_safe` 才允许

#### Layer 4: HealthMonitor (`health_monitor.cpp`)
- 独立订阅话题做频率统计 + TF 链完整性检查
- 发布 `/robot_health` 供外部设备 (绿/黄/红灯)
- FAULT → 双路径停车: 直接发 `/stop=2` + 回调通知 ModeManager
- 判定规则: SLAM < 20Hz → FAULT, TF 断裂 → FAULT, 地形 < 1Hz → CRITICAL

#### 解耦修复
- ControlService.SetMode 委托给 ModeManager 守卫 (不再直接写 mode atomic)
- pathFollower `/stop` 改为 max 优先级仲裁 (多发布者不冲突)
- HealthMonitor 有直接 `/stop` 发布 (不依赖 ControlService 存活)

### 1.3 远程监控 & 移动客户端 (之前会话)

| 模块 | 功能 |
|------|------|
| gRPC 服务端 (`remote_monitoring`) | Telemetry/Control/Data 三大服务 |
| Flutter 客户端 (`MapPilot`) | Status/Control/Map/Events/Settings 五页 |
| 文件管理 OTA | gRPC 分块上传模型/地图/配置到机器人 |
| 云端更新 | App 联网从 GitHub Releases 拉取 → gRPC 部署到机器人 |
| GitHub Actions CI | APK 自动编译 + 模型发布 workflow |
| robot_proto | 独立仓库, Protobuf 接口定义 |

---

## 二、对整个项目的贡献

### 2.1 从"能跑"到"敢放到户外跑"

```
之前:  感知 → 规划 → 控制 → 底盘    (前向链路)
       状态 → gRPC → 外部设备        (反馈链路)

之后:  + 状态 → 判定 → 自动降级/急停  (闭环决策)
       + 围栏 → 检测 → 告警/急停      (安全闭环)
       + 底盘 → 看门狗 → 自保护        (最后防线)
       + 外部 → 守卫 → 拒绝非法模式    (操作安全)
```

### 2.2 数学层面的可信度

- 消除了 3 处 NaN/Inf 传播路径 (平面估计、路径评分、Jacobian)
- IESKF 协方差更新从不稳定的 `.inverse()` 改为保证正定的 LDLT
- 修正了一个可能导致状态估计偏差的 Jacobian Bug (`t_wi` → `t_il`)

### 2.3 工程化提升

| 维度 | 之前 | 之后 |
|------|------|------|
| 安全层数 | 1 (SafetyGate) | 4 (解耦分层) |
| 状态机 | enum + switch (全连接) | 形式化守卫矩阵 |
| 围栏 | 被动避障 (路径绕开) | 主动检测 + 告警 + 急停 |
| 健康监控 | 无 | 自动频率/TF 监控 + 降级 |
| 停车冗余 | 单路径 | 4 条独立路径 |
| 远程管理 | 无 | gRPC + Flutter App |

---

## 三、接下来要做什么

### 优先级 1 — 补全守卫注入 (让状态机约束真正生效)

**现状**: ModeManager 定义了守卫接口 `TransitionGuards`, 但函数体尚未注入。

**需要做**:
```cpp
// grpc_gateway.cpp 中注入:
mode_manager_->SetTransitionGuards({
    .tf_ok = [this]() { return aggregator_->GetFastState().tf_ok(); },
    .localization_valid = [this]() { /* 检查 ICP 配准分数 */ },
    .has_lease = [this]() { return lease_mgr_->HasValidLease(); },
    .slam_running = [this]() { return aggregator_->... odom_hz > 20; },
    .estop_clear = [this]() { return !safety_gate_->...estop_active; },
    .tilt_safe = [this]() { return safety_gate_->...tilt < 30°; },
    .fence_safe = [this]() { return geofence_->GetState() != VIOLATION; },
});
```

**工作量**: ~30 行, 纯接线, 不改任何模块内部  
**影响**: "IDLE → AUTONOMOUS 必须定位有效" 等约束从纸面变成代码

### 优先级 2 — 断联自动降级策略

**现状**: gRPC 断联后, 自主模式下机器人继续全速跑, 无人监控。

**需要做**:
- 在 grpc_gateway 后台循环中追踪最后一次客户端心跳
- 断联 < 30s: 继续, 记录事件
- 断联 30s-5min: 发布 `/slow_down=2` (减速 50%)
- 断联 > 5min: 切换到 IDLE, 原地停车

**工作量**: ~50 行, 在 grpc_gateway.cpp 的 while 循环中  
**影响**: 户外 WiFi 信号丢失后的安全兜底

### 优先级 3 — 任务队列 (航点序列)

**现状**: 一次只能发一个 waypoint, 没有多点巡检。

**需要做**:
- 新建 `TaskManager` 模块
- 实现航点序列: 接收 N 个目标, 按序下发
- 任务暂停/恢复/取消
- 进度回调到 gRPC → App

**工作量**: ~200 行新模块  
**影响**: 支持巡检、多点任务等实际使用场景

### 优先级 4 — Proto 扩展 (gRPC 推送健康/围栏状态)

**现状**: 健康状态通过 ROS2 `/robot_health` String 发布, 围栏通过 `/geofence/status` 发布。外部设备只能在 ROS2 网络内订阅。

**需要做**:
- `telemetry.proto` 新增 `RobotHealth`, `SubsystemHealth`, `GeofenceStatus` 消息
- `StreamSlowState` 回填健康和围栏字段
- Flutter App Settings 页展示子系统健康绿/黄/红灯

**工作量**: ~100 行 proto + 服务端适配 + App UI  
**影响**: 远程操作员看到完整系统健康状态

### 优先级 5 — 定位质量指标

**现状**: 无法判断定位是否可信。ICP 配准分数、协方差没有暴露。

**需要做**:
- Localizer 发布 ICP fitness score / 协方差到一个话题
- StatusAggregator 收集 → HealthMonitor 纳入判定
- 定位质量下降 → DEGRADED → 自动减速

**工作量**: ~80 行 (localizer 改动 + aggregator 适配)  
**影响**: "机器人在不确定自己在哪的时候还全速跑"的风险消除

### 优先级 6 — rosbag 集成

**需要做**:
- 通过 gRPC 命令触发 rosbag record/stop
- 关键事件 (急停、围栏越界、规划失败) 自动触发录制
- 录制文件管理 (定时清理、容量限制)

**工作量**: ~150 行  
**影响**: 事后问题分析能力

---

## 四、路线图

```
2026 Q1 (当前)
├── ✅ 数学优化 (已完成)
├── ✅ 四层安全架构 (已完成)
├── ✅ 远程监控 + Flutter App (已完成)
├── 🔲 守卫注入 (优先级 1, 预计 0.5 天)
└── 🔲 断联降级 (优先级 2, 预计 0.5 天)

2026 Q2
├── 🔲 任务队列 (优先级 3)
├── 🔲 Proto 健康扩展 (优先级 4)
├── 🔲 定位质量指标 (优先级 5)
└── 🔲 rosbag 集成 (优先级 6)

2026 Q3+
├── 🔲 BehaviorTree 替代简单状态机
├── 🔲 多机器人协调
└── 🔲 仿真测试框架 (Gazebo/Isaac Sim)
```

---

## 五、文件变更清单 (本次会话)

### 新增文件
| 文件 | 行数 | 说明 |
|------|------|------|
| `remote_monitoring/include/.../geofence_monitor.hpp` | 111 | 围栏监控头文件 |
| `remote_monitoring/src/core/geofence_monitor.cpp` | 198 | 围栏监控实现 |
| `remote_monitoring/include/.../health_monitor.hpp` | 125 | 健康监控头文件 |
| `remote_monitoring/src/core/health_monitor.cpp` | 196 | 健康监控实现 |

### 修改文件
| 文件 | 改动 | 说明 |
|------|------|------|
| `drivers/robot_driver/driver_node.py` | 重写 | 加入独立看门狗 |
| `remote_monitoring/include/.../mode_manager.hpp` | 重写 | 形式化状态机 + 守卫接口 |
| `remote_monitoring/src/core/mode_manager.cpp` | 重写 | 转换守卫 + lock-free 急停 |
| `remote_monitoring/include/.../grpc_gateway.hpp` | +3 行 | 新增模块指针 |
| `remote_monitoring/src/grpc_gateway.cpp` | +20 行 | 实例化 + 接线 |
| `remote_monitoring/include/.../control_service.hpp` | +5 行 | ModeManager 委托 |
| `remote_monitoring/src/services/control_service.cpp` | ~30 行 | SetMode 走守卫 |
| `remote_monitoring/CMakeLists.txt` | +2 行 | 新源文件 |
| `base_autonomy/local_planner/src/pathFollower.cpp` | +10 行 | /stop max 优先级 |
| `slam/fastlio2/src/map_builder/commons.cpp` | +3 行 | 除零保护 |
| `slam/fastlio2/src/map_builder/ieskf.cpp` | ~6 行 | LDLT 分解 |
| `slam/fastlio2/src/map_builder/lidar_processor.cpp` | ~10 行 | 缓存+修 bug |
| `base_autonomy/local_planner/src/localPlanner.cpp` | +1 行 | NaN 防御 |
| `base_autonomy/terrain_analysis/src/terrainAnalysis.cpp` | +12/-6 行 | 旋转矩阵预计算 |
