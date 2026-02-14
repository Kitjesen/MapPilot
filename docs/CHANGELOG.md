# Changelog

所有重大变更、升级记录和修复日志。

> 格式参考 [Keep a Changelog](https://keepachangelog.com/)

---

## 路线图

```
2026 Q1 (当前)
├── ✅ 数学优化 (SLAM/规划核心改进)
├── ✅ 坐标系统一修复
├── ✅ 四层安全架构
├── ✅ 远程监控 + Flutter App (MapPilot)
├── ✅ gRPC Relocalize / SaveMap 实现
├── ✅ 状态机守卫注入
├── ✅ 一键启动 Launch 文件
├── ✅ TaskManager 任务管理
├── ✅ 断联自动降级
├── ✅ terrain_map_ext 接入
├── ✅ 定位质量监控
├── ✅ 近场急停
├── ✅ Proto 健康/围栏扩展
├── ✅ OTA 更新系统 (直接下载/断点续传/回滚)
├── ✅ OTA v2: Ed25519 签名 + 安全等级 + 依赖管理 + 事务日志
├── ✅ 遥控避障: SafetyGate 近场避障 + /cmd_vel 仲裁
├── ✅ OTA v3: 版本一致性 + 可观测性 + 安全加固 + 端到端实测通过
├── ✅ 航点管理系统 (GetActiveWaypoints / ClearWaypoints / 冲突检测)
├── ✅ 定位健康评分系统 (LocalizationScorer: 0-100 综合评分 → SafetyGate 自动降速)
├── ✅ 飞行数据记录器 (FlightRecorder: 环形缓冲黑盒 → 事故自动 dump)
├── ✅ Flutter App 航点集成 (状态栏 / 可视化 / 预启动检查)
├── ✅ 重连状态恢复 (Task + Mode 自动同步)
├── ✅ Lease 语义收敛 (ensureLease 幂等获取)
├── ✅ 代码卫生 (死代码清理 + proto 字段落地 + 版本号动态化)
├── ✅ Flutter App 健康状态 UI + 围栏状态 + 定位质量
├── ✅ 远程日志下载 + mapping_params/tracking_tolerance 透传
├── ✅ 建图任务完成流 (StopMapping + save_on_complete)
├── ✅ 工程化基建 (clang-format/tidy + pre-commit + gtest + logrotate + 版本同步)
├── ✅ Docker 容器化 (多阶段构建 + supervisord + compose 编排)
├── ✅ 算法解耦: Topic Interface Contract (/nav/* 标准接口 + profile 即插即用)
├── ✅ Proto Dart 重生成 (protoc 跨平台脚本 + Windows 支持)
├── ✅ 参数解耦 (robot_config.yaml + launch 加载器)
├── ✅ Flutter UI 品牌化 (DS Logo + 大算 3D NAV + 版本号)
├── ✅ Flutter 测试覆盖 (gateway + widget 56 tests all green)
├── ✅ 话题校验脚本 (validate_topics.py + pre-commit)
├── ✅ ROS 2 架构审计 + CycloneDDS 通信优化配置
├── ✅ 运行参数在线配置 (130+ 参数 Flutter ↔ gRPC ↔ ROS 2 全链路打通)
├── 🔲 colcon 构建验证
└── 🔲 端到端集成测试

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

## [v1.3.3] - 2026-02-11

运行参数在线配置 — Flutter ↔ gRPC ↔ C++ ↔ ROS 2 全链路打通，130+ 参数实时可调、心跳回读、磁盘持久化。

### 运行参数通信链路 (RuntimeConfig End-to-End)

从 Flutter UI 到导航节点的完整参数配置通道：

```
Flutter UI → RuntimeConfigGateway.pushToServer()
           → RobotClient.setRuntimeConfig() [gRPC]
           → SystemServiceImpl::SetRuntimeConfig() [C++]
           → ForwardConfigToNodes() → AsyncParametersClient::set_parameters()
           → pathFollower/localPlanner/terrainAnalysis::onParamChange()
           → PersistConfigToYaml() [磁盘持久化]
           → SlowState.config_json [心跳回读]
           → RuntimeConfigGateway.updateFromHeartbeat() [Flutter 自动同步]
```

#### Dart Proto 存根

- **system.pb.dart**: 新增 `GetRuntimeConfigRequest/Response`、`SetRuntimeConfigRequest/Response` 四个消息类，使用 JSON 字符串传输 (避免手写 130+ 字段强类型存根)
- **system.pbgrpc.dart**: `SystemServiceClient` 新增 `getRuntimeConfig()` / `setRuntimeConfig()` gRPC 方法 + 服务端 `SystemServiceBase` 对应抽象方法
- **telemetry.pb.dart**: `SlowState` 新增 `configJson` (field 20) + `configVersion` (field 21)，支持心跳配置回读

#### Flutter 客户端

- **robot_client_base.dart**: 新增 `getRuntimeConfig()` / `setRuntimeConfig()` 抽象接口
- **robot_client.dart**: 通过 `_systemClient` 发起真实 gRPC 调用 (5s 超时)
- **mock_robot_client.dart**: 实现 mock 版本，离线开发不编译报错
- **runtime_config_gateway.dart**: `fetchFromServer()` / `pushToServer()` 从 STUB 替换为真实 gRPC，接入 JSON 编解码 + `ErrorCode` 校验
- **main.dart**: 新增 SlowState → RuntimeConfigGateway 心跳自动同步监听器

#### Proto 定义

- **system.proto**: `GetRuntimeConfig/SetRuntimeConfig` RPC 改用 JSON 字符串传输 (`string config_json`)，全量 `RuntimeConfig` message 保留为字段文档参考
- **telemetry.proto**: `SlowState` 新增 `string config_json = 20` + `uint64 config_version = 21`

#### C++ 服务端

- **system_service.hpp/cpp**: 实现 `GetRuntimeConfig` / `SetRuntimeConfig` RPC
  - JSON 配置存储 + 乐观锁版本控制
  - `ForwardConfigToNodes()` — 通过 `AsyncParametersClient` 将变更参数批量推送到 ROS 2 导航节点
  - `CollectCurrentParams()` — 启动时从活跃节点收集当前参数作为基线
  - `LoadConfigFromYaml()` / `PersistConfigToYaml()` — 配置磁盘持久化 (原子写入 `.tmp` → `rename`)
  - 参数路由表 (`ParamRouteMap`): JSON key → (目标节点, ROS2 参数名) 映射，覆盖 pathFollower (28)、terrainAnalysis (11)、localPlanner (6) 共 45 个参数
- **status_aggregator.hpp/cpp**: 新增 `ConfigProvider` 回调，`update_slow_state()` 填充 `config_json` + `config_version` 到 SlowState
- **grpc_gateway.cpp**: 注入 `SystemService` 作为 `StatusAggregator` 的配置提供者

#### 导航节点动态参数

三个核心节点均添加 `on_set_parameters_callback`，支持运行时参数更新无需重启:

| 节点 | 可动态调整参数数 | 示例参数 |
|------|-----------------|---------|
| pathFollower | ~30 | yawRateGain, maxSpeed, maxAccel, lookAheadDis, noRotAtGoal |
| localPlanner | ~30 | pathScale, checkObstacle, obstacleHeightThre, dirWeight |
| terrainAnalysis | ~20 | obstacleHeightThre, useSorting, quantileZ, clearDyObs |

### 变更文件

| 文件 | 变更说明 |
|------|---------|
| `robot_proto/dart/lib/src/system.pb.dart` | +4 个 RuntimeConfig RPC 消息类 (JSON transport) |
| `robot_proto/dart/lib/src/system.pbgrpc.dart` | +getRuntimeConfig/setRuntimeConfig 客户端+服务端 stub |
| `robot_proto/dart/lib/src/telemetry.pb.dart` | SlowState +configJson/configVersion 字段 |
| `robot_proto/proto/system.proto` | RPC 改为 JSON 传输格式 |
| `robot_proto/proto/telemetry.proto` | SlowState +config_json/config_version |
| `client/.../robot_client_base.dart` | +getRuntimeConfig/setRuntimeConfig 抽象接口 |
| `client/.../robot_client.dart` | +真实 gRPC 实现 |
| `client/.../mock_robot_client.dart` | +mock 实现 |
| `client/.../runtime_config_gateway.dart` | STUB → 真实 gRPC + JSON 编解码 |
| `client/.../main.dart` | +SlowState 心跳配置回读监听 |
| `remote_monitoring/.../system_service.hpp` | +Get/SetRuntimeConfig + 持久化 + 路由表声明 |
| `remote_monitoring/.../system_service.cpp` | +RPC 实现 + ForwardConfigToNodes + Persist + Collect |
| `remote_monitoring/.../status_aggregator.hpp` | +ConfigProvider 回调接口 |
| `remote_monitoring/.../status_aggregator.cpp` | +SlowState config_json 填充 |
| `remote_monitoring/.../grpc_gateway.cpp` | +ConfigProvider 注入 |
| `base_autonomy/.../pathFollower.cpp` | +on_set_parameters_callback (~30 params) |
| `base_autonomy/.../localPlanner.cpp` | +on_set_parameters_callback (~30 params) |
| `base_autonomy/.../terrainAnalysis.cpp` | +on_set_parameters_callback (~20 params) |

---

## [v1.3.2] - 2026-02-13

质量冲刺 — Proto 跨平台、参数集中化、品牌 UI、测试全绿、话题校验。

### Proto 重生成 (跨平台)

- **PowerShell 脚本**: `scripts/proto_gen.ps1` — Windows 上运行 `protoc` + `protoc-gen-dart`，功能对齐 `proto_gen.sh`
- **`--check` 模式**: CI 可用 `proto_gen.ps1 -Check` 检测 proto 代码是否需要重新生成
- **清理**: 删除过期的 `client/flutter_monitor/generate_proto.sh` (指向不存在的 proto/ 目录)
- **文档**: `PROTO_REGEN_PLAYBOOK.md` 增加 Windows 章节

### 参数解耦

- **集中配置**: `config/robot_config.yaml` — 机器人 ID、几何尺寸、速度限制、安全阈值、驱动配置、LiDAR、gRPC 端口、控制增益
- **Launch 加载器**: `launch/_robot_config.py` — `robot_cfg(section, key)` / `robot_cfg_str()` 函数
- **Launch 重构**: `navigation_bringup.launch.py`、`navigation_run.launch.py`、`autonomy.launch.py`、`driver.launch.py`、`lidar.launch.py` 中硬编码默认值改为从 `robot_config.yaml` 读取
- **交叉引用**: `grpc_gateway.yaml` 添加 `robot_config.yaml` 对齐注释

### Flutter UI 品牌化

- **Splash 页**: `Icons.smart_toy` → 文字 "DS" monogram + `AppColors.brandGradient`，新增 "3D NAV" 副标题和版本号 `v1.3.2`
- **Home Header**: "Robot ID #X-402" → "大算 NAV" 品牌文字
- **Sidebar**: `Icons.smart_toy_rounded` → "DS" 字标 + "NAV" 标签
- **Settings**: 新增品牌信息卡 (DS Logo + 大算 3D NAV + 版本)，License 页面统一品牌名
- **App 标题**: `大算机器人` → `大算 3D NAV`

### Flutter 测试

- **test_helpers.dart**: 通用 `pumpApp()` + `createMockClient()` 工具
- **新增 Widget 测试**: `splash_screen_test.dart`、`home_screen_test.dart`、`map_manager_page_test.dart`、`status_screen_test.dart`
- **Gateway 测试修复**:
  - `file_gateway_test.dart`: 初始目录从 `/` 改为 `FileGateway.defaultDirectory`，导航测试先 `listFiles('/')` 归位
  - `map_gateway_test.dart`: `refreshMaps` 断连测试从 `throwsException` 改为检查 `error == '未连接'`
  - `ota_gateway_test.dart`: `cancelDeploy` 测试改为等待 future 后断言 `activeDeployment == null`；`fetchInstalledVersions` 改用 `expectLater` 捕获异步异常
- **结果**: 56 tests, 0 failures

### 话题校验脚本

- **validate_topics.py**: 纯 Python 脚本，加载 `topic_contract.yaml` 作为标准源，扫描 `grpc_gateway.yaml` + `launch/**/*.launch.py` 中的 `/nav/*` 话题引用
  - `ERROR`: 未在契约中定义的话题
  - `WARNING`: 契约中定义但未被引用的话题
- **Pre-commit 集成**: `.pre-commit-config.yaml` 新增 `validate-topics` hook

### 通信优化 (CycloneDDS + QoS)

- **架构审计**: `docs/DECOUPLING_ANALYSIS.md` — src/ 全部 40+ 源文件逐行 ROS 2 依赖审计 (92% 纯 C++)
- **务实结论**: 不做全量裸 DDS 解耦，保留 ROS 2 生态，通过配置优化
- **CycloneDDS 配置**: `config/cyclonedds.xml` — 网络接口/缓冲区/发现/共享内存配置
- **QoS 场景化**: `config/qos_profiles.yaml` — 7 类链路的 reliability/history/deadline/lifespan 推荐参数
- **优化指南**: `docs/COMM_OPTIMIZATION.md` — 完整 5 步优化路径 (RMW 切换 → QoS → Profiling → 零拷贝旁路)
- **Docker 切换**: `Dockerfile` / `Dockerfile.dev` RMW 从 `rmw_fastrtps_cpp` → `rmw_cyclonedds_cpp`
- **Launch 集成**: `navigation_run.launch.py` / `navigation_bringup.launch.py` 自动加载 cyclonedds.xml
- **robot_config 校验**: `scripts/validate_config.py` — schema 校验 + 值范围 + 交叉校验
- **版本同步**: `scripts/sync_versions.ps1` — Windows 版版本同步脚本 (对齐 .sh 版本)

### 测试修复

- **FileGateway 测试**: 初始目录从 `/` 改为 `FileGateway.defaultDirectory`
- **MapGateway 测试**: `refreshMaps` 断连改为断言 `error == '未连接'`
- **OtaGateway 测试**: `cancelDeploy` 改为等待 future 后断言 `null`；async 异常改用 `expectLater`
- **结果**: 56 tests, 0 failures (此前有 3 个预存失败)

---

## [v1.3.1] - 2026-02-12

工程化改进 + 算法解耦。系统从"能跑"升级到"能维护"。

### 工程化基建

- **C++ 代码规范**: 添加 `.clang-format` (Google 风格) + `.clang-tidy` (捕获 use-after-move、dangling-handle 等真实 bug)
- **Pre-commit hooks**: `.pre-commit-config.yaml` — 提交前自动检查 clang-format、trailing whitespace、YAML 校验、dart format/analyze、protobuf lint
- **C++ 单元测试**: 给 `remote_monitoring` 核心模块添加 gtest 骨架 — `IdempotencyCache`、`FlightRecorder`、`EventBuffer`、`LeaseManager` 四个模块完整覆盖。CI 增加 `colcon test` 步骤
- **版本同步**: 所有 `package.xml` + `pubspec.yaml` 统一到 `1.3.0`；新增 `scripts/sync_versions.sh` 一键从 `VERSION` 文件同步到所有清单
- **日志清理**: `deploy/logrotate.d/nav-system` (EventBuffer 日志轮转) + `deploy/cron.d/nav-cleanup` (FlightRecorder dump 保留最新 30 个、ROS bag 7 天清理)
- **Dart 依赖锁定**: `han_dog_message` 从 `ref: main` 锁定到具体 commit hash
- **部署自动化**: `install_services.sh` 新增 logrotate/cron 自动安装和运行时目录创建

### Docker 容器化

- **多阶段 Dockerfile**: `docker/Dockerfile` — build → test → runtime 三阶段，生产镜像 ~1.5GB
- **开发镜像**: `docker/Dockerfile.dev` — 含 gdb/valgrind/rviz2/ccache/clang-tidy，源码 volume 挂载
- **容器入口**: `docker/entrypoint.sh` — 自动 source ROS2、配置 LiDAR 网络、生成 DDS 无共享内存配置
- **进程管理**: `docker/supervisord.conf` — 替代 systemd，管理 6 个进程 (lidar/slam/autonomy/planning/grpc/ota)，每个可通过环境变量开关
- **编排**: `docker-compose.yml` (生产: host 网络、设备透传、Volume 持久化、8GB 限制、健康检查) + `docker-compose.dev.yml` (开发: 源码挂载、ccache、SYS_PTRACE、X11 转发)
- **文档**: `docs/DOCKER_GUIDE.md` — 快速开始、建图/导航模式切换、硬件配置、CI 集成

### 算法解耦: Topic Interface Contract

- **标准接口契约**: `config/topic_contract.yaml` — 定义所有模块间 `/nav/*` 标准话题名称
- **Profile 系统**: `launch/profiles/` 目录，每个算法一个 launch 文件负责 remap:
  - `slam_fastlio2.launch.py` / `slam_stub.launch.py`
  - `planner_pct.launch.py` / `planner_stub.launch.py`
  - `localizer_icp.launch.py`
- **Launch 重构**: `slam.launch.py` 和 `planning.launch.py` 改为动态 profile 加载 (`OpaqueFunction`)
- **全量 remap**: `autonomy.launch.py`、`lidar.launch.py`、`driver.launch.py` 所有话题 remap 到 `/nav/*`
- **C++ 参数化**: `task_manager.cpp`、`health_monitor.cpp`、`localization_scorer.cpp`、`system_service.cpp`、`status_aggregator.cpp`、`mode_manager.cpp`、`safety_gate.cpp`、`geofence_monitor.cpp`、`data_service.cpp`、`grpc_gateway.cpp` 中所有硬编码话题名改为 `declare_parameter` + `/nav/*` 默认值
- **配置更新**: `grpc_gateway.yaml` 所有话题名更新为 `/nav/*`
- **换算法**: 只需 `ros2 launch navigation_run.launch.py slam_profile:=liosam`，零代码修改
- **后向兼容**: 旧话题名可通过 YAML 参数覆盖
- **文档**: `docs/TOPIC_CONTRACT.md` — 完整接口契约 + 添加新算法指南

---

## [v1.3.0] - 2026-02-11

全新子系统 — 定位健康评分 + 飞行数据记录器（黑盒）。非基于已有代码的补丁，完全从零创建。

### 新增

- **LocalizationScorer (定位健康评分器)** — 全新子系统，聚合 4 个独立信号源:
  - ICP 匹配质量 (40 分, 从 `/localization_quality` 话题)
  - TF 新鲜度 (30 分, map→odom 变换年龄)
  - TF 稳定性 (15 分, 变换发布间隔抖动/标准差)
  - 帧间一致性 (15 分, map→odom 跳变检测)
  - 输出 0-100 综合分数 + LocQuality 枚举 (EXCELLENT/GOOD/DEGRADED/POOR/LOST)
  - 自动生成事件 (定位丢失/恢复/降级)
- **SafetyGate 定位降速** — LocalizationScorer 的 speed_scale (0.0-1.0) 注入 SafetyGate，自动限制遥控速度:
  - ≥80分 → 全速; 60-80 → 70%; 40-60 → 40%; 20-40 → 20%; <20 → 完全停车
- **FlightRecorder (飞行数据记录器/黑盒)** — 全新子系统:
  - 64 bytes/帧 POD 快照 (位姿/速度/IMU/定位分/模式/健康/TF/急停)
  - 环形缓冲区持续录制 (默认 300 帧 = 30s @ 10Hz，仅 19KB)
  - 事故触发: E-stop、健康 FAULT → 自动冻结 + post-trigger 继续录制 5s → dump 到二进制文件
  - dump 文件: `FLTREC01` 魔数 + 256B 头 + N×64B 快照，可通过 DataService 下载
  - 安全: 10s 冷却防高频 dump，路径穿越防护
  - 参数化: `flight_record_dir`, `flight_record_capacity`, `flight_record_post_trigger`
- **FastState Proto 扩展** — 新增 `localization_score` (field 9) 和 `speed_scale` (field 10)
- **Flutter UI: 定位健康指示器** — StatusScreen 新增 LOC HEALTH 卡片 (圆环 + 质量标签 + 颜色)
- **Flutter UI: HealthStatusPage 重构** — 定位卡片显示 0-100 评分圆环、限速因子、自动降速/停车标签

### 架构变更

- `GrpcGateway` 新增 `localization_scorer_` 和 `flight_recorder_` 成员
- `StatusAggregator` 注入 `LocalizationScorer` (score → FastState) 和 `FlightRecorder` (每帧快照)
- `SafetyGate` 新增 `SetLocSpeedScaleProvider` 回调注入
- `ModeManager` 新增 `SetEstopExtraCallback` → E-stop 自动触发黑盒 dump
- 健康监控 FAULT → 同时触发 ModeManager EmergencyStop + FlightRecorder dump

---

## [v1.2.2] - 2026-02-12

质量收口 — 生命周期、错误处理、文档补全。

### 新增

- **App 生命周期守卫** — `WidgetsBindingObserver` 监听应用销毁事件，自动释放 Lease 防止阻塞其他客户端
- **建图自动保存反馈** — `onMappingComplete` 回调增加 `await` + `try/catch` + `SnackBar` 通知，保存失败不再静默丢失
- **任务轮询容错** — `TaskGateway` 轮询增加连续失败计数，≥3 次打日志警告，≥5 次向用户显示"状态同步异常"

### 修复

- **建图任务完成流** — 后端 `StopMapping()` 方法以 `COMPLETED` 状态结束建图（非 `CANCELLED`），触发 App 侧自动保存
- **CancelTask 建图分支** — `ControlService::CancelTask` 检测建图任务后调用 `StopMapping`，返回 `TASK_STATUS_COMPLETED`
- **定位分数显示** — `status_screen.dart` 将 ICP fitness 转换为"定位质量"百分比 + 原始偏差值，不再误导用户
- **围栏警告横幅** — `MapScreen` 订阅 `SlowState.geofence`，在 WARNING/VIOLATION 时显示橙色/红色横幅叠加层
- **建图按钮 UX** — FAB 和状态栏的取消按钮在建图任务时改为绿色"停止建图"，文案"停止并保存"

### 文档

- `remote_monitoring/README.md` 补全 5 个 SystemService RPC 文档（Relocalize / SaveMap / ListMaps / DeleteMap / RenameMap）
- 路线图标记 4 项新完成

---

## [v1.2.1] - 2026-02-11

代码卫生清理、后端 proto 字段落地、Flutter 功能补齐。

### 新增

- **健康状态页面** (`health_status_page.dart`) — 综合健康等级、定位质量 (ICP fitness)、围栏状态、子系统频率条
- **远程日志下载** — `log_export_page.dart` 接入 `FileGateway.downloadFile`，下载后支持系统分享
- **Logout 释放租约** — `system_service.cpp` 读取 `release_lease` 字段并调用 `LeaseManager::ForceRelease()`
- **MapInfo.created_at** — `ListMaps` 填充文件创建时间戳
- **mapping_params 透传** — `control_service.cpp` 解析 `map_name`/`save_on_complete`/`resolution` 并传入 TaskManager
- **tracking_tolerance 透传** — `FollowPathParams.tracking_tolerance` 从 proto 解析到 TaskParams

### 修复

- **死代码清理** — 删除 `ServiceGateway`(314 行)、`cloud_ota_service.dart`、`firmware_rpc_types.dart`
- **硬编码版本号** — `support_page.dart` 改用 `String.fromEnvironment('APP_VERSION')`
- **运算符优先级** — `file_gateway.dart` 为 `.yaml && map` 条件加显式括号
- **建图任务空航点** — `TaskManager::StartTask` 跳过空航点检查和 `PublishCurrentWaypoint`

### 文档

- 新增 `docs/REGRESSION_CHECKLIST_V1_2.md` — 最小回归测试清单
- 新增 `docs/PROTO_REGEN_PLAYBOOK.md` — Proto Dart 重生成执行手册

---

## [v1.2.0] - 2026-02-11

航点管理系统完整集成：后端 RPC、Flutter App UI、重连恢复、Lease 收敛。

### 新增

#### 航点管理 RPC

- **GetActiveWaypoints** — 查询当前活跃航点列表、来源 (App/Planner)、当前索引、进度百分比
- **ClearWaypoints** — 清除所有活跃航点并立即停车，统一在 `TaskManager` 内完成
- **WaypointSource 枚举** — 区分 App 手动下发 (`WAYPOINT_SOURCE_APP`) 和全局规划器生成 (`WAYPOINT_SOURCE_PLANNER`) 的航点

#### Flutter App — 航点 UI 集成

- **MapScreen 任务状态栏** — 任务运行时在地图顶部叠加半透明状态条，显示任务类型、航点来源、当前/总计航点数、进度条及 暂停/恢复/取消/清除 按钮
- **MapScreen 航点可视化** — `_ActiveWaypointPainter` 在 2D 地图上绘制活跃航点（绿色=当前、蓝色=未来、灰色=已完成）、连接线和索引编号
- **MapScreen 预启动冲突检测** — 启动新任务前自动调用 `GetActiveWaypoints`，若有冲突弹出确认对话框引导用户清除
- **TaskPanel 后端航点卡片** — 任务运行面板中显示后端航点来源、计数、刷新/清除按钮；启动前同样检测冲突
- **FAB 模式感知** — 任务运行时隐藏目标设置 FAB，将启动按钮替换为红色取消按钮；ESTOP 下禁用启动并提示

#### 状态管理增强

- **ControlGateway.currentMode** — 新增 `_currentMode` 状态跟踪和 `syncModeFromString()` 方法，支持 SlowState 字符串到枚举的映射
- **TaskGateway.recoverState()** — 重连后重新查询活跃任务状态：运行中→恢复轮询，已终止→清除本地状态
- **RobotClient.ensureLease()** — 幂等 Lease 获取：若已持有则直接返回 true，避免重复请求导致 LEASE_CONFLICT
- **main.dart onReconnected** — 重连回调新增：`taskGateway.recoverState()` + `controlGateway.syncModeFromString()`

### 修复

- **MapScreen build 副作用** — 将 `_syncWaypointPolling()` 从 `_buildTaskStatusBar()` build 方法中移至 `build()` 的 `addPostFrameCallback`，避免在 widget 构建期间触发副作用
- **移动端状态栏重复渲染** — `_buildTaskStatusBar` 同时存在于 `_buildMapArea` 和 `_buildMobileLayout`，导致移动端双重叠加；从 `_buildMapArea` 移除，桌面端改由外层 Stack 包裹

---

## [v1.1.0] - 2026-02-09

OTA v3 增强：版本一致性、可观测性、安全加固。端到端实测全流程通过。

### 新增

#### OTA v3 — 版本一致性 + 可观测性 + 安全加固

- **P1.1 系统版本快照 (`system_version.json`)** — 每次安装/回滚后持久化全量制品版本，解决 `installed_manifest.json` 只记增量不记全局的问题
- **P1.2 设备端 Ed25519 验签** — `VerifyEd25519Signature()` 使用 OpenSSL EVP API 在设备端验证 manifest 签名，配置 `ota_public_key_path` 指定公钥文件
- **P1.3 安装后健康检查** — `PostInstallHealthCheck()` 集成 `HealthMonitor`，安装完成后自动检测 SLAM 系统状态，不通过则触发自动回滚；HOT/UNSPECIFIED 安全等级跳过检查以避免误判
- **P1.4 语义版本比较 (`CompareSemver`)** — 替换原有字符串比较，正确处理 `1.9.0 < 1.10.0` 等场景
- **P2.1 持久化升级历史 (`upgrade_history.jsonl`)** — 所有 install/rollback 事件追加到 JSONL 文件，支持 `GetUpgradeHistory` RPC 分页查询
- **P2.2 标准化失败码 (`OtaFailureCode`)** — 枚举覆盖网络/签名/完整性/磁盘/权限/健康检查等 12 种失败原因
- **P2.3 版本一致性 RPC (`ValidateSystemVersion`)** — App 可校验设备上各制品版本是否与 manifest 一致
- **P3.1 Flutter UI 增强** — 安装版本列表、升级历史面板、readiness 预检集成
- **P3.2 发布通道 (`channel`)** — manifest 新增 `channel` 字段 (stable/beta/nightly)，`CloudOtaService` 按通道过滤
- **P3.3 gRPC TLS 可选** — `grpc_gateway.yaml` 配置 `tls_cert_path` / `tls_key_path` 即可启用 TLS

### 修复

- **健康检查误触发回滚** — HOT 级别更新（模型/配置文件替换）在 SLAM 未运行时健康检查必然报 FAULT，导致安装成功后立即回滚。修复: 对 HOT/UNSPECIFIED 安全等级跳过 PostInstallHealthCheck
- **Rollback RPC 死锁** — `SaveSystemVersionJson()` 内部加锁与 `Rollback()` 外部持锁形成递归死锁。修复: 移除 `SaveSystemVersionJson()` 内部锁，由调用方保证互斥；同时将 `ApplyUpdate` 中的 `SaveInstalledManifest()` / `SaveSystemVersionJson()` 调用移入 `ota_mutex_` 锁作用域内

### 变更文件

| 文件 | 变更说明 |
|------|---------|
| `src/robot_proto/proto/data.proto` | 新增 `OtaFailureCode`, `GetUpgradeHistory`, `ValidateSystemVersion` RPC/消息 |
| `src/remote_monitoring/src/services/data_service.cpp` | +583 行: 版本快照、验签、健康检查、升级历史、死锁修复 |
| `src/remote_monitoring/include/.../data_service.hpp` | 新增方法声明与成员变量 |
| `src/remote_monitoring/src/grpc_gateway.cpp` | 注入 HealthMonitor、TLS 配置 |
| `src/remote_monitoring/config/grpc_gateway.yaml` | 新增 OTA v3 配置项 |
| `client/flutter_monitor/.../robot_client.dart` | 实现 getUpgradeHistory / validateSystemVersion / TLS |
| `client/flutter_monitor/.../firmware_ota_page.dart` | 安装版本列表 + 升级历史 UI |
| `client/flutter_monitor/.../cloud_ota_service.dart` | 通道过滤 |
| `scripts/ota/generate_manifest.py` | `--channel` / `--system-manifest` 参数 |
| `docs/OTA_GUIDE.md` | 新增 §16 v3 增强详解 + §17 Roadmap |

### 端到端测试验证

在实际机器人上完成完整闭环测试:

1. **Install**: `ApplyUpdate` → 模型文件 v1.0.0 → v2.0.0 替换成功
2. **Verify**: 文件内容 "THIS IS NEW MODEL v2.0.0" 确认
3. **State**: `installed_manifest.json` / `system_version.json` / `upgrade_history.jsonl` 正确更新
4. **Rollback**: 一键回滚 → 文件恢复为 "THIS IS OLD MODEL v1.0.0"
5. **History**: 升级历史包含 install + rollback 两条记录

---

## [v1.0.0] - 2026-02-08

首个稳定版本。涵盖从感知到控制的完整导航链路 + 远程监控 + OTA 更新。

### 新增

#### OTA 更新系统
- `DownloadFromUrl` — 机器人直接从 GitHub 下载，免手机中转
- `UploadFile` 断点续传 — WiFi 中断后可从断点继续
- `CheckUpdateReadiness` — 安装前预检查（磁盘/电量/硬件兼容/网络）
- `ApplyUpdate` — SHA256 校验 + 备份 + 安装 + manifest 管理
- `GetInstalledVersions` / `Rollback` — 版本查询与一键回滚
- `manifest.json` 格式定义与 `generate_manifest.py` 自动生成工具

#### OTA v2 产品级增强
- **Ed25519 签名链** — manifest 签名验证，防伪造；`generate_manifest.py --signing-key` 支持
- **安全等级分级** — HOT (地图/配置) / WARM (模型) / COLD (固件/MCU: sit → disable → 维护态)
- **原子安装 + 事务日志** — 写 `txn_{name}.json` → 安装 → 成功清理 / 崩溃自动回滚
- **依赖管理** — `dependencies` 字段表达制品间版本约束，CheckUpdateReadiness 自动检查
- **系统边界 (owner_module)** — brain / navigation / config_service / system / mcu 各负其责
- **用户体验规范** — 更新提示、COLD 确认流程、断电恢复、极端场景兜底
- 详见 [OTA_GUIDE.md](OTA_GUIDE.md)

#### 遥控避障 + /cmd_vel 仲裁
- **SafetyGate 近场避障** — 订阅 `/terrain_map` (odom 坐标系)，实时转 body 坐标系检测前方障碍
  - `obstacle_stop`：前方 < 0.8m 有超高障碍 → 线速度归零
  - `obstacle_slow`：前方 0.8~2.0m 有障碍 → 线性减速 `max(0.2, dist/2.0)`
  - 角速度不受影响，允许原地转向避让
- **模式门禁** — SafetyGate 仅在 TELEOP 模式下发布 `/cmd_vel`，从根本上消除与 pathFollower 的冲突
- **TELEOP 退出清除** — ModeManager.ExitState(TELEOP) 通过 SafetyGate 发零速度，清除残余 cmd_vel
- 参数：`obstacle_height_thre`, `stop_distance`, `slow_distance`, `vehicle_width`, `vehicle_width_margin`
- App 端通过 `TeleopFeedback.limit_reasons` 自动收到 "obstacle_stop" / "obstacle_slow" 原因

#### TaskManager 任务管理
- 航点队列：接收 N 个目标按序下发 `/way_point`
- 到达检测：订阅 `/Odometry`，欧氏距离 ≤ `arrival_radius` 判定到达
- 循环巡检：`INSPECTION` 类型自动 `loop=true`
- 状态机：IDLE → RUNNING → PAUSED → COMPLETED / FAILED / CANCELLED
- 进度回调 → EventBuffer → gRPC StreamEvents → App

#### 一键启动 Launch 文件
- `navigation_bringup.launch.py` — 建图模式
- `navigation_run.launch.py` — 运行模式（定位 + 自主导航）

#### 定位质量监控
- Localizer 发布 ICP fitness score → `/localization_quality`
- HealthMonitor 纳入判定：< 0.1 OK / < 0.3 DEGRADED / ≥ 0.3 CRITICAL

#### 近场急停
- local_planner 检测前方 0.5m 内障碍物 → 直接发布 `/stop=2`
- 带状态记忆，避免重复发布

#### Proto 健康/围栏扩展
- `telemetry.proto` 新增 `HealthStatus`、`GeofenceStatus` 消息
- `SlowState` 新增 `health` (field 8) 和 `geofence` (field 9)

### 改进

#### 系统安全架构升级 (2026-02-07)

8 项关键改进，从 "能用" 到 "敢户外用"：

| 项目 | 说明 |
|------|------|
| gRPC Relocalize/SaveMap | 空壳 → 实际调用 ROS 2 Service |
| 状态机守卫注入 | 7 个守卫从纸面变为代码 |
| 断联自动降级 | < 30s 正常 / 30s-5min 减速 50% / > 5min 停车 |
| terrain_map_ext 接入 | local_planner 合并连通性信息，避免死胡同 |

#### 四层解耦安全架构 (2026-02-06)

```
Layer 4: HealthMonitor   — 子系统健康聚合 + 自动降级
Layer 3: ModeManager     — 形式化状态机 (转换守卫矩阵)
Layer 2: GeofenceMonitor — 围栏越界检测 (射线法 + 三级预警)
Layer 1: Driver Watchdog — 底盘自保护 (200ms cmd_vel 超时)
```

核心原则：任何一层崩溃不影响其他层。4 条独立停车路径。

#### 数学优化 (2026-02-03)

| 改动 | 文件 | 收益 |
|------|------|------|
| 平面估计除零保护 | `commons.cpp` | 消除 NaN 传播 |
| IESKF `.inverse()` → `.ldlt().solve()` | `ieskf.cpp` | 数值稳定 |
| Jacobian Bug (`t_wi` → `t_il`) | `lidar_processor.cpp` | 修正偏导错误 |
| 删除重复 `transformPointCloud` | `lidar_processor.cpp` | 修复 T² 变换 |
| 缓存 `R_wi`/`R_wl` 到循环外 | `lidar_processor.cpp` | 省 N 次矩阵乘法 |
| 三步欧拉旋转 → 预计算矩阵 | `terrainAnalysis.cpp` | 每点 9 vs 18 乘法 |
| `sqrt(sqrt())` NaN 防御 | `localPlanner.cpp` | 负参数不再 NaN |
| `57.3` → `180.0/M_PI` | `lidar_processor.cpp` | 精确常量 |

### 修复

#### 坐标系统一修复 (2026-02-03)

**问题**: terrain_analysis / terrain_analysis_ext / local_planner 坐标系混用 — body 系点云与 odom 系位姿直接相减，输出声称 `map` 系实际不是。

**修复**:

| 模块 | 修复前 | 修复后 |
|------|--------|--------|
| terrain_analysis | 输入 `/cloud_registered` (body), 输出 `"map"` | 输入 `/cloud_map` (odom), 输出 `"odom"` |
| terrain_analysis_ext | 同上 | 同上 |
| local_planner | 输入 body+odom 混合, 输出 `"vehicle"` | 输入 odom, 转换后输出 `"body"` |
| pathFollower | 输出 `"vehicle"` | 输出 `"body"` |
| TF 树 | `sensor` → `vehicle` (不连续) | `body` → `lidar` → `camera` (完整) |

修复后 TF 树：`map → odom → body → {lidar, camera}`

**验证方法**:
```bash
ros2 topic echo /terrain_map --field header.frame_id --once  # 期望: odom
ros2 topic echo /path --field header.frame_id --once          # 期望: body
ros2 run tf2_tools view_frames                                 # 期望: 完整连续
```

---

## 待办事项

### 高优先级
- [ ] colcon 完整编译验证
- [ ] Proto 重新生成 Dart 代码 (`scripts/proto_gen.sh`)
- [ ] Flutter App 健康/围栏/巡检 UI

### 中优先级
- [ ] pct_adapters 到达事件（更精准的航点切换）
- [ ] TaskManager JSON 解析升级（nlohmann/json）
- [ ] 断联降级阈值可配置化
- [ ] 近场急停距离参数化
- [ ] 定位质量阈值实际标定

### 低优先级
- [ ] rosbag 集成（gRPC 触发录制）
- [ ] BehaviorTree.CPP 替代 ModeManager
- [ ] localization_valid 守卫结合 ICP score
- [ ] 多机器人协调
- [ ] 仿真测试框架 (Gazebo/Isaac Sim)

---

## 文件变更汇总

### OTA 更新 (2026-02-08)

| 文件 | 变更 |
|------|------|
| `robot_proto/proto/data.proto` | OTA RPCs + 消息; **v2: +OtaSafetyLevel, +ArtifactDependency, +OtaTransactionLog** |
| `remote_monitoring/src/services/data_service.cpp` | OTA 实现; **v2: +安全等级检查, +依赖检查, +事务日志** |
| `remote_monitoring/include/.../data_service.hpp` | OTA 声明 |
| `remote_monitoring/CMakeLists.txt` | +OpenSSL 依赖 |
| `remote_monitoring/config/grpc_gateway.yaml` | +OTA 参数 |
| `client/flutter_monitor/lib/core/grpc/robot_client.dart` | OTA 客户端 |
| `client/flutter_monitor/lib/features/settings/cloud_ota_service.dart` | manifest 解析 |
| `docs/OTA_GUIDE.md` | **v2 重写: 产品级规范 (签名/安全等级/原子安装/依赖/UX)** |
| `scripts/ota/manifest_template.json` | **v2: +signature, +dependencies, +safety_level** |
| `scripts/ota/generate_manifest.py` | **v2: +Ed25519 签名, +密钥生成** |

### 系统升级 (2026-02-07)

| 文件 | 变更 |
|------|------|
| `launch/navigation_bringup.launch.py` | 新增：建图模式启动 |
| `launch/navigation_run.launch.py` | 新增：运行模式启动 |
| `remote_monitoring/src/core/task_manager.{hpp,cpp}` | 新增：任务管理器 |
| `remote_monitoring/src/services/system_service.{hpp,cpp}` | Relocalize/SaveMap + 心跳 |
| `remote_monitoring/src/services/control_service.{hpp,cpp}` | StartTask/CancelTask |
| `remote_monitoring/src/grpc_gateway.{hpp,cpp}` | 守卫注入 + 断联降级 |
| `remote_monitoring/src/core/health_monitor.{hpp,cpp}` | +定位质量 |
| `remote_monitoring/src/status_aggregator.{hpp,cpp}` | +健康/围栏 |
| `robot_proto/proto/telemetry.proto` | +HealthStatus/GeofenceStatus |
| `base_autonomy/local_planner/src/localPlanner.cpp` | +terrain_map_ext + 近场急停 |
| `slam/localizer/src/localizers/icp_localizer.{h,cpp}` | +fitness score |
| `slam/localizer/src/localizer_node.cpp` | +/localization_quality |

### 安全架构 (2026-02-06)

| 文件 | 变更 |
|------|------|
| `remote_monitoring/src/core/geofence_monitor.{hpp,cpp}` | 新增：围栏监控 |
| `remote_monitoring/src/core/health_monitor.{hpp,cpp}` | 新增：健康监控 |
| `remote_monitoring/src/core/mode_manager.{hpp,cpp}` | 重写：形式化状态机 |
| `drivers/robot_driver/driver_node.py` | 重写：独立看门狗 |
| `base_autonomy/local_planner/src/pathFollower.cpp` | /stop max 优先级 |

### 数学优化 + 坐标修复 (2026-02-03)

| 文件 | 变更 |
|------|------|
| `slam/fastlio2/src/map_builder/commons.cpp` | 除零保护 |
| `slam/fastlio2/src/map_builder/ieskf.cpp` | LDLT 分解 |
| `slam/fastlio2/src/map_builder/lidar_processor.cpp` | Jacobian + 缓存 + 常量 |
| `base_autonomy/terrain_analysis/src/terrainAnalysis.cpp` | 坐标修复 + 旋转优化 |
| `base_autonomy/terrain_analysis_ext/src/terrainAnalysisExt.cpp` | 坐标修复 |
| `base_autonomy/local_planner/src/localPlanner.cpp` | 坐标修复 + NaN 防御 |
| `base_autonomy/local_planner/src/pathFollower.cpp` | frame_id 修复 |
| `base_autonomy/local_planner/launch/local_planner.launch` | TF 发布器修正 |

---

*最后更新: 2026-02-11*
