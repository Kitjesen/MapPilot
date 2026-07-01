# PRD：LingTu 原生 SLAM + 导航运行时

## 1. 摘要

这份 PRD 定义 LingTu 从“ROS 节点拼接”走向“Module-First 原生运行时”的产品和工程要求。

目标很明确：机器人先获取传感器数据，完成建图或加载地图，稳定定位，再由 OctoPlanner3D 规划全局路径，局部规划器跟踪路径，经过安全检查后，才允许速度命令到达机器狗驱动。

SLAM 只负责定位、地图、传感器观测和健康状态。SLAM 不产出导航路径，不产出 `cmd_vel`，也不直接控制机器狗。

## 2. 角色与职责（Stakeholders / Ownership）

英文里这里更应该写 `Stakeholders` 或 `Ownership`。`Contacts` 只是联系人列表，不适合指导工程实现。本节定义每个角色的决策权、交付物和验收责任。

原则：每条产品链路必须有一个主 owner。不能把一条链路拆给很多模块以后，最后没人回答“为什么机器人这样动，为什么不动，谁允许它动”。

| 角色 | 决策权 | 主要职责 | 必须交付 / 验收 |
| --- | --- | --- | --- |
| 产品负责人 | 决定六个产品模式的定义、优先级、上线范围。 | 收敛产品模式：遥控避障、遥控、建图、跟踪、导航、巡检；确认目标点不能直接变成电机命令；确认 Web/CLI/MCP 只发请求和显示状态。 | 发布前确认 profile 命名、用户流程、失败提示、验收门槛都可被现场操作者理解。 |
| 系统架构负责人 | 决定 Module-First 分层、跨层 wire、兼容层位置。 | 维护 `full_stack_blueprint()`、`compose_full_stack_modules()`、`apply_full_stack_wires()`；防止业务模块重新依赖 ROS；把 native path 和 explicit adapter path 分清楚。 | profile graph 与实际运行图一致；无隐式 ROS fallback；每条关键数据线和控制线有唯一 owner。 |
| 运行模式 / Profile 负责人 | 决定 `map/nav/explore/tare_explore` 等 profile 如何映射到六个产品模式。 | 维护 `cli/profiles_data.py`、`runtime_profiles.py`、profile catalog；定义 mode 切换、互斥、启动前检查；确认 `mode_cmd` 是被消费的控制命令，或降级为只读状态。 | 六个模式的启动命令、模块图、关键 wires 可审计；不存在“前端发了 mode，但后端没人处理”的空入口。 |
| Gateway / Web / MCP 负责人 | 决定操作入口、命令 schema、状态审计展示。 | 维护 Web/REST/WS/MCP 入口；Web 点击目标只发 `goal_pose` 请求，不算路径；遥控入口只发 teleop intent；状态页展示 phase、reason、failure code、active command source。 | `/ws/teleop`、`/api/v1/cmd_vel`、`/api/v1/mode`、MCP goal 都有明确后端消费方、拒绝原因和日志。 |
| 遥控控制负责人 | 决定遥控和遥控避障的输入语义、接管、释放、降级行为。 | 维护 `TeleopModule`、teleop WS/REST 到 `VelocityMux` 的链路；定义 joystick 归一化、超时释放、zero-on-release、无 `TeleopModule` 时 fallback；区分普通遥控和 guarded teleop。 | WS/REST -> Gateway -> Teleop -> CmdVelMux -> driver 链路可跑通；安全 stop 能阻断遥控；释放后必须零速并可追踪 active source。 |
| 驱动和传感器负责人 | 决定 Livox、IMU、GNSS、相机数据如何进入 Module 图。 | 使用官方 Livox-SDK2；保留 `offset_time_ns`、`line`、`tag`；提供 packet age、point rate、IMU rate、drop、time-sync 状态；维护传感器 frame 到 body 的标定。 | MID-360 原生接入可在 S100P 启动；mock source 可复现；传感器 stale 或 sync bad 能产生 typed failure。 |
| SLAM / 定位负责人 | 决定 Fast-LIO2 / Point-LIO 如何实现 `ISlamBackend`，以及建图、跟踪、重定位状态机。 | 实现 `feedImu()`、`feedLidar()`、`tick()`、`outputs()`、`saveMap()`、`loadMap()`、`relocalize()`；输出 odometry、map_cloud、saved_map、quality、map_odom_tf、map jump、GNSS health；不输出路径或速度。 | 原生 SLAM 无 ROS 节点启动；LiDAR/IMU sync 可验证；建图和跟踪日志能区分来源；漂移、重定位、失败分类通过验收。 |
| 在线地图层负责人 | 决定 Occupancy、Voxel、ESDF、Elevation、Traversability 的在线数据合同。 | 维护 L2 在线地图模块；消费 `map_cloud/odometry`，输出 occupancy、voxel、esdf、elevation、fused cost、exploration grid；不负责地图保存和机器人运动。 | 地图层断开、超时、空输入能产生健康告警；Navigation/LocalPlanner/Gateway 能看到同一套代价和可视化数据。 |
| 地图服务负责人 | 决定地图 bundle、版本、产物和生命周期。 | 维护 `MapService`、map record、active map、artifact gate；生成 `map.pcd`、metadata、OctoMap/tomogram/occupancy；定义 `EMPTY/BUILDING/READY/ACTIVE/STALE/FAILED`。 | `map save/use/delete/rename` 产物完整；地图版本和 calibration hash 可追踪；artifact 失败会阻止 nav 使用该图。 |
| 导航任务负责人 | 决定 `goal_pose` 进入后端后的 mission 生命周期、状态机、恢复策略。 | 维护 `NavigationModule` / `nav.mission.*`；接收 Gateway/CLI/MCP/Semantic/Exploration 的目标；负责 PLANNING、EXECUTING、RECOVERY、SUCCESS、FAILED、CANCELLED 等状态与 reason。 | 任意目标来源都统一进入 Navigation；SLAM/ODOM 不可信时明确拒绝或降级；mission status 能解释“不动、重规划、卡住、取消”。 |
| 全局规划负责人 | 决定 OctoPlanner3D 的输入输出合同和约束模型。 | 维护 planner service；接收 start pose、goal pose、map artifacts、robot constraints、cost inputs；输出 `global_path`、`adjusted_goal`、latency、cost、map_version、rejected_reason。 | OctoPlanner3D 是产品默认；PCT/direct 只能是显式实验；规划延迟和失败原因可被 QA 验证。 |
| 局部规划和路径跟踪负责人 | 决定 LocalPlanner、PathFollower、速度候选的实现。 | 把 `global_path/waypoint/traversability/esdf` 转成 `local_path` 和 `cmd_vel` 候选；处理 stuck、oscillation、waypoint timeout；不绕过 CmdVelMux。 | 局部规划执行中 >= 10 Hz；`cmd_vel` 候选到 driver 的 p95 延迟满足门槛；不接受 SLAM path 当 local path。 |
| 巡检 / 探索负责人 | 决定巡检目标从哪里来、什么时候切换、什么时候停止。 | 维护 frontier、TARE、patrol goal sequence；TARE/frontier 只生成 `exploration_goal` 或 `patrol_goals`，不接管导航执行；定义空目标、重复目标、远目标、卡死时的回填策略。 | `explore` 和 `tare_explore` 不双写冲突；目标来源、goal_id、切换原因、终止原因可观测；巡检记录可回放。 |
| 感知 / 语义 / 记忆负责人 | 决定自然语言、语义目标、对象目标如何变成导航目标。 | 维护 detector、semantic map、memory、`SemanticPlannerModule`、agent loop；处理歧义、低置信、找不到对象、LERa recovery；只输出 goal/servo intent，不直接控制驱动。 | 语义目标能解释“为什么去这里”；低置信时拒绝或请求确认；语义 goal 与普通 goal 走同一 Navigation 链路。 |
| 安全仲裁负责人 | 决定 SafetyRing、Geofence、CmdVelMux、VelocityLimiter 的规则。 | 维护命令优先级、timeout、zero-on-timeout、非有限值处理、e-stop、geofence、定位丢失时的 stop/hold 策略；定义 teleop 与 autonomy 的抢占和恢复。 | Safety stop p95 <= 100 ms；SLAM `LOST/FAILED` 时 autonomy 停止；所有 driver command 都经过安全仲裁。 |
| QA / 发布负责人 | 决定仿真、replay、实机的发布门槛。 | 维护 Python/C++ contract test、profile graph test、MuJoCo gate、S100P field gate、timing/frame/safety acceptance；保存验收记录。 | 13 节验证矩阵全部通过或显式豁免；不能用人工口头确认替代验收记录。 |
| 现场操作者 | 决定现场是否继续运行、停止、保存地图或重试任务。 | 使用 CLI/Web/MCP 启动遥控避障、遥控、建图、跟踪、导航、巡检；观察状态；在失败时按 recommended_action 处理。 | 能看懂机器人为什么不动、谁在控制、地图是否可用、是否需要重定位或急停解除。 |

### 2.1 六个产品模式的 Owner 矩阵

| 产品模式 | 主 owner | 必须协作 | 必须证明 |
| --- | --- | --- | --- |
| 遥控避障模式 | 遥控控制负责人 | Gateway / Web / MCP 负责人、安全仲裁负责人、在线地图层负责人、驱动和传感器负责人 | 人工输入不直接到 driver；近场安全、geofence、e-stop 可限制或阻断速度；`active_source=guarded_teleop` 或等价状态可见。 |
| 遥控模式 | 遥控控制负责人 | Gateway / Web / MCP 负责人、安全仲裁负责人、驱动和传感器负责人 | WS/REST 遥控链路跑通；teleop 优先级最高；断开、超时、stop 都会零速；不生成 `global_path/local_path`。 |
| 建图模式 | SLAM / 定位负责人 | 驱动和传感器负责人、在线地图层负责人、地图服务负责人、遥控控制负责人、安全仲裁负责人 | Livox/IMU -> SLAM -> `map_cloud/saved_map` -> MapService 保存；建图移动仍经过 CmdVelMux/SafetyRing；地图产物和 metadata 完整。 |
| 跟踪模式 | SLAM / 定位负责人 | 地图服务负责人、在线地图层负责人、安全仲裁负责人、Gateway / Web / MCP 负责人 | 加载 active map 后持续输出 odometry、localization_quality、map_odom_tf；LOST/DEGRADED/RELOCALIZING 可见；不产出导航路径。 |
| 导航模式 | 导航任务负责人 | 全局规划负责人、局部规划和路径跟踪负责人、SLAM / 定位负责人、地图服务负责人、安全仲裁负责人、Gateway / Web / MCP 负责人 | `goal_pose` 从任意入口进入同一 mission；OctoPlanner3D 产生 `global_path`；LocalPlanner/PathFollower 产生速度候选；最终命令由 CmdVelMux/SafetyRing 放行。 |
| 巡检模式 | 巡检 / 探索负责人 | 感知 / 语义 / 记忆负责人、导航任务负责人、全局规划负责人、局部规划和路径跟踪负责人、安全仲裁负责人、QA / 发布负责人 | frontier/TARE/语义/巡逻点只生成目标序列；执行仍走 Navigation；每个 goal 有来源、原因、执行结果、停止原因和任务记录。 |

### 2.2 产品负责人交付物

产品负责人先把产品边界定死。工程负责人按这个边界做，不再各层自己猜。

| 交付项 | 决策 |
| --- | --- |
| 产品模式 | 只保留六个一层模式：遥控避障、遥控、建图、跟踪、导航、巡检。`explore`、`tare_explore` 是巡检模式里的目标生成策略，不再作为一层产品模式解释。 |
| 上线优先级 | P0：遥控、建图、跟踪、导航。P1：遥控避障、巡检。原因：P0 先保证“能安全接管、能建图、能定位、能点到点导航”；P1 再做近场辅助和多目标任务。 |
| 操作入口 | Web、CLI、MCP 都只能发请求：teleop intent、map command、goal_pose、inspection command。它们不能算路径，不能绕过 Navigation，不能直接写 driver。 |
| 目标点原则 | 目标点必须先进入 Navigation mission。起点来自 SLAM/odometry 当前位姿，不从前端手填，除非仿真或调试工具显式标记。 |
| 运动控制原则 | 任何速度命令都必须经过 CmdVelMux 和 SafetyRing。前端、语义 planner、TARE、frontier 都不能直接控制机器狗。 |
| 发布门槛 | 六个模式在状态页都能显示：当前模式、控制来源、phase、reason、失败原因、推荐动作。现场操作者不看日志也能知道机器人为什么动或为什么不动。 |

P0 验收：

| 模式 | 验收口径 |
| --- | --- |
| 遥控 | WS/REST/MCP 遥控能接管；断开、超时、stop 都会零速；安全 stop 能阻断遥控。 |
| 建图 | 能从 Livox/IMU/SLAM 生成 `map_cloud`，保存地图 bundle，并能在 Web/CLI 看见保存结果。 |
| 跟踪 | 能加载 active map，持续输出 odometry、localization_quality、map_odom_tf；定位丢失时状态明确。 |
| 导航 | 点击目标或 CLI/MCP 发目标后，链路必须是 goal -> Navigation -> OctoPlanner3D -> LocalPlanner/PathFollower -> CmdVelMux/SafetyRing -> driver。 |

P1 验收：

| 模式 | 验收口径 |
| --- | --- |
| 遥控避障 | 人工方向输入被近场安全、地形代价、geofence 限制；状态页显示 blocked/limited reason。 |
| 巡检 | frontier/TARE/语义目标只生成目标序列；执行复用导航模式；每个 goal 有来源、原因、结果和停止原因。 |

## 3. 背景

LingTu 当前正在从“固定 ROS 版本 + 外部节点拼接”迁移到 Module-First 架构。

已有基础：

| 区域 | 当前状态 |
| --- | --- |
| SLAM Module 合同 | 已有 `SlamModule`，输出 odometry、map_cloud、saved_map、localization_status、gnss_fusion_health、map_odom_tf 等。 |
| 导航分离 | 已明确 SLAM 不输出 `global_path`、`local_path`、`waypoint`、`cmd_vel`、`path`。 |
| C++ SLAM 接口 | 已有 `src/localization/slam/cpp/slam.hpp`，但目前主要是合同层。 |
| Fast-LIO2 / Point-LIO 原生后端 | 还没有真正接到 `ISlamBackend` 后面。 |
| Livox 到 SLAM | 官方 Livox-SDK2 到 `feedLidar/feedImu` 的原生链路还没完成。 |
| Profile graph | 仍有测试和文档期待 `SlamBridgeModule`，需要更新。 |
| ROS2 bridge | 仍保留在显式兼容路径里，不能作为隐式 fallback。 |
| 地图生命周期 | 有地图产物和 MapService，但原生 save/load/relocalize 还不够完整。 |
| 全链路验证 | 有合同测试，但缺真实 native SLAM + MuJoCo + S100P 闭环验证。 |

真正的问题不是“某个文件错了”，而是系统语义还没有完全收敛。现在数据会穿过 driver、adapter、SLAM、maps、navigation、local planner、safety、gateway 和旧兼容代码。如果没有明确状态机、时间坐标合同、失败分类和验收门槛，实现时每一层都会自己猜。

## 4. 目标

建立一条产品主链路：

```text
Livox MID-360 + IMU
  -> 原生 SlamModule
  -> odometry + registered_cloud + map_cloud + saved_map + health
  -> 在线地图层 + Navigation
  -> OctoPlanner3D global_path + waypoint
  -> LocalPlanner local_path
  -> PathFollower cmd_vel 候选
  -> SafetyRing + CmdVelMux
  -> 机器狗驱动
```

兼容路径必须显式：

```text
外部 ROS2 / LCM localization
  -> explicit adapter
  -> 同一套下游端口
```

### 关键结果

| 结果 | 目标 |
| --- | --- |
| 无隐式 ROS 依赖 | 产品 profile 不默认依赖 ROS2，除非显式设置 compatibility adapter。 |
| 主链路去 ROS 化 | Livox、SLAM、地图、规划、局部跟踪、安全仲裁的产品链路不通过 ROS topic/service/tf 传递核心数据。 |
| 核心算法函数调用 | 传感器回调直接喂 `feedImu/feedLidar`，SLAM、planner、local planner 通过明确函数/Module 端口调用，不靠外部 ROS 节点串联。 |
| 原生 SLAM 后端 | Fast-LIO2 实现 `ISlamBackend`，无需启动 ROS 节点。 |
| 可替换 LIO | Point-LIO 实现同一接口，下游 Navigation/Gateway 不变。 |
| Livox 原始字段保真 | `offset_time_ns`、`line`、`tag` 保留到 SLAM 消费。 |
| 地图生命周期完整 | `saveMap()` 写 `map.pcd`、`poses.txt`、`patches/`，MapService 写元数据和规划产物。 |
| Mapping 有控制语义 | 建图支持手动和辅助移动，但所有运动都经过 CmdVelMux 和 SafetyRing。 |
| 运行时状态明确 | SLAM、Mapping、Navigation、Safety 都有状态、事件、原因和拒绝行为。 |
| 时间坐标合同明确 | 所有 profile 遵守统一 timestamp、frame、sync 规则。 |
| 失败分类明确 | 失败有 category/code/severity/recoverable/action，不再只有泛泛的 health。 |
| 导航责任清楚 | SLAM 输出不能接成 `global_path`、`local_path`、`waypoint` 或 `cmd_vel`。 |
| Profile graph 可信 | 静态 profile graph 和运行时实际模块一致。 |
| 全链路可验证 | 能证明 sensor -> SLAM -> map -> global plan -> local plan -> cmd_vel -> driver。 |

## 5. 用户和使用场景

| 用户 | 任务 |
| --- | --- |
| 现场操作者 | 启动建图或导航，并知道机器人是否可以安全移动。 |
| 机器人算法工程师 | 替换 SLAM 或 planner，不需要猜跨层 wiring。 |
| 硬件集成工程师 | 接入 Livox / IMU / GNSS，不依赖固定 ROS2 runtime。 |
| QA / 发布负责人 | 在上车前验证每个 product profile。 |
| Dashboard 用户 | 点击目标、查看地图/路径/状态，并相信后端负责导航决策。 |

## 6. 价值

| 需求 | 价值 |
| --- | --- |
| 职责清楚 | SLAM 管定位和地图，Navigation 管路径和控制候选。 |
| 少混乱 | 一条 native 主链路，一条显式兼容链路。 |
| 硬件数据完整 | Livox timing/channel/tag 不丢，支持 deskew。 |
| 更安全 | 目标必须经过规划、跟踪、安全检查，不能直接变成电机命令。 |
| 好调试 | 状态里有 phase 和 reason，不只是 failed。 |
| 可移植 | Ubuntu 版本变化不应该强依赖固定 ROS2 组合。 |

## 7. 方案

### 7.0 分层合同

系统要按层读，而不是追文件跳转。每层只拥有一种决策。

| 层级 | Runtime owner | 输入 | 输出 | 不允许拥有 |
| --- | --- | --- | --- | --- |
| L0 Safety / 命令仲裁 | SafetyRing、Geofence、CmdVelMux | 命令候选、e-stop、geofence、定位质量、驱动健康 | 允许的 driver command、stop command、blocked reason、command source | 全局路径、局部路径、建图策略 |
| L1 硬件和定位 | Driver、Camera、Livox、GNSS、SlamModule | 原始传感器包、标定、初始位姿、地图加载请求 | odometry、registered_cloud、map_cloud、saved_map、定位状态、诊断流 | 导航路径、局部路径、`cmd_vel` |
| L2 在线地图和地图生命周期 | OccupancyGrid、VoxelGrid、ESDF、ElevationMap、TraversabilityCost、MapService | map_cloud、registered_cloud、odometry、saved_map、map_command | occupancy、voxel、ESDF、elevation、fused cost、地图产物、元数据 | 机器人运动命令 |
| L3 感知和语义记忆 | detector、reconstruction、semantic mapper、memory | 图像、深度、点云、地图、里程计 | objects、scene graph、semantic map、memory matches | 最终运动控制 |
| L4 决策和任务意图 | semantic planner、exploration policy、patrol policy | 操作意图、语义状态、frontier/map 状态 | goal_pose、exploration_goal、task reason | 直接电机命令 |
| L5 导航规划和跟踪 | Navigation、OctoPlanner3D、LocalPlanner、PathFollower | goal_pose、odometry、地图产物、可通行代价、安全反馈 | global_path、waypoint、local_path、cmd_vel 候选、mission_status | 硬件 IO、SLAM 建图 |
| L6 接口和运维 | Gateway、CLI、MCP、Web UI | 点击、地图控制、遥控输入、操作命令 | goal request、map command、teleop command、状态显示 | 路径规划、避障、安全 override |

分层不变量：

- 前端可以启动建图、发送目标、发送遥控、显示状态，但不能算路径或绕障。
- SLAM 可以发布 pose、map、health、历史轨迹诊断，但不能发布路径或速度命令。
- MapService 管地图产物和元数据，不能控制机器人移动。
- Mapping 的移动只有两个合法来源：
  - 手动建图：operator teleop -> CmdVelMux -> SafetyRing/Driver
  - 辅助建图：exploration/task goal -> Navigation -> LocalPlanner -> PathFollower -> CmdVelMux -> SafetyRing/Driver

### 7.1 运行时装配

系统由一个产品图装配：

```text
full_stack_blueprint()
  -> compose_full_stack_modules()
       external_services
       device_manager
       driver
       lidar
       sim_lidar
       slam
       gnss
       maps
       perception
       memory
       planner
       services
       navigation
       exploration
       safety
       gateway
  -> apply_full_stack_wires()
```

关键代码 seam：

| Seam | 文件 | 作用 |
| --- | --- | --- |
| Product graph | `src/runtime/blueprints/full_stack.py` | full-stack 公开装配入口。 |
| Stack composition | `src/runtime/blueprints/stacks/composition.py` | 按 profile 选择哪些 stack 存在。 |
| Explicit wires | `src/runtime/blueprints/full_stack_wiring.py` | 应用跨 stack 数据线和控制线。 |
| Product profiles | `src/runtime/profiles/catalog/product_intents.py` | 声明产品模式和默认值。 |
| Simulation profiles | `src/runtime/profiles/catalog/simulation_profiles.py` | 声明仿真和开发验证模式。 |

### 7.2 产品模式

| 模式 | 目的 | 主链路 | 输出 |
| --- | --- | --- | --- |
| 遥控避障模式 | 人工给方向，系统负责近场安全约束。 | teleop intent -> local obstacle check -> CmdVelMux -> SafetyRing -> driver | 受限人工运动 |
| 遥控模式 | 人工直接接管移动，用于调试、回收、建图移动。 | teleop cmd_vel candidate -> CmdVelMux -> SafetyRing -> driver | 人工控制运动 |
| 建图模式 | 采集环境并生成可复用地图。 | sensors -> SlamModule(MAPPING) -> maps -> MapService | saved map bundle + planner artifacts |
| 跟踪模式 | 加载保存地图并持续输出可信定位。 | active map + sensors -> SlamModule(LOCALIZATION/TRACKING) -> odometry/status | odometry、localization_quality、map_odom_tf |
| 导航模式 | 执行点到点目标。 | goal_pose -> Navigation -> OctoPlanner3D -> LocalPlanner -> PathFollower -> CmdVelMux -> SafetyRing -> driver | 到目标的受控运动 |
| 巡检模式 | 按任务、语义目标、frontier 或外部策略连续生成导航目标。 | inspection policy -> goal sequence -> Navigation -> local planning -> safety -> driver | 多目标任务执行记录 |

### 7.3 按功能的数据流

数据流必须按功能理解，不按某个底层 callback 理解。Livox-SDK2 callback 只是 L1 传感器入口的一种实现；产品语义上真正重要的是遥控避障、遥控、建图、跟踪、导航、巡检六条模式链路。

#### 遥控避障模式数据流

目标：操作者给出运动方向，系统根据近场障碍、地形、电子围栏和定位安全限制速度，避免人工误操作直接撞障。

```text
operator joystick / Web teleop / MCP teleop
  -> Gateway or TeleopModule receives teleop intent
  -> normalize to desired Twist
  -> local obstacle / traversability / geofence check
  -> velocity limiter clamps speed, acceleration, and angular rate
  -> CmdVelMux marks guarded teleop as active source
  -> SafetyRing evaluates e-stop, localization, geofence, driver health
  -> Driver receives final cmd_vel only when safe
  -> Gateway shows active_source = guarded_teleop and blocked_reason when limited
```

遥控避障模式仍然不是 Navigation mission。它不生成 `global_path`，只对人工速度意图做近场安全约束。

#### 遥控模式数据流

目标：让操作者在建图、调试、接管、紧急避让时直接给出运动意图，但仍然不能绕过安全仲裁。

```text
operator joystick / Web teleop / MCP teleop
  -> Gateway or TeleopModule receives teleop input
  -> normalize to Twist cmd_vel candidate
  -> CmdVelMux marks teleop as active source with priority 100
  -> SafetyRing / Geofence evaluates localization, e-stop, geofence, driver health
  -> Driver receives final cmd_vel only when safety allows
  -> Gateway shows active_source = teleop and safety_state
```

遥控模式不进入 Navigation mission，不产生 `global_path`，不产生 `local_path`。它是人工控制候选源，只在 CmdVelMux 中以最高优先级参与仲裁。

#### 建图模式数据流

目标：把现场传感器观测变成可保存、可复用、可规划的地图包。

```text
map profile starts
  -> MappingSession enters MAPPING_IDLE
  -> SlamModule.setMode(MAPPING)
  -> sensor samples feed SLAM
  -> SlamModule outputs odometry + registered_cloud + map_cloud + localization_status
  -> online map layers update occupancy / voxel / elevation / ESDF / traversability
  -> Gateway shows live map and SLAM quality
  -> operator map save
  -> MappingSession enters SAVING_MAP and blocks new autonomous goals
  -> SlamModule.save_map()
  -> MapService writes metadata and builds planner artifacts
  -> map record enters READY
```

Mapping 的控制流是旁路，不属于 SLAM：

```text
manual mapping:
  teleop -> CmdVelMux -> SafetyRing -> Driver

assisted mapping:
  assisted_goal -> Navigation -> LocalPlanner -> PathFollower -> CmdVelMux -> SafetyRing -> Driver
```

#### 跟踪模式数据流

目标：使用保存地图持续输出可信定位，让 Navigation 和 Safety 判断是否允许运动。

```text
nav profile starts with active map
  -> MapService resolves map bundle and artifact gate
  -> SlamModule.setMode(LOCALIZATION, map_path)
  -> SlamModule.load_map()
  -> optional setInitialPose() or relocalize()
  -> sensor samples feed SLAM
  -> SlamModule outputs odometry + localization_quality + map_odom_tf + map_frame_jump_event
  -> Navigation consumes odometry and map jump events
  -> SafetyRing consumes localization_status and gnss_fusion_health
  -> Gateway/MCP shows tracking state, quality, reason, and failure code
```

跟踪模式不产出导航路径。旧 `/path` 只能当历史轨迹诊断，不能接成 `local_path`。

#### 导航模式数据流

目标：把目标点变成受安全约束的运动命令。

```text
Gateway / CLI / MCP / semantic / exploration sends goal_pose
  -> Navigation mission accepts goal
  -> start_pose comes from SlamModule.odometry
  -> GlobalPlanner.plan(start_pose, goal_pose, map_artifacts, robot_constraints)
  -> Navigation publishes global_path and waypoint
  -> LocalPlanner.step(global_path, waypoint, terrain, esdf)
  -> PathFollower.step(local_path, odometry)
  -> CmdVelMux selects active cmd_vel candidate
  -> SafetyRing / Geofence gate command
  -> Driver receives final cmd_vel
```

#### 巡检模式数据流

目标：把一组巡检任务或语义目标变成连续导航任务，并记录每个目标的执行结果。

```text
inspection profile starts
  -> inspection task loads route / semantic targets / frontier policy / external TARE goal source
  -> goal resolver produces next goal_pose
  -> Navigation executes one goal through normal navigation mode
  -> perception / memory / mission logger record observations and result
  -> next goal is selected or task finishes
  -> Gateway/MCP shows task progress, current goal, failure reason, and evidence
```

巡检模式自己不控制底盘。它只产生目标序列和任务记录；每个目标仍然进入导航模式执行。`explore`、`tare_explore` 属于巡检模式的目标生成策略，不再作为第一层产品模式理解。

#### 共享数据

| 数据 | 生产者 | 消费者 |
| --- | --- | --- |
| `odometry` | SlamModule 或 driver fallback | Navigation、local planner、path follower、maps、semantic memory、safety、Gateway/MCP |
| `map_cloud` | SlamModule、sim point cloud、sim driver | occupancy、voxel、elevation、terrain、Gateway、Rerun |
| `localization_status` | SlamModule / explicit adapter | Navigation、SafetyRing、DepthVisualOdom、Gateway |
| `gnss_fusion_health` | SlamModule / GNSS fusion | SafetyRing、Gateway |
| `scene_graph` | PerceptionModule | Gateway、MCP、memory、SemanticPlanner、VisualServo、frontier modules |
| `fused_cost` / `esdf_field` | TraversabilityCostModule | Navigation、local planner、Gateway、frontier modules |

#### 硬件入口实现

```text
Livox-SDK2 callback / replay / sim source
  -> normalized ImuSample + LidarFrame
  -> SlamRunner.feedImu()/feedLidar()
```

这只是把传感器样本送进 SLAM 的实现方式。PRD 和产品调试优先看 Mapping、Tracking/Localization、Navigation 的功能数据流。

### 7.4 控制面

控制面包含意图、目标、路径、速度候选和最终命令仲裁。

```text
operator / MCP / semantic planner / exploration
  -> goal_pose or instruction
  -> Navigation mission
  -> OctoPlanner3D global_path
  -> LocalPlanner local_path
  -> PathFollower cmd_vel candidate
  -> CmdVelMux
  -> SafetyRing + driver

teleop
  -> cmd_vel candidate
  -> CmdVelMux
  -> SafetyRing + driver
```

控制不变量：

- `goal_pose` 是请求，不是电机命令。
- `global_path` 只能由 Navigation / global planner 产出。
- `local_path` 只能由 LocalPlanner 产出。
- `cmd_vel` 候选可以来自普通遥控、遥控避障、visual servo、recovery、path follower。
- 只有 CmdVelMux 选择当前命令源。
- 普通遥控和遥控避障都是人工接管模式，二者互斥；同一时刻只能启用一个。
- 遥控优先级最高，但不是安全 override；SafetyRing 仍可阻止它。
- SafetyRing 和 geofence 可以无条件停止运动。

### 7.5 用户流程

#### 遥控避障模式

```text
operator opens guarded teleop
  -> TeleopModule session becomes active
  -> joystick payload becomes desired Twist
  -> local obstacle / traversability / geofence check limits the command
  -> CmdVelMux selects guarded teleop when fresh
  -> SafetyRing checks e-stop, localization safety, driver health
  -> driver receives final cmd_vel if allowed
  -> Gateway shows whether command was passed, clamped, or blocked
```

遥控避障适合人工移动但希望系统帮忙兜底的场景。它不是自主导航，不承诺自动绕到目标点。

#### 遥控模式

```text
operator opens Web/CLI teleop
  -> TeleopModule session becomes active
  -> joystick payload becomes teleop cmd_vel candidate
  -> CmdVelMux selects teleop when fresh because priority is highest
  -> SafetyRing checks e-stop, geofence, localization safety, driver health
  -> driver receives final cmd_vel if allowed
  -> timeout or release publishes zero / falls back to next active source
```

遥控用于人工接管、建图移动和调试。遥控不代表可以绕过急停、电子围栏、速度限制或驱动安全状态。

#### 建图模式

```text
operator starts map profile
  -> MappingSession enters MAPPING_IDLE
  -> Livox source starts
  -> SlamModule enters MAPPING
  -> operator chooses control mode
       manual: teleop command -> CmdVelMux -> Driver
       assisted: exploration goal -> Navigation -> LocalPlanner -> CmdVelMux -> Driver
  -> map_cloud updates online maps and Gateway
  -> operator can pause motion, resume motion, or emergency stop
  -> operator saves map
  -> MappingSession enters SAVING_MAP and blocks new motion goals
  -> SlamModule writes map.pcd + poses.txt + patches/
  -> MapService writes metadata and planner artifacts
  -> MappingSession enters MAP_READY
```

Mapping 状态：

| 状态 | 含义 |
| --- | --- |
| `MAPPING_IDLE` | 建图会话已启动，但不移动。 |
| `MAPPING_TELEOP` | 操作者遥控移动，同时 SLAM 建图。 |
| `MAPPING_ASSISTED` | exploration/navigation 驱动移动，同时 SLAM 建图。 |
| `MAPPING_PAUSED` | 停止运动，SLAM 可继续观测。 |
| `SAVING_MAP` | 正在保存地图，阻止新运动目标。 |
| `MAP_READY` | 地图保存完成，规划产物可用。 |
| `FAILED` | 传感器、SLAM、保存或安全失败。 |
| `CANCELLED` | 操作者取消建图。 |

#### 跟踪模式

```text
operator starts tracking profile with active map
  -> SlamModule loads map
  -> setInitialPose or relocalize runs
  -> odometry/localization_quality become healthy
  -> Gateway shows TRACKING / DEGRADED / LOST and reason
  -> SafetyRing receives localization_status
  -> Navigation can use odometry only when tracking is healthy enough
```

跟踪模式只解决“我在哪里、地图是否锁住、定位是否可信”。它不接目标点，不产出路径。

#### 导航模式

```text
frontend click
  -> Gateway goal_pose
  -> Navigation mission
  -> global planner
  -> local planner
  -> cmd_vel mux
```

前端不计算路径。

#### 巡检模式

```text
operator starts inspection task
  -> task loads route, semantic target, frontier policy, or TARE goal source
  -> goal resolver selects next goal_pose
  -> Navigation executes that goal
  -> perception / memory / mission logger record evidence
  -> task marks checkpoint success, failed, skipped, or needs human review
  -> next checkpoint starts or inspection finishes
```

巡检模式是任务层，不是底盘控制层。它的每个移动目标都必须复用导航模式。

### 7.6 关键功能

#### F1. 原生 SlamModule 是产品主路径

要求：

| 要求 | 说明 |
| --- | --- |
| 产品 profile | 建图、跟踪、导航、巡检默认选择 `SlamModule`，除非显式 adapter；遥控/遥控避障可以只使用 driver odometry 或 native localization。 |
| fail closed | native backend 缺失时启动失败，不能静默 fallback。 |
| health | `localization_status.reason` 说明为什么进入当前状态。 |
| 无路径输出 | 测试阻止 `SlamModule` 输出 `global_path`、`local_path`、`waypoint`、`cmd_vel`、`path`。 |

#### F2. Fast-LIO2 实现 `ISlamBackend`

要求：

| 要求 | 说明 |
| --- | --- |
| Backend class | 增加真实 `FastLioBackend : ISlamBackend`。 |
| 输入 | 直接消费 `ImuSample` 和 `LidarFrame`。 |
| Deskew | 使用 `offset_time_ns`、`line` 和 IMU buffer。 |
| 输出 | 输出 odometry、covariance、velocity、bias、registered cloud、local submap、map cloud、map->odom、quality、tracking metrics。 |
| 模式 | `Mapping` 建图，`Localization` 加载保存地图并重定位。 |
| Save | 写 `map.pcd`、`poses.txt`、`patches/`。 |
| Reset | 清 buffer 和 map state，不重启整个进程。 |

#### F3. Point-LIO 使用同一合同

Point-LIO 必须实现同一组 `feedImu/feedLidar/tick/outputs`，切换后不修改 Navigation 或 Gateway。

#### F4. Livox MID-360 原生数据源

要求：

| 要求 | 说明 |
| --- | --- |
| 官方 SDK2 | 使用官方 discovery、command、heartbeat、IMU enable、point callback。 |
| 原生 source | C++ source 调用 `feedImu()` 和 `feedLidar()`，或使用最小本地 transport。 |
| 无 ROS2 runtime | native 产品链路不能要求启动 `livox_ros_driver2_node`。 |
| 原始字段保留 | timing/channel/tag 保留到 SLAM 后端。 |
| health | 输出设备连接、packet age、point rate、IMU rate、drop、time-sync 状态。 |

#### F5. 地图保存、加载和重定位

要求：

| 要求 | 说明 |
| --- | --- |
| Save map | SLAM 写地图产物，MapService 写元数据。 |
| Load map | SLAM 加载 `map.pcd`，设置 `map_loaded`。 |
| Relocalize | `relocalize(guess)` 返回质量、状态和原因。 |
| Map jump | 回环或重定位发布 `map_frame_jump_event`。 |
| Planner artifacts | MapService 在保存后构建 OctoPlanner3D 产物。 |

#### F6. Mapping control mode

MappingSession 管 phase、reason 和 control mode，但不转发速度。

控制模式：

| 模式 | 运动来源 | 路由 | 保存时允许 |
| --- | --- | --- | --- |
| `hold` | 无 | 不接受新运动源 | 是 |
| `manual_teleop` | 操作者遥控 | Gateway/Teleop -> CmdVelMux -> SafetyRing -> Driver | 否 |
| `assisted_navigation` | exploration/frontier/task goal | MappingSession 接收目标 -> Navigation -> LocalPlanner -> PathFollower -> CmdVelMux -> SafetyRing -> Driver | 否 |
| `emergency_stop` | 安全或人工 stop | SafetyRing/CmdVelMux stop path | 是 |

端口：

| 端口 | 方向 | 含义 |
| --- | --- | --- |
| `map_command` | input | start、pause、resume、save、cancel、finish、set control mode |
| `control_mode_request` | input | hold、manual_teleop、assisted_navigation、emergency_stop |
| `teleop_activity` | input | 遥控活动状态，不是 velocity relay |
| `assisted_goal` | input | 辅助建图目标 |
| `goal_pose` | output | 被接受的辅助目标，送入 Navigation |
| `mapping_status` | output | phase、control mode、reason、active map、health |
| `map_save_result` | output | 保存产物路径和验证结果 |

#### F7. Navigation 链路保持独立

| 要求 | 说明 |
| --- | --- |
| 目标来源 | Frontend、CLI、MCP、patrol、semantic planner 都统一为 `goal_pose`。 |
| 起点来源 | 来自 `SlamModule.odometry`，不是前端手填。 |
| 全局规划 | OctoPlanner3D 是产品默认。 |
| 局部规划 | 使用 global path、waypoint、terrain/local obstacles。 |
| 安全 | SafetyRing 和 CmdVelMux 是 driver 前最后一道门。 |

#### F8. Profile graph 和文档清理

静态 profile graph 必须根据实际 adapter 配置选择 `SlamModule` 或 `SlamBridgeModule`。非 archive 文档不能再描述隐式 ROS fallback。

#### F9. 运行态状态看板

Gateway / CLI / MCP 必须能回答两件事：

- 机器人为什么不动？
- 当前命令是谁产生的？

状态来源：

| 状态 | 来源 |
| --- | --- |
| SLAM state/reason | `SlamModule.localization_status` |
| 定位质量 | `SlamModule.localization_quality` |
| GNSS fusion | `SlamModule.gnss_fusion_health` |
| active map | `SlamModule.localization_status` + MapService |
| global planner state | Navigation planner status |
| mission phase/reason | `Navigation.mission_status` |
| local planner state | LocalPlanner diagnostics |
| safety state | SafetyRing |
| command source | CmdVelMux |

## 8. 运行时规格

这一节是工程实现规则。一个 Module 如果不能暴露这些状态、时间、坐标和失败原因，就不能进入产品 profile。

### 8.1 状态机

#### SLAM 状态

| 状态 | 含义 | 进入条件 | 离开条件 | 运动策略 |
| --- | --- | --- | --- | --- |
| `UNCONFIGURED` | 后端未配置。 | Module 构造或 reset。 | `configure()` 成功。 | 不允许自治移动。 |
| `INITIALIZING` | 等待足够 sensor/map 输入。 | mode 启动。 | 初始化成功或失败。 | 不允许自治移动。 |
| `MAPPING` | 正在建图。 | `setMode(Mapping)` 成功。 | save/cancel/failure。 | 只允许手动或辅助建图移动。 |
| `LOCALIZING` | 正在加载地图或重定位。 | `setMode(Localization)`、`loadMap()`、`relocalize()`。 | 定位锁定、降级或失败。 | hold autonomous motion。 |
| `TRACKING` | 定位健康，可以导航。 | localization quality 过阈值。 | 质量下降或 map jump。 | safety 允许时可导航。 |
| `DEGRADED` | 定位可用但受限。 | 质量低于 normal 但未 lost。 | 恢复、重定位或 lost。 | 降速或 hold。 |
| `LOST` | 定位不安全。 | odometry stale、tracking lost、map inconsistency。 | 重定位成功。 | 停止自治，只允许显式配置的监督遥控。 |
| `FAILED` | 后端无法继续。 | fatal backend 或 sensor failure。 | operator reset。 | 停止运动。 |

#### Navigation 状态

Navigation 使用现有 `src/nav/mission/model/state.py` 里的 `MissionState` 和 `MissionEvent`，不要再造一套。

| 状态 | 产品含义 | 主要事件 |
| --- | --- | --- |
| `IDLE` | 无活动任务。 | `GOAL_ACCEPTED`、`PATROL_ACCEPTED`、`EXPLORE_ACCEPTED`。 |
| `PLANNING` | OctoPlanner3D 正在产出全局路径。 | `PLAN_OK`、`PLAN_FAILED`、`FRAME_ERROR`、`CANCEL`。 |
| `EXECUTING` | 局部规划和路径跟踪正在执行。 | `WAYPOINT_REACHED`、`PATH_COMPLETE`、`STUCK`、`REPLAN_REQUESTED`、`PAUSE`。 |
| `RECOVERING` | 正在从 stuck/blocked 中恢复。 | `RECOVERY_OK`、`RECOVERY_FAILED`、`CANCEL`。 |
| `PAUSED` | 暂停但不清空任务。 | `RESUME`、`STOP`、`CANCEL`。 |
| `SUCCESS` | 路径完成。 | 新任务或 `STOP`。 |
| `FAILED` | 任务失败，有 typed reason。 | 新任务或 `STOP`。 |
| `CANCELLED` | 操作者取消任务。 | 新任务或 `STOP`。 |
| `PATROLLING` / `STUCK` | 兼容旧公开状态。 | 新逻辑优先用 `mission_mode=PATROL` 和 `RECOVERING`。 |

`EMERGENCY_STOP` 不是 Navigation 状态。它是 SafetyRing/CmdVelMux 条件，会强制 zero driver command，并向 Navigation 发送 `STOP`。

### 8.2 时间、坐标和同步

坐标合同：

| Frame | 含义 | 规则 |
| --- | --- | --- |
| `map` | 保存地图和全局规划 frame。 | 只允许通过 `map_frame_jump_event` 跳变。 |
| `odom` | 连续局部 frame。 | 不能跳变，可以漂移，由 `map->odom` 修正。 |
| `base_link` / `body` | 机器人本体 frame。 | 所有 robot pose 必须能解析到这里。 |
| `lidar` / `imu` / `camera` | 传感器 frame。 | 产品 profile 中必须完成到 body 的标定。 |

时间合同：

| 项 | 产品门槛 |
| --- | --- |
| timestamp source | 优先硬件时间戳；否则单进程统一 monotonic host clock。 |
| LiDAR/IMU sync | 标定后目标 <= 2 ms；未知则 native SLAM gate 失败。 |
| LiDAR 点字段 | `offset_time_ns`、`line`、`tag` 保留到 SLAM 消费。 |
| SLAM odometry | 产品 profile 最低 >= 10 Hz，目标 20 Hz。 |
| 在线地图层 | occupancy/elevation/traversability 状态 >= 2 Hz。 |
| Local planner | 执行中 >= 10 Hz。 |
| CmdVelMux | 目标 20-50 Hz，source timeout 默认 0.5 s。 |
| 控制延迟 | S100P 上从选中 `cmd_vel` 候选到 driver command，p95 <= 80 ms。 |
| 停止延迟 | safety stop 到 zero driver command，p95 <= 100 ms。 |

### 8.3 SLAM 物理输出合同

现有 `SlamOutputs` 足够做 wiring test，但不够做 field-grade 状态估计。Fast-LIO2 / Point-LIO 进入产品前必须补齐：

| 输出 | 用途 |
| --- | --- |
| `pose_covariance` | Navigation/Safety 根据不确定性降速或 hold。 |
| `velocity_odom_body` | 局部规划、打滑诊断、振荡检测。 |
| `imu_bias` | SLAM 诊断和 replay 验证。 |
| `local_submap_cloud` | 局部规划和近场障碍。 |
| `relocalization_score` | 判断是否接受重定位假设。 |
| `tracking_metrics` | 点数、残差、inlier ratio、退化、scan age。 |
| `map_version_id` | 绑定 pose 和 planner artifacts 到具体地图版本。 |

外部接口仍保持小：`feedImu`、`feedLidar`、`feedGnss`、`feedVisualOdom`、`tick`、`outputs`、`saveMap`、`loadMap`、`relocalize`、`reset`。额外物理状态放进 `SlamOutputs`，不要增加一堆 public 方法。

### 8.4 地图生命周期

Map 是带版本的运行时对象，不是一个 `map.pcd` 文件。

| 状态 | 含义 |
| --- | --- |
| `EMPTY` | 没有地图数据。 |
| `BUILDING` | SLAM 正在追加 live observations。 |
| `OPTIMIZING` | PGO/filtering/cleanup 正在运行。 |
| `ARTIFACT_BUILDING` | OctoMap/tomogram/occupancy 产物正在生成。 |
| `READY` | map bundle 有效，但未激活。 |
| `ACTIVE` | runtime 正在用这张图定位/规划。 |
| `STALE` | artifact、calibration 或 version gate 失败。 |
| `FAILED` | save/build/load 失败。 |
| `ARCHIVED` | 只保留审计，不给产品 profile 选择。 |

每个 saved map record 必须包含：`map_id`、`version`、`parent_map_id`、`created_at`、`source_profile`、`frame_id`、calibration hash、artifact list、validation gate result。

Map merge/split 先不做，等出现两张真实 field map 需要合并/拆分时再加。

### 8.5 OctoPlanner3D 接口

OctoPlanner3D 是产品全局规划器，不是黑盒。

输入合同：

| 输入 | 要求 |
| --- | --- |
| `start_pose` | 来自 `SlamModule.odometry`，在 `planning_frame_id`。 |
| `goal_pose` | 来自 Gateway/CLI/MCP/semantic/exploration，在 `planning_frame_id`。 |
| `map_artifacts` | 来自 MapService gate 的 OctoMap/tomogram/occupancy bundle。 |
| `robot_constraints` | 半径、ground support、snap radius、traversability flags。 |
| `cost_inputs` | 静态 occupancy，加 live fused cost/traversability。 |

输出合同：

| 输出 | 要求 |
| --- | --- |
| `global_path` | `planning_frame_id` 下的有序 3D waypoints。 |
| `adjusted_goal` | safe-goal search 移动目标时必须给出。 |
| `cost` / `length_m` | 诊断和回归指标。 |
| `latency_ms` | QA gate 必须记录。 |
| `rejected_reason` | 无路径时必须给出。 |
| `map_version_id` | 必须匹配本次规划使用的 artifact gate。 |

重规划触发：

- 新目标
- map frame jump
- live cost 阻塞路径
- planner artifact 改变
- operator replan
- localization recovery
- local planner 多次 stuck

延迟目标：参考 saved map 上 p95 <= 500 ms，p99 <= 2 s。大地图达不到时 profile 必须报告 degraded planning，不能偷偷切别的 planner。

### 8.6 SafetyRing 和 CmdVelMux

CmdVelMux 优先级固定：

| 来源 | 优先级 | Timeout |
| --- | ---: | ---: |
| Teleop / 普通遥控 | 100 | 0.5 s |
| GuardedTeleop / 遥控避障 | 100 | 0.5 s |
| VisualServo | 80 | 0.5 s |
| Navigation recovery | 60 | 0.5 s |
| PathFollower | 40 | 0.5 s |

规则：

- 非有限速度必须 drop 或 zero。
- 最高优先级 active source 获胜。
- 普通遥控和遥控避障互斥，不能同时 active；切换模式时必须先清空上一种遥控 source。
- 所有 source 超时则输出 zero velocity。
- SafetyRing / geofence / e-stop 可以覆盖任何 source。
- SLAM 为 `LOST` 或 `FAILED` 时，autonomy 必须停止。
- 监督遥控只有 safety policy 明确允许时才可以继续。
- 产品 profile 必须在 driver 前做 velocity、acceleration、jerk limit。如果不放在 CmdVelMux，就加一个单独 VelocityLimiter Module。

### 8.7 失败分类和恢复策略

Gateway/MCP 暴露的失败必须包含：`category`、`code`、`severity`、`reason`、`recoverable`、`recommended_action`。

| Category | Codes | 默认动作 |
| --- | --- | --- |
| SLAM | `SLAM_TRACKING_LOST`、`IMU_STALE`、`LIDAR_STALE`、`TIME_SYNC_BAD`、`MAP_INCONSISTENT`、`RELOCALIZATION_FAILED` | hold autonomy，安全时尝试重定位，多次失败要求人工处理。 |
| Navigation | `NO_GLOBAL_PATH`、`PATH_BLOCKED`、`LOCAL_OSCILLATION`、`WAYPOINT_TIMEOUT`、`FRAME_MISMATCH`、`PLANNER_ARTIFACT_MISSING` | replan 或进入 `RECOVERING`，超过 retry budget 后 fail mission。 |
| Safety | `ESTOP`、`GEOFENCE`、`CMD_STALE`、`NONFINITE_CMD`、`LOCALIZATION_UNSAFE`、`DRIVER_UNSAFE` | 强制 zero command；e-stop/geofence 需要显式 clear。 |
| Map | `MAP_SAVE_FAILED`、`ARTIFACT_BUILD_FAILED`、`MAP_VERSION_MISMATCH`、`MAP_VALIDATION_FAILED` | 阻止 nav profile 使用该地图。 |
| System | `CLOCK_DRIFT`、`DRIVER_DISCONNECTED`、`ADAPTER_STALE`、`PROFILE_CONFIG_INVALID` | 停止受影响 profile，并显示操作建议。 |

### 8.8 ROS 兼容 adapter 规则

ROS2 是显式 adapter，不是隐式 fallback。

- Adapter 默认 passive，除非 profile 明确说它拥有 localization。
- Adapter 必须 mirror native ports：odometry、map_cloud、saved_map、localization_status、localization_quality、map_odom_tf、map_frame_jump_event、gnss_fusion_health。
- Adapter 必须把 TF 归一到同一套 `map`、`odom`、`base_link/body` 合同。
- Adapter 不允许让 ROS planner 绕过 Navigation、LocalPlanner、CmdVelMux 或 SafetyRing。
- Native 和 ROS adapter 的 status payload 必须使用同一 failure taxonomy。

### 8.9 去 ROS 化和函数调用关系

产品主链路必须是 native function path。ROS topic、ROS service、TF listener、ROS message type 不能成为核心数据通路。

目标函数调用关系按功能拆分：

遥控避障：

```text
Gateway.guarded_teleop_ws(payload) / TeleopModule.on_guarded_joystick(payload)
  -> normalize joystick to desired Twist
  -> guarded teleop limiter clamps desired_twist using obstacle_state, geofence, localization_status
  -> CmdVelMux.on_source("guarded_teleop", limited_twist)
  -> CmdVelMux.select(active cmd_vel source)
  -> SafetyRing.evaluate()
  -> Driver.cmd_vel(final command)
  -> Gateway publishes active_source, safety_state, blocked_reason
```

普通遥控：

```text
Gateway.teleop_ws(payload) / TeleopModule.on_joystick(payload)
  -> normalize joystick to Twist
  -> CmdVelMux.on_source("teleop", twist)
  -> CmdVelMux.select(active cmd_vel source)
  -> SafetyRing.evaluate()
  -> Driver.cmd_vel(final command)
  -> Gateway publishes active_source and safety_state
```

Mapping：

```text
MappingSession.start()
  -> SlamModule.set_mode(MAPPING)
  -> SlamRunner.feedImu()/feedLidar()
  -> ISlamBackend::tick()
  -> SlamModule publishes odometry/map_cloud/localization_status
  -> map layers update online maps
  -> MapService.save/build artifacts when operator saves
```

跟踪：

```text
MapService.use(active_map)
  -> SlamModule.set_mode(LOCALIZATION, map_path)
  -> SlamModule.load_map()
  -> SlamModule.relocalize(optional_guess)
  -> SlamRunner.feedImu()/feedLidar()
  -> ISlamBackend::tick()
  -> SlamModule publishes odometry/localization_quality/map_odom_tf/map_frame_jump_event
  -> Navigation/Safety/Gateway consume the same native ports
```

导航：

```text
Navigation.accept_goal(goal_pose)
  -> current pose from SlamModule.odometry
  -> GlobalPlanner.plan(start, goal, map_artifacts, constraints)
  -> LocalPlanner.step(global_path, waypoint, terrain, esdf)
  -> PathFollower.step(local_path, odometry)
  -> CmdVelMux.select(active cmd_vel source)
  -> SafetyRing.evaluate()
  -> Driver.cmd_vel(final command)
```

巡检：

```text
InspectionTask.start(route_or_policy)
  -> resolve next checkpoint / semantic target / frontier / external goal
  -> Navigation.accept_goal(goal_pose)
  -> mission logger records result and evidence
  -> repeat until task complete or failed
```

硬件入口：

```text
Livox-SDK2 callback / replay / sim source
  -> normalized ImuSample + LidarFrame
  -> SlamRunner.feedImu()/feedLidar()
```

实现规则：

- Livox native profile 不启动 `livox_ros_driver2_node`。
- Fast-LIO2 / Point-LIO native backend 不 include ROS message，不依赖 `rclcpp/rclpy` 生命周期。
- SLAM 输入是 `ImuSample` 和 `LidarFrame`，不是 ROS `CustomMsg`。
- SLAM 输出是 `SlamOutputs`，不是 `/path`、`/odom`、`/tf` topic。
- OctoPlanner3D 由 Navigation 进程内调用，不通过 ROS service 请求路径。
- LocalPlanner 和 PathFollower 由 Module 图调用，不通过 ROS topic 接 `global_path/local_path`。
- CmdVelMux 到 driver 是唯一速度出口，不能有 ROS 节点直接向机器狗驱动写速度。
- Gateway/Web/MCP 发的是 `goal_pose`、`map_command`、`teleop` 请求，不发布 ROS goal topic。

允许保留的 ROS：

| 场景 | 规则 |
| --- | --- |
| 老算法验证 | 只能放在显式 compatibility profile。 |
| 外部仿真 | 只能在 adapter 边界把 ROS 数据转成 LingTu Module 端口。 |
| 第三方 SLAM 临时接入 | 必须 mirror native ports，不能让下游知道 ROS 存在。 |
| 调试可视化 | 只能订阅或镜像数据，不能成为主控制链路。 |

禁止事项：

- 产品 native profile 里禁止用 ROS topic 作为 SLAM 到 Navigation 的主数据源。
- 禁止把 ROS `/path` 接成 LingTu `local_path`。
- 禁止 planner 绕过 Navigation 直接控制 driver。
- 禁止用 systemd 启动 ROS driver 作为 native Livox 的必要步骤。
- 禁止在普通业务 Module 里 import `rclpy/rclcpp` 或 ROS message type。

验收：

- `map` profile 在不安装 ROS2 的环境中能启动到 native SLAM source mock。
- `nav` native profile 在不安装 ROS2 的环境中能加载 saved map 并完成规划链路 mock。
- 代码扫描证明 `src/nav/`、`src/localization/slam/`、`src/drivers/real/lidar/` 的 native path 不依赖 ROS import。
- 全链路日志显示函数调用链，而不是 ROS topic relay 链。

### 8.10 部署 profile 合同

每个 runtime profile 必须声明或继承这些字段：

| 字段 | 含义 |
| --- | --- |
| `runtime_mode` | guarded_teleop、teleop、mapping、tracking、navigation、inspection、sim 或显式 experiment。 |
| `robot_preset` | 物理/仿真机器人驱动选择。 |
| `localization_mode` | native_slam、explicit_adapter、driver_odometry 或 none。 |
| `lidar_source` | native Livox-SDK2、DDS endpoint、sim cloud 或 none。 |
| `map_mode` | build、use_saved 或 none。 |
| `planner_backend` | 产品 profile 使用 octoplanner3d；pct/direct 只能是显式实验。 |
| `local_planner_backend` | nanobind/nav_kernel/pure pursuit/pid 等配置值。 |
| `safety_policy` | 定位丢失、geofence、stale command 的停止策略。 |
| `gateway_enabled` | REST/SSE/WS/MCP 是否启用。 |

### 8.11 运行时验收门槛

| Gate | 门槛 |
| --- | --- |
| SLAM drift | 参考路线 10 分钟内，位置漂移 < 行驶距离 1%。 |
| Relocalization | 有效初始 guess 下，5 s 内锁定保存地图。 |
| Time sync | native SLAM profile 中 LiDAR/IMU offset 已知且 <= 2 ms。 |
| ROS-free native path | native profile 不安装 ROS2 也能跑 sensor mock -> SLAM -> map -> planner -> local planner -> cmd_vel mock。 |
| Global planning | 参考 saved map 上 OctoPlanner3D p95 <= 500 ms，p99 <= 2 s。 |
| Local planning | 执行中 live traversability 输入下 >= 10 Hz。 |
| Control delay | 选中命令候选到 driver command，p95 <= 80 ms。 |
| Safety stop | stop event 到 zero driver command，p95 <= 100 ms。 |
| Failure reporting | critical input stale 后 1 s 内发布 typed failure。 |
| Map save | bundle 包含 `map.pcd`、metadata、planner artifacts，或 typed failed gate。 |
| Full chain | Web/MCP 目标只能经 Navigation -> LocalPlanner -> CmdVelMux -> SafetyRing 到达运动。 |

## 9. 技术落点

### 9.1 目标源码布局

| 路径 | 作用 |
| --- | --- |
| `src/localization/slam/module.py` | Python Module 边界。 |
| `src/localization/slam/cpp/slam.hpp` | C++ SLAM 后端合同。 |
| `src/localization/slam/cpp/fastlio.cpp` | Fast-LIO2 后端入口。 |
| `src/localization/slam/cpp/pointlio.cpp` | Point-LIO 后端入口。 |
| `src/localization/slam/cpp/bind.cpp` | `SlamRunner` Python binding。 |
| `src/drivers/real/lidar/Livox-SDK2/` | 官方 Livox SDK2 source/vendor 位置。 |
| `src/drivers/real/lidar/` | LingTu LiDAR Module/source 边界。 |
| `src/nav/mission/mapping.py` | Mapping session FSM、control mode、save gate。 |
| `src/nav/mission/` | Mission FSM 和全局规划 handoff。 |
| `src/nav/services/maps.py` | 地图产物生命周期和元数据入口。 |
| `src/nav/services/plan/global_planner/` | OctoPlanner3D 服务边界。 |
| `src/nav/services/plan/local_planner/` | 局部规划实现。 |
| `src/gateway/` | REST/SSE/WS/MCP 和状态看板。 |

### 9.2 SLAM 后端接口

所有 SLAM 算法实现：

```cpp
configure(config)
setMode(mode, map_path)
feedImu(sample)
feedLidar(frame)
feedGnss(sample)
feedVisualOdom(sample)
setInitialPose(pose)
relocalize(guess)
tick()
saveMap(path)
loadMap(path)
outputs()
reset()
```

`SlamOutputs` 必须包含：

```text
odometry_odom_body
state_estimation_at_scan
registered_cloud_body
map_cloud_map
saved_map_cloud_map
map_odom_tf
pose_covariance
velocity_odom_body
imu_bias
local_submap_cloud
relocalization_score
tracking_metrics
map_version_id
alive
map_loaded
map_frame_jump
localization_quality
gnss_fusion_health
scene_mode
imu/lidar buffers and drops
```

### 9.3 非目标

- 不让 SLAM 产出导航路径。
- 不让前端计算路径或避障。
- 不在只有一个真实实现时增加新 planner abstraction。
- 不保留隐式 ROS2 fallback。
- 不把 TARE 重写进 LingTu Python。TARE 只作为显式外部 exploration backend。
- 不在这份 PRD 里展开 detector、LLM、校准、部署的全部细节；这些做专项文档。

## 10. 假设

| 假设 | 验证方式 |
| --- | --- |
| 官方 Livox-SDK2 可以在 S100P 上无 ROS2 运行。 | build + device-connect gate。 |
| Fast-LIO2 map_builder 可以包装成非 ROS 生命周期。 | C++ compile + replay gate。 |
| Point-LIO 可以共享同一 IO 合同。 | backend parity test。 |
| S100P CPU 能同时跑 native SLAM、maps、planner、gateway。 | field performance gate。 |
| 现有 MapService 可继续做元数据 owner。 | save/load/replan integration test。 |
| Gateway 不需要新增 transport 就能显示状态。 | SSE/REST status test。 |

## 11. 发布计划

### 阶段 0：接口与运行规则定版

目的：先把跨模块可见的规则定下来，避免后面 SLAM、地图、导航、安全、Gateway 各写各的状态和失败格式。

要做的事：

- 明确 `SlamModule` 对外端口：只输出定位、地图、传感器观测和健康状态。
- 明确 C++ `ISlamBackend` 方法和 `SlamOutputs` 字段。
- 明确 SLAM、Mission、Map、Safety 的状态、事件、状态切换原因。
- 明确 `map / odom / body / lidar / imu / camera` 的坐标规则。
- 明确 timestamp、LiDAR/IMU sync、输出频率、控制延迟的验收口径。
- 明确 failure payload：`category`、`code`、`severity`、`reason`、`recoverable`、`recommended_action`。
- 明确 native stack 默认选择，ROS2 只能作为显式 adapter。
- 增加测试，阻止 SLAM 输出 `global_path`、`local_path`、`waypoint`、`cmd_vel`。

完成标准：

- native contract tests 通过。
- `mission_status` 和 `localization_status` 都包含 state、reason、typed failure。
- 8.11 的运行时验收门槛都有 owner 和验证方式。
- profile graph 与实际运行图的差异被列出，并分配修复 owner。

### 阶段 1：Native profile 清理

范围：

- static profile graph 区分 native 和 explicit adapter。
- 增加 deployment profile 必填字段和验证。
- 审计 native profile 的 ROS import、ROS topic、ROS service、TF listener 依赖。
- 把仍在主链路里的 ROS relay 改成 Module 端口或进程内函数调用。
- 更新还期待 `SlamBridgeModule` 的测试。
- 更新非 archive 文档和 smoke scripts。
- native backend 缺失时产品 profile fail closed。

退出条件：

- `test_profile_graph_snapshots.py` 通过。
- profile audit 显示 runtime_mode、localization_mode、lidar_source、map_mode、planner_backend、safety_policy。
- native profile 的主链路不需要 ROS2 installed。
- 无隐式 ROS2 fallback。

### 阶段 2：Mapping control + Livox 到 SLAM

范围：

- MappingSession 状态和 status。
- 手动建图走 CmdVelMux/SafetyRing。
- 辅助建图目标进入 Navigation。
- save gate 在写 artifact 前停止或拒绝运动。
- map lifecycle state 和 versioned map record。
- 官方 Livox-SDK2 callback 通过函数调用接入 `feedImu/feedLidar`。
- 删除 native Livox 对 `livox_ros_driver2_node` 的启动依赖。
- 校验 LiDAR/IMU timestamp offset。
- 保留 timing/channel/tag。
- mock source 和 hardware source health。

退出条件：

- map profile 可以 start/pause/resume/stop/save，并给出原因。
- 手动建图证明 teleop -> CmdVelMux -> Driver。
- 辅助建图证明 goal -> Navigation -> LocalPlanner -> CmdVelMux。
- saved map record 包含 id、version、frame、calibration hash、artifacts、gate result。
- native LiDAR source 输出 packet age、point rate、IMU rate、drop、time-sync。
- native Livox mock 到 SLAM backend 的链路不经过 ROS topic。

### 阶段 3：Fast-LIO2 原生后端

范围：

- 实现真实 `FastLioBackend`。
- 从 Fast-LIO2 产品后端中剥离 ROS message、ROS node、ROS parameter 和 TF runtime 依赖。
- 使用 IMU/LiDAR buffers 和 deskew。
- 输出 odometry、covariance、velocity、IMU bias、registered cloud、local submap、map cloud、quality、map->odom、tracking metrics。
- 保存地图产物。

退出条件：

- replay gate 产出稳定 odometry 和非空 map cloud。
- replay gate 报告 covariance、velocity、bias、relocalization score、tracking metrics。
- LiDAR/IMU sync gate 通过，或用 `TIME_SYNC_BAD` 失败。
- map profile 不启动 ROS2 SLAM node 也能保存地图。
- Fast-LIO2 replay 使用 `ImuSample/LidarFrame`，不使用 ROS bag 或 ROS message 作为必需输入。

### 阶段 4：保存地图定位 + Point-LIO 对齐

范围：

- 加载 saved map。
- 用可选 initial pose 重定位。
- 发布 map jump event。
- 实现 Point-LIO backend，或显式放到 disabled profile。

退出条件：

- `nav <map>` 能对保存地图定位。
- map jump 触发 Navigation replan。
- 切换 backend 不改 Navigation/Gateway。

### 阶段 5：完整导航证明

范围：

- MuJoCo native SLAM demo。
- S100P 实机 gate。
- Gateway 状态审计。
- MCP command proof。
- safety stop、command delay、global planning latency、local planning rate 证明。

退出条件：

- `goal_pose -> OctoPlanner3D -> local_path -> cmd_vel -> mux` 被证明。
- 8.11 运行时验收门槛通过或被显式豁免。
- 操作者能看到为什么允许或阻止运动。
- 日志证明没有把 SLAM path 当 local path 使用。

### 阶段 6：兼容层收缩

范围：

- 只保留仍需要的显式 ROS2/LCM adapters。
- 删除要求旧 bridge 名字的 dead smoke scripts 和文档。
- 旧 ROS-only launch path 归档，不作为产品路径。

退出条件：

- native 产品链路无 ROS2 installed 也能跑。
- 兼容路径被描述成 optional adapter，而不是普通 runtime。

## 12. 未决策项

| 问题 | 默认决策 |
| --- | --- |
| S100P 的 `nav` 是否立刻用 native SLAM？ | 否。Phase 5 通过前用 native profile gate。 |
| LCM localization adapter 是否保留？ | 是，只作为显式 endpoint adapter。 |
| Python contract runner 是否允许进产品 profile？ | 否，只用于测试。 |
| Point-LIO 是否阻塞首版？ | 否。Fast-LIO2 native 先发，Point-LIO 跟进。 |
| Gateway 是否接受导航 direct `cmd_vel`？ | 否。只有 teleop 可绕开 planner，仍受 CmdVelMux/Safety 控制。 |
| assisted mapping 是否早于完整 native navigation proof？ | 否。先发 manual mapping，assisted mapping 等 Navigation/Safety gate 后。 |

## 13. 验证矩阵

| Gate | 证明方式 |
| --- | --- |
| Python contract | `python -m pytest src/localization/tests/test_native_slam_contract.py -q` |
| SLAM stack | `python -m pytest src/localization/tests/test_slam_stack_services.py -q` |
| State semantics | SLAM、mission、map、safety status 暴露 state、reason、failure code、rejected transition。 |
| Time/frame contract | replay 或 hardware check 证明 frame id、monotonic timestamp、LiDAR/IMU sync、map-frame jump。 |
| ROS-free native path | 不安装 ROS2 时，native mock profile 能跑 sensor -> SLAM -> map -> planner -> local planner -> cmd_vel mock。 |
| ROS dependency audit | native path 代码扫描无 `rclpy/rclcpp`、ROS message、ROS topic/service/tf 作为主链路依赖。 |
| Teleop | 普通遥控只产生 `cmd_vel` 候选，不进入 Navigation mission；timeout/release 后输出 zero 或切换来源。 |
| Guarded teleop | 遥控避障能根据近场障碍/电子围栏/定位状态 clamp 或 block 人工速度，并显示 blocked_reason。 |
| Binding policy | `python -m pytest src/runtime/tests/test_runtime_binding_policy.py -q` |
| Profile graph | `python -m pytest src/runtime/tests/test_profile_graph_snapshots.py -q` |
| C++ contract | `cmake -S src/localization/slam/cpp -B <build> && cmake --build <build> && test_slam_contract` |
| Livox mock | mock source 给 native backend 喂 IMU + LiDAR。 |
| Mapping control | map profile start/pause/resume/save；所有运动经过 CmdVelMux/SafetyRing。 |
| OctoPlanner3D contract | plan result 包含 path、adjusted goal、latency、map version、cost/length、rejected reason。 |
| Safety arbitration | priority、timeout、zero-on-timeout、e-stop、non-finite command、velocity limiting 被证明。 |
| Failure taxonomy | SLAM、Navigation、Safety、Map、System 代表失败能发布 typed payload。 |
| MuJoCo | sim goal 产出 global path、local path、cmd_vel。 |
| Hardware mapping | S100P MID-360 建图并保存 `map.pcd`。 |
| Hardware navigation | saved map 加载、重定位、规划、跟踪，安全允许受控运动。 |

## 14. 待办

| 优先级 | 事项 | 主要文件 |
| --- | --- | --- |
| P0 | 更新 profile graph native/adapter selection | `src/runtime/blueprints/profile_graph.py`、profile graph tests |
| P0 | 收敛六个产品 runtime mode | `src/runtime/profiles/catalog/product_intents.py`、Gateway/CLI profile help |
| P0 | 定义遥控和遥控避障互斥控制入口 | `src/drivers/teleop_module.py`、`src/gateway/`、`src/nav/services/safety/velocity_mux.py` |
| P0 | 剥离 native 主链路 ROS relay | `src/localization/`、`src/drivers/real/lidar/`、`src/nav/` |
| P0 | 增加 ROS-free native mock profile gate | `src/runtime/profiles/`、`src/runtime/tests/`、`sim/tests/` |
| P0 | 锁定 status/failure payload schema | `src/localization/slam/module.py`、`src/nav/mission/model/status.py`、`src/nav/services/safety/` |
| P0 | 增加 time/frame/sync 验收检查 | `src/runtime/`、`src/localization/tests/`、`sim/tests/` |
| P0 | native backend 缺失时产品启动失败 | `src/localization/slam/module.py`、profile startup checks |
| P0 | 增加 native binding target | `src/localization/slam/cpp/bind.cpp`、CMake |
| P0 | 增加 SLAM 物理输出到 C++ contract | `src/localization/slam/cpp/slam.hpp`、bindings、tests |
| P0 | 增加 MappingSession FSM 和 status | `src/nav/mission/mapping.py`、`src/gateway/` |
| P0 | 增加 versioned map lifecycle states | `src/nav/services/map/`、`src/nav/services/maps.py` |
| P0 | 形式化 OctoPlanner3D request/result diagnostics | `src/nav/services/plan/global_planner/` |
| P0 | 增加 driver 前速度限制证明 | `src/nav/services/safety/velocity_mux.py` 或单独 `VelocityLimiter` |
| P0 | 官方 Livox SDK2 接入 backend feed | `src/drivers/real/lidar/Livox-SDK2/`、`src/drivers/real/lidar/` |
| P0 | 实现真实 Fast-LIO2 backend | `src/localization/slam/cpp/fastlio.cpp`、`src/localization/fastlio2/` |
| P1 | save/load/relocalize 产品路径 | `src/localization/slam/`、`src/nav/services/maps.py` |
| P1 | assisted mapping 接入 Navigation | `src/nav/exploration/`、`src/nav/mission/mapping.py`、`src/nav/mission/navigation.py` |
| P1 | MuJoCo native SLAM gate | `sim/scripts/`、`sim/tests/` |
| P1 | Gateway native chain 状态审计 | `src/gateway/` |
| P1 | 更新仍使用 `SlamBridgeModule` 名字的 smoke scripts | `tests/scripts/smoke/` |
| P2 | 实现 Point-LIO backend | `src/localization/slam/cpp/pointlio.cpp`、`src/localization/pointlio/` |
| P2 | 把旧 ROS2 文档降级成 compatibility docs | `docs/` 非 archive 部分 |
