# Mountain Valley Runtime 设计

日期：2026-10-18。状态：**架构设计，未完成 UE 导入、MuJoCo 碰撞或机器狗山地验收。**

本文定义正式山地环境接入方式。当前 Blender 山地预览只保留为材质、地形和传感器布置的技术样片；正式视觉环境优先来自已授权的 UE 样例/环境包，首选 `Valley of the Ancient`，备选为已授权的完整 Mountain Forest 或 Real3D 山地包。

## 1. 目标

- 在 UE5.8 中运行一个具有山谷、岩壁、林线、坡面、巡检路线和观测设施的连续山地世界。
- Thunder 机器狗可以在同一份场景定义中生成，并由 MuJoCo 负责真实地形接触、坡面运动和碰撞。
- UE 视觉环境、MuJoCo 物理环境、传感器挂点和巡检区域共享实体 ID、坐标系、尺度和内容指纹。
- 视觉资源可以使用 Nanite、Lumen、World Partition、Data Layers、PCG 和 HLOD，但这些能力不能成为物理或任务真值。
- 失败时能区分“视觉未加载”“物理未编译”“机器人未绑定”“传感器未就绪”和“任务不可运行”。

## 2. 非目标

- 不把外部 UE 环境包直接改造成我们的 MuJoCo 模型。
- 不让 UE Chaos 和 MuJoCo 同时负责同一机器人或同一静态障碍的真实接触。
- 不把外部环境包中的任务、角色、动画、蓝图逻辑自动视为 LingTu 任务逻辑。
- 不在首版实现运行时动态增删山体或大规模重编译 MuJoCo 模型。
- 不把当前 Blender 预览的树、岩石和路线直接升级为生产资产。
- 不以一张好看的 UE 截图替代坡面、障碍、Reset、传感器和机器人运动验收。

## 3. 总体架构

```text
SessionSpec
  ├── world: mountain_valley@version
  ├── robot: thunder_v4@version
  ├── sensor_rig: thunder_navigation@version
  └── runtime_profile: ue5_mountain_preview / playable
                    │
                    ▼
              Catalog Resolver
                    │
                    ▼
             ResolvedSessionBundle
       ┌────────────┼─────────────┐
       ▼            ▼             ▼
  VisualPlan   PhysicsPlan    SensorPlan
       │            │             │
       ▼            ▼             ▼
  UE World     MuJoCo World   SensorRuntime
  Partition    Heightfield    RGB/Depth/LiDAR/IMU
  + DataLayer  + proxies      + timestamps
       └────────────┬────────────┘
                    ▼
             Runtime Coordinator
                    │
                    ▼
          RobotSimUE / Headless Runtime
```

物理权威保持不变：

```text
MuJoCo = 机器人位姿、速度、接触、地形碰撞、传感器真值来源
UE     = 地形视觉、材质、植被、光照、视觉传感器渲染
LingTu = 控制命令、导航、任务和证据合同
```

## 4. UE 世界组织

正式山地地图使用一个持久 World Partition 世界，不使用大量手工命名的传统子关卡。

建议地图：

```text
/Game/RobotSim/Worlds/MountainValley/WP_MountainValley
```

建议 Data Layers：

| Data Layer | 内容 | 默认状态 |
| --- | --- | --- |
| `DL_Mountain_Base` | Landscape、远景山体、主要岩壁、基础天空 | Activated |
| `DL_Mountain_Vegetation` | 树、灌木、草、风场和 PCG 结果 | Activated |
| `DL_Mountain_Outpost` | 观测站、通信杆、设备台和任务地标 | Loaded/Activated 依任务 |
| `DL_Mountain_Route` | 巡检路线语义、出生点、检查区域、导航标记 | Activated |
| `DL_Mountain_SensorRig` | UE 视觉传感器投影和相机标定辅助物 | Activated |
| `DL_Mountain_Debug` | 碰撞代理可视化、坐标轴、传感器视锥、诊断标记 | Unloaded |

必须区分三个状态：

```text
Loaded       = 资源在内存中，但不一定显示
Visible      = 资源已经可见，但不一定允许任务使用
Activated    = 资源可见，并已通过当前会话的运行条件检查
```

在进入 `RUNNING` 前，必须等待需要的 Data Layer 到达 `Activated`，不能仅因为异步加载请求已发出就开始机器人运动。

World Partition 的网格、HLOD 和加载半径需要在 UE5.8 工程中实际验证。首版建议从 128–256 m cell 规模开始，并以真实内存和传感器帧率测试结果调整，不能把建议值当作性能保证。

## 5. WorldPackage 合同

正式包不保存外部工程的绝对路径，不把 UE 资产路径当成跨系统实体 ID。

```yaml
schema: lingtu.sim.world-package.v1
id: mountain_valley
version: 0.1.0
compatibility:
  runtime_abi: lingtu.sim.world.v1
  unreal: "5.8"
  mujoco: "3.3.5"
visual:
  project_reference: external/valley_of_the_ancient
  persistent_map: /Game/RobotSim/Worlds/MountainValley/WP_MountainValley
  data_layers:
    - DL_Mountain_Base
    - DL_Mountain_Vegetation
    - DL_Mountain_Outpost
    - DL_Mountain_Route
physics:
  heightfield: physics/mountain_valley_heightfield.bin
  collision_manifest: physics/collision-proxies.json
  frame: lingtu_world_zup
entities:
  - id: mountain_outpost_001
    semantic_class: inspection_station
    visual_ref: /Game/RobotSim/Worlds/MountainValley/Outpost_001
    physics_ref: proxy/outpost_001
    transform_ref: transforms/mountain_outpost_001.json
```

WorldPackage 只描述统一实体、坐标、语义和后端引用；它不把外部环境包的完整 Content 目录复制成第二个配置真值源。

## 6. 地形与碰撞

地形必须从同源数据生成两种结果：

```text
原始 Landscape / 视觉高度数据
        ├── UE Landscape / Nanite visual terrain
        └── MuJoCo heightfield + static collision proxies
```

MuJoCo 首版使用：

- 一份统一 Z-up 高度场，保存采样间距、原点、宽高和高程范围；
- 山路和可行走区域的真实坡面高度，不用一个平面代替；
- 岩壁、观测台、护栏和大石的低模碰撞代理；
- 视觉网格不参与 MuJoCo 碰撞，除非经过专门的代理检查；
- 所有碰撞代理带稳定 `entity_id` 和 `collision_revision`。

物理生成验收：

1. 高度场与 UE Landscape 在原点、轴向、尺度和采样误差内一致。
2. Thunder 出生点没有穿地或悬空。
3. 机器人沿低坡、横坡和下坡运行时接触点连续。
4. 岩石和观测台能产生接触，但不可行走区域不会被导航误认为可通行。
5. Reset 后高度场、障碍、机器人状态和传感器代际一致。

## 7. 机器人接入

Thunder 作为 `RobotInstance` 接入，不修改外部环境包的机器人/角色逻辑。

```text
RobotPackage thunder_v4
      ↓
RobotInstance thunder_01
      ├── MuJoCo namespace: thunder_01__*
      ├── UE visual binding: thunder_01
      ├── sensor rig: thunder_navigation
      └── control contract: existing LingTu typed command/state
```

生成顺序：

1. 编译 WorldPackage 和 PhysicsPlan。
2. 创建会话级 `mjModel/mjData`。
3. 建立 Thunder 的 body/joint/actuator Descriptor。
4. 将出生点投影到高度场，验证足端和机身包络。
5. 创建 UE 视觉实例并绑定同代 `model_generation`。
6. 安装传感器挂点和外参。
7. 控制接口与传感器就绪后才进入 `READY`。

UE 机器人外观更新只能跟随 MuJoCo Snapshot。UE 动画、Chaos、蓝图移动和摄像机观察都不能改变机器人的物理真值。

## 8. 传感器与任务接线

山地纵向切片首批传感器：

| 传感器 | 权威 | 首版用途 |
| --- | --- | --- |
| RGB | UE SensorRuntime | 山路、岩石和巡检站视觉 |
| Depth | UE SensorRuntime | 坡面、障碍和近距离几何 |
| Mid360/LiDAR | 传感器模拟层 | 地形点云、障碍和导航输入 |
| IMU | MuJoCo/传感器桥 | 机身角速度、线加速度和姿态相关数据 |
| Odometry | Physics Runtime | 仿真真值/合成里程计，单独标注为 ground truth |

每帧必须带：

```text
session_digest
model_generation
reset_generation
sim_time_ns
sequence
frame_id / parent_frame_id
```

任务层只消费导航和检查合同，不从 UE Overlap、镜头距离或动画完成状态推断“已到达”“已采集”或“已接受”。

## 9. Runtime 状态机

会话状态：

```text
DECLARED
  → RESOLVING
  → RESOLVED
  → VISUAL_LOADING
  → PHYSICS_COMPILING
  → PHYSICS_READY
  → ROBOT_BOUND
  → SENSORS_READY
  → READY
  → RUNNING
  → QUIESCING
  → STOPPED
```

失败必须保留阶段和首因：

```text
VISUAL_LOAD_FAILED
PHYSICS_COMPILE_FAILED
FRAME_BIND_FAILED
SENSOR_READINESS_FAILED
GENERATION_MISMATCH
WORLD_CONTENT_MISMATCH
```

`RUNNING` 的前置条件不是“UE 窗口已经出现”，而是：

- 指定 Data Layers 已 Activated；
- MuJoCo model/data 已生成；
- ModelDescriptor 与 UE namespace 一致；
- Thunder 已绑定；
- 传感器已达到配置频率和 freshness 条件；
- 控制和 Reset 合同可用。

## 10. 所有权与模块边界

| 模块 | 所有者 | 不负责 |
| --- | --- | --- |
| `CatalogResolver` | 包版本、许可、摘要和引用解析 | 启动 UE、创建目录、编译物理 |
| `WorldCompiler` | VisualPlan、PhysicsPlan、实体图 | 运行时状态 |
| `PhysicsSceneComposer` | MuJoCo heightfield、碰撞代理、ModelDescriptor | UE 材质和任务真值 |
| `UE WorldRuntime` | World Partition、Data Layers、视觉资产 | 机器人物理位姿和碰撞权威 |
| `SensorRuntime` | RGB/Depth/LiDAR 渲染与时间戳 | 任务接受和机器人运动 |
| `Runtime Coordinator` | 会话阶段、代际和故障收口 | 自己解释外部包细节 |
| `Inspection Executor` | 目标顺序、停稳、采集、证据接受 | 修改 UE 场景或物理 |

不新增第二套 SessionSpec、第二套机器人配置或第二个 UE 任务协调器。

## 11. 失败模式与处理

| 失败 | 检测 | 处理 |
| --- | --- | --- |
| 外部 UE 工程缺少插件/资源 | 加载日志、资产引用和版本检查 | 标记 `VISUAL_LOAD_FAILED`，不启动物理运行 |
| UE5.5 包在 UE5.8 迁移失败 | 复制工程离线打开并记录迁移日志 | 不覆盖源工程，保留兼容性报告 |
| 高度场轴向/尺度不一致 | 原点、边界和采样点对比 | 阻止 PhysicsPlan 发布 |
| Data Layer 可见但未激活 | 查询运行时状态 | 不进入 `READY` |
| 机器人生成点无效 | 足端、机身包络和地面采样检查 | 使用明确失败，不自动抬高机器人 |
| 传感器 generation 不一致 | 每帧元数据比对 | 丢弃旧帧并重新绑定 |
| 视觉很漂亮但物理代理缺失 | WorldPackage 完整性检查 | 视觉可预览，运行模式拒绝 |
| MuJoCo 失败但 UE 仍在显示 | Coordinator 状态广播 | UE 进入诊断/暂停，不显示假运行 |

## 12. 分阶段交付

### Gate 0：样例与许可

- Epic Launcher 登录成功；
- 样例工程下载到 D 盘；
- 记录工程版本、来源、许可证和摘要；
- 原工程只读保存，不覆盖 RobotSimUE。

### Gate 1：UE5.8 只读预览

- 样例在 UE5.8 打开；
- World Partition、Data Layers、Nanite、Lumen 状态可记录；
- 不接机器狗，不改源工程；
- 输出一张 UE5.8 真实山地截图和迁移日志。

### Gate 2：WorldPackage 投影

- 生成 `mountain_valley@0.1.0`；
- 实体、坐标、出生点、观测站和路线可解析；
- VisualPlan 可以让 UE 加载指定 Data Layers；
- PhysicsPlan 仍可先标记未编译，不能伪装完成。

### Gate 3：MuJoCo 地形

- 高度场和碰撞代理编译成功；
- headless MuJoCo 高程、坡度、接触和 Reset 测试通过；
- 生成 ModelDescriptor 和 physics snapshot。

### Gate 4：Thunder 山地运动

- UE 视觉 Thunder 与 MuJoCo `thunder_01` 同代绑定；
- 不按 Shift 即可运动；
- 前进、后退、转向、坡面、岩石、受阻和 Reset 都有证据；
- 机器狗位姿不由 UE 自己推进。

### Gate 5：传感器与巡检任务

- RGB、Depth、LiDAR、IMU、Odometry 同一 session digest；
- 山路目标、观测站、路线和人工接管接入原生任务链；
- 缺证据、传感器过期和受阻都能正确失败；
- 不以 HUD 或动画制造任务完成。

## 13. 测试与性能目标

自动化测试：

- `WorldPackage` 解析和摘要稳定性；
- UE Data Layer 状态转换；
- 视觉实体 ID 与 PhysicsPlan 一致性；
- 高度场边界、采样和坐标转换；
- 碰撞代理完整性；
- Thunder 出生、坡面、岩石接触和 Reset；
- Snapshot generation、sequence 和时间戳一致性；
- 传感器就绪和旧帧丢弃；
- 会话失败后不显示假 `RUNNING`。

性能目标沿用当前可玩配置作为测量目标：1920×1080 视口，RGB/Depth 640×480@30 Hz。RTF、真实传感器频率、CPU/GPU 帧时间、World Partition 加载耗时和内存必须实测后记录；本设计不预先宣称达标。

## 14. 决策与延期

已决定：

- UE World Partition 作为山地连续世界组织方式；
- Data Layers 管理基础世界、植被、观测站、路线和诊断层；
- MuJoCo 是机器人和接触唯一物理权威；
- WorldPackage 是 UE 和 MuJoCo 的统一实体入口；
- 首版只做静态会话组合，不做运行时山体增删。

延期：

- 是否采购 Mountain Forest 或 Real3D 山顶天文台包：等待授权和成本确认；
- Valley 样例中哪些资产可以再分发：等待 Epic/Quixel 许可核对；
- 山地是否加入动态行人/车辆：等基础地形和机器人运动验收后决定；
- 是否启用 UE5.8 的新地形/植被实验能力：等最小场景性能基线后决定。

延期触发器：外部环境工程成功在 UE5.8 打开、WorldPackage 完成、Thunder 基础运动通过后，再重新评估上述项目。
