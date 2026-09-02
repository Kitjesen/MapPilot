# LingTu Go2 EDU + 外置 MID-360 实机避障部署手册

状态：2026-08-23，部署准备中，尚未完成 Go2 实机避障验收。

本文由 LingTu 项目自行编写，不是 Unitree 官方手册。Unitree 和 Livox
资料只作为硬件接口与名义安装参数的参考来源。真正的运行契约由 LingTu
的 `RobotConfig`、`teleop_avoid` Product、RunPlan 和原生服务实现决定。

## 1. 本次目标

在 Go2 EDU 扩展 Linux/NX 计算板上运行 LingTu，使操作员直接使用
W/S/A/D/Q/E 控制机器人，同时用外置 Livox MID-360 的点云和内置 IMU
完成实时里程计、地形风险计算和局部绕障。

| 项目 | 本次选择 |
| --- | --- |
| 生产运行主机 | Go2 扩展 Linux/NX 板 |
| 临时开发桥 | Sunrise；生产运行不依赖它 |
| LingTu Product | `teleop_avoid` |
| 运动接口 | Unitree SDK2 `SportClient` |
| 避障传感器 | 外置 Livox MID-360 |
| SLAM 输入 | MID-360 点云和 MID-360 包内 IMU |
| Go2 内置 L1 | 不使用 |
| 相机 | 不参与本次避障验收 |
| 保存地图 | 不要求；SLAM 运行在 `mapping` 模式 |

文件名不再包含 `L1`，因为 Go2 内置 `radar_joint` 与本次外置 MID-360
是两个不同传感器，不能共用外参。

## 2. 运行拓扑

```text
MID-360
  -> lt-lidar: rt/lidar/raw_frame + rt/imu/raw
  -> lt-slam: odometry + registered cloud
  -> lt-maps + lt-terrain: live scene + traversability
                                      |
Web Teleop / lingtu-drive             v
  -> typed operator motion -> lt-nav (teleop_avoid)
  -> LocalPlanner -> PathFollower -> final safety
  -> rt/nav/cmd_vel                   # 唯一最终速度流
  -> lt-driver
  -> Unitree SDK2 SportClient::Move()
  -> Go2
```

`teleop_avoid` 的声明进程必须恰好覆盖 `lidar`、`imu`、`slam`、`maps`、
`traversability`、`nav`、`driver` 和 `host`。相机不属于这条链。

Web 和 `lingtu-drive` 的活动速度样本以 50 Hz 发布；原生导航安全与局部规划循环为 20 Hz；
Go2 驱动控制循环为 50 Hz。Web 是持续键盘控制入口；`lingtu-drive` 只用于
最长 5 秒的单方向监督动作检查。Web 空闲时不发送 heartbeat，也没有独立 Web
Lease。`navd` 内部的一秒原生 Lease 只负责失联归零；下一次按键需要时由 Gateway
自动完成 CLAIM、epoch 和零速屏障恢复。松开最后一个方向键、按 Space、页面失焦或
连接断开都必须立即进入零速/hold；不再使用“恢复控制”按钮。

Web 操作顺序是：打开场景中的“遥控”面板，确认当前 Product 为
`teleop_avoid`，点击“连接”，按住 W/S/A/D/Q/E 移动；Shift 是 40% 精细
模式，Space 或“保持”立即归零，“停车”进入原生停止路径。Gateway 显示的
入口回执只说明意图已排队，必须继续查看最终速度和电机确认。

## 3. 网络和设备地址

下面是当前 Go2 EDU 扩展坞的名义配置，不代表所有 Go2 都必须使用相同
主机地址。部署前以目标 NX 的实际网卡名为准。

| 设备 | 当前地址 | 用途 |
| --- | --- | --- |
| NX 机器人侧网卡 | `192.168.123.18/24` | LingTu、Livox host、SDK2 运行主机 |
| 外置 MID-360 | `192.168.123.20` | 点云和包内 IMU |
| Go2 运动主机 | `192.168.123.161` | 启动探测；不是 SDK2 的目标地址 |
| `driver.network_interface` | 当前配置为 `eth0` | SDK2 DDS 发现使用的网卡 |
| Sunrise | 现场地址，不进入 RobotConfig | 临时 SSH/转发开发机 |

机器人侧有线网卡使用静态地址且不配置默认网关。SDK2 通过
`driver.network_interface` 做 DDS 发现；`192.168.123.161` 只用于启动前
连通性探测。NX 的 SSH 地址属于部署状态，不属于 Product 或机器人运动接口。

部署前必须看到：

```bash
ip -br -4 addr
ip route
ping -c 3 192.168.123.161
ping -c 3 192.168.123.20
```

## 4. MID-360 名义外参

当前配置针对 Go2 EDU 扩展坞和 Unitree 配套 MID-360 支架：

```text
T_body_lidar.xyz = [0.16143, 0.0, 0.12262] m
T_body_lidar.rpy = [0.0, 0.22689280275926285, 0.0] rad
                 = [0.0, 13.0, 0.0] deg
```

来源层级必须区分：

1. [Unitree 官方 Go2 URDF](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go2_description/urdf/go2_description.urdf)
   提供 `base -> imu` 平移 `[-0.02557, 0, 0.04232]`。同一 URDF 中的
   `radar_joint` 是 Go2 内置鼻端雷达，不是外置 MID-360。
2. [Go2 EDU 扩展坞 SLAM 接口归档镜像](https://github.com/mamdaliof/Unitree-GO2-Quadruped-Robot-for-autonomus-navigation-and-SLAM/blob/master/Go2_Documentation/archive/Software_Interface_Services/SLAM_and_Navigation_Services_Interface.md)
   给出内部 IMU 到配套 MID-360 的名义位姿 `[0.1870, 0, 0.0803]`、
   pitch `+13 deg`。这是第三方仓库中的归档镜像，不是 Unitree 当前官方
   仓库，因此只能作为名义参考。

配置写入位置：

- `config/robots/unitree/go2/robot.yaml`：机械 `T_body_lidar`。
- `config/robots/unitree/go2/sensors/mid360_fastlio2.yaml`：Fast-LIO2
  包内 `T_imu_lidar` 与求解后的 `T_body_imu`。
- `src/runtime/runtime_interface.py`：运行时 `go2_mid360` 外参 profile。

这些文件目前在数值上保持一致。`calibration.slam.status` 只代表这台 Go2
的 MID-360/LiDAR-IMU 外参；`calibration.camera.status` 独立维护，禁止用
SLAM 证据批准相机。
`tools/calibration/verify.py` 只能检查配置数值和变换链一致性，不能证明
物理支架安装正确。

## 5. 当前完成度

| 能力 | 当前状态 | 结论 |
| --- | --- | --- |
| `teleop_avoid` Product 和进程图 | 已实现并有编译契约测试 | 可以继续部署准备 |
| Go2 SDK2 速度转发 | 已接入 `lt-driver` | 不能替代完整避障验收 |
| Go2/DOSO Thunder V4 MID-360 外参隔离 | 已实现并有定向测试 | 配置没有串线 |
| 地形服务读取 RunPlan 外参 | 已实现并有定向测试 | 不再硬编码 Thunder 位姿 |
| 外置 MID-360 物理标定 | 未完成 | 阻止实机避障放行 |
| NX Product 部署入口 | 已按 Product 生成构建清单 | 仍需 NX 实机执行证据 |
| NX Python 运行时 | 必须现场确认并固定同一个 Python 3.10+ 虚拟环境 | 部署前必须收口 |
| Go2 端到端避障实机报告 | 没有 | 当前不能宣称功能已交付 |

## 6. 部署前置条件和当前限制

本节描述进入实机步骤前必须满足的条件，不在这里维护研发排期。跨 Product
的实施优先级和完成状态统一记录在 `docs/plans/current-roadmap.md`。

### 已完成 1：`deploy_robot.sh` 按 Product 构建

脚本现在从 Product 声明的逻辑进程角色生成构建列表。`teleop_avoid` 会构建
以下原生运行时：

```text
build/livox_sdk2_stream/livox_sdk2_stream
build/slam_core/slamd
build/maps/mapd
build/dds_probe/lingtu_dds_probe
build/nav_endpoint/navd
build/nav_endpoint/lingtu_traversability_dds
build/driver/lingtu_driver
build/native-recording/lingtu_recorder
```

相机只在 Product 声明 `camera` 时构建；`teleop_avoid` 跳过 Orbbec，`map`
和 `inspection` 保留相机构建。代码路径和无副作用计划模式已验证，完整原生
编译仍需在 NX 上执行后才能形成实机部署证据。

### 已完成 2：统一 Python 3.10+ 解释器

字段部署入口现在统一解析并验证 `LINGTU_PYTHON`，要求 Python 3.10 或更高，
且保留虚拟环境的 `bin/python` 路径而不解引用到系统 Python。service 安装器
把该路径写入 `/opt/lingtu/config/python.env`；Host、Product session guard 和
Livox 配置生成复用同一解释器。DDS readiness 直接调用原生 probe，不再额外
启动 `dds_probe.py`。

### 已完成 3：SLAM 与相机标定已解耦

启动预检现在仅在语义/相机能力启用时要求相机标定；SLAM profile 只控制
`require_slam`。`teleop_avoid` 因启用 SLAM 而错误要求相机标定的问题已有
回归测试覆盖。

### 已完成 4：无运动传感采集入口

传感器相关 Product 会在物理标定未批准时 fail closed，但采集静态平面和
原地旋转证据又需要启动 MID-360/SLAM。专用入口为：

- `scripts/gates/field/go2_mid360_no_motion.sh` 只启动 LiDAR 和 SLAM，并在
  同一进程内采样 native DDS Topic。
- 启动前和采样后都拒绝存在 `navd`、`lingtu_driver` 或对应 systemd unit。
- 必须看到持续点云、IMU、里程计、registered cloud，同时
  `rt/nav/cmd_vel` 存在探测结果且样本数严格为零。
- 把生成的 Livox JSON、SLAM 状态、点云快照、日志和 `report.json`
  保存在独立证据目录，退出时停止两个临时进程。
- 此 gate 不自动批准物理外参；证据人工确认后才更新
  `calibration.slam.status: verified`，相机状态不会随之改变。

不允许用临时修改 `calibration.slam.status` 的方式绕过预检。

### 后续隔离项：清理 Go2 机器人配置

当前 Go2 `robot_config.yaml` 仍带有 Thunder V4 相机外参和 WTRTK GNSS
配置。它们不参与本次 `teleop_avoid`，但在启用 Go2 相机或 GNSS 前必须
移入各自机器人配置，避免把无关硬件误认为 Go2 标定结果。

## 7. 分阶段部署与验收

每一级只证明自己的范围。前一级失败时，不进入后一级。

以下命令统一使用目标 NX 上的 LingTu Python 3.10+ 虚拟环境。先设置一次
绝对路径；不要把示例占位符原样执行：

```bash
export LINGTU_PYTHON=/ABSOLUTE/PATH/TO/LINGTU_VENV/bin/python
"${LINGTU_PYTHON}" --version
```

### A. 离线代码与 RunPlan

```bash
"${LINGTU_PYTHON}" -m pytest -q \
  tests/runtime/test_robot_sensor_mounts.py::test_mid360_contract_keeps_go2_and_thunder_mounts_distinct \
  tests/runtime/test_robot_sensor_mounts.py::test_robot_specific_fastlio_config_reconstructs_its_lidar_mount \
  tests/runtime/test_thunder_deployment_entrypoints.py::test_thunder_traversability_dds_service_runs_cpp_runtime

bash scripts/lingtu --robot unitree/go2 --env real \
  switch teleop_avoid --dry-run --json
```

通过条件：RunPlan 选择 Go2、`eth0`（或目标实际网卡）、上述八个角色、
`mapping` SLAM、无 saved map、无 camera。

### B. NX 网络和依赖

```bash
uname -m                         # 期望 aarch64
"${LINGTU_PYTHON}" --version     # 期望 >= 3.10
ip -br -4 addr
ping -c 3 192.168.123.161
ping -c 3 192.168.123.20
```

同时确认 Unitree SDK2、Livox SDK2、CycloneDDS、Eigen、PCL 和 yaml-cpp 能被
对应构建脚本找到。依赖安装详见 `docs/01-getting-started/BUILD_GUIDE.md`。

### C. 构建和安装原生服务

先用无副作用模式检查 `teleop_avoid` 的构建计划，再执行部署：

```bash
LINGTU_DEPLOY_PLAN_ONLY=1 bash scripts/deploy/deploy_robot.sh teleop_avoid
bash scripts/deploy/deploy_robot.sh teleop_avoid
```

`scripts/deploy/thunder/` 是当前共享 field service catalog 的历史目录名，
并不表示 Go2 运行 Thunder 后端。机器人实现仍由 RobotConfig 和 RunPlan
选择。

### D. 无运动传感链与外参证据

保持 Go2 不动，且不要启动任何 Product：

```bash
bash scripts/gates/field/go2_mid360_no_motion.sh --seconds 10 --json
```

通过条件：报告 `ok=true`，四条传感/SLAM Topic 达到最低频率，
`rt/nav/cmd_vel.samples=0`，SLAM 为 `TRACKING`。随后检查证据目录中的点云
快照，完成静态地面与旋转一致性记录。该命令不会修改
`calibration.slam.status`。

### E. Product 无运动启动与数据就绪

完成前置条件 3、前置条件 4 和物理标定后，机器人支撑稳固、周围清空并
准备物理急停：

```bash
bash scripts/lingtu --robot unitree/go2 --env real switch teleop_avoid
bash scripts/lingtu --robot unitree/go2 --env real status

PYTHONPATH=/opt/lingtu/current/src "${LINGTU_PYTHON}" \
  -m diagnostics.field.doctor --non-motion --json --strict

jq . /dev/shm/lingtu/driver_status.json
jq . /dev/shm/lingtu/nav_endpoint_status.json
jq . /dev/shm/lingtu/traversability_status.json
jq . /dev/shm/lingtu/mapd_status.json

# 连续采样地图链路；输出包含 hz、max_gap_s、reset_epoch、sequence、generation 和 live
/opt/lingtu/current/bin/lingtu_dds_probe --json --seconds 5 \
  --domain "${LINGTU_DDS_DOMAIN_ID:-0}" \
  /slam/map_observation /maps/state /maps/local_collision \
  /maps/occupancy /maps/scene | tee /tmp/go2-mapd-dds.json

# 只读契约 gate：验证 exact RunPlan、角色、topic 和运行策略
PYTHONPATH=/opt/lingtu/current/src "${LINGTU_PYTHON}" \
  -m diagnostics.field.service_readiness \
  --teleop-avoid-stage contract --strict --pretty \
  --json-out /tmp/go2-teleop-avoid-contract.json

# 只读运动 gate：验证传感新鲜度、InputGate、Go2 控制状态、最终零速和同源时序关联 driver ACK
PYTHONPATH=/opt/lingtu/current/src "${LINGTU_PYTHON}" \
  -m diagnostics.field.service_readiness \
  --teleop-avoid-stage motion --strict --pretty \
  --json-out /tmp/go2-teleop-avoid-motion.json
```

`diagnostics.field.service_readiness` 是共享 field collector
名；`--teleop-avoid-stage motion` 会按状态中的 `backend=go2` 检查 Unitree
SDK2 控制，不会切换到 Thunder/DOSO。两个 gate 都是只读的，不申请控制权、
不发布命令。

通过条件：

- `lt-lidar`、`lt-slam`、`lt-maps`、`lt-terrain`、`lt-nav`、`lt-driver`、
  `lt-host` 就绪，`lt-camera` 未被选择。
- MID-360 点云和 IMU 持续更新。
- SLAM 处于 `TRACKING`，里程计、registered cloud 和 traversability 新鲜。
- 连续 10 Hz SLAM 观测期间，`/maps/local_collision` 应接近 10 Hz，至少达到
  `8 Hz`，且 `max_gap_s <= 0.5`；`/maps/state`、occupancy 和 scene 正常约为
  `2 Hz`。所有地图输出的 `reset_epoch` 一致，sequence/generation 不倒退。
- `/nav/cmd_vel` 为零，且只有 `navd` 是最终命令写入者。
- Go2 驱动报告状态新鲜、SDK2 调用可确认，未出现旧命令恢复。
- 两份 gate 报告均为 `ok=true`、`blockers=[]`；motion gate 另外必须是
  `nonzero_motion_allowed=true`、`command_published=false` 和
  `authority_acquired=false`。

### F. 开阔区域的限界运动

先验证停止，再验证运动。操作员必须能直接看到机器人。首次六方向检查可
使用下面的短动作工具；持续操控使用 Web 遥控面板：

```bash
lingtu-drive forward --speed 0.15 --seconds 1 --robot unitree/go2
lingtu-drive backward --speed 0.15 --seconds 1 --robot unitree/go2
lingtu-drive left --speed 0.15 --seconds 1 --robot unitree/go2
lingtu-drive right --speed 0.15 --seconds 1 --robot unitree/go2
lingtu-drive turn-left --speed 0.20 --seconds 1 --robot unitree/go2
lingtu-drive turn-right --speed 0.20 --seconds 1 --robot unitree/go2
```

通过条件：运动连续、不再“走一下停一下”；每次命令结束、页面失焦、断开
WebSocket 或按 Space 后，都在配置的 deadman/command timeout 内归零。

### G. 单障碍局部绕障

1. 在平整开阔地放置 MID-360 能稳定观测的实体障碍物。
2. 从障碍物约 2–3 m 前方，以 `0.15–0.20 m/s` 请求向前。
3. 记录 operator sample、local path、traversability、final cmd_vel、driver ACK
   和里程计。
4. 分别验证正前方障碍、略偏左和略偏右三种位置。

通过条件：

- 有安全候选路径时，LocalPlanner 在当前 `2.0 m` horizon 和 `55 deg` 最大
  偏转范围内绕行。
- 正前方近障进入安全膨胀区、但没有侵入 Go2 物理足迹时，优先生成
  “侧移 -> 向前越障 -> 回归原意图线”的全向路径；该段保持机身朝向，
  由 `vy` 完成侧移，不要求先原地转向。
- 绕行路径在连续控制周期中保持提交状态，直到回归原意图线、操作员改变
  方向、松开控制或最终安全层否决，不能每周期重新选择一条平行偏移线。
- 自主导航使用同一全向近障策略；局部路径回归全局路线后继续执行原目标，
  不因一次近场障碍取消目标。
- 没有安全候选路径或任一关键数据过期时，最终输出保持零，不允许盲目前进。
- 最终安全层是路径执行前的最后否决边界：新障碍、硬地形、真实足迹碰撞或
  两侧通道都封堵时停车；它不替代 LocalPlanner 生成绕行路径。
- `teleop_avoid` 不自动倒车；两侧都封堵并停车后，由操作员重新给出意图。
- 障碍移除后风险栅格能恢复，不留下永久幽灵障碍。

### H. 自主导航恢复旋转（`nav` Product）

这项不是 `teleop_avoid` 的自动行为；必须切换到获批准的 `nav` Product，使用
低速短目标并由操作员全程看护。先在开阔区域验证左右原地旋转和 Space/取消
立即归零，再布置“平移候选被封堵、至少一侧旋转走廊开放”的障碍形状。

目标 50 Hz 控制层记录每个 20 ms 样本；若现场版本仍为 20 Hz，则明确按
实际 50 ms 周期记录，不得把插值结果当作实测。同步记录
里程计 yaw、`recovery_action`、`recovery_attempt`、
`recovery_rotation_target_rad`、`recovery_progress`、`recovery_reason`、
最终 `cmd_vel` 和 driver ACK。通过条件：

- 旋转方向和角度来自已验证的开放走廊，不固定为约 20°；
- yaw 里程计持续推进，动作完成后立即输出零，并等待新 cloud 与
  traversability generation 后才重新规划；
- 若新观测下仍无路径，`recovery_action` 按配置进入下一动作且
  `recovery_attempt` 单调增加；达到 `max_attempts` 后保持零速并报告耗尽，
  不能重新从第一个动作无限循环；
- 旋转期间任何足迹角点障碍、硬地形、地图覆盖不足或数据过期都使最终安全层
  归零；
- 记录实际角速度、角加速度、停止延迟和停止后 yaw 超调。当前软件测试覆盖
  50 Hz 角速度整形与足迹/距离边界，但这些记录仍是 Go2 实机验收必需证据。

## 8. 最终交付证据

Go2 避障只有在以下证据同时存在时才算完成：

- 目标 NX 的版本、构建日志和完整原生二进制清单。
- 激活的 `teleop_avoid` RunPlan 和 product session ID。
- 网卡、MID-360、Go2 运动主机连通性记录。
- 安装支架的外参验证记录和 `calibration.slam.status: verified` 审批依据。
- 无运动 doctor 报告。
- 开阔地六方向限界运动记录及全部停止场景。
- 单障碍绕行、无路可走时零输出、障碍清除后的恢复记录。
- 测试结束后的 Product stop、最终零速和 driver ACK。

没有这些证据时，只能说某个组件或链路已经接入，不能说 Go2 实机避障已经交付。
