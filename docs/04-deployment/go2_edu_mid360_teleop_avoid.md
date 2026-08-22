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
Web / lingtu-drive                    v
  -> typed operator motion -> lt-nav (teleop_avoid)
  -> LocalPlanner -> PathFollower -> final safety
  -> rt/nav/cmd_vel                   # 唯一最终速度流
  -> lt-driver
  -> Unitree SDK2 SportClient::Move()
  -> Go2
```

`teleop_avoid` 的声明进程必须恰好覆盖 `lidar`、`imu`、`slam`、`maps`、
`traversability`、`nav`、`driver` 和 `host`。相机不属于这条链。

Web 活动速度样本以 50 Hz 发布；原生导航安全与局部规划循环为 20 Hz；
Go2 驱动控制循环为 50 Hz。400 ms 是空闲时的控制权续约间隔，不是释放
超时；一秒内既无速度样本也无 heartbeat 时，Web lease 才过期。松开最后
一个方向键、按 Space、页面失焦或连接断开都必须立即进入零速/hold。

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

- `config/robot_config.yaml`：机械 `T_body_lidar`。
- `config/robots/unitree/go2/sensors/mid360_fastlio2.yaml`：Fast-LIO2
  包内 `T_imu_lidar` 与求解后的 `T_body_imu`。
- `src/runtime/runtime_interface.py`：运行时 `go2_mid360` 外参 profile。

这些文件目前在数值上保持一致，但 `calibration.status` 必须继续为
`unverified`，直到安装在这台 Go2 上的支架通过静态平面和原地旋转验证。
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
| NX 一键部署 | 未完成 | `deploy_robot.sh` 构建清单仍不完整 |
| NX Python 运行时 | 必须现场确认并固定同一个 Python 3.10+ 虚拟环境 | 部署前必须收口 |
| Go2 端到端避障实机报告 | 没有 | 当前不能宣称功能已交付 |

## 6. 部署前置条件和当前限制

本节描述进入实机步骤前必须满足的条件，不在这里维护研发排期。跨 Product
的实施优先级和完成状态统一记录在 `docs/plans/current-roadmap.md`。

### 前置条件 1：补全 `deploy_robot.sh`

当前脚本只构建 maps C API、导航 kernel/endpoint、相机和驱动，不能产出
`teleop_avoid` 的完整进程闭包。它必须按 Product 选择并验证至少以下产物：

```text
build/livox_sdk2_stream/livox_sdk2_stream
build/slam_core/slamd
build/maps/mapd
build/dds_probe/lingtu_dds_probe
build/nav_endpoint/navd
build/nav_endpoint/lingtu_traversability_dds
build/driver/lingtu_driver
```

相机只应在 Product 声明 `camera` 时构建；`teleop_avoid` 不应无条件构建
Orbbec。修复前不要把 `deploy_robot.sh` 当成可重复的一键生产部署。

### 前置条件 2：固定 Python 3.10+ 解释器

LingTu 要求 Python 3.10 或更高，但 `deploy_robot.sh` 默认使用 `python3`，
`lt-host.service` 也把 `LINGTU_PYTHON` 写成 `python3`。必须让构建、
ProductControl、Host 和诊断命令统一指向 NX 上同一个受控虚拟环境，不能
依赖系统 Python 版本或交互 shell 的 PATH。

### 前置条件 3：解除 SLAM 与相机标定的错误耦合

当前启动预检把“启用任意 SLAM profile”同时解释为“必须有相机标定”。
`teleop_avoid` 明确不使用相机，因此预检必须根据 Product 的真实 camera
能力决定 `require_camera`，根据 SLAM 决定 `require_slam`，不能把两者绑定。

### 前置条件 4：提供无运动的物理标定入口

传感器相关 Product 会在物理标定未批准时 fail closed，但采集静态平面和
原地旋转证据又需要启动 MID-360/SLAM。需要提供一个明确的无运动标定入口：

- 只启动或直接运行 LiDAR、IMU、SLAM 和记录工具。
- 不启动运动驱动，不产生 `rt/nav/cmd_vel`。
- 输出静态平面、旋转一致性和时间同步证据。
- 验证通过后才人工更新 `calibration.status: verified`。

在这个入口落地之前，不允许用临时修改 `calibration.status` 的方式绕过预检。

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
  src/runtime/tests/test_robot_sensor_mounts.py::test_mid360_contract_keeps_go2_and_thunder_mounts_distinct \
  src/runtime/tests/test_robot_sensor_mounts.py::test_robot_specific_fastlio_config_reconstructs_its_lidar_mount \
  src/runtime/tests/test_thunder_deployment_entrypoints.py::test_thunder_traversability_dds_service_runs_cpp_runtime

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

在 `deploy_robot.sh` 修复前，使用明确的构建清单：

```bash
bash scripts/build/build_livox_sdk2_stream.sh
LINGTU_SLAM_BUILD_DDS_RUNTIME=ON bash scripts/build/build_slam_core.sh
bash scripts/build/build_mapd.sh
bash scripts/build/build_dds_probe.sh
bash scripts/build/build_nav_endpoint.sh
LINGTU_DRIVER_BACKEND=go2 bash scripts/build/build_driver.sh

# 使用浏览器键盘时还需要 Web 静态资源
(cd web && npm ci && npm run build)

# 只安装 unit；不要手工启动整组服务
LINGTU_DRIVER_BACKEND=go2 \
  bash scripts/deploy/thunder/install_services.sh field-cpp
```

`scripts/deploy/thunder/` 是当前共享 field service catalog 的历史目录名，
并不表示 Go2 运行 Thunder 后端。机器人实现仍由 RobotConfig 和 RunPlan
选择。

### D. 无运动启动与数据就绪

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

# 只读契约 gate：验证 exact RunPlan、角色、topic 和运行策略
PYTHONPATH=/opt/lingtu/current/src "${LINGTU_PYTHON}" \
  scripts/gates/thunder_service_readiness_collect.py \
  --teleop-avoid-stage contract --strict --pretty \
  --json-out /tmp/go2-teleop-avoid-contract.json

# 只读运动 gate：验证传感新鲜度、InputGate、Go2 控制状态、最终零速和精确 driver ACK
PYTHONPATH=/opt/lingtu/current/src "${LINGTU_PYTHON}" \
  scripts/gates/thunder_service_readiness_collect.py \
  --teleop-avoid-stage motion --strict --pretty \
  --json-out /tmp/go2-teleop-avoid-motion.json
```

`thunder_service_readiness_collect.py` 是现有共享 field collector 的历史文件
名；`--teleop-avoid-stage motion` 会按状态中的 `backend=go2` 检查 Unitree
SDK2 控制，不会切换到 Thunder/DOSO。两个 gate 都是只读的，不申请控制权、
不发布命令。

通过条件：

- `lt-lidar`、`lt-slam`、`lt-maps`、`lt-terrain`、`lt-nav`、`lt-driver`、
  `lt-host` 就绪，`lt-camera` 未被选择。
- MID-360 点云和 IMU 持续更新。
- SLAM 处于 `TRACKING`，里程计、registered cloud 和 traversability 新鲜。
- `/nav/cmd_vel` 为零，且只有 `navd` 是最终命令写入者。
- Go2 驱动报告状态新鲜、SDK2 调用可确认，未出现旧命令恢复。
- 两份 gate 报告均为 `ok=true`、`blockers=[]`；motion gate 另外必须是
  `nonzero_motion_allowed=true`、`command_published=false` 和
  `authority_acquired=false`。

### E. 开阔区域的限界运动

先验证停止，再验证运动。操作员必须能直接看到机器人：

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

### F. 单障碍局部绕障

1. 在平整开阔地放置 MID-360 能稳定观测的实体障碍物。
2. 从障碍物约 2–3 m 前方，以 `0.15–0.20 m/s` 请求向前。
3. 记录 operator sample、local path、traversability、final cmd_vel、driver ACK
   和里程计。
4. 分别验证正前方障碍、略偏左和略偏右三种位置。

通过条件：

- 有安全候选路径时，LocalPlanner 在当前 `2.0 m` horizon 和 `55 deg` 最大
  偏转范围内绕行。
- 没有安全候选路径或任一关键数据过期时，最终输出保持零，不允许盲目前进。
- `teleop_avoid` 不自动倒车、不自动恢复；阻塞时由操作员重新给出意图。
- 障碍移除后风险栅格能恢复，不留下永久幽灵障碍。

## 8. 最终交付证据

Go2 避障只有在以下证据同时存在时才算完成：

- 目标 NX 的版本、构建日志和完整原生二进制清单。
- 激活的 `teleop_avoid` RunPlan 和 product session ID。
- 网卡、MID-360、Go2 运动主机连通性记录。
- 安装支架的外参验证记录和 `calibration.status: verified` 审批依据。
- 无运动 doctor 报告。
- 开阔地六方向限界运动记录及全部停止场景。
- 单障碍绕行、无路可走时零输出、障碍清除后的恢复记录。
- 测试结束后的 Product stop、最终零速和 driver ACK。

没有这些证据时，只能说某个组件或链路已经接入，不能说 Go2 实机避障已经交付。
