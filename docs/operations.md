# Operations

**Status:** Current deployment, release, and robot operations guide
**Audience:** Release engineers, field operators, and robot maintainers
**Runs on:** Linux field releases and supported robot targets

Product operations use `switch / status / stop` through ProductControl. Build,
packaging, installation, service diagnosis, and Product lifecycle remain
separate responsibilities.

调试速查：[Go2/NX 连接与当前交接](operations/go2-offline-mapping.md) ·
[早期 Go2 连接记录](../config/robots/unitree/go2/README.md#连接-nx-与网络核对) ·
[MuJoCo SCAN 时钟修复记录](#mujoco-scan-clock-debugging-status)。

## Production layout

```text
/opt/lingtu/
  current -> releases/<version>/
  releases/<version>/
    bin/
    lib/
    etc/lingtu/
    share/lingtu/
  config/                 mutable machine configuration
  logs/

/var/lib/lingtu/maps/     mutable saved maps
```

Services execute from `/opt/lingtu/current`. They do not execute a developer
`build/` tree. Credentials, machine network settings, logs, and mutable maps
stay outside the immutable release.

## Build, package, install

The three steps are explicit:

```text
make build
  -> build/

scripts/deploy/package_native_release.sh
  -> install/linux-<arch>/Release/{bin,lib,etc,share}
  -> dist/

install_native_release.sh
  -> /opt/lingtu/releases/<version>/
  -> /opt/lingtu/current
```

Build a complete release input set:

```bash
LINGTU_DRIVER_BACKEND=go2 make build BUILD_TYPE=Release
bash scripts/deploy/package_native_release.sh vX.Y.Z dist
```

Use `LINGTU_DRIVER_BACKEND=doso` for Thunder. `deploy_robot.sh` is a separate,
Product-scoped checkout workflow; its build output is not a complete release
package input.

The OTA packager targets Linux field releases. Windows x64 validates the same
`bin/lib/etc/share` shape through CMake install under
`install/windows-x64/Release`; it is not passed through the Linux OTA script.

## Install and activate

After transferring and extracting the package:

```bash
sudo bash /tmp/lingtu-X.Y.Z-aarch64-native-release/install_nav.sh
```

The installer validates the package, stages a versioned release, atomically
switches `current`, and rolls back on failure. If a Product was active, it
reapplies the same Product and map through ProductControl rather than manually
starting a service list.

On a first install, install the unit catalog before switching a Product:

```bash
export LINGTU_PYTHON=/ABSOLUTE/PATH/TO/LINGTU_VENV/bin/python
bash /opt/lingtu/current/scripts/deploy/thunder/install_services.sh field-cpp
bash /opt/lingtu/current/scripts/lingtu \
  --robot unitree/go2 --env real switch nav --map MAP_NAME
```

## Canonical-layout transition

The temporary dual-layout transition has three release stages:

1. Keep the old units installed and activate a dual-layout release.
2. Publish and activate the next dual-layout release while the old units still
   run, making the first dual-layout release the rollback target. Then install
   canonical units with `scripts/deploy/thunder/install_services.sh`. The
   installer rejects missing, legacy-only, and canonical-only `current` trees.
3. After the dual-layout release leaves the rollback window, stop including
   the legacy `build/` copy in later packages.

Current units execute the canonical install tree. The legacy packaged copy
exists only to keep rollback to an older unit set atomic during the defined
window.

## Product operations

The robot-side adapter executes `python -m lingtu.control`:

```bash
bash scripts/lingtu --robot unitree/go2 --env real status --json
bash scripts/lingtu --robot unitree/go2 --env real switch map
bash scripts/lingtu --robot unitree/go2 --env real switch nav --map MAP_NAME
bash scripts/lingtu --robot unitree/go2 --env real stop
```

ProductControl validates the resolved RunPlan, stops conflicting motion,
stages map/session state, starts declared processes in dependency order, checks
readiness, and commits only on success.

Native processes 可按各自角色独立观测；它们的启动、停止、回滚和当前状态由当前 RunPlan 与 ProductControl 统一拥有。
`systemctl` is for diagnosis, not a second Product startup path.

## Web mapping and navigation switches

The Web controls are **开始建图 → 累计地图 → 保存地图 → 使用此地图导航**.
Saving opens the saved PCD preview. Loading a map ends the current mapping
session; save any recent scans before confirming. A successful switch does not
send a motion goal: the existing map, localization, and navigation readiness
checks still gate goal selection and execution. `?observe=1` remains read-only.

The new controls require `lt-control.service` alongside the existing Host. This
service is outside the Product's process set and remains running during Host
replacement. It only listens on robot loopback port 5051; Gateway forwards the
three `/api/v1/product-control` routes. No extra laptop software or internet
connection is required. A running Host is needed to access these Web routes.

After installing a release containing the new service, configure the fixed
robot in `/etc/lingtu/control.env` on the robot:

```ini
LINGTU_ROBOT=unitree/go2
# Set only when the deployed map/nav Products should include the camera:
# LINGTU_CONTROL_VARIANT=camera
```

Use the actual robot profile for other platforms. Install the optional
transport and start it once; it does not start or switch a Product by itself:

```bash
sudo bash /opt/lingtu/current/scripts/deploy/thunder/install_catalog_service.sh control
sudo systemctl enable --now lt-control.service
curl http://127.0.0.1:5051/status
```

The service and CLI must use the same current-run directory (`/run/lingtu` in
the field unit). On Windows simulation the equivalent standalone process is
`uv run --locked python -m lingtu.control serve --robot unitree/go2 --env sim
--backend mujoco --state-dir PATH_TO_EXISTING_SESSION_ROOT`, using the actual
session path. Keep the existing simulation supervisor as process owner.

The browser retains a request ID before submission. After a Host disconnect it
queries that request instead of repeating the switch. A service restart marks
unfinished requests `interrupted`; inspect the current Product before retrying.
Receipts are kept in `web-control-operations.json` under the state directory
(latest 32 requests; `/run` is not durable across a robot reboot). If Host cannot
recover, inspect `http://127.0.0.1:5051/operations/REQUEST_ID` on the robot and use
the normal ProductControl CLI; the Web must not report success from a timeout.

This transport has local contract coverage. Deployment, complete browser
interaction, and robot mapping/navigation acceptance are separate gates.

## Service roles

| Unit | Role |
| --- | --- |
| `lt-lidar` | Native MID-360/IMU ingestion |
| `lt-slam` | Native mapping/localization |
| `lt-maps` | Live map state and saved-map operations |
| `lt-terrain` | Unique native traversability writer |
| `lt-nav` | Navigation state and final logical command owner |
| `lt-driver` | Unique physical robot command writer |
| `lt-camera` | Native camera capture and publication |
| `lt-explore` | Native exploration policy when declared |
| `lt-host` | Python Host with Gateway, Agent, MCP, and adapters |
| `lt-control` | Optional Host-independent transport for ProductControl; not a Product process |

There is no separate Gateway unit; Gateway runs inside `lt-host`. Unit
liveness alone does not prove Product readiness.

## No-motion diagnosis

Start from read-only state:

```bash
bash scripts/lingtu status
PYTHONPATH=src python -m diagnostics.field.doctor --non-motion --json --strict
PYTHONPATH=src python -m diagnostics.field.soak \
  --duration 120 --interval 2 --json --strict
```

Inspect a role only after Product status identifies it as declared:

```bash
journalctl -u lt-host.service -n 80 --no-pager
journalctl -u lt-slam.service -n 80 --no-pager
journalctl -u lt-lidar.service -n 80 --no-pager
journalctl -u lt-driver.service -n 80 --no-pager
```

For saved-map navigation, `TRACKING` alone is insufficient. Verify exact map
identity, valid `map -> odom`, fresh odometry, navigation readiness, zero
barriers, and driver state.

## Recording and replay

Record native DDS to MCAP without a camera:

```bash
/opt/lingtu/current/bin/lingtu_recorder record \
  --output-dir SESSION_DIR --dds on --camera off
```

Inspect or dry-run replay:

```bash
/opt/lingtu/current/bin/lingtu_recorder status
/opt/lingtu/current/bin/lingtu_dds_player SESSION_DIR/dds/sensors.mcap --info
/opt/lingtu/current/bin/lingtu_dds_player \
  SESSION_DIR/dds/sensors.mcap --dry-run
```

Recording does not authorize motion and never becomes a new runtime authority.

## Robot and credential boundaries

Robot addresses, interfaces, devices, and calibration come from RobotConfig and
machine configuration. Product source does not embed them.

Thunder remote Brainstem access requires its deployed mTLS files and explicit
source-IP allowlist. Go2 uses the declared network interface. Never store
credentials in `docs/`, Product YAML, or release defaults.

The optional go2rtc service is a machine-level media sidecar outside
ProductControl and readiness. See [Architecture](./architecture.md) for the WHEP
and JPEG fallback contract.

## Go2 connection and debug records

记录核对日期：2026-09-10。以下区分当前配置、历史调试记录和待验证结果，
不代表设备现在在线，也不构成本轮实机运动授权。

连接拓扑、地址、SSH 直连／跳板命令和只读网络核对统一维护在
[Go2 README：连接 NX 与网络核对](../config/robots/unitree/go2/README.md#连接-nx-与网络核对)。
下面保留运动链路与历史调试结论，当前 Product 状态按前面的运维流程核对。

运动链路：

```text
操作页面 / 导航目标
  -> Gateway
  -> Navd：规划、最终速度仲裁
  -> lingtu-driver
  -> Unitree SDK2 SportClient
  -> Go2
```

当前 [Go2 Driver](../src/drivers/real/motion/robots/unitree/go2/go2.cpp)
使用 `ChannelFactory::Init(0, network_interface)`，由所选网卡进行 DDS 发现；
运动调用 `SportClient::Move(vx, vy, wz)`，零速调用 `StopMove()`。
RobotConfig 的控制频率为 `50 Hz`，`auto_enable`、`auto_standup` 均为 `false`。
不要绕过 Navd 和 Driver，直接运行 SDK 运动示例来检查网络。

### What the earlier Go2 debugging established

历史来源：[go2 任务](codex://threads/01a0149a-ca4f-7a90-a1b1-60571f7eaad4)。
此链接需要能访问原 Codex 任务；以下摘要便于离线阅读。

| 记录 | 已知事实 | 不应据此宣称 |
| --- | --- | --- |
| NX 接入 | 曾在 NX 上编译、部署并调试 Go2；Sunrise 仅作为临时开发桥 | 当前候选版本已经部署到 NX 或通过实机验收 |
| MID-360 / SLAM | 当前配置保留 2026-08-23 无运动证据：`TRACKING`、零丢包/零速度、地面倾角约 `0.65°`、RMS `4.3 mm` | 所有传感器已标定；相机仍为 `unverified`；更换支架后无需复核 |
| 按 W 不动 | 历史采样确认非零遥控意图到达 Navd，但出现 `obstacle_stop`、`obstacle_distance_m=0` 和最终输出归零 | 所有“不动”都由键盘、网络或同一个安全条件引起 |
| 近身硬栅格 | 曾记录机器人周围 1 m 内有 31 个硬栅格，格宽 0.20 m；调查涉及自体点、自滤接线和足迹参数 | 这是 31 个独立物体，或已经证明所有占据均为自体误检 |
| 验收状态 | 旧记录包含连接、调试和局部修复证据 | 最新 SCAN 长程导航和实机运动已经整体通过 |

旧任务中的安全参数、直接重启服务和临时构建路径属于历史调试过程，
不作为当前操作步骤。当前启停、地图切换和回滚统一走 ProductControl。

## MuJoCo SCAN clock debugging status

以下记录对应本地 `codex/scan-nav-integration` 分支截至 2026-09-05 的修复，
不是对主分支、已安装版本或当前 Viewer 的部署确认。

| 提交 | 修改内容 | 验证边界 |
| --- | --- | --- |
| `b37577cd` | `REFERENCE_PATH` 重规划从本次测量位置、速度重新初始化，不继续沿用旧 spline 的预定起点 | 这是明确记录的 LingTu 行为差异，不声称与官方该分支输出等价 |
| `755460e2` | 区分单调时钟与外部物理时钟；Task、PlannerManager 不再把排队/计算的电脑耗时加到仿真轨迹时间上 | 已有 Task 和 Follower 定向回归；整体 Native 回归为 117/118，CMU `RollsPastBoxWithPhysicalClearance` 仍未通过 |
| `cc6194d8` | 正式 MuJoCo feeder 将 `engine.sim_time` 经现有传感器记录管道、DDS `/sim/clock` 送到 Navd | 已有记录解析、发布进程、DDS、输入门和 RunPlan 定向验证；不等于长程动态通过 |

确定修复的错误：物理仿真变慢时，机器人世界尚未前进相同时间，
旧轨迹时间却按电脑时间推进，造成参考目标抢跑。现在 SCAN 和 Follower
使用同一物理时间；相同时间样本不会因等待或重复调用而使执行进度前进。
上一轮测试覆盖了物理时间保持 `1.0 s`，随后仅随新样本推进到 `1.05 s`。

Driver / 传感器新鲜度、最终控制超时及 A* 的 `0.2 s` 计算截止仍使用
单调时钟。物理时钟缺失或过期会暂停规划运动；物理时间回退要求重新启动
Product，不能继续执行旧世界的 spline。实机仍使用单调执行时间，不依赖
`/sim/clock`。这项修复不会让渲染或物理计算本身变快。

绿色预览显示的是已提交 spline，不是每个渲染帧从机器人重新画出的一条线。
应分别核对“轨迹起点与规划时测量位置”和“当前参考点与当前实际位置”；
不能通过移动显示线掩盖跟踪误差。转弯振荡和明显落后是否消失，仍需动态验证。

下一次复验按以下顺序记录同一条时间线，不降低验收阈值：

1. 确认实际运行的 RunPlan、二进制版本、真值定位和仿真时钟选择。
2. 短程直道、转弯、约半速物理仿真：对齐物理时间、规划起点、trajectory ID、
   执行时间、参考点、实际位姿、最终指令、实际速度和停止原因。
3. 短程通过后运行同源工业园长程：起点 XY `(3,4)`、目标 XY `(56,32)`，
   实际行走至少 50 m、`REACHED`、终点误差不超过 0.5 m、无实体碰撞、终态零速确认。

截至本次记录，新时钟版本尚未完成上述带窗口长程复验，也没有由此产生的实机证据。
详细实现说明随这三个提交保存在 SCAN 的 `upstream/UPSTREAM.md` 和
`sim/scripts/README.md`；验证层级见 [Testing](./testing.md)。

## Motion boundary

Field motion is a separately approved, supervised action after no-motion gates
pass. Gateway, MCP, Web, and semantic code submit intent; native navigation
retains final arbitration and `lingtu-driver` remains the only hardware writer.

An API or navigation ACK does not prove actuator motion. Stop evidence includes
terminal zero output and driver acknowledgement. Follow
[Testing](./testing.md) before any motion-capable procedure.

## 动态避障的运动证据采集

默认 SCAN 使用测量占据图和独立的短时运动预测。状态快照的
`last_local.dynamic_avoidance` 区分等待、寻找绕行、绕行、观测过期和超时；
`prediction_count` 是当前参与查询的预测体数量，不是识别到的人数。

在另一个终端启动只读采集，再通过正常控制页面进行已经确认场地条件的测试：

```powershell
python tools/diagnostics/navigation_motion_evidence.py --platform go2-nx --url http://127.0.0.1:15052 --seconds 30 --output build/dynamic-avoidance-run-01
```

输出目录需使用新的名字，原始记录不会被覆盖。`samples.jsonl` 保留完整状态；
`summary.json` 分开统计最终指令、同一输出序号的驱动确认及新鲜里程计运动。
此工具只发送 GET，不申请控制权、不发送目标。驱动确认不等于机身已运动；
安静的单个里程计样本也不等于停车距离达标。横穿、迎面、堵路、观测断流的验收仍需
结合场景录像、距离和连续停车证据；Go2/NX、S100P、原生仿真分别保存结果。

## 导航控制周期与后台点云处理

`navd` 的主线程仍按 RunPlan 配置的周期运行，使用单调时钟和 deadline sleep。
动态障碍更新、跟踪与障碍快照由 C++ `MotionWorker` 独立线程计算；主线程负责
接收完整结果、输入新鲜度检查、命令仲裁和最终停车检查。

后台最多保留一帧待处理点云；新帧替换尚未开始的旧帧，避免处理队列越来越落后。
正在处理的一帧会完成，但地图/坐标纪元重置或地图清理后，其结果不能重新进入规划。
只有完成的结果才推进 `cloud_generation`。结果沿用原始接收时间，后台延迟会消耗
新鲜度预算，不能以“刚处理完”为由恢复运动许可。

查看耗时时注意：`timing_ms.motion_update_last` 和 `obstacle_snapshot_last` 是最近
完成的后台任务耗时；`control_loop_health.work_ms` 才是主控制线程每轮工作耗时。
后台计算与主循环同时进行，不应将两者相加当作控制周期。健康统计复用缓冲区和
同一样本的结果；每个新样本仍立即更新健康判定，阈值保持不变。

MuJoCo 组件验收可通过 manifest 的 `navigation_runtime.tick_hz` 指定被测频率，
默认保留原组件基线 20 Hz。测试正式导航的 100 Hz 时必须检查报告中的实际
`period_ms = 10`，20 Hz 通过不能替代 100 Hz 或实机时序验收。
