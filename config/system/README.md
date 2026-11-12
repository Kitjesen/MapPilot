# System configuration

This directory declares operating modes under `products/` and their real/sim
implementations under `envs/`. ProductControl resolves them once before startup.

## 从哪里修改配置

先到 `settings/` 按功能找共同配置；某个模式要使用不同的值，再看对应的
`products/<product>.yaml`。机器人尺寸、安装标定和硬件限制由对应机器人配置负责。
产品运行时使用启动时生成的 RunPlan；修改 YAML 后需重新切换产品生成新 RunPlan。
独立运行的原生程序使用编译生成的默认值，修改这些值后还需重新构建对应程序。

| 要修改什么 | 配置位置 |
| --- | --- |
| 模式选择、规划器、是否允许遥控接管 | `products/<product>.yaml` |
| 模型名称、服务地址、密钥环境变量、请求超时 | `models.yaml`，产品通过 `host.llm` 选择 |
| 行走速度、加速度、制动 | `settings/limits.yaml` |
| SCAN 和 CMU 局部规划、动态障碍和碰撞输入 | `settings/local.yaml` |
| 全局规划、地面支撑、目标点吸附 | `settings/global.yaml` |
| 路线跟踪、到达判断、跟踪增益 | `settings/follower.yaml` |
| 遥控速度、输入有效期、辅助规划 | `settings/teleop.yaml` |
| 相机采集、图像传输和视频 | `settings/camera.yaml` |
| 遇阻恢复顺序、超时和速度 | `settings/recovery.yaml` |
| 控制频率、输入有效期、停止确认、速度平滑 | `settings/control.yaml` |
| 雷达和 IMU 采样、Livox SDK 通信 | `settings/lidar.yaml` |
| SLAM 运行、定位、建图和电梯参数 | `settings/slam.yaml` |
| 地图发布、占据、点云过滤与保存 | `settings/maps.yaml` |
| 探索任务、分段和风险阈值 | `settings/explore.yaml` |
| 真实或仿真的进程与接口 | `envs/<env>.yaml` |
| 机器人尺寸、设备标定、驱动限制 | `config/robots/<vendor>/<model>/robot.yaml` |
| 实机 IMU 噪声、雷达到 IMU 的外参 | `config/robots/<vendor>/<model>/sensors/*.yaml` |
| 仿真定位算法的专用参数 | `src/localization/ltlio/config/sim_mid360*.yaml` |
| 相机设备、标定和明确指定的采集参数 | 选定机器人的 RobotConfig；`config/devices.yaml` 用于兼容驱动 |
| 设备侧视频编码器和可选视频服务 | `config/video.env.example`、`config/go2rtc.yaml` |
| DDS 消息类型、话题名称和 QoS | `src/message/topics/*.yaml` |
| 语义类别和评分 | `config/semantics/` |

A Product's `includes` lists shared files from `settings/` in load order.
Later files override earlier keys within `native_nav` or `parameters`; inline
Product values then override those shared values. The source groups `follower`,
`teleop` and `global` expand to existing flat fields before each merge, so a
follower file can add tracking tolerances without replacing the limits file's
speed limits. Other nested mappings, such as `recovery`, are replaced as a whole.
Map, camera and hardware LiDAR mappings are compiled into the same RunPlan process
environment, along with exploration parameters and SLAM runtime cadence.
Shared Product settings files cannot include other files. SLAM retains its
native configuration-file contract. Each installation or simulation profile
references the common `slam` mapping and applies its overrides in the native loader.

```yaml
includes: [control, lidar, slam, maps, limits, global, local, follower, teleop, recovery]
variants:
  standard: {}
  camera:
    includes: [camera]
    processes_add: [camera, video]
    optional_processes_add: [video]
    topics_add: [/camera/color/image_raw, /camera/depth/image_raw, /camera/color/camera_info]
    capabilities_add: [camera_preview]
```

Products select functions and keep intentional mode-specific overrides. Camera
variants explicitly select preview processes and interfaces. `camera.yaml` owns
the acquisition/transport defaults for every camera role, including perception
without preview; explicit RobotConfig capture overrides take precedence. Physical
camera selection and calibration remain with the robot. The field launcher does
not read `devices.yaml`; that file remains a compatibility-driver input.
Capture dimensions control the stream, while RobotConfig intrinsic dimensions
describe the calibration. The frame name comes from `runtime.tf.frames`, and
status-file paths come from the real service catalogue used by readiness checks.
The video process uses the same color SHM as capture, and slot capacity derives
from the effective color/depth dimensions.
Service diagnostics and Gateway metadata report SHM names from the current
RunPlan environment rather than a fixed catalogue copy.

`control.yaml` declares command I/O and loop settings; `limits.yaml` contains motion bounds.
It does not grant control or choose a Product. Teleop request limits and recovery
action speeds stay with those behaviors, while physical driver limits stay with
the robot. An omitted Product-specific yaw limit retains its existing default;
it is not replaced by another mode's value.

The unused `input_recovery_frames` setting, environment variable and CLI option
were removed with their simulation callers. Fresh valid inputs resume motion
without a frame-count delay; the obsolete `recovering` and `fresh_frames`
status fields have also been removed.

ProductControl writes resolved field settings to `session.env`. The LiDAR
compiler builds `LINGTU_LIVOX_CONFIG_JSON` from the SDK settings and the selected
RobotConfig. The field launcher writes that snapshot to the SDK configuration
file. LiDAR/IMU rates come from `lidar.yaml`, frame names come from RuntimeFrames,
and IP addresses, NIC and extrinsics belong to RobotConfig. `lidar.publish_freq`
has been removed. An optional robot `lidar.frame_id` describes a calibration
alias, while simulated rates and frames come from SensorPlan.
The field scan rate is `lidar.scan_hz`; the checked Go2 and Thunder mapping,
navigation and assisted-teleop plans currently resolve it to 10 Hz. SLAM loop
and map publication rates have separate purposes.
The explicit `LINGTU_LIVOX_CONFIG` file override remains available for SDK
diagnostics and replaces the generated JSON; the field unit does not set it.
Standalone Host capture uses its explicit RobotConfig and the same YAML defaults,
not an inherited Product SDK snapshot.

Mapd derives local update ranges from half the rolling occupancy window. SCAN
inflation derives from the resolved robot collision dimensions and clearances.
The generic inflation parameters apply to the other mapd paths. Publication,
occupancy and save parameters are passed to mapd in both Envs. The unused
carving bounds and independently configurable local ranges have been removed.
SCAN receives that same resolved occupancy resolution at runtime, including
after YAML changes without rebuilding navd. Its map-resolution consistency
check remains in place.
SLAM-backed map saving uses the fixed `map` coordinate contract; the misleading
`maps.save.frame` tuning field has been removed. Historical metadata and explicit
offline artifact frame metadata retain their existing meanings.

The SLAM runtime section supplies the common tick and snapshot cadence. Its
`sim` subsection disables periodic status logging and tracking against the
saved map on a timer. These runtime choices are separate from estimator profile
overrides such as sensor calibration, buffering and the simulation pose prior.
The scheduler supports `tick_hz >= 1` and tracking periods of zero (disabled)
or at least one second. Configuration outside those ranges is rejected at the
entry point rather than silently raised by the scheduling loop.

Product launch scripts read the resolved settings. Native default values for
these component parameters are generated from the same YAML files by
`scripts/build/settings.py`; they are not independently maintained in C++.
`scripts/build/cmake/settings.cmake` registers the YAML as build dependencies,
so an ordinary incremental native build regenerates `lingtu/settings.hpp` after
a source edit. Do not edit or copy values into the generated header.
Direct CLI/library users get the common component defaults too. A caller that
needs a different experiment setting must pass it explicitly.
The generated session environment overrides same-named tuning values previously
placed in `/etc/lingtu/maps.env` or `video.env`; move those values to the owning
component settings (or the existing explicit RobotConfig capture overrides).
Deployment-only endpoints, device selection and encoder availability keep their
existing target configuration. Release the settings with the updated binaries:
older SLAM binaries do not load the new common-profile reference, and older
mapd binaries do not read all of the newly wired environment parameters.
Products keep their actual interface differences: map
activation, saved/cumulative cloud outputs, goal status and traversability are
not enabled merely because a related process exists. Inspection-only interfaces
stay in `products/inspection.yaml`; no empty per-feature files are introduced.

Both exploration variants explicitly include `limits`, `local` and `explore`.
Variant `parameters` replace the entire base section, so splitting these values
between base and variant would drop local or segment parameters. The loader's
existing replacement rule is unchanged.

Shared files may also declare `topics`. These are combined with the Product's
topic list in file order without duplicates; message types, DDS names and QoS
remain defined once in `src/message/topics/`. Variant whole-field replacement
still applies: a variant with its own navigation topic list replaces the base
list; use the existing `topics_add` for an incremental variant.

`global.yaml` lists all 23 platform-independent OctoPlanner algorithm settings and the
selected planner, plus `/nav/global_path` and the existing preview request/result
topics. `nav`, `inspection`, `tracking`, and both `explore` variants load it.
The `nav` Product explicitly keeps its existing 0.15 m terminal tolerances;
other Products retain the shared 0.5 / 0.35 / 0.75 m values. Robot radius,
support height/tolerance, slope limit and upper/lower body clearance are documented there
but continue to come from RobotConfig or the selected simulation geometry.
Writing one fixed Go2 value into the shared file would override Thunder too.
The planner reads its saved OctoMap through the map path; live local collision
topics and final velocity outputs are not owned by the global planner.

Common tuning stays short within its owner:

```yaml
native_nav:
  follower:
    max_speed_mps: 1.4142135623730951
    max_accel_mps2: 0.5
    lookahead_m: 0.35
    goal_tolerance_m: 0.15
  teleop:
    max_speed_mps: 1.0
    horizon_m: 3.5
  global:
    terminal_tolerance_m: 0.15
```

These are source names, not new runtime parameters. Units remain in the keys.
Numeric bounds are still checked by the existing native configuration compiler.
`parameters.scan_planner` and `parameters.scan_follower` may also be written as
groups. They expand to the existing dotted runtime parameter names before merging.
Local planning speed/acceleration continue to derive from the motion budget;
the source files do not independently fix those values.

File names use a short responsibility noun; fields use their enclosing group
instead of repeating a module prefix. Keep unit suffixes such as `_m`, `_mps`
and `_rad_s`. English comments explain units not present in legacy keys,
override rules and non-obvious behavior. Public topics, capability identifiers
and runtime fields retain their contract names.

Two teleop settings have different owners and must not be confused:

| YAML key under `native_nav` | Meaning | Native environment field |
| --- | --- | --- |
| `teleop.takeover` | Allow an autonomy Product to expose operator takeover. This does not itself grant a client control. | `LINGTU_NAV_ALLOW_TELEOP_TAKEOVER` |
| `teleop.planner` | Use local planning for assisted teleop translation. Explicit manual mode and pure rotation follow their existing control paths. | `LINGTU_TELEOP_LOCAL_PLANNER` |

```yaml
native_nav:
  teleop:
    takeover: true
    planner: true
```

`src/lingtu/assembly/graph/settings.py` translates these source keys to `allow_teleop_takeover` and
`teleop_local_planner` once. RunPlan, native fields, environment variables and
status retain their existing names. The environment mapping is in
`src/lingtu/assembly/config/native_navigation.py`; native nav reads it in
`src/nav/cpp/endpoint/nav/app/config/parse.cpp`. These are control-mode settings,
not SCAN algorithm parameters.

## Runtime Control Model

The control plane has Product and Env declarations, one resolved plan, one
ProductControl owner, and one Host-local graph:

| Object | Owns | Must not own |
| --- | --- | --- |
| Product | One immutable operating mode: required capabilities, topics, Host Modules, and logical process roles. | Env selection, runtime side effects, or domain algorithms. |
| Env | The outer `real` or `sim` implementation of hardware, process roles, and transports. | Product policy or domain algorithms. |
| RunPlan | Internal immutable execution snapshot of one Product in one Env; it is the Host/systemd execution input, not a public lifecycle object. | Re-resolving Product or Env policy at execution time. |
| Endpoint | A genuine communication access contract nested under an Env implementation. | Selecting a Product, Env, or simulation backend. |
| ProductControl | Resolves and publishes the private execution snapshot, stages the transient session, applies systemd processes, checks readiness, and owns Product operations. | Domain algorithms or Module wiring. |

The machine path is:

```text
products/<product>.yaml + envs/<real-or-sim>.yaml
  -> ProductControl resolves one private execution snapshot
  -> internal snapshot (identity, launch, host, checks)
  -> ProductControl publishes that artifact and session.env
  -> ProductControl.switch() -> internal SystemdRunner.apply(plan)
  -> the Env implementation's declared process owner
```

`scripts/lingtu` and Gateway both invoke this same ProductControl operation.
Neither one recompiles the Product or owns process ordering.

Each Product uses `lingtu.runtime_graph.product.v3`. Its top-level `topics` and
`capabilities` are the complete environment-independent runtime boundary;
`host.capabilities` selects the in-process Host Modules. `conversation` adds the
conversational Agent independently of `semantic` perception and semantic planning;
standard `map`/`nav` declare it and resolve their `llm` connection from `models.yaml`.
Semantic variants reuse the same Agent factory and keep exactly one instance.
Topic definitions
and message types remain canonical under `src/message/topics/*.yaml`; a Product only lists
which canonical topics it needs. Product navigation settings stay under
`native_nav`, and rolling launch parameters stay under `parameters`. Shared
values are declared by `includes` and expanded from the
YAML files above. There is no separate Python Product-default or Product
contract registry beside the YAML.

Products declare `processes` using stable logical roles such as `lidar`,
`slam`, `nav`, and `host`. The ProductControl-managed `real` Env maps those roles to
deployment `target` values. Topic names validate the selected Env
implementation; they are never used to infer which process should start.

Only an Env implementation with `process_control: systemd` exposes Product
processes to ProductControl. The `sim` Env uses the MuJoCo backend selected by
`env_config.backend`.
The simulation runner owns
their child-process setup, teardown, evidence capture, and timeouts as one
transaction. The control plane must not fabricate partial Product processes
for them.

The `real` Env is fail closed for required processes: every required target must be
installed, become active within its declared timeout, and pass its role-specific
readiness check. The internal execution snapshot also carries all mode-owned
and conflicting targets, so switching Products removes stale process ownership
before startup. ProductControl resolves topics, capabilities, code defaults,
Env parameters, Product parameters, and the current `--set` values into one
private execution snapshot (currently serialized as RunPlan v11). Runners
execute that final result and do not resolve Product policy a second time.

The graph loader preserves raw variant declarations. Assembly selects the
variant once and passes its resolved values explicitly to Host construction.
Variants inherit top-level Product fields; an override replaces that whole
field, without recursively merging nested mappings. `topics_add`,
`capabilities_add`, `processes_add` and `optional_processes_add` append only
the variant's extra declarations, preserving
order and omitting duplicates; they are expanded before validation and are
not stored in RunPlan. Declare only differences:
`standard: {}` retains the base Product, and camera variants inherit its Host
configuration when they do not change it.
Public selection uses `python -m lingtu.control switch PRODUCT --variant NAME`.
For `nav`, `standard` is the default; `camera` adds native camera and video processes
and the Host preview relay while retaining the same navigation configuration.
The real-only `semantic` variant keeps that native navigation and camera process
set, then adds RGB-D perception plus semantic and Agent planning in the Host.
It selects the `yolo_world` detector and the shared `kimi` model entry.
Simulation supports only `standard`.
For `tracking`, `standard` keeps the existing RGB-D perception and Follow graph;
`camera` adds video preview over that same camera source. It does not add a
second camera or change the selected local planner, identity checks or calibration.
The Product declares `optional_processes: [video]`; only the preview may degrade
without blocking activation. RunPlan stores this as `required: false`. The real
runner reports that failure and continues core startup; stopping and rollback
still own the video process. Missing mappings and invalid core-role declarations
remain errors. The sim map/nav/tracking backends support only `standard`.
The release installer preserves the selected variant when restoring a Product.

### Host models

`nav/semantic` and `tracking` select `kimi` in [models.yaml](models.yaml).
This entry sets `kimi-k3`, `https://api.moonshot.cn/v1`, `MOONSHOT_API_KEY`
and a 30-second timeout. Tracking uses the vision-capable client for person
selection. Inspection selects `qwen`, which sets `qwen-turbo`,
`DASHSCOPE_API_KEY`, the SDK's default endpoint and a 10-second timeout.
Calling Qwen requires the DashScope SDK, which the current LLM extra does not
install.

Assembly resolves the selected backend once. Explicit Product fields override
the shared values, Env Host fields override Product fields, and permitted
caller overrides have the highest priority. RunPlan stores the resolved
`llm` and `llm_*` values; Host reads that snapshot. Set the credential value in
the Host environment, for example in `/etc/lingtu/gateway.env` on the robot.
YAML and RunPlan store the variable name only.
The internal snapshot stores structured navigation settings under `launch.native_nav` and
other process environment values under `launch.process_environment`; the final
launch environment is derived from those values and `launch.parameters`.

## Naming

Names reflect scope instead of repeating implementation details:

```text
logical process id: nav
executable:         navd
systemd unit:       lt-nav.service
Env:                real
```

The executable is short because its package and directory already establish
the product and domain. Systemd units use the short `lt-<function>.service`
namespace because units share one host-wide namespace. Transport details such
as DDS belong in the implementation and documentation, not in the process
identity. ProductControl logic still depends only on the logical `nav` id.

Runtime Graph plus `runtime.route_contract` describes the Host and native DDS
boundaries without duplicating navigation inside the Host:

```text
Host wires define Module ports:
  host.bus.navigation_goal_status -> nav.goals.navigation_goal_status

RouteContract defines native cross-process transport:
  /slam/odometry -> rt/slam/odometry -> lingtu.dds.Odometry
```

Do not move DDS imports into normal Modules. Modules keep typed `In`/`Out`
ports; adapters and endpoint services translate between those ports and DDS.
Python code gets the generated topic view from
`message.topics.TOPICS`; see
`docs/architecture.md` for the static guard and allowed
boundary exceptions.

Runtime resolver now exposes the selected contract as `route_contract`.
For the `real` Env this should read:

```text
env=real
endpoint_contract=field_dds_v1
route_contract=robot
```

`route_contract` is topic ownership and transport intent. It does not change
Host delivery: internal Module wires remain process-local, while native
endpoints own DDS communication.

## Boundaries

- Real and real-equivalent simulation data planes must use native DDS / C++
  runtime contracts.
- The physical field command sink is `endpoint_only + driver`: native nav
  publishes the canonical command, and `lt-driver.service` is the sole hardware
  bridge to the RobotConfig-selected Go2 or Thunder adapter.
- Python Module wiring remains for mock runs, CI harnesses, Gateway/status,
  semantic modules, visualization, and compatibility adapters.
- Runtime Graph YAML declares what a Product or Env implementation must expose before a
  report can claim real-equivalent behavior.

## Files

The loader reads `products/*.yaml`, their referenced `settings/*.yaml` and
`envs/*.yaml`. Host configuration resolution reads the selected connection
from `models.yaml` for Products with semantic Modules.
Qualification manifests belong under `config/acceptance/`, and the canonical
topic catalogue belongs under `src/message/topics/`; neither is a Runtime
Graph subdirectory. Empty `acceptance/` or `topics/` folders left by older
layouts have no runtime meaning and are not tracked by Git.

- `src/message/topics/*.yaml` (outside this directory): one file per domain. Each entry owns the logical name, DDS
  wire name, exact `lingtu.dds.*` message type, QoS profile, and optional
  producer/consumer bindings. `message/topics.py` and
  `message/generated/topics.hpp` are generated views, never edited by hand.
- `products/*.yaml`: product modes such as `map`, `nav`, `explore`, and
  `teleop_avoid`.
- `envs/real.yaml`: the shared physical implementation; the selected robot model owns its RobotConfig,
  ProductControl process-role ownership, and genuine endpoint contract.
- `envs/sim.yaml`: simulation backends, per-robot session bindings, process
  owners, Host configuration, and the genuine endpoint contract.

Local qualification manifests live under `config/acceptance/`, outside this
tree. They exercise a resolved runtime but do not participate in Product/Env
resolution.

`operator_switchable` controls whether a Product appears in the normal
operator/Gateway mode catalog; it does not decide whether the compiler can
produce a RunPlan. Local-planner algorithms are not Products: `nav` remains the
single saved-map navigation Product and defaults to SCAN; CMU is selected with
`local_planner="cmu"` when required.

Each Env implementation declares the algorithms it has qualified in
`local_planners`. Resolution fails before launch when a Product selects an
unqualified backend. The physical `real` Env and MuJoCo simulation both expose
CMU and SCAN.

## SCAN runtime parameters

SCAN tuning uses the existing Product `parameters` mapping and session
`parameter_overrides` / `--set` interface. Values come from the component YAML.
The remaining `standalone` entries describe explicit mode choices, such as a
bare endpoint not enabling recovery; they do not define another motion budget.
Python parameter specifications declare names, types, bounds and environment
keys. The resolved values are saved in
`RunPlan.launch.parameters` and rendered into the native process environment.
There is no separate `native_nav.scan_follower` or `native_nav.scan_planner`
configuration block.

```yaml
parameters:
  scan_planner.control_point_spacing_m: 0.20
  scan_planner.replan_distance_m: 1.0
  scan_planner.no_replan_distance_m: 0.10
  scan_planner.planning_horizon_m: 3.5
  scan_planner.collision_weight: 1.0
  scan_follower.time_forward_s: 0.8
  scan_follower.finish_distance_m: 0.15
  local_collision.max_age_s: 0.50
```

For SCAN Products, planner velocity/acceleration defaults derive from
`native_nav.path_follower_max_speed_mps` / `path_follower_max_accel_mps2`.
Follower forward speed and yaw-rate defaults derive from the corresponding
Product motion limits. Explicit parameter values override these defaults:
Env overrides, then Product parameters, then session overrides. Planner
constraints, follower axis limits, and final command safety limits retain
their separate roles; an explicit SCAN override does not raise the final
Product command limit. CMU planning/tracking settings are grouped under `cmu`
in `local.yaml` and `follower.yaml` and passed through the existing native
environment input. CMU tracking does not replace those values with fixed numbers.

`local_collision.max_age_s` is Endpoint input policy, shared with recovery.
Robot geometry still comes from RobotConfig. Map resolution and upstream FSM
cadences remain part of the SCAN port contract. C++ parameter-struct initializers
reference the generated YAML constants; Product runs receive resolved settings
from the saved RunPlan. Plans created before these fields were recorded must be
recompiled before use with this runtime.

Editing YAML does not mutate a running Product. Start a new Product run to apply
its runtime settings. Rebuild native code to update standalone generated defaults
or settings consumed directly by a native library. Explicit CLI/session/platform
overrides retain their documented precedence over the common defaults.

Regeneration uses the normal `ProductControl.switch` entry: it compiles and
publishes a new RunPlan from the current Product, Env, and session parameters.
For example, a saved-map-free MuJoCo SCAN run starts with:

```bash
python -m lingtu.control switch teleop_avoid --robot doso/thunder_v4 --env sim --backend mujoco --local-planner scan
```

Use `--set scan_planner.planning_horizon_m=4.0` for a session override. Do not
edit the old RunPlan JSON; retain it as the snapshot of its original run.

## Real-Equivalent Rule

The `real` Env and the `sim` Env's `mujoco` backend must share the native
contract topics that cross the process boundary:

```text
/tf
/tf_static
/lidar/raw_frame
/imu/raw
/slam/odometry
/slam/state_at_scan
/slam/registered_cloud
/slam/map_cloud
/slam/saved_map_cloud
/slam/localization_health
/slam/localization_quality
/nav/command/request
/nav/command/ack
/nav/inspection/task/request
/nav/inspection/task/ack
/nav/inspection/status
/nav/inspection/task/event
/nav/inspection/evidence/request
/nav/inspection/evidence/result
/nav/traversability
/nav/global_path
/nav/local_path
/nav/way_point
/nav/cmd_vel
```

`/nav/semantic/instruction` remains an in-process Gateway -> SemanticPlanner
channel. Only its resolved goal enters native typed `/nav/command/request`;
`/nav/command/ack` confirms business acceptance or rejection.

Inspection route lifecycle has its own typed interface: the persistent C++
client publishes task-addressed start/pause/resume/cancel requests on
`/nav/inspection/task/request`, waits for the matching business ACK on
`/nav/inspection/task/ack`, and the native endpoint publishes current
route/point progress on `/nav/inspection/status` plus ordered task facts on
`/nav/inspection/task/event`. Direct task goals remain on
`/nav/command/request`; operator velocity owns the separate
`/nav/operator_motion/control`, `/nav/operator_motion/sample`, and
`/nav/operator_motion/ack` contract.

Inspection evidence has a separate worker bridge:
`/nav/inspection/evidence/request` carries frame/evidence work requests and
`/nav/inspection/evidence/result` carries the analysis result. These topics are
inspection evidence exchange, not motion or route-control topics.

The canonical `mujoco` backend runs the C++ typed-DDS chain. The old Gazebo and
manual ROS2 localPlanner/pathFollower navigation stack is retired and is not a
Product backend. MuJoCo component scripts remain development tools, not Product
backends.

## Validation

Use the Python helper in `src/lingtu/assembly/graph`:

```python
from lingtu.assembly.graph import assert_runtime_graph_valid

assert_runtime_graph_valid()
```

Focused tests live in `tests/runtime/test_runtime_graph_contract.py`.

Route and DDS binding validation:

```python
from runtime.route_contract import assert_route_contract_valid

assert_route_contract_valid(route="robot")
```

Focused route/DDS tests live in `tests/runtime/test_route_contract.py`.
