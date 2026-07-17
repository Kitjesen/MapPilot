# LingTu Driver Structure Rules

This file is the local contract for `src/drivers`. Keep names short and keep
real/sim device surfaces aligned.

## 1. Roles

Allowed runtime roles:

```text
hw
driver
camera
lidar
imu
gnss
```

Meaning:

- `hw`: hardware inventory/status only. It does not own sensor streams.
- `driver`: robot base motion and base feedback only.
- `camera`: image/depth/camera_info/points stream owner.
- `lidar`: raw LiDAR stream owner.
- `imu`: IMU stream owner.
- `gnss`: GNSS fix/status stream owner.

Do not create new long runtime role names such as `CameraBridgeModule`,
`LidarModule`, or `DeviceManager`. Those may exist only as compatibility
aliases while migration is in progress.

## 2. Backend Names

Backend names are short hardware/source names:

```text
orbbec
mid360
livox
mujoco
dds
replay
serial
thunder
endpoint
```

Do not register names like `NativeCameraBridgeAdapter`,
`OrbbecNativeCameraRuntime`, or `MujocoDepthCameraProvider`.

## 3. Folder Shape

`real` and `sim` must keep the same device shape. The only difference is where
data comes from.

```text
src/drivers/{real|sim}/{device}/
  __init__.py
  module.py
  native/
  impl/
    {backend}/
  deps/
    {vendor}/
  tests/
```

Layer meanings:

- `module.py`: LingTu runtime Module, ports, lifecycle, config, and publishing.
- `native`: LingTu-owned interface and protocol. No vendor headers here.
- `impl/{backend}`: one backend implementation.
- `deps/{vendor}`: third-party SDKs, headers, and shared libraries.
- `tests`: contract tests for the device role and backend.

Application code must not import or include files from `deps` directly.

Additional folders used in practice:

- `api/`: shared Python data contracts used by multiple sources (for example
  `real/lidar/api/` for `LivoxPointFrame` and `LidarFrameStream`).
- `compat/`: legacy adapters and readback paths that bridge old import names
  to the current module layout (for example `real/lidar/compat/`).

## 4. Topic Naming

There is one canonical topic form:

```text
/{role}/{name}
```

Examples:

```text
/camera/color/image_raw
/lidar/raw_frame
/imu/raw
/slam/odometry
/driver/odometry
```

DDS wire names are derived, not hand-authored:

```text
dds_topic_name("/driver/odometry", typed=True) -> "rt/driver/odometry"
```

Module port references are not topics:

```text
driver.odometry        # module port reference
/driver/odometry       # canonical runtime topic
rt/driver/odometry     # typed DDS wire topic
```

Rules:

- New code stores canonical runtime topics in `runtime.runtime_interface.TOPICS`.
- New code uses `message.dds.dds_topic_name(..., typed=True)` for DDS names.
- Do not put animal, robot-shape, or product nicknames in topic names.
- `/nav/dog_odometry` is legacy. New code must use `/driver/odometry`.
- `nav` owns navigation state and commands, not base odometry.
- `slam` owns localization odometry: `/slam/odometry`.
- `driver` owns base/proprioceptive odometry: `/driver/odometry`.

## 5. Device Interface

Each device exposes one small interface:

```text
connect(config)
disconnect()
is_connected()
info()
read(timeout_ms) -> Sample
```

`Sample` may contain optional fields:

```text
color
depth
points
imu
intrinsics
odometry
status
timestamp
```

A backend leaves unsupported fields empty. Do not create many unrelated read
methods unless the native API requires it for performance.

## 6. Camera Contract

Camera output ports:

```text
color_image
depth_image
camera_info
points
imu
status
alive
```

Current backends:

```text
real/camera/impl/orbbec     # C++ Orbbec SDK capture (product path)
real/camera/dds_module.py   # DdsCameraModule - native DDS subscriber
sim/camera/impl/mujoco      # MuJoCo simulated camera
```

Compatibility:

- `real/camera/native_camera_module.py` is a compatibility re-export of
  `OrbbecNativeCameraModule` from `module.py`.
- `OrbbecSDK_ROS2` under `deps/orbbec/` is fallback only. The long-term real
  path is pure Orbbec SDK plus native DDS.

Registry names: `camera` role with `orbbec` or `dds` backend.

## 7. LiDAR Contract

LiDAR output ports:

```text
scan
raw_scan
imu
status
alive
```

Current backends:

```text
real/lidar/impl/livox       # Sdk2Source - C++ Livox SDK2 stream process
real/lidar/compat/          # Legacy Python DDS readback (Lidar, LivoxDdsAdapter)
sim/lidar/impl/mujoco       # MuJoCo simulated LiDAR (ray casting)
sim/lidar/mujoco_lidar/     # High-fidelity MuJoCo LiDAR cores (CPU, JAX, Warp, TI)
```

The `LidarModule` in `real/lidar/module.py` is the canonical runtime module.
It delegates to a `LidarSource` protocol defined in `real/lidar/native/sdk.py`.
The default source factory creates an `Sdk2Source` from `impl/livox/`.

The upper SLAM stack must not know whether raw LiDAR came from Livox hardware,
MuJoCo, DDS, or replay.

## 8. IMU Contract

IMU output ports:

```text
imu
status
alive
```

Current backends:

```text
real/imu/module.py          # Livox MID-360 IMU facade (no duplicate publisher)
real/imu/dds_module.py      # Native DDS IMU subscriber
sim/imu/impl/mujoco         # MuJoCo simulated IMU
```

The current field IMU is carried by the Livox MID-360 LiDAR source. The
`real/imu/` module is a facade that documents the IMU role without opening a
second hardware reader. Runtime IMU publishing remains in `drivers.real.lidar`
until the IMU stream is split from the LiDAR module wiring.

## 9. Driver Contract

`driver` means robot base motion, not all hardware.

Allowed driver responsibilities:

```text
cmd_vel
stop_signal
odometry
robot_state
alive
```

Not allowed as primary driver ownership:

```text
camera stream
lidar stream
imu stream
global device management
semantic perception
navigation logic
```

Current driver backends:

```text
real/thunder/native/             # Product C++ DDS -> Brainstem driver
real/thunder/han_dog_module.py   # Compatibility Module driver
real/thunder/connection.py       # DEPRECATED: NovaDogConnection legacy bridge
sim/mujoco/driver.py             # MuJoCo in-process driver
sim/endpoint.py                  # Externally owned simulation streams
```

Temporary compatibility outputs may remain during migration, but new wiring
must prefer device roles.

## 10. Blueprint Rule

Blueprints wire by role, not by backend.

Preferred:

```text
camera.color_image -> perception.color_image
camera.depth_image -> perception.depth_image
camera.camera_info -> perception.camera_info
lidar.raw_scan -> slam.lidar_raw_scan
lidar.imu -> slam.lidar_imu
driver.odometry -> diagnostics.driver_odometry
```

Avoid:

```text
MujocoDriverModule.depth_image -> perception.depth_image
OrbbecNativeCameraModule.depth_image -> perception.depth_image
MujocoDriverModule.raw_scan -> slam.lidar_raw_scan
```

Backend choice belongs in config or a stack factory.

## 11. Service Rule

Product services use short catalog names:

```text
camera
lidar
slam
nav
traversability
explore
gateway
lingtu
```

Systemd owns long-running native services. `lingtu.service` owns the Python
runtime/gateway/orchestration layer. Python modules must not secretly start
product hardware daemons unless a compatibility flag explicitly enables that.

## 12. Test Rule

Every device must have contract tests for:

```text
real and sim folder shape
native has no vendor dependency
module exposes canonical ports
real backend satisfies native interface
sim backend satisfies native interface
blueprint wires by role
control-plane import does not load heavy deps
topic names derive DDS names through dds_topic_name()
```

## 13. Migration Order

Refactor one device at a time:

```text
1. camera
2. lidar
3. imu
4. driver
5. gnss
6. blueprint wiring
7. service deployment
8. alias removal
```

During migration:

- Keep compatibility aliases.
- Keep old ports only while tests and field profiles still need them.
- Add tests before deleting old paths.
- Delete compatibility only after field validation passes.

## 14. Final Target

The target mental model is:

```text
role -> native interface -> backend impl -> deps
```

Examples:

```text
camera -> native -> orbbec -> Orbbec SDK
camera -> native -> mujoco -> MuJoCo engine
lidar -> native -> livox -> Livox SDK2/DDS
lidar -> native -> mujoco -> MuJoCo lidar
driver -> native -> thunder -> robot base service
driver -> native -> mujoco -> MuJoCo control
imu   -> native -> livox -> Livox MID-360 (via lidar source)
gnss  -> native -> wtrtk980 -> serial/NMEA
```

Upper layers consume only LingTu ports and messages. They must not know vendor
names.
