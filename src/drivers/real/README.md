# Real Hardware Drivers

This package holds physical robot and sensor drivers. Normal business modules
should consume their Module ports, not ROS topics or vendor SDK APIs directly.

## `thunder/` - Thunder Robot

- `native/` - canonical product C++ `driver`: reads typed DDS
  `rt/nav/cmd_vel`, enforces frame/finite/watchdog/sequence rules, acquires
  the `lingtu-driver` Brainstem control lease, and calls the checked
  Brainstem `RobotControl.WalkChecked` RPC. It never enables motors or changes
  pose.
- `han_dog_module.py` - compatibility in-process `ThunderDriver`, registered
  as `("driver", "thunder")` for lite/local Module graphs.
- `connection.py` - **DEPRECATED**: `NovaDogConnection` legacy gRPC bridge.
  Superseded by `ThunderDriver`. Kept only for backward compatibility.
- `blueprints.py` - Legacy blueprint helpers (`nova_dog_basic`, `nova_dog_nav`,
  `nova_dog_semantic`).

## `camera/` - Orbbec RGB-D Camera

- `module.py` - `OrbbecNativeCameraModule` (also aliased as `CameraModule`):
  starts/monitors the native C++ capture service and republishes validated SHM
  frames into `color_image`, `depth_image`, `camera_info`, and `alive` ports.
  Registered as `("camera", "orbbec")`.
- `dds_module.py` - `DdsCameraModule`: compatibility registry name for the
  SHM/DDS reader path. Full image payloads use POSIX SHM by default; typed DDS
  image publishing is opt-in diagnostics/compatibility, while CameraInfo can
  remain on DDS.
- `native_camera_module.py` - compatibility re-export of
  `OrbbecNativeCameraModule` for old import paths.
- `native/capture_process.cpp` - no-ROS Orbbec SDK capture process.
- `native/camera.cpp`, `native/camera_dds.cpp` - C++ camera and DDS
  publishing sources; `native/shm_frame_ring.hpp` defines the binary SHM ring.
- `native/sdk.cpp` / `sdk.hpp` - C++ SDK wrapper.
- `impl/orbbec/camera.cpp` / `camera.hpp` - Orbbec SDK C++ implementation.
- `deps/orbbec/OrbbecSDK/` - preferred no-ROS Orbbec SDK checkout.
- `deps/orbbec/OrbbecSDK_ROS2/` - temporary ROS2 wrapper fallback,
  quarantined below the camera driver.

The ROS2 `orbbec_camera` launch remains a compatibility path only. Product
dataflow should prefer the native Orbbec SDK capture plus SHM rings; do not add
a Python DDS image reader to the main camera path.

## `lidar/` - Livox MID-360 LiDAR

- `module.py` - `LidarModule`: canonical LiDAR Module registered as
  `("lidar", "mid360")`. Delegates to a `LidarSource` protocol. Ports:
  `scan`, `raw_scan`, `imu`, `alive`.
- `native/sdk.py` - `LidarSource` protocol and factory used by `LidarModule`.
- `sdk2_stream/main.cpp` - C++ Livox SDK2 stream executable (product hot path).
- `impl/livox/sdk2_stream_source.py` - Python-managed SDK2 process source for
  local runs.
- `api/frames.py` - `LivoxPointFrame` and `POINT_DTYPE` re-exports.
- `api/frame_stream.py` - `LidarFrameStream` ring buffer.
- `compat/lidar.py` - legacy `Lidar` class (DDS subscriber interface).
- `compat/dds.py` - Livox CustomMsg + Imu DDS IDL types for CycloneDDS.
- `compat/dds_adapter.py` - `LivoxDdsAdapter` for DDS topic subscription.
- `native/dds_module.hpp` / `dds_module.cpp` - C++ native DDS publisher.
- `deps/livox/Livox-SDK2/` - official Livox SDK2 source.
- The former ROS2 Livox compatibility fallback has been removed. Product
  LiDAR transport is native CycloneDDS through `sdk2_stream/`.

## `imu/` - IMU (Livox MID-360 Facade)

- `module.py` - `ImuModule`: documents the field IMU role without opening a
  second hardware reader. The actual IMU stream is carried by the LiDAR source.
  Registered as `("imu", "livox")`.
- `dds_module.py` - native DDS IMU subscriber for cross-process consumers.
- `native/sdk.py` - IMU source interface.
- `impl/livox/` - Livox IMU backend placeholder (data flows through LiDAR).

## `gnss/` - GNSS Native DDS

- `native/module.hpp` / `module.cpp` - C++ GNSS service boundary.
- `native/dds_module.hpp` / `dds_module.cpp` - C++ DDS publisher for GNSS
  topics (`/gnss/fix`, `/gnss/status`, `/gnss/odom`).
- `native/gnss_dds.cpp` - process entrypoint.
- `native/sdk.hpp` - C++ `FixSample`, `StatusSample`, and `FixType` types.
- `impl/wtrtk980/` - WTRTK-980 serial/NMEA backend (`nmea.cpp`, `serial.cpp`).
- ROS2 GNSS readers are legacy compatibility assets only and are not present in
  the real GNSS product tree.

## `teleop_module.py` - Joystick Teleop

- `TeleopModule`: joystick remote control with live camera stream. Scales
  joystick inputs to `Twist`, manages active/idle state (3s auto-release),
  and encodes camera frames for Gateway. Ports: `color_image`, `joy_input`
  (in); `cmd_vel`, `teleop_active` (out).
