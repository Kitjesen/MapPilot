# Real Hardware Drivers

This package holds physical robot and sensor drivers. Normal business modules
should consume their Module ports or native typed contracts, not vendor SDK APIs
directly.

## `motion/` - Native Robot Motion

- The common C++ driver reads typed DDS `rt/nav/cmd_vel`, enforces
  frame/finite/watchdog/sequence and single-writer rules, and owns status plus
  fail-closed shutdown behavior.
- `motion/robots/unitree/go2/` translates the common contract to Unitree SDK2 Sport API.
- `motion/robots/doso/` translates it to the checked Brainstem Client SDK.
- `RobotConfig.driver.backend` selects one adapter at build and runtime; Products
  do not encode robot vendor identity. See `motion/README.md`.
- Python Host code has no physical motion client. Real motion is owned by the
  Product's native `lingtu_driver` process under `lt-driver.service`.

## `camera/` - Orbbec RGB-D Camera

- `module.py` - `OrbbecNativeCameraModule` (also aliased as `CameraModule`):
  starts/monitors the native C++ capture service and republishes validated SHM
  frames into `color_image`, `depth_image`, `camera_info`, and `alive` ports.
  Registered as `("camera", "orbbec")`.
- `dds_module.py` - `DdsCameraModule`, registered as `("camera", "dds")`,
  reads the native SHM/DDS path. Full image payloads use POSIX SHM by default;
  typed DDS image publishing is opt-in diagnostics, while CameraInfo can
  remain on DDS.
- `native/capture_process.cpp` - native Orbbec SDK capture process.
- `native/camera.cpp`, `native/camera_dds.cpp` - C++ camera and DDS
  publishing sources; `native/shm_frame_ring.hpp` defines the binary SHM ring.
- `native/sdk.cpp` / `sdk.hpp` - C++ SDK wrapper.
- `impl/orbbec/camera.cpp` / `camera.hpp` - Orbbec SDK C++ implementation.
- `build/deps/orbbec-sdk/` - ignored Orbbec SDK package fetched by
  `scripts/build/fetch_orbbec_sdk.sh`.

The camera runtime uses the native capture and SHM/DDS path. Product dataflow
selects the canonical `camera` role with the `orbbec` or `dds` backend.

## `lidar/` - Livox MID-360 LiDAR

- `module.py` - `LidarModule`: canonical LiDAR Module registered as
  `("lidar", "mid360")`. Delegates to a `LidarSource` protocol. Ports:
  `scan`, `raw_scan`, `imu`, `alive`.
- `native/sdk.py` - `LidarSource` protocol and factory used by `LidarModule`.
- `native/model.py` - source lifecycle and health status model.
- `sdk2_stream/main.cpp` - C++ Livox SDK2 stream executable (product hot path).
- `impl/livox/sdk2_stream_source.py` - Python-managed SDK2 process source for
  local runs.
- `api/frames.py` - `LivoxPointFrame` and `POINT_DTYPE` re-exports.
- `api/frame_stream.py` - `LidarFrameStream` ring buffer.
- `native/dds_module.hpp` / `dds_module.cpp` - C++ native DDS publisher.
- `deps/livox/Livox-SDK2/` - official Livox SDK2 source.
- Product LiDAR transport is native CycloneDDS through `sdk2_stream/`.

## IMU ownership

The Livox MID-360 native service publishes LiDAR and IMU DDS topics from one
device connection. There is no separate Python IMU Module or second reader.

## `gnss/` - GNSS Native DDS

- `native/module.hpp` / `module.cpp` - C++ GNSS service boundary.
- `native/dds_module.hpp` / `dds_module.cpp` - C++ DDS publisher for GNSS
  topics (`/gnss/fix`, `/gnss/status`, `/gnss/odom`).
- `native/gnss_dds.cpp` - process entrypoint.
- `native/sdk.hpp` - C++ `FixSample`, `StatusSample`, and `FixType` types.
- `impl/wtrtk980/` - WTRTK-980 serial/NMEA backend (`nmea.cpp`, `serial.cpp`).
- The real GNSS Product uses only this native DDS path.

## Teleop ownership

Field teleop does not use a Python driver Module. The Web panel sends typed
physical velocity intent through Gateway to the native navigation endpoint;
the native endpoint owns arbitration and final safety, and `lingtu_driver` is
the only physical motion writer. See
[`docs/04-deployment/operator_drive.md`](../../../docs/04-deployment/operator_drive.md).
