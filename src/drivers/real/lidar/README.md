# lidar

`lidar` is the short role for Livox MID-360 data.

## Product path

- `sdk2_stream/` is the product hot path: C++ Livox SDK2 capture plus optional
  CycloneDDS publishing.
- `deps/livox/Livox-SDK2/` is the only vendor SDK kept under `real/lidar`.
- `lingtu-livox-dds.service` should run the C++ stream binary in field builds.

## Folder layout

```text
real/lidar/
  __init__.py              # package exports and compatibility aliases
  module.py                # LidarModule - Blueprint runtime Module
  api/
    frames.py              # LivoxPointFrame, POINT_DTYPE (re-export)
    frame_stream.py        # LidarFrameStream ring buffer
  native/
    sdk.py                 # LidarSource protocol + LidarSourceFactory
    module.hpp             # C++ native service boundary
    dds_module.hpp/.cpp    # C++ native DDS publisher module
  sdk2_stream/
    main.cpp               # C++ Livox SDK2 stream executable
    CMakeLists.txt
  impl/
    livox/
      sdk2_stream_source.py  # Python-managed SDK2 process source
      native_factory.py      # Livox driver process factory
  compat/
    lidar.py               # Legacy Lidar class (DDS subscriber interface)
    dds.py                 # Livox CustomMsg + Imu DDS IDL types
    dds_adapter.py         # LivoxDdsAdapter for DDS topic subscription
    native_factory.py      # Compatibility hook for old factory path
  deps/
    livox/Livox-SDK2/      # Official Livox SDK2 source
```

## Module architecture

`LidarModule` (in `module.py`) is registered as `("lidar", "mid360")` and
`("driver", "lidar_mid360")`. It delegates to a `LidarSource` protocol defined
in `native/sdk.py`. The default source factory creates an `Sdk2Source` from
`impl/livox/sdk2_stream_source.py`, which manages the C++ SDK2 stream process
for local runs.

Output ports: `scan` (PointCloud2), `raw_scan` (Livox frame), `imu` (Imu),
`alive` (bool).

## Python boundary

Python here is not the high-rate device driver.

- `module.py` is the LingTu runtime module shell.
- `native/sdk.py` defines the `LidarSource` protocol used by the module.
- `impl/livox/sdk2_stream_source.py` starts the C++ stream process for managed
  local runs and tests.
- `compat/lidar.py`, `compat/dds.py`, and `compat/dds_adapter.py` are
  compatibility/readback paths. Old imports such as
  `drivers.real.lidar.lidar` and `drivers.real.lidar._dds` are aliases only,
  mapped through `__init__.py` `_SUBMODULE_ALIASES`.

## ROS2 fallback

The ROS2 Livox driver is not part of the real LiDAR product path. It lives under
`src/drivers/adapters/ros2/lidar/livox_driver2/` and is reachable only through
explicit compatibility flags.

## Capture And Replay

The native stream format is `LTU1`: bounded binary cloud, IMU, and odometry
prior records. Hardware capture does not require ROS:

```bash
livox_sdk2_stream MID360_config.json > mid360.ltu
```

Validate a recording without DDS or hardware:

```bash
livox_sdk2_stream --validate-records < mid360.ltu
```

Replay to the product CycloneDDS topics at original speed or accelerated:

```bash
livox_sdk2_stream --stdin-records --dds --replay-rate 1.0 < mid360.ltu
livox_sdk2_stream --stdin-records --dds --replay-rate 5.0 < mid360.ltu
```

When the record producer and DDS runtime use different operating-system clocks
(for example Windows-hosted MuJoCo feeding a WSL C++ runtime), rebase the first
record to the publisher clock while preserving all relative sensor timing:

```bash
livox_sdk2_stream --stdin-records --restamp-stdin-records --dds < sim.ltu
```

Do not use restamping for live MID-360 input; hardware timestamps pass through
the SDK path unchanged.

The parser rejects bad magic, unknown record types, truncated records, payload
shape mismatches, and payloads over 256 MiB before allocation. A replay rate of
`0` disables pacing for throughput tests.
