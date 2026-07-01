# Real Hardware Drivers

This package holds physical robot and sensor drivers. Normal business modules
should consume their Module ports, not ROS topics or vendor SDK APIs directly.

## `thunder/` - Thunder Robot

- `connection.py` - gRPC channel lifecycle to the Brainstem control service.
- `han_dog_module.py` - quadruped motion driver Module.

## `camera/` - Orbbec RGB-D Camera

- `native/capture_process.cpp` - no-ROS Orbbec SDK capture process.
- `native_camera_module.py` - converts native C++ records into LingTu
  `color_image`, `depth_image`, and `camera_info` streams.
- `OrbbecSDK_ROS2/` - local Orbbec SDK/ROS2 source used by the native build
  and kept beside the camera driver instead of under `third_party/`.

The ROS2 `orbbec_camera` launch remains a compatibility path. Product dataflow
should prefer the native module when `build/orbbec_native/orbbec_capture` is
available. The C++ process uses stdout as a local IPC boundary; LingTu dataflow
starts at the Python Module ports.

## `lidar/` - Livox MID-360 LiDAR

- `lidar.py` - configuration and scan-pattern utilities.
- `lidar_module.py` - LiDAR Module.
- `_dds.py` - DDS publishing helper for cross-process consumers.
- `Livox-SDK2/` - official Livox SDK2 source.
- `livox_ros_driver2/` - official ROS2 driver compatibility package.

## `gnss/` - GNSS ROS2 Compatibility

- `wtrtk980_ros2_reader/` - GNSS reader package kept with real hardware
  sources.
