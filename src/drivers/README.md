# Drivers

`src/drivers/` owns robot and sensor ingress/egress. It is the hardware and
simulation boundary, not a planning layer.

## Layout

| Folder | Role |
| --- | --- |
| `real/` | Real robot and sensor backends: Thunder driver, Orbbec camera, Livox LiDAR, IMU facade, GNSS, and teleop. |
| `sim/` | Simulation drivers (MuJoCo in-process driver, endpoint adapter, stub) and simulated sensor backends. |
| `adapters/ros2/` | Declared ROS 2 compat boundary (for example `livox_driver.py`, `wtrtk980_reader`). ROS 2 imports outside this boundary are not allowed. |

Root files stay minimal. Runtime driver code lives under `real/` or `sim/`;
driver tests live under `tests/drivers/`.

## Device Roles

Each device folder maps to a short runtime role:

| Role | Real backend | Sim backend |
| --- | --- | --- |
| `driver` | `real/thunder/` (ThunderDriver, gRPC → brainstem CMS) | `sim/mujoco/` (MuJoCo in-process driver) |
| `camera` | `real/camera/` (Orbbec SDK native + DDS) | `sim/camera/` (MuJoCo camera) |
| `lidar` | `real/lidar/` (Livox MID-360 SDK2 + DDS) | `sim/lidar/` (MuJoCo LiDAR ray casting) |
| `imu` | `real/imu/` (Livox MID-360 IMU facade) | `sim/imu/` (MuJoCo IMU) |
| `gnss` | `real/gnss/` (WTRTK-980 native DDS) | — |

`real/teleop_module.py` provides joystick teleop as a standalone Module.

## Contract

Drivers expose typed runtime ports such as `cmd_vel`, `stop_signal`,
`odometry`, `color_image`, `depth_image`, `camera_info`, `scan`, `raw_scan`,
`imu`, and `map_cloud`.

Upper layers select drivers through `runtime.registry` and Blueprint factories.
They should not import concrete driver classes for behavior.

## Boundary

- `drivers/` does not decide goals, plans, safety policy, or semantic actions.
- ROS 2/native device code stays behind explicit bridge modules or real-device
  backends.
- Simulation and real backends should keep the same port shape where practical.
