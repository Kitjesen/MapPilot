# Drivers

`src/drivers/` owns robot and sensor ingress/egress. It is the hardware and
simulation boundary, not a planning layer.

## Layout

| Folder | Role |
| --- | --- |
| `real/` | Real robot and sensor backends: motion driver, Orbbec camera, Livox LiDAR/IMU, and native GNSS. |
| `sim/` | Simulation drivers (MuJoCo in-process driver, endpoint adapter, stub) and simulated sensor backends. |
| ROS 2 compatibility | No current product directory under `src/drivers/`. Keep any legacy ROS 2 reader/driver assets isolated outside the real/sim product backends. |

Root files stay minimal. Runtime driver code lives under `real/` or `sim/`;
driver tests live under `tests/drivers/`.

## Device Roles

Each device folder maps to a short runtime role:

| Role | Real backend | Sim backend |
| --- | --- | --- |
| `driver` | `real/motion/` (`lingtu_driver` with the `Go2` or `Doso` robot implementation, supervised by `lt-driver.service`) | `sim/mujoco/` (MuJoCo in-process driver) |
| `camera` | `real/camera/` (Orbbec SDK native process + POSIX SHM rings; DDS image publishing is compatibility/diagnostic) | `sim/camera/` (MuJoCo camera) |
| `lidar` | `real/lidar/` (Livox MID-360 SDK2 + DDS) | `sim/lidar/` (MuJoCo LiDAR ray casting) |
| `imu` | `real/lidar/` (Livox MID-360 native DDS stream) | native MuJoCo sensor publisher |
| `gnss` | `real/gnss/` (WTRTK-980 native DDS) | — |

## Contract

Drivers expose typed runtime ports such as `cmd_vel`, `stop_signal`,
`odometry`, `color_image`, `depth_image`, `camera_info`, `scan`, `raw_scan`,
`imu`, and `map_cloud`. Cross-process product topics use the typed DDS registry
in `src/message/topics.py`; high-bandwidth camera images use SHM by default.

Upper layers select drivers through `runtime.registry` and Blueprint factories.
They should not import concrete driver classes for behavior.

## Boundary

- `drivers/` does not decide goals, plans, safety policy, or semantic actions.
- ROS 2 code stays behind explicit compatibility adapters. Product device code
  is native DDS/SHM or direct vendor SDK behind a real-device backend.
- Simulation and real backends should keep the same port shape where practical.
