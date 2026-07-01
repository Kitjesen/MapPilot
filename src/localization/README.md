# Localization

`src/localization` is the localization domain. It owns pose sources and localization health, not navigation decisions.

## Responsibilities

- Native `SlamModule` contract: pose, registered cloud, map cloud, saved map,
  localization health, GNSS fusion health, map->odom, and scene mode.
- LiDAR-inertial odometry assets and adapters: Fast-LIO2, Point-LIO, and
  saved-map relocalization compatibility bridges.
- Relocalization adapters and saved-map alignment helpers.
- GNSS, NTRIP, and GNSS/localization fusion diagnostics.
- Runtime outputs used by navigation: odometry, map cloud, map->odom transform, localization quality, localization status.

## Boundaries

- Navigation decisions stay in `src/nav`.
- Sensor device ownership stays in `src/drivers`.
- ROS2 bridge code is compatibility-only under `adapters/ros2`.
- SLAM does not publish navigation paths or velocity commands. Historical pose
  tracks may be saved as `poses.txt`, but `global_path`, `local_path`,
  `waypoint`, and `cmd_vel` belong to `src/nav`.
- No-ROS localization implementations attach behind `localization/slam/cpp`
  through the `ISlamBackend` contract.

## Main Files

- `slam/module.py`: native Python Module boundary for downstream consumers.
- `slam/cpp/`: ROS-free C++ `ISlamBackend` contract shared by Fast-LIO2 and
  Point-LIO backends.
- `bridge.py`: thin compatibility facade for the ROS2-backed localization bridge.
- `adapters/ros2/slam_bridge.py`: ROS2/DDS compatibility bridge implementation.
- `gnss_module.py`, `gnss_serial_driver.py`, `ntrip_client_module.py`: GNSS/RTK input and correction support.
- `localizer/`: ICP localizer native package.
- `fastlio2/`, `pointlio/`: native LIO packages kept as algorithm assets.
- `pgo/`, `hba/`: pose-graph/map optimization native packages.

The product/runtime names may still use `slam_profile` and `slam.service` for compatibility with deployed robots. New Python imports should use `localization.*`.
