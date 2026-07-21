# nav.local

`nav.local` owns the complete short-horizon motion chain. It is runtime
execution, not a service API.

The normal path is in-process:

- `terrain.py` turns odometry and point clouds into local traversability.
- `local_planner.py` owns the Module ports and publishes a local path.
- `path_follower.py` turns the selected local path into velocity commands.
- `local_planner_runtime.py` selects and validates the configured backend.
- `local_planner_backend.py`, `native.py`, `parameters.py`,
  `geometry.py`, and `obstacles.py` hold the local planner implementation
  details.
- `cpp/` is the ROS-free C++ hot path.
- `paths/` contains the precomputed CMU candidate path bank.

`nav.services.plan` owns only global-planning request/service contracts. It
must not own LocalPlanner, PathFollower, or Terrain Modules.

The historical ROS2 shells that used to live under `legacy_ros/` (`terrain_analysis`,
`terrain_analysis_ext`, `local_planner`, `sensor_scan_generation`) were `COLCON_IGNORE`d,
never part of the product Module graph, and have been retired outright. Current
ROS replacement status is tracked in `docs/architecture/ROS_ROLE_REPLACEMENT_MAP.md`.
Their source remains available in git history if ever needed for reference.
