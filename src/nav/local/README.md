# nav.local

`nav.local` owns terrain analysis and path following. The local planner Module
now lives with the planner service boundary under `nav.services.plan`.

The normal path is in-process:

- `terrain.py` turns odometry and point clouds into local traversability.
- `../services/plan/local_planner/service.py` owns the Module ports and
  publishes a local path.
- `path_follower.py` turns the selected local path into velocity commands.
- `../services/plan/local_planner/cmu_py.py`, `native.py`, `parameters.py`,
  `geometry.py`, and `obstacles.py` hold the local planner implementation
  details.
- `../services/plan/local_planner/paths/` contains the precomputed CMU
  candidate path bank.

The historical ROS2 shells that used to live under `legacy_ros/` (`terrain_analysis`,
`terrain_analysis_ext`, `local_planner`, `sensor_scan_generation`) were `COLCON_IGNORE`d,
never part of the product Module graph, and have been retired outright. Current
ROS replacement status is tracked in `docs/architecture/ROS_ROLE_REPLACEMENT_MAP.md`.
Their source remains available in git history if ever needed for reference.
