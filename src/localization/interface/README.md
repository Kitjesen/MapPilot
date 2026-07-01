# SLAM Interface Compatibility

This directory contains ROS service definitions and notes for SLAM/map
compatibility. Product navigation does not start global planning through the
old PCT ROS launch stack.

## Services

| Service | Type | Owner | Purpose |
| --- | --- | --- | --- |
| `/save_map` | `SaveMaps.srv` | Fast-LIO2 / map bridge | Save current map artifacts |
| `/pgo/save_maps` | `SaveMaps.srv` | PGO bridge | Save optimized map and patches |
| `/pgo/save_poses` | `SavePoses.srv` | PGO bridge | Save optimized trajectory |
| `/relocalize` | `Relocalize.srv` | Localizer bridge | Relocalize against a saved map |
| `/relocalize_check` | `IsValid.srv` | Localizer bridge | Check relocalization health |
| `/refine_map` | `RefineMap.srv` | Map tool bridge | Refine a saved map directory |

## Runtime Boundary

SLAM/localization provides map and odometry data to LingTu modules. It does not
own the navigation state machine.

```text
SLAM / localization
  -> odometry + map_cloud + localization_health
  -> Navigation / map layers / Gateway
```

Navigation goals enter the LingTu backend as `goal_pose`. The start pose comes
from current localization/odometry. Global planning then runs through
`GlobalPlanner` and the selected backend, normally `octoplanner3d`.

```text
goal_pose + odometry + map
  -> Navigation
  -> GlobalPlanner
  -> OctoPlanner3DBackend
  -> global_path / waypoint
  -> LocalPlanner
```

Legacy ROS commands that started `pct_planner` launch files or published
`/way_point` directly are intentionally not documented here. ROS compatibility
belongs in `src/*/adapters/ros2/` and bridge modules.
