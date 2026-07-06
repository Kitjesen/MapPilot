# SLAM Interface Compatibility

This directory contains service definitions and notes for SLAM/map
compatibility surfaces. Product navigation uses the native map-save adapter and
native DDS endpoint contracts; ROS service names in this directory are legacy
compatibility surfaces only.

## Services

| Service | Type | Owner | Purpose |
| --- | --- | --- | --- |
| `/save_map` | `SaveMaps.srv` | Fast-LIO2 / map bridge | Compatibility map-save entry |
| `/pgo/save_maps` | `SaveMaps.srv` | legacy ROS 2 PGO bridge | Compatibility optimized-map save; product callers use `save_slam_map` |
| `/pgo/save_poses` | `SavePoses.srv` | legacy ROS 2 PGO bridge | Compatibility optimized-trajectory save |
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

## Native Map Save

The canonical product API is `save_slam_map`. The saved-map directory is the
contract, not a single service name:

```text
Fast-LIO scan/map output
  -> patches/*.pcd + poses.txt
  -> native patch pose-graph loop correction
  -> native voxel refine / cleanup
  -> map.pcd + map.raw.pcd + map_optimization.json
  -> occupancy.npz / octomap.ot / metadata.json
```

`save_pgo_map` remains as a compatibility alias because old ROS 2 adapters and
tests still call it. New product code should call `save_slam_map` or the
registered map-save adapter contract.
