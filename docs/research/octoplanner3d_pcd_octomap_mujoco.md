# OctoPlanner3D PCD/OctoMap vs MuJoCo Stair Map Gap

Status: dated investigation; not current planner acceptance evidence

Date: 2026-07-07

## Confirmed Planner Route

OctoPlanner3D's product route remains:

```text
PCD point cloud
-> OctoMap
-> traversable space extraction
-> collision / ground-support checks
-> 3D 26-neighbor A*
```

Do not replace this with a different map artifact contract unless the global
planner implementation is intentionally changed.

## Building2_9 Control Run

Input:

```text
sim/fixtures/octoplanner3d/building2_9.pcd
start = [7.9, -2.7, 0.3]
goal  = [2.5, -0.3, 13.7]
```

Because the local headless runtime cannot read binary PCD directly, the control
run converts the source PCD to ASCII first, then runs the same production-style
chain:

```text
building2_9_ascii.pcd
-> octoplanner3d_pcd_to_octomap
-> octomap.ot
-> octoplanner3d_headless
```

Evidence:

```text
artifacts/building2_9_octoplanner3d_rerun/plan.json
artifacts/building2_9_octoplanner3d_rerun/map_quality.json
```

Result:

```text
planner ok: true
reached_goal: true
path_points: 41
source points: 853,227
converted occupied voxels: 93,306
```

This confirms the current OctoPlanner3D route can solve a real multi-floor
building map when the source PCD/OctoMap has enough map evidence.

## MuJoCo Synthetic Stair Map

Input:

```text
artifacts/mujoco_nav_suite/multifloor_stack_3_dense_center_band_plan/same_source_map/map.pcd
```

Evidence:

```text
artifacts/octoplanner3d_map_quality_mujoco_multifloor_dense_center_band.json
```

Result:

```text
planner ok: true
path_points: 125
source points: 73,968
occupied voxels at 0.09 m: 15,830
avg points per occupied voxel: 4.67
nearest same-layer XY gap median: 1.73 m
```

The path is accepted by support checks, but the path is not tightly backed by
dense same-layer PCD evidence. This explains why the viewer can look like the
path is walking at the side of the stair volume.

## MuJoCo Live LiDAR Stair Map Without Artificial Support

Command profile:

```text
--scene-preset multifloor_stack_3
--map-source mujoco_lidar
--trajectory-support-radius-m 0
--lidar-duration 8
--resolution 0.09
```

Evidence:

```text
artifacts/mujoco_nav_suite/multifloor_stack_3_live_lidar_no_support_plan/report.json
```

Result:

```text
planner ok: false
error: Start is occupied/out of map and no nearby free cell
source points: 424,540
converted occupied voxels: 10,210
live map z_max: 2.08 m
goal z: 4.15 m
trajectory_support_point_count: 0
```

The live MuJoCo LiDAR map does not cover the upper floors. It also has no
support under the configured start cell. This is a map-generation/coverage
problem, not evidence that OctoPlanner3D cannot use OctoMap.

## Code Change

Artificial trajectory support in MuJoCo LiDAR map generation is now opt-in:

```text
--trajectory-support-radius-m default: 0.0
```

This prevents the saved-map gate from silently adding fake support points along
the robot trajectory.

Relevant files:

```text
sim/scripts/mujoco/saved_map_quality_gate.py
sim/scripts/saved_map_relocalize_runtime_gate.py
sim/scripts/mujoco/native_navigation_acceptance.py
```

The earlier Python combined plan/tracking experiment was removed. Current
Product evidence keeps saved-map quality, native relocalization, and native
navigation acceptance as separate gates.

## Next Fix

The next meaningful fix is not an OctoPlanner3D parameter sweep. It is to make
MuJoCo mapping produce a complete multi-floor PCD before planning:

```text
1. drive a real mapping trajectory through every floor and stair connector;
2. publish LiDAR frames from that trajectory;
3. save map.pcd from the accumulated SLAM/map cloud;
4. convert map.pcd to octomap.ot;
5. run OctoPlanner3D;
6. only then run tracking/local planner.
```

The saved-map stair gate should fail if the map source is live LiDAR and the
target floor is not present in the generated PCD/OctoMap.
