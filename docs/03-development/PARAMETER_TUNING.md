# Parameter Tuning Guide

LingTu reads runtime parameters from `config/robot_config.yaml` at module
startup. Restart the relevant service or profile after changing config.

For field operation commands, see
`docs/04-deployment/lingtu_cli.md`. For architecture boundaries, see
`docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md`.

## Path Follower

Implementation:

- C++ core: `src/nav/cpp/include/nav_kernel/path_follower_core.hpp`
- Module wrapper: `src/nav/local/path_follower.py`

Key parameters:

```yaml
path_follower:
  max_speed: 1.0
  max_accel: 1.0
  base_look_ahead: 0.3
  look_ahead_ratio: 0.5
  min_look_ahead: 0.2
  max_look_ahead: 2.0
  yaw_rate_gain: 7.5
  max_yaw_rate: 45.0
```

Guidance:

| Symptom | Adjustment |
| --- | --- |
| robot oscillates | raise `base_look_ahead` by 0.1, lower `yaw_rate_gain` |
| cuts corners | lower `look_ahead_ratio`, lower speed in tight spaces |
| turns too slowly | raise `yaw_rate_gain`, check `max_yaw_rate` clamp |
| jerky acceleration | lower `max_accel` |

Start indoor tests at 0.5-0.8 m/s. Increase only after odometry, costmap, and
safety state are stable.

## Local Planner

Implementation:

- C++ core: `src/nav/cpp/planning/local/`
- Module wrapper: `src/nav/local/local_planner.py`
- Hot path backend: `nav_kernel` / nanobind

Key parameters:

```yaml
local_planner:
  laser_voxel_size: 0.1
  obstacle_height_thre: 0.2
  min_path_range: 2.5
  adjacent_range: 3.5
  path_scale: 1.0
  min_path_scale: 0.75
  path_scale_step: 0.25
  path_range_step: 0.5
  dir_weight: 0.02
  slope_weight: 0.0
  point_per_path_thre: 2
  use_cost: true
```

Guidance:

| Symptom | Adjustment |
| --- | --- |
| misses thin obstacles | lower `laser_voxel_size`, lower `point_per_path_thre` |
| too conservative | lower `adjacent_range`, raise `point_per_path_thre` carefully |
| ignores slopes | enable/raise `slope_weight` |
| follower outruns planner | keep `min_path_range >= path_follower.max_look_ahead` |

Use `LocalPlannerModule.health()["local_planner"]["effective_params"]` to
verify the active backend received the expected values.

## Terrain Analysis

Implementation:

- C++ core: `src/nav/cpp/include/nav_kernel/terrain_core.hpp`
- Module wrapper: `src/nav/local/terrain.py`

Key parameters:

```yaml
terrain:
  scan_voxel_size: 0.1
  terrain_voxel_size: 1.0
  decay_time: 10.0
  no_decay_dis: 2.0
  ground_height_thre: 0.1
  dis_ratio_z: 0.1
```

Guidance:

| Environment | Adjustment |
| --- | --- |
| static indoor | longer `decay_time`, smaller `scan_voxel_size` if CPU allows |
| crowded indoor | shorter `decay_time` |
| uneven outdoor | tune `ground_height_thre` and enable slope cost in local planner |

## Global Planner

Primary backend:

- `octoplanner3d`: default product global planner.

Compatibility/experiment backends:

- `pct`: legacy/manual experiment when explicitly selected.
- `astar`: deterministic tests and old comparisons.

Product navigation should use saved-map artifacts:

```text
map.pcd
metadata.json
octomap.ot
occupancy.npz
```

If the planner returns no path:

1. Confirm the goal is inside the loaded map.
2. Run `lingtu plan-preview --internal-only --strict`.
3. Check `octomap.ot` and `metadata.json`.
4. Rebuild map artifacts from `lingtu map save <name>` or the Gateway map API.
5. Lower traversability/cost strictness only after map artifacts are healthy.

## SLAM And Saved Map Quality

Map quality problems usually come from one of these layers:

1. LiDAR hit geometry or scan density.
2. LiDAR/IMU timestamp alignment.
3. LiDAR-to-body extrinsics.
4. SLAM odometry drift or motion mismatch.
5. Patch poses and save-time optimization.
6. 2D projection or height-slice filters.

Inspect:

```text
map.raw.pcd
map.pcd
patches/*.pcd
poses.txt
map_optimization.json
occupancy.npz
octomap.ot
```

For MuJoCo, remember that simulated odometry priors and ground truth are
diagnostic tools only. Product localization still needs the real SLAM chain.

## Common Scenarios

### Robot Too Slow

1. Raise `path_follower.max_speed`.
2. Check `safety_ring` is not throttling.
3. Confirm localization health and costmap freshness.
4. Verify the active velocity source in `CmdVelMux`.

### Robot Cuts Corners

1. Raise `path_follower.base_look_ahead` to 0.4-0.5.
2. Lower `look_ahead_ratio` to 0.3-0.4.
3. Lower speed in tight spaces.

### Robot Oscillates

1. Lower `yaw_rate_gain` to 5-6.
2. Raise `base_look_ahead`.
3. Check odometry rate and delay.
4. Smooth local terrain with a slightly larger `scan_voxel_size`.

### Planner Returns Empty Path

1. Check active map and goal coordinates.
2. Run `lingtu plan-preview --internal-only --strict`.
3. Rebuild `octomap.ot` and `occupancy.npz`.
4. Treat `tomogram.pickle` as legacy/PCT unless the profile explicitly selects
   PCT.

## Inspecting Live Values

Gateway:

```bash
export LINGTU_HOST=ROBOT_IP_OR_HOSTNAME
curl "http://${LINGTU_HOST}:5050/api/v1/config" | jq '.local_planner'
curl "http://${LINGTU_HOST}:5050/api/v1/config" | jq '.path_follower'
curl "http://${LINGTU_HOST}:5050/api/v1/health" | jq '.'
```

REPL:

```bash
python lingtu.py nav
> config local_planner
> module PathFollowerModule
> teleop status
```

Robot operations CLI:

```bash
bash scripts/lingtu status
bash scripts/lingtu plan-preview --internal-only --strict
```

## Safety Baselines

| Parameter | Conservative | Aggressive |
| --- | ---: | ---: |
| `path_follower.max_speed` | 0.5 m/s | 2.0 m/s |
| `path_follower.max_accel` | 0.5 m/s2 | 2.0 m/s2 |
| `local_planner.laser_voxel_size` | 0.05 m | 0.2 m |
| `local_planner.adjacent_range` | 5.0 m | 2.0 m |
| `terrain.decay_time` | 30 s | 3 s |
| `local_planner.obstacle_height_thre` | 0.1 m | 0.3 m |

Start conservative. Relax only after odometry, map, and safety health are
verified for the environment.
