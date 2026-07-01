# Parameter Tuning Guide

Detailed tuning notes for the four performance-critical layers. The runtime entry is
`python lingtu.py <profile>` (managed by `lingtu.service`); all parameters are read from
`config/robot_config.yaml` at module start. There are no per-launch XML files in the current
codebase 鈥?restart the service to pick up changes.

For a one-page cheat sheet see `docs/TUNING.md`. For symptom-based diagnostics see
`TROUBLESHOOTING.md`.

---

## 1. Path Follower (`local_planner.path_follower.*`)

Implementation: `src/nav/core/include/nav_core/path_follower_core.hpp` (C++, called via
`_nav_core` nanobind). Wired to `PathFollowerModule` in `src/base_autonomy/path_follower_module.py`.

### Speed and acceleration

```yaml
path_follower:
  max_speed: 1.0          # m/s, ceiling for autonomous mode
  max_accel: 1.0          # m/s^2
```

Indoor: 0.6-1.0 m/s. Outdoor open: 1.5-3.0 m/s. Start at 0.5 m/s for first run.
Higher `max_accel` is more responsive but jerkier.

### Adaptive lookahead

```yaml
path_follower:
  base_look_ahead: 0.3    # m at 0 velocity
  look_ahead_ratio: 0.5   # extra m per m/s
  min_look_ahead: 0.2
  max_look_ahead: 2.0
```

Formula: `L = base + ratio * speed`. At 2 m/s -> L = 1.3 m.

| Symptom                   | Adjust                                                            |
|---------------------------|-------------------------------------------------------------------|
| Oscillation, overshoot    | `base_look_ahead` too small -> raise +0.1                         |
| Cuts corners              | `look_ahead_ratio` too large -> drop -0.1                         |

### Turning

```yaml
path_follower:
  yaw_rate_gain: 7.5      # P-gain
  max_yaw_rate: 45.0      # deg/s safety limit
```

Sluggish: 10-15. Oscillating: 5-7. `max_yaw_rate` is a clamp, not a gain.

---

## 2. Local Planner (`local_planner.*`)

Implementation: `src/nav/core/include/nav_core/local_planner_full.hpp`. Module:
`src/base_autonomy/local_planner_module.py`. Builds a CSR sparse candidate-path bank,
scores 36 rotations in OpenMP, picks lowest-cost free path.

### Obstacle handling

```yaml
local_planner:
  laser_voxel_size: 0.1        # m, point cloud downsample
  obstacle_height_thre: 0.2    # m, points above ground that count as obstacles
```

Smaller voxel = finer detail, more CPU. On S100P aarch64, 0.1 is the sweet spot.

### Planning horizon

```yaml
local_planner:
  min_path_range: 2.5     # m, MUST be >= path_follower.max_look_ahead
  adjacent_range: 3.5     # m, obstacle consideration radius
  path_scale: 1.0
  min_path_scale: 0.75
  path_scale_step: 0.25
  path_range_step: 0.5
```

`adjacent_range` controls the local point-cloud crop. `path_scale*` changes the
candidate-path bank scale, while `path_range_step` shortens the horizon after a
blocked attempt. Keep `min_path_range` large enough for the follower lookahead.

### Cost weights

```yaml
local_planner:
  dir_weight: 0.02        # higher -> prefers straight path
  slope_weight: 0.0       # 0 disables; 3-6 for outdoor slopes
  point_per_path_thre: 2  # obstacle hits before a path is blocked
  use_cost: true          # enable terrain cost (requires terrain analysis)
```

`slope_weight` was added so outdoor courses penalize ramps; defaults to 0 indoor.
The `nanobind` backend reports the applied values under
`LocalPlannerModule.health()["local_planner"]["effective_params"]`; use that
instead of assuming the YAML reached the active backend.

---

## 3. Terrain Analysis (`terrain.*`)

Implementation: `src/nav/core/include/nav_core/terrain_core.hpp`. Module:
`src/base_autonomy/terrain_module.py`. Maintains a rolling voxel grid centered on the robot.

### Voxel configuration

```yaml
terrain:
  scan_voxel_size: 0.1    # m, scan downsample
  terrain_voxel_size: 1.0 # m, rolling map resolution
```

### Temporal filtering

```yaml
terrain:
  decay_time: 10.0        # s, how long to remember an obstacle
  no_decay_dis: 2.0       # m, points within this radius never decay
```

Static scenes: `decay_time: 30`. Crowded scenes: `decay_time: 5`.

### Ground detection

```yaml
terrain:
  ground_height_thre: 0.1 # m, points below = ground
  dis_ratio_z: 0.1        # extra Z tolerance per meter of distance
```

---

## 4. Global Planner

Two backends registered in `core.registry` (`src/core/registry.py`):

- **`astar`** 鈥?pure Python, `src/nav/global_planner_service.py`, default in
  `stub`/`dev`/`s100p` profiles.
- **`pct`** 鈥?C++ PCT planner (`ele_planner.so`), used by the `explore` profile and
  selectable via `--planner pct`.

PCT tuning: `src/global_planning/pct_planner/config/params.yaml`.

```yaml
w_traversability: 1.0
w_smoothness: 0.2
w_length: 0.1
trajectory_resolution: 0.1
max_iterations: 100
convergence_threshold: 0.01
```

Off-road -> raise `w_traversability`. Smoother paths -> raise `w_smoothness`.

---

## Common Scenarios

### Robot too slow

1. Raise `path_follower.max_speed`.
2. Verify `safety_ring_module` is not throttling 鈥?`lingtu log error | grep safety`.
3. Check terrain analysis is publishing 鈥?`lingtu status` `[2] SLAM hz`.

### Robot cuts corners

1. Raise `path_follower.base_look_ahead` to 0.4-0.5.
2. Drop `look_ahead_ratio` to 0.3-0.4.
3. Raise `yaw_rate_gain` for sharper turns.
4. Drop `max_speed` in tight spaces.

### Robot oscillates

1. Drop `yaw_rate_gain` to 5-6.
2. Raise `base_look_ahead` to 0.4.
3. Check `/Odometry` rate (`ros2 topic hz /Odometry`, expect ~100 Hz).
4. Smooth terrain map: `scan_voxel_size: 0.15`.

### Global planner returns empty path

1. Goal outside loaded map 鈥?check `lingtu status` `[1] Session map=`.
2. Drop `w_traversability` (too picky on cost).
3. Rebuild tomogram (`lingtu map save <name>` then `build_tomogram` POST 鈥?see
   `docs/04-deployment/lingtu_cli.md`).

---

## Verifying live values

The `_nav_core` C++ modules don't expose ROS2 parameters. Inspect via the Gateway:

```bash
curl http://192.168.66.190:5050/api/v1/config | jq '.local_planner'
curl http://192.168.66.190:5050/api/v1/config | jq '.path_follower'
```

For framework-side modules, REPL:

```bash
python lingtu.py nav
> config local_planner
> module PathFollowerModule
```

---

## Performance vs Safety

| Parameter                       | Conservative | Aggressive |
|---------------------------------|--------------|------------|
| `path_follower.max_speed`       | 0.5 m/s      | 2.0 m/s    |
| `path_follower.max_accel`       | 0.5 m/s^2    | 2.0 m/s^2  |
| `local_planner.laser_voxel_size`| 0.05 m       | 0.2 m      |
| `local_planner.adjacent_range`  | 5.0 m        | 2.0 m      |
| `terrain.decay_time`            | 30 s         | 3 s        |
| `local_planner.obstacle_height_thre` | 0.1 m   | 0.3 m      |

Start conservative; relax once odometry health is verified for the environment.

---

## Stuck Detection (`waypoint_tracker.py`)

Implementation: `src/nav/waypoint_tracker.py`. Periodic check of robot displacement
inside a sliding window.

| Key                       | Default | Indoor | Outdoor |
|---------------------------|---------|--------|---------|
| `robot.stuck_timeout`     | 10.0 s  | 8.0    | 15.0    |
| `robot.stuck_dist_thre`   | 0.15 m  | 0.10   | 0.20    |

Behavior:
1. At 50% of `stuck_timeout` -> `WARN_STUCK` event.
2. At 100% -> `STUCK` event, recovery cmd_vel published via `CmdVelMux` (priority 60).
3. Reverse-motion detection: if commanded forward but actually moving backward, the
   timeout is compressed to at most 3 s.
4. Recovery requires 3 consecutive frames of |v| > 0.05 m/s before clearing the flag.

---

## Related

- `docs/TUNING.md` 鈥?one-page cheat sheet
- `docs/03-development/TROUBLESHOOTING.md` 鈥?diagnostics
- `docs/04-deployment/lingtu_cli.md` 鈥?`lingtu` operations CLI
