# LingTu Tuning Cheat Sheet

Runtime parameters are owned by `config/robot_config.yaml`. Module-specific
files such as `config/perception.yaml`, `config/decision.yaml`,
`config/semantic_scoring.yaml`, `config/pointlio.yaml`, and `config/dufomap.toml`
are secondary overrides.

After editing, restart the affected service:

```bash
sudo systemctl restart lingtu
sudo systemctl restart robot-fastlio2
```

## 1. Robot Kinematic Limits - `config/robot_config.yaml`

| Key | Default | Range | Effect |
| --- | --- | --- | --- |
| `robot.max_linear_vel` | 1.0 m/s | 0.3-2.0 | Path-follower velocity ceiling |
| `robot.max_angular_vel` | 1.0 rad/s | 0.3-2.0 | Turn-rate ceiling |
| `robot.stuck_timeout` | 10.0 s | 5-20 | Stuck detector window |
| `robot.stuck_dist_thre` | 0.15 m | 0.05-0.30 | Min movement before stuck |
| `robot.cruise_speed` | 0.6 m/s | 0.3-1.0 | Default mission speed |

## 2. Local Planner - `local_planner.*`

The production local planner is the in-process `nav_kernel LocalPlanner` exposed
through the private `_nav_kernel` nanobind extension. It is not a ROS2 node.

| Key | Default | Effect |
| --- | --- | --- |
| `local_planner.adjacent_range` | 3.5 m | Obstacle consideration radius |
| `local_planner.path_scale` | 1.0 | Candidate path length multiplier |
| `local_planner.min_path_scale` | 0.75 | Shortest candidate-path scale |
| `local_planner.path_scale_step` | 0.25 | Scale decrement during recovery |
| `local_planner.min_path_range` | 1.0 m | Shortest accepted planning horizon |
| `local_planner.path_range_step` | 0.5 m | Horizon decrement step |
| `local_planner.point_per_path_thre` | 2 | Hit count before a path is blocked |
| `local_planner.dir_weight` | 0.02 | Direction-alignment cost |
| `local_planner.slope_weight` | 0.0 | Per-voxel slope penalty |
| `local_planner.near_field_stop_dis` | 0.5 m | Emergency stop distance |
| `local_planner.check_obstacle` | true | Enable collision filtering |
| `local_planner.two_way_drive` | true | Allow reverse motion candidates |

The legacy ROS2 `cmu` backend is not a Module runtime backend. Use
`local_planner.effective_params` in module health to confirm tuning reached the
in-process planner.

## 3. Path Follower - `path_follower.*`

The production path follower is also in-process `nav_kernel`.

| Key | Default | Effect |
| --- | --- | --- |
| `path_follower.base_look_ahead` | 0.3 m | Lookahead at zero velocity |
| `path_follower.look_ahead_ratio` | 0.5 | Extra lookahead per m/s |
| `path_follower.yaw_rate_gain` | 7.5 | Turning gain |
| `path_follower.max_yaw_rate` | 45 deg/s | Yaw-rate limit |
| `path_follower.stop_dis_thre` | 0.5 m | Stop distance to final waypoint |

## 4. Global Planner Backend

Primary backends:

- `octoplanner3d`: default for product, simulation, development, map, navigation, and exploration profiles.
- `pct`: compatibility/experiment backend, selected explicitly with `--planner pct`.
- `astar`: legacy unit-test/benchmark backend only.

Switch with:

```bash
python lingtu.py nav --planner pct
```

## 5. Verification

```bash
python -m pytest src/runtime/tests/ -q
ssh sunrise@192.168.66.190 'lingtu status'
ssh sunrise@192.168.66.190 'lingtu health'
```

Gateway config snapshot: `http://<robot>:5050/api/v1/config`.
