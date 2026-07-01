# LingTu Tuning Cheat Sheet

> Edit YAML, restart the systemd service, no rebuild needed.

The single source of truth for runtime parameters is `config/robot_config.yaml`. Other YAML files
(`config/semantic_*.yaml`, `config/pointlio.yaml`, `config/far_planner.yaml`,
`config/dufomap.toml`) hold module-specific overrides. After editing, restart the affected
service:

```bash
sudo systemctl restart lingtu          # algorithm layer (gateway + nav + perception)
sudo systemctl restart robot-fastlio2  # SLAM only
```

---

## 1. Robot kinematic limits 鈥?`config/robot_config.yaml`

| Key                     | Default | Range     | Effect                                                       |
|-------------------------|---------|-----------|--------------------------------------------------------------|
| `robot.max_linear_vel`  | 1.0 m/s | 0.3-2.0   | Path-follower velocity ceiling                               |
| `robot.max_angular_vel` | 1.0 rad/s | 0.3-2.0 | Turn-rate ceiling                                            |
| `robot.stuck_timeout`   | 10.0 s  | 5-20      | Stuck detector window (`waypoint_tracker.py`)                |
| `robot.stuck_dist_thre` | 0.15 m  | 0.05-0.30 | Min displacement within window before declaring stuck        |
| `robot.cruise_speed`    | 0.6 m/s | 0.3-1.0   | Default mission speed                                        |

Tuning notes:
- False stuck triggers in tight indoor turns -> raise `stuck_timeout` to 12-15.
- Slow recovery in open terrain -> drop to 7-8.

## 2. Local planner 鈥?`config/robot_config.yaml` `local_planner.*`

The C++ local planner lives in `src/nav/core/include/nav_core/local_planner_full.hpp`
and is called from Python via `_nav_core` (nanobind). Parameters are read once at
`LocalPlannerModule.start()`.

| Key                            | Default | Effect                                                   |
|--------------------------------|---------|----------------------------------------------------------|
| `local_planner.adjacent_range` | 3.5 m   | Obstacle consideration radius                            |
| `local_planner.path_scale`     | 1.0     | Candidate path length multiplier                         |
| `local_planner.min_path_scale` | 0.75    | Shortest candidate-path scale during recovery search     |
| `local_planner.path_scale_step` | 0.25   | Scale decrement when no full-size path is feasible       |
| `local_planner.min_path_range` | 1.0 m   | Shortest accepted planning horizon                       |
| `local_planner.path_range_step` | 0.5 m  | Horizon decrement when shortening the search             |
| `local_planner.point_per_path_thre` | 2  | Hit count before a candidate path is treated as blocked  |
| `local_planner.dir_weight`     | 0.02    | Direction-alignment cost (higher -> straighter paths)    |
| `local_planner.slope_weight`   | 0.0     | Per-voxel slope penalty (0 disables, 3-6 outdoor)        |
| `local_planner.near_field_stop_dis` | 0.5 m | Emergency stop distance directly in front of body    |
| `local_planner.check_obstacle` | true    | Enable collision filtering                               |
| `local_planner.two_way_drive`  | true    | Allow reverse motion candidates                          |

`nanobind` and the managed C++ `cmu` backend now read the same CMU/FALCO-style
keys for the fields they both support. The module health payload exposes
`local_planner.effective_params`; use that to confirm a YAML edit reached the
in-process C++ planner.

Symptom -> change:
- Robot oscillates around path -> raise `dir_weight` to 0.05, lower `path_scale` to 0.8.
- Cuts corners in narrow halls -> drop `adjacent_range` to 2.5.
- Climbs ramps it should avoid -> set `slope_weight` to 4-6.

## 3. Path follower 鈥?`config/robot_config.yaml` `path_follower.*`

The path follower is also `_nav_core` C++ (`PathFollowerCore`). Lookahead is adaptive:
`L = base + ratio * speed`.

| Key                              | Default | Effect                                              |
|----------------------------------|---------|-----------------------------------------------------|
| `path_follower.base_look_ahead`  | 0.3 m   | Lookahead at zero velocity                          |
| `path_follower.look_ahead_ratio` | 0.5     | Extra lookahead per m/s                             |
| `path_follower.yaw_rate_gain`    | 7.5     | Proportional turning gain                           |
| `path_follower.max_yaw_rate`     | 45 deg/s | Hard limit on commanded yaw                        |
| `path_follower.stop_dis_thre`    | 0.5 m   | Stop distance to final waypoint                     |

Symptom -> change:
- Snaking left-right on straight path -> drop `yaw_rate_gain` to 5-6, raise `base_look_ahead`.
- Sluggish turns -> raise `yaw_rate_gain` to 10-12.

## 4. Global planner backend 鈥?`core.registry`

Two backends are registered in `src/global_planning/`:
- `astar` 鈥?pure Python, default for `stub`/`dev`/`sim_nav` profiles.
- `pct` 鈥?C++ `ele_planner.so` (PCT planner), default for the real `nav` profile.

Switch via the profile flag or REPL:

```bash
python lingtu.py nav --planner pct
```

PCT-specific tuning lives in `src/global_planning/pct_planner/config/params.yaml`.

## 5. Frontier exploration 鈥?`config/semantic_exploration.yaml`

Used only by the `explore` profile. Weights should sum to ~1.0.

| Key                               | Default | Effect                                              |
|-----------------------------------|---------|-----------------------------------------------------|
| `frontier_distance_weight`        | 0.25    | Prefer near frontiers                               |
| `frontier_novelty_weight`         | 0.35    | Prefer unexplored regions                           |
| `frontier_language_weight`        | 0.20    | Bias by language goal embedding                     |
| `frontier_grounding_weight`       | 0.20    | Bias by scene-graph spatial cue                     |
| `frontier_score_threshold`        | 0.15    | Reject candidates below this                        |
| `frontier_min_size`               | 5 cells | Minimum frontier patch size                         |
| `confidence_threshold`            | 0.5     | Below this, drop into frontier mode                 |

## 6. Semantic planner 鈥?`config/semantic_planner.yaml`

| Key                  | Default       | Effect                                                |
|----------------------|---------------|-------------------------------------------------------|
| `backend`            | `kimi`        | LLM: kimi / openai / claude / qwen / mock             |
| `model`              | `kimi-k2.5`   | Model name                                            |
| `timeout_sec`        | 15.0 s        | Primary LLM timeout                                   |
| `confidence_threshold` | 0.6         | Below this, fall back to exploration                  |

Offline mode: set `backend: mock` and unset `MOONSHOT_API_KEY`.

## 7. Perception 鈥?`config/semantic_perception.yaml`

| Key                            | Default | Effect                                          |
|--------------------------------|---------|-------------------------------------------------|
| `detector.confidence_threshold`| 0.4     | YOLO/BPU detection cutoff                       |
| `detector.iou_threshold`       | 0.45    | NMS overlap threshold                           |

## 8. Verifying changes

```bash
python -m pytest src/core/tests/ -q             # framework tests, no robot needed
ssh sunrise@192.168.66.190 'lingtu status'       # one-screen status on the robot
ssh sunrise@192.168.66.190 'lingtu health'       # GET /api/v1/health
```

Web Gateway exposes module config snapshot at `http://<robot>:5050/api/v1/config`.
