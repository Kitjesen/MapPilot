# Native Navigation Tuning

LingTu has one field navigation implementation: native C++ `navd` under
`src/nav/cpp/`. Do not tune a second Python planner, follower, terrain Module,
or command mux.

## Sources of truth

| Setting | Edit here | Compiled by |
| --- | --- | --- |
| Product control mode and planner backend | `config/runtime_graph/products/*.yaml` under `native_nav` | `src/lingtu/assembly/native_nav.py` |
| Product path-following thresholds and speed defaults | `config/runtime_graph/products/*.yaml` under `native_nav` | `src/lingtu/assembly/native_nav.py` |
| Rolling segment and risk parameters | `config/runtime_graph/products/*.yaml` under `parameters` | `src/lingtu/assembly/parameters.py` |
| Robot dimensions, sensor offsets, and calibration | `config/robots/<vendor>/<model>/` and the selected RobotConfig | Product assembly |
| Native endpoint parsing and hard defaults | `src/nav/cpp/endpoint/nav/runtime/config/config.hpp` and `parse.cpp` | `navd` |
| Local-planner and tracker behavior | `src/nav/cpp/planning/local/` and `src/nav/cpp/tracking/` | native CMake targets |

Product assembly renders the selected values into the `navd` process
environment. Do not hand-edit generated RunPlan files or duplicate the same
defaults in a systemd unit.

## Product-facing choices

The supported planner choices are explicit:

| Field | Values | Meaning |
| --- | --- | --- |
| `native_nav.global_planner` | `octoplanner3d`, `far` | Saved-map global planner. |
| `native_nav.local_planner` | `cmu`, `scan` | Native local-planning backend. |
| `native_nav.check_obstacle` | boolean | Enables native obstacle checking. |
| `native_nav.use_traversability_cost` | boolean | Uses native traversability risk. |
| `native_nav.teleop_local_planner` | boolean | Enables native assisted-teleop detours. |
| `native_nav.velocity_smoother` | mapping | Configures the native final velocity smoother. |

`teleop`, `teleop_avoid`, and `autonomy` are Product control modes, not runtime
toggles. Change Product intent in the Product declaration instead of switching
algorithm ownership inside a running Host.

The principal numeric Product inputs compiled for `navd` are:

- `waypoint_reached_m` and `goal_reached_m`;
- `path_follower_goal_tolerance_m`, `path_follower_lookahead_m`,
  `path_follower_min_speed_mps`, `path_follower_max_speed_mps`, and
  `path_follower_max_accel_mps2`;
- `teleop_max_speed_mps` and `teleop_max_yaw_rate_rad_s`;
- `vehicle_length_m`, `vehicle_width_m`, and sensor offsets;
- the `octoplanner3d_*` settings accepted by
  `src/lingtu/assembly/native_nav.py`.

If a low-level C++ option is not compiled from a Product source, do not add an
undocumented service override. First decide whether it is a real operator
setting; if it is, expose it once through Product assembly.

## Tuning workflow

1. Change one owning source.
2. Compile the Product for the intended `real` or `sim` environment.
3. Inspect the resolved `navd` process environment in the RunPlan.
4. Run the narrowest native or MuJoCo gate that exercises the changed behavior.
5. Compare path, tracking, safety, and terminal-goal evidence before changing a
   second parameter.

Do not infer field safety from a local import or a planner-only test. A speed,
acceleration, clearance, obstacle, or freshness change needs evidence from the
same command path it affects.

## Common symptoms

| Symptom | Inspect first | Typical adjustment |
| --- | --- | --- |
| Robot oscillates on a valid route | odometry delay, local path, follower lookahead | increase lookahead or reduce speed before changing planner scoring |
| Robot cuts corners | route geometry and follower lookahead | reduce speed or lookahead while preserving goal tolerance |
| Local path is repeatedly rejected | obstacle cloud, traversability age, selected local backend | fix stale/misaligned inputs before relaxing collision thresholds |
| Goal cannot be planned | active map identity, goal frame, saved `octomap.ot` or `occupancy.npz` | repair the map artifact or goal; do not bypass native admission |
| Assisted teleop does not move | native control mode, operator lease, input freshness, obstacle/traversability blockers | clear the reported blocker; do not publish a second velocity path |
| Command is accepted but motion stays held | native command receipt and control-authority status | follow the reported resume/reissue requirement with a fresh command |

## Map quality

Navigation tuning cannot compensate for a bad map. Inspect the active map,
LiDAR/IMU timing, extrinsics, localization quality, and the exact native planner
artifact before changing planner constraints. OctoPlanner3D consumes
`octomap.ot`; FAR consumes the validated occupancy artifact selected by the
Product.

## Runtime evidence

The generated RunPlan records both final `launch.parameters` and the exact
`navd` environment. Native endpoint
status reports the selected global/local planner, control mode, input blockers,
path-follower limits, control authority, and terminal goal state. Gateway
status presents that native state; it does not host another planner or expose
live Python planner objects.

For algorithm details and parameter semantics, use
[`LOCAL_PLANNING_AND_TRACKING_CONTRACT.md`](../architecture/LOCAL_PLANNING_AND_TRACKING_CONTRACT.md)
and the native endpoint's [`README.md`](../../src/nav/cpp/endpoint/README.md).
