# LingTu Tuning Cheat Sheet

Runtime parameters are owned by the selected `config/robots/<vendor>/<model>/robot.yaml` and the Product
runtime graph. The remaining secondary inputs are purpose-specific:
`config/semantic_scoring.yaml` owns semantic scoring; the selected robot's
`mid360_fastlio2.yaml` owns LiDAR/IMU calibration inputs. Retired ROS node
parameter files are not runtime configuration.

After editing, switch the Product again through the robot-side operations CLI.
ProductControl owns process ordering and readiness; do not restart field units
with `systemctl` directly:

```bash
bash scripts/lingtu --robot <vendor/model> --env real switch <product> [--map <name>]
```

## 1. Robot Speed Inputs - selected `robot.yaml`

| Key | Default | Range | Effect |
| --- | --- | --- | --- |
| `speed.max_linear` | 1.0 m/s | robot-specific | Chassis/legacy linear limit; not the native endpoint path-follower limit |
| `speed.max_angular` | 1.0 rad/s | robot-specific | Chassis/legacy angular limit |
| `speed.max_speed` | 0.875 m/s | 0.3-1.0 | Python Module/dev/compat planner normalization denominator |
| `speed.autonomy_speed` | 0.875 m/s | 0.3-1.0 | Python Module/dev/compat requested planning speed |

The native endpoint owns a separate speed surface. Its defaults are
`--max-speed-mps 0.4` and `--max-accel-mps2 1.0`; it does not read the Python
Module's `speed.max_speed` as its command ceiling.

## 2. Local Planner - `local_planner.*`

In the physical `env=real` Product runtime, the production local planner runs
inside the native C++ nav endpoint and uses the `nav_kernel` planning cores.
This is also the simulation Product boundary; it is not a ROS2 node. The exact
algorithm and runtime-lane differences are defined in
[Local Planning and Tracking Contract](architecture/LOCAL_PLANNING_AND_TRACKING_CONTRACT.md).

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
| `local_planner.use_traversability_cost` | true | Python Module: enable traversability hard/soft scoring |
| `local_planner.traversability_hard_cost` | 90 | Reject a path group at or above this risk |
| `local_planner.traversability_soft_cost` | 40 | Begin multiplicative risk penalty above this value |
| `local_planner.traversability_weight` | 0.01 | Soft-risk penalty slope |

The legacy ROS2 `cmu` backend is not a Module runtime backend. Use
`local_planner.effective_params` in module health to confirm tuning reached the
in-process planner.

`point_per_path_thre=2` means two or more voxel-correspondence obstacle hits
hard-reject a primitive. `use_cost=false` together with `slope_weight=0.0`
keeps the height-derived score factor at 1; obstacle collision and the separate
traversability grid still remain active when enabled.

The native endpoint defaults differ: traversability scoring is disabled unless
explicitly enabled, and its hard/soft/weight defaults are `80/40/0.01`. Its
`--local-planner-obstacle-height-max-m` ceiling defaults to `1.20 m` (environment
equivalent: `LINGTU_NAV_LOCAL_PLANNER_OBSTACLE_HEIGHT_MAX_M`); non-finite or
higher body-relative points are treated as invalid/overhead evidence and are
not admitted to near-field stop, collision scoring, or recovery checks.

## 3. Path Follower Effective Parameters

The production path follower is also in-process `nav_kernel`. It is a
lookahead-point holonomic geometric tracker, not a local planner and not the
classic curvature-form Pure Pursuit controller.

The names below describe `nav_kernel::PathFollowerParams`; not every field is
an independently exposed YAML setting. The endpoint exposes speed/acceleration
through CLI/environment, while the Python Module derives these fields from its
constructor/profile settings.

| Core field | Native endpoint default | Effect |
| --- | --- | --- |
| `baseLookAheadDis` | 0.3 m | Lookahead at zero velocity |
| `lookAheadRatio` | 0.5 | Extra lookahead per m/s |
| `yawRateGain` | 7.5 | Turning gain |
| `maxYawRateRadS` | 0.8 rad/s | Yaw-rate limit in public physical units |
| `headingAlignEnterRad` | 0.785 rad | Enter pure heading alignment |
| `headingAlignExitRad` | 0.35 rad | Resume translation; must remain below enter angle |
| `maxSpeed` | 0.4 m/s | Native endpoint velocity ceiling |
| `maxAccel` | 1.0 m/s^2 | Native endpoint signed scalar-speed ramp |
| `stopDisThre` | 0.2 m | Follower stop band around the tracked path end |

Adaptive lookahead uses the follower's internal commanded scalar speed, not
measured odometry speed. With the native endpoint's `0.4 m/s` ceiling, the
reachable lookahead is `0.30-0.50 m`. `max_accel` does not limit yaw
acceleration, jerk, or `vx/vy` components independently.

Autonomous recovery is configured in the Product's `native_nav.recovery`
mapping and compiled to `LINGTU_NAV_RECOVERY_*` environment values. The `nav`
Product defaults are:

| Field | Default | Effect |
| --- | --- | --- |
| `behavior_order` | `[translate, rotate]` | Round-robin action order; failure or successful-but-ineffective recovery advances to the next entry |
| `translation_speed_mps` | 0.15 | Tracked side-step/reverse recovery speed |
| `rotation_rate_rad_s` | 0.25 | Requested recovery yaw rate |
| `min_rotation_rad` / `max_rotation_rad` | 0.20 / 1.20 | Adaptive candidate-angle interval |
| `rotation_candidate_step_rad` | 0.20 | Candidate-angle spacing |
| `rotation_sample_step_rad` | 0.05 | Footprint-sweep validation spacing |
| `rotation_timeout_s` / `translation_timeout_s` | 2.5 / 1.5 | No-odometry-progress timeouts, not total action duration |
| `max_attempts` | 3 | Bounded total action attempts before terminal recovery exhaustion |

Do not add a navigation-side "clear map" recovery action. Mapd owns live map
state; after a successful recovery action Executor waits for fresh cloud and
traversability evidence and then invokes the ordinary planner again. If that
replan is still blocked, recovery continues at the next `behavior_order` entry;
the cursor resets only after ordinary planning succeeds, a new task resets the
controller, or recovery exhausts its bounded attempts.

Do not confuse the follower stop band with `local_planner.near_field_stop_dis`
(`0.5 m`) or the native mission goal-reached threshold (`0.35 m`). Profile and
endpoint overrides must be checked from the resolved runtime config instead of
copying historical `thunder_nav` values into field deployments.

## 4. Global Planner Backend

Supported native backends:

- `octoplanner3d`: default saved-map 3-D planner.
- `far`: explicit 2-D occupancy planner.

Select the backend in the Product's `native_nav.global_planner` declaration.
Do not start a Python planner beside `navd`.

## 5. Verification

```bash
python -m pytest tests/runtime/ -q
export LINGTU_HOST=ROBOT_IP_OR_HOSTNAME
ssh sunrise@${LINGTU_HOST} 'bash /opt/lingtu/current/scripts/lingtu status'
ssh sunrise@${LINGTU_HOST} 'curl -fsS http://localhost:5050/api/v1/health'
```

Gateway config snapshot: `http://<robot-ip>:5050/api/v1/config`.
