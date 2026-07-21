# LingTu Tuning Cheat Sheet

Runtime parameters are owned by `config/robot_config.yaml`. Module-specific
files such as `config/perception.yaml`, `config/decision.yaml`,
`config/semantic_scoring.yaml`, `config/pointlio.yaml`, and `config/dufomap.toml`
are secondary overrides.

After editing, restart only the affected service. The physical
`thunder_field` stack normally uses the native services below:

```bash
sudo systemctl restart lingtu-livox-dds       # LiDAR source
sudo systemctl restart lingtu-slam-dds        # SLAM / localization
sudo systemctl restart lingtu-traversability-dds
sudo systemctl restart lingtu-nav-dds         # native planner and tracker
sudo systemctl restart lingtu-driver          # unique Brainstem speed exit
sudo systemctl restart lingtu                 # gateway and product modules
```

## 1. Robot Speed Inputs - `config/robot_config.yaml`

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

On the physical `thunder_field` endpoint, the production local planner runs
inside the native C++ Nav Endpoint and uses the `nav_kernel` planning cores. The
Python `LocalPlannerModule` remains available for simulation, development, and
explicit compatibility profiles. Neither product path is a ROS2 node. The exact
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
| `maxYawRate` | 45 deg/s | Yaw-rate limit |
| `maxSpeed` | 0.4 m/s | Native endpoint velocity ceiling |
| `maxAccel` | 1.0 m/s^2 | Native endpoint signed scalar-speed ramp |
| `stopDisThre` | 0.2 m | Follower stop band around the tracked path end |

Adaptive lookahead uses the follower's internal commanded scalar speed, not
measured odometry speed. With the native endpoint's `0.4 m/s` ceiling, the
reachable lookahead is `0.30-0.50 m`. `max_accel` does not limit yaw
acceleration, jerk, or `vx/vy` components independently.

Do not confuse the follower stop band with `local_planner.near_field_stop_dis`
(`0.5 m`) or the native mission goal-reached threshold (`0.35 m`). Profile and
endpoint overrides must be checked from the resolved runtime config instead of
copying historical `thunder_nav` values into field deployments.

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
export LINGTU_HOST=ROBOT_IP_OR_HOSTNAME
ssh sunrise@${LINGTU_HOST} 'bash /opt/lingtu/current/scripts/lingtu status'
ssh sunrise@${LINGTU_HOST} 'bash /opt/lingtu/current/scripts/lingtu health'
```

Gateway config snapshot: `http://<robot-ip>:5050/api/v1/config`.
