# 2026-07-07 Sunrise Native Nav No-Motion Run

## Scope

Target: `sunrise@192.168.66.13`

Purpose: validate the ROS-free native chain up to PathFollower output without publishing motor velocity.

Motion boundary:

- `lingtu-nav-dds.service` was restarted with `LINGTU_NAV_PUBLISH_CMD_VEL=0`.
- No real motor command was published during this run.
- Final state was cleared with cancel: `active_path=false`, `cmd_vel_published=0`.

## Runtime State

Active services:

- `lingtu-livox-dds.service`: active
- `lingtu-slam-dds.service`: active
- `lingtu-traversability-dds.service`: active
- `lingtu-nav-dds.service`: active
- `lingtu.service`: active

Active map after repair:

```text
/home/sunrise/data/nova/maps/active
-> /home/sunrise/data/nova/maps/accept_ready_20260702_162847
```

The previous active map and SLAM map were not same-source:

- Gateway active map had pointed at `accept_live_20260702_162432`.
- SLAM localization drop-in used `accept_ready_20260702_162847/map.pcd`.

This was corrected for the run by making `active` point to `accept_ready_20260702_162847`.

## SLAM Stationary Check

30 second sample from `/tmp/lingtu_slam_status.json`:

- State: `TRACKING` for all samples
- XY drift: `0.00178 m`
- XY span: `0.00982 m`
- Z drift: `-0.00091 m`
- Yaw drift: `-0.023 deg`
- `processed_scan_hz`: avg `9.56`, max `10.21`
- `lidar_input_hz`: avg `9.53`
- `imu_input_hz`: avg `200.67`
- Registered/map points: avg about `7551`
- Dropped lidar/IMU frames: `0`

Remote artifact:

```text
/home/sunrise/data/SLAM/navigation/artifacts/field_runs/2026-07-07_sunrise_stationary_localization.json
```

## OctoMap Repair

Problem found:

- `active/octomap.ot` was missing or stale for the selected navigation map.
- Default OctoPlanner3D ground-support planning failed on the prior artifact.
- Disabling ground support made the route possible, proving the problem was the map support layer, not DDS or goal transport.

Fix applied and built on the robot:

- `Pcd2OctomapConverter` now support-dilates occupied support voxels before carving free envelope.
- `octoplanner3d_pcd_to_octomap` is PCL-backed when PCL is available, so it can read binary PCD maps.
- Native nav endpoint can now receive OctoPlanner3D options through env/CLI instead of hard-coded defaults.

Rebuilt artifact:

```text
map.pcd:     1,421,223 source points
octomap.ot:  support_dilation_cells=1
free_layers_above=3
free_dilation_cells=1
```

Direct headless verification with ground support enabled:

```text
start: [0.001044, -0.006375, -0.000251]
goal:  [1.0, -0.03, 0.3]
path_points: 7
reached_goal: true
goal_error_m: 0.403609
```

## Traversability Tuning

Initial settings were too heavy for the live board:

```text
radius=15
max_points=20000
cache_points=80000
```

Observed effect: nav only saw fresh traversability intermittently.

Runtime perf preview settings:

```text
radius=8
max_points=8000
cache_points=30000
```

10 second sample after tuning:

- Nav received `61` traversability updates
- Nav received `61` terrain map updates
- Final `has_traversability=true`
- Final `has_terrain_map=true`

Follow-up tuning on the same board:

```text
radius=6
max_points=5000
cache_points=20000
```

20 second stable sample:

- SLAM registered cloud into nav: about `10.0 Hz`
- Odometry/TF into nav: about `20.0 Hz`
- Traversability/terrain into nav: about `6.4 Hz`
- Traversability process CPU after tuning: much lower than the initial saturated run, but still not producing at the requested `10 Hz`.

Reason:

- DDS and SLAM are not the limiting point; `registered_clouds` reaches nav at about `10 Hz`.
- The bottleneck is inside `lingtu_traversability_dds`.
- Each traversability publish currently computes occupancy, elevation, ESDF, terrain risk, terrain core output, and writes three DDS outputs (`traversability`, `terrain_map`, `terrain_map_ext`).
- The loop also gates publishes by `publish_hz`; when DDS batches multiple point-cloud samples, later samples in the same batch can be skipped by the gate.

Decision:

- Keep `radius=6`, `max_points=5000`, `cache_points=20000` for short no-motion and first low-speed navigation tests.
- Do not claim 10 Hz traversability yet.
- Next code improvement should split near-field obstacle output from slower traversability/elevation processing, or make the producer process the latest scan on a fixed timer instead of doing all terrain outputs inside the point-cloud callback.

## No-Motion Goal Result

Command:

```text
lingtu_nav_control goal 1.0 -0.03 0.3 0.0 --domain-id 0
```

Result:

- `publish_cmd_vel=false`
- `active_path=true` during preview
- `has_odom=true`
- `has_map_odom_tf=true`
- `has_traversability=true`
- `has_terrain_map=true`
- `has_terrain_map_ext=true`
- `global_path_points=7`
- `local_path_points=101`
- `last_plan.reason=accepted`
- `last_plan.reached_goal=true`
- `last_plan.elapsed_ms=379.399`
- `last_local.path_found=true`
- `last_local.reason=control_ready`
- Computed pre-output command: `vx=0.298624`, `vy=-0.028702`, `wz=-0.71864`
- Published motor velocity count: `cmd_vel_published=0`

Repeat after the `radius=6/max_points=5000/cache_points=20000` tuning:

- `has_traversability=true`
- `has_terrain_map=true`
- `has_terrain_map_ext=true`
- `global_path_points=7`
- `local_path_points=101`
- `last_plan.reason=accepted`
- `last_local.reason=control_ready`
- Pre-output command remained available, with motor publishing disabled.
- `cmd_vel_published=0`

After cancel:

- `active_path=false`
- `global_path_points=0`
- `local_path_points=0`
- `cmd_vel_published=0`

Remote artifact:

```text
/home/sunrise/data/SLAM/navigation/artifacts/field_runs/2026-07-07_sunrise_nav_no_motion.json
```

## Remaining Work

- The no-motion chain is now validated through PathFollower pre-output, but real motion remains intentionally disabled.
- `cancel` sometimes needs repeated DDS writes before the status snapshot reflects path clearing.
- Gateway readiness still has stale/old routecheck logic that assumes a `localizer` baseline instead of native DDS.
- Traversability perf settings should be promoted into the robot profile instead of remaining a field drop-in.
- Real motion test must be separate and start with low-speed, short-distance command release.
