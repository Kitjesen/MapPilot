# 2026-07-04 Native DDS Field Closure

This note records the final no-motion validation after the runtime boundary
cleanup and sunrise board sync.

## Scope

- Target: `sunrise@192.168.66.13`
- Board path: `/opt/lingtu/current`
- Runtime code validated on the board: `c1bd8791`
- The final source marker may include this documentation-only closure commit.
- Validation mode: no-motion; velocity output remains disabled
- Main objective: prove Livox -> SLAM -> traversability -> native nav endpoint
  -> OctoPlanner3D preview without relying on Python DDS adapters in the robot
  main path.

## Services

All required field services were active:

| Service | State |
| --- | --- |
| `lingtu-livox-dds.service` | active |
| `lingtu-slam-dds.service` | active |
| `lingtu-traversability-dds.service` | active |
| `lingtu-nav-dds.service` | active |
| `lingtu.service` | active |

Gateway health reported `status=ok`, `modules_ok=31`, `modules_fail=0`.

## Sensor And SLAM Evidence

The Livox configuration is generated at service startup from LingTu config
instead of using the vendor sample path. On sunrise it resolved the data host
interface to `192.168.127.10`.

Observed live status:

| Metric | Value |
| --- | --- |
| `lidar_input_hz` | about `10.0 Hz` |
| `processed_scan_hz` | about `10.0 Hz` |
| `imu_input_hz` | about `200 Hz` |
| `registered_points` | about `8k` |
| `map_points` | about `8k` |
| `has_odom` | true |
| `relocalization_state` | completed |
| `frames_ok` | true |

`track_against_map` is supported and enabled through the C++ SLAM control path.
The continuous tracking period is configurable through
`LINGTU_SLAM_TRACK_AGAINST_MAP_PERIOD_S`; sunrise is configured at `5s` to avoid
stealing time from the 10 Hz live SLAM loop.

## Navigation Endpoint Evidence

Native navigation endpoint status:

| Metric | Value |
| --- | --- |
| `tick_hz` | `20` |
| `has_odom` | true |
| `has_traversability` | true |
| `obstacle_points` | about `8k` |
| `active_octomap` | `/home/sunrise/data/nova/maps/active/octomap.ot` |
| `publish_cmd_vel` | false |
| `has_goal` | false |
| `has_global_path` | false |

The endpoint is wired for odometry, registered cloud, traversability, active
OctoMap, goal/cancel, global plan, local plan, and pre-command tracking output.
It is still intentionally configured with `publish_cmd_vel=false` for field
preview.

## Map And Planning Evidence

Active map: `accept_ready_20260702_162847`.

No-motion map validation result:

| Field | Value |
| --- | --- |
| `artifact_gate.ok` | true |
| `map_plan_ok` | true |
| `selected_planner` | `octoplanner3d` |
| `preview.feasible` | true |
| `preview.path_points` | `2` |
| `motion_published` | false |

This proves the saved map artifact gate and OctoPlanner3D preview path are
working. It does not approve real motion.

## Remaining Motion Blockers

The remaining blockers are safety/product blockers, not transport blockers:

- `navigation_session_inactive`: the nav session has not been started.
- `start_snap_too_large`: the executable preview start pose is still too far
  from the saved map graph. This must be resolved before enabling velocity
  output.

Real motion remains blocked until start pose alignment and executable preview
both pass with `publish_cmd_vel=false`; only then should `/nav/cmd_vel` be
enabled in a controlled field test.

## Follow-up No-motion Nav Session

After the first closure, a navigating session was started on
`accept_ready_20260702_162847` with `publish_cmd_vel=false`.

The Gateway readiness blocker `real_runtime_evidence_missing_or_stale` was
caused by the no-motion evidence gate still requiring a hardware command route.
That gate now respects `hardware_boundary_required=false` for preflight
evidence. Full real-motion evidence still fails until `/nav/cmd_vel` is
actually validated.

Observed no-motion state:

| Metric | Value |
| --- | --- |
| Session mode | `navigating` |
| `can_accept_goal` | true |
| `real_runtime_evidence_ok` | true for preflight |
| `real_runtime_evidence_full_ok` | false |
| SLAM | `TRACKING`, about `10 Hz` |
| Native nav endpoint | `tick_hz=20`, odom/traversability present |
| `publish_cmd_vel` | false |
| `cmd_vel_published` | `0` |

One short forward `/api/v1/navigation/plan` preview returned an OctoPlanner3D
path with no path-safety blockers, but strict saved-map validation still failed
after relocalization:

```text
start_snap_xy_distance_m ~= 0.59
MAX_EXECUTABLE_START_SNAP_M = 0.5
blocker = start_snap_too_large
motion_published = false
```

Conclusion: the communication and no-motion planning path are working, but the
robot's current pose is not close enough to the saved map's executable
traversable graph. Do not enable velocity output from this standing pose. The
next field action is physical placement or map editing/regeneration around the
start cell, then rerun the same no-motion preview.

## 2026-07-05 Native Endpoint Closure

The active map was repaired with explicit voxel edits around the start cell.
`octoplanner3d_edit_octomap` now reads binary OctoMap data even when the file is
named `octomap.ot`; the board smoke test
`octoplanner3d_edit_octomap_binary_ot_smoke` passed.

The C++ nav endpoint now applies `map -> odom` before planning. The old behavior
used raw odometry as map-frame pose, which made OctoPlanner3D reject valid
goals. It also reports `last_local.reason` and `target_distance_m`, so field
debugging no longer has to infer why local planning stopped.

Board validation on `sunrise`:

| Check | Result |
| --- | --- |
| `lingtu-nav-dds.service` | active |
| `has_odom` / `has_map_odom_tf` / `has_traversability` | true / true / true |
| `publish_cmd_vel` | false |
| OctoPlanner3D goal `0.8, 0.0, 0.001` | accepted |
| `global_path_points` | 4 |
| `last_local.reason` | `control_ready` |
| `local_path_points` | 69 |
| `cmd_vel_published` | 0 |

Cancel cleared `active_path`, `global_path_points`, and `local_path_points`.
This proves the no-motion chain:

```text
Livox/Fast-LIO2 -> odometry + registered_cloud
-> C++ traversability
-> C++ native nav endpoint
-> OctoPlanner3D global path
-> C++ LocalPlanner local path
-> PathFollower pre-command output
```

Velocity output remains disabled. Real motion still needs a separate controlled
test with `publish_cmd_vel=true`.

## Local Verification

Local tests run for this change set:

```text
python -m pytest src/localization/tests/test_native_slam_contract.py src/runtime/tests/test_thunder_deployment_entrypoints.py -q
49 passed
```

Earlier focused deployment/config tests also passed:

```text
python -m pytest src/runtime/tests/test_livox_config_gen.py src/runtime/tests/test_thunder_deployment_entrypoints.py -q
33 passed
```

## 2026-07-05 Parallel Revalidation Attempt

This pass rechecked the missing acceptance groups separately: local contracts,
robot access, Gateway status, map artifacts, native nav state, and DDS status.

Local contract checks passed on branch `codex/runtime-boundary-cleanup` at
`383be2c5`:

```text
python -m pytest src/runtime/tests/test_server_setup_contract.py src/runtime/tests/test_profile_graph_snapshots.py -q --tb=short
59 passed

python -m pytest src/localization/tests/test_native_slam_contract.py src/runtime/tests/test_services_modules.py -q --tb=short
29 passed

python -m pytest src/nav/tests/test_nav_services.py src/gateway/tests/test_gateway_readiness.py src/gateway/tests/test_gateway_frames.py -q --tb=short
134 passed
```

The local workspace was not clean during this revalidation. The pushed source
state is `383be2c5`, but there are separate uncommitted simulation, docs, and
profile edits in the working tree. Treat those as unvalidated until committed
and tested in their own pass.

Robot-side revalidation did not reach the service layer:

| Target | Result |
| --- | --- |
| `192.168.66.13:22` | TCP opens, but no SSH banner before timeout |
| `192.168.66.13:5050` | TCP opens, raw HTTP GET sends, but no bytes are returned before timeout |
| `192.168.66.190:22` | TCP opens, but no SSH banner before timeout |
| `192.168.66.190:5050` | TCP opens, raw HTTP GET sends, but no bytes are returned before timeout |
| `77f7756a79211622.natapp.cc:53453` | SSH banner OK, but host is `bsrl-ESC8000-G4`, not the sunrise robot |
| `bsrl-ESC8000-G4 -> 192.168.66.13/190` | HTTP to Gateway times out |

Because the board service layer was unreachable, these items remain pending for
the next live pass:

| Acceptance group | Pending proof |
| --- | --- |
| Source and binary alignment | `/opt/lingtu/current/.lingtu_source_commit`, board `git rev-parse HEAD`, native binary hashes, and service restart timestamps match the pushed source being claimed |
| Native services | `lingtu-livox-dds`, `lingtu-slam-dds`, `lingtu-traversability-dds`, `lingtu-nav-dds`, and `lingtu` are active in the same boot session |
| SLAM and TF | `processed_scan_hz >= 10`, `has_odom=true`, `has_map_odom_tf=true`, valid `map -> odom`, `odom -> body`, `body -> lidar_link` samples |
| Map artifacts | active map has same-source `metadata.json`, `map.pcd`, `octomap.ot`, and `occupancy.npz`; OctoPlanner3D artifact gate is green |
| Native nav no-motion | nav session active, goal accepted, `global_path_points > 0`, `local_path_points > 0`, `last_local.reason=control_ready`, `cmd_vel_published=0` while `publish_cmd_vel=false` |
| DDS evidence | typed DDS endpoints are the only field writers for Livox/SLAM/traversability/nav; Python DDS nav adapters are absent from the field profile |
| Real motion | still not approved; requires a separate controlled pass with velocity output enabled |

## 2026-07-05 Connection Restored Test

`192.168.66.13` became reachable again after the earlier network-layer timeout.

| Check | Result |
| --- | --- |
| SSH `22` | reachable; banner `OpenSSH_8.9p1 Ubuntu-3ubuntu0.13` |
| Gateway `5050` | reachable; `/api/v1/health` returns `status=ok`, `map_points=8134`, `has_odom=true` |
| MCP `8090` | TCP reachable |
| Brainstem `13145` | TCP reachable |

## 2026-07-07 Local Audit Correction

This pass rechecked the repository state after the native DDS cleanup work. It
does not replace the historical no-motion field evidence above; it records the
current local code contract and the remaining live-board proof needed before
claiming another field closure.

Important correction:

- The historical July 4-5 no-motion runs used `publish_cmd_vel=false` and
  proved no velocity output.
- The current local Thunder service template now sets
  `LINGTU_NAV_PUBLISH_CMD_VEL=1` in
  `scripts/deploy/thunder/lingtu-nav-dds.service`.
- Therefore future field claims must read the live board's effective systemd
  environment and status snapshot before saying velocity output is disabled or
  enabled. The local template alone is not live-board proof.

Native nav endpoint ownership is now explicit in the status snapshot:

| Field | Meaning |
| --- | --- |
| `navigation_compute_owner` | `lingtu_nav_native_endpoint` owns field global planning, local planning, and path-following compute. |
| `local_path_role` | `/nav/local_path` is DDS telemetry/preview output for Gateway/UI/audit and optional external observers. |
| `path_follower_role` | PathFollower is embedded inside the C++ endpoint before the command gate. |
| `cmd_vel_role` | `/nav/cmd_vel` is the final navigation command output when `publish_cmd_vel=true`. |

This resolves the earlier ambiguity that `/nav/local_path` needed a separate
always-on consumer before motor output. In the field native path it does not: the
consumer is internal to `lingtu_nav_native_endpoint`, and the externally
observable proof is `last_local.cmd_vel` plus the `cmd_vel_published` counter.

Current code-level native field chain:

```text
Livox DDS
  -> Fast-LIO2 SLAM DDS
  -> C++ Traversability DDS
  -> C++ native nav endpoint
     -> OctoPlanner3D global path
     -> C++ LocalPlanner local path
     -> embedded PathFollower cmd_vel
     -> /nav/cmd_vel only when publish_cmd_vel=true
```

Live board revalidation was not completed in this local audit because SSH
password auth was not available to the Codex process. Required next live proof:

```text
systemctl cat lingtu-nav-dds.service
systemctl show lingtu-nav-dds.service -p Environment
cat /dev/shm/lingtu/nav_endpoint_status.json
```

The next acceptance note must include those three outputs, plus one no-motion
goal with `global_path_points > 0`, `local_path_points > 0`, and an explicit
`cmd_vel_published` count.
| SSH login | success as `sunrise`, host `ubuntu` |

Robot services were active:

| Service | State |
| --- | --- |
| `lingtu-livox-dds.service` | active |
| `lingtu-slam-dds.service` | active |
| `lingtu-traversability-dds.service` | active |
| `lingtu-nav-dds.service` | active |
| `lingtu.service` | active |

Source alignment is still not closed:

| Field | Observed |
| --- | --- |
| `/opt/lingtu/current/.lingtu_source_commit` | `54e6e89d` |
| board path | `/home/sunrise/data/inovxio/lingtu` |
| git metadata on board path | not available from this path |
| local pushed source previously claimed | `383be2c5` |

Active map artifacts are present and same-source:

| Artifact | Observed |
| --- | --- |
| active map | `/home/sunrise/data/nova/maps/accept_ready_20260702_162847` |
| `map.pcd` | `39861466` bytes, sha prefix `c88e849252ce8bd9` |
| `octomap.ot` | `172128` bytes, sha prefix `6b2cd15eba48bbfa` |
| `occupancy.npz` | `1350` bytes, sha prefix `4c527f773faf7fce` |
| metadata frame | `map` |
| metadata octomap source hash | matches `map.pcd` hash |

The active PCD map bounds are near the saved-map origin:

```text
x: [-8.68, 2.34]
y: [-13.76, 3.86]
z: [-0.99, 3.33]
```

However current SLAM odometry is far outside that map:

```text
odometry frame: odom -> body
odom pose: x ~= 4836.95, y ~= -199.89, z ~= -13.94
map -> odom: tx ~= -3.46, ty ~= -4.96, tz ~= 2.32
relocalization_state: failed
```

The native nav endpoint is alive and receives goal/cancel messages, but cannot
produce a route from this pose:

| Field | Observed |
| --- | --- |
| `has_odom` / `has_map_odom_tf` / `has_traversability` | true / true / true |
| `tick_hz` | `20` |
| `publish_cmd_vel` | false |
| no-motion test goal | `(0.8, 0.0, 0.001)` |
| endpoint goal counter | increased |
| `global_path_points` / `local_path_points` | `0 / 0` |
| last planning start | `[4154.89, -2482.61, 77.83]` |
| last planning goal | `[0.8, 0.0, 0.001]` |
| `cmd_vel_published` | `0` |

Relocalization control was tested without motion:

| Command | Result |
| --- | --- |
| `messages_control status` | reports map loaded and `relocalization_map_body` near origin, but `map_odom_tf` still does not reconcile the huge odom pose |
| `global-relocalize active/map.pcd` | failed: `native_relocalizer_icp_failed` |
| `track-against-map active/map.pcd` | starts, but status returns to `relocalization_state=failed` and failure count increases |

Conclusion: connectivity and service liveness are restored, and the endpoint DDS
boundary is receiving commands. Navigation acceptance is blocked by localization
alignment against the active map, not by Gateway, DDS, OctoPlanner3D process
launch, or missing map artifacts. Do not enable velocity output until
`map -> odom` makes the current body pose land inside the active map bounds and
the same no-motion goal produces non-empty global and local paths.

## 2026-07-05 MuJoCo Native DDS Motion Gate

This check used an isolated DDS domain on sunrise and did not publish to the
production robot domain.

Initial red artifact:

```text
artifacts/sunrise_mujoco_native_dds_slam_motion_gate_domain88_20260705_112017/report.json
```

The MuJoCo raw sensor bridge published `/imu/raw=572` and
`/lidar/raw_frame=115`. Native SLAM reached `TRACKING`, reported
`map_points=7296`, and produced `/slam/odometry`, `/slam/registered_cloud`,
`/slam/map_cloud`, and `/slam/localization_health`.

The new motion-consistency gate failed:

```text
motion.sim_xy_m = 1.132
motion.slam_odom_xy_m = 0.000047
remaining_gaps = native_slam_motion_mismatch:sim_xy=1.132,slam_xy=0.000,min_slam_xy=0.226
```

Conclusion: the realtime MuJoCo-to-DDS-to-SLAM data path is alive, and the
native SLAM runtime produces realtime map-frame point output. However the
map/odom/body motion transform is not validated for MuJoCo yet. Do not treat
stacked `/slam/map_cloud` snapshots as a complete cumulative map; use native
`save-map` for cumulative PCD artifacts, and accept them for navigation only
after this motion gate passes.

Follow-up passing artifact:

```text
artifacts/sunrise_mujoco_native_dds_slam_fullmap_domain103_20260705_131252/report.json
artifacts/sunrise_mujoco_native_dds_slam_fullmap_domain103_20260705_131252/native_saved_map.pcd
artifacts/sunrise_mujoco_native_dds_slam_fullmap_domain103_20260705_131252/native_saved_map_topdown.png
```

The fix path was:

- Publish a stationary warmup before driving so Fast-LIO initializes at rest.
- Change ZUPT detection to require both low IMU variance and a near-static mean.
- Add a MuJoCo-specific Fast-LIO config:
  `src/localization/fastlio2/config/sim_mid360_slam.yaml`.
- Use the native DDS sensor bridge `imu_acc_axis_scale=auto`, which resolves to
  `[-0.43, 1.0, 1.0]` only for kinematic finite-difference MuJoCo IMU samples.
- Use physical rolling scan timing and a 5 second kinematic command ramp.

Passing evidence:

```text
published: /imu/raw=1236, /lidar/raw_frame=264
observed_slam_outputs: /slam/odometry, /slam/registered_cloud, /slam/map_cloud, /slam/localization_health
state: TRACKING
map_points: 6168
motion.sim_xy_m: 2.116
motion.slam_odom_xy_m: 2.444
motion.slam_to_sim_xy_ratio: 1.155
gate range: [0.5, 1.6]
remaining_gaps: []
native saved map: native_saved_map.pcd, 24312 finite points, 778233 bytes
```

Conclusion: the MuJoCo raw LiDAR/IMU -> native DDS publisher -> native Fast-LIO2
runtime -> odometry/map chain is closed on sunrise for the isolated simulation
gate. This is still simulation-only evidence; it does not publish to the real
robot domain and does not authorize hardware velocity output by itself.

### Saved-map preview diagnosis

The saved-map screenshots in this artifact are diagnostic plots, not product
navigation maps.

Files:

```text
artifacts/sunrise_mujoco_native_dds_slam_fullmap_domain103_20260705_131252/native_saved_map.pcd
artifacts/sunrise_mujoco_native_dds_slam_fullmap_domain103_20260705_131252/native_saved_map_topdown.png
artifacts/sunrise_mujoco_native_dds_slam_fullmap_domain103_20260705_131252/native_saved_map_occupancy_preview.png
artifacts/sunrise_mujoco_native_dds_slam_fullmap_domain103_20260705_131252/native_saved_map_layer_diagnostics.json
```

The first filtered preview was generated by projecting the saved PCD to a
0.2 m XY grid, then keeping cells with at least 3 points, z span >= 0.35 m,
and z max >= 0.25 m. That rule is intentionally only a quick inspection view.
It is not a costmap builder because it does not raycast free space, remove
ground, separate wall-height points from overhead hits, or apply persistence
logic.

Layer diagnostics on the same PCD show why the mixed preview looks noisy:

```text
finite_points: 24312
xy_cells_res_0p2: 6949
ground_like_points_-0.35_to_0.25m: 3343
obstacle_band_points_0.30_to_1.60m: 5831
high_points_above_1.60m: 12989
previous_preview_kept_cells: 1952
previous_preview_sparse_kept_cells_count_lt_5: 750
obstacle_band_cells_obs_count_ge_2: 1111
```

Interpretation:

- The motion gate passed, so this is no longer the old zero-odometry failure.
- The raw saved PCD still contains floor-adjacent points, full-height wall
  returns, overhead/roof points, and sparse far hits from the industrial park
  scene.
- A direct top-down plot or the simple z-span preview can make the map look like
  a TF failure because it collapses all height layers into one XY plane.
- The obstacle-height slice is more structured, but it still shows wall
  thickening and sparse outliers; this saved PCD should not be used directly as
  the local planner map.

Product navigation evidence must be generated from the map-layer pipeline:
saved PCD or live `/slam/map_cloud` -> height/ground filtering -> raycast
free-space occupancy -> inflation/traversability cost -> local planner/global
planner input. A raw PCD scatter or quick filtered preview is insufficient for
planning acceptance.

### Follow-up 3D and motion isolation

The first domain103 map was further checked with 3D layer plots and a MuJoCo
scene-footprint overlay:

```text
artifacts/sunrise_mujoco_native_dds_slam_fullmap_domain103_20260705_131252/native_saved_map_3d_layers_interactive.html
artifacts/sunrise_mujoco_native_dds_slam_fullmap_domain103_20260705_131252/native_saved_map_3d_layers_iso.png
artifacts/sunrise_mujoco_native_dds_slam_fullmap_domain103_20260705_131252/native_saved_map_scene_overlay.png
```

The 3D view shows that high/overhead points are real in the saved PCD, but the
main planning problem is not only high-point projection. After height and
density filtering, only 54.9% of candidate obstacle cells land within 0.4 m of
the expected MuJoCo obstacle footprint, and 45.1% are farther than 0.6 m from
expected geometry. The saved map therefore has real thickening/ghosting, not
just a bad 2D screenshot.

Additional isolated sunrise runs:

| Run | Change | Motion result | Map result |
| --- | --- | --- | --- |
| domain104 | straight, default MuJoCo config | fail: sim 2.233 m, SLAM 0.003 m | overlay clean because map is effectively near-static: 99.4% within 0.4 m |
| domain105 | straight, `imu_acc_axis_scale=1,1,1` | fail: sim 2.234 m, SLAM 0.005 m | same failure class as domain104 |
| domain106 | gentle arc `drive_wz=0.01` | fail: sim 2.257 m, SLAM 0.022 m | same low-motion failure class |
| domain107 | straight, ZUPT disabled by setting `zupt_min_static_frames=999999` | pass: sim 2.255 m, SLAM 1.173 m, ratio 0.520 | overlay poor: 38.0% within 0.4 m, 62.0% farther than 0.6 m |
| domain108 | original arc, ZUPT disabled | pass under loose `[0.2,3.0]`: sim 2.105 m, SLAM 3.710 m, ratio 1.763 | overshoots the stricter previous max ratio 1.6 |

Interpretation:

- Default MuJoCo config still false-detects low-acceleration straight and
  gentle-arc motion as static; ZUPT clamps velocity.
- Disabling ZUPT restores odometry movement, proving the false-static diagnosis,
  but it also exposes drift/scale error and does not produce an acceptable map.
- A motion-consistency gate alone is insufficient. The MuJoCo native DDS SLAM
  gate also needs a map-quality gate for known worlds: saved PCD -> filtered
  obstacle cells -> distance to expected scene footprint, plus a strict motion
  ratio.
- Current status is not "map ready for localplanner". The raw LiDAR/IMU DDS
  chain is alive, but MuJoCo native Fast-LIO mapping remains under tuning.

### Saved-map quality gate added

The saved-map quality gate is now codified as:

```bash
python3 sim/scripts/mujoco/saved_map_quality_gate.py \
  --pcd artifacts/<run>/native_saved_map.pcd \
  --world industrial_park \
  --json-out artifacts/<run>/native_saved_map_quality_gate.json \
  --plot-out artifacts/<run>/native_saved_map_quality_gate.png
```

It reads the native saved PCD directly, keeps obstacle-height cells
(`0.30 <= z <= 1.60`), removes sparse cells and small isolated components, and
reports both raw scene-frame overlap and bounded 2D aligned overlap against the
expected MuJoCo obstacle footprint.

Local replay over the sunrise artifacts:

| Run | Quality result | Key metric |
| --- | --- | --- |
| domain103 | aligned pass | raw near `66.97%`, raw far `30.49%`; aligned near `86.65%`, aligned far `2.99%`, yaw correction `-8.0 deg` |
| domain104 | aligned pass | aligned near `100.0%`, aligned far `0.00%` |
| domain107 | fail | aligned near `49.63%`, aligned far `43.52%` |

This closes the previous blind spot: a run can no longer be accepted only
because SLAM odometry moved. The map itself must also pass the aligned
geometry-quality gate before it is considered usable for map-layer acceptance.

### Domain111 yaw-gated rerun

After adding yaw consistency to `mujoco_native_dds_sensors.py`, sunrise domain111
was run with the colocated MuJoCo extrinsic (`t_il=[0,0,0]`) used by
`industrial_park_scene.xml`'s `robot_placeholder` fallback:

```text
artifacts/sunrise_mujoco_native_dds_yawgate_domain111_20260705_205310/summary.json
```

Result:

- `motion_ok=false`
- `quality_ok=true`
- XY motion ratio: `1.152`, within `[0.5, 1.6]`
- MuJoCo yaw delta: `0.870 rad`
- SLAM odom yaw: `1.092 rad`
- yaw error: `0.222 rad`, above `0.150 rad`
- aligned map quality: near `86.54%`, far `2.55%`

Conclusion: the current MuJoCo native DDS saved map is no longer best described
as a local point-cloud ghosting failure after alignment. The remaining blocker
is yaw/TF consistency: SLAM over-rotates relative to MuJoCo, which makes raw
scene-frame top-down overlays look thick or shifted.

Negative tuning checks:

| Run | Change | Result |
| --- | --- | --- |
| domain112 | `lidar_cov_inv=100` | yaw error improved to `0.138 rad`, but XY ratio exploded to `4.156` and aligned map quality failed |
| domain113 | `--imu-gyro-axis-scale 1,1,0.8` | yaw error improved to `0.016 rad`, but XY ratio exploded to `4.163` and aligned map quality failed |

Do not use either change as a product fix. The default MuJoCo native DDS path
keeps `lidar_cov_inv=1000` and gyro scale `1,1,1`; the new yaw gate remains the
acceptance blocker until a tuning change improves yaw without breaking XY and
map shape.

### Domain114-119 timing and IMU isolation

The next sunrise pass tested whether the apparent wall thickening was caused by
scan timing, timestamp time base, or simple IMU scale tuning.

The bridge now exposes `--timestamp-clock wall|sim` and records
`timestamp_clock` in the report. The default remains `wall` because it is the
only tested path that preserves the domain111 saved-map shape; `sim` is kept as
a diagnostic mode, not an accepted fix.

| Run | Change | Motion result | Map result | Decision |
| --- | --- | --- | --- | --- |
| domain114 | `scan_time_profile=instantaneous`, wall clock | yaw error improved to `-0.066 rad`, but XY ratio `1.650` exceeds `1.6` | aligned quality failed: near `56.43%`, far `28.87%` | reject |
| domain115 | `scan_time_profile=synthetic_rolling`, wall clock | XY ratio exploded to `9.957` | aligned quality failed: near `31.63%`, far `59.10%` | reject |
| domain116 | `physical_rolling`, `timestamp_clock=sim`, finite-difference IMU | yaw error improved to `-0.013 rad`, but XY ratio exploded to `5.452` | aligned quality failed: near `15.55%`, far `80.31%` | reject |
| domain117 | `timestamp_clock=sim`, `imu_acc_mode=gravity_only` | yaw error improved to `-0.009 rad`, but XY ratio remained high at `3.468` | aligned quality failed: near `22.62%`, far `70.75%` | reject |
| domain118 | domain117 plus stricter temporary Fast-LIO update limits | XY ratio worsened to `8.010` | not accepted after motion gate failure | reject |
| domain119 | wall clock plus `imu_gyro_axis_scale=1,1,0.9` | yaw entered threshold at `0.123 rad`, but XY exploded to `97.5` and odom left bounds | quality run stopped after motion failure | reject |

Trajectory replay confirmed the yaw and XY gates are not just comparing against
an arbitrary nonzero initial SLAM pose: the saved trajectories start at
`(0,0,0 yaw)` for these isolated mapping runs. Domain111's yaw error is a real
SLAM delta mismatch, and domain116/domain117/domain118 are real translation
overshoot failures.

### Domain120-123 split-clock closure

This sunrise pass isolated the temporary mixed timestamp problem. A single
legacy clock for both simulated raw IMU and LiDAR records could improve one
metric while breaking another:

- `wall` for both sensors preserved the best XY/map shape, but SLAM yaw
  over-rotated.
- `sim` for both sensors fixed yaw, but translation direction/scale overshot
  badly.

The accepted fix keeps LiDAR scan timestamps on wall time, uses simulated time
for IMU records, and keeps the current MuJoCo gate extrinsic colocated
(`t_il=[0,0,0]`) until the bridge publishes lever-arm-consistent samples.

| Run | Change | Motion result | Map result | Decision |
| --- | --- | --- | --- | --- |
| domain120 | physical `t_il=[0,0,0.28]`, wall clock, acc auto | fail: sim `2.113 m`, SLAM `9.648 m`, yaw error `0.204 rad` | quality failed | reject |
| domain121 | `t_il=[0,0,0]`, single `timestamp_clock=sim`, acc `1,1,1` | fail: sim `2.108 m`, SLAM `4.826 m`, XY ratio `2.290`; yaw error `-0.004 rad` | quality failed: near `26.6%`, far `68.5%` | reject |
| domain122 | `t_il=[0,0,0]`, single `timestamp_clock=wall`, acc `1,1,1` | fail: sim `2.109 m`, SLAM `6.389 m`, yaw error `0.225 rad` | quality failed: near `21.0%`, far `74.8%` | reject |
| domain123 | `t_il=[0,0,0]`, LiDAR timestamp `wall`, IMU timestamp `sim`, acc auto | pass: sim `2.103 m`, SLAM `1.938 m`, XY ratio `0.921`, yaw error `-0.002 rad` | pass: aligned near `95.29%`, far `0.46%` | accept |

Accepted sunrise artifact:

```text
artifacts/sunrise_mujoco_native_dds_hybrid_imu_sim_lidar_wall_til0_auto_domain123_20260705_220202/summary.json
```

Domain123 evidence:

- Native sensor bridge `ok=true`.
- Native saved-map quality gate `ok=true`.
- `bridge_rc=0`, `save_rc=0`, `quality_rc=0`.
- DDS inputs: `/imu/raw=1231`, `/lidar/raw_frame=268`.
- Native SLAM outputs: `/slam/odometry`, `/slam/registered_cloud`,
  `/slam/map_cloud`, `/slam/localization_health`, and
  `/slam/localization_quality`.
- SLAM reached `state=TRACKING`.
- Live map reported `map_points=7346`.
- Saved PCD contained `21051` points.
- Additional 3D layer preview:
  `native_saved_map_3d_layers.png`. It shows the saved PCD has many high or
  overhead returns (`11565` points above `1.60 m`), while the obstacle-height
  layer used by the quality gate contains `4661` points. Do not judge planner
  readiness from a raw all-height top-down projection.

Current accepted and rejected boundaries:

- Accepted evidence: MuJoCo raw LiDAR/IMU -> native DDS publisher -> native
  Fast-LIO2 runtime -> odometry -> map cloud -> native save-map -> saved-map
  quality gate is closed on sunrise for the dense industrial scene.
- Accepted configuration: physical rolling scan, LiDAR timestamp `wall`, IMU
  timestamp `sim`, auto MuJoCo finite-difference acceleration compensation,
  gyro scale `1,1,1`, and `sim_mid360_slam.yaml` with
  `t_il=[0,0,0]`.
- Rejected fixes: physical `t_il=0.28` without lever-arm-consistent samples,
  a single `sim` timestamp clock, a single identity acceleration scale,
  disabling rolling timing, lowering LiDAR covariance, and scaling gyro Z.
  Each improved one metric while breaking XY motion or saved-map quality.

This solved the immediate sunrise MuJoCo native SLAM closure, but it still used
a split timestamp workaround. That workaround has been replaced by the unified
simulated hardware clock below.

### Domain124-131 simulated hardware clock closure

The MuJoCo DDS bridge now has a product clock profile:
`--timestamp-clock sim_hardware`. In this mode LiDAR scan timing, IMU sampling,
and publication pacing all derive from one simulated hardware clock. The legacy
split controls remain available only for diagnostics.

The first `sim_hardware` run exposed why the old auto IMU acceleration
compensation could not be reused. The old `auto` value `[-0.43,1,1]` was tuned
under the split-clock workaround. Under a true simulated hardware clock it
over-estimated long-run motion.

| Run | Change | Motion result | Map result | Decision |
| --- | --- | --- | --- | --- |
| domain124 | `sim_hardware`, 30.0 s, old auto `[-0.43,1,1]` | pass motion gate: XY ratio `1.451`, yaw error `-0.004 rad` | fail: aligned near `75.56%` | reject for map quality |
| domain125 | `sim_hardware`, 24.2 s, old auto `[-0.43,1,1]` | fail: XY ratio `1.674`, yaw error `-0.005 rad` | fail: aligned near `69.20%`, far `21.64%` | reject |
| domain130 | `sim_hardware`, 24.2 s, explicit `[-0.20,1,1]` | pass: XY ratio `0.614`, yaw error `-0.004 rad` | pass: aligned near `89.41%`, far `1.10%` | accept candidate |
| domain131 | `sim_hardware`, 24.2 s, `auto` -> `[-0.20,1,1]` | pass: XY ratio `0.606`, yaw error `-0.008 rad` | pass: aligned near `89.41%`, far `1.10%` | accept |

Accepted sunrise artifact:

```text
artifacts/sunrise_mujoco_native_dds_sim_hardware_clock_duration24p2_auto_domain131_20260706_004529/summary.json
```

Domain131 evidence:

- Native sensor bridge `ok=true`.
- Native saved-map quality gate `ok=true`.
- `bridge_rc=0`, `save_rc=0`, `quality_rc=0`.
- `clock_profile=sim_hardware`.
- `timestamp_clock=sim_hardware`.
- `imu_timestamp_clock=sim_hardware`.
- `lidar_timestamp_clock=sim_hardware`.
- `imu_acc_axis_scale_source=auto_kinematic_sim_hardware`.
- `imu_acc_axis_scale=[-0.2,1.0,1.0]`.
- DDS inputs: `/imu/raw=1261`, `/lidar/raw_frame=253`.
- Native SLAM outputs: `/slam/odometry`, `/slam/registered_cloud`,
  `/slam/map_cloud`, `/slam/localization_health`, and
  `/slam/localization_quality`.
- SLAM reached `state=TRACKING`.
- MuJoCo motion: `2.103 m`, yaw `0.868 rad`.
- SLAM odom motion: `1.274 m`, yaw `0.860 rad`.
- Saved PCD contained `19103` points.
- Saved-map quality gate: aligned near `89.41%`, far `1.10%`.
- Aligned overlay correction: yaw `0.0 deg`, translation `(0.40,0.40) m`.

Current default MuJoCo native DDS contract:

- Use `--timestamp-clock sim_hardware`.
- Leave `--imu-timestamp-clock` and `--lidar-timestamp-clock` empty unless
  intentionally running a legacy diagnostic.
- Use `--imu-acc-axis-scale auto`; under `sim_hardware` it resolves to
  `[-0.20,1.0,1.0]`.
- The old split-clock path remains useful only as a regression comparison.

Real robot note: the physical MID-360/IMU stack does not have MuJoCo
`sim_time` versus process `wall_time`. It can still fail from real timestamp
issues, but those are different: driver/device clock skew, incorrect receive
time stamping, inaccurate `time_diff_lidar_to_imu`, or extrinsic calibration
error. Do not copy the MuJoCo `sim_hardware` acceleration compensation into
`config/robots/doso/thunder_v4/sensors/mid360_fastlio2.yaml`.

### Domain132-134 long-trajectory map quality regression

After domain131 accepted the short local closure, sunrise long-trajectory runs
were added to test whether the same MuJoCo native DDS stack can build a usable
larger map. These runs do not pass the product map-quality bar yet.

| Run | Change | Motion result | Map result | Decision |
| --- | --- | --- | --- | --- |
| domain132 | `sim_hardware`, 60.0 s, `box_explore`, `auto` -> `[-0.20,1,1]` | fail: MuJoCo path length `4.381 m`, final XY `0.782 m`; SLAM XY `0.094 m`; yaw error `-0.038 rad` | saved PCD `16001` points; aligned local obstacle gate passed, but coverage stayed near the start area | reject for SLAM translation under-tracking |
| domain133 | `sim_hardware`, 60.0 s, continuous arc, `auto` -> `[-0.20,1,1]` | fail: MuJoCo path length `7.079 m`, final XY `5.547 m`; SLAM XY `114.039 m`; yaw error `-0.005 rad` | saved PCD `460001` points; obstacle slice stretched to roughly `120 m`, quality gate was stopped because the PCD was already visibly invalid and slow to score | reject for SLAM translation over-shoot |
| domain134 | `sim_hardware`, 30.0 s, continuous arc, `auto` -> `[-0.20,1,1]` | fail: MuJoCo path length `3.481 m`, final XY `3.289 m`; SLAM XY `16.834 m`; yaw error `-0.005 rad` | saved PCD `77436` points; obstacle slice already stretched to roughly `30 m` | reject for 30 s translation scale drift |

Long-trajectory artifacts:

```text
artifacts/sunrise_mujoco_native_dds_sim_hardware_box_explore_60s_auto_domain132_20260706_011441/
artifacts/sunrise_mujoco_native_dds_sim_hardware_arc_60s_auto_domain133_20260706_012052/
artifacts/sunrise_mujoco_native_dds_sim_hardware_arc_30s_auto_domain134_20260706_013410/
```

Interpretation:

- The unified simulated hardware clock is not the remaining long-run blocker.
  All long runs used `clock_profile=sim_hardware` and produced synchronized
  LiDAR/IMU DDS input.
- Yaw is consistently good on the long runs. The failures are translation
  scale failures: domain132 under-tracks XY while domain133/domain134
  over-shoot XY.
- The raw saved PCD is not a planner-ready map. Costmap/obstacle layers must
  slice and filter height bands, but filtering cannot fix a wrong SLAM
  trajectory.
- The accepted domain131 result should be treated as short local SLAM closure
  only. Long-run MuJoCo mapping needs a separate fix in the native SLAM
  simulation input contract, likely around the kinematic IMU motion model,
  acceleration compensation, and/or Fast-LIO velocity constraints.

### Domain160-184 long-trajectory diagnosis

The domain133/domain134 stretched-map screenshots were caused by translation
scale drift, not by the plotting script. The key failure was MuJoCo kinematic
drive acceleration being fed to Fast-LIO as if it were physical IMU acceleration.
That finite-difference X acceleration is a simulator control artifact. Under
the unified `sim_hardware` clock `--imu-acc-axis-scale auto` suppresses it:
`[0.0,1.0,1.0]`.

The later runs showed that the previous domain178 "pass" was only a
motion-gate false positive. It used `physical_rolling` with
`physical_rolling_sample_mode=full_frame`, which repeats broad full-frame
raycasts inside one scan window. That can make the odometry ratio look
acceptable while still producing a thick, ghosted saved map. It must not be
used as saved-map acceptance evidence.

Parameter isolation on sunrise:

| Run | Change | Motion result | Decision |
| --- | --- | --- | --- |
| domain160 | previous auto acceleration scale | fail: XY ratio `5.118` | reject |
| domain163 | explicit `--imu-acc-axis-scale 0,1,1` | pass candidate: XY ratio `1.485`, yaw error `-0.005 rad` | accept candidate |
| domain169 | new physical subscan budget, base extrinsic | fail: XY ratio `5.818` | reject |
| domain170 | physical subscan budget plus yaw-180 `r_il` | fail: XY ratio `6.207` | reject |
| domain171 | `synthetic_rolling` timing | fail: XY ratio `35.522` | reject |
| domain174 | yaw-180 `r_il`, full-frame rolling | fail: XY ratio `2.807` | reject |
| domain175 | physical `t_il=[0,0,0.28]` | fail: XY ratio `6.393` | reject |
| domain176 | full-frame rolling, `0,1,1` acc scale, tight velocity constraints | pass: XY ratio `1.485`, yaw error `-0.005 rad` | accept candidate |
| domain178 | full-frame rolling repository default | motion-only pass: XY ratio `1.500`, yaw error `-0.001 rad`; saved-map quality fail | reject |
| domain181 | instantaneous scan offsets, `0,1,1` acc scale | fail: XY ratio `0.269`; map visually cleaner but under-tracked | reject |
| domain182 | physical subscan timing, `0,1,1` acc scale | fail: XY ratio `0.268`; realistic timing but under-tracked | reject |
| domain183 | physical subscan, `1,1,1` acc scale | fail: XY ratio `8.626` | reject |
| domain184 | physical subscan, `0.08,1,1` acc scale | fail: XY ratio `5.788` | reject |

Rejected but useful sunrise artifact:

```text
artifacts/sunrise_mujoco_native_dds_default_fixed_saved_map_30s_domain178_20260706_0415/
```

Domain178 evidence:

- Native sensor bridge `ok=true`.
- `bridge_rc=0`, `save_rc=0`.
- `clock_profile=sim_hardware`.
- `scan_time_profile=physical_rolling`.
- `physical_rolling_sample_mode=full_frame`.
- `imu_acc_axis_scale_source=auto_kinematic_sim_hardware`.
- `imu_acc_axis_scale=[0.0,1.0,1.0]`.
- MuJoCo motion: `3.289 m`, yaw `1.160 rad`.
- SLAM odom motion: `4.934 m`, yaw `1.159 rad`.
- SLAM/MuJoCo XY ratio: `1.500`.
- SLAM yaw error: `-0.001 rad`.
- Saved PCD exists and contains `53619` points.
- Obstacle-height slice contains `15253` points.
- Filtered 0.2 m occupancy preview contains `2255` cells.
- Saved-map XY span is about `26.4 m x 28.5 m`, not the invalid `120 m`
  fan seen in domain133.
- Saved-map quality gate fails:
  `near_ratio=0.296 < 0.800`, `far_ratio=0.627 > 0.150`.

Follow-up artifacts:

```text
artifacts/sunrise_mujoco_native_dds_instantaneous_30s_domain181_20260706_041600/
artifacts/sunrise_mujoco_native_dds_subscan_30s_domain182_20260706_041950/
artifacts/sunrise_mujoco_native_dds_subscan_scale1_30s_domain183_20260706_042255/
artifacts/sunrise_mujoco_native_dds_subscan_scale008_30s_domain184_20260706_042438/
```

Raw MuJoCo LiDAR control diagnosis:

- A single raw raycast obstacle slice is structured, not random:
  `6097` obstacle-height points.
- MuJoCo ground-truth pose accumulation of the same trajectory is structured:
  `39426` obstacle-height points and `239683` voxelized 3D points.
- Therefore the primary failure is not the MuJoCo raycast hit generation. The
  failure is the contract between simulated rolling-scan timing, simulated IMU,
  and Fast-LIO motion estimation.

MuJoCo odom-prior diagnostic map artifact run:

```text
/tmp/lingtu_mujoco_odom_prior_gate_20260706_061031/
```

Key sunrise results:

- Native C++ `livox_sdk2_stream` and `messages_cyclone_runtime` rebuilt
  successfully on sunrise.
- Focused runtime tests passed on sunrise:
  `8 passed, 150 deselected` before the motion-gate fix and
  `4 passed, 155 deselected` for the odom-prior/motion-report subset.
- Gate command used DDS domain `212`, MuJoCo `kinematic` drive,
  `box_explore` profile, `sim_hardware` clock, `physical_rolling` scan timing,
  and explicit `--publish-odom-prior`.
- Sensor publication counts:
  `/imu/raw=1001`, `/lidar/raw_frame=201`, `/slam/odom_prior=1001`.
- Native SLAM status:
  `state=TRACKING`, `has_odom=true`, `odom_prior_enabled=true`,
  `odom_prior_active=true`, `odom_prior_age_s=0.0`,
  `odom_prior_error_xy_m=0.012645`.
- Motion consistency passed:
  `sim_xy_m=1.250050584`, `slam_odom_xy_m=1.233897598`,
  `slam_to_sim_xy_ratio=0.987078`, yaw error about `3.8e-7 rad`.
- Native gate result:
  `report_ok=true`, `remaining_gaps=[]`, `gate_rc=0`.
- Native save-map command succeeded:
  `save_rc=0`, `message=map_saved`.
- Saved map artifact:
  `/tmp/lingtu_mujoco_odom_prior_gate_20260706_061031/saved_map/map.pcd`,
  `36417` voxelized points, `1165593` bytes.
- Local preview copies:
  `artifacts/mujoco_odom_prior_gate_20260706_061031/saved_map_topdown.png`
  and
  `artifacts/mujoco_odom_prior_gate_20260706_061031/saved_map_3d.png`.

Interpretation:

- This is not accepted product mapping evidence. It proves only that MuJoCo
  raycast XYZI hits can be accumulated into native C++ map artifacts when a
  synchronized ground-truth pose prior is injected.
- The odom prior is simulation-only diagnostic input. It must not hide
  Fast-LIO-only motion-estimation errors, and it must not be used as the
  default MuJoCo acceptance path.
- Real S100P/MID-360 configs must keep `odom_prior_enabled=false`; real maps
  continue to depend on calibrated hardware timestamps, LiDAR-IMU extrinsics,
  and Fast-LIO/localizer behavior.

Current default MuJoCo native DDS contract, superseding the domain131 notes:

- Use `--timestamp-clock sim_hardware`.
- Leave split clock overrides empty unless intentionally running a diagnostic.
- Use `--settle-s 3.0` before any sensor publication, then `--warmup-s 2.0`
  while publishing stationary LiDAR/IMU before policy drive. This keeps
  Fast-LIO initialization out of the MuJoCo model drop/stand-up transient.
- Use `--drive-mode policy` for Fast-LIO saved-map evidence. The policy/physics
  path is the only MuJoCo mode that gives the bridge hardware-like LiDAR/IMU
  motion. `--drive-mode kinematic` remains useful for navigation smoke tests and
  GT-map diagnostics, but it is no longer accepted as Fast-LIO saved-map proof
  unless `--allow-kinematic-fastlio-acceptance` is explicitly set.
- Use `--imu-acc-mode sensor` with
  `--imu-acc-conditioning realistic`. The MuJoCo bridge stays on the normal
  Fast-LIO route: LiDAR raw frame plus IMU gyro/accelerometer, no odom prior.
  The conditioning models finite IMU/driver bandwidth by applying low-pass,
  dynamic-acceleration clipping, and slew limiting before publishing raw IMU.
  `gravity_only` is retained only as a diagnostic baseline.
- Use `--imu-acc-axis-scale auto`; in policy sensor mode this resolves to
  identity `[1.0,1.0,1.0]`. Kinematic finite-difference compensation is a
  diagnostic-only path and is not accepted as product Fast-LIO evidence.
- Keep `--scan-time-profile physical_rolling`.
- Use `--physical-rolling-sample-mode subscan` for physical correctness.
  `full_frame` is a legacy diagnostic compatibility mode only and must fail
  saved-map acceptance until the simulated IMU/Fast-LIO motion model is fixed.
- Keep MuJoCo Fast-LIO extrinsic colocated for this bridge contract:
  `t_il=[0,0,0]`, `r_il=I`. The ThunderV4 MJCF now exposes
  `lidar-orientation`, `lidar-angular-velocity`, and
  `lidar-linear-acceleration` at `lidar_site`, and the engine prefers those
  sensors for the raw SLAM package.
- Use MuJoCo-specific Fast-LIO update limits:
  `max_update_velocity_mps=3.0`,
  `max_update_velocity_delta_mps=1.0`,
  `max_update_translation_m=0.5`,
  `reject_nonconverged_update=false`, and
  `reject_degenerate_nonconverged_update=true`. This lets the LiDAR update
  correct sensor-IMU prediction error while still rejecting truly degenerate
  non-converged updates.
- Keep `/slam/odom_prior` disabled by default. Enable it only for an explicitly
  labelled diagnostic run that checks raycast/map-layer plumbing, not Fast-LIO
  mapping quality.

2026-07-06 sunrise correction:

- Root cause of the post-odom-prior map smear was not missing LiDAR
  constraints. It was the combination of MuJoCo contact/policy accelerometer
  dynamics and a MuJoCo-specific Fast-LIO config that rejected every
  non-converged LiDAR correction. With sensor acceleration enabled under the
  old config, Fast-LIO reverted to IMU prediction and produced large false XY
  velocity. `gravity_only` passed only because it removed the dynamic
  acceleration that exposed the rejected-update path.
- The native DDS bridge default is now back on the real Fast-LIO route:
  `imu_acc_mode=sensor`, `imu_acc_conditioning=realistic`, `publish_odom_prior=false`.
  Real S100P hardware configs are unchanged.
- Short sensor-conditioned run on sunrise:
  `/home/sunrise/data/inovxio/lingtu/artifacts/sunrise_mujoco_fastlio_sensor_fixed_20260706`.
  Fast-LIO-only gate passed with no odom prior:
  `sim_xy_m=0.527`, `slam_odom_xy_m=0.497`, ratio `0.944`, yaw error
  `-0.003 rad`. The status snapshot reported `fastlio_degeneracy.converged=true`
  and final velocity near zero (`x=-0.0045`, `y=0.0275`, `z=-0.0754 m/s`).
- 30 s policy `box_explore` sensor-conditioned run on sunrise:
  `/home/sunrise/data/inovxio/lingtu/artifacts/sunrise_mujoco_fastlio_sensor_fixed_30s_20260706`.
  Fast-LIO-only gate passed with `sim_xy_m=1.407`, `slam_odom_xy_m=1.381`,
  ratio `0.982`, yaw error `-0.0006 rad`. Native save-map succeeded, and the
  saved-map quality gate passed with aligned near ratio `99.65%`, far-ghost
  ratio `0.0%`, median nearest obstacle distance `0.077 m`, and p90 distance
  `0.176 m`.
- 240 s policy `box_explore` long-run on sunrise:
  `/home/sunrise/data/inovxio/lingtu/artifacts/sunrise_mujoco_fastlio_sensor_fixed_240s_20260706`.
  This run is red and must not be used as full-map closure evidence. It used
  the same Fast-LIO-only route (`imu_acc_mode=sensor`,
  `imu_acc_conditioning=realistic`, `publish_odom_prior=false`) and saved a
  native map successfully, but the motion gate failed:
  `sim_path_length_xy_m=15.257`, `sim_xy_m=1.175`,
  `slam_odom_xy_m=2.635`, ratio `2.243`, and yaw error `-0.263 rad`.
  The saved-map quality gate also failed with aligned near ratio `45.73%`,
  far-ghost ratio `40.78%`, median nearest obstacle distance `0.463 m`, and
  p90 distance `1.262 m`. The map contains 300 saved patches and `65020`
  saved-map points, so the failure is long-trajectory geometric consistency,
  not missing LiDAR data or failed map serialization.

Important remaining interpretation:

- MuJoCo odom-prior maps remain diagnostic only. Accepted MuJoCo mapping
  evidence must be Fast-LIO-only: no odom prior, unified simulated hardware
  timestamps, physical rolling subscan timing, 200 Hz IMU target,
  sensor-conditioned MuJoCo accelerometer, Livox-compatible `tag/line`, and a
  saved obstacle layer that passes the map-quality gate.
- The current 30 s `box_explore` quality margin is strong for the tested
  industrial scene. It is not full-map proof. The 240 s `box_explore` run
  shows that pure Fast-LIO odometry accumulation still drifts on long loop-like
  trajectories; full-scene mapping acceptance needs loop closure/PGO or an
  equivalent global correction in the native map-save path.
- `native_saved_map.pcd` is a raw 3D SLAM map. It contains ground, obstacle,
  ceiling/overhead, and sparse returns. It should not be judged or used as a
  2D obstacle map by direct top-down projection.
- Planner/costmap consumers must use a filtered obstacle layer: height window,
  per-cell hit count, connected-component filtering, and optional scene/map
  alignment for validation plots.
- Native Fast-LIO currently ignores IMU quaternion for initial yaw; it uses
  acc/gyro only. Therefore the SLAM map frame can have an arbitrary global yaw
  relative to MuJoCo world. This is normal for LiDAR-inertial SLAM without an
  absolute heading source, but validation overlays must align frames before
  comparing against scene truth.
- Real S100P/MID-360 should not copy the MuJoCo acceleration conditioning.
  Real issues of the same visual kind usually come from timestamp skew,
  incorrect `time_diff_lidar_to_imu`, wrong LiDAR-IMU extrinsic calibration, or
  degenerate mapping geometry.

## 2026-07-06 Continuous Mapping Quality Gate

The short-window sensor-conditioned closure above is necessary but not
sufficient. A dedicated 3–5 minute gate now checks bridge continuity, scale
convergence against MuJoCo ground truth (`sim_motion.jsonl` vs saved
`trajectory.txt`), and saved-map quality in one isolated session.

Gate script:

```bash
python3 sim/scripts/mujoco/continuous_mapping_quality_gate.py \
  --duration 180 \
  --domain-id 231 \
  --drive-profile box_explore
```

Sunrise runner:

```bash
python sim/scripts/run_sunrise_continuous_mapping_gate.py \
  --host 192.168.66.13 \
  --duration 180 \
  --domain-id 231
```

First 180 s result on domain **232** (`box_explore`, policy sensor-conditioned
bridge):

| Group | Result | Notes |
| --- | --- | --- |
| Bridge | pass | endpoint ratio `1.19`, yaw error `0.024 rad` |
| Continuity tracking | pass | zero drops/rollbacks, `TRACKING` throughout |
| Continuity rates | fail (first run) | status under-reported Hz under CPU backlog; gate now also checks bridge published counts |
| Convergence | fail | cumulative path ratio **5.04**; max window ratio **8.17** |
| ATE (rigid 2D) | pass | RMSE `0.12 m`, max `0.45 m` |
| Map quality | pass | aligned near **98.6%** |

Artifact:

```text
artifacts/sunrise_mujoco_continuous_mapping_gate_20260706_231109/
```

Verdict matrix and follow-up actions:
[2026-07-06-mujoco-continuous-mapping-gate.md](./2026-07-06-mujoco-continuous-mapping-gate.md).

Current status:

- MuJoCo raw LiDAR/IMU -> native DDS -> Fast-LIO2 transport is closed.
- 30 s local mapping can pass endpoint ratio and map-quality gates.
- 180–240 s loop-like `box_explore` trajectories still fail continuous scale
  convergence even when saved-map geometry looks good after loop closure.
- Do not approve full sim mapping acceptance until cumulative/window path ratios
  stay within gate bounds for the full drive duration.

Domain constraint: keep isolated MuJoCo gates on CycloneDDS domains **`200–232`**
(default **`231`**). Domain **234** fails on sunrise with multicast port out of
range. Production robot SLAM remains on domain **`0`**.
## 2026-07-11 Native DDS MuJoCo Navigation Closure

The current canonical closure report is:

```text
artifacts/mujoco_native_navigation_acceptance_final_v7/report.json
```

It supersedes the earlier MuJoCo notes in this file that described the native
sensor path as alive but not yet suitable for complete navigation evidence.
The accepted chain is now:

```text
MuJoCo MID-360/IMU
-> C++ Livox typed DDS publisher
-> C++ Fast-LIO2 localization + continuous saved-map alignment
-> C++ traversability DDS
-> C++ native nav endpoint
   -> OctoPlanner3D
   -> LocalPlannerCore
   -> embedded PathFollower
   -> final command safety
-> typed DDS /nav/cmd_vel
-> ThunderV4 ONNX policy
```

Both phases passed against a newly generated 536,904-point MuJoCo MID-360 map;
the run did not reuse an untracked map artifact. No-motion produced a 5-point
global path and 101-point local path while publishing no final speed. Motion
produced 121 nonzero final speed samples, moved 1.121 m along the executed path,
reduced goal distance by 0.996 m, and held map-frame XY localization error to
0.035 m. Continuous map tracking completed 38 accepted PCL GICP updates with
zero rejections. PCL GICP
now reports real inlier and position-covariance diagnostics; missing metrics no
longer disable `track_against_map` on systems without small_gicp.

Raw odometry remains a local frame and is reported separately. The navigation
accuracy gate uses `map->odom * odom->body`, rejects stale/invalid map tracking,
and stops TF publication if repeated map-alignment failures disable tracking.

The LiDAR replay boundary uses C++ `--restamp-stdin-records`: the WSL-native
publisher rebases the first simulated record to its own system clock while
preserving LiDAR/IMU relative timing and point offsets. This removes the
Windows/WSL epoch skew that previously made startup intermittently reject fresh
clouds as stale.
