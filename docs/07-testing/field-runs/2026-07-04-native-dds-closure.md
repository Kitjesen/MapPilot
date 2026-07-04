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
