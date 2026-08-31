# Sunrise Profile Sequential Validation - 2026-07-06

Scope: sequentially validate LingTu product profiles on `sunrise` up to
`/nav/cmd_vel`. Real hardware command forwarding remains out of scope unless
explicitly enabled later.

## Start State

```text
host = ubuntu
repo = /home/sunrise/data/inovxio/lingtu
date = 2026-07-06 CST

lingtu-livox-dds.service = active
lingtu-slam-dds.service = active
lingtu-traversability-dds.service = active
lingtu-nav-dds.service = active
lingtu.service = active
```

## Evidence Scale

| Level | Meaning |
|---|---|
| L0 | code/config exists |
| L1 | isolated mock or synthetic data |
| L2 | sunrise live data/service path |
| L3 | official profile entry accepted end to end |

## Profile Progress

| Profile | Target output | Status | Evidence |
|---|---|---:|---|
| `map` | fresh map package + artifact gate | pass | `scripts/lingtu map start/save/end` produced `map.pcd`, `octomap.ot`, `occupancy.npz`, `metadata.json`; saved-map artifact gate passes |
| SLAM restart/relocalization | stable `TRACKING` + valid `map->odom` | pass | `scripts/lingtu nav start active --initial-pose ...` resolves active map, seeded relocalize succeeds, Gateway reports valid `map_odom_tf` |
| `tracking` | official tracking route to `/nav/cmd_vel` | pending | not executed in this run |
| `nav` | official nav goal to `/nav/cmd_vel` | partial | official Gateway goal is accepted and native endpoint publishes `/nav/cmd_vel`; live terrain/near-field input holds speed at zero. Commissioning no-obstacle smoke proves non-zero cmd_vel generation |
| `teleop` | teleop command to nav output boundary | pending | not started |
| `teleop_avoid` | teleop command with avoidance/cost influence | pending | not started |
| `inspection` | inspection goal delegated to nav | pending | not started |
| `tare_explore` | exploration goal/path delegated to nav | pending | not started |

## Running Log

### 10:03 CST - Connection And Service Preflight

Result:

```text
ssh reachable = yes
/opt/lingtu/current = /home/sunrise/data/inovxio/lingtu
core services = active
```

Next: inspect map profile readiness, current active map, artifact converter,
and map service entrypoints.

### 10:06-10:12 CST - Active Map Preflight

Current active map:

```text
/home/sunrise/data/nova/maps/active
  -> /home/sunrise/data/nova/maps/accept_ready_20260702_162847
```

Runtime processes:

```text
lingtu_nav_native_endpoint
  --map /home/sunrise/data/nova/maps/active/octomap.ot
  --publish-cmd-vel 1
  --use-traversability-cost 1

messages_cyclone_runtime
  --mode localization
  --map /home/sunrise/data/nova/maps/accept_ready_20260702_162847/map.pcd
  --track-against-map-seed-file .../track_seed.json
```

Required binaries are present and executable:

```text
build/octoplanner3d_headless/octoplanner3d_headless OK
build/octoplanner3d_headless/octoplanner3d_pcd_to_octomap OK
build/nav_endpoint/lingtu_nav_control OK
build/nav_endpoint/lingtu_nav_native_endpoint OK
```

Active map files:

```text
map.pcd       39861466 bytes
octomap.ot      172128 bytes
occupancy.npz     1350 bytes
metadata.json    33457 bytes
```

Saved-map artifact gate:

```text
ok = true
frame = map
map_pcd exists = true, format_ok = true
octomap exists = true, format_ok = true
occupancy_grid exists = true, format_ok = true
blockers = none
```

Map planning check (historical; the `map check` command is now retired):

```text
command = bash scripts/lingtu map check active --forward 0.8
artifact_gate = PASS
map_plan_ok = false
executable_feasible = false
points = 0
planner = octoplanner3d
reasons = planning_failed,start_occupied_or_out_of_map
```

Conclusion: active map package provenance passes, but the current start pose is
not yet accepted by the saved-map plan validation. `map` is L2 for artifact
readiness, not L3 for profile acceptance.

### 10:28-10:40 CST - Fresh Map Profile

Official map entrypoints were exercised:

```text
bash scripts/lingtu map start
bash scripts/lingtu map save seq_profile_map_20260706_1028
bash scripts/lingtu map end
```

Fresh map package result:

```text
map.pcd = present
octomap.ot = present
occupancy.npz = present
metadata.json = present
navigation_ready = true
octomap_ok = true
metadata_ok = true
occupancy_ok = true
tomogram_ok = false
```

`tomogram_ok=false` is from missing optional `open3d`; it does not block
OctoPlanner3D, which consumes `octomap.ot` plus metadata.

Finding: after `map end`, the SLAM systemd runtime drop-in still had
`LINGTU_SLAM_MODE=mapping`. The navigation path recovers when `nav start <map>`
sets localization mode again, but `map end` should restore the previous SLAM
mode explicitly.

### 10:45-10:58 CST - Active Map Repair And Seeded Relocalization

An earlier failed `nav start active` left the active map symlink as a self-loop:

```text
/home/sunrise/data/nova/maps/active -> active
```

The active map was restored to:

```text
/home/sunrise/data/nova/maps/active
  -> /home/sunrise/data/nova/maps/accept_ready_20260702_162847
```

Code fixes made during this run:

```text
scripts/lingtu
  - nav start resolves raw "active" to the concrete active map name before
    setting SLAM mode, starting the Gateway session, and relocalizing.
  - seeded relocalize retries timeout responses, not only scan/cloud readiness
    responses.

src/gateway/routes/session.py
  - session start normalizes map=active/map_name=active to the concrete map
    directory before activating the symlink.

src/gateway/routes/maps.py
  - validate_plan response is sanitized so non-finite planner preview values do
    not produce HTTP 500.

src/gateway/routes/auth.py
  - validation errors are JSON-encoded so invalid request bodies do not produce
    HTTP 500.
```

Targeted tests passed locally and on sunrise:

```text
gateway session/map contract focused tests = 3 passed
deployment relocalize contract test = 1 passed
bash -n scripts/lingtu = pass
```

Relocalization result:

```text
scripts/lingtu nav start active --initial-pose 0 0 -0.496
map_name = accept_ready_20260702_162847
relocalization_state = completed
quality ~= 0.016-0.018
map_odom_tf.valid = true
```

### 11:00-11:10 CST - Official Nav Goal And Live Safety Hold

With real-runtime-evidence gate disabled for commissioning only:

```text
/etc/systemd/system/lingtu.service.d/commissioning-cmdvel.conf
  LINGTU_REQUIRE_REAL_RUNTIME_EVIDENCE=0
```

Gateway then reports:

```text
can_accept_goal = true
real_runtime_evidence.reason = disabled_for_commissioning
```

Hardware forwarding stayed out of scope and inactive:

```text
lingtu-thunder-dds-endpoint.service = inactive/disabled
robot-brainstem.service = inactive
can-setup.service = failed
```

Official Gateway goals:

```text
scripts/lingtu nav goal 0.063 -0.219 -1.330
scripts/lingtu nav goal 0.074 -0.268 -1.328
```

Both were accepted by Gateway and the native endpoint observed the goals. They
incremented native path/cmd_vel counters, but the targets were within the C++
`goal_reached_m=0.35` threshold, so the local loop immediately produced zero
speed.

Direct native DDS far goal:

```text
build/nav_endpoint/lingtu_nav_control goal 0.8 0 0.001 --domain-id 0
```

Result with normal live obstacle/terrain inputs enabled:

```text
last_plan.accepted = true
global_path_points = 4
local_path_points = 66
last_local.path_found = true
last_local.reason = near_field_stop
cmd_vel = {vx: 0, vy: 0, wz: 0}
cmd_vel_published increased by 57
```

Conclusion: the native navigation chain is alive through global planning, local
planning, local path publication, and `/nav/cmd_vel` publication. The zero speed
is caused by live near-field/terrain safety input, not by Gateway, DDS, or the
hardware endpoint.

### 11:14-11:22 CST - Commissioning CmdVel Smoke

Added a deployment-safe commissioning switch:

```text
LINGTU_NAV_CHECK_OBSTACLE=1           # production default
--check-obstacle true|false           # native endpoint CLI
```

Important behavior: this switch is handled at the native endpoint input layer.
When set to `0`, the endpoint temporarily stops feeding live obstacle,
terrain-map, and traversability inputs into the local planner. The local planner
itself still runs its normal no-obstacle scoring path.

sunrise verification:

```text
python3 -m pytest \
  src/runtime/tests/test_thunder_deployment_entrypoints.py::test_thunder_nav_dds_service_diagnoses_missing_endpoint_binary -q

result = 1 passed

LINGTU_BUILD_JOBS=2 bash scripts/build/build_nav_endpoint.sh
result = lingtu_nav_native_endpoint rebuilt successfully
```

Temporary smoke configuration:

```text
/etc/systemd/system/lingtu-nav-dds.service.d/commissioning-check-obstacle.conf
  LINGTU_NAV_CHECK_OBSTACLE=0
```

Smoke result for `goal 0.8 0 0.001`:

```text
check_obstacle = false
active_path = true
global_path_points = 4
local_path_points = 66
obstacle_points = 0
last_plan.accepted = true
last_local.reason = control_ready
last_local.near_field_stop = false
last_local.path_found = true
cmd_vel = {vx: 0.0975, vy: 0.168875, wz: 0.785398}
outputs = 101
cmd_vel_published = 102
```

Raw status snapshots copied locally:

```text
pytest_runtime_output_cmdvel_check2/before.json
pytest_runtime_output_cmdvel_check2/during.json
pytest_runtime_output_cmdvel_check2/after_cancel.json
```

The temporary drop-in was removed and the service restored:

```text
check_obstacle = true
use_traversability_cost = true
active_path = false
obstacle_points = 20000
```

Conclusion: `/nav/cmd_vel` non-zero speed generation is proven on sunrise when
the live obstacle inputs are withheld. The remaining nav work is the
terrain/near-field interpretation, not the command output path.

## Current Conclusions

1. Start sequential profile validation at `map`, then SLAM/relocalization, then
   `nav` to `/nav/cmd_vel`. This order is correct because every downstream
   route depends on a concrete map package and stable `map->odom`.
2. `map` and seeded relocalization are now validated on sunrise.
3. Native `nav` is validated through OctoPlanner3D global path, local path, and
   `/nav/cmd_vel` publication.
4. Non-zero `/nav/cmd_vel` is proven with obstacle input withheld; with live
   terrain input enabled, the local loop stops at `near_field_stop`.
5. The next fix should not touch real robot motion control. It should focus on
   registered cloud / terrain-map / traversability interpretation and
   near-field stop thresholds so live data no longer creates false stops.

## Remaining Profile Order

| Order | Route | Next validation |
|---:|---|---|
| 1 | `tracking` | run official tracking entrypoint and confirm it consumes live SLAM health without disturbing localization |
| 2 | `nav` live terrain | inspect `registered_cloud`, terrain height fields, body/map transform, and near-field stop points that trigger the hold |
| 3 | `teleop` | verify teleop command reaches mux/output boundary without hardware endpoint |
| 4 | `teleop_avoid` | verify avoidance layer changes or blocks teleop command when obstacle input is present |
| 5 | `inspection` | verify inspection task delegates a navigation goal and observes nav status |
| 6 | `tare_explore` | verify exploration output delegates to nav path/goal boundary |
