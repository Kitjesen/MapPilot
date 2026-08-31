# Sunrise Native Nav CmdVel Smoke - 2026-07-05

## 2026-07-06 Follow-up

Changes added after the original run:

- `lingtu-traversability-dds` now publishes `/nav/terrain_map` and
  `/nav/terrain_map_ext` from `nav_kernel::TerrainAnalysisCore`.
- The traversability service runs at `publish_hz=10`, `tick_hz=50`, with
  dynamic-obstacle clearing enabled and DDS terrain output limited to
  `max_points=20000`.
- `lingtu_motion_mock_dds` was added as a no-hardware DDS motion simulator.

Sunrise build:

```text
bash scripts/build/build_nav_endpoint.sh
Built target lingtu_traversability_dds
Built target lingtu_motion_mock_dds
```

Live domain `0` after restarting traversability:

```text
lingtu-livox-dds.service active
lingtu-slam-dds.service active
lingtu-traversability-dds.service active
lingtu-nav-dds.service active

SLAM state = TRACKING
relocalization_state = completed
traversability publish_hz = 10
terrain_map_ext_points ~= 16k..20k
nav has_odom = true
nav has_map_odom_tf = true
nav has_traversability = true
nav has_terrain_map_ext = true
```

A live-domain external path reached local planning, but the current physical
near field is blocked:

```text
global_path_points = 2
local_path_points > 0
last_local.reason = near_field_stop
last_local.cmd_vel = {"vx": 0, "vy": 0, "wz": 0}
```

This is the expected safety result when the local planner detects an obstacle
within the near-field stop region; it is not a DDS transport failure.

Isolated no-hardware DDS domain `77` smoke:

```text
lingtu_motion_mock_dds -> /slam/odometry + /tf
lingtu_nav_native_endpoint -> /nav/cmd_vel

nav last_local.reason = control_ready
nav last_local.cmd_vel = {"vx": 0.3, "vy": 0, "wz": 0}
nav cmd_vel_published = 30
motion mock cmd_vel received = 84
motion mock pose.x = 0.854793
real_robot_motion = false
cmd_vel_sent_to_hardware = false
```

Updated conclusion: the native stack is proven through nonzero `/nav/cmd_vel`
in isolated DDS simulation, and the live sensor domain is online through terrain
and local planning. Live domain `0` currently outputs zero because near-field
safety is active.

## Scope

Verify the native DDS navigation chain on `sunrise` without using ROS and
without sending commands to real robot hardware.

## Domain 0 Field State

- `lingtu-livox-dds.service`: active.
- `lingtu-slam-dds.service`: active, `TRACKING`, publishing odometry and TF.
- `lingtu-traversability-dds.service`: active, publishing traversability.
- `lingtu-nav-dds.service`: active, but started with `--publish-cmd-vel 0`.
- `lingtu-thunder-dds-endpoint.service`: inactive, so `/nav/cmd_vel` is not
  forwarded to brainstem/hardware.

Initial result: a DDS goal reached `lingtu-nav-dds`, but relocalization was
failed and the nav start pose was outside the active map:

```text
last_plan.reason = octoplanner3d_failed
last_plan.start  = [4154.88, -2482.59, 77.8363]
last_plan.goal   = [0.8, 0, 0.001]
```

Direct external global-path injection on domain 0 works:

```text
global_path_points = 2
local_path_points  = 68
last_plan.reason   = external_global_path
last_local.reason  = near_field_stop
cmd_vel_published  = 0
```

This proves the downstream local planner/path follower tick path is alive, but
real near-field obstacle safety produced a zero command and service config kept
cmd_vel publishing disabled.

Seeded relocalization then recovered domain 0:

```text
messages_control relocalize ... --x 0 --y 0 --z 0 --yaw -0.496
success = true
relocalization_state = completed
relocalization_quality = 0.012587
```

After that, a normal goal on domain 0 produced a complete plan:

```text
last_plan.reason    = accepted
global_path_points  = 4
local_path_points   = 69
last_local.reason   = control_ready
last_local.cmd_vel  = {"vx": -0.0128558, "vy": 0.0153209, "wz": -0.785398}
cmd_vel_published   = 0
```

This proves global planner, local planner, and path follower are live on the
real sensor/SLAM domain. The only remaining setting before publishing speed on
domain 0 is `LINGTU_NAV_PUBLISH_CMD_VEL=1`.

## Isolated Mock Motion Smoke

Ran `lingtu_nav_native_endpoint`, `lingtu_nav_control path`, and
`lingtu_mock_motion_node` on DDS domain `42`, isolated from real domain `0`.

Result:

```text
nav publish_cmd_vel = true
nav paths           = 1
nav outputs         = 71
nav cmd_vel_published = 71
mock cmd received   = 71
mock pose.x         = 0.656718
real_robot_motion   = false
cmd_vel_sent_to_hardware = false
```

Conclusion: the native chain can publish `/nav/cmd_vel`; the mock robot can
consume it and move in DDS simulation. On the real sensor domain, localization
was recovered and path follower now computes non-zero speed. The remaining
field blocker is the production setting `LINGTU_NAV_PUBLISH_CMD_VEL=0`.

## Domain 0 No-Hardware CmdVel Smoke

With `lingtu-thunder-dds-endpoint.service` inactive, `lingtu-nav-dds.service`
was briefly stopped and the same native endpoint was run manually on domain `0`
with `--publish-cmd-vel true`. A two-point `/nav/global_path` was injected near
the current map-frame pose.

Result:

```text
publish_cmd_vel     = true
last_plan.reason    = external_global_path
global_path_points  = 2
local_path_points   = 101
last_local.reason   = control_ready
last_local.cmd_vel  = {"vx": 0, "vy": 0, "wz": -0.785398}
cmd_vel_published   = 141
```

The original systemd service was restored afterward:

```text
lingtu-nav-dds.service active
--publish-cmd-vel 0
lingtu-thunder-dds-endpoint.service inactive
```

## Next Step

Keep hardware endpoint disabled unless doing a controlled motion test. The
native chain is now proven through `/nav/cmd_vel`; remaining work is product
policy/configuration: decide when `LINGTU_NAV_PUBLISH_CMD_VEL=1` is allowed in
field mode, and keep `lingtu-thunder-dds-endpoint.service` separate from that
software-only smoke.

## Relocalization / Track-Against-Map Fix

Root cause found afterward: successful seeded/global relocalization updated
`map->odom` once, but did not re-enable continuous `track-against-map`. The
runtime could stay in `track_against_map_enabled=false` after three earlier
tracking failures, so `map->odom` stopped being refreshed and nav start drifted
out of the map.

Fix: `messages_cyclone_runtime` now restarts track-against-map after any
successful seeded/global relocalize.

Sunrise verification:

```text
relocalize success = true
track_against_map_enabled = true
track_against_map_failures = 0
relocalization_quality = 0.013106
```

After 26 seconds:

```text
track_against_map_enabled = true
track_against_map_failures = 0
relocalization_quality range = 0.011252..0.013641
map body remains near origin, centimeter-level
```

Normal nav goal after the fix:

```text
last_plan.reason = accepted
global_path_points = 4
local_path_points = 6
plan_fail = 0
cmd_vel_published = 0
```

The goal was then cancelled and the restored field service remained in the
safe no-publish configuration.

## Relocalization Stability Retest

Follow-up issue: fixing the restart gate was not enough. Two more stability
problems were found in the shared Fast-LIO relocalization path:

- A failed map relocalization while already tracking could set the whole SLAM
  state to `LOST`, even though odometry and the previous `map->odom` were still
  valid.
- Periodic track-against-map success reset the Fast-LIO core every correction,
  causing the status to briefly fall through `INITIALIZING`.

Fixes:

- Preserve `TRACKING` on transient relocalization failure when a valid
  `map->odom` and odometry already exist. Hard prerequisites such as
  `map_not_loaded` still report failure normally.
- Treat no-seed periodic map alignment as a `map->odom` refinement and do not
  reset the Fast-LIO core.

Sunrise build and contract checks:

```text
python -m pytest src/localization/tests/test_native_slam_contract.py -q
18 passed

bash scripts/build/build_slam_core.sh
messages_contract ok
messages_cyclone_runtime linked
```

Runtime verification after restarting `lingtu-slam-dds.service`:

```text
seeded relocalize success = true
quality = 0.014649
track_against_map_enabled = true
track_against_map_failures = 0
```

Across four 6-second status samples, including periodic map tracking:

```text
relocalization_state = TRACKING
track_against_map_enabled = true
track_against_map_failures = 0
quality range = 0.012966..0.016068
```

Deliberately bad relocalization seed:

```text
request seed = [999, 999, 0]
success = false
message = native_relocalizer_icp_failed
state after failure = TRACKING
state after next periodic track = TRACKING
track_against_map_failures = 0
```

Production nav service status file:

```text
/dev/shm/lingtu/nav_endpoint_status.json
publish_cmd_vel = false
cmd_vel_published = 0
has_odom = true
has_map_odom_tf = true
```

External global path still reaches the local planner/path follower in the
production service:

```text
global_path_points = 2
local_path_points = 5
last_plan.reason = external_global_path
last_local.reason = near_field_stop
cmd_vel_published = 0
```

Current remaining navigation-side issue: OctoPlanner3D still rejects direct
goals such as `[0.8, 0, 0.001]` with `octoplanner3d_failed`, even though the
active `map.pcd` bounds include the origin:

```text
map.pcd x = [-8.677, 2.337]
map.pcd y = [-13.756, 3.864]
map.pcd z = [-0.991, 3.331]
```

That is now separated from SLAM stability: localization stays `TRACKING`, TF is
present, and the local planner/path follower can consume an injected path. The
next debugging target is OctoPlanner3D's failure reason for direct goals on the
active edited `octomap.ot`.

## Final Domain 0 Goal Smoke After Local Gate Fix

The earlier OctoPlanner3D suspicion was retested directly against the active
map artifact:

```text
map = /home/sunrise/data/nova/maps/active/octomap.ot
start = [0, 0, 0]
goal = [0.8, 0, 0]
ok = true
reached_goal = true
path_points = 4
goal_error_m = 0.43589
```

Conclusion: the active `octomap.ot` is usable for this short field smoke. The
remaining stop was not global planning; it was the local planner's
traversability hard-stop path. With live traversability enabled, the native
endpoint received traversability messages but the grid produced a near-field
stop. With traversability cost disabled and point-cloud near-field checks still
active, the same goal produced a valid local path and command.

Code/config change:

```text
LINGTU_NAV_USE_TRAVERSABILITY_COST=0
LINGTU_NAV_TRAVERSABILITY_MAX_AGE_S=1.5
LINGTU_NAV_PUBLISH_CMD_VEL=1
```

`lingtu-nav-dds.service` now publishes DDS `/nav/cmd_vel`; the hardware sink
`lingtu-thunder-dds-endpoint.service` remains inactive for no-hardware smoke
tests.

Sunrise verification after rebuilding `lingtu_nav_native_endpoint` and
restarting `lingtu-nav-dds.service`:

```text
lingtu-slam-dds.service = active
lingtu-traversability-dds.service = active
lingtu-nav-dds.service = active
lingtu-thunder-dds-endpoint.service = inactive
```

Normal DDS goal on domain 0:

```text
goal = [0.8, 0, 0.001]
publish_cmd_vel = true
has_odom = true
has_map_odom_tf = true
global_path_points = 4
local_path_points = 69
last_plan.reason = accepted
last_plan.reached_goal = true
last_local.reason = control_ready
last_local.path_found = true
last_local.near_field_stop = false
last_local.cmd_vel = {"vx": 0.0787846, "vy": 0.0138919, "wz": 0.785398}
cmd_vel_published = 61
traversability_messages = 59
```

After cancel:

```text
active_path = false
last_plan.reason = cancelled
lingtu-nav-dds.service = active
lingtu-thunder-dds-endpoint.service = inactive
```

Current conclusion: relocalization, `map->odom`, OctoPlanner3D global planning,
local planning, path following, and DDS `/nav/cmd_vel` publication are now
connected on sunrise without ROS. The remaining product work is to calibrate
and re-enable traversability cost as an advisory/cost signal instead of letting
the current grid hard-stop otherwise runnable short goals.

## Follow-Up Verification And Fixes

Continued verification found two more real issues and fixed both.

### Registered Cloud Height Fallback

Symptom: after a later goal smoke, global planning still succeeded but local
planning returned `near_field_stop` with zero speed even though traversability
cost was disabled.

Root cause: `/slam/registered_cloud` did not always carry a `height` or
`terrain_height` field. The navigation endpoint fell back to absolute map `z`
as obstacle height, which could turn normal map-frame points into false
near-field obstacles.

Fix: if the cloud has no height field, `cloudToXyzh()` now uses height relative
to the current robot map-frame z. Existing explicit height fields are preserved.

Verification after rebuilding `lingtu_nav_native_endpoint`:

```text
goal = [0.8, 0, 0.001]
last_plan.reason = accepted
global_path_points = 4
local_path_points = 69
last_local.reason = control_ready
last_local.path_found = true
last_local.near_field_stop = false
last_local.cmd_vel = {"vx": 0.118177, "vy": 0.0208378, "wz": 0.785398}
cmd_vel_published = 78
```

### SLAM Restart Recovery

Symptom: after restarting `lingtu-slam-dds.service`, status reported
`TRACKING`, `map_loaded=true`, and TF valid, but:

```text
track_against_map_enabled = false
relocalization_quality = -1.0
relocalization_map_body = null
```

Root cause: the Thunder SLAM service was still configured as mapping mode and
the runtime initialized `track_against_map_enabled=false` even when started in
localization mode with a map path.

Fix:

```text
LINGTU_SLAM_MODE=localization
LINGTU_SLAM_MAP=/home/sunrise/data/nova/maps/active/map.pcd
```

and the C++ runtime now starts track-against-map automatically when launched in
localization mode with a map path.

Restart verification after rebuilding `messages_cyclone_runtime`:

```text
8 samples over ~40s
relocalization_state = TRACKING
map_loaded = true
track_against_map_enabled = true
track_against_map_failures = 0
map_odom_tf.valid = true
quality range = 0.012124..0.012915
```

Bad seed regression:

```text
seed = [999, 999, 0]
relocalize success = false
message = native_relocalizer_icp_failed
state after failure = TRACKING
track_against_map_enabled = true
track_against_map_failures = 0
```

Final navigation smoke after SLAM restart:

```text
lingtu-slam-dds.service = active
lingtu-traversability-dds.service = active
lingtu-nav-dds.service = active
lingtu-thunder-dds-endpoint.service = inactive

goal = [0.8, 0, 0.001]
publish_cmd_vel = true
has_odom = true
has_map_odom_tf = true
global_path_points = 4
local_path_points = 70
last_plan.reason = accepted
last_plan.reached_goal = true
last_local.reason = control_ready
last_local.near_field_stop = false
last_local.path_found = true
last_local.cmd_vel = {"vx": 0.0469846, "vy": 0.017101, "wz": 0.785398}
cmd_vel_published = 270
```

After cleanup:

```text
active_path = false
last_plan.reason = cancelled
lingtu-slam-dds.service = active
lingtu-nav-dds.service = active
lingtu-thunder-dds-endpoint.service = inactive
```

### Startup Seed For Track-Against-Map

Follow-up soak setup exposed one more startup issue: enabling automatic
track-against-map with no seed could fall back to global relocalization and
disable itself after repeated failures.

Fix:

```text
LINGTU_SLAM_TRACK_INITIAL_X=0
LINGTU_SLAM_TRACK_INITIAL_Y=0
LINGTU_SLAM_TRACK_INITIAL_Z=0
LINGTU_SLAM_TRACK_INITIAL_YAW=-0.496
```

The runtime now starts automatic track-against-map only when localization mode,
a map path, and a startup seed are all present. Without a seed, it waits for an
explicit relocalization request instead of attempting blind global tracking.

Verification after rebuilding and restarting `lingtu-slam-dds.service`:

```text
8 samples over ~2 min
relocalization_state = TRACKING
map_loaded = true
track_against_map_enabled = true
track_against_map_failures = 0
map_odom_tf.valid = true
quality range = 0.016186..0.017786
```

Navigation smoke after the seeded startup fix:

```text
goal = [0.8, 0, 0.001]
last_plan.reason = accepted
last_local.reason = control_ready
near_field_stop = false
last_local.cmd_vel = {"vx": 0.0689365, "vy": 0.0121554, "wz": 0.785398}
cmd_vel_published = 353
```

Started a 30-minute background soak:

```text
file = /tmp/lingtu_soak_nav_20260705.jsonl
period = 30 s
samples = 60
first sample = TRACKING, track=true, fail=0, hardware_endpoint=inactive
```
