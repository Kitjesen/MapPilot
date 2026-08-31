# Navigation Runtime Dataflow

Status: current product/native-DDS dataflow contract
Audience: navigation, deployment, Gateway, and field-readiness maintainers
Replaced by: not replaced

This document records the shipped navigation-base dataflow. It is scoped to
mapping, saved-map planning, local planning, path following, safety, command
ownership, teleoperation, and the goal-producing modes that reuse navigation.
Semantic memory internals and UI rendering details remain out of scope.

## Transport Model

LingTu topics are runtime stream names, not ROS 2 topics.

## SLAM Runtime Rule

Do not write or ship Python SLAM. MuJoCo may simulate the robot, MID-360 raw
frames, and IMU, but pose estimation, map building, and saved-map
relocalization must come from the same native C++ SLAM/localization runtime used
on the robot (`slamd` / `lt-slam.service`) or an
explicit external native SLAM service. Python code may adapt streams, status,
reports, and test evidence only; it must not become a SLAM backend.

For Windows-hosted MuJoCo acceptance, ordinary replay records cross into the
WSL-native Livox DDS publisher with `--restamp-stdin-records`. The C++ publisher
rebases those replay records to its own system clock while preserving relative
LiDAR/IMU timing and per-point offsets. The navigation fixture is deliberately
different: `--navigation-fixture` preserves its single simulated-hardware clock
across IMU, odometry, TF, and clouds, while native consumers measure liveness
from local steady-clock DDS receipt time. This avoids coupling control safety to
Windows/WSL `CLOCK_REALTIME` steps. Real MID-360 input uses neither replay mode.

| Boundary | Transport | Payload shape | ROS 2 required |
| --- | --- | --- | --- |
| Hardware / external runtime to LingTu | typed DDS | IDL-generated native C types | no |
| LingTu module to module, same process | `Out.publish()` to wired `In._deliver()` callback | Python objects | no |
| Field OctoPlanner3D global planner | direct C++ call inside `navd` | `PlanRequest` / `PlanResult` in memory | no |
| Dev/compat OctoPlanner3D wrapper | subprocess stdin/stdout | JSON request/result | no |
| Map artifact conversion | subprocess + files | `map.pcd` to `octomap.ot` | no |
| Command output to Thunder brainstem | typed DDS then native `lingtu-driver` gRPC client | `FinalVelocityCommand` to Brainstem `WalkChecked(seq, Vector3)` | no |

Default internal wiring is callback delivery:

```text
ModuleA.out.publish(msg)
  -> Blueprint wire callback
  -> ModuleB.in._deliver(msg)
  -> ModuleB's subscribed handler
```

Only explicit cross-boundary wires use `dds`, `shm`, or `lcm`.

## Field DDS Contract

Registered typed DDS topics are published as `rt/...` DDS topic names. Example:
LingTu `/nav/cmd_vel` becomes DDS `rt/nav/cmd_vel`.

| Producer -> consumer | LingTu stream | DDS type | Runtime meaning |
| --- | --- | --- | --- |
| Livox -> SLAM | `/lidar/raw_frame` | `lingtu.dds.LivoxFrame` | scan-level Livox frame |
| Livox -> SLAM | `/imu/raw` | `lingtu.dds.Imu` | time-aligned IMU sample |
| SLAM -> terrain/nav | `/tf`, `/tf_static` | `lingtu.dds.TFMessage` | frame transforms |
| SLAM -> terrain/nav | `/slam/odometry` | `lingtu.dds.Odometry` | current pose and twist |
| SLAM -> terrain/nav | `/slam/registered_cloud` | `lingtu.dds.PointCloud2` | current body-aligned obstacle cloud |
| SLAM -> mapping/UI | `/slam/map_cloud` | `lingtu.dds.PointCloud2` | map-frame live map increment |
| SLAM -> map/UI | `/slam/saved_map_cloud` | `lingtu.dds.PointCloud2` | saved-map snapshot |
| SLAM -> safety/UI | `/slam/localization_health` | `lingtu.dds.Text` | structured localization health |
| SLAM -> safety/UI | `/slam/localization_quality` | `lingtu.dds.Float32` | quality in `[0,1]` |
| native command clients -> nav | `/nav/command/request` | `lingtu.dds.NavigationCommandRequest` | typed goal, cancel, or operator velocity request with `request_id` |
| nav -> native command clients | /nav/command/ack | lingtu.dds.NavigationCommandAck | authoritative admission acceptance/rejection for the matching request; not task completion |
| Gateway -> native explore | /nav/exploration/command | lingtu.dds.ExplorationCommandRequest | typed START/PAUSE/RESUME/STOP with request and session identity |
| native explore -> Gateway | /nav/exploration/ack | lingtu.dds.ExplorationCommandAck | authoritative exploration FSM transition acceptance/rejection |
| map/terrain control -> nav | `/nav/map_clearing` | `lingtu.dds.Bool` | clear map-derived planner caches |
| map/terrain control -> nav | `/nav/cloud_clearing` | `lingtu.dds.Bool` | clear near-field obstacle caches |
| traversability -> nav | `/nav/traversability` | `lingtu.dds.OccupancyGrid` | local terrain risk grid |
| traversability -> nav | `/nav/terrain_map` | `lingtu.dds.PointCloud2` | planner terrain points `(x,y,z,height)` |
| traversability -> diagnostics | `/nav/terrain_map_ext` | `lingtu.dds.PointCloud2` | diagnostics-only terrain overlay |
| nav -> UI/recorders | `/nav/global_path` | `lingtu.dds.Path` | accepted saved-map path or live exploration segment |
| nav -> UI/recorders | `/nav/local_path` | `lingtu.dds.Path` | current local-plan telemetry, not an internal follower input |
| nav -> UI/recorders | `/nav/way_point` | `lingtu.dds.PoseStamped` | current look-ahead target |
| nav -> Thunder driver | `/nav/cmd_vel` | `lingtu.dds.FinalVelocityCommand` | identity-bound final post-gate command; nav endpoint is the single writer |

Natural-language instructions are not a robot DDS topic. Gateway and MCP feed
`SemanticPlanner` in-process; only its resolved goal enters the typed C++ DDS
command request. Field Products expose no secondary goal, cancel, or teleop
command topic.

The Thunder field service sets `LINGTU_NAV_COMMANDS_REQUIRED=1` and carries
`command_output_mode=endpoint_only` in the Product declaration/RunPlan contract.
If the persistent native command client is missing or the endpoint does not acknowledge a request,
Gateway rejects it. It never falls back to Python `goal_pose` or `cmd_vel`
publication in a field Product. Local publication exists only in a local or
simulation Host whose compiled mode is `local_driver`.

## Product Command Chains

### Autonomous Navigation

```text
Web coordinate goal / CLI / MCP resolved goal
  -> process-wide NavigationCommandClient
  -> liblingtu_nav_client.so
  -> DDS /nav/command/request (kind=goal, request_id=...)
  -> navd (control_mode=autonomy)
  -> validate frame, authority, localization, map, and goal admission
  -> DDS /nav/command/ack (accepted=true, reason=planning_started)
  -> OctoPlanner3D::runPlan(request)
  -> Executor::setRoute(path)
  -> Executor::tick(...)
       -> select look-ahead point from global path
       -> local::Planner::plan(obstacles, traversability)
       -> PathFollower::computeControl(local_path_body)
  -> DDS /nav/cmd_vel
  -> Thunder native driver -> Brainstem WalkChecked
```

The command ACK closes command submission, not navigation execution. For a
goal, `accepted=true` means the authoritative endpoint admitted the request and
started asynchronous planning. Planner failure, cancellation, path progress,
and goal completion are reported by navigation status/path outputs; an accepted
ACK never means that a path exists or that the robot arrived.

The global path and local path are ordinary C++ vectors inside this endpoint.
There is no DDS or LCM hop between OctoPlanner3D, LocalPlanner, and
PathFollower.

Input freshness is checked before `Executor::tick()`, collision/traversability
is checked while selecting the local path, PathFollower applies speed and
acceleration limits, and the resulting non-zero autonomous command passes the
same independent `CommandSafety` stop/slow/limit gate used for operator
requests before `/nav/cmd_vel` can be published.

The final motion gate also requires fresh driver-control evidence from
`lingtu-driver`: connected, ready, motors enabled, no critical motor fault,
valid Brainstem lease, owner `grpc`, and owner id `lingtu-driver`. If that
readiness becomes stale or false, `lt-nav` clears endpoint motion and
publishes/holds zero rather than continuing to emit stale non-zero commands.

`/nav/local_path` is not the control hand-off. `local::Planner::plan()`
returns `local_path_body` in memory and `PathFollower::computeControl()`
consumes that vector in the same `Executor::tick()`. The DDS local-path writer
only exposes the map-frame copy to Web, recorders, and acceptance tools.

In localization mode, global pose is:

```text
map->body = map->odom * odom->body
```

The SLAM runtime does not publish an identity `map->odom` before the first
saved-map alignment. Periodic `track_against_map` reports attempts, successes,
rejections, waits, consecutive failures, and last-success age in the SLAM
status snapshot. If repeated alignment failures disable map tracking, the
runtime stops publishing `/tf`; the native nav endpoint then fails closed on a
stale transform instead of continuing on an old global pose.

### Pure Teleoperation

```text
Web keyboard / hand controller / MCP physical velocity request
  -> NavigationCommandClient::sendTeleop()
  -> DDS /nav/command/request (kind=teleop)
  -> endpoint (control_mode=teleop)
  -> DDS /nav/command/ack (accepted for safety arbitration)
  -> command age gate + linear/yaw limits
  -> DDS /nav/cmd_vel
```

Pure `teleop` deliberately does not require SLAM, odometry, point cloud, map,
or traversability. It is not obstacle avoidance. The endpoint rejects goals
and external global paths while this mode owns control.

### Teleoperation With Obstacle Avoidance

```text
/nav/command/request (kind=teleop)
  + /slam/odometry
  + /slam/registered_cloud
  + /nav/traversability
  -> endpoint (control_mode=teleop_avoid)
  -> InputGate (fresh pose/cloud)
  -> command age + finite-value + speed-limit precheck
  -> Executor::tickIntent()
  -> local::Planner::plan(LocalPlanRequest{MotionIntentTarget})
  -> PathFollowerCore
  -> curved-path final safety gate
  -> DDS /nav/cmd_vel
```

`teleop_avoid` fails closed without current localization or obstacle context.
With `LINGTU_TELEOP_LOCAL_PLANNER=1`, translational velocity intent is a short
rolling local-planning target. LocalPlanner may select a safe path within
`LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG` of the requested direction, and the
PathFollower tracks that path without exceeding the requested speed. The
default planning horizon is controlled by
`LINGTU_TELEOP_PLANNER_HORIZON_M`. OctoPlanner3D is not involved because this
mode has no global goal.

Pure rotation still uses the direct swept-footprint safety check. If no safe
local path exists, the endpoint publishes zero and keeps operator control
latched. Assisted teleoperation never invokes LocalPlanner's autonomous
rotation/back-up recovery.

### Other Goal Producers

Inspection, patrol, object tracking, and exploration do not own a
second motion stack. They select lifecycle actions, the next goal, or an
explicit path, then reuse the autonomous chain above:

```text
inspection/patrol scheduler  -> /nav/inspection/task/request -> native inspection executor -> autonomous chain
TARE exploration lifecycle  -> /nav/exploration/command -> native ExploreControl -> /nav/exploration/ack
saved-map exploration        -> selected target -> typed Goal -> GlobalPlanner -> local chain
map-free exploration         -> selected target -> typed live segment -> rolling planner -> local chain
semantic instruction         -> SemanticPlanner -> resolved goal -> typed request -> autonomous chain
explicit tracking path       -> /nav/global_path -> Executor -> LocalPlanner -> PathFollower
```

For `explore --map MAP`, the native exploration endpoint uses route `Map`. For
`explore` without a map, it uses route `Live` and does not call GlobalPlanner.
The endpoint
starts idle. A fresh
map-frame START command and fresh odometry, TF, and identity-versioned rolling
occupancy snapshot are required before it can choose a goal. Duplicate request
IDs replay the cached ACK without repeating state changes. PAUSE and STOP cancel
or clear pending goal work. `/dev/shm/lingtu/explore_status.json` is bounded-age
telemetry only; a stale status blocks START readiness but never blocks an
operator STOP request.

The native operator/diagnostic CLI uses the same request/ACK client; it does
not publish a second raw control topic:

```bash
lingtu_nav_control explore start field-session --request-id field-start
lingtu_nav_control explore pause operator_pause --request-id field-pause
lingtu_nav_control explore resume operator_resume --request-id field-resume
lingtu_nav_control explore stop operator_stop --request-id field-stop
```

Mapping is different: LiDAR/IMU -> SLAM -> `map.pcd` -> map artifacts. The map
Product does not start the nav endpoint and does not publish motion commands.

### Stop, Loss, And Cache-Control Chains

```text
/nav/command/request (kind=cancel)
  -> /nav/command/ack
  -> clear global/local path and follower state
  -> publish zero /nav/cmd_vel when command output is enabled

stale odometry / TF / registered cloud
  -> InputGate closes
  -> autonomous or teleop_avoid output becomes zero

/nav/map_clearing or /nav/cloud_clearing
  -> clear terrain/live-obstacle planner cache
  -> next local plan uses fresh observations only
```

## Why `/nav/local_path` Is Not Re-subscribed

`Executor::tick()` calls `local::Planner::plan()` and immediately passes the
returned `local_path_body` to `computeControl()` in the same call. The endpoint
then publishes `local_path_map` for Web, diagnostics, recording, and acceptance
tests. Re-subscribing to `/nav/local_path` inside the same process would add
latency, duplicate ownership, and a stale-path race.

## Module Fallback Dataflow

The table below describes the Python Module/dev compatibility graph. It is not
the Thunder field command path when `lt-nav.service` owns navigation.

| Step | Producer | Consumer | Data | Type | Transport |
| ---: | --- | --- | --- | --- | --- |
| 1 | Livox runtime | DDS endpoint / localization adapter | raw LiDAR frame | `LivoxFrame` -> `PointCloud2` | DDS |
| 2 | IMU runtime | DDS endpoint / localization adapter | IMU | `Imu` | DDS |
| 3 | SLAM/localizer | localization adapter | odometry | `Odometry` | DDS |
| 4 | SLAM/localizer | localization adapter | registered cloud | `PointCloud2` | DDS |
| 5 | SLAM/localizer | localization adapter | live map cloud | `PointCloud2` | DDS |
| 6 | SLAM/localizer | localization adapter | localization health/quality | `dict`, `float` | DDS |
| 7 | localization adapter | Navigation, maps, local planner, path follower, safety, Gateway | odometry fan-out | `Odometry` | callback |
| 8 | localization adapter | MapService, occupancy, elevation, voxel, terrain, Gateway | map cloud fan-out | `PointCloud2` / `MapCloudFrame` | callback |
| 9 | MapsServiceCore / SaveMapEngine | MapPipelineCore | saved source map | `map.pcd`, patches, poses | C ABI -> native transaction |
| 10 | MapPipelineCore | native cleanup | filtered source map | `map.pcd` | direct C++ call |
| 11 | MapPipelineCore | artifact builders | occupancy, OctoMap, ESDF, traversability | typed map artifacts | native C++ build |
| 12 | SaveMapEngine | canonical map package | cleaned source, metadata, and built artifacts | `<map_root>/<map_id>/` | direct map replacement |
| 13 | Gateway/MCP/native client | `navd` command ingress | goal/cancel/stop/teleop intent | typed DDS command | DDS |
| 14 | `navd` map gate | native global planner | current pose, goal, active map snapshot | direct C++ call | in-process |
| 15 | native global planner | `Executor` | verified global route | native route result | in-process |
| 16 | live cloud/traversability inputs | native local planner | collision and terrain evidence | native views | in-process |
| 17 | `Executor` | native local planner | active route slice or assisted intent | `LocalPlanRequest` | in-process |
| 18 | native local planner | native follower | local path or exact B-spline | native result | in-process |
| 19 | native follower | final control | pre-safety body velocity | native `Twist` | in-process |
| 20 | final safety and authority | native driver | checked final velocity | DDS `FinalVelocityCommand` | DDS |
| 21 | native driver | Brainstem | sequence-checked body velocity (`m/s`, `rad/s`) | gRPC `WalkChecked(seq, Vector3)` | gRPC |

## Live Map Cloud Cleanup

The field live-map path is native and scan-synchronous. It does not reconstruct
ray origins from a later odometry sample and it does not send the high-rate map
hot path through Gateway.

```text
/slam/map_observation
  -> mapd: reset_epoch + sequence admission
  -> range/z filtering + voxel insertion + bounded column carving + decay
  -> /maps/state + /maps/scene + non-control map layers
  -> HostBus -> Gateway viewer

/slam/registered_cloud + /slam/odometry
  -> standalone traversability rolling layer
  -> /nav/traversability
  -> navd planner safety
```

Rules:

- `MapObservation` carries the accepted incremental scan, exact
  `map <- sensor` pose, sensor origin, timestamp, `reset_epoch`, and sequence.
  mapd rejects duplicate, stale, malformed, or oversized observations.
- An epoch change atomically clears realtime map state so old obstacles cannot
  survive a SLAM reset or map-frame discontinuity.
- Column carving is restricted to a configured sensor-relative height band;
  it must not clear a different floor sharing the same XY column.
- Decay runs from an independent clock, so stale display cells can expire even
  after input stops. Live voxel, accumulated-grid, snapshot, and serialized
  scene sizes have configured hard limits and expose rejection counters.
- `/maps/scene` is visualization/state data. `/nav/traversability` remains the
  sole control-risk map and has one standalone native writer.
- Gateway consumes the coherent native scene. mapd is the only live map-layer
  owner in field and simulation; the retired Python layer wrappers are absent.
- The navd endpoint keeps its own short-horizon collision/motion state. It may
  consume the current registered cloud and traversability product, but it must
  not become a persistent map manager.
- Save-time cleanup under `src/maps/prune/cpp` is a separate persistent-artifact
  operation. It cannot substitute for realtime decay or obstacle clearing.
- `src/maps` is the native map domain for reusable spatial products and saved
  map assets: source `map.pcd`, occupancy, OctoMap/voxel artifacts, ESDF,
  traversability artifacts, active-map control, health, and bundle queries.
  It must not own the 20 Hz endpoint-local collision cache. Conversely,
  `MotionLayer` must not become a saved-map asset manager.

Column carving, decay, and bounded storage are implemented contracts, but the
claim that moving people leave no unacceptable residual still requires the
named MuJoCo/replay and MID-360 long-run acceptance gates.

## Gateway Scene Layer Contract

Gateway exposes the scene-layer contract in both:

```text
GET /api/v1/app/bootstrap     -> scene.layers
GET /api/v1/app/capabilities  -> realtime.scene_layers
```

The contract is intentionally transport-aware, not viewer-specific:

| Layer | Kind | Transport | Source | Role |
| --- | --- | --- | --- | --- |
| `saved_map` | point cloud | HTTP | saved map points endpoint | static reference |
| `live_cloud` | point cloud | WebSocket | `map_scene` point-cloud layer | clean live map |
| `elevation` | raster texture | opt-in SSE | `maps.elevation` scene layer | lowest observed Z only; not ground or walkability |
| `native_traversability` | raster texture | SSE | `native_traversability` event | native 0..100 control-risk projection |
| `path` | polyline | SSE | `global_path` / `local_path` events | navigation plan |
| `robot` | pose marker | SSE | `odometry` event | robot pose |

The internal terrain module may still compute `slope_grid` for local planning,
but slope is not a Gateway/Web layer and must not be presented as a field
control truth.  The native traversability layer is the only Web projection that
may be labelled as control risk, and it remains read-only.

Web clients treat the point-cloud layer in `map_scene` as the live map product.
Raw SLAM clouds remain available to explicit development tools such as Rerun,
but are not a Product Gateway fallback.

## Rerun-Style Point Cloud Display

The operator viewer follows the same display model as Rerun's live spatial
view: point clouds are 3D geometry, not a 2D canvas projection.

Rules:

- Live scan/map data is decoded into typed arrays and rendered as one
  GPU-backed 3D point layer: `positions + colors + point_size`.
- Height/risk coloring belongs in the layer producer or decoder, not in
  per-frame DOM/canvas drawing code.
- Voxel boxes are reserved for explicit voxel/block layers. A live LiDAR scan
  should render like Rerun `Points3D`, while occupancy/voxel-map products may
  render like Rerun `Boxes3D` or textured raster layers.
- Gateway owns transport and scene-layer contracts. MapService owns map assets.
  Neither should contain browser rendering policy.

Current implementations:

- `src/kernels/gateway/pointcloud_codec` provides the native C++ PCLD encoder
  for Gateway live-cloud frames.
- `src/runtime/utils/binary_codec.py` loads that native encoder when the shared
  library is built; otherwise it uses the NumPy reference encoder.
- `web/src/workers/cloudDecoder.ts` decodes `/ws/cloud` frames into
  `Float32Array` positions and colors.
- `web/src/components/scene3d/layers/liveCloudLayer.ts` renders live point
  clouds as Three.js `Points`.
For the `nav` Product in `env=real`, the DDS navigation boundary is owned
by the C++ `lt-nav` service, not Python `nav.in` / `nav.out` adapters.
It subscribes to `rt/nav/command/request`, `rt/slam/odometry`,
`rt/slam/registered_cloud`, `rt/nav/traversability`,
`rt/nav/terrain_map`, and `rt/nav/terrain_map_ext`; it publishes
`rt/nav/command/ack`, `rt/nav/global_path`, `rt/nav/local_path`, `rt/nav/way_point`, and
`rt/nav/cmd_vel`.

## Historical Sunrise Service Snapshot (2026-07-05)

This dated field observation is retained as evidence, not as the current deployment
contract. Current process ownership comes from Product + env -> RunPlan. These
were systemd process boundaries, not necessarily one algorithm per service.

| Service | Current state on 2026-07-05 | Responsibility | Inputs | Outputs | Required for `/nav/cmd_vel` |
| --- | --- | --- | --- | --- | --- |
| `nav-lidar-network.service` | active, exited | Configure LiDAR Ethernet (`eth1`, `192.168.1.5/24`) | none | LiDAR network reachable | yes |
| `lt-lidar.service` | active | Livox MID-360 and IMU DDS producer | Livox hardware | `/lidar/raw_frame`, `/imu/raw` | yes |
| `lt-slam.service` | active | Native SLAM/localization DDS runtime | `/lidar/raw_frame`, `/imu/raw` | `/slam/odometry`, `/slam/registered_cloud`, `/slam/map_cloud`, `/tf`, localization health/quality | yes |
| `lt-terrain.service` | active, disabled | Native traversability grid producer | `/slam/odometry`, `/slam/registered_cloud` | `/nav/traversability` | yes for obstacle/risk-aware local planning |
| `lt-nav.service` | active | Native navigation endpoint: global planning, local planning, path following, DDS output | odometry, TF, goal, traversability, cloud, OctoMap | `/nav/global_path`, `/nav/local_path`, `/nav/way_point`, `/nav/cmd_vel` | yes |
| `lt-host.service` | active | Python Gateway/API/MCP/task/status process | user/task commands, module state | Gateway `5050`, MCP, navigation command entry | yes for external command entry |
| `lt-driver.service` | product default | Native hardware command sink and Brainstem lease owner | `/nav/cmd_vel` | Brainstem `WalkChecked`, driver control status | no for cmd_vel-only validation; yes for real motion validation |
| `lingtu-thunder-dds-endpoint.service` | compatibility-only historical observation; unit now removed | Historical Python endpoint/sink | `/nav/cmd_vel` | legacy Brainstem command path | no |
| `robot-brainstem.service` | not found / inactive | Real robot low-level control bridge | hardware command | robot control | no for cmd_vel-only validation |
| `can-setup.service` | failed | CAN interface setup | none | `can0..can3` available | no for cmd_vel-only validation |
| `robot-camera.service` | not found / inactive | Camera source | Orbbec/camera hardware | camera streams | no for LiDAR-only navigation; needed for inspection |

The Python field unit, installer, and deployment wrapper in that historical
observation have since been physically removed. Exact legacy unit names remain
only as stop/conflict cleanup tombstones. Diagnostics use the native DDS probe.

The field navigation process is:

```text
nav-lidar-network
  -> lt-lidar
  -> lt-slam
  -> lt-terrain
  -> lt-nav
  -> /nav/cmd_vel
```

`lingtu-driver`, `robot-brainstem`, and `can-setup` are below
`/nav/cmd_vel`. They matter for real motion, not for proving the navigation
stack can produce a speed command.

## Native Traversability DDS

`lt-terrain.service` runs:

```text
/opt/lingtu/current/build/nav_endpoint/lingtu_traversability_dds
```

The service subscribes:

| Topic | Type | Use |
| --- | --- | --- |
| `/slam/odometry` | `lingtu.dds.Odometry` | current pose for map-frame cloud placement |
| `/slam/registered_cloud` | `lingtu.dds.PointCloud2` | current registered scan |
| `/nav/map_clearing` | `lingtu.dds.Bool` | clear accumulated terrain/map caches |
| `/nav/cloud_clearing` | `lingtu.dds.Bool` | clear near-field terrain/cloud caches |

The service publishes:

| Topic | Type | Data |
| --- | --- | --- |
| `/nav/traversability` | `lingtu.dds.OccupancyGrid` | 2D fused obstacle/risk grid |
| `/nav/terrain_map` | `lingtu.dds.PointCloud2` | terrain analysis cloud with fields `x,y,z,intensity`; `intensity` is height above ground |
| `/nav/terrain_map_ext` | `lingtu.dds.PointCloud2` | same schema, plus removed rolling dynamic candidates for diagnostics; removed candidates use negative intensity and are ignored by the native planner obstacle merge |

The terrain cloud is generated by `nav_kernel::TerrainAnalysisCore`, not by the
older simplified z-quantile helper in the DDS service. The slow terrain map path
then passes through `nav_kernel::DynamicClearCore`:

```text
rt/slam/registered_cloud
  -> current 10 Hz traversability grid
  -> TerrainAnalysisCore rolling terrain cache
  -> DynamicClearCore rolling free-space/raycast stale-residue filter
  -> rt/nav/terrain_map      kept points for local planning
  -> rt/nav/terrain_map_ext  kept points plus removed candidates for display/debug
```

This does not delete points from the raw LiDAR or SLAM streams. It only filters
the local planning terrain product so stopped scans do not leave stale obstacle
residue. `lt-nav` treats negative-height `terrain_map_ext` points as a
diagnostic overlay and does not merge them into the planner obstacle cloud.
Current runtime settings on sunrise:

```text
LINGTU_TRAVERSABILITY_PUBLISH_HZ=10
LINGTU_TRAVERSABILITY_TERRAIN_MAP_HZ=5
LINGTU_TRAVERSABILITY_TICK_HZ=50
LINGTU_TRAVERSABILITY_RADIUS=6
LINGTU_TRAVERSABILITY_MAX_POINTS=5000
LINGTU_TRAVERSABILITY_TERRAIN_CACHE_MAX_POINTS=20000
LINGTU_TRAVERSABILITY_TERRAIN_CLEAR_DY_OBS=1
LINGTU_TRAVERSABILITY_TERRAIN_NO_DATA_OBSTACLE=0
LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR=1
LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR_VOXEL_SIZE=0.20
LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR_WEAK_TTL_S=0.80
LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR_STATIC_TTL_S=3.0
LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR_RAYCAST=1
LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR_RAYCAST_MIN_FRAMES=2
LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR_RAYCAST_MAX_RANGE=6.0
```

`LINGTU_TRAVERSABILITY_MAX_POINTS` limits the published terrain cloud before DDS
write. This keeps `/nav/terrain_map(_ext)` fresh enough for `lt-nav`
while retaining the rolling terrain cache internally.
`LINGTU_TRAVERSABILITY_TERRAIN_CACHE_MAX_POINTS` is a hard global limit on the
rolling cache. Compaction keeps the highest point in each compacted cell, uses
newer data as the tie-breaker, and prioritizes voxels nearest the robot when the
global limit is smaller than the active area.
`LINGTU_TRAVERSABILITY_TERRAIN_CLEAR_DY_OBS` enables dynamic-obstacle evidence
inside the terrain core. `LINGTU_TRAVERSABILITY_DYNAMIC_CLEAR` enables the
separate rolling clear layer after terrain analysis. The raycast mode marks
voxels between the robot pose and current returns as observed free; stale
terrain points inside those repeatedly cleared voxels are removed before they
reach local planning. `cache_points`, slow-path timing, and loop overrun are
reported in the traversability status file so a frequency regression is visible
without disabling the safety layer.

`prune` is not in this realtime path. It is the optional saved-map cleaner used
after a mapping run, before rebuilt occupancy and OctoMap artifacts are
accepted. It should not be treated as better than DUFOMap or
ERASOR2; those are stronger references for map cleaning.

Realtime dynamic handling has two native C++ layers:

| Layer | File | Stage | Role |
| --- | --- | --- | --- |
| `DynamicClearCore` | `src/nav/cpp/include/nav_kernel/dynamic_clear_core.hpp` | traversability producer | filters stale rolling terrain points before publishing `rt/nav/terrain_map` |
| dynamic obstacle layer / `MotionLayer` | `src/nav/cpp/endpoint/nav/input/obstacle.*` | nav consumer | keeps explicit unknown/free/occupied/static/cleared voxel evidence plus separate moving-object tracks |

`MotionLayer` is the realtime obstacle layer. It raycasts free space from the
current robot pose to current scan endpoints, does not clear cells hidden behind
a nearer endpoint, requires repeated free evidence before demoting static
structure, and preserves current dynamic objects in the measured obstacle
snapshot for collision avoidance. Confirmed moving tracks also produce a bounded
one-second constant-velocity future-occupancy envelope. Measured obstacles and
terrain are merged first; future occupancy is stored separately and appended
afterwards, with at most 800 points. Prediction therefore adds risk but can
never consume or replace the measured-obstacle budget. A confirmed track
survives a brief missed scan until its 0.6 s track TTL expires, so one occluded
frame does not immediately remove the predicted risk.

This is a conservative possible-occupancy envelope, not time-aligned TTC,
velocity-obstacle negotiation, or MPC. It affects LocalPlanner and the final
path safety gate. It intentionally does not enter the global active-path replan
admission, which continues to use current measured blockage only. Native status
reports `motion_layer.predicted_points`, `prediction_clusters`, and
`prediction_horizon_s` so field tests can prove whether prediction participated.
Registered clouds use message-time pose
sampling from `PoseBuffer`; a cloud without a pose inside the configured gap is
rejected and cannot refresh the navigation input gate. Producer header stamps
must be finite, positive, and ordered inside their source epoch; a large rollback
opens a new source epoch. They are not compared with the receiver wall clock.
Odom, TF, cloud, traversability, localization-health, and driver-control
freshness instead use local steady-clock receipt timestamps, including the
future/stale tolerance checks. Moving-object tracks expire after their TTL even
when no newer scan arrives. The endpoint uses `MotionLayer` directly; there is
no second compatibility wrapper or duplicate obstacle algorithm.

## `lt-nav` Internal Nodes

`lt-nav.service` runs one binary:

```text
/opt/lingtu/current/build/nav_endpoint/navd
```

That binary contains the native planning and command pipeline:

| Internal node | Process | Responsibility | Inputs | Outputs |
| --- | --- | --- | --- | --- |
| Command receiver | `navd` | Validate typed goal/cancel/teleop requests and emit business ACK | `/nav/command/request` | internal command + `/nav/command/ack` |
| OctoPlanner3D global planner | `navd` | Plan a 3D saved-map route | current `map` pose, goal, `octomap.ot` | internal global path |
| Global path publisher | `navd` | Publish accepted global path | internal global path | `/nav/global_path` |
| Executor target selector | `navd` | Pick the next lookahead target from global path | current pose, global path | internal target waypoint |
| local::Planner | `navd` | Generate near-field local path | target, current pose, obstacle cloud, `/nav/traversability` | internal local path |
| Local path publisher | `navd` | Publish the local path | internal local path | `/nav/local_path` |
| PathFollowerCore | `navd` | Convert local path to velocity | local path, follower state | internal `cmd_vel` |
| Waypoint publisher | `navd` | Publish current target waypoint | internal target waypoint | `/nav/way_point` |
| CmdVel publisher | `navd` | Publish speed command when enabled | internal `cmd_vel`, `LINGTU_NAV_PUBLISH_CMD_VEL=1` | `/nav/cmd_vel` |

Runtime order inside `navd`:

```text
/nav/command/request (kind=goal)
  -> validate request_id and control mode
  -> OctoPlanner3D with octomap.ot
  -> nav.setRoute(...)
  -> /nav/global_path
  -> Executor::tick(...)
  -> local::Planner::plan(...)
  -> PathFollowerCore::computeControl(...)
  -> ExecutionOutput {local_path, waypoint, cmd_vel}
  -> DDS /nav/command/ack + /nav/local_path + /nav/way_point + /nav/cmd_vel
```

There is no DDS hop between `local::Planner` and `PathFollowerCore`. They run
sequentially in the same `Executor::tick()` call. DDS publishes the accepted
global path, local-path telemetry, waypoint telemetry, and the final command;
it is not the internal planner-to-follower transport.

The native chain enforces these invariants:

- Target reacquisition searches the complete forward remainder of the global
  path, so a localization jump cannot trap the selector in the next 12 points.
- Final-goal slowdown and stop use distance to the global goal. The end of the
  rolling local path is not treated as mission completion.
- Acceleration limiting uses measured control-loop elapsed time, clamped by the
  follower configuration, rather than assuming a fixed 100 Hz loop.
- Goal completion reports the actual final global-path point and emits a zero
  command before the endpoint deactivates the path.
- Path, goal, grid, odometry, and teleop ingress pass an explicit frame gate.
  `odom` paths/goals are transformed to `map`; unsupported frames fail closed.
- In `autonomy`, goals/paths are accepted until a deliberate non-zero teleop
  request latches operator takeover. The endpoint first cancels in-flight
  planning, clears the old path, and writes a zero-command barrier. While the
  latch is held, new goals/paths and late planner completions are rejected.
  `resume_autonomy` only releases the latch; it never restores the old path,
  so the caller must submit a fresh goal/path from the current robot pose. The
  resume command timestamp becomes a strict not-before boundary for subsequent
  goals and external paths, preventing queued pre-takeover requests from being
  reactivated after resume.
- `/ws/teleop` accepts physical `vx_mps`, `vy_mps`, and `yaw_rps` values and
  requires `deadman: true` on every motion sample. Each
  WebSocket connection a unique, expiring control lease. Missing/false deadman,
  disconnect, or command timeout produces a zero command and keeps the endpoint
  in manual hold. A zero-command delivery failure is reported as rejected and
  the native takeover latch remains fail-closed.
- In `teleop`, goals/paths are rejected and odometry/cloud are not required;
  command age and speed limits remain active.
- In `teleop_avoid`, goals/paths are rejected and fresh odometry/cloud are
  required before assisted LocalPlanner output can be published. A candidate
  detour is checked again along the planned curved path; this final safety gate
  may slow or stop it. No-path is always a zero command with recovery disabled.
- The same assisted branch is available during latched operator takeover in
  `tracking`, `nav`, `inspection`, and saved-map `explore`. Non-zero teleop first
  cancels/invalidates the old autonomous plan; releasing the deadman does not
  silently restore that plan. Explicit `resume_autonomy` plus a fresh goal/path
  is required.

`tick_hz` and `max_accel_mps2` are different quantities. At the field default
of 20 Hz, the nominal control period is `0.05 s`. With
`LINGTU_NAV_MAX_ACCEL_MPS2=1.0`, one normal tick may change linear speed by at
most `1.0 * 0.05 = 0.05 m/s`. Runtime status exposes `max_speed_mps`,
`max_accel_mps2`, and `nominal_dt_s` under `path_follower`. This ramp applies
to normal tracking speed changes; a safety stop bypasses the comfort ramp and
commands zero immediately. It limits the scalar tracking speed, not yaw
acceleration or the full 2D velocity-vector derivative. Those remain separate
future constraints if field tuning shows that the robot needs them.

The isolated-DDS and field evidence for these rules is recorded in
`docs/07-testing/field-runs/2026-07-10-local-planner-path-follower.md`.

`Executor::tick(...)` only runs when both are true:

```text
map_body pose is available
has_path == true
```

If `has_path=false`, the local planner and path follower do not enter an
effective tick.

## Motion Mock DDS

For no-hardware validation, the build now includes:

```text
/opt/lingtu/current/build/nav_endpoint/lingtu_motion_mock_dds
```

It is not a field service. It is a manual test/simulation node for an isolated
DDS domain.

| Direction | Topic | Type | Behavior |
| --- | --- | --- | --- |
| subscribe | `/nav/cmd_vel` | `lingtu.dds.FinalVelocityCommand` | validate identity/freshness, clamp, and integrate body-frame `vx,vy,wz` |
| publish | `/slam/odometry` | `lingtu.dds.Odometry` | simulated `odom -> body` pose |
| publish | `/tf` | `lingtu.dds.TFMessage` | identity `map -> odom` |
| file | status JSON | `lingtu.motion_mock.status.v1` | pose, last command, counters |

The expected smoke topology is:

```text
domain 77:
  lingtu_motion_mock_dds
    -> /slam/odometry + /tf
  navd
    -> /nav/cmd_vel
  lingtu_motion_mock_dds
    -> integrated simulated pose
```

This proves navigation command generation and DDS communication without
touching the real robot hardware endpoint.

## Field Frames And TF

Current field frame contract:

| Frame | Meaning |
| --- | --- |
| `map` | saved/global map frame used by OctoPlanner3D and global paths |
| `odom` | continuous local odometry frame |
| `body` | normalized robot body frame |
| `base_link` | body alias accepted at compatibility boundaries |
| `lidar_link` | normalized LiDAR frame entering LingTu |
| `livox_frame` | physical Livox frame before normalization |
| `camera_link` | camera frame used by perception/inspection |

Required TF links:

| Link | Producer | Consumer | Required for |
| --- | --- | --- | --- |
| `map -> odom` | SLAM/localizer | `lt-nav`, Gateway, evidence gates | converting odometry pose into map frame for saved-map planning |
| `odom -> body` | SLAM/localizer | local planning, path following, Gateway | current robot pose and yaw |
| `body -> lidar_link` | calibration/static TF from SLAM runtime | cloud normalization and evidence gates | aligning LiDAR points to body/map |
| `body -> camera_link` | calibration/static TF | perception/inspection | camera geometry |

Topic frame expectations:

| Topic | Expected frame |
| --- | --- |
| `/lidar/raw_frame` | `lidar_link` |
| `/imu/raw` | `imu_link` |
| `/slam/odometry` | header `odom`, child `body` |
| `/slam/registered_cloud` | `body` |
| `/slam/map_cloud` | `map` |
| `/nav/traversability` | `map` or `odom` |
| `/nav/global_path` | `map` |
| `/nav/local_path` | `map`, `odom`, or `body` |
| `/nav/way_point` | `map` or `odom` |
| `/nav/cmd_vel` | `body` |

Sunrise read-only evidence on 2026-07-05 observed these TF links as OK:

| Evidence link | Observed | Samples | Status |
| --- | --- | ---: | --- |
| `map_to_odom` | `map -> odom` | 8 | OK |
| `odom_to_body` | `odom -> body` | 8 | OK |
| `body_to_lidar` | `body -> lidar_link` | 8 | OK |
| `body_to_camera` | `body -> camera_link` | 8 | OK |

The same evidence observed required topic frames as valid, including
`/nav/local_path` in `body` frame, which is allowed by the runtime contract.
Current TF is therefore not the blocker for cmd_vel-only validation.

## Sunrise Status Snapshots

### 2026-07-10 Terrain And Motion Performance Fix

The original `slow_terrain ~= 3627 ms` was caused by an unbounded rolling
terrain store growing to roughly 900k points. Each slow cycle expanded and
traversed that store, then `DynamicClearCore` traversed it again inside the DDS
callback. The nav endpoint also rebuilt the same obstacle snapshot on every
20 Hz tick and repeated voxel pruning within one scan.

After bounding terrain points per voxel, limiting and deduplicating ray work,
reusing same-frame prune results, and using a one-pass bounded obstacle sample,
the sunrise field result is:

```text
registered cloud:            10.0 Hz
traversability:              10.0 Hz
terrain map:                  5.0 Hz
terrain slow path:        12-13 ms
terrain core:               2-3 ms
dynamic clear:            10-11 ms
nav motion update:        13-18 ms
nav obstacle snapshot:      2-7 ms
nav process CPU:            ~18%
traversability process CPU: ~10%
terrain cache:        9712/20000 points
input gate:                  ready
static-scene dynamic tracks: 0
cloud timestamp rejected:    0
cloud pose rejected:         0
```

`terrain_map_ext` remains diagnostics-only and is never merged into planner
obstacles; its endpoint merge share is fixed to `0.0` and status reports
`terrain_map_ext_diagnostics_only=true`. Obstacle inflation is applied once by the local planner footprint;
the live voxel layer radius is zero to avoid double inflation.

### 2026-07-06 Terrain And Mock Update

Live domain `0` after the terrain migration:

```text
lt-lidar.service active
lt-slam.service active
lt-terrain.service active
lt-nav.service active
SLAM state = TRACKING
relocalization_state = completed
traversability publish_hz = 10
terrain_points ~= 16k..20k after DDS publish limiting
nav has_odom = true
nav has_map_odom_tf = true
nav has_traversability = true
nav has_terrain_map_ext = true
```

A live-domain external path currently triggers local near-field safety:

```text
global_path_points = 2
local_path_points > 0
last_local.reason = near_field_stop
last_local.cmd_vel = {"vx": 0, "vy": 0, "wz": 0}
```

That is a safety/local-environment result, not a DDS or path-follower transport
failure. The endpoint still publishes `/nav/cmd_vel` messages; the value is zero
because the local planner reports a near-field stop.

Isolated domain `77` mock smoke:

```text
nav publish_cmd_vel = true
nav last_local.reason = control_ready
nav last_local.cmd_vel = {"vx": 0.3, "vy": 0, "wz": 0}
nav cmd_vel_published = 30
motion mock cmd_vel received = 84
motion mock pose.x = 0.854793
real_robot_motion = false
cmd_vel_sent_to_hardware = false
```

This proves the route through `/nav/cmd_vel` and a simulated robot-motion
consumer. It does not claim that the real Thunder brainstem/CAN outlet was used.

### 2026-07-05 Status Snapshot

Latest read-only status:

```text
has_odom=true
has_map_odom_tf=true
has_traversability=true
publish_cmd_vel=false
global_path_points=0
local_path_points=0
cmd_vel_published=0
last_local.reason=not_seen
```

Interpretation:

```text
SLAM, TF, registered cloud, and traversability are online.
No active global path is currently loaded.
Because has_path=false, Executor::tick(...) is not effectively running.
Because publish_cmd_vel=false, /nav/cmd_vel will not be published even if a
local command is computed.
```

The next cmd_vel-only validation should ignore real motion and check only:

```text
/nav/command/ack accepted for goal request
  -> /nav/global_path non-empty
  -> /nav/local_path non-empty
  -> /nav/cmd_vel has non-zero samples
```

## Core Runtime Types

| Type | Required fields |
| --- | --- |
| `PointCloud2` | `points: float32 Nx3/Nx4`, `ts`, `frame_id`, `height`, `width`, `fields`, `is_dense` |
| `MapCloudFrame` | `points`, `mode`, `ts`, `frame_id`, `map_id`, `source`, `sequence`, `metadata` |
| `Imu` | `orientation`, `angular_velocity`, `linear_acceleration`, covariances, `ts`, `frame_id` |
| `Odometry` | `pose`, `twist`, `ts`, `frame_id`, `child_frame_id` |
| `PoseStamped` | `pose`, `ts`, `frame_id` |
| `Path` | `poses: list[PoseStamped]`, `ts`, `frame_id` |
| `Twist` | `linear: Vector3`, `angular: Vector3` |
| `OccupancyGrid` | `grid: int8 HxW`, `resolution`, `origin`, `ts`, `frame_id` |

`dict` payloads are used for diagnostics, risk grids, and metadata-heavy state:
`localization_status`, `mission_status`, `traversability`, `fused_cost`,
`esdf_field`, internal `slope_grid`, and `control_hint`.  Internal slope data
is not part of the Gateway/Web scene contract.

## OctoPlanner3D Map Inputs

OctoPlanner3D is the product global planner. It does not consume the live local
planner terrain stream. It consumes a saved map artifact bundle.

Required for a valid OctoPlanner3D plan:

| Input | Required | Source | Purpose |
| --- | --- | --- | --- |
| `octomap.ot`, `octomap.bt`, or `.octomap` | yes | built from saved `map.pcd` | 3D occupancy tree used by OctoPlanner3D |
| `metadata.json` | yes | MapPipelineCore / SaveMapEngine | records frame and built-artifact status inside the canonical map package |
| `map.pcd` | yes for artifact gate | SLAM map save | source map used by the saved-map package |
| metadata `frame_id` | yes | metadata | must match expected saved-map/planning frame, normally `map` |
| metadata `artifacts.map_pcd.point_count` | yes | metadata | must be positive |
| metadata `artifacts.octomap.uri` | yes | metadata | points to the OctoMap file |
| native artifact `exists` / `format_ok` | yes | mapd | verifies required files are present and readable in their declared format |
| metadata artifact source/profile/frame fields | yes | metadata | must be consistent across required artifacts |
| `occupancy.npz` | optional | MapService | loaded when present as static 2D occupancy/preblocked context |

The active map snapshot and goal are passed to the selected global planner by a
direct C++ call inside `navd`. The verified result is retained by `Executor`
and published as `/nav/global_path` telemetry.

## Local Planning, Tracking, and Command Output

One local planning tick combines the active route slice or assisted-motion
intent with coherent odometry, transform, obstacle, collision, and optional
traversability evidence. CMU returns a geometric local path; SCAN also returns
the exact B-spline used by the follower. Sampled trajectory points remain
telemetry. The handoff stays in memory.

Final command path:

```text
navd Executor
  -> CMU or SCAN Local Planner
  -> native Follower
  -> final safety and control authority
  -> rt/nav/cmd_vel
  -> lingtu-driver
  -> Brainstem WalkChecked(seq, Vector3)
```

`/nav/local_path` is output telemetry. The Host does not subscribe to it to run
another follower or publish another velocity command.

## What Must Be Validated On The Robot

Minimum field proof for the navigation base:

1. DDS receives `/lidar/raw_frame`, `/imu/raw`, `/slam/odometry`, and `/slam/map_cloud`.
2. `map save` creates `map.pcd`, `octomap.ot`, `occupancy.npz`, and `metadata.json`.
3. OctoPlanner3D artifact gate reports `ok: true`.
4. `Navigation` publishes a non-empty `global_path`.
5. `LocalPlanner` publishes a non-empty `local_path`.
6. `PathFollower` publishes non-zero `cmd_vel`.
7. The native command arbiter is the only `/nav/cmd_vel` writer.
8. Native safety is not holding a hard stop.
9. `lingtu-driver` receives `/nav/cmd_vel`, owns the `grpc` Brainstem lease as
   `lingtu-driver`, and Brainstem accepts `WalkChecked`.
