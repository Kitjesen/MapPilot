# Navigation Runtime Dataflow

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
on the robot (`lingtu_slam_cyclone_runtime` / `lingtu-slam-dds.service`) or an
explicit external native SLAM service. Python code may adapt streams, status,
reports, and test evidence only; it must not become a SLAM backend.

For Windows-hosted MuJoCo acceptance, replay records cross into the WSL-native
Livox DDS publisher with `--restamp-stdin-records`. The C++ publisher rebases
the first record to its own system clock and preserves all relative LiDAR/IMU
timing and per-point offsets. Real MID-360 input never uses this replay option.

| Boundary | Transport | Payload shape | ROS 2 required |
| --- | --- | --- | --- |
| Hardware / external runtime to LingTu | typed DDS | IDL structs under `message.dds_types` | no |
| LingTu module to module, same process | `Out.publish()` to wired `In._deliver()` callback | Python objects | no |
| LingTu worker subprocess boundary | SHM transport when enabled | serialized runtime messages | no |
| Field OctoPlanner3D global planner | direct C++ call inside `lingtu_nav_native_endpoint` | `PlanRequest` / `PlanResult` in memory | no |
| Dev/compat OctoPlanner3D wrapper | subprocess stdin/stdout | JSON request/result | no |
| Map artifact conversion | subprocess + files | `map.pcd` to `octomap.ot` | no |
| Command output to Thunder brainstem | typed DDS then native `driver` gRPC client | `TwistStamped` to Brainstem `Walk(Vector3)` | no |

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
| C++ command client -> nav | `/nav/command/request` | `lingtu.dds.NavigationCommandRequest` | typed goal, cancel, or operator velocity request with `request_id` |
| nav -> C++ command client | `/nav/command/ack` | `lingtu.dds.NavigationCommandAck` | authoritative admission acceptance/rejection for the matching request; not task completion |
| map/terrain control -> nav | `/nav/map_clearing` | `lingtu.dds.Bool` | clear map-derived planner caches |
| map/terrain control -> nav | `/nav/cloud_clearing` | `lingtu.dds.Bool` | clear near-field obstacle caches |
| traversability -> nav | `/nav/traversability` | `lingtu.dds.OccupancyGrid` | local terrain risk grid |
| traversability -> nav | `/nav/terrain_map` | `lingtu.dds.PointCloud2` | planner terrain points `(x,y,z,height)` |
| traversability -> diagnostics | `/nav/terrain_map_ext` | `lingtu.dds.PointCloud2` | diagnostics-only terrain overlay |
| nav -> UI/recorders | `/nav/global_path` | `lingtu.dds.Path` | accepted OctoPlanner3D path |
| nav -> UI/recorders | `/nav/local_path` | `lingtu.dds.Path` | current local-plan telemetry, not an internal follower input |
| nav -> UI/recorders | `/nav/way_point` | `lingtu.dds.PoseStamped` | current look-ahead target |
| nav -> Thunder driver | `/nav/cmd_vel` | `lingtu.dds.TwistStamped` | final post-gate command; nav endpoint is the single writer |

Natural-language instructions are not a robot DDS topic. Gateway and MCP feed
`SemanticPlanner` in-process; only its resolved goal enters the typed C++ DDS
command request. The legacy goal/cancel/teleop topics remain read-only
compatibility inputs and are not written by the product client.

The Thunder field service sets `LINGTU_NAV_COMMANDS_REQUIRED=1` and
`LINGTU_TELEOP_CMD_DDS=1`. If the persistent native command client is missing
or the endpoint does not acknowledge a request, Gateway rejects it. It never
falls back to Python `goal_pose` or `cmd_vel` publication in the field profile.
Local callback fallbacks exist only when a dev/sim/compat profile explicitly
leaves those field requirements disabled.

## Product Command Chains

### Autonomous Navigation

```text
Web coordinate goal / CLI / MCP resolved goal
  -> process-wide NavigationCommandClient
  -> liblingtu_nav_client.so
  -> DDS /nav/command/request (kind=goal, request_id=...)
  -> lingtu_nav_native_endpoint (control_mode=autonomy)
  -> validate frame, authority, localization, map, and goal admission
  -> DDS /nav/command/ack (accepted=true, reason=planning_started)
  -> OctoPlanner3D::runPlan(request)
  -> NavLoop::setGlobalPath(path)
  -> NavLoop::tick(...)
       -> select look-ahead point from global path
       -> LocalPlannerCore::plan(obstacles, traversability)
       -> PathFollower::computeControl(local_path_body)
  -> DDS /nav/cmd_vel
  -> Thunder native driver -> Brainstem Walk
```

The command ACK closes command submission, not navigation execution. For a
goal, `accepted=true` means the authoritative endpoint admitted the request and
started asynchronous planning. Planner failure, cancellation, path progress,
and goal completion are reported by navigation status/path outputs; an accepted
ACK never means that a path exists or that the robot arrived.

The global path and local path are ordinary C++ vectors inside this endpoint.
There is no DDS or LCM hop between OctoPlanner3D, LocalPlanner, and
PathFollower.

Input freshness is checked before `NavLoop::tick()`, collision/traversability
is checked while selecting the local path, PathFollower applies speed and
acceleration limits, and the resulting non-zero autonomous command passes the
same independent `CommandSafety` stop/slow/limit gate used for operator
requests before `/nav/cmd_vel` can be published.

`/nav/local_path` is not the control hand-off. `LocalPlannerCore::plan()`
returns `local_path_body` in memory and `PathFollower::computeControl()`
consumes that vector in the same `NavLoop::tick()`. The DDS local-path writer
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
Web joystick / hand controller / MCP velocity request
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
  -> NavLoop::tickTeleopIntent()
  -> LocalPlannerCore::planIntent() (operator direction/speed, no recovery)
  -> PathFollowerCore
  -> curved-path final safety gate
  -> DDS /nav/cmd_vel
```

`teleop_avoid` fails closed without current localization or obstacle context.
With `LINGTU_TELEOP_LOCAL_PLANNER=1`, translational joystick intent is a short
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

Inspection, patrol, object tracking, and TARE/frontier exploration do not own a
second motion stack. They select lifecycle actions, the next goal, or an
explicit path, then reuse the autonomous chain above:

```text
inspection/patrol scheduler  -> /nav/inspection/command -> native inspection executor -> autonomous chain
TARE/frontier exploration    -> NavigationCommandClient -> typed goal request -> autonomous chain
semantic instruction         -> SemanticPlanner -> resolved goal -> typed request -> autonomous chain
explicit tracking path       -> /nav/global_path -> NavLoop -> LocalPlanner -> PathFollower
```

Mapping is different: LiDAR/IMU -> SLAM -> `map.pcd` -> map artifacts. The map
profile does not start the nav endpoint and does not publish motion commands.

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

`NavLoop::tick()` calls `LocalPlannerCore::plan()` and immediately passes the
returned `local_path_body` to `computeControl()` in the same call. The endpoint
then publishes `local_path_map` for Web, diagnostics, recording, and acceptance
tests. Re-subscribing to `/nav/local_path` inside the same process would add
latency, duplicate ownership, and a stale-path race.

## Module Fallback Dataflow

The table below describes the Python Module/dev compatibility graph. It is not
the Thunder field command path when `lingtu-nav-dds.service` owns navigation.

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
| 9 | MapsServiceCore / SaveMapEngine | version staging area | saved source map | `map.pcd` | C ABI -> native transaction |
| 10 | MapPipelineCore | committed map version | OctoMap artifact | `octomap.ot` or explicit compatibility `octomap.bt` | native builder or C++-owned converter |
| 11 | SaveMapEngine | committed map version | manifest, checksums, provenance | `save_manifest.json`, `metadata.json` | atomic version commit |
| 12 | MapPipelineCore | committed map version | occupancy, ESDF, traversability | typed map artifacts | native C++ build |
| 13 | Gateway/MCP/DDS | GoalService or Navigation | goal | `PoseStamped` | HTTP/MCP/DDS then callback |
| 14 | Navigation | OctoPlanner3D runtime | global plan request | JSON | subprocess |
| 15 | OctoPlanner3D runtime | Navigation | global path result | JSON -> `Path` | subprocess |
| 16 | Navigation | LocalPlanner | global path | `Path` | callback |
| 17 | Navigation | LocalPlanner | current waypoint | `PoseStamped` | callback |
| 18 | Terrain | LocalPlanner | near-field terrain cloud | `PointCloud2` | callback |
| 19 | Terrain | LocalPlanner | traversability | `dict` | callback |
| 20 | Map layers | TraversabilityCost | occupancy/elevation/ESDF | `OccupancyGrid`, `dict` | callback |
| 21 | TraversabilityCost | Navigation | global risk gate | `dict` | callback |
| 22 | TraversabilityCost | LocalPlanner | ESDF relay | `dict` | callback |
| 23 | LocalPlanner | PathFollower, SafetyRing, Gateway | local path | `Path` | callback |
| 24 | LocalPlanner | PathFollower | control hint | `dict` | callback |
| 25 | PathFollower | CmdVelMux | autonomous velocity | `Twist` | callback |
| 26 | Teleop / VisualServo / Navigation recovery | CmdVelMux | override or recovery velocity | `Twist` | callback |
| 27 | command arbiter | native `driver` | final velocity | DDS `TwistStamped` | DDS |
| 28 | native `driver` | Brainstem | normalized walk command | gRPC `Walk(Vector3)` | gRPC |
| 29 | SafetyRing / Geofence | Navigation and driver | stop command | `int` (`0`, `1`, `2`) | callback |

## Live Map Cloud Cleanup

LingTu now uses column carving on the live map cloud path. The goal is the same
operator-facing behavior as the DimOS/Unitree-style sparse voxel view without
making Open3D or ROS part of the runtime contract.

```text
/slam/map_cloud
  -> VoxelGridModule: CPU voxel hash, 5-15 cm class resolution, XY column replace
  -> /maps/scene + /maps/voxel_cloud + voxel_map stats

/slam/map_cloud
  -> GatewayModule: raw display fallback only when no clean map layer is fresh
  -> voxel downsample + hit-count dynamic filter
  -> /ws/cloud binary point cloud + small SSE metadata

VoxelGridModule.scene
  -> GatewayModule.map_scene
  -> /ws/cloud binary point cloud + small SSE metadata

VoxelGridModule.voxel_cloud
  -> GatewayModule.voxel_cloud legacy compatibility input
  -> /ws/cloud binary point cloud + small SSE metadata
```

Rules:

- In `mapping` and `exploring`, a newly observed `(x, y)` voxel column replaces
  old `z` voxels in that column before the new frame is inserted. This clears
  stale people, old obstacle surfaces, and map smear when LiDAR revisits space.
- In `navigating` and `idle`, Gateway does not accumulate; it keeps the latest
  scan because saved-map visualization is the stable base layer.
- `VoxelGridModule` enables `column_carving` by default. Stack config can set
  `voxel_column_carving=false` or `voxel_grid.column_carving=false` for tests
  that need pure accumulation.
- Gateway does not own column carving. It prefers a fresh clean map layer for
  `LINGTU_CLEAN_MAP_LAYER_PREFER_S` seconds and uses raw `map_cloud` only as a
  compatibility fallback.
- `VoxelGridModule` defaults to a 5 cm voxel edge. Gateway's browser-facing
  accumulator defaults to 15 cm for memory/bandwidth safety; set
  `LINGTU_MAP_VIEWER_VOXEL_SIZE=0.05` when the operator view should mirror a
  5 cm preprocessed Unitree/DimOS-style stream.
- This is a ROS-free native C++ implementation exposed through thin Module
  ports. It is not Open3D `VoxelBlockGrid`; the contract is the behavior, not
  that dependency.
- Runtime local-planner ghost suppression and saved-map cleanup are separate
  stages. Runtime suppression belongs in `lingtu_traversability_dds`:
  `nav_kernel::TerrainAnalysisCore` builds the current terrain product and
  `nav_kernel::DynamicClearCore` removes stale rolling residues using current
  hits, short TTL evidence, and raycast free-space evidence while keeping current
  obstacles in `rt/nav/traversability`. Save-time cleanup belongs in
  `src/maps/prune/cpp/prune`: it is optional offline map-asset cleanup for
  persistent `map.pcd`, not the realtime dynamic obstacle solution.
- The C++ `lingtu-nav-dds` endpoint owns a second, shorter-horizon realtime
  `MotionLayer` for planner safety. It consumes the current registered cloud
  plus terrain products, ray-clears from the configured LiDAR origin
  (`body pose + LINGTU_NAV_SENSOR_OFFSET_X/Y/Z_M`), exposes explicit sparse
  cell states (`Unknown`, `Free`, `Occupied`, `Static`, `Cleared`), and keeps
  moving-object classification as a separate current-frame track label. A
  moving candidate must occupy space that was repeatedly observed free, move
  consistently for several frames, and remain inside configured speed and
  height limits. Current moving objects remain planner obstacles; only stale
  residues are cleared. The exact planner-used state is published in
  `/dev/shm/lingtu/nav_endpoint_status.json`.
- `src/maps` is the native map domain for reusable spatial products and saved
  map assets: source `map.pcd`, occupancy, OctoMap/voxel artifacts, ESDF,
  traversability artifacts, active-map control, health, and bundle queries.
  It must not own the 20 Hz endpoint-local collision cache. Conversely,
  `MotionLayer` must not become a saved-map asset manager.

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
| `live_cloud` | point cloud | WebSocket | `voxel_cloud` preferred, raw `map_cloud` fallback | clean live map |
| `costmap` | raster texture | SSE | `costmap` event | navigation risk |
| `slope` | raster texture | SSE | `slope_grid` event | terrain slope |
| `path` | polyline | SSE | `global_path` / `local_path` events | navigation plan |
| `robot` | pose marker | SSE | `odometry` event | robot pose |

Web clients should treat `live_cloud.source=voxel_cloud_preferred` as the clean
map product. If only raw `map_cloud` is available, Gateway may show the raw
fallback but must not claim it is the canonical live map.

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
- `src/gateway/visualization/rerun_bridge.py` logs `world/point_cloud` as
  Rerun `Points3D` with per-point colors and radii.

For the `thunder_field` product endpoint, the DDS navigation boundary is owned
by the C++ `lingtu-nav-dds` service, not Python `nav.in` / `nav.out` adapters.
It subscribes to `rt/nav/command/request`, `rt/slam/odometry`,
`rt/slam/registered_cloud`, `rt/nav/traversability`,
`rt/nav/terrain_map`, and `rt/nav/terrain_map_ext`; it publishes
`rt/nav/command/ack`, `rt/nav/global_path`, `rt/nav/local_path`, `rt/nav/way_point`, and
`rt/nav/cmd_vel`.

## Sunrise Native Navigation Services

This is the current robot-side service map for the sunrise board. These are
systemd process boundaries, not necessarily one algorithm per service.

| Service | Current state on 2026-07-05 | Responsibility | Inputs | Outputs | Required for `/nav/cmd_vel` |
| --- | --- | --- | --- | --- | --- |
| `nav-lidar-network.service` | active, exited | Configure LiDAR Ethernet (`eth1`, `192.168.1.5/24`) | none | LiDAR network reachable | yes |
| `lingtu-livox-dds.service` | active | Livox MID-360 and IMU DDS producer | Livox hardware | `/lidar/raw_frame`, `/imu/raw` | yes |
| `lingtu-slam-dds.service` | active | Native SLAM/localization DDS runtime | `/lidar/raw_frame`, `/imu/raw` | `/slam/odometry`, `/slam/registered_cloud`, `/slam/map_cloud`, `/tf`, localization health/quality | yes |
| `lingtu-traversability-dds.service` | active, disabled | Native traversability grid producer | `/slam/odometry`, `/slam/registered_cloud` | `/nav/traversability` | yes for obstacle/risk-aware local planning |
| `lingtu-nav-dds.service` | active | Native navigation endpoint: global planning, local planning, path following, DDS output | odometry, TF, goal, traversability, cloud, OctoMap | `/nav/global_path`, `/nav/local_path`, `/nav/way_point`, `/nav/cmd_vel` | yes |
| `lingtu.service` | active | Python Gateway/API/MCP/task/status process | user/task commands, module state | Gateway `5050`, MCP, navigation command entry | yes for external command entry |
| `lingtu-driver.service` | product default | Native hardware command sink | `/nav/cmd_vel` | Brainstem/hardware command | no for cmd_vel-only validation |
| `lingtu-thunder-dds-endpoint.service` | compatibility only | Python command sink | `/nav/cmd_vel` | Brainstem/hardware command | no |
| `robot-brainstem.service` | not found / inactive | Real robot low-level control bridge | hardware command | robot control | no for cmd_vel-only validation |
| `can-setup.service` | failed | CAN interface setup | none | `can0..can3` available | no for cmd_vel-only validation |
| `robot-camera.service` | not found / inactive | Camera source | Orbbec/camera hardware | camera streams | no for LiDAR-only navigation; needed for inspection |
| `lingtu-thunder-lite.service` | not found / inactive | legacy/light Thunder path | mixed | mixed | no |

The field navigation process is:

```text
nav-lidar-network
  -> lingtu-livox-dds
  -> lingtu-slam-dds
  -> lingtu-traversability-dds
  -> lingtu-nav-dds
  -> /nav/cmd_vel
```

`lingtu-driver`, `robot-brainstem`, and `can-setup` are below
`/nav/cmd_vel`. They matter for real motion, not for proving the navigation
stack can produce a speed command.

## Native Traversability DDS

`lingtu-traversability-dds.service` runs:

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
residue. `lingtu-nav-dds` treats negative-height `terrain_map_ext` points as a
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
write. This keeps `/nav/terrain_map(_ext)` fresh enough for `lingtu-nav-dds`
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
after a mapping run, before rebuilt artifacts such as occupancy, OctoMap, and
tomogram are accepted. It should not be treated as better than DUFOMap or
ERASOR2; those are stronger references for map cleaning.

Realtime dynamic handling has two native C++ layers:

| Layer | File | Stage | Role |
| --- | --- | --- | --- |
| `DynamicClearCore` | `src/nav/kernel/include/nav_kernel/dynamic_clear_core.hpp` | traversability producer | filters stale rolling terrain points before publishing `rt/nav/terrain_map` |
| `motion` / `LiveVoxelLayer` | `src/nav/services/endpoint/cpp/motion_layer.*` | nav consumer | keeps explicit unknown/free/occupied/static/cleared voxel evidence plus separate moving-object tracks |
| `LiveObstacleLayer` | `src/nav/services/endpoint/cpp/live_obstacle_layer.*` | compatibility API | preserves existing endpoint calls while delegating to `motion` |

`motion` is the realtime local-planner layer. It raycasts free space from the
current robot pose to current scan endpoints, does not clear cells hidden behind
a nearer endpoint, requires repeated free evidence before demoting static
structure, and preserves current dynamic objects in the planner obstacle
snapshot for collision avoidance. Registered clouds use message-time pose
sampling from `PoseBuffer`; a cloud without a pose inside the configured gap is
rejected and cannot refresh the navigation input gate. Odom, TF, or cloud stamps
more than 50 ms in the future are also rejected instead of being treated as
fresh. Moving-object tracks expire after their TTL even when no newer scan
arrives.
`LiveObstacleLayer` is no longer a separate algorithm; it is the old name kept
for native endpoint compatibility.

## `lingtu-nav-dds` Internal Nodes

`lingtu-nav-dds.service` runs one binary:

```text
/opt/lingtu/current/build/nav_endpoint/lingtu_nav_native_endpoint
```

That binary contains the native planning and command pipeline:

| Internal node | Process | Responsibility | Inputs | Outputs |
| --- | --- | --- | --- | --- |
| Command receiver | `lingtu_nav_native_endpoint` | Validate typed goal/cancel/teleop requests and emit business ACK | `/nav/command/request` | internal command + `/nav/command/ack` |
| OctoPlanner3D global planner | `lingtu_nav_native_endpoint` | Plan a 3D saved-map route | current `map` pose, goal, `octomap.ot` | internal global path |
| Global path publisher | `lingtu_nav_native_endpoint` | Publish accepted global path | internal global path | `/nav/global_path` |
| NavLoop target selector | `lingtu_nav_native_endpoint` | Pick the next lookahead target from global path | current pose, global path | internal target waypoint |
| LocalPlannerCore | `lingtu_nav_native_endpoint` | Generate near-field local path | target, current pose, obstacle cloud, `/nav/traversability` | internal local path |
| Local path publisher | `lingtu_nav_native_endpoint` | Publish the local path | internal local path | `/nav/local_path` |
| PathFollowerCore | `lingtu_nav_native_endpoint` | Convert local path to velocity | local path, follower state | internal `cmd_vel` |
| Waypoint publisher | `lingtu_nav_native_endpoint` | Publish current target waypoint | internal target waypoint | `/nav/way_point` |
| CmdVel publisher | `lingtu_nav_native_endpoint` | Publish speed command when enabled | internal `cmd_vel`, `LINGTU_NAV_PUBLISH_CMD_VEL=1` | `/nav/cmd_vel` |

Runtime order inside `lingtu_nav_native_endpoint`:

```text
/nav/command/request (kind=goal)
  -> validate request_id and control mode
  -> OctoPlanner3D with octomap.ot
  -> nav.setGlobalPath(...)
  -> /nav/global_path
  -> NavLoop::tick(...)
  -> LocalPlannerCore::plan(...)
  -> PathFollowerCore::computeControl(...)
  -> NavLoopOutput {local_path, waypoint, cmd_vel}
  -> DDS /nav/command/ack + /nav/local_path + /nav/way_point + /nav/cmd_vel
```

There is no DDS hop between `LocalPlannerCore` and `PathFollowerCore`. They run
sequentially in the same `NavLoop::tick()` call. DDS publishes the accepted
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
- `/ws/teleop` requires `deadman: true` on every motion sample and gives each
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
  `tracking`, `nav`, `inspection`, and `tare_explore`. Non-zero teleop first
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

`NavLoop::tick(...)` only runs when both are true:

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
| subscribe | `/nav/cmd_vel` | `lingtu.dds.TwistStamped` | clamp and integrate body-frame `vx,vy,wz` |
| publish | `/slam/odometry` | `lingtu.dds.Odometry` | simulated `odom -> body` pose |
| publish | `/tf` | `lingtu.dds.TFMessage` | identity `map -> odom` |
| file | status JSON | `lingtu.motion_mock.status.v1` | pose, last command, counters |

The expected smoke topology is:

```text
domain 77:
  lingtu_motion_mock_dds
    -> /slam/odometry + /tf
  lingtu_nav_native_endpoint
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
| `map -> odom` | SLAM/localizer | `lingtu-nav-dds`, Gateway, evidence gates | converting odometry pose into map frame for saved-map planning |
| `odom -> body` | SLAM/localizer | local planning, path following, Gateway | current robot pose and yaw |
| `body -> lidar_link` | calibration/static TF from SLAM runtime | cloud normalization and evidence gates | aligning LiDAR points to body/map |
| `body -> camera_link` | calibration/static TF | perception/inspection | camera geometry |

Topic frame expectations:

| Topic | Expected frame |
| --- | --- |
| `/lidar/raw_frame` | `lidar_link` |
| `/imu/raw` | `lidar_link` |
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
lingtu-livox-dds.service active
lingtu-slam-dds.service active
lingtu-traversability-dds.service active
lingtu-nav-dds.service active
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
Because has_path=false, NavLoop::tick(...) is not effectively running.
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
`esdf_field`, `slope_grid`, and `control_hint`.

## OctoPlanner3D Map Inputs

OctoPlanner3D is the product global planner. It does not consume the live local
planner terrain stream. It consumes a saved map artifact bundle.

Required for a valid OctoPlanner3D plan:

| Input | Required | Source | Purpose |
| --- | --- | --- | --- |
| `octomap.ot`, `octomap.bt`, or `.octomap` | yes | built from saved `map.pcd` | 3D occupancy tree used by OctoPlanner3D |
| `metadata.json` | yes | MapPipelineCore / SaveMapEngine | records source, frame, and artifact provenance inside the verified version |
| `map.pcd` | yes for artifact gate | SLAM map save | source map and same-source hash anchor |
| metadata `frame_id` | yes | metadata | must match expected saved-map/planning frame, normally `map` |
| metadata `artifacts.map_pcd.sha256` | yes | metadata | source hash for same-source validation |
| metadata `artifacts.map_pcd.point_count` | yes | metadata | must be positive |
| metadata `artifacts.octomap.uri` | yes | metadata | points to the OctoMap file |
| metadata `artifacts.octomap.sha256` | yes | metadata | verifies the OctoMap file |
| metadata `artifacts.octomap.source_map_sha256` | yes | metadata | must match `map_pcd.sha256` |
| `occupancy.npz` | optional | MapService | loaded when present as static 2D occupancy/preblocked context |

The runtime request sent to `octoplanner3d_headless` is JSON:

```json
{
  "planner": "octoplanner3d",
  "protocol_version": 1,
  "map_path": "/path/to/octomap.ot",
  "map_source": {
    "kind": "octomap_file",
    "path": "/path/to/octomap.ot",
    "format": "ot",
    "frame": "map"
  },
  "start": [0.0, 0.0, 0.0],
  "goal": [2.0, 1.0, 0.0],
  "options": {
    "planner_family": "octoplanner3d_constrained_global_planner",
    "search_algorithm": "octomap_3d_astar",
    "constraint_model": "quadruped_bounding_cylinder_ground_support",
    "robot_radius": 0.25,
    "max_iterations": 800000,
    "snap_search_radius_cells": 12,
    "require_ground_support": true,
    "floor_change_penalty": 4.0,
    "max_step_height": 0.45,
    "same_floor_preference": true,
    "obstacle_clearance_radius_cells": 4
  }
}
```

The result is JSON:

```json
{
  "planner": "octoplanner3d",
  "protocol_version": 1,
  "ok": true,
  "path": [[0.0, 0.0, 0.0], [1.0, 0.5, 0.0], [2.0, 1.0, 0.0]],
  "reached_goal": true,
  "diagnostics": {}
}
```

`Navigation` converts the returned `path` into runtime `Path` and publishes it
as `global_path`.

## Local Planner Inputs

Local planning uses live runtime data. It does not require the saved map package
to run, although it follows the `global_path` created from the saved map.

| LocalPlanner port | Type | Required for normal navigation | Producer | Notes |
| --- | --- | --- | --- | --- |
| `odometry` | `Odometry` | yes | SLAM/localizer or driver fallback | robot pose and yaw |
| `waypoint` | `PoseStamped` | yes | Navigation | current target selected from global path |
| `global_path` | `Path` | yes | Navigation | corridor reference |
| `terrain_map` | `PointCloud2` | yes for obstacle-aware planning | Terrain | near-field terrain cloud |
| `terrain_map_ext` | `PointCloud2` | diagnostics only | Terrain | removed-candidate overlay; never a planner obstacle input |
| `traversability` | `dict` | yes for risk scoring | Terrain | native backend uses traversability grid |
| `clear_path` | `bool` | optional | Navigation | reset/clear local path |
| `map_odom_tf` | `dict` | required when map/odom differ | SLAM/localizer | map-to-odom transform |
| `map_frame_jump_event` | `dict` | optional | SLAM/localizer | clears stale path state after relocalization jumps |
| `boundary` | `PointCloud2` | optional | no default producer | reserved overlay |
| `added_obstacles` | `PointCloud2` | optional | no default producer | manual/dynamic obstacle overlay |
| `check_obstacle` | `bool` | optional | no default producer | toggles overlay checking |
| `esdf` | `dict` | wired but reserved | TraversabilityCost | stored, not yet primary native scoring input |

LocalPlanner outputs:

| Output | Type | Consumer |
| --- | --- | --- |
| `local_path` | `Path` | PathFollower, SafetyRing, Gateway/nav out |
| `control_hint` | `dict` | PathFollower |
| `alive` | `bool` | health/diagnostics |

## Path Follower and Command Outputs

| Module | Input | Type | Output | Type |
| --- | --- | --- | --- | --- |
| PathFollower | `odometry`, `local_path`, `control_hint`, `map_odom_tf`, `map_frame_jump_event` | `Odometry`, `Path`, `dict` | `cmd_vel` | `Twist` |
| CmdVelMux | `teleop_cmd_vel`, `visual_servo_cmd_vel`, `recovery_cmd_vel`, `path_follower_cmd_vel` | `Twist` | `driver_cmd_vel` | `Twist` |
| SafetyRing | `odometry`, `path`, `cmd_vel`, `mission_status`, `localization_status` | mixed | `stop_cmd`, `safety_state`, `execution_eval` | `int`, structured status |

CmdVelMux priority:

| Source | Priority |
| --- | ---: |
| teleop | 100 |
| visual servo | 80 |
| recovery | 60 |
| path follower | 40 |

Final command path:

```text
PathFollower.cmd_vel
  -> CmdVelMux.path_follower_cmd_vel
  -> CmdVelMux.driver_cmd_vel
  -> local simulation/development driver
  -> brainstem Walk(Vector3)
```

The physical `thunder_field` profile does not use this Python command chain;
the C++ command chain is:

```text
lingtu-nav-dds PathFollower
  -> native command arbiter and safety
  -> rt/nav/cmd_vel
  -> lingtu-driver
  -> Brainstem Walk(Vector3)
```

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
9. `lingtu-driver` receives `/nav/cmd_vel` and Brainstem accepts `Walk(Vector3)`.
