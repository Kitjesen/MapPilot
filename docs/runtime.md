# Runtime

**Status:** Current Product runtime and field ownership contract
**Audience:** Product, Host, native endpoint, Gateway, and deployment maintainers
**Runs on:** Primarily `env=real`; lifecycle semantics also apply to `env=sim`

This page defines the current Product runtime and physical-field ownership.
Product declarations in `config/system/products/*.yaml` and mappings in
`config/system/envs/*.yaml` remain authoritative.

## Products

| Product | Runtime intent |
| --- | --- |
| `teleop` | Operator control through native limits and final output gates |
| `teleop_avoid` | Mapping SLAM, live 3D collision data, and native local avoidance |
| `map` | Build a live map and transactionally save persistent artifacts |
| `explore` | Live mapping without a map, or saved-map localization with a map |
| `nav` | Saved-map localization, global/local planning, tracking, and control |
| `tracking` | Host publishes bounded map-frame goals; native navigation owns motion |
| `inspection` | Execute a persisted, task-addressed multi-point route |

All current field Products use cold restart. `explore --map MAP` is a
saved-map variant of the same Product, not a separate Product.

Product activation does not itself begin exploration motion. The explicit
task-start surface owns that action.

The first product delivery path is:

```text
map Product -> durable SaveMap -> stop/restart -> nav Product with that map
  -> localization ready -> goal -> autonomous motion -> terminal result/stop
```

Each arrow has its own evidence. Save admission is not durable completion,
process liveness is not localization readiness, and route preview is not motion
permission. The same flow applies to every supported RobotConfig; each robot
still needs its own calibration and field evidence.

## Lifecycle

```text
ProductControl request (Product + RobotConfig + env=real)
  -> internal resolved execution snapshot
  -> internal SystemdRunner
  -> declared lt-*.service processes
  -> readiness
  -> commit product_session_id
```

`ProductControl` is the public lifecycle entry. The resolved execution
snapshot is currently serialized as a `RunPlan` for restart, readiness, and
rollback; that file and its path are implementation details. Operators,
Gateway callers, and public SDK users select a Product through ProductControl
and never construct or pass a RunPlan.

The implementation path is:

- `src/lingtu/control.py`: public lifecycle.
- `src/lingtu/real/switch.py`: field transaction and rollback.
- `src/lingtu/real/systemd.py`: declared process execution and readiness.
- `src/lingtu/run_plan.py`: internal immutable execution snapshot.
- `config/system/envs/real.yaml`: real implementation mapping.

Direct `systemctl` is a service-level diagnostic, not a second Product startup
API. ProductControl stops current motion, stages map/session state, applies the
internal execution snapshot, checks readiness, and only then commits the
current run.

## Field owners

| Owner | Canonical source | Owns |
| --- | --- | --- |
| LiDAR/IMU | `src/drivers/real/lidar/sdk2_stream/` | MID-360 ingestion and typed DDS publication |
| SLAM | `src/localization/slam/cpp/` | Mapping/localization, status, snapshot, and relocalization |
| Maps | `src/maps/cpp/mapd/` | Live layers, scene, SaveMap, artifacts, and saved-map lifecycle |
| Traversability | `src/nav/cpp/endpoint/` | Unique `/nav/traversability` control-risk writer |
| Navigation | `src/nav/cpp/endpoint/` and `src/nav/cpp/navigation/` | Goal lifecycle, planning, tracking, and final command arbitration |
| Driver | `src/drivers/real/motion/` | Unique robot hardware writer |
| Host | `src/runtime/` and `src/gateway/` | Gateway, Agent, MCP, semantic behavior, and low-rate adapters |

Deployed roles use `lt-lidar`, `lt-slam`, `lt-maps`, `lt-terrain`, `lt-nav`,
`lt-driver`, `lt-camera`, `lt-explore`, and `lt-host` units when declared by
the selected Product and environment.

## Native dataflow

```text
MID-360 / IMU
  -> native sensor process
  -> slamd
  -> odometry + registered cloud + MapObservation
  -> mapd (standalone traversability only when declared by the Product)
  -> navd
  -> rt/nav/cmd_vel
  -> lingtu-driver
  -> selected RobotConfig adapter
```

`navd` is the field navigation state authority and final logical command
writer. Standalone traversability is the unique control-risk grid writer for
Products that declare it. The default SCAN `nav` and `teleop_avoid` Products
consume Mapd's live 3D collision volume and do not require `/nav/traversability`.
Visualization layers do not authorize motion.

`lingtu-driver` is the unique hardware writer. Field Products have no Python
algorithm fallback. Gateway translates requests and projects typed state; it
does not infer a second navigation or map success state.

The real execution snapshot selects `cpp_slam_status`,
`command_output_mode=endpoint_only`, and
`hardware_control_boundary=driver`. `lt-host.service` keeps
`LINGTU_ENABLE_ROBOT_DRIVER=0` so the Host cannot open a second hardware writer.

## Command modes

### Teleop

```text
operator lease + deadman + body-frame samples
  -> navd final gates -> rt/nav/cmd_vel -> driver
```

An acknowledgement proves command admission only. Native endpoint state,
driver acknowledgement, and actuator evidence are separate claims.

### Teleop with avoidance

```text
operator sample
  + SLAM odometry and cloud
  + Mapd 3D collision volume
  -> LocalPlanner -> PathFollower -> final gates -> cmd_vel
```

`teleop_avoid` has `requires_map: false`. It does not receive localization or
planner saved-map arguments. If no valid local path exists, output must be
zero; blind direct motion is not a fallback.

### Autonomous Products

```text
typed goal
  -> goal lifecycle
  -> exact active MapIdentity
  -> global planner -> LocalPlanner -> PathFollower -> final gates -> cmd_vel
```

`NavigationCommandAck`, `NavigationGoalStatus`, and `NavigationState` carry
different semantics. A terminal task result is correlated by `request_id`; it
must not be inferred from a current-state snapshot.

The live UI displays the lifecycle in fresh native `NavigationState` rather
than reconstructing it from command receipts or goal events. Durable task
completion still comes from the matching execution event. Inspection waypoints
reuse the route's task and command IDs; their planning epochs stay internal.
Leg results advance the native Inspection Executor without publishing a second
single-goal lifecycle. Inspection events update the route's live state. Runtime
input holds affect motion permission, not the task's committed pause state.

Waypoint completion is taken from the committed native goal result after stop
confirmation, including replay of a pending terminal. The route does not run
another planning timer or straight-line distance watchdog: the planner owns
search termination, and the navigation executor owns motion progress and local
recovery. Inspection owns settling, point actions, and the route failure policy.

OctoPlanner owns the saved-map route and terrain constraints. LocalPlanner owns the
heading-dependent body envelope, live obstacle avoidance, and local trajectory.
Product owns arrival tolerances; RobotConfig owns the physical body model.
Executor supplies one identified map-frame reference, while LocalPlanner selects the
local target. Preview proves only that a global route was found. Motion waits
for fresh localization and collision input, authority, map identity, driver
readiness, and final native checks. Gateway projects these reasons without
reimplementing them.

### Route, trajectory and motion conditions

| Stage | Required evidence | Handling a failure |
| --- | --- | --- |
| Global preview/search | Current map pose, matching map, valid target, supported connected 3D route | Restore localization/map binding, correct the target, or inspect map evidence |
| Goal admission | Active session, exact identity, control ownership, no E-stop/takeover hold | Restore the reported session, authority, or map condition |
| Local trajectory | Measured state, reference, fresh collision data, feasible collision-free trajectory | Restore inputs or allow local recovery before requesting a replacement route |
| Nonzero command | Active route, recovered InputGate, authority, executable result, final safety and driver gates | Remain stopped while any condition fails |

Recovery is local wait/retry, checked local recovery, then a replacement route
when local recovery is exhausted. An accepted goal has no lifetime replan-count
limit. An ordinary replacement search with no path retains the destination and
task/request identity, keeps output zero, and retries after the existing 0.5 s
backoff. Only one search may be in flight. Success returns to local tracking;
another obstruction can start this cycle again. Stop confirmation that has not
arrived yet keeps the task waiting rather than consuming a retry.

The UI shows this ordinary no-path wait as `replan_waiting` (waiting for a route,
continue automatically), not `FAILED` or `PAUSED`. Deliberate operator takeover
pauses and retains the ordinary goal, including first planning or retry waiting.
Release keeps motion held; an explicit TaskResume continues the same task from
the current measured pose. A source Claim alone does not pause or cancel it.
Explicit cancel, stop, identity changes, planner exceptions, and command-delivery
faults keep their own interruption/terminal handling. Initial goal admission and preview
failures retain their existing behavior. Measured obstacle evidence stays in
local planning/recovery and does not rewrite the saved map. Detailed native
behavior and tuning belong in the
[navigation implementation guide](../src/nav/cpp/README.md).

### Go2 navigation parameter reference

Go2 is one supported RobotConfig, not the product boundary. Its physical values
live in [`config/robots/unitree/go2/robot.yaml`](../config/robots/unitree/go2/robot.yaml);
Product navigation values live in
[`config/system/products/nav.yaml`](../config/system/products/nav.yaml).
Inspect ProductControl's resolved runtime status and the native status for the
deployed robot instead of copying parameter values from this page. The backing
execution snapshot is diagnostic detail, not an operator input.

## Maps and localization

Canonical saved maps live at `<map-root>/<map-id>/`. Map identity has two
fields: `map_id` and a positive numeric `content_epoch`. DDS carries
`map_content_epoch`.

`map:v123` and `map:e123` are invalid encoded map IDs. There is no
`.versions/`, `current_version.txt`, or version-directory rollback contract.

ProductControl is the only public map activation owner. SLAM, mapd, and the
planner in one Product run must bind the same MapIdentity.

Delete, rename, retire, source replacement, artifact rebuild, and voxel edits
reject changes to the active map with `active_map_conflict`. SaveMap does not
activate maps and cannot replace the active map under the same name. Save to a
new map ID, or switch maps through ProductControl before modifying the old one.

| Artifact | Role |
| --- | --- |
| `map.pcd`, `metadata.json` | Required canonical source and metadata |
| `octomap.ot` | Current native navigation artifact |
| `poses.txt`, `scan_origin.txt`, `patches/` | Required for a saved-ray navigation artifact; optional only for preview/diagnostic maps |
| `occupancy.npz` | Default save output; consumed by saved-map exploration |
| `esdf.npz`, `traversability.npz` | Explicit offline build outputs; not required by default save or 3D navigation |
| `semantic_map.bin` | Optional semantic product; does not alone make a map activation-ready |

Semantic artifacts are bound to the same `map_id` and `content_epoch` as the
geometric package. They can answer object/place queries, but they do not alter
the native collision, support, traversability, global-route, or motion
contracts. See [Persistent semantic mapping](research/semantic-map.md)
for the proposed `object_memory` structure and the current implementation
gaps.

SaveMap follows one transaction:

```text
mapd save_map
  -> typed SLAM snapshot request/ack
  -> SaveMapEngine
  -> optional native PGO
  -> validate staged artifacts
  -> transactional canonical replacement
```

Realtime DDS observations support live layers and visualization. They are not
the sole persistent source because the transport may intentionally drop older
samples.

Saved-map navigation requires a valid `map -> odom` relationship and fresh
odometry within the active map frame. Process liveness or `TRACKING` alone does
not establish navigation readiness.

## Topics, transport, and frames

Field processes use typed CycloneDDS. Python canonical topic names come from
`message.topics.TOPICS`; native schemas and topic bindings come from
`src/message/idl/` and `src/message/topics/`. Products reference this catalogue;
they do not redefine message types. Frame tools live in `runtime.tf.frames`.

The logical `/nav/cmd_vel` stream uses DDS name `rt/nav/cmd_vel`. Saved maps and
global paths use `map`. Local sensor and control payloads use the declared
`body` or `odom` frame.

ROS TF and ROS topic aliases are explicit compatibility surfaces. They cannot
become the normal Product data plane.

`/nav/state` publishes on the first control tick that observes a changed payload,
including progress, authority, hold reason, and failure code. Unchanged state
is refreshed every 0.2 seconds. Failed writes retry on the next tick; successful
writes use a fresh timestamp and increasing sequence. This limits unchanged
state traffic, not the control loop or motion-command rate. The browser uses
the shared state-freshness rule for keyboard control and requires a new key
press after control state becomes unknown or stale.

The preview request includes `acceptance_radius_m` in DDS `PlanRequest`. Its
native client export is `lingtu_nav_client_preview_plan_v2` with client ABI 11.
Rebuild and deploy generated DDS types, native endpoints/client, and Python
bindings together; an old native client is not compatible with this binding.

## Safety and readiness

Native navigation owns final motion arbitration. Stale input, authority loss,
cancel, stop, cleanup, and failed readiness must produce zero output.

Readiness requires the declared processes, fresh required topics, unique
writers, correct map identity, acceptable navigation state, and driver
acknowledgement where applicable. A running unit is not permission to move.

Use [Testing](./testing.md) for no-motion and supervised-motion evidence rules.

## Stable commands

Read status:

```bash
python -m lingtu.control status \
  --robot unitree/go2 --env real --json
```

Start mapping:

```bash
python -m lingtu.control switch map \
  --robot unitree/go2 --env real
```

Start saved-map navigation:

```bash
python -m lingtu.control switch nav \
  --robot unitree/go2 --env real --map building_a
```

Stop the Product:

```bash
python -m lingtu.control stop \
  --robot unitree/go2 --env real
```

The robot-side thin adapter is equivalent:

```bash
bash scripts/lingtu --robot unitree/go2 --env real status --json
```

Deployment, release, and diagnosis are covered in
[Operations](./operations.md).
