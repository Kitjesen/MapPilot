# Runtime

**Status:** Current Product runtime and field ownership contract
**Audience:** Product, Host, native endpoint, Gateway, and deployment maintainers
**Runs on:** Primarily `env=real`; lifecycle semantics also apply to `env=sim`

This page defines the current Product runtime and physical-field ownership.
Product declarations in `config/runtime_graph/products/*.yaml` and mappings in
`config/runtime_graph/envs/*.yaml` remain authoritative.

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

## Lifecycle

```text
Product + env=real
  -> one RunPlan
  -> ProductControl
  -> internal SystemdRunner
  -> declared lt-*.service processes
  -> readiness
  -> commit product_session_id
```

The implementation path is:

- `src/lingtu/control.py`: public lifecycle.
- `src/lingtu/real/switch.py`: field transaction and rollback.
- `src/lingtu/real/systemd.py`: declared process execution and readiness.
- `src/lingtu/run_plan.py`: immutable resolved artifact.
- `config/runtime_graph/envs/real.yaml`: real implementation mapping.

Direct `systemctl` is a service-level diagnostic, not a second Product startup
API. ProductControl stops current motion, stages map/session state, applies the
RunPlan, checks readiness, and only then commits the current run.

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
`lt-driver`, `lt-camera`, `lt-explore`, and `lt-host` units when declared by the
RunPlan.

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

The real RunPlan selects `cpp_slam_status`,
`command_output_mode=endpoint_only`, and
`hardware_control_boundary=driver`. `lt-host.service` keeps
`LINGTU_ENABLE_ROBOT_DRIVER=0` so the Host cannot open a second hardware writer.

## Command modes

### Teleop

```text
operator lease + deadman + body-frame samples
  -> navd final gates
  -> rt/nav/cmd_vel
  -> driver
```

An acknowledgement proves command admission only. Native endpoint state,
driver acknowledgement, and actuator evidence are separate claims.

### Teleop with avoidance

```text
operator sample
  + SLAM odometry and cloud
  + Mapd 3D collision volume
  -> LocalPlanner
  -> PathFollower
  -> final gates
  -> cmd_vel
```

`teleop_avoid` has `requires_map: false`. It does not receive localization or
planner saved-map arguments. If no valid local path exists, output must be
zero; blind direct motion is not a fallback.

### Autonomous Products

```text
typed goal
  -> goal lifecycle
  -> exact active MapIdentity
  -> global planner
  -> LocalPlanner
  -> PathFollower
  -> final gates
  -> cmd_vel
```

`NavigationCommandAck`, `NavigationGoalStatus`, and `NavigationState` carry
different semantics. A terminal task result is correlated by `request_id`; it
must not be inferred from a current-state snapshot.

OctoPlanner owns the saved-map route and terrain constraints. SCAN owns the
heading-dependent body envelope, live obstacle avoidance, and local trajectory.
The global route radius is independent of SCAN's two-cylinder dimensions.
The goal adapter preserves the global planner's returned points, including a
single-point arrival; it does not append the requested goal to make a longer
path. SCAN-specific reference-spacing adjustments belong inside the SCAN
adapter, not in Executor's shared route construction.

Product owns task arrival policy through `native_nav.goal_reached_m` and
`native_nav.goal_height_tolerance_m`. Switching between SCAN and CMU does not
select a different arrival policy. The former
`scan_planner.route_z_tolerance_m` parameter and its
`LINGTU_NAV_SCAN_ROUTE_Z_TOLERANCE_M` binding are removed; regenerate RunPlan
with the task-owned field. The `nav` Product explicitly retains 0.35 m for Z.

Endpoint snapping follows the upstream OctoPlanner policy: choose a nearby
traversable start and search from that candidate. There is no separate rejection
of the measured start or mandatory straight-line connection from that start to
the snapped candidate. The former `start_connection_blocked` gate is removed.
The searched route retains its 3D support, unknown-space, clearance, step and
slope constraints. These constraints cover the searched route, not the omitted
measured-start-to-candidate connection.

Executor anchors the local reference to the measured body pose. SCAN checks the
resulting trajectory against live 3D inflated occupancy and dynamic predictions.
It does not certify ground support or negative obstacles on that connection;
snapping must not be described as physical relocation, nor preview success as
complete terrain certification from the measured pose.

In the production map-frame SCAN path, Executor constructs the complete
reference once and reuses it until route replacement, suspension, or a frame
epoch change. SCAN selects the local target; Executor does not rebuild a CMU
corridor segment every control tick. Local target diagnostics use SCAN's
generated target, falling back to the global endpoint until a new target is
available. For SCAN, `target_index` identifies that global endpoint rather than
local progress. CMU and odom-frame reference construction remain unchanged.

SCAN receives fresh dynamic predictions directly. Executor does not add a
second prediction-based wait/resume timer before local planning; it retains
route progress, recovery, tracking, and final motion gates. A prediction that
blocks the current global reference may still permit a local detour.

Same-floor preference is a soft global search cost. A supported route is not
discarded solely because its total height excursion exceeds a fixed threshold;
per-edge terrain and collision constraints still apply.

Preview carries the same requested acceptance radius as goal submission and
uses the smaller of that radius and the configured terminal XY tolerance.
Preview success describes a global route, not permission to move or a promise
that the live local trajectory is executable.

Goal admission does not require SCAN's local collision sample to be ready.
The known `local_collision_missing`, `local_collision_future`,
`local_collision_stale`, and `local_collision_incomplete` holds allow an initial
global route request while motion remains held. Endpoint/Product readiness
still reports the hold, and native autonomy waits for InputGate recovery before
running the local planner. Missing endpoint status, unhealthy localization,
driver authority, E-stop and map identity failures retain their own checks.

### Route, trajectory and motion conditions

| Stage | Required evidence | Handling a failure |
| --- | --- | --- |
| Global preview | Valid map-frame target and tolerance, current map pose, configured planner map, available planner | Restore localization/map binding, correct the target, or wait for the current search |
| Global route search | Traversable snapped endpoints and a connected 3D route satisfying support, unknown-space, clearance, step and slope rules | Inspect the failing map cells and input evidence; repair/rebuild a candidate map when the geometry is wrong |
| Goal admission | Active navigation session, current endpoint status, valid task/request identity, control ownership, no E-stop/takeover hold, matching active map | Restore the identified session/authority/map condition; local collision holds alone do not prevent initial planning |
| SCAN trajectory | Measured pose/velocity, valid reference, fresh complete local collision data, successful trajectory optimization and static/dynamic collision checks | Restore mapd input or let SCAN replan around observed obstacles; repeated spatial blockage can request a global replan |
| Trajectory feasibility | Finite spline samples within configured speed/acceleration bounds | Inspect the reported quantity/time and reference continuity; change physical limits only with measured robot evidence |
| Nonzero command | Active route, matching current map, recovered InputGate, motion authority, executable local result, follower/final safety checks, enabled command publication | Remain stopped while the failed condition is present; route-preview success does not bypass it |

For Go2, input ages are bounded at 0.25 s for odometry/TF, 0.35 s for the
required cloud, 0.50 s for local collision/localization health, and 0.35 s for
driver control. Future-stamp tolerance is 0.05 s and recovery requires three
valid frames. These are execution-input bounds, not additional terrain tests.

SCAN's reference must retain at least two finite points after the upstream
0.50 m waypoint-spacing filter. Its local target is selected along the
reference within the 3.5 m horizon, outside inflated occupancy and normally at
least 0.20 m ahead; the final goal is exempt from that distance condition.
Blocked targets trigger an along-reference search before rejection. The
rebound seed requires at least seven samples/control points. Optimization must
succeed, the complete spline must pass swept 3D collision validation (also
after refinement), and sampled velocity/acceleration must pass their configured
acceptance bounds. A new reference is received on one FSM tick and generated
on a later tick; asynchronous `Pending` is not a failed route.

The follower's 0.80 rad heading freeze and final live-grid braking sweep act on
execution after a spline exists. Goal-height tolerance likewise governs
arrival, not whether SCAN may generate a trajectory.

Persistent obstacle feedback follows the measured obstacle snapshot generation,
not a traversability-grid generation. A viable local path suppresses this
feedback. Otherwise, repeated observed blockage can request a bounded global
replan with temporary measured obstacle voxels. Their Z extents remain one
observation voxel, with different heights at the same XY retained separately;
they are neither robot-inflated nor extended into vertical columns.

A local planner can report typed collision evidence outside the global route
corridor. SCAN extracts nearby measured occupied cells from the same collision
snapshot that rejected its trajectory. Coordination consumes that evidence
without interpreting SCAN's private attempt diagnostics or inflated-only cells.
The failed body sample is not itself obstacle geometry.
Persistence counts new sensor observation sequences, not collision-map version
changes caused by decay. New observations still count when geometry is unchanged.
Input or acceleration failures without spatial collision evidence do not create
blocked regions. This feedback does not rewrite the saved OctoMap or bypass the
existing stop-before-replan transaction.

### Go2 navigation parameter reference

Resolving `compile_run_plan("nav", "real", robot="unitree/go2")` without
session overrides gives the following values. They describe repository
configuration, not the currently installed field release; the active RunPlan
and native status are the field source of truth.

| Parameter | Resolved value | Meaning |
| --- | --- | --- |
| Global route radius | 0.155 m | Half the Go2 width; independent of SCAN's body model |
| Start/goal snap search | 24 cells per axis | At 5 cm resolution, up to 1.2 m per axis, not a 1.2 m Euclidean sphere |
| Ground support | Required; strict direct support off; XY neighborhood 1 cell | Applied to searched global-route cells |
| Body-to-support height | 0.35 m, tolerance 0.05 m | Go2 calibration |
| Global maximum step / slope | 0.45 m / 0.57735 (30 degrees) | Geometric search bounds, not verified stair-climbing capability |
| Terminal tolerance | 0.15 m in 3D, XY and Z | Preview also honors the requested acceptance radius |
| SCAN body cylinders | Radius 0.25 m, offsets +/-0.19 m | Heading-dependent 0.50 m wide, 0.88 m long model |
| Body vertical clearance | 0.10 m below and above | Applied when inflating the 3D collision volume |
| Local collision maximum age | 0.50 s | Freshness condition for local planning and collision queries |
| SCAN planning horizon | 3.5 m | Local target selection along the reference |
| SCAN control-point spacing | 0.20 m | B-spline seed spacing |
| SCAN replan / no-replan distance | 1.00 m / 0.10 m | FSM distance thresholds, not a fixed trajectory-update frequency |
| Nominal speed / acceleration | 0.75 m/s / 0.50 m/s² | Product values; a goal's speed cap can reduce execution speed |
| Sampled velocity allowance | 1.00 m/s | Spline acceptance allowance; follower commands remain capped at the configured/session speed |
| Sampled acceleration allowance | 1.20 m/s² | Added to the nominal acceleration for the trajectory validation threshold (1.70 m/s²) |
| Goal height tolerance | 0.35 m | Product task-arrival Z value, independent of the planar acceptance radius; not a SCAN setting |
| Native tick rate | 100 Hz | FSM/controller cadence, not 100 complete optimizations per second |

## Maps and localization

Canonical saved maps live at `<map-root>/<map-id>/`. Map identity has two
fields: `map_id` and a positive numeric `content_epoch`. DDS carries
`map_content_epoch`.

`map:v123` and `map:e123` are invalid encoded map IDs. There is no
`.versions/`, `current_version.txt`, or version-directory rollback contract.

ProductControl is the only public map activation owner. SLAM, mapd, and the
planner in one RunPlan must bind the same MapIdentity.

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
