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
| `teleop_avoid` | Mapping SLAM, live traversability, and native local avoidance |
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
  -> mapd + standalone traversability
  -> navd
  -> rt/nav/cmd_vel
  -> lingtu-driver
  -> selected RobotConfig adapter
```

`navd` is the field navigation state authority and final logical command
writer. Standalone traversability is the unique control-risk grid writer.
`mapd` scene and visualization layers cannot replace that control input.

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
  + traversability
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

## Maps and localization

Canonical saved maps live at `<map-root>/<map-id>/`. Map identity has two
fields: `map_id` and a positive numeric `content_epoch`. DDS carries
`map_content_epoch`.

`map:v123` and `map:e123` are invalid encoded map IDs. There is no
`.versions/`, `current_version.txt`, or version-directory rollback contract.

ProductControl is the only public map activation owner. SLAM, mapd, and the
planner in one RunPlan must bind the same MapIdentity.

| Artifact | Role |
| --- | --- |
| `map.pcd`, `metadata.json` | Required canonical source and metadata |
| `octomap.ot` | Current native navigation artifact |
| `poses.txt`, `patches/` | Optional SLAM/optimization inputs |
| `occupancy.npz`, `esdf.npz`, `traversability.npz` | Optional derived products |
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
