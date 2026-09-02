# LingTu Field Products

Status: current field Product guide
Audience: Product, Host, native endpoint, Gateway, and deployment maintainers
Replaced by: not replaced

This document defines runtime ownership for top-level field Products. The
machine-readable source of truth is `config/runtime_graph/products/*.yaml`.
Documentation must not invent a second process list, topic list, or fallback
policy.

## Two Runtime Scopes

| Scope | Owner | Contains | Does not own |
| --- | --- | --- | --- |
| Product | Assembly output applied by ProductControl | Native process roles, one Host process, topics, capabilities, switch policy | Domain algorithms or Module wires |
| Host | Blueprint | Gateway, Agent, MCP, semantic logic, low-rate adapters, selected development Modules | systemd, native endpoints, field hot paths |

`Module` and `Blueprint` remain valid inside the Host. They are not the field
process orchestrator. Field Products use native typed DDS between processes;
algorithms inside one C++ endpoint call each other directly.
Every field Product that declares `host` follows this same Host scope; native
`mapd`, `navd`, or `slamd` ownership does not create a Blueprint bypass.

## Field Ownership

```text
LiDAR / IMU
  -> native sensor process
  -> slamd
  -> odometry + registered cloud + MapObservation
  -> mapd + standalone traversability
  -> navd
  -> /nav/cmd_vel
  -> driver
  -> Brainstem

HostBus <-> Gateway / Agent / MCP
```

The following ownership rules are mandatory:

- `navd` is the only field navigation state authority and final
  `/nav/cmd_vel` writer.
- standalone native traversability is the only `/nav/traversability` writer.
  A mapd visualization or artifact layer must never replace that control input.
- `mapd` owns realtime `MapObservation` ingestion, live map layers, bounded
  accumulation, `/maps/scene`, persistent map records, SaveMap, and artifacts.
- ProductControl is the only public saved-map activation owner.
- the driver is the only Brainstem hardware writer.
- HostBus projects typed native state into Host messages. Gateway must not
  infer a second navigation or map state machine.
- a missing or stale required native component blocks readiness; field
  Products have no Python algorithm fallback.

## Product Matrix

| Product | SLAM mode | Saved map | Control owner | Native purpose |
| --- | --- | ---: | --- | --- |
| `teleop` | none | no | operator | Typed operator authority and direct velocity intent through native limits and final output gates. |
| `teleop_avoid` | mapping | no | operator | Live odometry/cloud/traversability feed native local detour planning; no global planner or saved map. |
| `map` | mapping | no | operator | Build a live map and transactionally save persistent map products. |
| `explore` | mapping without a map; localization with a map | optional | exploration endpoint | Use the live route to grow a map, or the map route to cover a validated saved map. |
| `nav` | localization | yes | native navigation | Saved-map global planning, local avoidance, tracking, and final command output. |
| `tracking` | localization | yes | native navigation | Follow a selected RGB-D person; Host publishes bounded map-frame goals and native navigation owns motion. |
| `inspection` | localization | yes | native navigation | Execute a persisted, task-addressed multi-point inspection route. |

`scripts/lingtu switch explore` selects live mapping.
`scripts/lingtu switch explore --map MAP`
selects saved-map localization. These are two cold-restart variants of one
Product, not separate operator modes. Product activation leaves the exploration
task idle; authenticated `POST /api/v1/explore/start` is the explicit
motion-capable step.

Every current field Product uses `switch_policy: cold_restart`. A future hot
switch requires an explicit Product contract and acceptance evidence; sharing
similar processes is not sufficient.

## Command Contracts

### Teleop

```text
operator client
  -> /nav/operator_motion/control  (claim / hold / release)
  -> /nav/operator_motion/sample   (fresh body-frame intent)
  -> navd final gates
  -> /nav/cmd_vel
  -> driver
```

An ACK proves command admission only. Native operator-motion status proves the
logical endpoint state; driver ACK and actuator evidence are separate.

### Teleop With Avoidance

```text
operator sample
  + /slam/odometry
  + /slam/registered_cloud
  + /nav/traversability
  -> native LocalPlanner
  -> native PathFollower
  -> final collision/staleness/authority gates
  -> /nav/cmd_vel
```

`teleop_avoid` runs SLAM in mapping mode and has `requires_map: false`. It must
not receive localization-map arguments, planner-map arguments, or a promoted
temporary `map.pcd`. No local path means zero output, not blind direct motion.

### Autonomous Navigation

```text
typed goal command
  -> navd goal lifecycle
  -> validated active map identity
  -> native global planner
  -> native LocalPlanner
  -> native PathFollower
  -> final gates
  -> /nav/cmd_vel
```

`NavigationCommandAck`, `NavigationGoalStatus`, and `NavigationState` have
different meanings. Clients correlate terminal task results by `request_id`;
they do not infer completion from the current state snapshot.

### Mapping And Save

```text
MapObservation
  -> mapd realtime layers and /maps/scene

SaveMap
  -> mapd save_map
  -> typed SlamMapSnapshotRequest
  -> slamd freezes session/epoch/sequence and returns SlamMapSnapshotAck
  -> mapd submits the snapshot to SaveMapEngine
  -> complete patch bundle automatically enters native PGO
  -> stage and validate source/derived artifacts
  -> transactionally replace the canonical map content
```

Realtime observations are for live layers and visualization. They are not the
sole persistent-map truth because DDS may intentionally drop old frames.

## Host Boundary

The field Host may contain `host.bus`, Gateway, command adapters, and semantic
logic. Navigation planning, command arbitration, path following, final motion
output, and map management belong to native endpoints. Gateway's Python map
code is only a stateless same-host UDS transport to `mapd`; there is no
`maps.service`, Python SaveMap coordinator, or Python map-state owner.

## Development And Simulation

Development and simulation use the same native navigation capability boundary
as the field Product. Python semantic/API helpers may support isolated tooling,
but they do not own map state, map algorithms, motion, or field lifecycle and do
not prove field equivalence.

## Readiness And Validation

A field Product is ready only when its declared process, topic, freshness,
single-writer, and control-owner gates pass. At minimum verify:

- selected Product, Product session ID, RunPlan path, and env, plus each required communication endpoint contract;
- required native binaries and source-closure freshness;
- unique `/nav/cmd_vel` and `/nav/traversability` writers;
- fresh SLAM, mapd scene/state, navigation state, and driver ACK where required;
- map identity for saved-map Products;
- zero output after stop, cancel, stale input, authority loss, or cleanup.

Useful checks:

```bash
uv run --locked python -m pytest tests/runtime/test_runtime_graph_contract.py -q
uv run --locked python tools/validate/validate_architecture_boundaries.py
uv run --locked python tools/validate/validate_topics.py
uv run --locked python -m pytest tests/contracts/test_runtime_architecture_boundaries.py -q
uv run --locked python -m pytest tests/lingtu/assembly/test_compile.py tests/lingtu/test_run_plan.py -q
```

MuJoCo, replay, no-motion target checks, and supervised field motion are
different evidence classes. Record only the claim proved by the selected gate.
