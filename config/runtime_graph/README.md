# LingTu Runtime Graph

Status: current runtime graph contract as of 2026-07-18.

Runtime Graph is the readable contract layer for product and endpoint wiring.
It is not the runtime data plane.

## Runtime Control Model

The control plane has four separate objects. They must not be collapsed into
one class or one configuration file:

| Object | Owns | Must not own |
| --- | --- | --- |
| Product | Required capabilities, topics, and logical process names. | systemd units, executable paths, startup commands. |
| Endpoint | Declares transport/topic availability and which lifecycle owner controls the endpoint. RuntimePlan-managed endpoints also map logical processes to deployment targets. | Product policy or navigation algorithms. |
| RuntimePlan | The resolved, ordered process plan for one Product on one Endpoint. | Business logic or Module wiring. |
| Blueprint | In-process Python Module construction and port wiring. | Native process startup or systemd ownership. |

The machine path is:

```text
products/<product>.yaml + endpoints/<endpoint>.yaml
  -> runtime.graph.build_runtime_plan()
  -> lingtu.py switch-plan
  -> scripts/lingtu executes the ordered plan
  -> systemd or the endpoint-specific process manager
```

Products declare `processes` using stable logical names such as `lidar`,
`slam`, `nav`, and `runtime`. RuntimePlan-managed endpoint files declare
`process_manager` and map those names to deployment `target` values. Topic
names are validation evidence; they are never used to infer which process
should start.

Only endpoints with `process_control: runtime_plan` expose an operator process
plan. Acceptance harnesses such as `mujoco_native_dds` declare
`process_control: acceptance_runner`; their runner owns child-process setup,
teardown, evidence capture, and timeout handling as one test transaction. The
control plane must not fabricate a partial RuntimePlan for such an endpoint.

The field endpoint is fail closed: every selected process target must be
installed, become active within its declared timeout, and pass its role-specific
readiness check. The plan also carries all mode-owned and conflicting targets,
so switching products removes stale process ownership before startup.

## Naming

Names reflect scope instead of repeating implementation details:

```text
logical process id: nav
executable:         navd
systemd unit:       lingtu-nav-dds.service
field endpoint id:  thunder_field
```

The executable is short because its package and directory already establish
the product and domain. The systemd unit remains globally namespaced because
unit names share a host-wide namespace. `dds` is allowed in the deployment
target, but Product and RuntimePlan logic depend only on the logical `nav` id.

Runtime Graph plus `runtime.route_contract` is the migration bridge between
the in-process Blueprint graph and the field DDS boundary:

```text
Blueprint wires define Module ports:
  SlamAdapterModule.odometry -> nav.mission.odometry

RouteContract defines the external transport for the same canonical topic:
  /slam/odometry -> rt/slam/odometry -> lingtu.dds.Odometry
```

Do not move DDS imports into normal Modules. Modules keep typed `In`/`Out`
ports; adapters and endpoint services translate between those ports and DDS.
Python code must get canonical runtime topics from
`runtime.runtime_interface.TOPICS`; see
`docs/architecture/TOPIC_CONTRACT_POLICY.md` for the static guard and allowed
boundary exceptions.

Runtime resolver now exposes the selected contract as `route_contract`.
For the field endpoint this should read:

```text
runtime_contract=thunder_field
endpoint_contract=thunder_field_dds_v1
route_contract=robot
```

`route_contract` is topic ownership and transport intent. It is not a request
to make every Blueprint wire use DDS. Internal Module wires remain local under
`Blueprint.route_contract(...)`; only `Blueprint.routed_delivery(...)` opts a
Blueprint into routed DDS/SHM delivery for matching explicit topics.

## Boundaries

- Real and real-equivalent simulation data planes must use native DDS / C++
  runtime contracts.
- The physical field command sink is `endpoint_only + driver`: native nav
  publishes the canonical command, and `lingtu-driver` is the sole hardware
  bridge to Brainstem.
- Python Module wiring remains for mock runs, CI harnesses, Gateway/status,
  semantic modules, visualization, and compatibility adapters.
- Runtime Graph YAML declares what a product or endpoint must expose before a
  report can claim real-equivalent behavior.

## Files

- `topics.yaml`: canonical topic, frame, schema, producer/consumer roles, and
  `port_bindings` that identify the endpoint or Module port touching the topic.
- `products/*.yaml`: product modes such as `map`, `nav`, `explore`, and
  `teleop_avoid`.
- `endpoints/*.yaml`: endpoint contracts such as `thunder_field`,
  `mujoco_native_dds`, and `sim_mujoco_live`.

## Real-Equivalent Rule

`thunder_field` and `mujoco_native_dds` must share the native field contract
topics that cross the process boundary:

```text
/tf
/tf_static
/lidar/raw_frame
/imu/raw
/slam/odometry
/slam/registered_cloud
/slam/map_cloud
/slam/saved_map_cloud
/slam/localization_health
/slam/localization_quality
/nav/command/request
/nav/command/ack
/nav/inspection/command
/nav/inspection/ack
/nav/inspection/status
/nav/inspection/evidence/request
/nav/inspection/evidence/result
/nav/traversability
/nav/global_path
/nav/local_path
/nav/way_point
/nav/cmd_vel
```

`/nav/semantic/instruction` remains an in-process Gateway -> SemanticPlanner
channel. Only its resolved goal enters native typed `/nav/command/request`;
`/nav/command/ack` confirms business acceptance or rejection.

Inspection route lifecycle has its own typed interface: the persistent C++
client publishes start/pause/resume/cancel requests on
`/nav/inspection/command`, waits for the matching business ACK on
`/nav/inspection/ack`, and the native endpoint publishes current route/point
progress on `/nav/inspection/status`. Direct goals and operator velocity
requests remain on `/nav/command/request`.

Inspection evidence has a separate worker bridge:
`/nav/inspection/evidence/request` carries frame/evidence work requests and
`/nav/inspection/evidence/result` carries the analysis result. These topics are
inspection evidence exchange, not motion or route-control topics.

`sim_mujoco_live` is intentionally marked as `module_sim_harness` and
`real_equivalent: false`. It can validate downstream Python Module graph
behavior, but it must not be used as evidence for native field-runtime closure.

## Validation

Use the Python helper in `src/runtime/graph`:

```python
from runtime.graph import assert_runtime_graph_valid

assert_runtime_graph_valid()
```

Focused tests live in `src/runtime/tests/test_runtime_graph_contract.py`.

Route and DDS binding validation:

```python
from runtime.route_contract import assert_route_contract_valid

assert_route_contract_valid(route="robot")
```

Focused route/DDS tests live in `src/runtime/tests/test_route_contract.py`.
