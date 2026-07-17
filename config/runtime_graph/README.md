# LingTu Runtime Graph

Runtime Graph is the readable contract layer for product and endpoint wiring.
It is not the runtime data plane.

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
