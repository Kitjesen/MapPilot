# LingTu Runtime Graph

Status: current runtime graph contract as of 2026-08-31.

Runtime Graph is the readable contract layer for Product and Env resolution.
It is not the runtime data plane.

## Runtime Control Model

The control plane has Product and Env declarations, one resolved plan, one
ProductControl owner, and one Host-local graph:

| Object | Owns | Must not own |
| --- | --- | --- |
| Product | One immutable operating mode: required capabilities, topics, Blueprint, and logical process roles. | Env selection, runtime side effects, or domain algorithms. |
| Env | The outer `real` or `sim` implementation of hardware, process roles, and transports. | Product policy or domain algorithms. |
| RunPlan | The immutable resolved record of one Product in one Env and the sole Host/systemd execution identity. | Re-resolving Product or Env policy at execution time. |
| Endpoint | A genuine communication access contract nested under an Env implementation. | Selecting a Product, Env, or simulation backend. |
| Blueprint | In-process Python Module construction and port wiring. | Native process startup or systemd ownership. |
| ProductControl | Publishes the resolved RunPlan, stages the transient session, applies systemd processes, checks readiness, and owns Product operations. | Domain algorithms or Module wiring. |

The machine path is:

```text
products/<product>.yaml + envs/<real-or-sim>.yaml
  -> resolve one RunPlan
  -> RunPlan(identity, launch, host, checks)
  -> ProductControl publishes that immutable artifact and session.env
  -> ProductControl.switch() -> internal SystemdRunner.apply(plan)
  -> the Env implementation's declared process owner
```

`scripts/lingtu` and Gateway both invoke this same ProductControl operation.
Neither one recompiles the Product or owns process ordering.

Each Product uses `lingtu.runtime_graph.product.v2`. Its `host.capabilities`
list is the environment-independent Host declaration; Assembly translates
those capability names into the existing Blueprint Module graph. Product
navigation settings stay under `native_nav`, and rolling launch parameters
stay under `parameters`. There is no Python Product-default table or named
parameter profile beside the YAML.

Products declare `processes` using stable logical roles such as `lidar`,
`slam`, `nav`, and `host`. The ProductControl-managed `real` Env maps those roles to
deployment `target` values. Topic names validate the selected Env
implementation; they are never used to infer which process should start.

Only an Env implementation with `process_control: systemd` exposes Product
processes to ProductControl. The `sim` Env uses the MuJoCo backend selected by
`env_config.backend`.
The simulation runner owns
their child-process setup, teardown, evidence capture, and timeouts as one
transaction. The control plane must not fabricate partial Product processes
for them.

The `real` Env is fail closed: every selected process target must be
installed, become active within its declared timeout, and pass its role-specific
readiness check. The RunPlan also carries all mode-owned and
conflicting targets, so switching Products removes stale process ownership
before startup. Product compilation resolves code defaults, Env parameters,
Product parameters, and the current `--set` values into `launch.parameters` in
RunPlan v8. Runners execute that final result and do not resolve parameters a
second time.

## Naming

Names reflect scope instead of repeating implementation details:

```text
logical process id: nav
executable:         navd
systemd unit:       lt-nav.service
Env:                real
```

The executable is short because its package and directory already establish
the product and domain. Systemd units use the short `lt-<function>.service`
namespace because units share one host-wide namespace. Transport details such
as DDS belong in the implementation and documentation, not in the process
identity. ProductControl logic still depends only on the logical `nav` id.

Runtime Graph plus `runtime.route_contract` describes the Host and native DDS
boundaries without duplicating navigation inside the Host:

```text
Blueprint wires define Host Module ports:
  host.bus.navigation_goal_status -> nav.goals.navigation_goal_status

RouteContract defines native cross-process transport:
  /slam/odometry -> rt/slam/odometry -> lingtu.dds.Odometry
```

Do not move DDS imports into normal Modules. Modules keep typed `In`/`Out`
ports; adapters and endpoint services translate between those ports and DDS.
Python code must get canonical runtime topics from
`runtime.runtime_interface.TOPICS`; see
`docs/architecture/TOPIC_CONTRACT_POLICY.md` for the static guard and allowed
boundary exceptions.

Runtime resolver now exposes the selected contract as `route_contract`.
For the `real` Env this should read:

```text
env=real
endpoint_contract=field_dds_v1
route_contract=robot
```

`route_contract` is topic ownership and transport intent. It does not change
Blueprint delivery: internal Module wires remain process-local, while native
endpoints own DDS communication.

## Boundaries

- Real and real-equivalent simulation data planes must use native DDS / C++
  runtime contracts.
- The physical field command sink is `endpoint_only + driver`: native nav
  publishes the canonical command, and `lt-driver.service` is the sole hardware
  bridge to the RobotConfig-selected Go2 or Thunder adapter.
- Python Module wiring remains for mock runs, CI harnesses, Gateway/status,
  semantic modules, visualization, and compatibility adapters.
- Runtime Graph YAML declares what a Product or Env implementation must expose before a
  report can claim real-equivalent behavior.

## Files

- `topics.yaml`: canonical topic, frame, schema, producer/consumer roles, and
  `port_bindings` that identify the endpoint or Module port touching the topic.
- `products/*.yaml`: product modes such as `map`, `nav`, `explore`, and
  `teleop_avoid`.
- `envs/real.yaml`: the shared physical implementation; the selected robot model owns its RobotConfig,
  ProductControl process-role ownership, and genuine endpoint contract.
- `envs/sim.yaml`: simulation backends, per-robot session bindings, process
  owners, Host configuration, and the genuine endpoint contract.

`operator_switchable` controls whether a Product appears in the normal
operator/Gateway mode catalog; it does not decide whether the compiler can
produce a RunPlan. Local-planner algorithms are not Products: `nav` remains the
single saved-map navigation Product, defaults to CMU, and may compile with
`local_planner="scan"`. MuJoCo acceptance scripts select their runner and
manifest explicitly; passing the SCAN target is evidence for a future
default-backend decision, not a second navigation mode.

Each Env implementation declares the algorithms it has qualified in
`local_planners`. Resolution fails before launch when a Product selects an
unqualified backend. The physical `real` Env currently qualifies only CMU;
MuJoCo simulation qualifies both CMU and SCAN.

## Real-Equivalent Rule

The `real` Env and the `sim` Env's `mujoco` backend must share the native
contract topics that cross the process boundary:

```text
/tf
/tf_static
/lidar/raw_frame
/imu/raw
/slam/odometry
/slam/state_at_scan
/slam/registered_cloud
/slam/map_cloud
/slam/saved_map_cloud
/slam/localization_health
/slam/localization_quality
/nav/command/request
/nav/command/ack
/nav/inspection/task/request
/nav/inspection/task/ack
/nav/inspection/status
/nav/inspection/task/event
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
client publishes task-addressed start/pause/resume/cancel requests on
`/nav/inspection/task/request`, waits for the matching business ACK on
`/nav/inspection/task/ack`, and the native endpoint publishes current
route/point progress on `/nav/inspection/status` plus ordered task facts on
`/nav/inspection/task/event`. Direct task goals remain on
`/nav/command/request`; operator velocity owns the separate
`/nav/operator_motion/control`, `/nav/operator_motion/sample`, and
`/nav/operator_motion/ack` contract.

Inspection evidence has a separate worker bridge:
`/nav/inspection/evidence/request` carries frame/evidence work requests and
`/nav/inspection/evidence/result` carries the analysis result. These topics are
inspection evidence exchange, not motion or route-control topics.

The canonical `mujoco` backend runs the C++ typed-DDS chain. The old Gazebo and
manual ROS2 localPlanner/pathFollower navigation stack is retired and is not a
Product backend. MuJoCo component scripts remain development tools, not Product
backends.

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
