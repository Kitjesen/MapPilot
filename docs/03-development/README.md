# Develop LingTu

This guide explains how to change LingTu without breaking its Product, Host,
domain, adapter, or field command boundaries. It is a contributor
guide: use it to decide *where a change belongs*, implement the smallest
correct change, and collect evidence at the right level.

> **Status:** Current development guide<br>
> **Audience:** Contributors, maintainers, and integration engineers<br>
> **Runs on:** Local development hosts, simulations, and field-targeted builds
> **Prerequisite:** Read [Core concepts](../02-concepts/README.md) before
> changing Module wiring, transports, Products, or backends.

## Start from the requested outcome

Choose the owning surface before opening an editor. A correct implementation
in the wrong layer is still a maintenance problem.

| Requested change | Primary owner | Typical supporting surface |
| --- | --- | --- |
| New runtime lifecycle or typed port behavior | `src/runtime/` | Runtime unit tests and a Blueprint/wire change if the graph changes. |
| Product declarations or runtime defaults | `config/runtime_graph/products/` and `src/lingtu/assembly/` | Product resolution and focused compiler tests. |
| Navigation commands, goals, Agent skills, inspection, or building flow | `src/nav/commands/`, `src/nav/services/`, `src/nav/skills/`, `src/nav/inspection/`, or `src/nav/building/` | Typed native-client and Host wiring tests. |
| Global/local planning, tracking, recovery, or final motion safety | `src/nav/cpp/` | Native CMake tests plus Host/native boundary tests. |
| Saved maps, map artifacts, or persistent map lifecycle | `src/maps/` | Map service/artifact contract and no-motion validation. |
| Perception, semantic reasoning, LLM, memory, or visual servo behavior | `src/perception/`, `src/decision/`, or `src/memory/` | Runtime messages and explicit full-stack wires. |
| Device SDK, robot connection, camera/LiDAR/IMU source | `src/drivers/` or a native endpoint | Explicit hardware/DDS adapter contract. |
| Localization/SLAM behavior or native localization status | `src/localization/` | Native endpoint/service contract and frame validation. |
| REST, SSE, WebSocket, MCP, or dashboard API behavior | `src/gateway/` | Typed request/status contract; never planner internals. |
| C++ hot-path algorithm | `src/nav/cpp/` or the owning native package | CMake/native tests plus Python boundary tests. |

The repository map in [REPO_LAYOUT.md](../REPO_LAYOUT.md) is the authoritative
directory index. Package-level READMEs are the first source for local ownership:
[`src/runtime/README.md`](../../src/runtime/README.md),
[`src/nav/README.md`](../../src/nav/README.md), and the matching domain package.

## Non-negotiable boundaries

LingTu's source layout reflects runtime ownership. Keep these rules intact:

```text
All product Modules -> runtime/ shared contracts and utilities

nav/        must not import perception/, decision/, drivers/, gateway/
perception/ must not import nav/, decision/, drivers/, gateway/
decision/   consumes perception outputs through runtime messages and wires
drivers/    do not import navigation/semantic business logic
gateway/    submits requests and displays status; it does not implement navigation
```

| Rule | Practical consequence |
| --- | --- |
| Module is the Host runtime unit. | Put Host-local lifecycle, port subscriptions, and local state in the owning Module; do not treat it as a native process manager. |
| Blueprint assembles one Host graph. | Module constructors do not discover or wire the Host graph. Product processes remain in Assembly/ProductControl. |
| Message before backend. | Domain code depends on `runtime.msgs` and contracts, not a concrete DDS/ROS/simulator class. |
| Adapter isolation. | SDK, DDS, ROS, HTTP, simulator, file, and OS-process code stay at an explicit boundary. |
| Goal is not a motor command. | A new API, skill, or policy emits a goal/intent into the navigation path; it does not write a physical command. |
| One field command owner. | Do not add a Python or ROS `cmd_vel` writer beside the Product's native command Endpoint. |

The detailed source-of-truth documents are the
[system design](../architecture/SYSTEM_DESIGN.md),
[runtime bus decision](../architecture/LINGTU_RUNTIME_BUS_DECISION.md), and
[Module/service boundary](../architecture/MODULE_SERVICE_BOUNDARY.md).

## The development loop

Use this sequence for ordinary changes. It prevents architecture drift while
keeping the diff small and reviewable.

1. **Name the user-visible contract.** State what input, output, Product, or
   safety behavior changes. Identify whether it is local,
   simulation-only, or affects the field `env=real` RunPlan.
2. **Find the current owner.** Read the package README, current architecture
   contract, and existing tests before adding an abstraction or import.
3. **Choose the narrowest boundary.** Decide whether the work is a pure helper,
   Module, internal service, Blueprint/wire, backend, adapter, or native
   endpoint. Do not use a new Module merely to wrap a small pure function.
4. **Define or reuse the contract.** Reuse a typed runtime message where
   possible. When the contract crosses a process/language boundary, define its
   schema, frame, timestamp, QoS, error behavior, and owner first.
5. **Implement one focused slice.** Preserve existing behavior unless the task
   explicitly changes it. Avoid cross-layer imports and compatibility shortcuts.
6. **Wire/configure deliberately.** Add the Module through the existing product
   assembly and make critical wires explicit. Resolve backends through the
   registry/factory rather than importing a concrete implementation in business
   logic.
7. **Validate the claim.** Start with targeted unit or contract tests; then run
   a simulation, no-motion, native, or field gate only when that is what the
   claim requires.
8. **Update the documentation contract.** Update a task guide, reference,
   configuration note, or troubleshooting route when a user-visible behavior
   changes.

Do not treat a passing import, an alive systemd unit, or a successful local
test as proof of field safety. Those are different claims with different gates.

## Know the layers before you modify them

### Runtime and product assembly

`src/runtime/` owns the small shared framework: `Module`, `In`/`Out`,
`Blueprint`, the registry, shared messages, transport abstractions, and graph
introspection. It must stay free of navigation policy,
device SDK behavior, gateway routes, and ROS-node business logic.

For a Product change, trace this path:

```text
config/runtime_graph/products/ + envs/
                               resolve Product + env into RunPlan
  -> lingtu/assembly/compiler.py
                               select product Blueprint and validate route contract
  -> lingtu/assembly/products/
                               product-level Module composition
  -> lingtu/assembly/stacks/
                               reusable Module groups
  -> lingtu/assembly/wires/
                               explicit critical connections
```

`compiler.py` is the Product-to-RunPlan entry used by ProductControl and
robot-side startup. Do not introduce a parallel
"special" builder for a new product feature.

### Domain packages

| Package | Owns | Must not absorb |
| --- | --- | --- |
| `src/nav/` | Host commands, goals, skills, inspection/building facades, native adapters, and the native C++ navigation implementation. | Gateway/UI logic, device SDKs, perception/decision implementation. |
| `src/maps/` | Map packages, artifacts, map queries/control, and saved-map lifecycle. | Per-tick navigation policy or gateway presentation. |
| `src/perception/` | Detection, encoding, tracking, reconstruction, scene products. | Navigation mission decisions or robot drivers. |
| `src/decision/` | Goal resolution, LLM tools, semantic planning, visual servo intent. | Direct driver control or navigation implementation imports. |
| `src/memory/` | Semantic, episodic, tagged, vector, and temporal memory. | Transport-specific adapters. |
| `src/drivers/` | Hardware and simulation I/O adapters. | Goal selection, planning, or semantic behavior. |
| `src/localization/` | SLAM/localization implementations and status normalization. | Gateway request policy. |
| `src/gateway/` | REST/SSE/WS/MCP boundaries, request validation, status/media exposure. | Planner algorithms or direct field command ownership. |

When a proposed change appears to belong in two domain packages, it usually
needs a typed message and an explicit Blueprint wire, not a new direct import.

## Add or change a Module

### 1. Reuse a message contract

Look in [`src/runtime/msgs/`](../../src/runtime/msgs/README.md) and
`src/runtime/contracts/` before creating a new payload. An in-process message
must describe domain data, not a topic name or serialized transport object.

Create a new shared message only when all of these are true:

- an existing message cannot express the semantics without ambiguous optional
  fields;
- more than one Module/domain needs the contract; and
- the type can remain transport-neutral.

If the data is internal to one Module, use a private dataclass or helper value
instead. If it crosses a product process boundary, design a typed endpoint/IDL
contract rather than assuming a Python dataclass can be serialized safely.

### 2. Declare ports and lifecycle explicitly

```python
from runtime.module import Module
from runtime.stream import In, Out


class QualityGateModule(Module):
    measurement: In[float]
    accepted: Out[float]

    def __init__(self, *, minimum: float = 0.0, **config: object) -> None:
        super().__init__(**config)
        self._minimum = minimum

    def preflight(self) -> str | None:
        if self._minimum < 0.0:
            return "minimum must be non-negative"
        return None

    def setup(self) -> None:
        self.measurement.subscribe(self._on_measurement)

    def _on_measurement(self, value: float) -> None:
        if value >= self._minimum:
            self.accepted.publish(value)
```

Guidelines:

- Keep `__init__` cheap and deterministic. It should not start a device,
  thread, process, network client, or hidden subscription.
- Use `preflight()` to report missing declared prerequisites. Return a reason;
  do not silently swap the profile, map, sensor, or backend.
- Subscribe each `In` port once in `setup()`. Select a delivery policy there
  only when the consumer's correctness permits it.
- Keep callback work bounded. Move a genuine expensive computation behind a
  domain service, worker design, or native kernel after measuring it.
- `stop()` owns cleanup of resources created by that Module only. It must be
  safe to call more than once.
- Use `@rpc` for a request/response action and `@skill` only when that RPC is
  intentionally part of the MCP/agent tool surface. Do not use RPC calls as a
  hidden continuous data bus.

Read the [Module lifecycle and port rules](../02-concepts/README.md#module-the-runtime-unit)
for the behavior of `SystemHandle` and `In` policies.

### 3. Place the Module in the graph

Adding a Python class alone does not make it Product behavior. Place it through
the existing Product/assembly/stacks/wires path and give every critical input
and output an explicit wire.

```python
bp.add(QualityGateModule, alias="quality.gate", minimum=0.5)
bp.wire("SensorModule", "measurement", "quality.gate", "measurement")
bp.wire("quality.gate", "accepted", "ConsumerModule", "measurement")
```

Use aliases when a class has multiple instances or when the graph needs a
stable semantic identity. Do not change an existing alias casually: wires,
graph snapshots, diagnostics, and external contracts can rely on it.

### 4. Test the behavior and the wiring separately

Test a Module's decision logic with direct `runtime.msgs` values or its input
port. Test Product graph selection and critical wires through the owning
assembly tests. This keeps a failure actionable: either the Module is
wrong, or the graph is wrong.

## Change Products, Blueprints, or wires

### Blueprint rules

Blueprint code may choose Modules, pass constructor configuration, assign
aliases, declare wires, select known boundary adapters, and attach a route
contract. It must not implement domain algorithms, open client connections,
write map artifacts, run an external command, or hide a policy callback.

For a graph change:

1. Start at `lingtu/assembly/products/` for a product behavior change.
2. Reuse or extend a small factory in `stacks/` for a reusable Module group.
3. Add critical cross-stack data flow in `wires/` rather than a constructor
   lookup or a new direct import.
4. Add/adjust a targeted graph or wire test.
5. Confirm the selected Product does not create duplicate hardware, sensor, or
   command owners.

`Blueprint.auto_wire()` is only a convenience for unique same-name,
same-type matches. Use explicit `wire()` calls for safety, command, map,
localization, external-boundary, and ambiguous connections.

### Products resolve inside one env

```text
Product + env -> RunPlan -> ProductControl -> systemd
```

Product declarations are env-independent. `env=real` references static
RobotConfig internally and selects typed DDS/native process boundaries; `env=sim`
selects an internal simulation backend when required. Do not infer env from a
hostname, insert a hardware address into product source, or make a normal
Module branch on simulator/ROS/DDS details.

Current Thunder field command ownership is:

```text
lt-nav -> /nav/cmd_vel -> lingtu-driver -> remote Brainstem gRPC
```

`lt-driver.service` conflicts with legacy Python Thunder DDS endpoint
units by design. A different physical command writer or a localhost Brainstem
target is an architecture change, not an ordinary development shortcut.

For field-side resolution evidence, use the operations CLI's non-motion
inspection surface:

```bash
python -m lingtu.control switch nav --robot unitree/go2 --env real --map MAP_NAME --dry-run --json
python -m pytest src/runtime/tests/test_runtime_graph_contract.py -q
python tools/validate/validate_architecture_boundaries.py
python tools/validate/validate_topics.py
```

These commands describe the selected Product and validate its repository contracts; they are not a
replacement for a field-readiness or motion authorization gate.

### Route contracts do not silently change local delivery

Use `Blueprint.route_contract(...)` to declare external topic/schema ownership
for a graph. It is metadata and validation; it never changes Module delivery.

The default physical field runtime is a typed native DDS endpoint boundary.
Do not model it by adding generic `delivery="dds"` to ordinary navigation wires.
The exact policy is in [runtime bus decision](../architecture/LINGTU_RUNTIME_BUS_DECISION.md)
and [Native runtime](../architecture/NATIVE_RUNTIME.md).

## Add a selectable backend

Use the registry and the owning factory when a Host implementation is
intentionally pluggable. A registry entry is not enough by itself; the factory,
catalog, configuration, and tests must agree on the stable interface.

Native global and local planners are not Python registry plugins. Select their
supported backend in the Product's `native_nav` declaration and implement the
backend behind the native C++ planner interface in `src/nav/cpp/`.

Before registering a backend, answer:

| Question | Required answer |
| --- | --- |
| What stable interface does the owner call? | Name the protocol/methods and error behavior. |
| Which category and canonical name identify it? | Reuse an existing category unless a new domain contract is justified. |
| Where is it seeded/resolved? | Identify the owning stack/factory or runtime registration path. |
| Which Products may select it? | Make capability/platform restrictions explicit. |
| Can it switch at runtime? | Assume no; `reconfigure_backend()` is fail-closed unless explicitly implemented and tested. |
| How is it validated? | Add contract and selected-backend tests, then the relevant performance/native gate. |

Business logic should call `runtime.registry.get(...)` through its owning
factory; it should not grow a chain of direct concrete-backend imports.

## Add an adapter or native process boundary

First decide whether the feature is truly external. This choice determines the
package, test shape, and failure model.

| Need | Correct place | Important requirement |
| --- | --- | --- |
| Pure calculation or domain policy | Owning Module/service/kernel | No DDS, ROS, HTTP, SDK, or filesystem protocol imports. |
| Same-process Module data flow | Typed port and Blueprint wire | Keep the message transport-neutral. |
| Hardware SDK, device ownership, reconnect loop | Driver/native endpoint | One clear process owns the physical device. |
| Cross-process/cross-language product data | Typed endpoint/DDS contract | Define IDL/type, frame, timestamp, QoS, version, and owner. |
| Simulator/replay format | Explicit simulator or replay adapter | Do not leak its format into product Modules. |
| ROS 2 interop | `*/adapters/ros2/` or an explicit compatibility boundary | Opt-in only; never make it a default product dependency. |

### Native DDS path

The field product path uses typed CycloneDDS at native service boundaries.
Normal Modules still use local `runtime.msgs` ports and direct calls inside
their process. The field endpoint owns final navigation command publication;
do not add a competing Python DDS writer, ROS publisher, or direct driver
write.

For a new native boundary, document and test all of the following before
connecting it to a Product/env declaration:

- producer and consumer process ownership;
- typed schema/IDL and compatibility/version rule;
- frame ID, timestamp, covariance/quality semantics where relevant;
- QoS/backpressure and startup ordering;
- error, timeout, stale-data, and restart behavior;
- safety consequences and whether the boundary can influence motion;
- Product/env selection and diagnostic evidence.

Use the [runtime bus decision](../architecture/LINGTU_RUNTIME_BUS_DECISION.md) and
[topic/frame contracts](../architecture/TOPIC_CONTRACT_POLICY.md) as design
inputs. Do not add a dependency just to make a boundary easier to prototype.

### ROS compatibility is a separate product decision

ROS 2 remains available for explicit adapters, legacy services, simulator
bridges, and comparison work. It is not the normal field product path. A
`ros2: command not found` error is therefore not automatically a production
failure. Conversely, sourcing ROS or starting a legacy ROS service is not a
safe general repair: it can create duplicate sensor or command ownership.

If the task explicitly targets the compatibility path, follow the deployment
guide's [Legacy ROS2 compatibility](../04-deployment/README.md#legacy-ros2-compatibility)
section and add a compatibility-specific test. Keep ROS imports confined to
the adapter boundary.

## Navigation and safety changes

Navigation changes deserve an extra ownership review. The normal flow is:

```text
external intent
  -> Host navigation command/goal/skill surface
  -> typed native command
  -> navd: global planning -> local planning -> tracking -> final safety
  -> /nav/cmd_vel
  -> driver
```

Do not bypass an earlier stage because an API already has coordinates or a
velocity-shaped value. In particular:

- Gateway, MCP, web, semantic planner, and visual servo code submit intent to
  the correct command/goal surface.
- Global planning produces a path, not `cmd_vel`.
- Operator motion uses the native operator-motion command boundary.
- Field native navigation preserves one final command writer.
- Map frame, artifact provenance, localization freshness, and safety state
  remain input gates rather than optional diagnostics.

Read the [navigation compute contract](../architecture/NAVIGATION_COMPUTE_CONTRACT.md),
[global planning contract](../architecture/GLOBAL_PLANNING_CONTRACT.md), and
[map service contract](../architecture/MAP_SERVICE_CONTRACT.md) before
changing mission, map, planning, or safety code.

## Configuration, secrets, and maps

- Put durable robot/device calibration and product configuration in the
  existing `config/` sources. Do not copy hardware addresses or credentials
  into Python code, docs, or duplicate defaults.
- Keep API keys in environment variables; use the documented mock backend for
  deterministic/offline development.
- Treat a saved map as a package with provenance and planner artifacts, not a
  standalone point cloud. Changes to saved-map behavior need map-artifact
  validation and usually a no-motion planner preview.
- Tuning parameters belong with their established configuration source and
  require an explanation of units, default, range, safety impact, and active
  backend. See [Parameter tuning](./PARAMETER_TUNING.md).

## Verification matrix

Run the smallest command that can prove your actual change, then widen only
when a dependency requires it.

| Change | Minimum fresh evidence |
| --- | --- |
| `Module`, port, stream policy, registry, or runtime helper | Focused test in the owning package; for core runtime changes, start with `python -m pytest src/runtime/tests/test_runtime.py src/runtime/tests/test_registry.py -q`. |
| Stack factory, wire, Product compiler, or Product graph | Targeted assembly and graph tests for the changed Product. |
| Navigation/domain behavior | Focused `src/nav/tests/` test, then a wider navigation test only if the changed contract spans the package. |
| C++ hot path | Owning CMake build/test target plus a Python boundary/contract test if exposed to Python. |
| Map artifact or planner integration | Saved-map artifact gate in the relevant environment. |
| Field endpoint, localization, safety, or command path | Native service/dataflow evidence and the named field-readiness gate; simulation success is not field proof. |
| API or MCP contract | Request/response test plus generated reference/doc update where applicable. |
| Documentation links/navigation | The documentation navigation test and a local-link check. |

Useful contract checks include:

```bash
python -m pytest src/lingtu/assembly/tests/test_compile.py -q
python -m pytest src/nav/tests/ -q
```

For field-side, non-motion diagnosis use:

```bash
PYTHONPATH=src python -m diagnostics.field.doctor --non-motion --json --strict
PYTHONPATH=src python scripts/diagnostics/soak.py --duration 120 --interval 2 --json --strict
python scripts/gates/saved_map_artifact_gate.py <map-id> --require-occupancy
```

Only run a command that changes a robot session, sends a goal, publishes a
velocity, or restarts a field service when the task and operating authority
explicitly call for it. See [Troubleshooting](./TROUBLESHOOTING.md) for the
native-first diagnostic ladder.

## Review checklist

Before asking for review or declaring a change complete, confirm:

- [ ] The owning package and public contract are clear.
- [ ] No new cross-layer import, hardware address, secret, or unnecessary
      dependency was introduced.
- [ ] Modules use typed ports; important new graph edges are explicit wires.
- [ ] A transport/endpoint change names schema, frame, timestamp, QoS,
      ownership, and fallback/fail-closed behavior.
- [ ] Field command ownership is unchanged unless the task explicitly changes
      it and includes safety evidence.
- [ ] Backend selection goes through the owning registry/factory.
- [ ] Tests cover both changed behavior and Product/graph selection where
      relevant.
- [ ] Any limitation is documented as a limitation, not implied away by a
      local or simulation pass.
- [ ] User-facing workflow, API, configuration, or failure modes have a
      corresponding documentation update.

## Related documentation

| Need | Read |
| --- | --- |
| Module, Blueprint, Port, Wire, transport vocabulary | [Core concepts](../02-concepts/README.md) |
| Build and environment prerequisites | [Build guide](../01-getting-started/BUILD_GUIDE.md) |
| Current runtime architecture contracts | [Architecture index](../architecture/README.md) |
| Navigation, planning, map, and safety boundaries | [Task guides](../05-guides/README.md) |
| Native-first field diagnostics and explicit ROS compatibility | [Troubleshooting](./TROUBLESHOOTING.md) |
| Commit and push acceptance | [Commit and push policy](./COMMIT_PUSH_POLICY.md) |
| Deployment services and operator actions | [Deployment](../04-deployment/README.md) |
| Tests, simulation closure, and field evidence | [Testing and validation](../07-testing/README.md) |
| REST, MCP, and interface inventory | [Reference](../08-reference/README.md) |
