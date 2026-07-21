# Core Concepts

LingTu is a Module-First autonomous-navigation runtime. A product profile
declares the robot behavior; a Blueprint assembles the Module graph; typed
ports and explicit wires carry data through that graph. DDS, shared memory,
simulators, and ROS 2 are boundary mechanisms, not the business API.

> **Status:** Current product architecture<br>
> **Audience:** Developers, integration engineers, and reviewers<br>
> **Runs on:** Local development hosts, simulations, and field deployments
> **Start here:** Read this page before changing a Module, Blueprint, backend,
> transport, or profile.

## The model in one picture

```text
Product profile
  -> resolved configuration
  -> Blueprint (modules + wires + boundary contract)
  -> SystemHandle
  -> Module ports
  -> local calls or an explicit external transport

Operator / UI / MCP intent
  -> Gateway or goal service
  -> mission and planning
  -> safety and command ownership
  -> selected driver or native endpoint
```

The words above describe different decisions. Keeping them separate is the
main way LingTu stays replaceable across a stub, simulation, and field robot.

| Term | Answers | Does not answer |
| --- | --- | --- |
| **Profile** | What product behavior is requested? | Which process owns every I/O boundary? |
| **Resolved configuration** | Which product, robot preset, endpoint, and overrides apply? | How individual Modules exchange a message. |
| **Blueprint** | Which Modules exist and how their ports connect? | How a planner computes a path. |
| **Module** | Which focused runtime capability owns this behavior? | Which transport carries every message. |
| **Port** | What typed data enters or leaves one Module? | Which other Module is allowed to consume it. |
| **Wire** | Which output feeds which input? | Whether that connection crosses a process. |
| **Transport / endpoint** | How an explicit boundary moves data or commands? | The product meaning of that data. |

The canonical architecture overview is [System design](../architecture/SYSTEM_DESIGN.md).
The detailed vocabulary and cleanup rules are in the
[Module/service boundary](../architecture/MODULE_SERVICE_BOUNDARY.md).

## Module: the runtime unit

A `Module` is the only normal in-process runtime unit. It owns one focused
capability, its configuration, state, typed input/output ports, and lifecycle.
It should be possible to test its behavior with `runtime.msgs` objects and
ordinary function calls, without a robot, ROS client, DDS client, HTTP client,
or systemd service.

Modules declare ports as class annotations. The `Module` base class discovers
`In[T]` and `Out[T]` annotations when it constructs an instance and makes them
available through `ports_in`, `ports_out`, and `all_ports`.

```python
from runtime.module import Module
from runtime.stream import In, Out


class SampleFilter(Module):
    sample: In[float]
    filtered_sample: Out[float]

    def setup(self) -> None:
        # "latest" prevents a slow callback from accumulating stale samples.
        self.sample.set_policy("latest")
        self.sample.subscribe(self._on_sample)

    def _on_sample(self, value: float) -> None:
        self.filtered_sample.publish(value)
```

### Lifecycle

The normal lifecycle is:

```text
construct -> preflight() -> setup() -> start() -> running -> stop()
```

| Hook | Use it for | Do not use it for |
| --- | --- | --- |
| `__init__(**config)` | Save inexpensive configuration and initialize local state. | Opening devices, starting threads, or assuming other Modules exist. |
| `preflight()` | Report a missing model, artifact, capability, or required configuration before startup. Return `None` when ready, otherwise a human-readable reason. | Recovering by silently selecting another robot or backend. |
| `setup()` | Subscribe input ports, create local helpers, and load resources needed before running. | Choosing the global graph or reaching across layers. |
| `start()` | Start the Module's runtime behavior. | Connecting arbitrary external product APIs from business logic. |
| `stop()` | Release Module-owned resources. It is designed to be idempotent. | Stopping unrelated Modules or system services. |
| `on_system_modules()` | Limited system-level discovery such as MCP skill discovery after the graph is built. | Replacing declared wires with hidden dependencies. |

`Blueprint.build()` creates a `SystemHandle`; `SystemHandle.start()` performs
preflight, setup, and start in dependency order. A Module failure is reported
as failed startup state rather than automatically changing the graph. Inspect
`SystemHandle.health()` and `SystemHandle.comm_health()` when debugging a
local graph.

### Module versus service versus endpoint

These names are deliberately not interchangeable.

| Kind | Boundary | Example responsibility |
| --- | --- | --- |
| Module | In-process ports and lifecycle | Mission state, map facade, gateway, safety evaluator. |
| Internal service class | Direct function calls inside an owning Module/domain | Planner request shaping or map-artifact validation. |
| System service | OS-supervised process | Native LiDAR, SLAM, or navigation runtime. |
| Adapter / bridge | External protocol boundary | DDS, simulator, hardware, or ROS compatibility conversion. |
| Native endpoint | Typed process boundary, normally C++ for field hot paths | Native navigation command/path endpoint. |

For example, `PlannerService` is an internal planning interface used by
navigation; it is not a systemd service and it does not publish motor commands.
The field native navigation endpoint is a separate process boundary. Read the
[Module/service boundary](../architecture/MODULE_SERVICE_BOUNDARY.md) before
creating a new `*Service`, adapter, or endpoint.

## Ports: typed local contracts

An `Out[T]` publishes values. An `In[T]` accepts one subscriber callback and
receives values from an explicit wire or transport binding. `T` documents and
helps validate the Python-side contract; it is not automatically a cross-language
serialization schema.

```text
producer.out: Out[MapCloudFrame]
  -> explicit Blueprint wire
consumer.in_: In[MapCloudFrame]
```

Use a message type from [`src/runtime/msgs/`](../../src/runtime/msgs/README.md)
when the data is a shared in-process contract. Keep the message transport-neutral:
it describes data, not a ROS topic, DDS reader, socket, or file path. Cross-process
DDS types and IDL/C++ bindings live under `src/message/`, not in a normal Module
port declaration.

### Input delivery policy

Every `In[T]` begins with the `all` policy. A consumer may select a different
policy in `setup()` when its work cannot keep up with its producer.

| Policy | When to use it | Effect |
| --- | --- | --- |
| `all` | Control/status events where every event matters. | Calls the subscriber for each delivered value. |
| `latest` | Camera, point cloud, or slow semantic work. | Keeps the newest value while the callback is busy; old work is dropped. |
| `throttle` | A fast source with a known maximum useful rate. | Delivers no more often than `interval`. |
| `sample` | Periodic sampling is sufficient. | Delivers every `n`th value. |
| `buffer` | A consumer genuinely needs batches. | Collects `size` values and delivers a list. |
| `async` | A bounded, single-worker asynchronous consumer is appropriate. | Runs the callback in a background executor. |

Choose a policy from the downstream correctness requirement, not merely to hide
performance problems. Safety-critical input should normally fail closed on
staleness rather than continue with an arbitrary old value. Port diagnostics
include message counts, rate, drops, callback errors, callback timing, and
staleness.

## Blueprint: declarative orchestration

A `Blueprint` is a graph declaration before it starts. It is the only normal
place to decide:

1. which Module classes or instances participate;
2. which configuration reaches each Module;
3. which important ports are wired; and
4. which external boundary contract applies.

It is not a place to run SLAM, build a map, open a socket, implement a Gateway
route, or encode navigation policy. Those actions belong to an owning domain
Module, service, adapter, or kernel.

```python
from runtime.blueprint import Blueprint

bp = (
    Blueprint("sample-flow")
    .add(SampleSource)
    .add(SampleFilter)
    .wire("SampleSource", "sample", "SampleFilter", "sample")
)

system = bp.build()
system.start()
```

An alias passed to `add(..., alias="...")` is the Module identity used by
later wires. Without an alias, the class name is used. `build()` instantiates
the Modules, checks explicit port names and type compatibility, applies wires,
calculates startup order, and notifies Modules about the completed graph.
Treat a Blueprint as immutable after `build()`.

### Explicit wires first

Use `wire(out_module, out_port, in_module, in_port, ...)` for safety paths,
cross-stack inputs, fan-in/fan-out that matters to product behavior, and any
connection where the correct producer is not obvious.

`auto_wire()` is available for unambiguous convenience wiring: it connects only
remaining ports with the same name and message type, allows one output to fan
out, and skips ambiguous input matches with a warning. It does not replace
explicit safety, navigation, hardware, or external-boundary wiring.

Critical full-stack wires are organized under
[`src/lingtu/assembly/wires/`](../../src/lingtu/assembly/wires/). The
[Blueprint guide](../../src/lingtu/assembly/README.md) explains the intended
order: profile -> product Blueprint -> stack factories -> explicit wires ->
route contract -> build.

### Stack factories and product assembly

Stack factories in `src/lingtu/assembly/stacks/` add small reusable groups,
such as maps, safety, navigation, or gateway. They do not decide a product
mission. Product-level assembly lives in `products/`, and
`profile_builder.py` is the profile-to-Blueprint entry point used by the CLI,
SDK, smoke tests, and robot startup.

This split makes a change reviewable:

```text
profile configuration
  -> product assembly chooses modules
  -> stack factories add module groups
  -> wire files declare important data flow
  -> domain Modules implement behavior
```

## Wire and transport are separate decisions

A wire declares semantic routing; a transport declares delivery mechanics.
Most Module-to-Module traffic remains in process and uses direct callbacks or
`LocalTransport` by default.

| Boundary | Preferred mechanism | Why |
| --- | --- | --- |
| Two Modules in one process | Typed `In`/`Out` plus an explicit wire | Lowest complexity and no serialization boundary. |
| High-volume traffic on one host | Explicit shared-memory adapter when the schema and performance gate exist | Avoid unnecessary copies while keeping ownership explicit. |
| Field service or cross-language boundary | Typed CycloneDDS endpoint contract | Stable IDL/schema/QoS ownership across processes. |
| Replay, test, or compatibility seam | Explicit local/LCM/ROS adapter selected by its endpoint/profile | Keeps legacy format out of product business code. |

`Blueprint.wire(..., delivery=..., topic=...)` can select a per-wire delivery
mechanism for an explicit experiment or compatibility seam. New product sensor,
SLAM, and navigation paths must **not** become generic `delivery="dds"` wires
sprinkled throughout the graph. They use typed endpoint contracts and native
adapters with fixed message schemas and QoS.

### Route contract versus routed delivery

These two APIs intentionally have different effects.

| API | Effect |
| --- | --- |
| `route_contract(...)` | Attaches external topic/schema/ownership metadata and validates the boundary. It does not move ordinary Module wires off local delivery. |
| `routed_delivery(...)` | Explicitly routes selected, explicitly topiced Module wires through the chosen backend. Use only when that transport move is the deliberate feature. |

The legacy `Blueprint.route(...)` spelling means routed delivery and is
deprecated for new code. The transport package and
[runtime bus decision](../architecture/LINGTU_RUNTIME_BUS_DECISION.md) provide
the full policy.

## Profiles, robot presets, endpoints, and sessions

These concepts are commonly collapsed into a single word such as "mode." They
should not be.

```text
profile -> product intent and high-level behavior
robot preset -> hardware-facing defaults
runtime endpoint -> where data/commands cross a boundary
user overrides -> explicit final changes
```

The resolver merges them in that order: product profile, robot preset, robot
runtime defaults, endpoint adaptation, then user overrides. A profile alias is
normalized before resolution; use canonical names in new documentation and
configuration. Ask the running CLI for the current catalog instead of copying
a stale profile list:

```bash
uv run --locked python lingtu.py --list
uv run --locked python lingtu.py --list --all
```

The active runtime also has separate operator-facing state:

| Field | Meaning | Example |
| --- | --- | --- |
| `profile` | Product behavior selected for startup. | `nav` |
| `endpoint` | I/O boundary/data source. | `thunder_field` or a simulation endpoint |
| `session_mode` | Coarse resource session. | `mapping`, `navigating`, `exploring` |
| `product_session` | Operator task shown in the product. | `navigation`, `inspection`, `teleop` |
| `slam_mode` | SLAM is mapping or localizing. | `mapping`, `localization` |

For a physical field navigation profile, the usual shape is a local Python
Module graph plus a typed DDS endpoint boundary. That does not mean every
Python Module speaks DDS. It means the native field endpoint owns the typed
process boundary while Modules retain their normal local contracts.

See the [product mode runtime contract](../architecture/PRODUCT_MODE_RUNTIME_CONTRACT.md)
for profile/session rules and
[`src/runtime/profiles/resolver.py`](../../src/runtime/profiles/resolver.py)
for the resolution order.

## Backends and the registry

Backends are selectable implementations behind a stable category and contract:
drivers, detectors, encoders, planners, LLM clients, local planners, and other
pluggable surfaces are registered by name.

```python
from runtime.registry import register


@register("detector", "my_detector", description="Example detector backend")
class MyDetector:
    ...
```

Factories resolve the selected class with `runtime.registry.get(category, name)`;
they should not import every concrete backend into business logic. The registry
also carries optional priority, platform, and description metadata. New backend
work must include the owning factory/catalog path, contract validation, and
targeted tests—not merely the decorator.

An in-place backend swap is deliberately fail-closed by default:
`Module.reconfigure_backend()` reports unsupported unless a Module explicitly
implements a safe switch contract. Do not promise live switching just because a
backend is registered.

## Native DDS product path and ROS compatibility

LingTu is native-first, not "ROS renamed." The current field product path uses
typed CycloneDDS at native service boundaries and direct calls/local ports
inside a process.

```text
native sensor service
  -> native SLAM/localization service
  -> Module graph for mission, maps, status, and gateway
  -> native navigation endpoint
  -> typed DDS /nav/cmd_vel
  -> unique lingtu-driver
  -> remote Brainstem gRPC
```

| Native product path | ROS 2 compatibility path |
| --- | --- |
| Typed DDS/IDL contracts at field service boundaries. | Explicit adapter, legacy service, simulator bridge, or comparison workflow. |
| Normal product Modules use `runtime.msgs`, typed ports, and pure helpers. | ROS message conversion stays in `*/adapters/ros2/` or another quarantined boundary. |
| The native field navigation endpoint owns the final `/nav/cmd_vel` writer, and `lingtu-driver` is the only hardware consumer in the default field chain. | A ROS topic is an adapter alias, not a new business API. |
| Product diagnostics use Gateway, ModulePort/dataflow, runtime-contract, and native-service evidence first. | Use ROS inspection only when an explicit compatibility test calls for it. |

Do not import ROS clients, ROS messages, or `cyclonedds` directly into a normal
planning, perception, decision, or product Module. Do not start a legacy ROS
service beside the native DDS chain unless a compatibility test explicitly
requires it; duplicate sensor or command owners are unsafe.

The exact topic/frame rules remain in the
[frame contract](../architecture/ros_frame_contract.md) and
[ROS role replacement map](../architecture/ROS_ROLE_REPLACEMENT_MAP.md).

## Invocation is not a data wire

`@rpc` marks a Module method as callable across Module boundaries. `@skill`
marks an RPC method as an AI-callable subset; `MCPServerModule` discovers those
skills after the graph is built. Use these for request/response actions, not as
a substitute for continuous typed data flow.

```text
continuous state or sensor data -> In[T] / Out[T] ports and wires
request/response action         -> explicit RPC or @skill contract
external HTTP / WS / MCP        -> Gateway or MCP boundary, then a goal/service contract
```

External callers submit intent and observe status. They must not bypass
navigation, safety, map validation, or velocity ownership by addressing a
driver directly.

## Safety and ownership invariants

The following rules are architectural invariants, not merely style choices.

1. A goal is not a motor command.
2. All velocity candidates pass through the designated command/safety
   ownership path; modules do not write directly to a physical driver.
3. Field navigation has one final native command writer for its selected
   endpoint; do not add a second Python or ROS writer.
4. High layers consume lower-layer outputs through ports/messages, not direct
   imports into their implementation packages.
5. A transport change must preserve message schema, frame, timestamp, QoS,
   and ownership—not just make bytes arrive.

The [navigation compute contract](../architecture/NAVIGATION_COMPUTE_CONTRACT.md)
and [global planning contract](../architecture/GLOBAL_PLANNING_CONTRACT.md)
define the detailed planning, safety, and map boundaries.

## Where to go next

| Goal | Next page |
| --- | --- |
| Add or change product code safely | [Develop LingTu](../03-development/README.md) |
| Build a first local/simulation environment | [Get Started](../01-getting-started/README.md) |
| Run a map, route preview, navigation, or exploration workflow | [Task guides](../05-guides/README.md) |
| Diagnose an existing system | [Troubleshooting](../03-development/TROUBLESHOOTING.md) |
| Operate a deployed robot | [Operations](../06-operations/README.md) |
| Look up contracts, APIs, and CLI surfaces | [Reference](../08-reference/README.md) |
