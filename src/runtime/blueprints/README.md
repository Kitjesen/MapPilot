# Runtime Blueprints

Blueprints are the orchestration layer. They answer only three questions:

1. Which Modules start for a profile or product mode?
2. Which ports are wired explicitly?
3. Which external adapters are allowed at the runtime boundary?

Do not put algorithms, drivers, map building, SLAM logic, planner logic, or
transport implementations here.

## Main Path

```text
profile_builder.py
  -> products/
  -> stacks/
  -> wires/
```

`profile_builder.py` is the only profile-to-Blueprint entrypoint used by the
CLI, SDK, smoke tests, and robot-side startup.

`products/` owns product-level assembly. Product profiles should route through
`products/thunder.py`. There is no generic compatibility Blueprint builder in
the product path.

`stacks/` owns small Module groups such as hardware, SLAM, maps, navigation,
safety, and gateway. A stack factory returns Modules and config only; it does
not decide mission behavior.

`wires/` owns explicit cross-stack connections. Critical robot behavior should
be visible here, not hidden in Module constructors.

`adapters/` owns Module choices at external boundaries: driver runtime,
mapping/SLAM bridge, navigation IO, and perception/gateway IO.

## Management Model

Blueprint management is not a separate manager service. It is a fixed set of
small management surfaces:

| Surface | Owns | Must not own |
| --- | --- | --- |
| `profile_builder.py` | Resolve a profile into one product graph. | Domain logic, module callbacks, filesystem mutation. |
| `products/` | Product-level composition such as `thunder`, `thunder_lite`, map/nav/explore variants. | Algorithms, map building, HTTP routes. |
| `stacks/` | Small reusable module groups: driver, lidar, slam, maps, navigation, safety, gateway. | Cross-stack behavior hidden in constructors. |
| `wires/` | Explicit important connections between stacks. | Module creation, planner policy, data conversion. |
| `route_contract()` | External topic/schema metadata. | Moving internal wires to DDS by accident. |
| `routed_delivery()` | Deliberate routed delivery for selected explicit wires. | Default product composition. |
| `export_graph()` / graph tests | Introspection, snapshots, static boundary checks. | Runtime behavior. |

The product graph should be readable in this order:

```text
Profile config
  -> Product blueprint
  -> Stack factories add Modules
  -> Wire files connect ports
  -> Route contract declares external topics
  -> Blueprint.build() instantiates runtime Modules
```

If a change cannot be explained in that order, it probably belongs in a
domain Service, Module, Adapter, or Kernel instead of Blueprint.

## Blueprint Boundary Rules

Allowed in Blueprint code:

- choose Module classes;
- pass configuration into Module constructors;
- assign stable aliases;
- declare wires and route contracts;
- select optional adapters at known external boundaries;
- export graph/introspection metadata.

Forbidden in Blueprint code:

- run subprocesses;
- open network sockets or HTTP clients;
- build maps, run SLAM, plan paths, or compute costs;
- read or write map artifacts;
- implement Gateway routes;
- encode domain policy that belongs to a Service or Agent.

Existing compatibility exceptions should be moved out rather than copied.

## Product Modes

Each product mode is a profile-level graph, not a separate runtime framework.

| Mode | Graph intent |
| --- | --- |
| `teleop` | Gateway/teleop commands through `VelocityMux` to the driver boundary. |
| `teleop_avoid` | Teleop plus localization, map layers, and velocity collision guard. |
| `map` | Sensor/localization data into map layers and map save/build services. |
| `tracking` | Localization plus local goal tracking and path following. |
| `nav` | Saved map plus OctoPlanner3D, local planner, path follower, safety, and command mux. |
| `inspection` | Navigation plus perception, semantic planner, memory, and task outputs. |

## Compatibility

`stub.py` remains the no-hardware framework test Blueprint.

`full_stack_wiring.py` remains the shared wire collector used by
`products/thunder.py`. It is not a standalone runtime entrypoint.

## Moved Out

Profile and Module graph inspection live in `runtime.introspection`.

Simulation runtime contracts live in `runtime.contracts`.

Profile catalogs live in `runtime.profiles.catalog`. There is no
`runtime.blueprints.catalog` source of truth.
