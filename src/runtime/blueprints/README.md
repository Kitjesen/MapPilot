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

## Product Modes

Each product mode is a profile-level graph, not a separate runtime framework.

| Mode | Graph intent |
| --- | --- |
| `teleop` | Gateway/teleop commands through `CmdVelMux` to the driver boundary. |
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
