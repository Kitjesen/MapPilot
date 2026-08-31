# LingTu Core

`src/runtime/` is the runtime foundation of LingTu.

Keep this folder small in meaning: it explains how modules exist, how modules
talk, and how runtime execution works. Product stack recipes live in
`src/lingtu/assembly`. Runtime should not contain navigation
decisions, perception algorithms, robot behavior, HTTP endpoint logic, or ROS
node logic.

## Read This First

| If you need to... | Start here |
| --- | --- |
| write or inspect a module | `module.py`, `stream.py` |
| understand the graph mechanism | `blueprint.py`, `wiring.py` |
| connect LingTu product modules | `../lingtu/assembly/wires/` |
| understand product data flow | `../lingtu/assembly/products/thunder.py`, then `../lingtu/assembly/wires/full_stack.py` |
| change Product/env defaults | `config/runtime_graph/`, `../lingtu/assembly/products/` |
| find shared messages | `msgs/`, `contracts/` |
| find topic, frame, or runtime contract names | `runtime_interface.py` |
| register or resolve a backend | `registry.py` |
| inspect runtime diagnostics and acceptance evidence | `../diagnostics/field/` |

## Mental Model

```text
Module      = one runtime unit
In / Out    = typed input and output ports
WireSpec    = one declared data-flow connection
Blueprint   = one application Module graph before it starts
Route       = runtime contract (metadata) and optional routed delivery mode
SystemHandle = the running graph
ProcessSpec = one env-resolved deployment process
Product     = one env-independent operating-mode declaration
RunPlan     = one Product resolved inside one env
ProductControl = the only Product transaction owner
SystemdRunner = ProductControl's internal process executor
```

Normal Module flow is:

```text
goal / sensor / map input
  -> Module ports
  -> explicit wires
  -> command adapters, business logic, or Gateway
  -> native endpoint request or user-facing status
```

Navigation uses the same native boundary in `real` and `sim`:

```text
GatewayModule.goal_pose
  -> nav.goals.goal_request
  -> native navigation command client
  -> lt-nav
  -> host.bus navigation status/path telemetry
  -> GatewayModule / NavSkills

sensor endpoint
  -> lt-slam
  -> lt-terrain
  -> lt-nav
  -> /nav/cmd_vel
  -> driver endpoint
```

`src/runtime` assembles Host command/status, map, and business Modules. Native
processes own planning, path following, arbitration, and final motion output.

## Runtime Routes

Routes choose the runtime path. Built-in presets live in
`runtime.route_contract`:

- `robot()`: physical robot route, typed DDS for sensor, SLAM, navigation,
  and command boundaries.
- `replay()`: replay/development route, typed LCM bindings where declared.
- `sim()`: in-process simulation route.

Blueprint records the external route contract for boundary validation and
topic naming:

```python
from runtime.route_contract import robot

system = (
    Blueprint()
    .route_contract(robot())
    .build()
)
```
This never changes Module-to-Module wiring.

DDS itself is started by systemd services on real hardware (for example
`lt-lidar.service`, `lt-slam.service`, and
`lt-nav.service`). The hardware command sink is the separate
`lt-driver.service`, which consumes the final typed DDS command and owns the
Brainstem lease. Internal validators still check DDS topic/type contracts
against `src/message/topics.py` and C++ topic constants.

## Minimal Example

```python
from runtime.module import Module
from runtime.stream import In, Out


class Doubler(Module):
    value: In[float]
    doubled: Out[float]

    def setup(self):
        self.value.subscribe(lambda x: self.doubled.publish(x * 2.0))
```

```python
bp.wire("SourceModule", "doubled", "SinkModule", "value")
```

## What Belongs Here

| Area | Examples |
| --- | --- |
| runtime unit model | `module.py`, `stream.py`, `blueprint.py` |
| shared contracts | `msgs/`, `contracts/`, `runtime_interface.py` |
| graph mechanism | `blueprint.py`, `wiring.py`, generic `introspection/` |
| runtime model | `profiles/`, `graph/`, endpoint and topic contracts |
| backend lookup | `registry.py`, `plugin_seed.py` |
| local Host transport and frames | `transport/local.py`, `tf/` |
| small cross-cutting utilities | `utils/`, config helpers |

## What Does Not Belong Here

| Logic | Put it in |
| --- | --- |
| global/local planning, tracking, and motion safety | `src/nav/cpp/` |
| Host navigation commands, goals, skills, and adapters | `src/nav/` |
| perception and decision reasoning | `src/perception/`, `src/decision/` |
| robot drivers and hardware behavior | `src/drivers/` |
| REST, SSE, WebSocket, MCP endpoint behavior | `src/gateway/` |
| simulator/calibration bridges and quarantined vendor trees | their owning non-Product roots |

## Current State

`src/runtime` currently contains three kinds of files:

| Group | Status |
| --- | --- |
| core runtime | should stay here |
| runtime profile and process contracts | should stay here, but keep them declarative |
| audit and evidence helpers | moved to `src/diagnostics/field/` |
| compatibility helpers | keep them outside Product runtime; delete them when no supported tool consumes them |

Non-Product support surfaces include:

- Gazebo bridge scripts: explicit compatibility adapters, never default Product
  processes
- `adapters/dds/reader.py`: Python CycloneDDS reader utility for diagnostics,
  and LiDAR/GNSS diagnostics. It is a DDS adapter.

Audit and evidence tooling now lives outside `src/runtime`, in
`src/diagnostics/field/`:

- `evidence.py`
- `gates.py`
- `gateway_acceptance.py`
- `inspection.py`
- `field_check.py`
- `dds_readiness.py`

Simulation diagnostics live in `sim/diagnostics/`, migration planning helpers in
`tools/migration/`, and benchmark-only status helpers in `tests/benchmark/`.
Keep field diagnostics focused on operator acceptance evidence.

## Quick Checks

```bash
python -m pytest src/runtime/tests/test_runtime.py src/runtime/tests/test_registry.py -q
```

For a broader runtime check, use the targeted tests that match the files you
changed instead of running unrelated hardware or ROS tests from this folder.
