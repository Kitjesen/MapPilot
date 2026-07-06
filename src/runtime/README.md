# LingTu Core

`src/runtime/` is the runtime foundation of LingTu.

Keep this folder small in meaning: it explains how modules exist, how modules
talk, and how a product stack is assembled. It should not contain navigation
decisions, perception algorithms, robot behavior, HTTP endpoint logic, or ROS
node logic.

## Read This First

| If you need to... | Start here |
| --- | --- |
| write or inspect a module | `module.py`, `stream.py` |
| connect modules together | `blueprint.py`, `blueprints/wires/` |
| understand product data flow | `blueprints/products/thunder.py`, then `blueprints/full_stack_wiring.py` |
| change profile defaults | `profiles/catalog/` |
| find shared messages | `msgs/`, `contracts/` |
| find topic, frame, or runtime contract names | `runtime_interface.py` |
| register or resolve a backend | `registry.py` |
| inspect runtime diagnostics and acceptance evidence | `diagnostics/` |
| understand every file in this folder | `FILES.md` |

## Mental Model

```text
Module      = one runtime unit
In / Out    = typed input and output ports
WireSpec    = one declared data-flow connection
Blueprint   = a module graph before it starts
SystemHandle = the running graph
```

Normal Module flow is:

```text
goal / sensor / map input
  -> Module ports
  -> explicit wires
  -> planning, tracking, safety, or gateway modules
  -> robot command or user-facing status
```

Example Module-owned navigation path, used by simulation, local-driver, and
compatibility profiles:

```text
GatewayModule.goal_pose
  -> nav.mission.goal_pose
  -> nav.mission.global_path
  -> nav.local_planner.global_path
  -> nav.path_follower.local_path
  -> VelocityMuxModule
  -> Driver
```

The default physical `thunder_field` navigation path is not this Python
autonomy chain. Field navigation is owned by native DDS services:

```text
lingtu-livox-dds
  -> lingtu-slam-dds
  -> lingtu-traversability-dds
  -> lingtu-nav-dds
  -> /nav/cmd_vel
```

In that mode `src/runtime` still assembles Gateway, mission/status, map, and
business modules, but it must not load Python `nav.terrain`,
`nav.local_planner`, or `nav.path_follower` as the product control loop.

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
| graph assembly | `blueprints/`, `profiles/catalog/`, `introspection/` |
| backend lookup | `registry.py`, `plugin_seed.py` |
| shared transports and frames | `transport/`, `portable/`, `tf/` |
| small cross-cutting utilities | `utils/`, config helpers, device registry |

## What Does Not Belong Here

| Logic | Put it in |
| --- | --- |
| global or local planning algorithms | `src/nav/services/plan/`, `src/nav/local/`, `src/nav/kernel/` |
| navigation task policy and recovery | `src/nav/` |
| perception and decision reasoning | `src/perception/`, `src/decision/` |
| robot drivers and hardware behavior | `src/drivers/` |
| REST, SSE, WebSocket, MCP endpoint behavior | `src/gateway/` |
| ROS 2 adapters or compatibility code | `src/*/adapters/ros2/` |

## Current State

`src/runtime` currently contains three kinds of files:

| Group | Status |
| --- | --- |
| core runtime | should stay here |
| product assembly and runtime catalog | should stay here, but keep it declarative |
| audit and evidence helpers | `diagnostics/`; useful today, but not part of the clean kernel |
| legacy compatibility helpers | isolated files or adapter packages; do not add product behavior there |

Known compatibility candidates include:

- `native_module.py`, `native_install.py`: legacy native/ROS process helpers
- `adapters/ros2/`: ROS 2 compatibility boundary only
- `adapters/lcm/`, `transport/lcm.py`: replay/dev compatibility, not field
  default
- `dds.py`, `adapters/dds/*`: Python DDS utilities that must not become the
  Thunder field control loop while the native C++ DDS endpoints own that path

Audit and evidence tooling now lives in `diagnostics/`:

- `diagnostics/dimos_*.py`
- `diagnostics/runtime_evidence.py`
- `diagnostics/runtime_validation_gates.py`
- `diagnostics/gateway_runtime_acceptance.py`
- `diagnostics/inspection_acceptance.py`
- `diagnostics/product_field_check.py`
- `diagnostics/migration_catalog.py`
- `diagnostics/efficiency_status.py`

Do not delete these just because they look old. Some are still used by tests,
deployment checks, or status commands. Move them only after replacing imports
and validating the affected entry points.

## Quick Checks

```bash
python -m pytest src/runtime/tests/test_runtime.py src/runtime/tests/test_registry.py -q
```

For a broader runtime check, use the targeted tests that match the files you
changed instead of running unrelated hardware or ROS tests from this folder.
