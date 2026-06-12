# src/ - Module-First source map

`src/` is organized as one first-level package per runtime layer. Keep it that
way: normal Modules depend on `core/`, communicate through typed ports, and are
assembled by Blueprints instead of importing each other across layer boundaries.

## Primary entry points

| Entry | Purpose |
| --- | --- |
| `core.blueprint.autoconnect()` | Compose stack Blueprints and infer safe port wiring. |
| `core/blueprints/full_stack.py` | Production full-stack assembly plus explicit critical wires. |
| `core/blueprints/stacks/` | Small stack factories: `driver`, `lidar`, `slam`, `maps`, `perception`, `memory`, `planner`, `navigation`, `exploration`, `safety`, `gateway`. |
| `core.registry.get(...)` | Resolve pluggable backends by category and name. |

Minimal composition shape:

```python
from core.blueprint import autoconnect
from core.blueprints.stacks import *

system = autoconnect(
    driver("thunder", dog_host="192.168.66.190"),
    lidar(enabled=True),
    slam("bridge"),
    maps(),
    perception("bpu", "mobileclip"),
    memory(),
    planner("qwen"),
    navigation("pct"),
    exploration("none"),
    safety(),
    gateway(5050),
).build()
```

## Layer map

| Layer | Package | Runtime role |
| --- | --- | --- |
| L0 Safety | `nav/` | `SafetyRingModule`, geofence, `CmdVelMux`, plan safety checks. |
| L1 Hardware | `drivers/`, `slam/` | Real/sim robot drivers, camera/LiDAR/GNSS bridges, SLAM/localization. |
| L2 Maps | `nav/` | Occupancy, voxel, ESDF, elevation, traversability, map manager. |
| L3 Perception | `semantic/perception/`, `memory/` | Detection, encoding, scene graph, semantic map and memories. |
| L4 Decision | `semantic/planner/` | Goal resolution, LLM/tool loop, visual servo, semantic frontier scoring. |
| L5 Planning | `nav/`, `global_planning/`, `base_autonomy/` | Navigation FSM, A*/PCT dispatch, terrain, local planner, path follower. |
| L6 Interface | `gateway/`, `webrtc/` | REST/SSE/WS, MCP, teleop, optional WebRTC/Rerun. |

## First-level packages

| Package | What belongs here | Notes |
| --- | --- | --- |
| `core/` | Module framework, streams, Blueprint, transports, registry, messages, devices, framework tests. | Shared dependency target for all Modules. |
| `drivers/` | Replaceable robot/sensor backends. | Real Thunder code is under `drivers/real/thunder/`; sim backends stay under `drivers/sim/`. |
| `slam/` | Managed SLAM modules, bridge modules, Fast-LIO2/Point-LIO/localizer integrations. | Real `nav` uses bridge mode so robot-side services own LiDAR. |
| `nav/` | Navigation execution, map services, safety, frontier exploration, velocity arbitration. | Wavefront frontier lives here and is enabled by `navigation(enable_frontier=True)`. |
| `global_planning/` | PCT planner tree and Python adapters. | Treat `pct_planner/` as third-party/native planner surface; do not casually reshuffle. |
| `base_autonomy/` | C++/nanobind terrain, local planner, path follower hot paths. | Performance-sensitive aarch64 code. |
| `semantic/` | Semantic perception, semantic planner, reconstruction. | `perception/`, `planner/`, and `reconstruction/` are subtrees here, not top-level `src/` packages. |
| `memory/` | Semantic, episodic, tagged, vector, temporal and KG-backed memory modules. | Optional ChromaDB falls back to numpy search. |
| `exploration/` | CMU TARE exploration stack. | TARE profile only; wavefront is not selected through this factory. |
| `gateway/` | FastAPI gateway, MCP server, route/service helpers. | External interface layer. |
| `webrtc/` | Optional H.264 WebRTC camera stream module. | Requires optional `aiortc`. |
| `lingtu/` | Package-facing compatibility namespace. | Keep runtime ownership in the layer packages above. |

## Boundary rules

```text
All Modules -> core/ (Module, In/Out, Registry, utils, msgs)

nav/       must not import semantic/, drivers/, gateway/
semantic/  must not import nav/, drivers/, gateway/
drivers/   must not import nav/, semantic/ except lazy blueprint registration
gateway/   must not import nav/, semantic/, drivers/
```

Use registry lookups and Blueprint wiring for cross-layer behavior. Add explicit
wires in `core/blueprints/full_stack.py` for critical fan-in/fan-out or
ambiguous port names.

## Tests

| Location | Use |
| --- | --- |
| `core/tests/` | Framework and cross-package contract gates that do not need ROS 2. |
| `<package>/tests/` | Unit tests owned by a package such as `nav`, `gateway`, `drivers`, or `semantic`. |
| root `sim/tests/` | Simulation integration and validation gates. |

Run the narrowest relevant test first, then broaden only when the touched
surface crosses package boundaries.
