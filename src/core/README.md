# LingTu Core Framework

`src/core/` provides the foundational runtime framework for the LingTu autonomous navigation system. It is a **module-first** framework where `Module` is the only runtime unit and `Blueprint` is the only orchestration mechanism.

## Key Classes

### Module

Base class for all runtime units. Modules declare typed input/output ports via class annotations, and follow a defined lifecycle.

```python
from core.module import Module
from core.stream import In, Out

class PerceptionModule(Module):
    image: In[Image]
    scene_graph: Out[SceneGraph]
    detections: Out[Detection3D]

    def setup(self):
        self.image.set_policy("latest")  # drop if busy
        self.image.subscribe(self._on_image)

    def _on_image(self, img: Image):
        sg = self._build_scene_graph(img)
        self.scene_graph.publish(sg)
```

**Lifecycle**: `__init__` -> `setup()` -> `start()` -> [running] -> `stop()`

Ports are discovered from type hints at init time — no boilerplate registration required.

### In[T] / Out[T]

Typed data-flow ports defined in `core.stream.py`. Supports five backpressure policies:

| Policy | Behavior |
|--------|----------|
| `"latest"` | Drop old, keep newest |
| `"throttle"` | Max frequency (e.g. 50 Hz) |
| `"sample"` | Every Nth message |
| `"buffer"` | Ring buffer of N items |
| `"block"` | Blocking queue |

Out ports publish via `.publish(value)`, In ports receive via `.subscribe(callback)`.

### Blueprint

Orchestration engine defined in `core.blueprint.py`. Composes modules, wires ports across layer boundaries, and manages the system lifecycle.

```python
from core.blueprint import autoconnect

system = autoconnect(
    driver("thunder"),
    slam("localizer"),
    maps(),
    perception("bpu"),
    navigation("astar"),
    safety(),
    gateway(5050),
).build()

system.start()
```

Wires can specify explicit transport per connection:
```python
bp.wire("SlamModule", "cloud", "TerrainModule", "cloud", transport="shm")
bp.wire("Perception", "scene_graph", "Planner", "scene_graph", transport="dds")
```

### @register decorator

Plugin registration decorator from `core.registry.py`. Backends self-register by category + name, enabling zero-if/else backend selection.

```python
@register("driver", "nova_dog", platforms={"aarch64"})
class NovaDogModule(Module, layer=1):
    ...
```

Categories include: `driver`, `slam_backend`, `detector`, `encoder`, `llm`, `planner_backend`, `path_follower_backend`.

### @skill decorator (Module method)

Marks a Module method as an MCP-exposed skill. Auto-discovered by the gateway's MCP server.

```python
class NavigationModule(Module):
    @skill
    def navigate_to(self, x: float, y: float, yaw: float) -> dict:
        """Navigate to a goal position."""
        ...
```

### @rpc decorator (Module method)

Marks a method as callable across module boundaries via internal RPC.

```python
class MyModule(Module):
    @rpc
    def get_status(self) -> dict:
        return {"state": self.state}
```

## Transport Layer (`core/transport/`)

Pluggable inter-module communication backends:

| Module | File | Purpose |
|--------|------|---------|
| `local` | `local.py` | In-process zero-copy bus (default, testing) |
| `dds` | `dds.py` | CycloneDDS transport for cross-process ROS2 interop |
| `shm` | `shm.py` | Shared memory transport for high-bandwidth data (point clouds) |
| `dual` | `dual.py` | Dual-channel transport (local + DDS mirror) |
| `abc` | `abc.py` | Abstract base class for transports |
| `adapter` | `adapter.py` | Transport adapter / bridge helpers |
| `factory` | `factory.py` | Transport factory — create by name |

## Sub-packages

| Directory | Role |
|-----------|------|
| `blueprints/` | Blueprint builders: `full_stack.py` (top-level system assembly), `stacks/` (14 composable factory functions for driver/slam/maps/perception/memory/planner/navigation/safety/gateway/exploration/lidar), `multi_robot.py`, `stub.py` |
| `transport/` | Pluggable transport backends (local, dds, shm, dual, factory, adapter) |
| `msgs/` | Typed message definitions: `geometry.py`, `nav.py`, `semantic.py`, `sensor.py`, `gnss.py`, `robot.py`, `scene.py` |
| `devices/` | Hardware device abstraction: base classes, decoder spec, manager pattern for live device discovery |
| `utils/` | Cross-layer utilities: `calibration_check.py`, `sanitize.py`, `validation.py`, `robustness.py`, `binary_codec.py`, `blackbox_recorder.py`, `scene_mode_detector.py`, `livox_config.py`, `redact.py` |
| `contracts/` | Contract message definitions for cross-module communication |
| `introspection/` | System introspection: DOT graph export (`dot.py`), text visualization (`text.py`) |
| `resource_monitor/` | Background resource usage monitor (`monitor.py`) |
| `tests/` | 1226+ framework tests (no ROS2 needed) |

## Other Key Files

| File | Purpose |
|------|---------|
| `module.py` | Module base class (lifecycle, port scanning, @rpc, @skill) |
| `blueprint.py` | Blueprint builder with `autoconnect()`, per-wire transport |
| `stream.py` | In[T]/Out[T] ports with 5 backpressure policies |
| `registry.py` | Plugin registry — `@register()`, `get()`, `auto_select()` |
| `config.py` | Configuration management with YAML merging |
| `config_loader.py` | Profile-based configuration loading |
| `clock.py` | Module clock synchronization |
| `dds.py` | CycloneDDS integration |
| `native_module.py` | C++ native module bridge (nanobind) |
| `native_factories.py` | Factory for native module backends |
| `native_install.py` | C++ native module install/build helpers |
| `coordinator.py` | Multi-module coordination |
| `worker.py` / `worker_manager.py` | Background worker pool |
| `service_manager.py` | External service lifecycle management (systemd wrapper) |
| `rpc_client.py` | Internal RPC client |
| `rerun_module.py` | Rerun.io visualization bridge |
| `plugin_seed.py` | Plugin discovery and seeding |
| `ros2_context.py` | ROS2 context wrapper |
| `runtime_interface.py` | Runtime introspection interface |
| `runtime_evidence.py` | Runtime evidence recording |
| `runtime_validation_gates.py` | Pre/post condition validation |
| `runtime_policy.py` | Runtime policy enforcement |
| `runtime_switch.py` | Runtime mode switching |
| `same_source_map_artifacts.py` | Map artifact diff/merge utilities |
| `swap_manager.py` | Multi-robot profile swap management |
| `backend_status.py` / `efficiency_status.py` | Status reporting |
| `algorithm_gates.py` | Algorithm pre/post condition checks |
| `gateway_runtime_acceptance.py` | Gateway integration acceptance tests |
| `inspection_acceptance.py` | Inspection-driven acceptance framework |
| `product_field_check.py` | Field deployment sanity checks |
| `dynamic_filter.py` | Dynamic obstacle filtering utilities |
| `yaml_helpers.py` | YAML load/dump helpers |
| `remote_ports.py` | Remote port binding helpers |

## Dependency Rules

```
All Modules  -->  core/ (Module, In/Out, Registry, utils, msgs)
                   ^ only legal dependency direction

nav/          does NOT import semantic/, drivers/, gateway/
semantic/     does NOT import nav/, drivers/, gateway/
drivers/      does NOT import nav/, semantic/
gateway/      does NOT import nav/, semantic/, drivers/
```

Planner backends are resolved via `core.registry.get()`, not direct imports.

## Creating a Minimal Module

```python
from core.module import Module
from core.stream import In, Out

class MyModule(Module):
    input_data: In[float]
    output_data: Out[float]

    def setup(self):
        self.input_data.subscribe(self._on_input)

    def _on_input(self, value: float):
        result = value * 2.0
        self.output_data.publish(result)
```

Register it as a backend:
```python
from core.registry import register

@register("my_category", "my_backend")
class MyBackendModule(MyModule):
    ...
```

Wire it in a blueprint:
```python
from core.blueprint import autoconnect
system = autoconnect(MyModule()).build()
system.start()
```

## Running Framework Tests

```bash
python -m pytest src/core/tests/ -q    # 1226+ tests, no ROS2 needed
```
