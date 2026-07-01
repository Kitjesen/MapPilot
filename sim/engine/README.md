# sim/engine — Simulation Platform Core

`sim/engine` is the canonical simulation runtime runtime. Code here must stay
hardware-free: it can create synthetic worlds, MuJoCo engines, bridges, and
scenario assets, but it must not start robot services, publish to real robot
topics, or depend on field hardware being present.

## Package Structure

| Path | Responsibility |
| --- | --- |
| `core/` | Engine abstractions: `SimEngine` (ABC), `SimWorld`, `WorldConfig`, `RobotConfig`, sensor configs (`CameraConfig`, `LidarConfig`, `IMUConfig`), and data types (`RobotState`, `CameraData`, `VelocityCommand`). |
| `mujoco/` | MuJoCo engine implementation: `MuJoCoEngine` (physics stepping, ONNX policy → joint control), `MuJoCoCamera`, `MuJoCoLidar`, `PolicyRunner`. |
| `bridge/` | Simulation ↔ ROS2 bridges: `SimROS2Bridge` (unified topic publisher), `GazeboBridgeConfig` (Gazebo topic mapping), `GazeboRuntimeAdapter` (Gazebo→LingTu topic normalization), `CmuUnityLingtuAdapter` (CMU TARE→LingTu relay). |
| `scenarios/` | Test scenario builders: `NavigationScenario` (A-to-B), `SemanticNavScenario` (natural language instruction), plus asset generators for large-terrain, multi-floor, and corridor maps. |
| `worlds/` | `WorldRegistry` — scene XML lookup, alias registration, empty-world fallback. Resolves paths under `sim/worlds/`. |
| `cli.py` | CLI entry point (`python -m sim.engine.cli`) — engine/world/scenario selection. |

## Usage

### Import as package

```python
import sys; sys.path.insert(0, "src")

from sim.engine.core import SimEngine, WorldConfig, RobotConfig
from sim.engine.mujoco import MuJoCoEngine
from sim.engine.bridge import SimROS2Bridge
from sim.engine.scenarios import NavigationScenario
from sim.engine.worlds import WorldRegistry
```

### CLI

```bash
python -m sim.engine.cli                           # default MuJoCo + factory
python -m sim.engine.cli --world office            # switch scene
python -m sim.engine.cli --headless                # no GUI
python -m sim.engine.cli --scenario navigation     # A-to-B nav test
python -m sim.engine.cli --scenario semantic_nav   # semantic nav test
```

### Via LingTu profiles

```bash
python lingtu.py sim                  # MuJoCo full simulation
python lingtu.py sim_gazebo           # Gazebo ROS-native simulation
python lingtu.py sim_cmu_tare         # CMU Unity + external TARE
```

## Scenario Architecture

Scenarios inherit from `Scenario` (ABC) and implement three methods:

```python
class Scenario(ABC):
    name: str           # unique identifier
    max_time: float     # timeout (seconds)

    def setup(self, engine) -> None:       # place robot, set goal, spawn obstacles
    def is_complete(self, engine) -> bool: # check termination condition
    def is_success(self, engine) -> bool:  # check pass/fail after completion
```

### Built-in scenarios

| Scenario | Class | Success Condition |
|----------|-------|-------------------|
| `navigation` | `NavigationScenario` | Robot enters `goal_radius` around target (x, y) |
| `semantic_nav` | `SemanticNavScenario` | Robot reaches target object matching natural language instruction |

### Scenario assets

Asset generators under `scenarios/` produce MJCF scene fragments for
validation tests:

| File | Purpose |
|------|---------|
| `large_terrain_assets.py` | Large outdoor terrain with elevation variation |
| `multifloor_assets.py` | Multi-floor building with stairs/ramps |
| `nav_corridor_assets.py` | Corridor with obstacles for path planning tests |

### CLI flow

```
cli.py::main()
  → _build_engine(engine_name, world_name, headless)   # MuJoCoEngine + WorldRegistry
  → _build_scenario(scenario_name, scenario_args)       # Scenario subclass
  → scenario.setup(engine)
  → loop: engine.step() → scenario.is_complete() → timeout check
  → _print_scenario_result(scenario, engine)
```

## Bridge Architecture

Bridges convert simulation engine output into ROS2 messages so the navigation
stack runs identically in sim and on real hardware:

| Bridge | Backend | Topics |
|--------|---------|--------|
| `SimROS2Bridge` | MuJoCo direct | `/slam/odometry`, `/slam/registered_cloud`, `/camera/*`, TF |
| `GazeboBridgeConfig` | Gazebo/GZ | Topic name mapping for `ros_gz_bridge` |
| `GazeboRuntimeAdapter` | Gazebo ROS | `/lingtu/gazebo/raw/*` → `/nav/*` normalization |
| `CmuUnityLingtuAdapter` | CMU Unity | External TARE topics → LingTu `/nav/*` + `/exploration/*` |
| `GazeboCmdVelAdapter` | Gazebo | `TwistStamped` → `Twist` conversion for Gazebo diff-drive |

## Entrypoints

Use `python lingtu.py sim`, profile launchers, or the stable scripts under
`sim/scripts/`. Do not add new top-level simulation entrypoints here unless the
profile graph and tests need a new stable contract.

## Evidence Boundary

Generated reports belong under `artifacts/`, not under `sim/engine`. G4 server
closure evidence must remain simulation-only and should preserve:

- `simulation_only=true`
- `real_robot_motion=false`
- `cmd_vel_sent_to_hardware=false`

