# Global Planning (`src/global_planning/`)

Global path planning module for LingTu. Provides long-horizon path computation from a start pose to a goal pose over pre-built maps, with C++-accelerated and pure-Python backends.

## Directory Structure

```
global_planning/
├── __init__.py                  # Package init
├── pct_planner/                 # C++ PCT planner (ele_planner.so, ROS2 package)
│   ├── CMakeLists.txt           # ROS2 ament build
│   ├── package.xml              # ROS2 package manifest
│   ├── planner/scripts/         # Python wrappers that call the C++ binary
│   ├── launch/                  # ROS2 launch files
│   ├── tests/                   # Planner unit tests
│   └── tomography/              # Tomographic map utilities
├── pct_adapters/                # Registry-compatible Python wrappers
│   ├── __init__.py
│   ├── global_planner_module.py # GlobalPlannerModule — Registry-backed planner
│   ├── pct_path_adapter.cpp     # C++ nanobind adapter (PCT path conversion)
│   ├── CMakeLists.txt           # ROS2 ament build for path adapter
│   ├── config/                  # Planner configuration presets
│   └── tests/                   # Adapter tests
└── pct_planner_runnable/        # Standalone runnable wrappers (no ROS2)
    ├── __init__.py
    ├── runtime.py               # Runtime — orchestrates C++ planner in subprocess
    └── run_pct_preview.py       # Preview mode (visualize planned path)
```

## Backends

Planner backends are registered via `@register("planner_backend", name)` in `core.registry.py` and resolved in `global_planner_module.py` with zero if/else.

### AStarBackend (pure Python)

Fallback global planner. Implements 2D A* on the occupancy grid. No C++ dependencies — runs on any platform.

### PCTBackend (C++, primary)

Production global planner using the PCT (Path-Costmap-Time) algorithm. Requires the C++ `ele_planner.so` binary compiled for the target platform.

- Source: `src/global_planning/pct_planner/`
- Binary: `ele_planner.so` (built via ROS2 colcon or standalone cmake)
- Python wrapper: `src/global_planning/pct_planner/planner/scripts/planner_wrapper.py`
- Package: ROS2 `pct_planner`

The C++ planner uses nanobind for the `pct_path_adapter.cpp` bridge, converting between C++ internal path representations and Python dataclasses.

## How Backend Selection Works

In `global_planner_module.py`:
```python
from core.registry import get

class GlobalPlannerModule(Module):
    def setup(self):
        backend_name = self.config.get("planner", "astar")  # from profile
        backend_cls = get("planner_backend", backend_name)
        self._backend = backend_cls(self.config)
```

## How to Add a New Planner Backend

1. Create a new class that implements the planner interface:

```python
# src/global_planning/pct_adapters/my_planner.py
from core.registry import register

@register("planner_backend", "my_planner")
class MyPlannerBackend:
    def __init__(self, config: dict):
        self.config = config

    def plan(self, start: tuple, goal: tuple, costmap) -> list:
        """Return list of (x, y) waypoints from start to goal."""
        ...
```

2. Set the planner profile to use it:

```bash
python lingtu.py nav --planner my_planner
```

Or update `cli/profiles_data.py`:
```python
Profile("my_profile", ..., planner="my_planner")
```

## Key Files

| File | Purpose |
|------|---------|
| `pct_adapters/global_planner_module.py` | Module wrapper — backend resolution, path validation, I/O ports |
| `pct_adapters/pct_path_adapter.cpp` | C++ nanobind conversion between internal path formats |
| `pct_planner/planner/scripts/planner_wrapper.py` | Python subprocess wrapper for the C++ binary |
| `pct_planner_runnable/runtime.py` | Direct subprocess runner (no ROS2 overhead) |
| `pct_planner_runnable/run_pct_preview.py` | Preview/visualization mode |
| `pct_planner/launch/system_launch.py` | ROS2 launch integration |

## Tests

```bash
python -m pytest src/global_planning/tests/ -q
python -m pytest src/global_planning/pct_adapters/tests/ -q
```
