# LingTu Explore C++ Core

This directory owns field exploration algorithms that must not depend on
Python, ROS, or the LingTu Module runtime.

## Layout

| Path | Owns |
| --- | --- |
| `bindings/` | Nanobind exposure for Python development and simulation. |
| `endpoint/` | `lingtu_explore_dds` process entry and exploration lifecycle support. |
| other sources | Transport-free exploration contracts and policies. |

## Current core

`explore_contract.hpp` defines the interface all exploration algorithms must
implement:

```cpp
class IExplorePlanner {
 public:
  virtual const char* name() const = 0;
  virtual ExploreDecision plan(const ExploreInput& input) const = 0;
};
```

`TarePolicy` is one implementation. It selects the next exploration viewpoint
from:

- `Grid2D`: `0` free, `100` occupied, `-1` unknown;
- `Pose2D`: current robot pose in map frame;
- visited goals, used to suppress repeated viewpoints.

It returns:

- `TareDecision.has_goal`;
- selected goal in map frame;
- scored candidates;
- a machine-readable reason.

The policy performs:

```text
reachable free-space flood fill
-> frontier detection next to unknown cells
-> frontier clustering
-> reachable candidate viewpoint search
-> score by frontier coverage, distance, heading momentum, novelty
```

## Python bindings (nanobind)

The C++ core is exposed to Python through the `lingtu_explore_kernel` nanobind
extension. The bindings live in `bindings/`:

- `bindings.cpp` — `NB_MODULE(lingtu_explore_kernel, m)` entrypoint.
- `bind_types.cpp` — `Grid2D`, `Pose2D`, `ExploreInput`, `ExploreCandidate`, `ExploreDecision`.
- `bind_tare.cpp` — `TarePolicyConfig` and `TarePolicy`.

Build with:

```bash
scripts/build/build_explore_py.sh
```

This produces the extension under `src/explore/cpp/build_nb` (Linux) or
`build_nb_win` (Windows). The Python loader in `src/explore/kernel/` discovers
it at runtime; `explore.explore_kernel_available()` reports availability.

## Runtime target

The robot-side runtime should use `lingtu_explore_dds`:

```text
/nav/exploration_grid + /slam/odometry
  -> C++ TarePolicy
  -> NavigationCommandClient
  -> /nav/command/request + /nav/command/ack
  -> C++ nav endpoint
  -> OctoPlanner3D / LocalPlanner / PathFollower
```

Python wrappers may remain for compatibility tests and old profiles, but field
behavior should be implemented here.

`endpoint/` is built by `src/nav/cpp/endpoint/CMakeLists.txt`; this keeps the
existing navigation linkage and public `lingtu_explore_dds` target while the
source remains owned by the exploration domain.
