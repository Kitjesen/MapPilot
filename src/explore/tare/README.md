# TARE Exploration

This folder is the product-facing entrypoint for TARE exploration.

The normal LingTu path is in-process, with the algorithm living **only in C++**
(`src/explore/cpp/tare_policy.cpp`) and exposed through the
`lingtu_explore_kernel` nanobind extension:

```text
OccupancyGridModule.exploration_grid
  -> TAREExplorerModule (ExploreModule, layer=5)
    -> PortableTAREPolicy.select(...)
      -> NanobindExploreBackend.plan(...)
        -> C++ TarePolicy
    -> Navigation (exploration_goal / exploration_path / tare_stats)
```

| File | Role |
| --- | --- |
| `backend.py` | `create_nanobind_explore_backend` — wraps the native `TarePolicy` for Python callers. |
| `policy.py` | Thin Python data contract (`TAREPolicyConfig`/`TAREDecision`) + `PortableTAREPolicy` delegating to C++. |
| `module.py` | `TAREExplorerModule(ExploreModule)` — standardized exploration module. |
| `supervisor.py` | Tracks TARE bridge health and progress. |
| `topics.py` | TARE topic/remap contract. |

The kernel is optional at runtime: `explore.explore_kernel_available()` reports
whether the extension is importable. When it is not, the module stays in
``configured`` state and tests/DDS paths remain usable. Build it with
`scripts/build/build_explore_py.sh`.

The CMU C++/ROS TARE project is not vendored here. External benchmark runs must
provide that runtime outside the LingTu Python package.
ROS2 bridge code for that external runtime belongs under `adapters/ros2`.
