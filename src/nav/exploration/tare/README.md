# TARE Exploration

This folder is the product-facing entrypoint for TARE exploration.

The normal LingTu path is in-process:

```text
OccupancyGridModule.exploration_grid
  -> PortableTAREPolicy
  -> TAREExplorerModule
  -> Navigation
```

| File | Role |
| --- | --- |
| `policy.py` | LingTu-owned portable TARE-style frontier/viewpoint selection |
| `module.py` | Publishes the next exploration goal/path from either in-process policy or DDS TARE output |
| `supervisor.py` | Tracks TARE bridge health and progress |
| `topics.py` | TARE topic/remap contract |

The CMU C++/ROS TARE project is not vendored here. External benchmark runs must
provide that runtime outside the LingTu Python package.
ROS2 bridge code for that external runtime belongs under `adapters/ros2`.
