# sim/bridge/ Thin Entrypoints

This directory keeps legacy import paths for old simulation bridge callers.
Do not add new bridge implementations here.

## Canonical Location

ROS-backed bridge implementations now live under `src/compat/ros2/`:

| Legacy import | Canonical implementation |
| --- | --- |
| `sim.bridge.mujoco_ros2_bridge` | `compat.ros2.mujoco_ros2_bridge` |
| `sim.bridge.mujoco_viz_bridge` | `compat.ros2.mujoco_viz_bridge` |
| `sim.bridge.nova_nav_bridge` | `compat.ros2.nova_nav_bridge` |

The `sim.bridge` package redirects imports lazily so heavy dependencies such as
MuJoCo and ROS 2 are loaded only when a specific bridge is imported.

See `src/drivers/sim/README_bridge.md` for legacy command compatibility notes.
