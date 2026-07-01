# Simulation Bridge Boundary

`sim/bridge/` contains legacy top-level bridge entrypoints that connect a
physics runtime to a navigation runtime.

## Files

| File | Role |
| --- | --- |
| `mujoco_ros2_bridge.py` | Compatibility entrypoint for `compat.ros2.mujoco_ros2_bridge`. |
| `nova_nav_bridge.py` | Compatibility entrypoint for `compat.ros2.nova_nav_bridge`. |
| `mujoco_viz_bridge.py` | Compatibility entrypoint for `compat.ros2.mujoco_viz_bridge`. |

## Contract

- Keep these paths stable while existing scripts still launch them directly.
- ROS-backed bridge logic belongs in `compat.ros2`; reusable non-ROS simulation
  logic belongs in `sim/engine/` or module adapters.
- Simulation bridge commands must not start robot services or connect to a
  physical driver.
- Robot model defaults must use `sim/robots/` or `sim/assets/`; do not re-add
  deleted `sim/robot/` paths.
