# Core TF

ROS-free transform support for normal LingTu modules.

## Where It Is Used

- `src/localization/adapters/ros2/slam_bridge.py`: parses ROS/DDS-shaped `/tf` messages and feeds `map -> odom`.
- DDS localization adapters feed `map -> odom` from endpoint localization health.
- `src/localization/portable/localization_adapter.py`: installs static `map -> odom` identity for portable runs.
- `src/gateway/gateway_module.py`: caches `map -> odom` for viewer/map-cloud alignment.
- `src/gateway/routes/diagnostics.py`: exposes `FrameTree.snapshot()` in frame diagnostics.

## Math Contract

`Transform(frame_id=P, child_frame_id=C)` stores the pose of child frame `C`
in parent frame `P`. Applying it converts child-frame data into parent-frame
data:

```text
p_P = R_PC * p_C + t_PC
q_P = q_PC * q_C
```

Composition follows the same convention:

```text
T_AC = T_AB * T_BC
```

`lookup(target, source)` returns `T_target_source`.

Dynamic edges are timestamped and interpolated. Static edges are valid for any
timestamp.

`Odometry.pose` is transformed into the requested target frame. `Odometry.twist`
is preserved because it is tied to `child_frame_id` in the ROS odometry contract.

## Not Included

This is not a ROS2 drop-in package. ROS2 nodes, executors, DDS subscriptions,
`tf2_ros.BufferClient`, `MessageFilter`, and full geometry message coverage stay
outside runtime. ROS-facing code belongs under `src/*/adapters/ros2/`.
