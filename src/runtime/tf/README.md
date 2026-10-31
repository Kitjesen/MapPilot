# Core TF

Frame contracts, diagnostic mirrors, and development transform utilities.

## Native ownership

Field map/body/scan poses are resolved in
`src/localization/slam/cpp/frames.hpp` from one SLAM snapshot. Native DDS map
observations and the Host status export use this same resolver. `map_odometry`
contains a map-frame body pose and the synchronized body-frame linear/angular
velocity. `map_scan_pose` contains the scan-frame pose and the physical LiDAR
ray origin, which differs from the body origin for a mounted sensor.

The Python SLAM adapter forwards those resolved values. Navigation skills use
the published map height. The browser displays map-frame poses and paths; it
does not compose separately received odometry and TF. Raw `odometry` and
`map_odom_tf` remain available as diagnostics/native inputs, not Host fallbacks
for reconstructing a missing map pose. Deploy native SLAM, Host and web together.

Native navigation retains time-indexed input buffers for algorithm inputs.
Python `FrameTree` is not their owner. Gateway's tree is a diagnostic mirror;
its cached edges must not be used to authorize or project navigation commands.

## Where It Is Used

- Native DDS localization adapters feed `map <- odom` from endpoint localization health.
- `src/gateway/gateway_module.py`: mirrors published frames for diagnostics.
- `src/gateway/routes/diagnostics.py`: exposes `FrameTree.snapshot()` in frame diagnostics.

## Math Contract

`Transform(frame_id=P, child_frame_id=C)` stores the pose of child frame `C`
in parent frame `P`. Tree topology writes this as `P -> C`; numeric transform
direction writes it as `P <- C` or `T_P_from_C`. Applying it converts
child-frame data into parent-frame data:

```text
p_P = R_PC * p_C + t_PC
q_P = q_PC * q_C
```

Composition follows the same convention:

```text
T_AC = T_AB * T_BC
```

`lookup(target, source)` returns `T_target_from_source`.

For localization, `map_odom_tf` is the compatibility field name for
`T_map_from_odom`: the pose of `odom` expressed in `map`. It is not a map and
not an odometry pose. Module ports serialize it with one `ts` source-time field.
The canonical chain is:

```text
T_map_from_body = T_map_from_odom * T_odom_from_body
```

Dynamic edges are timestamped and interpolated. Static edges are valid for any
timestamp.

`Odometry.pose` is transformed into the requested target frame. `Odometry.twist`
is preserved because it is tied to `child_frame_id` in the ROS odometry contract.

## Not Included

This is not a ROS2 drop-in package. ROS2 nodes, executors, DDS subscriptions,
`tf2_ros.BufferClient`, `MessageFilter`, and full geometry message coverage stay
outside runtime. ROS-facing code belongs under `src/*/adapters/ros2/`.
