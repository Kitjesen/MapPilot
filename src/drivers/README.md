# Drivers

`src/drivers/` owns robot and sensor ingress/egress. It is the hardware and
simulation boundary, not a planning layer.

## Layout

| Folder | Role |
| --- | --- |
| `real/` | Real robot and sensor backends, including Thunder, camera, GNSS, and Livox sources. |
| `sim/` | Simulation drivers and endpoint-backed simulation adapters. |

Root files stay minimal. Runtime driver code lives under `real/` or `sim/`;
driver tests live under `tests/drivers/`.

## Contract

Drivers expose typed runtime ports such as `cmd_vel`, `stop_signal`,
`odometry`, `camera_image`, `depth_image`, `camera_info`, and `map_cloud`.

Upper layers select drivers through `runtime.registry` and Blueprint factories.
They should not import concrete driver classes for behavior.

## Boundary

- `drivers/` does not decide goals, plans, safety policy, or semantic actions.
- ROS 2/native device code stays behind explicit bridge modules or real-device
  backends.
- Simulation and real backends should keep the same port shape where practical.
