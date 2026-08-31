# Simulation scripts

Product simulation uses native DDS processes. New acceptance work belongs in
the native MuJoCo entrypoints below, not in a second Python navigation stack.

## Product acceptance

- `mujoco/product_acceptance.py` — Product-level MuJoCo acceptance.
- `mujoco/native_navigation_acceptance.py` — native SLAM, traversability,
  navigation, driver command, and optional video evidence.
- `mujoco/native_control_mode_acceptance.py` — autonomy, teleop, and
  teleop-avoid control-mode acceptance.
- `mujoco/inspection_native_acceptance.py` — inspection Product acceptance.
- `mujoco/explore_native_acceptance.py` — explore Product acceptance.
- `mujoco/map_native_acceptance.py` — native saved-map acceptance.
- `mujoco/teleop_native_acceptance.py` — native teleop acceptance.
- `mujoco/teleop_avoid_native_acceptance.py` — native assisted teleop acceptance.

## Sensors, mapping, and evidence

- `mujoco/native_dds_sensors.py` — MuJoCo sensor feed over typed DDS.
- `mujoco/continuous_mapping_quality_gate.py` — continuous native mapping check;
  consumes a map already saved through SDK/Gateway -> mapd and never invokes
  `slamctl save-map` directly.
- `saved_map_relocalize_runtime_gate.py` — native saved-map relocalization.
- `navigation_replay_deviation_gate.py` — offline navigation trace comparison.
- `mujoco/native_recording_acceptance.py` — native recording acceptance.
- `mujoco/native_navigation_video.py` — native navigation evidence video.

MCAP recording and replay are diagnostic tools. They do not add Products or
own Product lifecycle.

## Diagnostics and setup

- `sim_diagnostics.py` — summarize current gate reports, or run one simulation
  Product through `ProductControl` before collecting the reports.
- `dimos_gap_report.py` — read-only gap view over the current native gate set.
- `setup_linux_validation_host.sh` — ROS-free Linux native validation setup.

These scripts prove simulation behavior only. They do not prove S100P field
readiness or authorize physical motion.
