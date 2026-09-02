# Simulation scripts

Product simulation uses native DDS processes. New acceptance work belongs in
the native MuJoCo entrypoints below, not in a second Python navigation stack.

## Contents

- `mujoco/` is the tracked, stable Product/native acceptance surface.
- `factory_demo/`, when present locally, is a generated demo workspace rather
  than a Python package or stable command surface.
- Runtime verdict construction belongs to `sim/runtime/qualification/`; do not
  recreate a parallel `scripts/qualification/` implementation.

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
- `mujoco/saved_map_relocalization.py` — native saved-map relocalization.
- `mujoco/sunrise_mapping.py` — remote continuous-mapping `run` and `sweep` entrypoints.
- `mujoco/continuous_walk.py` — Thunder V4 continuous-walk qualification.
- `mujoco/native_recording_acceptance.py` — native recording acceptance.
- `mujoco/native_navigation_video.py` — native navigation evidence video.

Offline navigation comparison lives at
`python -m sim.evaluation.navigation_replay`; it is evaluation, not a Product
acceptance launcher.

MCAP recording and replay are diagnostic tools. They do not add Products or
own Product lifecycle.

## Diagnostics and setup

- `python -m sim.diagnostics` — summarize current gate reports, or run one
  simulation Product through `ProductControl` before collecting the reports.
- `python -m sim.diagnostics.dimos_gap` — read-only gap view over the current
  native gate set.
- `python -m sim.diagnostics.gazebo_tf` — read-only Gazebo/TF contract smoke.
- `scripts/sim/setup_linux_validation_host.sh` — ROS-free Linux native
  validation setup.

## Boundary

These scripts prove simulation behavior only. They do not prove S100P field
readiness or authorize physical motion.
