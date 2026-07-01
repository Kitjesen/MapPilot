# LingTu SLAM Simulation Evaluation

This package is the LingTu-owned boundary for simulated SLAM evaluation. It is
kept separate from `src/localization/` runtime modules and from `sim/engine/` physics so
we can compare SLAM backends without changing robot behavior.

## Current Scope

- TUM trajectory read/write helpers.
- Timestamp association and translation error metrics.
- Declarative evaluation case manifests.
- A NOVA Dog + Fast-LIO2 baseline case manifest.

## External Reference Notes

`github.com/17863958533/QR_SimEval_Code` is useful as a reference, but should not
be copied directly into LingTu. It is a ROS1/catkin evaluation bundle with
Unitree/Gazebo assumptions, generated build artifacts, and hardcoded local
paths.

Useful ideas to adapt into LingTu-owned code:

- Evaluation loop: simulation or bag replay -> SLAM output -> TUM trajectory ->
  APE/RPE style metrics.
- CMAPS-LIVO-style keyframe logging: pose, point cloud, and image snapshots for
  post-run inspection.
- Loop-closure direction: Scan Context candidate retrieval followed by ICP and a
  global optimizer. This belongs behind a LingTu SLAM backend boundary, not in
  the generic simulator.

## Boundary

- `sim/evaluation/slam/`: deterministic evaluation utilities and case manifests.
- `sim/engine/`: robot/world stepping and sensor generation.
- `src/localization/`: runtime SLAM modules and bridges.
- `src/runtime/blueprints/`: profile composition and wiring.

Future replay scripts should depend on this package for manifest parsing and
metrics, then call existing LingTu runtime profiles instead of introducing a
parallel robot stack.
