# Acceptance configuration

This directory contains local acceptance orchestration configuration. It is not
part of the Product runtime graph and does not define field evidence.

[`official_global_planner.json`](official_global_planner.json) fixes the source
and cases for the [offline official-PCD regression](../../tools/bench/README.md).

[`elevator_native.yaml`](elevator_native.yaml) is the public Elevator-LIO
sensor and upstream-parameter control profile for `ltlio_replay`. It runs the
same sole `ltlio_core` as the runtime and has no estimator selector. It uses
dataset MID360 calibration, not a field robot's extrinsics. Additional native
stationary constraints are disabled for the upstream numerical control; a
transport-off control changes only `elevator_mode` to `off`. The former
`elevator_ltlio.yaml` belongs to the retired 24-state implementation and is
archived in `build/ltlio-unified-20261017/retired-elevator-ltlio.yaml`, not offered
as a second supported core. See the
[LTLIO replay instructions](../../src/localization/ltlio/README.md#verification-and-public-replay).

MuJoCo manifests live in [`mujoco/`](mujoco/). Their filenames omit
`mujoco`, `native`, and `acceptance` because those meanings are already expressed
by this directory. Each manifest continues to be run by its existing consumer;
the manifest does not introduce a second command or lifecycle entry point.

Select the manifest for the implementation being measured:

| Manifest | Selected planner and scope |
| --- | --- |
| `mujoco/local_scan.json` | SCAN component with truth localization; excludes SLAM and full Product lifecycle |
| `mujoco/local_scan_lidar.json` | SCAN component with truth localization and LiDAR-only live cloud; use a saved ray map of the same scene to avoid the inherited synthetic-map fixture |
| `mujoco/local_cmu.json` | CMU component comparison using the same local fixture |
| `mujoco/industrial_park_60m.json` | CMU, native LTLIO, and the 59.94 m navigation scenario |
| `mujoco/scan_goal.json` | Native goal test fixture; no `product_contract` for ProductControl acceptance |

A direct native runner measures a component. Product acceptance additionally
uses the published RunPlan and `sim.scripts.mujoco.product_acceptance`; the
manifest must agree with that RunPlan. See [simulation commands](../../sim/README.md)
and [validation levels](../../docs/testing.md).
