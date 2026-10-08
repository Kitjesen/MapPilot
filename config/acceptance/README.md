# Acceptance configuration

This directory contains local acceptance orchestration configuration. It is not
part of the Product runtime graph and does not define field evidence.

MuJoCo manifests live in [`mujoco/`](mujoco/). Their filenames omit
`mujoco`, `native`, and `acceptance` because those meanings are already expressed
by this directory. Each manifest continues to be run by its existing consumer;
the manifest does not introduce a second command or lifecycle entry point.

Select the manifest for the implementation being measured:

| Manifest | Selected planner and scope |
| --- | --- |
| `mujoco/local_scan.json` | SCAN component with truth localization; excludes SLAM and full Product lifecycle |
| `mujoco/local_cmu.json` | CMU component comparison using the same local fixture |
| `mujoco/industrial_park_60m.json` | CMU, native Fast-LIO2, and the 59.94 m navigation scenario |

A direct native runner measures a component. Product acceptance additionally
uses the published RunPlan and `sim.scripts.mujoco.product_acceptance`; the
manifest must agree with that RunPlan. See [simulation commands](../../sim/README.md)
and [validation levels](../../docs/testing.md).
