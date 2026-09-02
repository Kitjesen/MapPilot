# Simulation Validation

Status: current simulation-gate index as of 2026-09-03.

These documents define simulator acceptance and fidelity boundaries. A passing
simulation gate proves only the named simulator, platform, Product/component
scope, and scenario.

## Gate Index

| Gate | Purpose |
| --- | --- |
| [Native control-mode acceptance](MUJOCO_NATIVE_CONTROL_MODE_ACCEPTANCE.md) | Defines `autonomy`, `teleop`, and `teleop_avoid` promotion gates. |
| [Navigation acceptance](MUJOCO_NAVIGATION_ACCEPTANCE.md) | Defines native-DDS navigation and long-range scenario evidence. |
| [MID-360 fidelity contract](MUJOCO_MID360_FIDELITY.md) | Defines scan-pattern, timing, and sensor-fidelity requirements. |
| [MuJoCo scene design](mujoco_scene_design_guidelines.md) | Defines reproducible room, terrain, stair, and dynamic-obstacle scenes. |
| [ThunderV4 MID-360 recording](thunderv4_mujoco_lidar_recording_requirements.md) | Defines ThunderV4 policy, sensor, video, and native SLAM recording evidence. |

## Product Versus Component Evidence

Direct component runners can prove a native subchain, but they do not prove the
complete ProductControl transaction. A Product pass additionally requires the
exact Product/RunPlan identity, readiness/current commit, stop barrier,
rollback, cleanup, and selected driver acknowledgement required by that Product.

Platform promotion is independent. A Windows run does not prove Linux/aarch64,
and a Linux/WSL run does not prove the Windows native artifact set.

## Evidence Destination

Date-prefixed simulator results remain under
[`../field-runs/`](../field-runs/README.md) for historical continuity. Every
record must state `env=sim` or an equivalent explicit simulation boundary and
must not be presented as field or physical-motion evidence.
