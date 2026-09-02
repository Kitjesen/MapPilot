# Compute Kernels

`src/kernels/` is the home for portable algorithm kernels and their stable ABI
contracts.

This folder is not a runtime layer. It does not own Module lifecycle, Blueprint
wiring, Gateway routes, ROS adapters, or deployment scripts. Owning runtime
domains call kernels when they need fast or portable compute.

## Plain Meaning

```text
runtime Module = owns state, ports, lifecycle, wiring
kernel         = pure compute behind a stable C ABI or process ABI
```

## Folder Map

| Path | Role |
| --- | --- |
| `FILES.md` | Short file map for humans |
| `catalog.py` | Migration catalog: current path, target path, ABI kind, status |
| `CONTRACT.md` | ABI and migration rules |
| `nav/` | Navigation safety kernels |
| `slam/` | SLAM/localization kernel targets |
| `calibration/` | Calibration optimizer kernel targets |

## Materialized Kernels

The current materialized kernel targets are listed in `catalog.py`:

- `path_safety`
- `camera_lidar_calibration_optimizer`

The production C++/nanobind navigation kernel is already owned by the nav
domain at `src/nav/cpp/`; this folder tracks portable kernels with active
consumers.

Build the path-safety kernel directly with Cargo:

```bash
cargo build --manifest-path src/kernels/nav/path_safety/Cargo.toml --release
```

## Boundary Rule

Put code here only when it is algorithm compute that can be called through a
stable `c_abi` or `process_abi`. If the code publishes ports, handles HTTP,
starts ROS, starts subprocesses, or chooses product behavior, it belongs
elsewhere.
