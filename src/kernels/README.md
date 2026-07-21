# Compute Kernels

`src/kernels/` is the home for portable algorithm kernels and their stable ABI
contracts.

This folder is not a runtime layer. It does not own Module lifecycle, Blueprint
wiring, Gateway routes, ROS adapters, or deployment scripts. Runtime modules in
`nav/`, `nav/local/`, `slam/`, or `nav/services/plan/` call kernels when they
need fast or portable compute.

## Plain Meaning

```text
runtime Module = owns state, ports, lifecycle, wiring
kernel         = pure compute behind a stable C ABI or process ABI
```

Example:

```text
PCT runtime
  -> GPMP optimizer adapter
  -> kernels/planning/gpmp_trajectory_optimizer
  -> optimized trajectory
```

## Folder Map

| Path | Role |
| --- | --- |
| `FILES.md` | Short file map for humans |
| `catalog.py` | Migration catalog: current path, target path, ABI kind, status |
| `CONTRACT.md` | ABI and migration rules |
| `nav/` | Navigation-related kernels such as local planning and path tracking |
| `planning/` | Planner adapter or optimizer kernel targets |
| `slam/` | SLAM/localization kernel targets |
| `calibration/` | Calibration optimizer kernel targets |

## Materialized Kernels

The current materialized kernel targets are listed in `catalog.py`:

- `path_safety`
- `pose_graph_optimizer`
- `gpmp_trajectory_optimizer`
- `camera_lidar_calibration_optimizer`

The production C++/nanobind navigation kernel is already owned by the nav
domain at `src/nav/cpp/`; `src/nav/kernel/` remains the Python loader, while this folder tracks portable kernel extraction
targets and Rust/process ABI experiments that have real source directories.

## Boundary Rule

Put code here only when it is algorithm compute that can be called through a
stable `c_abi` or `process_abi`. If the code publishes ports, handles HTTP,
starts ROS, starts subprocesses, or chooses product behavior, it belongs
elsewhere.
