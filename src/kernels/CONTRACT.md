# LingTu Compute Kernel Contract

This directory is the convergence point for deployable LingTu compute kernels.
The goal is not to rewrite every algorithm at once. The goal is to make every
core algorithm runnable behind a small, stable, language-neutral boundary.

## What Belongs Here

Portable compute kernels:

- Localization math, LiDAR-inertial update, ICP-style registration.
- Terrain, elevation, obstacle, traversability, and safety scoring kernels.
- Local planning, path scoring, path following, and waypoint state kernels.
- Thin adapter kernels that normalize native planner outputs.

Runtime integration does not belong here:

- ROS2 nodes, launch files, systemd services, shell deployment scripts.
- Gateway, CLI, REST, WebSocket, LCM, DDS, or hardware drivers.
- Python Module lifecycle, Blueprint wiring, logging dashboards, or UI code.

## Required Boundary

Every kernel must declare one of these boundaries:

- `c_abi`: hot-path in-process ABI for Python, Rust, Dart FFI, or C++ callers.
- `process_abi`: coarse planner/optimizer ABI over files, JSON lines, CBOR, or a
  local service process.

Do not expose C++ classes, Python objects, STL containers, exceptions, or
allocator ownership across the stable boundary.

## C ABI Rules

Use fixed-width fields only:

- `uint32_t`, `uint64_t`, `int32_t`, `float`, `double`.
- Pointers to caller-owned contiguous buffers.
- Explicit count, stride, dtype, frame, timestamp, and unit fields.

Required exported probes:

- ABI version.
- Config struct size.
- Input struct size.
- Output struct size.
- Optional field-offset probes when structs are not trivially obvious.

Required lifecycle:

- `create`
- `destroy`
- `reset`
- `configure`
- `step` or `process`
- `last_error` or diagnostic output

## Language Strategy

- C++ portable remains valid for existing heavy kernels.
- Rust is preferred for new small deterministic kernels such as path safety,
  waypoint tracking, geometry validation, and path follower pilots.
- Dart should consume kernels through FFI or process ABI. It should not own
  real-time robot compute loops.
- Python remains orchestration and fallback. Python fallback is useful for dev
  and smoke tests, but it is not the production hot path.

## Fixture Rules

Each migrated kernel needs golden fixtures before replacement:

- `manifest.json` with schema version, units, frames, tolerances, and source.
- Binary arrays as `.npy` or `.npz` when NumPy is already available in tests.
- A small JSON fixture is acceptable for scalar/state-machine kernels.
- The C++ reference, Python fallback, and any Rust candidate must run the same
  fixture and produce equivalent output within the declared tolerance.

## Current Kernel Catalog

The current catalog is defined in `src/kernels/catalog.py`.

Materialized targets:

- `path_safety`
- `pose_graph_optimizer`
- `gpmp_trajectory_optimizer`
- `camera_lidar_calibration_optimizer`

Navigation C++ kernels that already live in `src/nav/kernel/` stay there.
Do not add placeholder target directories here for speculative rewrites.
