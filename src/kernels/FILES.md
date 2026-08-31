# `src/kernels` File Map

Use this folder only for portable compute kernels and ABI contracts.

## Start Here

| Path | Role |
| --- | --- |
| `README.md` | Human overview: what kernels are and are not |
| `CONTRACT.md` | ABI rules for C/process boundaries |
| `catalog.py` | Ordered migration catalog for kernel targets |

## Materialized Rust Kernels

| Path | Role |
| --- | --- |
| `nav/path_safety/` | 2D path safety C ABI kernel |
| `calibration/camera_lidar_optimizer/` | Camera-LiDAR calibration optimizer kernel |

The production navigation kernel lives with its owning domain at `src/nav/cpp/`.

## Do Not Commit

These are generated and should stay out of source review:

- `target/`
- `__pycache__/`
- compiled native outputs such as `*.pyd`, `*.so`, `*.dll`
