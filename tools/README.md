# tools/ - developer utilities

`tools/` only holds developer-side utilities: validation, benchmarks, model
conversion, offline reconstruction, and one-shot packaging.

It is not a robot runtime entrypoint and not a systemd service directory.
Long-running commands, build/deploy/OTA flows, and field operations belong
under `scripts/`; runtime code belongs under `src/`.

## Layout

| Path | Purpose |
| --- | --- |
| `bench/` | Native kernel, PCT/GPMP, pose graph, and camera-LiDAR parity/benchmark scripts. |
| `calibration/` | Offline sensor calibration, result application, and configuration verification tools. |
| `perception/` | BPU model export and ONNX-to-HBM conversion. |
| `reconstruction/` | Offline 3D reconstruction and recorded dataset replay helpers. |
| `validate/` | Static contract validators for config, topics, architecture, packages, and migration gates. |
| `package_lite.py` | `lite` Profile package builder; runs its validator first. |

Generated code, caches, and long-running robot services do not belong here.
Protobuf generation belongs under `scripts/proto/`; Python bytecode caches stay
untracked.

## Common Commands

```bash
# robot_config.yaml structure validation
python tools/validate/validate_config.py

# Offline sensor calibration result application and verification
python tools/calibration/apply_calibration.py --help
python tools/calibration/verify.py

# Topic contract validation
python tools/validate/validate_topics.py

# Product, Host, and package boundary validation
python tools/validate/validate_architecture_boundaries.py

# `lite` Profile package contract + package build
python tools/validate/validate_lite_package.py
python tools/package_lite.py --output artifacts/lite-package --force

# Thunder field native DDS / systemd deployment contract
python tools/validate/validate_real_deployment.py

# YOLO-World -> ONNX -> BPU
python tools/perception/export_yoloworld_bpu.py
bash tools/perception/convert_onnx_to_hbm.sh <onnx_path>

# Offline 3D reconstruction
python tools/reconstruction/reconstruct_local.py --dataset datasets/recording/<id>
```

Some field diagnostics may be copied into `/opt/lingtu/current/tools/` for
manual robot-side use. They should not be installed as systemd services from
this directory.
