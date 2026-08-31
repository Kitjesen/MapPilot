# tools/ - developer utilities

`tools/` holds one-shot developer and operator utilities: validation,
calibration, diagnostics, release maintenance, and robot setup.

It is not a robot runtime entrypoint and not a systemd service directory.
Build, release installation, and Product lifecycle entrypoints belong under
`scripts/`; runtime code belongs under `src/`.

## Layout

| Path | Purpose |
| --- | --- |
| `bench/` | Native navigation, pose-graph, and camera-LiDAR parity/benchmark scripts. |
| `calibration/` | Offline sensor calibration, result application, and configuration verification tools. |
| `datasets/` | Offline SLAM dataset capture and conversion. |
| `deploy/` | Workstation helpers for updating a robot checkout. |
| `diagnostics/` | Read-only field comparison and inspection tools. |
| `docs/` | Documentation generation helpers. |
| `maps/` | Optional saved-map cleanup and comparison tools. |
| `perception/` | BPU model export and ONNX-to-HBM conversion. |
| `reconstruction/` | Offline 3D reconstruction and recorded dataset replay helpers. |
| `release/` | Repository release metadata maintenance. |
| `robot/` | One-shot robot hardware and network setup tools. |
| `simstudio/` | Local simulation authoring and run-inspection application; not a robot Product. |
| `validate/` | Static contract validators for config, topics, architecture, and migration gates. |
| `visualization/` | Rerun viewers for Gateway and native DDS streams. |

`scaffold_robot.py` creates a new vendor/model configuration skeleton; it is a
developer tool, not a runtime loader.

Generated code, caches, and long-running robot services do not belong here.
Python bytecode caches stay untracked.

## Common Commands

```bash
# selected RobotConfig structure validation
python tools/validate/validate_config.py

# Offline sensor calibration result application and verification
python tools/calibration/apply_calibration.py --help
python tools/calibration/verify.py

# Topic contract validation
python tools/validate/validate_topics.py

# Product and Host boundary validation
python tools/validate/validate_architecture_boundaries.py

# Thunder field native DDS / systemd deployment contract
python tools/validate/validate_real_deployment.py

# Local simulation authoring UI/API
python -m tools.simstudio

# YOLO-World -> ONNX -> BPU
python tools/perception/export_yoloworld_bpu.py
bash tools/perception/convert_onnx_to_hbm.sh <onnx_path>

# Offline 3D reconstruction
python tools/reconstruction/reconstruct_local.py --dataset datasets/recording/<id>
```

Some field diagnostics may be copied into `/opt/lingtu/current/tools/` for
manual robot-side use. They should not be installed as systemd services from
this directory.
