# Simulation Dataset Boundary

`sim/evaluation/data/` stores offline LiDAR/IMU datasets and small metadata fixtures
used by replay and SLAM evaluation scripts.

## Current Dataset Families

| Directory | Role | Retention |
| --- | --- | --- |
| `Avia/` | Livox Avia replay data or placeholders for LiDAR-inertial tests. | Keep locally when running Avia replay/evaluation. |
| `legkilo_outdoor/` | Outdoor LEG-KILO trajectory and ROS 2 metadata fixtures. | Keep locally when validating outdoor replay scenarios. |
| `legkilo_full/` | Expanded LEG-KILO corridor fixtures and metadata. | Keep locally when running corridor replay or SLAM comparison gates. |
| `legkilo_all/` | Aggregated LEG-KILO dataset material when present locally. | Optional local cache; do not commit large raw files. |

## Contract

- This folder is for offline replay inputs, not generated validation evidence.
- Reproducible gate outputs belong under `artifacts/`, not `sim/evaluation/data/`.
- Large raw bags and ROS 2 `.db3` files may be kept locally for replay, but
  should stay out of git unless there is an explicit small fixture contract.
- Incomplete download fragments such as `*.part` are disposable and should be
  deleted during cleanup.
- Dataset scripts must report whether evidence is raw replay, SLAM algorithm
  replay, or saved-map relocalization; do not treat those levels as equivalent.

## Public SLAM Datasets

The repository-owned catalog is
`sim/evaluation/slam/configs/public_datasets.json`. It records official source
URLs, licenses, sensor topics, point-time capability, ground-truth type, and
the intended regression use. Raw datasets remain outside git under
`$LINGTU_DATASET_ROOT`.

List the catalog:

```bash
python tools/datasets/public_slam_dataset.py list
python tools/datasets/public_slam_dataset.py list --json
```

Recommended order:

1. `aist-driving-mid360`: small transport/importer smoke only; its published
   PointCloud2 point-time fields have not been asserted by the catalog.
2. `aist-flatwall-avia`: fully degenerate flat-wall behavior.
3. `aist-hard-localization-mid360`: fast motion, interruption, and kidnapped
   saved-map localization.
4. `iilabs3d-mid360`: IMU calibration, indoor loops, ramps, and multilevel
   motion. Inspect PointCloud2 fields before using it for deskew-dependent
   frontend scoring.
5. `rtk-slam-mid360`: long-range absolute checkpoint error without SE(3)
   alignment.
6. `aliw-farm-mid360`: off-road vibration and feature-sparse robustness; it
   has no independent absolute trajectory reference.

Create a replay manifest after downloading a sequence:

```bash
python tools/datasets/public_slam_dataset.py manifest \
  aist-hard-localization-mid360 \
  "$LINGTU_DATASET_ROOT/aist-hard/outdoor_hard_01a" \
  "$LINGTU_DATASET_ROOT/normalized/aist-hard" \
  --sequence outdoor_hard_01a \
  --write "$LINGTU_DATASET_ROOT/normalized/aist-hard/outdoor_hard_01a.replay.json"
```

Dataset-specific ROS readers run only in an offline compatibility environment.
They must emit normalized JSONL and preserve source ordering and timestamps:

```json
{"type":"imu","timestamp_ns":1000,"gyro":[0.1,-0.2,0.3],"acc":[0.0,0.0,9.80665]}
{"type":"cloud","timestamp_ns":2000,"points":[{"x":1.0,"y":2.0,"z":3.0,"intensity":42.0,"offset_time_ns":17,"tag":0,"line":0,"flags":0}]}
```

Convert normalized records to the existing native LTU1 replay format:

```bash
python tools/datasets/normalized_lidar_imu_to_ltu1.py \
  normalized.jsonl sensors.ltu
```

Missing point-level time is rejected by default. For transport/runtime smoke
only, it may be made explicit:

```bash
python tools/datasets/normalized_lidar_imu_to_ltu1.py \
  --allow-undeskewed normalized.jsonl sensors.ltu
```

An output with nonzero `undeskewed_points` is not valid evidence for Fast-LIO
frontend accuracy, time synchronization, or motion compensation.

Validate and replay through the same native path used by MID-360 capture:

```bash
livox_sdk2_stream --validate-records < sensors.ltu
livox_sdk2_stream --stdin-records --dds --replay-rate 1.0 < sensors.ltu
```

The catalog's `conversion_adapter` field names the required source-specific
offline adapter contract. The dependency-free normalized-JSONL-to-LTU1
converter is implemented now; raw ROS bag readers remain host-side tooling and
must not become a product runtime dependency.

## Cleanup Policy

Safe to delete without changing repository behavior:

- `*.part` partial downloads
- generated reports, videos, and temporary output under `sim/output/`
- Python cache directories such as `__pycache__/`

Do **not** delete complete `.bag`, `.db3`, metadata, or trajectory files unless
you are intentionally freeing local disk space and can restore the dataset from
its source.
