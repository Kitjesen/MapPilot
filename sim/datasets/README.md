# Simulation Dataset Boundary

`sim/datasets/` stores offline LiDAR/IMU datasets and small metadata fixtures
used by replay and SLAM evaluation scripts.

## Current Dataset Families

| Directory | Role | Retention |
| --- | --- | --- |
| `Avia/` | Livox Avia replay data or placeholders for LiDAR-inertial tests. | Keep locally when running Avia replay/evaluation. |
| `legkilo_outdoor/` | Outdoor LEG-KILO trajectory and ROS 2 metadata fixtures. | Keep locally when validating outdoor replay scenarios. |
| `legkilo_full/` | Expanded LEG-KILO corridor fixtures and metadata. | Keep locally when running corridor replay or SLAM comparison gates. |
| `legkilo_all/` | Aggregated LEG-KILO dataset material when present locally. | Optional local cache; do not commit large raw files. |
| `legkilo-dataset/` | Compatibility path for older scripts. | Keep only if an older script still expects this path. |

## Contract

- This folder is for offline replay inputs, not generated validation evidence.
- Reproducible gate outputs belong under `artifacts/`, not `sim/datasets/`.
- Large raw bags and ROS 2 `.db3` files may be kept locally for replay, but
  should stay out of git unless there is an explicit small fixture contract.
- Incomplete download fragments such as `*.part` are disposable and should be
  deleted during cleanup.
- Dataset scripts must report whether evidence is raw replay, SLAM algorithm
  replay, or saved-map relocalization; do not treat those levels as equivalent.

## Cleanup Policy

Safe to delete without changing repository behavior:

- `*.part` partial downloads
- generated reports, videos, and temporary output under `sim/output/`
- Python cache directories such as `__pycache__/`

Do **not** delete complete `.bag`, `.db3`, metadata, or trajectory files unless
you are intentionally freeing local disk space and can restore the dataset from
its source.
