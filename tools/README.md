# Tools

Developer and operator tools that support the shared **mapping -> map
save/reload -> localization -> autonomous navigation** workflow live here.
Start with the task below; optional authoring and research tools are listed
separately.

Product lifecycle belongs to `python -m lingtu.control` and the thin
[`scripts/lingtu`](../scripts/lingtu) adapter. Build, installation, robot checkout
sync, and release version sync belong under [`scripts/`](../scripts/README.md).
Runtime implementations belong under `src/`.

## Find a tool by task

| Task | Directory | Starting point |
| --- | --- | --- |
| Calibrate sensors and apply measured results | `calibration/` | [Calibration guide](calibration/README.md), `apply_calibration.py`, `verify.py` |
| Capture, convert, or evaluate SLAM recordings | [`datasets/`](datasets/README.md) | `public_slam_dataset.py`, `capture_slam_status_jsonl.py`, `normalized_lidar_imu_to_ltu1.py`, `elevator_bag_to_ltu1.py`, `inspection_localization_readiness.py` |
| Inspect semantic, RGB-D, and person-follow datasets | [`datasets/`](datasets/README.md) | `semantic_dataset_inventory.py`; Habitat, ReplicaCAD, 3DSSG, TUM RGB-D and person evaluation tools |
| Inspect field inputs and navigation evidence | `diagnostics/` | `navigation_motion_evidence.py`, `elevator_parity.py`; native probes and mapping replay are built by their owning components |
| Check saved-map pruning | `maps/` | `prune_check.sh`; `erasor2_check.sh` is its compatibility alias, and `dufomap_offline.py` is an optional external comparison |
| Prepare robot hardware or a simulation model | `robot/` | `setup_network.sh`, `scaffold_robot.py`; Go2 mounting assets stay with their hardware instructions |
| Validate configuration and repository contracts | `validate/` | `validate_config.py`, `validate_topics.py`, `validate_architecture_boundaries.py`, `validate_real_deployment.py` |
| View a saved map, Gateway, or SLAM streams | [`visualization/`](visualization/README.md) | `map.html` for saved maps; `rerun_gateway_live.py`, `run_rerun_mapping.py` for streams |

Saving/loading maps and starting navigation use ProductControl and the
[operator workflow](../docs/operations.md), not a separate tool-owned lifecycle.
The robot scaffolder creates a simulation model skeleton; it does not calibrate
hardware or qualify a new robot for autonomous motion.

```bash
# Validate the selected robot's configuration structure
python tools/validate/validate_config.py config/robots/unitree/go2/robot.yaml

# Inspect calibration application options, then verify existing configuration
python tools/calibration/apply_calibration.py --help
python tools/calibration/verify.py

# Inspect the navigation evidence collector without connecting to a robot
python tools/diagnostics/navigation_motion_evidence.py --help

# Inspect the simulation model scaffolder
python tools/robot/scaffold_robot.py --help
```

## Specialized development tools

For an official Elevator-LIO ROS1 recording, the converter reads bag V2 records
and `none`/`bz2` chunks directly with Python's standard library and the existing
NumPy installation. It does not require ROS or `rosbags`:

```bash
python -m tools.datasets.elevator_bag_to_ltu1 /data/Office1.bag build/elevator-input
cmake --build build/slam_core --target ltlio_replay
build/slam_core/ltlio_replay build/elevator-input/Office1.ltu DATASET_CONFIG.yaml build/elevator-output/office-off
```

Use the dataset's calibration, not a robot model's calibration. LTU1 retains
sensor timestamps and full Livox offset/tag/line values; the companion records
index retains bag timestamps, frame IDs and timebase. IMU values are not
rescaled by conversion. The dataset's Livox acceleration values require the
appropriate scale in the replay configuration. Source, license, message counts
and conversion details are recorded in the manifest.

`ltlio_replay` calls the native `ISlamBackend` and writes continuous odom/map TUM
trajectories plus state/attempt and transition JSONL. Its summary distinguishes
LiDAR attempts, accepted updates, rejected attempts and published observations;
input clouds without an attempt retain initialization, dropped or unfinished
tail frames. A returned process exit code does not imply good localization.
An optional final argument accepts recorded manual event rows
`bag_timestamp_ns 0_or_1`; an empty event file does not enable manual transport.
These diagnostic tools do not start navigation or robot services. Without
independent ground truth, replay is a stability/consistency comparison, not an
ATE/RPE accuracy qualification.

| Directory or file | Purpose |
| --- | --- |
| [`simstudio/`](simstudio/README.md) | Optional local scene authoring and run inspection; start with `python -m tools.simstudio`. |
| [`inference/`](inference/README.md) | Optional GPU vision inference service and fixed-person detector export for person-follow development. |
| `perception/` | Optional BPU model export/conversion and camera preview. |
| `reconstruction/` | Offline reconstruction from recorded data; see `reconstruct_local.py --help`. |
| [`bench/`](bench/README.md) | Offline planner regression, camera/follow load measurements, semantic-map previews, diagnostic videos and optimizer C ABI checks. |
| `docs/` | [`extract_api_docs.py`](docs/extract_api_docs.py) maintains the API reference; [`export_codex_thread.py`](docs/export_codex_thread.py) exports a selected local rollout. |
| [`generate_topic_contracts.py`](generate_topic_contracts.py) | Generate shared Python/C++/IDL contracts from canonical message sources. |

These tools are not all prerequisites for the first delivery. Keep their
dependencies and output out of the default robot runtime.

## Keeping this directory useful

- Put a reusable tool beside the tools for its task. Keep only shared contract
  generation at the tools root; robot helpers belong under `robot/`.
- Keep build/deploy operations under `scripts/` and runtime behavior under
  `src/`; tools must not become a second Product lifecycle owner.
- Keep dated, machine-specific experiment launchers and retained investigation
  material outside this maintained tree. Reproducible output belongs under
  `build/` or the command's declared output directory; bytecode stays untracked.
- Preserve calibration inputs, field recordings, and models when retiring a
  tool. Retained upstream tools keep their license and source information.
- Update callers, tests, and this index together when an entrypoint moves.

Directory indexes describe maintained entrypoints. Hardware mounting assets and
third-party calibration sources stay with their existing instructions and
licenses; experimental outputs belong in the selected output directory.

Selected diagnostics may be installed for manual field use. Nothing in this
directory is itself a systemd service entrypoint.
