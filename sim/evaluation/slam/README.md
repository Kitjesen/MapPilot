# LingTu SLAM Simulation Evaluation

This package is the LingTu-owned boundary for simulated SLAM evaluation. It is
kept separate from `src/localization/` runtime modules and from `sim/engine/` physics so
we can compare SLAM backends without changing robot behavior.

## Current Scope

- TUM trajectory read/write helpers.
- Timestamp association and translation error metrics.
- Declarative evaluation case manifests.
- License-aware public dataset catalog and native LTU1 replay manifests.
- A NOVA Dog + Fast-LIO2 baseline case manifest.

Public data is development and regression evidence, not a replacement for the
sealed Thunder V4 field corpus. In particular, sparse surveyed checkpoints,
continuous TUM trajectories, and no-ground-truth robustness sequences remain
distinct evidence types in the manifest instead of being normalized into one
misleading score.

## Public MID-360 Replay

Convert ROS 2 bags in an offline ROS 2 environment; the robot runtime remains
ROS-free. For an extracted AIST hard-localization sequence:

```bash
python3 scripts/datasets/public_slam_dataset.py manifest \
  aist-hard-localization-mid360 \
  /data/raw/outdoor_hard_01a \
  /data/ltu \
  --sequence outdoor_hard_01a \
  --write /data/ltu/outdoor_hard_01a.replay.json

python3 scripts/compat/ros2/datasets/ros2_bag_to_normalized_jsonl.py \
  /data/ltu/outdoor_hard_01a.replay.json \
  /data/ltu/outdoor_hard_01a.normalized.jsonl \
  --storage-id sqlite3

python3 scripts/datasets/normalized_lidar_imu_to_ltu1.py \
  /data/ltu/outdoor_hard_01a.normalized.jsonl \
  /data/ltu/outdoor_hard_01a.ltu

build/livox_sdk2_stream/livox_sdk2_stream \
  --validate-records < /data/ltu/outdoor_hard_01a.ltu
```

Run the native Fast-LIO2 runtime and the LTU1 publisher in two terminals on an
isolated DDS domain. This keeps replay traffic away from the robot's live domain
and does not start navigation, the driver, or any motion endpoint.

Terminal 1:

```bash
LINGTU_SLAM_BIN="$PWD/build/slam_core/slamd" \
LINGTU_SLAM_MODE=mapping \
LINGTU_SLAM_CONFIG="$PWD/src/localization/fastlio2/config/mid360_s100p.yaml" \
LINGTU_DDS_DOMAIN_ID=83 \
LINGTU_SLAM_STATUS_JSON=/tmp/lingtu_slam_status.json \
bash scripts/deploy/thunder/run_slam_dds.sh
```

Terminal 2:

```bash
build/livox_sdk2_stream/livox_sdk2_stream \
  --stdin-records \
  --dds \
  --domain-id 83 \
  --replay-rate 1.0 \
  < /data/ltu/outdoor_hard_01a.ltu
```

The end-to-end path is therefore `rosbag2 -> normalized JSONL -> LTU1 -> native
CycloneDDS -> Fast-LIO2 -> /tmp/lingtu_slam_status.json`. Use the readiness
commands below to capture unique status observations and evaluate the result.

The ROS adapter lazily imports `rosbag2_py`, `rclpy.serialization`, and the
message packages, so normal LingTu startup has no ROS dependency. PointCloud2
fields are decoded from their declared offsets, datatypes, row stride, and
endianness; Livox `CustomMsg` uses `timebase` plus manifest-declared
`offset_time` units.

Datasets without verified per-point timing require `--allow-undeskewed` on
both conversion commands. Those records retain `transport_only=true` and are
rejected by the LTU1 converter unless that downgrade is explicitly repeated;
they may test transport stability, not Fast-LIO deskew or accuracy.

## External Reference Notes

`github.com/17863958533/QR_SimEval_Code` is useful as a reference, but should not
be copied directly into LingTu. It is a ROS1/catkin evaluation bundle with
Unitree/Gazebo assumptions, generated build artifacts, and hardcoded local
paths.

Useful ideas to adapt into LingTu-owned code:

- Evaluation loop: simulation or bag replay -> SLAM output -> TUM trajectory ->
  APE/RPE style metrics.
- CMAPS-LIVO-style keyframe logging: pose, point cloud, and image snapshots for
  post-run inspection.
- Loop-closure direction: Scan Context candidate retrieval followed by ICP and a
  global optimizer. This belongs behind a LingTu SLAM backend boundary, not in
  the generic simulator.

## Boundary

- `sim/evaluation/slam/`: deterministic evaluation utilities and case manifests.
- `sim/engine/`: robot/world stepping and sensor generation.
- `src/localization/`: runtime SLAM modules and bridges.
- `src/lingtu/assembly/`: profile composition and wiring.

Future replay scripts should depend on this package for manifest parsing and
metrics, then call existing LingTu runtime profiles instead of introducing a
parallel robot stack.

## Patrol Localization Readiness

Use `scripts/datasets/inspection_localization_readiness.py` after replaying a
public or field SLAM run into native status snapshots. It reads JSONL snapshots,
explicit evidence and stationary windows, optional annotated degeneracy windows,
and an optional relocalization case file with map hashes. The output is a JSON
report whose top-level status is scoped as `LOCALIZATION_PASS`,
`LOCALIZATION_FAIL`, or `LOCALIZATION_INCOMPLETE`. The nested patrol readiness
remains `INCOMPLETE`, and `motion_authorization=false` is invariant because the
tool does not exercise planners, path following, the driver, motors, thermal
behavior, or camera actions. CLI exit codes 0/1/2 mean localization
pass/fail/incomplete only; they are not permission to move the robot.

The native SLAM runtime writes one atomic snapshot rather than a history file.
Capture unique observation sequences first; repeated reads of a frozen snapshot
are counted as duplicates and cannot satisfy the evidence gate:

```bash
python3 scripts/datasets/capture_slam_status_jsonl.py \
  /tmp/lingtu_slam_status.json \
  /tmp/inspection-static-60m.jsonl \
  --duration-s 3600 \
  --poll-hz 20 \
  --min-samples 30000 \
  --min-observed-duration-s 3590 \
  --summary /tmp/inspection-static-60m.capture.json
```

When the robot was physically stationary for the complete capture, explicitly
assert that fact and evaluate the full captured timestamp range:

```bash
python3 scripts/datasets/inspection_localization_readiness.py \
  /tmp/inspection-static-60m.jsonl \
  --full-evidence-window \
  --stationary-full-window \
  --min-status-samples 30000 \
  --min-stationary-duration 3590 \
  --max-yaw-drift-deg-per-min 0.5 \
  --write /tmp/inspection-static-60m.readiness.json
```

Both commands are evidence-only. Neither starts SLAM, changes the active map,
publishes a goal or velocity, or authorizes motion.

The same static acceptance can be captured and evaluated in one process:

```bash
python3 scripts/datasets/inspection_localization_readiness.py \
  /tmp/inspection-static-60m.jsonl \
  --capture-source /tmp/lingtu_slam_status.json \
  --capture-duration-s 3600 \
  --capture-poll-hz 20 \
  --capture-min-samples 30000 \
  --capture-min-observed-duration-s 3590 \
  --capture-summary /tmp/inspection-static-60m.capture.json \
  --full-evidence-window \
  --stationary-full-window \
  --min-status-samples 30000 \
  --min-stationary-duration 3590 \
  --write /tmp/inspection-static-60m.readiness.json
```

If capture completeness fails, a localization `PASS` is downgraded to
`INCOMPLETE`; capture errors never become motion authorization.
