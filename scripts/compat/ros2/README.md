# ROS2 Compatibility Scripts

This directory is the explicit quarantine boundary for ROS2-only developer and
diagnostic tools. Nothing here is a default Product process, a canonical field
operation, or a dependency of `scripts/lingtu`.

## Isolated Tools

Run these only when an explicit ROS2 compatibility environment is intended:

| Purpose | Compatibility entrypoint |
| --- | --- |
| Convert ROS2 bags to normalized replay JSONL | `datasets/ros2_bag_to_normalized_jsonl.py` |
| Live ROS camera detection demo | `perception/live_detect.py` |
| Live ROS camera tracking demo | `perception/live_track.py` |
| Capture ROS2 topics for legacy calibration workflows | `hardware/record_bag.sh` |
| Prepare the optional Fast-LIO2 validation host | `setup_fastlio2_validation_host.sh` |

The compatibility setup remains opt-in:

```bash
LINGTU_INSTALL_ROS2=1 LINGTU_RUN_ROS2_FASTLIO2=1 \
  bash scripts/compat/ros2/setup_fastlio2_validation_host.sh
```

The canonical viewers remain `scripts/visualization/rerun_gateway_live.py`
(Gateway) and `scripts/visualization/rerun_live.py` (native CycloneDDS). Neither
viewer belongs in this ROS2 compatibility boundary.

## Native Gateway Recording

The following compatibility-named Gateway routes control the native
CycloneDDS/MCAP recorder, not ROS2 or rosbag2:

- `POST /api/v1/bag/start`;
- `GET /api/v1/bag/status`;
- `POST /api/v1/bag/stop`.

They are a thin HTTP adapter over the native C++ session manager. The `bag`
path segment is retained only for released client compatibility.

For native recording and replay use:

```bash
scripts/lingtu record
scripts/lingtu record --camera
scripts/lingtu record status SESSION_DIR
scripts/lingtu record verify SESSION_DIR
scripts/lingtu play SESSION_DIR
```

See the canonical
[native recording guide](../../../docs/04-deployment/native_recording.md).


## Deferred Compatibility Surfaces

The following ROS2 surfaces have active deployment, test, or Gateway
references and remain at their stable paths until separate bounded migrations:

- `scripts/build/build_ros_workspace.sh`
- `scripts/build/clone_orbbec_ros2.sh`
- `scripts/deploy/thunder/ros2-env.sh`
- `scripts/lingtu doctor --ros2`
- `scripts/gates/real_runtime_evidence_collect.py --collector ros2`

New ROS-only utilities belong under this compatibility tree. Do not add ROS
environment sourcing, `ros2` commands, `rclpy`, rosbag, ament, or colcon to
default Product, build, release, or field-operation entrypoints.
