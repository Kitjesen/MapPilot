# Camera-LiDAR Extrinsic Calibration

The default workflow uses [direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration) for target-less camera-LiDAR extrinsic calibration.

This directory is for offline calibration and comparison. ROS1/ROS2 tools here do not change the LingTu product boundary: the robot runtime remains ROS-free and consumes only the resulting extrinsic parameters.

## Why This Tool

- **No calibration target needed** — uses environment texture and structure.
- **Automatic** — NID-based optimization, no manual correspondence required after initialization.
- **ROS2 native for offline use** — builds with `colcon` in a calibration workspace.
- **Supports Livox** — non-repetitive scan LiDARs via static integration mode.

## What It Calibrates

- `T_lidar_camera`: 6DoF rigid transform from LiDAR frame to camera frame.
- Output format: `[x, y, z, qx, qy, qz, qw]`.

## Prerequisites

```bash
# Build from repository root; requires PCL, OpenCV, Ceres, Rust/Cargo
cd tools/calibration/camera_lidar/direct_visual_lidar_calibration
source /opt/ros/humble/setup.bash
colcon build --packages-select direct_visual_lidar_calibration
source install/setup.bash
```

LingTu's default build uses the Rust CT-ICP/CT-GICP optimizer and skips the legacy graph-optimizer dependency. For old comparison runs only, configure `-DLINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER=OFF`.

## Procedure

### 1. Collect Data

Record 5–10 bags. Keep the robot stationary for each 15 second recording, and choose textured scenes rather than blank walls.

```bash
ros2 bag record /camera/color/image_raw /camera/camera_info /livox/lidar \
  -o calib_bag_01 --duration 15
```

### 2. Preprocess

```bash
ros2 run direct_visual_lidar_calibration preprocess \
  calib_bag_01 preprocessed_01 \
  --image_topic /camera/color/image_raw \
  --camera_info_topic /camera/camera_info \
  --points_topic /livox/lidar \
  -v

# Do not pass -d for Livox; use static integration mode.
```

### 3. Initial Guess

```bash
# Manual GUI: pick 3+ corresponding points
ros2 run direct_visual_lidar_calibration initial_guess_manual preprocessed_01

# Optional automatic path; SuperGlue is a separate non-commercial dependency
ros2 run direct_visual_lidar_calibration find_matches_superglue.py preprocessed_01
ros2 run direct_visual_lidar_calibration initial_guess_auto preprocessed_01
```

The base calibration package is MIT-licensed. For commercial/product work, use manual initialization unless a separate SuperGlue license has been reviewed and approved.

### 4. Calibrate

```bash
ros2 run direct_visual_lidar_calibration calibrate preprocessed_01
# Result: preprocessed_01/calib.json
```

### 5. Verify

```bash
ros2 run direct_visual_lidar_calibration viewer preprocessed_01
```

### 6. Apply

```bash
python tools/calibration/apply_calibration.py \
  --camera-lidar preprocessed_01/calib.json
```

## LingTu Defaults

| Parameter | Value | Notes |
| --- | --- | --- |
| Image topic | `/camera/color/image_raw` | Orbbec RGB |
| Camera info | `/camera/camera_info` | Includes K + D |
| LiDAR topic | `/livox/lidar` | Livox Mid-360 CustomMsg |
| Camera model | `plumb_bob` | Standard pinhole + Brown-Conrady |
| Integration mode | Static | No `-d` flag for Livox |

## Alternative Tools

These vendored tools are kept for field comparison and special calibration setups. Do not import their ROS launch/runtime assumptions into LingTu native navigation or bundle the snapshots into a product runtime/release artifact.

### livox_calib_standalone

`livox_calib_standalone/` is the preferred ROS-free comparison path when a calibration workstation should avoid ROS launch/runtime dependencies. It derives from and includes headers from `livox_camera_calib/`, so the adaptation is GPL-2.0-only and remains an offline/vendor tool.

### livox_camera_calib (HKU-MARS)

[livox_camera_calib](https://github.com/hku-mars/livox_camera_calib) is a targetless extrinsic calibration tool optimized for Livox high-resolution LiDARs. It uses edge information in the scene and supports single-scene and multi-scene calibration.

- **Prerequisites**: ROS1 (Kinetic/Melodic), Ceres, PCL, Eigen.
- **Strengths**: pixel-level accuracy, no calibration target, Livox-optimized.
- **License**: GPL-2.0-only.
- **Boundary**: vendored snapshot for offline comparison only; do not merge or bundle it into the product runtime/release.

### livox_camera_lidar_calibration (Livox-SDK Official)

[livox_camera_lidar_calibration](https://github.com/Livox-SDK/livox_camera_lidar_calibration) is Livox's official target-based calibration solution. It uses calibration board corners and includes camera intrinsic calibration, point cloud projection, and coloring tools.

- **Prerequisites**: ROS1, Livox SDK, PCL, Ceres.
- **Strengths**: official Livox support.
- **Boundary**: MIT-licensed vendored snapshot for offline use; requires a physical calibration board, typically around 1 x 1.5 m.

### mlcc (HKU-MARS — Multi-LiDAR/Camera)

[mlcc](https://github.com/hku-mars/mlcc) provides targetless extrinsic calibration for multiple LiDARs and cameras using adaptive voxelization.

- **Use case**: S100P upgrades with additional LiDARs or cameras.
- **Supports**: multi-LiDAR extrinsic, multi-LiDAR-camera extrinsic, single LiDAR-camera.
- **Prerequisites**: ROS1, Ceres, OpenCV, PCL, Eigen.
- **Boundary**: GPL-2.0-only vendored reference/offline tool; do not merge or bundle it into the product runtime/release.

## Important Notes

- Livox Mid-360 is a non-repetitive scan LiDAR. Keep the robot stationary during recording for good point cloud accumulation.
- The algorithm matches image textures to LiDAR intensity; featureless walls produce poor results.
- Use 5–10 bags from different viewpoints for best accuracy.
- Always check the projected point cloud overlay before accepting results.
- Accepted results must be written through `tools/calibration/apply_calibration.py` and checked with `tools/calibration/verify.py`.
