# Camera-LiDAR Extrinsic Calibration

Uses [direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration)
for target-less camera-LiDAR extrinsic calibration.

## Why This Tool

- **No calibration target needed** — uses environment texture and structure
- **Automatic** — NID-based optimization, no manual correspondence required
- **ROS2 native** — works directly with `colcon build`
- **Supports Livox** — non-repetitive scan LiDARs (static integration mode)

## What It Calibrates

- `T_lidar_camera`: 6DoF rigid transform from LiDAR frame to camera frame
- Output format: `[x, y, z, qx, qy, qz, qw]` (position + quaternion)

## Prerequisites

```bash
# Build from the repository root (default path requires PCL, OpenCV, Ceres, Rust/Cargo)
python scripts/build/build_rust_kernels.py --target camera_lidar_optimizer --release
cd calibration/camera_lidar/direct_visual_lidar_calibration
source /opt/ros/humble/setup.bash
colcon build --packages-select direct_visual_lidar_calibration
source install/setup.bash
```

The LingTu default build uses the Rust CT-ICP/CT-GICP optimizer and skips the
legacy graph-optimizer dependency. For old comparison runs only, configure
`-DLINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER=OFF`.

## Procedure

### 1. Collect Data

Record **5-10 bags**, each 15 seconds, robot **stationary**, in a
**textured environment** (avoid blank walls).

```bash
# Each recording: keep robot still for 15 seconds
ros2 bag record /camera/color/image_raw /camera/camera_info /livox/lidar \
    -o calib_bag_01 --duration 15
```

Repeat from different viewpoints.

### 2. Preprocess

```bash
# For each bag
ros2 run direct_visual_lidar_calibration preprocess \
    calib_bag_01 preprocessed_01 \
    --image_topic /camera/color/image_raw \
    --camera_info_topic /camera/camera_info \
    --points_topic /livox/lidar \
    -v

# Note: NO -d flag for Livox (non-repetitive scan, use static integrator)
```

### 3. Initial Guess

```bash
# Manual (GUI — pick 3+ corresponding points)
ros2 run direct_visual_lidar_calibration initial_guess_manual preprocessed_01

# Or automatic (requires SuperGlue, non-commercial license)
ros2 run direct_visual_lidar_calibration find_matches_superglue.py preprocessed_01
ros2 run direct_visual_lidar_calibration initial_guess_auto preprocessed_01
```

### 4. Calibrate

```bash
ros2 run direct_visual_lidar_calibration calibrate preprocessed_01
# Result: preprocessed_01/calib.json
```

### 5. Verify

```bash
# Visual inspection — LiDAR points projected onto image
ros2 run direct_visual_lidar_calibration viewer preprocessed_01
```

### 6. Apply

```bash
python calibration/apply_calibration.py \
    --camera-lidar preprocessed_01/calib.json
```

## Our Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| Image topic | `/camera/color/image_raw` | Orbbec RGB |
| Camera info | `/camera/camera_info` | Includes K + D |
| LiDAR topic | `/livox/lidar` | Livox Mid-360 CustomMsg |
| Camera model | `plumb_bob` | Standard pinhole + Brown-Conrady |
| Integration mode | Static (default) | No `-d` flag for Livox |

## Alternative Tools

### livox_camera_calib (HKU-MARS)

[livox_camera_calib](https://github.com/hku-mars/livox_camera_calib) is a
**targetless** extrinsic calibration tool specifically optimized for **Livox**
high-resolution LiDARs. It uses edge information in the scene and supports
both single-scene and multi-scene calibration.

- **Prerequisites**: ROS1 (Kinetic/Melodic), Ceres, PCL, Eigen
- **Strengths**: Pixel-level accuracy, no calibration target, Livox-optimized
- **Note**: Requires ROS1; use rosbag replay or a ROS1 bridge

### livox_camera_lidar_calibration (Livox-SDK Official)

[livox_camera_lidar_calibration](https://github.com/Livox-SDK/livox_camera_lidar_calibration)
is **Livox's official** calibration solution. It uses a **target-based** approach
(calibration board corners) and has been verified on Mid-40, Horizon, and Tele-15.

- **Prerequisites**: ROS1, Livox SDK, PCL, Ceres
- **Strengths**: Official Livox support, includes camera intrinsic calibration,
  point cloud projection and coloring tools
- **Note**: Requires physical calibration board (1x1.5m recommended)

### mlcc (HKU-MARS — Multi-LiDAR/Camera)

[mlcc](https://github.com/hku-mars/mlcc) provides targetless extrinsic
calibration for **multiple LiDARs and cameras** using adaptive voxelization.

- **Use case**: If S100P is upgraded with additional LiDARs or cameras
- **Supports**: Multi-LiDAR extrinsic, multi-LiDAR-camera extrinsic, single LiDAR-camera
- **Prerequisites**: ROS1, Ceres, OpenCV, PCL, Eigen

## Important Notes

- **Static integrator**: Livox Mid-360 is non-repetitive scan. Robot must be **stationary**
  during recording for good point cloud accumulation.
- **Textured environment**: The algorithm matches image textures to LiDAR intensity.
  Featureless walls will cause poor results.
- **Multiple bags**: Use 5-10 bags from different viewpoints for best accuracy.
- **Verify visually**: Always check the projected point cloud overlay before accepting results.
