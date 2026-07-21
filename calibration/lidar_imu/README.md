# LiDAR-IMU Extrinsic Calibration

Uses [LiDAR_IMU_Init (LI-Init)](https://github.com/hku-mars/LiDAR_IMU_Init) from HKU-MARS to calibrate LiDAR-IMU extrinsics: rotation, translation, time offset, initial IMU bias, and gravity direction.

This is an offline compatibility workflow. LI-Init is ROS1/catkin-based and must not be treated as a LingTu product runtime dependency.

## What It Calibrates

- **Rotation** (`r_il`): 3x3 rotation matrix from LiDAR frame to IMU frame.
- **Translation** (`t_il`): 3D offset from LiDAR origin to IMU origin.
- **Time offset**: temporal delay between LiDAR and IMU timestamps.
- **IMU bias**: initial accelerometer and gyroscope bias estimates.
- **Gravity vector**: gravity direction in the initial IMU frame.

## Prerequisites

LI-Init is ROS1/catkin. Use one of these calibration-workstation approaches.

### Option A: ROS1 Environment

```bash
cd catkin_ws/src
ln -s /path/to/lingtu/calibration/lidar_imu/LiDAR_IMU_Init .
cd ..
catkin_make
source devel/setup.bash
```

### Option B: ros1_bridge for ROS2 Bag Replay

```bash
# Terminal 1: ros1_bridge
source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash
ros2 run ros1_bridge dynamic_bridge

# Terminal 2: ROS1 LI-Init
roslaunch lidar_imu_init livox_mid360.launch

# Terminal 3: play ROS2 bag
ros2 bag play lidar_imu_bag
```

The bridge is only for replaying calibration data into LI-Init. Do not install or run this bridge in the Thunder/S100P native navigation stack.

## Procedure

### 1. Collect Data

Move the robot in a figure-8 pattern to excite all IMU axes. About 60 seconds is enough.

```bash
ros2 bag record /livox/lidar /livox/imu -o lidar_imu_bag --duration 60
```

### 2. Run Calibration

```bash
# Using our adapted config; key setting: mean_acc_norm=1 for Livox built-in IMU
roslaunch lidar_imu_init livox_mid360.launch

# Result:
#   calibration/lidar_imu/LiDAR_IMU_Init/result/Initialization_result.txt
```

### 3. Parse Result

```bash
python calibration/lidar_imu/ros2_adapter/parse_result.py \
  --input calibration/lidar_imu/LiDAR_IMU_Init/result/Initialization_result.txt \
  --output calibration/lidar_imu/output/lidar_imu_calib.yaml
```

### 4. Apply

```bash
python calibration/apply_calibration.py \
  --lidar-imu calibration/lidar_imu/output/lidar_imu_calib.yaml
```

## Config Adaptation

LingTu's `calibration/lidar_imu/config/mid360.yaml` differs from upstream:

| Parameter | Upstream | LingTu | Reason |
| --- | --- | --- | --- |
| `imu_topic` | `/mavros/imu/data_raw` | `/livox/imu` | LingTu standard topic |
| `mean_acc_norm` | `9.805` | `1` | Livox built-in IMU outputs in g |
| `scan_line` | `6` | `4` | Mid-360 has 4 beams |

## Important Notes

- `mean_acc_norm = 1` is critical for the Livox built-in IMU. If using an external IMU that outputs m/s^2, set it to `9.805`.
- The robot must move during calibration; stationary data will fail.
- Calibration usually converges in 10–20 seconds of motion data.
- Results should have rotation angle under 5 degrees and translation under 10 cm for co-located sensors.
- Product acceptance still requires `python calibration/verify.py` and a short SLAM sanity run after applying the result.
