# IMU Noise Calibration (Allan Variance)

Uses [allan_variance_ros2](https://github.com/Autoliv-Research/allan_variance_ros2) to characterize
Livox Mid-360 built-in IMU noise parameters via Allan Variance analysis.

## Output Parameters

| Parameter | Symbol | Unit | Used by |
|-----------|--------|------|---------|
| Accelerometer noise density | na | m/s^2/sqrt(Hz) | Fast-LIO2, Point-LIO |
| Gyroscope noise density | ng | rad/s/sqrt(Hz) | Fast-LIO2, Point-LIO |
| Accelerometer random walk | nba | m/s^3/sqrt(Hz) | Fast-LIO2, Point-LIO |
| Gyroscope random walk | nbg | rad/s^2/sqrt(Hz) | Fast-LIO2, Point-LIO |

## Prerequisites

```bash
# Build allan_variance_ros2
cd calibration/imu/allan_variance_ros2
colcon build --packages-select allan_variance_ros2
source install/setup.bash
```

## Procedure

### 1. Record Static Data

Place robot on a **stable, level surface**. No vibration, no movement.

```bash
ros2 bag record /livox/imu -o imu_static_bag --duration 7200
```

Minimum 2 hours. 3 hours is better.

### 2. Run Analysis

```bash
ros2 run allan_variance_ros2 allan_variance \
    imu_static_bag \
    calibration/imu/config/livox_mid360_imu.yaml

# Output: allan_variance.csv
```

### 3. Fit Noise Model

```bash
python3 calibration/imu/allan_variance_ros2/src/allan_variance_ros2/scripts/analysis.py \
    --data allan_variance.csv

# Output: imu.yaml (Kalibr-compatible format)
# Copy to calibration/imu/output/imu.yaml
```

### 4. Apply

```bash
python calibration/apply_calibration.py --imu calibration/imu/output/imu.yaml
```

This updates:
- `config/robot_config.yaml` (if IMU section exists)
- `src/localization/fastlio2/config/lio.yaml` (na, ng, nba, nbg)
- `config/pointlio.yaml` (imu_meas_acc_cov, imu_meas_omg_cov, b_acc_cov, b_gyr_cov)

## Important Notes

- Livox built-in IMU outputs acceleration in **g** (not m/s^2) â€?the SLAM stack handles conversion
- `mean_acc_norm` in LiDAR_IMU_Init should be set to `1` for Livox built-in IMU
- Default noise values (na=0.01, ng=0.01) are conservative; calibrated values are typically lower
