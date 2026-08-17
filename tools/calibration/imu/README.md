# IMU Noise Calibration (Allan Variance)

Uses [allan_variance_ros2](https://github.com/Autoliv-Research/allan_variance_ros2) to characterize Livox Mid-360 built-in IMU noise parameters via Allan Variance analysis.

This is an offline calibration workflow. It may use ROS 2 and rosbag on a calibration workstation, but ROS 2 is not part of the LingTu product runtime.

## Output Parameters

| Parameter | Symbol | Unit | Used by |
| --- | --- | --- | --- |
| Accelerometer noise density | `na` | m/s^2/sqrt(Hz) | Fast-LIO2, Point-LIO |
| Gyroscope noise density | `ng` | rad/s/sqrt(Hz) | Fast-LIO2, Point-LIO |
| Accelerometer random walk | `nba` | m/s^3/sqrt(Hz) | Fast-LIO2, Point-LIO |
| Gyroscope random walk | `nbg` | rad/s^2/sqrt(Hz) | Fast-LIO2, Point-LIO |

## Prerequisites

```bash
# Run from the repository root.
source /opt/ros/humble/setup.bash
pushd tools/calibration/imu/allan_variance_ros2 >/dev/null
colcon build --packages-select allan_variance_ros2
source install/setup.bash
popd >/dev/null
```

Use this environment only for calibration data capture and analysis. Commit or deploy only the generated calibration values, not the ROS 2 workspace.

For the standard three-hour workflow, use the repository entry point:

```bash
bash scripts/hardware/run_allan_variance.sh all
```

## Procedure

### 1. Record Static Data

Place the robot on a stable, level surface. Avoid vibration and motion.

```bash
export LINGTU_AV_TS="$(date +%Y%m%d_%H%M%S)"
bash scripts/hardware/run_allan_variance.sh record
```

The entry point records three hours by default. Override it with `LINGTU_AV_DURATION`; do not use less than two hours.

### 2. Run Analysis

```bash
bash scripts/hardware/run_allan_variance.sh analyze \
  "$HOME/data/imu_calib/$LINGTU_AV_TS/imu_static_bag"

# Output: $HOME/data/imu_calib/$LINGTU_AV_TS/imu.yaml
```

### 3. Fit Noise Model

The `analyze` phase runs both Allan Variance extraction and noise-model fitting. Review the generated `imu.yaml`, then retain the accepted values in the calibration record.

### 4. Apply

```bash
bash scripts/hardware/run_allan_variance.sh apply \
  "$HOME/data/imu_calib/$LINGTU_AV_TS/imu.yaml"
```

This updates, when present:

- `src/localization/fastlio2/config/mid360_s100p.yaml` (`na`, `ng`, `nba`, `nbg`)
- `config/pointlio.yaml` (`imu_meas_acc_cov`, `imu_meas_omg_cov`, `b_acc_cov`, `b_gyr_cov`)

IMU noise application does not write `config/robot_config.yaml`.

## Important Notes

- Livox built-in IMU outputs acceleration in **g** (not m/s^2); the SLAM stack handles conversion.
- `mean_acc_norm` in LiDAR_IMU_Init should be set to `1` for the Livox built-in IMU.
- Default noise values (`na=0.01`, `ng=0.01`) are conservative; calibrated values are typically lower.
- After applying the result, run `python tools/calibration/verify.py` and a short SLAM sanity check.
