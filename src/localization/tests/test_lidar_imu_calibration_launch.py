from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
CALIBRATION_ROOT = REPO_ROOT / "tools" / "calibration" / "lidar_imu"


def test_livox_mid360_launch_accepts_the_lingtu_calibration_config() -> None:
    launch_path = (
        CALIBRATION_ROOT
        / "LiDAR_IMU_Init"
        / "launch"
        / "livox_mid360.launch"
    )
    launch = launch_path.read_text(encoding="utf-8")

    assert '<arg name="config"' in launch
    assert 'default="$(find lidar_imu_init)/config/mid360.yaml"' in launch
    assert '<rosparam command="load" file="$(arg config)"' in launch


def test_lingtu_mid360_config_matches_the_built_in_sensor_contract() -> None:
    config = (CALIBRATION_ROOT / "config" / "mid360.yaml").read_text(
        encoding="utf-8"
    )

    assert 'imu_topic:  "/livox/imu"' in config
    assert "scan_line: 4" in config
    assert "mean_acc_norm: 1" in config
