from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest
from tools.calibration.lidar_imu.ros2_adapter.parse_result import (
    parse_initialization_result,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
REAL_LI_INIT_RESULT = (
    REPO_ROOT / "tools" / "calibration" / "lidar_imu" / "LiDAR_IMU_Init" / "result" / "Initialization_result.txt"
)
PARSER = REPO_ROOT / "tools" / "calibration" / "lidar_imu" / "ros2_adapter" / "parse_result.py"


def test_parser_uses_refined_homogeneous_transform_from_repository_fixture(
    caplog: pytest.LogCaptureFixture,
) -> None:
    result = parse_initialization_result(str(REAL_LI_INIT_RESULT))

    assert result["r_il"] == pytest.approx(
        [
            0.032476,
            -0.999468,
            -0.002884,
            0.999154,
            0.032539,
            -0.025158,
            0.025239,
            -0.002065,
            0.999679,
        ]
    )
    assert result["t_il"] == pytest.approx([-0.050233, 0.032472, 0.161842])
    assert result["gyr_bias"] == pytest.approx([0.002907, 0.001678, 0.005512])
    assert result["acc_bias"] == pytest.approx([0.008483, -0.007028, 0.015659])
    assert result["gravity"] == pytest.approx([-1.143729, 0.370277, -9.749325])
    assert "Could not parse rotation matrix" not in caplog.text


def test_parser_preserves_legacy_matrix_and_label_format(tmp_path: Path) -> None:
    legacy_result = tmp_path / "legacy_result.txt"
    legacy_result.write_text(
        """Rotation LiDAR to IMU =
1.0 0.0 0.0
0.0 1.0 0.0
0.0 0.0 1.0
Translation LiDAR to IMU =
-0.011 -0.023 0.044
Time offset (lidar to imu) = 0.001
Accelerometer bias = 0.01 0.02 -0.03
Gyroscope bias = 0.001 -0.002 0.003
Gravity = 0.0 0.0 -9.81
""",
        encoding="utf-8",
    )

    result = parse_initialization_result(str(legacy_result))

    assert result["r_il"] == pytest.approx([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    assert result["t_il"] == pytest.approx([-0.011, -0.023, 0.044])
    assert result["time_offset"] == pytest.approx(0.001)
    assert result["acc_bias"] == pytest.approx([0.01, 0.02, -0.03])
    assert result["gyr_bias"] == pytest.approx([0.001, -0.002, 0.003])
    assert result["gravity"] == pytest.approx([0.0, 0.0, -9.81])


@pytest.mark.parametrize(
    ("content", "missing_field"),
    (
        (
            """Translation LiDAR to IMU = 0.1 0.2 0.3
Time offset (lidar to imu) = 0.001
""",
            "r_il",
        ),
        (
            """Rotation LiDAR to IMU =
1.0 0.0 0.0
0.0 1.0 0.0
0.0 0.0 1.0
""",
            "t_il",
        ),
    ),
)
def test_cli_fails_before_writing_when_required_field_is_missing(
    tmp_path: Path,
    content: str,
    missing_field: str,
) -> None:
    incomplete_result = tmp_path / "incomplete_result.txt"
    incomplete_result.write_text(
        content,
        encoding="utf-8",
    )
    output = tmp_path / "should_not_exist.yaml"

    completed = subprocess.run(  # noqa: S603 - executes the repository parser with this interpreter
        [
            sys.executable,
            str(PARSER),
            "--input",
            str(incomplete_result),
            "--output",
            str(output),
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        check=False,
        text=True,
    )

    assert completed.returncode != 0
    assert missing_field in completed.stderr
    assert not output.exists()
