from __future__ import annotations

import logging
import subprocess
import sys
from math import pi
from pathlib import Path

import pytest
import yaml
from tools.calibration import verify


def _write_yaml(path: Path, payload: dict) -> None:
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def test_pointlio_noise_is_read_from_ros_mapping(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    caplog: pytest.LogCaptureFixture,
) -> None:
    pointlio_config = tmp_path / "pointlio.yaml"
    _write_yaml(
        pointlio_config,
        {
            "/**": {
                "ros__parameters": {
                    "mapping": {
                        "imu_meas_acc_cov": 0.123,
                        "imu_meas_omg_cov": 0.456,
                    }
                }
            }
        },
    )
    monkeypatch.setattr(verify, "FASTLIO2_CONFIG", tmp_path / "missing-fastlio.yaml")
    monkeypatch.setattr(verify, "POINTLIO_CONFIG", pointlio_config)
    result = verify.CheckResult()

    with caplog.at_level(logging.INFO, logger=verify.__name__):
        verify.check_imu(result, verbose=False)

    assert result.failed == 0
    assert result.passed == 1
    assert "Point-LIO: acc_cov=0.123, omg_cov=0.456" in caplog.text


def test_cli_reports_invalid_utf8_without_traceback(tmp_path: Path) -> None:
    config = tmp_path / "robot_config.yaml"
    config.write_bytes(b"camera:\n  model: \xff\n")

    completed = subprocess.run(  # noqa: S603 - runs the repository-owned CLI.
        [sys.executable, str(Path(verify.__file__)), "--config", str(config)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=False,
    )
    output = completed.stdout + completed.stderr

    assert completed.returncode == 1
    assert f"Failed to read YAML config {config}" in output
    assert "not valid UTF-8" in output
    assert "Traceback" not in output


def test_cli_reports_malformed_yaml_without_traceback(tmp_path: Path) -> None:
    config = tmp_path / "robot_config.yaml"
    config.write_text("camera: [unterminated\n", encoding="utf-8")

    completed = subprocess.run(  # noqa: S603 - runs the repository-owned CLI.
        [sys.executable, str(Path(verify.__file__)), "--config", str(config)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=False,
    )
    output = completed.stdout + completed.stderr

    assert completed.returncode == 1
    assert f"Failed to parse YAML config {config}" in output
    assert "Traceback" not in output


def test_projection_uses_body_forward_for_rotated_v4_lidar() -> None:
    cfg = {
        "lidar": {
            "offset_x": 0.402876074867229,
            "offset_y": 0.0,
            "offset_z": 0.0582019450665819,
            "roll": -pi,
            "pitch": -pi / 4,
            "yaw": 0.0,
        },
        "camera": {
            "position_x": 0.423358800364963,
            "position_y": -0.000496202974186816,
            "position_z": 0.11370714960317,
            "roll": -1.91986217719376,
            "pitch": 0.0,
            "yaw": -1.57079632501758,
            "fx": 615.0,
            "fy": 615.0,
            "cx": 320.0,
            "cy": 240.0,
            "width": 640,
            "height": 480,
        },
    }
    result = verify.CheckResult()

    verify.check_lidar_camera_projection(cfg, result, verbose=False)

    assert result.failed == 0
    assert result.passed == 2
