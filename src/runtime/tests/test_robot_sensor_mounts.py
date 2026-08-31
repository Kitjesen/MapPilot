"""Regression checks for active robot sensor mounting profiles."""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from drivers.real.camera.module import _default_camera_config
from runtime.runtime_interface import LIDAR_EXTRINSICS

ROOT = Path(__file__).resolve().parents[3]
GO2_OFFICIAL_MID360_XYZ = (
    0.16143,
    0.0,
    0.12262,
)
GO2_OFFICIAL_MID360_RPY = (
    0.0,
    math.radians(13.0),
    0.0,
)
THUNDER_V4_MID360_XYZ = (
    0.402876074867229,
    0.0,
    0.0582019450665819,
)
THUNDER_V4_MID360_RPY = (
    -math.pi,
    -math.pi / 4.0,
    0.0,
)
V4_CAMERA1_XYZ = (
    0.423358800364963,
    -0.000496202974186816,
    0.11370714960317,
)
V4_CAMERA1_RPY = (
    -1.91986217719376,
    0.0,
    -1.57079632501758,
)


def _matrix(values: list[float]) -> list[list[float]]:
    assert len(values) == 9
    return [values[0:3], values[3:6], values[6:9]]


def _mat_vec(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3)]


def test_mid360_contract_keeps_go2_and_thunder_mounts_distinct() -> None:
    robot = yaml.safe_load(
        (ROOT / "config" / "robots" / "unitree" / "go2" / "robot.yaml").read_text(
            encoding="utf-8"
        )
    )
    lidar = robot["lidar"]
    authoritative_xyz = (lidar["offset_x"], lidar["offset_y"], lidar["offset_z"])
    authoritative_rpy = (lidar["roll"], lidar["pitch"], lidar["yaw"])
    assert authoritative_xyz == pytest.approx(GO2_OFFICIAL_MID360_XYZ)
    assert authoritative_rpy == pytest.approx(GO2_OFFICIAL_MID360_RPY)

    expected = {
        "go2_mid360": (GO2_OFFICIAL_MID360_XYZ, GO2_OFFICIAL_MID360_RPY),
        "thunder_v4_mid360": (THUNDER_V4_MID360_XYZ, THUNDER_V4_MID360_RPY),
    }
    for profile, (xyz, rpy) in expected.items():
        mount = LIDAR_EXTRINSICS[profile]
        assert (mount.x, mount.y, mount.z) == pytest.approx(xyz)
        assert (mount.roll, mount.pitch, mount.yaw) == pytest.approx(rpy)

    devices = yaml.safe_load((ROOT / "config" / "devices.yaml").read_text(encoding="utf-8"))["devices"]
    mid360 = next(device for device in devices if device.get("id") == "livox_mid360_main")
    assert mid360["network"]["ip"] == lidar["lidar_ip"] == "192.168.123.20"
    assert lidar["host_ip"] == "192.168.123.18"


@pytest.mark.parametrize(
    ("model_id", "expected_xyz"),
    (
        ("unitree/go2", GO2_OFFICIAL_MID360_XYZ),
        ("doso/thunder_v4", THUNDER_V4_MID360_XYZ),
    ),
)
def test_robot_specific_fastlio_config_reconstructs_its_lidar_mount(
    model_id: str,
    expected_xyz: tuple[float, float, float],
) -> None:
    model_dir = ROOT / "config" / "robots" / model_id
    model = yaml.safe_load(
        (model_dir / "model.yaml").read_text(encoding="utf-8")
    )
    config = yaml.safe_load(
        (model_dir / model["sensors"]["mid360"]["fastlio2_config"]).read_text(
            encoding="utf-8"
        )
    )
    rotation_body_imu = _matrix(config["navigation_body_from_imu_rotation"])
    translation_body_imu = config["navigation_body_from_imu_translation"]
    rotated_internal_offset = _mat_vec(rotation_body_imu, config["t_il"])
    reconstructed_body_lidar = [
        translation_body_imu[index] + rotated_internal_offset[index]
        for index in range(3)
    ]

    assert reconstructed_body_lidar == pytest.approx(expected_xyz)
    assert config["r_il"] == pytest.approx(
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    )


def test_go2_config_does_not_claim_thunder_camera_calibration() -> None:
    config = yaml.safe_load(
        (ROOT / "config" / "robots" / "unitree" / "go2" / "robot.yaml").read_text(
            encoding="utf-8"
        )
    )

    assert "camera" not in config
    assert "rotate" not in _default_camera_config()
