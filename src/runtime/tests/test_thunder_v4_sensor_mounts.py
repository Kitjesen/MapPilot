from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from drivers.real.camera.module import _default_camera_config
from runtime.runtime_interface import LIDAR_EXTRINSICS

ROOT = Path(__file__).resolve().parents[3]
V4_LIDAR1_XYZ = (
    0.402876074867229,
    0.0,
    0.0582019450665819,
)
V4_LIDAR1_RPY = (
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


def test_real_mid360_contract_uses_thunder_v4_front_lidar_mount() -> None:
    robot = yaml.safe_load((ROOT / "config" / "robot_config.yaml").read_text(encoding="utf-8"))
    lidar = robot["lidar"]
    authoritative_xyz = (lidar["offset_x"], lidar["offset_y"], lidar["offset_z"])
    authoritative_rpy = (lidar["roll"], lidar["pitch"], lidar["yaw"])
    assert authoritative_xyz == pytest.approx(V4_LIDAR1_XYZ)
    assert authoritative_rpy == pytest.approx(V4_LIDAR1_RPY)

    mount = LIDAR_EXTRINSICS["real_mid360"]

    assert (mount.x, mount.y, mount.z) == pytest.approx(authoritative_xyz)
    assert (mount.roll, mount.pitch, mount.yaw) == pytest.approx(authoritative_rpy)

    topic_text = (ROOT / "config" / "topic_contract.yaml").read_text(encoding="utf-8")
    profile_text = topic_text.split("  body_to_lidar_profiles:\n", 1)[1]
    profile_text = profile_text.split("    real_mid360:\n", 1)[1]
    profile_text = profile_text.split("    gazebo_proxy:\n", 1)[0]
    declared = yaml.safe_load("real_mid360:\n" + profile_text)["real_mid360"]
    assert (declared["x"], declared["y"], declared["z"]) == pytest.approx(authoritative_xyz)
    assert (declared["roll"], declared["pitch"], declared["yaw"]) == pytest.approx(authoritative_rpy)


def test_s100p_body_imu_transform_reconstructs_v4_lidar_mount() -> None:
    robot = yaml.safe_load((ROOT / "config" / "robot_config.yaml").read_text(encoding="utf-8"))
    lidar = robot["lidar"]
    authoritative_xyz = [lidar["offset_x"], lidar["offset_y"], lidar["offset_z"]]
    config = yaml.safe_load((ROOT / "src" / "localization" / "fastlio2" / "config" / "mid360_s100p.yaml").read_text())
    rotation_body_imu = _matrix(config["navigation_body_from_imu_rotation"])
    translation_body_imu = config["navigation_body_from_imu_translation"]
    translation_imu_lidar = config["t_il"]

    rotated_internal_offset = _mat_vec(rotation_body_imu, translation_imu_lidar)
    reconstructed_body_lidar = [translation_body_imu[index] + rotated_internal_offset[index] for index in range(3)]

    assert reconstructed_body_lidar == pytest.approx(authoritative_xyz)
    assert config["r_il"] == pytest.approx([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])

    # A level robot with this V4 mount sees gravity at roughly +45/-45 degrees
    # in the sensor frame, matching the live MID-360 stationary sample.
    gravity_in_imu = [rotation_body_imu[2][column] for column in range(3)]
    assert gravity_in_imu == pytest.approx([math.sqrt(0.5), 0.0, -math.sqrt(0.5)], abs=1e-12)


def test_robot_config_uses_v4_front_camera_and_flips_current_view() -> None:
    config = yaml.safe_load((ROOT / "config" / "robot_config.yaml").read_text(encoding="utf-8"))
    camera = config["camera"]

    assert (
        camera["position_x"],
        camera["position_y"],
        camera["position_z"],
    ) == pytest.approx(V4_CAMERA1_XYZ)
    assert (camera["roll"], camera["pitch"], camera["yaw"]) == pytest.approx(V4_CAMERA1_RPY)
    assert camera["rotate"] == 180

    devices = yaml.safe_load((ROOT / "config" / "devices.yaml").read_text(encoding="utf-8"))["devices"]
    camera_device = next(device for device in devices if device.get("id") == "orbbec_gemini335_main")
    assert "rotate" not in camera_device["config"]
    assert _default_camera_config()["rotate"] == camera["rotate"]
