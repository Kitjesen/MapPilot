from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
import yaml
from tools.calibration import apply_calibration, verify


def _write_yaml(path: Path, payload: dict) -> None:
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def _read_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_camera_lidar_apply_accepts_direct_visual_results_and_preserves_fixed_axis_rpy(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    robot_config = tmp_path / "robot_config.yaml"
    calibration_result = tmp_path / "calib.json"
    lidar_mount = {
        "offset_x": 0.402876074867229,
        "offset_y": 0.0,
        "offset_z": 0.0582019450665819,
        "roll": -np.pi,
        "pitch": -np.pi / 4,
        "yaw": 0.0,
    }
    _write_yaml(robot_config, {"lidar": lidar_mount, "camera": {"fx": 615.0}})
    calibration_result.write_text(
        json.dumps(
            {
                "results": {
                    "T_lidar_camera": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                }
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(apply_calibration, "ROBOT_CONFIG", robot_config)

    apply_calibration.apply_camera_lidar(str(calibration_result))

    camera = _read_yaml(robot_config)["camera"]
    assert [camera["position_x"], camera["position_y"], camera["position_z"]] == pytest.approx(
        [lidar_mount["offset_x"], lidar_mount["offset_y"], lidar_mount["offset_z"]],
        abs=1e-5,
    )
    written_rotation = verify._rotation_from_rpy(camera["roll"], camera["pitch"], camera["yaw"])
    expected_rotation = verify._rotation_from_rpy(
        lidar_mount["roll"],
        lidar_mount["pitch"],
        lidar_mount["yaw"],
    )
    assert written_rotation == pytest.approx(expected_rotation, abs=1e-6)


def test_camera_lidar_apply_accepts_top_level_flat_matrix_and_composes_rotation(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    robot_config = tmp_path / "robot_config.yaml"
    calibration_result = tmp_path / "calib.json"
    lidar_mount = {
        "offset_x": 0.4,
        "offset_y": -0.05,
        "offset_z": 0.1,
        "roll": -0.3,
        "pitch": 0.2,
        "yaw": -0.4,
    }
    lidar_camera_rotation = verify._rotation_from_rpy(0.25, -0.15, 0.35)
    lidar_camera_translation = np.asarray([0.1, -0.2, 0.05])
    lidar_camera_transform = np.eye(4)
    lidar_camera_transform[:3, :3] = lidar_camera_rotation
    lidar_camera_transform[:3, 3] = lidar_camera_translation
    _write_yaml(robot_config, {"lidar": lidar_mount, "camera": {}})
    calibration_result.write_text(
        json.dumps({"T_lidar_camera": lidar_camera_transform.reshape(-1).tolist()}),
        encoding="utf-8",
    )
    monkeypatch.setattr(apply_calibration, "ROBOT_CONFIG", robot_config)

    apply_calibration.apply_camera_lidar(str(calibration_result))

    camera = _read_yaml(robot_config)["camera"]
    body_lidar_rotation = verify._rotation_from_rpy(
        lidar_mount["roll"],
        lidar_mount["pitch"],
        lidar_mount["yaw"],
    )
    expected_rotation = body_lidar_rotation @ lidar_camera_rotation
    expected_translation = (
        np.asarray([lidar_mount["offset_x"], lidar_mount["offset_y"], lidar_mount["offset_z"]])
        + body_lidar_rotation @ lidar_camera_translation
    )
    written_rotation = verify._rotation_from_rpy(camera["roll"], camera["pitch"], camera["yaw"])
    assert written_rotation == pytest.approx(expected_rotation, abs=1e-6)
    assert [camera["position_x"], camera["position_y"], camera["position_z"]] == pytest.approx(
        expected_translation,
        abs=1e-5,
    )


def test_camera_lidar_apply_rejects_non_rigid_matrix_without_writing(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    caplog: pytest.LogCaptureFixture,
) -> None:
    robot_config = tmp_path / "robot_config.yaml"
    calibration_result = tmp_path / "calib.json"
    _write_yaml(
        robot_config,
        {
            "lidar": {
                "offset_x": 0.4,
                "offset_y": 0.0,
                "offset_z": 0.1,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            },
            "camera": {"position_x": 123.0},
        },
    )
    invalid_transform = np.eye(4)
    invalid_transform[0, 0] = 2.0
    calibration_result.write_text(
        json.dumps({"T_lidar_camera": invalid_transform.reshape(-1).tolist()}),
        encoding="utf-8",
    )
    monkeypatch.setattr(apply_calibration, "ROBOT_CONFIG", robot_config)
    before = robot_config.read_bytes()

    apply_calibration.apply_camera_lidar(str(calibration_result))

    assert robot_config.read_bytes() == before
    assert "rigid transform" in caplog.text


@pytest.mark.parametrize(
    ("payload", "expected_error"),
    [
        ({"results": [{"T_lidar_camera": [0.0] * 7}]}, "must be a JSON object"),
        ({"T_lidar_camera": [0.0] * 7}, "quaternion"),
    ],
)
def test_camera_lidar_apply_rejects_malformed_results_without_writing(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    caplog: pytest.LogCaptureFixture,
    payload: dict,
    expected_error: str,
) -> None:
    robot_config = tmp_path / "robot_config.yaml"
    calibration_result = tmp_path / "calib.json"
    _write_yaml(robot_config, {"lidar": {}, "camera": {"position_x": 123.0}})
    calibration_result.write_text(json.dumps(payload), encoding="utf-8")
    monkeypatch.setattr(apply_calibration, "ROBOT_CONFIG", robot_config)
    before = robot_config.read_bytes()

    apply_calibration.apply_camera_lidar(str(calibration_result))

    assert robot_config.read_bytes() == before
    assert expected_error in caplog.text


def test_lidar_imu_apply_preserves_body_mount_and_updates_body_from_imu(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    robot_config = tmp_path / "robot_config.yaml"
    fastlio_config = tmp_path / "mid360_fastlio2.yaml"
    calibration_result = tmp_path / "lidar_imu.yaml"

    body_lidar = {
        "offset_x": 1.0,
        "offset_y": 2.0,
        "offset_z": 3.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
    }
    _write_yaml(robot_config, {"lidar": body_lidar})
    _write_yaml(
        fastlio_config,
        {
            "r_il": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "t_il": [0.0, 0.0, 0.0],
            "navigation_body_from_imu_rotation": [
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
            "navigation_body_from_imu_translation": [1.0, 2.0, 3.0],
        },
    )
    _write_yaml(
        calibration_result,
        {
            "r_il": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "t_il": [0.1, -0.2, 0.3],
            "time_offset": 0.004,
        },
    )

    monkeypatch.setattr(apply_calibration, "ROBOT_CONFIG", robot_config)
    monkeypatch.setattr(apply_calibration, "FASTLIO2_CONFIG", fastlio_config)
    replace_calls: list[tuple[Path, Path]] = []
    original_replace = apply_calibration.os.replace

    def _track_replace(source: str | Path, target: str | Path) -> None:
        replace_calls.append((Path(source), Path(target)))
        original_replace(source, target)

    monkeypatch.setattr(apply_calibration.os, "replace", _track_replace)

    apply_calibration.apply_lidar_imu(str(calibration_result))

    assert _read_yaml(robot_config)["lidar"] == body_lidar
    fastlio = _read_yaml(fastlio_config)
    assert fastlio["t_il"] == pytest.approx([0.1, -0.2, 0.3])
    assert fastlio["navigation_body_from_imu_translation"] == pytest.approx([0.9, 2.2, 2.7])
    assert np.asarray(fastlio["navigation_body_from_imu_rotation"]).reshape(3, 3) == pytest.approx(np.eye(3))
    assert len(replace_calls) == 1
    temporary, target = replace_calls[0]
    assert temporary.parent == target.parent == fastlio_config.parent
    assert target == fastlio_config
    assert temporary.name.startswith(f".{fastlio_config.name}.")


def test_imu_noise_apply_updates_only_fastlio2(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    fastlio_config = tmp_path / "mid360_fastlio2.yaml"
    legacy_config = tmp_path / "pointlio.yaml"
    calibration_result = tmp_path / "imu.yaml"
    _write_yaml(
        fastlio_config,
        {"na": 0.01, "ng": 0.01, "nba": 0.0001, "nbg": 0.0001},
    )
    legacy_config.write_bytes(b"\xff\xfelegacy")
    _write_yaml(
        calibration_result,
        {
            "accelerometer_noise_density": 0.001,
            "gyroscope_noise_density": 0.0001,
            "accelerometer_random_walk": 0.00005,
            "gyroscope_random_walk": 0.000001,
        },
    )
    monkeypatch.setattr(apply_calibration, "FASTLIO2_CONFIG", fastlio_config)
    monkeypatch.setattr(apply_calibration, "POINTLIO_CONFIG", legacy_config, raising=False)
    legacy_before = legacy_config.read_bytes()

    apply_calibration.apply_imu_noise(str(calibration_result))

    updated = _read_yaml(fastlio_config)
    assert updated == {
        "na": pytest.approx(0.001),
        "ng": pytest.approx(0.0001),
        "nba": pytest.approx(0.00005),
        "nbg": pytest.approx(0.000001),
    }
    assert legacy_config.read_bytes() == legacy_before


def test_verify_accepts_composed_body_imu_lidar_chain(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    fastlio_config = tmp_path / "mid360_fastlio2.yaml"
    _write_yaml(
        fastlio_config,
        {
            "r_il": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "t_il": [0.1, -0.2, 0.3],
            "navigation_body_from_imu_rotation": [
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
            "navigation_body_from_imu_translation": [0.9, 2.2, 2.7],
        },
    )
    monkeypatch.setattr(verify, "FASTLIO2_CONFIG", fastlio_config)
    result = verify.CheckResult()

    verify.check_consistency(
        {
            "lidar": {
                "offset_x": 1.0,
                "offset_y": 2.0,
                "offset_z": 3.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            }
        },
        result,
    )

    assert result.failed == 0
    assert result.warned == 0
    assert result.passed == 1


def test_verify_rejects_body_mount_chain_mismatch(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    fastlio_config = tmp_path / "mid360_fastlio2.yaml"
    _write_yaml(
        fastlio_config,
        {
            "r_il": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            "t_il": [0.1, 0.0, 0.0],
            "navigation_body_from_imu_rotation": [
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
            "navigation_body_from_imu_translation": [0.0, 0.0, 0.0],
        },
    )
    monkeypatch.setattr(verify, "FASTLIO2_CONFIG", fastlio_config)
    result = verify.CheckResult()

    verify.check_consistency(
        {
            "lidar": {
                "offset_x": 1.0,
                "offset_y": 0.0,
                "offset_z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            }
        },
        result,
    )

    assert result.failed == 1
