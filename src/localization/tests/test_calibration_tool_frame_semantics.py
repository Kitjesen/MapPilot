from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml
from calibration import apply_calibration, verify


def _write_yaml(path: Path, payload: dict) -> None:
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def _read_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_lidar_imu_apply_preserves_body_mount_and_updates_body_from_imu(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    robot_config = tmp_path / "robot_config.yaml"
    fastlio_config = tmp_path / "mid360_s100p.yaml"
    pointlio_config = tmp_path / "missing_pointlio.yaml"
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
    monkeypatch.setattr(apply_calibration, "POINTLIO_CONFIG", pointlio_config)
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


@pytest.mark.parametrize(
    "invalid_pointlio",
    [
        b"\xff\xfeinvalid utf-8",
        b"/**:\n  ros__parameters: [unterminated",
    ],
    ids=["unreadable-utf8", "malformed-yaml"],
)
def test_lidar_imu_apply_is_all_or_nothing_when_pointlio_is_invalid(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    invalid_pointlio: bytes,
) -> None:
    robot_config = tmp_path / "robot_config.yaml"
    fastlio_config = tmp_path / "mid360_s100p.yaml"
    pointlio_config = tmp_path / "pointlio.yaml"
    calibration_result = tmp_path / "lidar_imu.yaml"

    _write_yaml(
        robot_config,
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
    )
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
    pointlio_config.write_bytes(invalid_pointlio)
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
    monkeypatch.setattr(apply_calibration, "POINTLIO_CONFIG", pointlio_config)

    robot_before = robot_config.read_bytes()
    fastlio_before = fastlio_config.read_bytes()

    apply_calibration.apply_lidar_imu(str(calibration_result))

    assert robot_config.read_bytes() == robot_before
    assert fastlio_config.read_bytes() == fastlio_before


def test_yaml_batch_rolls_back_prior_file_when_later_write_fails(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    first = tmp_path / "fastlio.yaml"
    second = tmp_path / "pointlio.yaml"
    _write_yaml(first, {"value": "first-before"})
    _write_yaml(second, {"value": "second-before"})
    first_before = first.read_bytes()
    second_before = second.read_bytes()
    original_save = apply_calibration.save_yaml

    def _fail_second(path: Path, data: dict) -> None:
        if path == second:
            raise OSError("injected second-file failure")
        original_save(path, data)

    monkeypatch.setattr(apply_calibration, "save_yaml", _fail_second)

    with pytest.raises(OSError, match="second-file failure"):
        apply_calibration.save_yaml_batch([(first, {"value": "first-after"}), (second, {"value": "second-after"})])

    assert first.read_bytes() == first_before
    assert second.read_bytes() == second_before


def test_verify_accepts_composed_body_imu_lidar_chain(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    fastlio_config = tmp_path / "mid360_s100p.yaml"
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
    fastlio_config = tmp_path / "mid360_s100p.yaml"
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
