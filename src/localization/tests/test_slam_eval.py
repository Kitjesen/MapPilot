from io import StringIO
from pathlib import Path

import pytest

from runtime.msgs.geometry import Pose, Quaternion, Vector3
from runtime.msgs.nav import Odometry
from sim.evaluation.slam import (
    TumPose,
    associate_by_timestamp,
    evaluate_trajectory,
    load_case,
    read_tum_trajectory,
    write_tum_trajectory,
)


class _NonClosingStringIO(StringIO):
    def close(self):
        pass


def test_tum_roundtrip(monkeypatch):
    poses = [
        TumPose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        TumPose(2.0, 1.0, 0.2, 0.0, 0.0, 0.0, 0.1, 0.995),
    ]
    written = _NonClosingStringIO()

    def fake_mkdir(self, parents=False, exist_ok=False):
        return None

    def fake_open(self, mode="r", encoding=None, newline=None):
        if "w" in mode:
            return written
        if "r" in mode:
            return _NonClosingStringIO(written.getvalue())
        raise AssertionError(f"unexpected open mode: {mode}")

    monkeypatch.setattr(Path, "mkdir", fake_mkdir)
    monkeypatch.setattr(Path, "open", fake_open)

    path = Path("trajectory.tum")
    write_tum_trajectory(path, poses)
    loaded = read_tum_trajectory(path)

    assert "1.000000000" in written.getvalue()
    assert len(loaded) == 2
    assert loaded[0].timestamp == pytest.approx(1.0)
    assert loaded[1].x == pytest.approx(1.0)


def test_tum_from_odometry():
    odometry = Odometry(
        pose=Pose(Vector3(1.0, 2.0, 0.1), Quaternion.from_yaw(0.5)),
        ts=12.5,
        frame_id="odom",
    )

    pose = TumPose.from_odometry(odometry)

    assert pose.timestamp == pytest.approx(12.5)
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(2.0)
    assert pose.frame_id == "odom"


def test_trajectory_metrics_translation_error():
    reference = [
        TumPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        TumPose(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        TumPose(2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    ]
    estimate = [
        TumPose(0.005, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 1.0),
        TumPose(1.005, 1.0, 0.1, 0.0, 0.0, 0.0, 0.0, 1.0),
        TumPose(2.005, 2.0, 0.1, 0.0, 0.0, 0.0, 0.0, 1.0),
    ]

    summary = evaluate_trajectory(reference, estimate, max_dt=0.02)

    assert summary.matched_count == 3
    assert summary.coverage_ratio == pytest.approx(1.0)
    assert summary.mean_translation_error_m == pytest.approx(0.1)
    assert summary.rmse_translation_error_m == pytest.approx(0.1)
    assert summary.max_translation_error_m == pytest.approx(0.1)
    assert summary.path_length_reference_m == pytest.approx(2.0)
    assert summary.drift_per_meter == pytest.approx(0.05)


def test_association_respects_time_gate():
    reference = [
        TumPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        TumPose(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    ]
    estimate = [
        TumPose(0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        TumPose(1.20, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    ]

    matches = associate_by_timestamp(reference, estimate, max_dt=0.02)

    assert len(matches) == 1
    assert matches[0].reference.timestamp == pytest.approx(0.0)


def test_manifest_baseline_case_validates():
    case = load_case("sim/evaluation/slam/configs/nova_dog_fastlio2.json")

    assert case.name == "nova_dog_factory_fastlio2"
    assert case.robot.name == "nova_dog"
    assert case.backend.name == "fastlio2"
    assert case.validate(repo_root=".") == []
