from __future__ import annotations

import json
from pathlib import Path

import pytest

from lingtu.real.backend import FieldBackend


def _snapshot() -> dict:
    return {
        "schema_version": "lingtu.slam.status_snapshot.v1",
        "runtime_instance_id": "slam-native-instance",
        "alive": True,
        "snapshot_written_at_s": 100.0,
        "mode": "localization",
        "state": "LOCALIZING",
        "reason": "waiting_for_relocalization",
        "has_odom": True,
        "map_loaded": True,
        "saved_map_points": 3836924,
        "registered_cloud_frame_id": "body",
        "saved_map_cloud_frame_id": "",
        "map_odom_tf": None,
        "map_odometry": None,
        "track_against_map": {"enabled": False, "successes": 0, "degraded": False},
    }


def _wait(tmp_path: Path, status: dict, *, require_localization: bool = False, mode: str = "localization", require_map: bool = True) -> None:
    status_path = tmp_path / "slam.json"
    status_path.write_text(json.dumps(status), encoding="utf-8")
    ticks = iter((0.0, 0.0, 1.0))
    FieldBackend(
        environment={"LINGTU_SLAM_STATUS_JSON": str(status_path)},
        monotonic=lambda: next(ticks), wall_clock=lambda: 100.1,
        sleep=lambda _seconds: None,
    ).wait_slam(mode, require_map=require_map, require_localization=require_localization, timeout_s=0.5)


def test_live_local_odometry_with_loaded_map_can_prepare_relocalization(tmp_path: Path) -> None:
    _wait(tmp_path, _snapshot())


def test_unaligned_local_odometry_is_not_ready_for_localized_motion(tmp_path: Path) -> None:
    with pytest.raises(RuntimeError, match="require_localization=True"):
        _wait(tmp_path, _snapshot(), require_localization=True)


@pytest.mark.parametrize("change", [
    {"has_odom": False}, {"has_odom": None}, {"map_loaded": False},
    {"alive": False}, {"runtime_instance_id": ""},
    {"snapshot_written_at_s": 90.0}, {"snapshot_written_at_s": 101.0},
    {"mode": "mapping"}, {"state": "INITIALIZING"}, {"state": "LOST"},
])
def test_pre_relocalization_still_requires_live_fresh_bound_frontend(tmp_path: Path, change: dict) -> None:
    with pytest.raises(RuntimeError, match="SLAM did not become ready"):
        _wait(tmp_path, {**_snapshot(), **change})


def test_localizing_state_is_not_a_mapping_readiness_shortcut(tmp_path: Path) -> None:
    with pytest.raises(RuntimeError, match="SLAM did not become ready"):
        _wait(tmp_path, {**_snapshot(), "mode": "mapping"}, mode="mapping", require_map=False)


def test_final_gate_requires_tracking_and_healthy_saved_map_alignment(tmp_path: Path) -> None:
    status = {**_snapshot(), "state": "TRACKING", "map_odom_tf": {"valid": True},
              "track_against_map": {"enabled": True, "successes": 1, "degraded": False}}
    _wait(tmp_path, status, require_localization=True)
    for change in [
        {"state": "LOCALIZING"}, {"map_odom_tf": None},
        {"track_against_map": {"enabled": False, "successes": 0, "degraded": False}},
        {"track_against_map": {"enabled": True, "successes": 1, "degraded": True}},
    ]:
        with pytest.raises(RuntimeError, match="require_localization=True"):
            _wait(tmp_path, {**status, **change}, require_localization=True)


def test_mapping_readiness_remains_independent_of_saved_map(tmp_path: Path) -> None:
    _wait(tmp_path, {**_snapshot(), "mode": "mapping", "state": "MAPPING", "map_loaded": False}, mode="mapping", require_map=False)
