from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

from sim.evaluation.slam.inspection_readiness import (
    FAIL,
    INCOMPLETE,
    PASS,
    EvidenceWindow,
    InspectionLocalizationReadinessConfig,
    RelocalizationRecoveryWindow,
    evaluate_inspection_localization_readiness,
)

ROOT = Path(__file__).resolve().parents[2]
CLI_PATH = ROOT / "tools" / "datasets" / "inspection_localization_readiness.py"
CLI_SPEC = importlib.util.spec_from_file_location(
    "inspection_localization_readiness_cli",
    CLI_PATH,
)
assert CLI_SPEC is not None and CLI_SPEC.loader is not None
CLI = importlib.util.module_from_spec(CLI_SPEC)
CLI_SPEC.loader.exec_module(CLI)


def _pose(yaw_deg: float = 0.0) -> dict[str, float]:
    half = math.radians(yaw_deg) / 2.0
    return {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": math.sin(half),
        "qw": math.cos(half),
    }


def _sample(stamp_s: float, sequence: int, *, yaw_deg: float = 0.0, **overrides):
    sample = {
        "stamp_s": stamp_s,
        "runtime_instance_id": "slam-runtime-1",
        "state": "TRACKING",
        "reported_state": "TRACKING",
        "observation_sequence": sequence,
        "map_frame_jump": False,
        "dropped_lidar_frames": 0,
        "dropped_imu_frames": 0,
        "imu_rollback_count": 0,
        "lidar_rollback_count": 0,
        "pose": _pose(yaw_deg),
        "fastlio_degeneracy": {"detected": False},
        "relocalization_state": "TRACKING",
    }
    sample.update(overrides)
    return sample


def test_complete_stationary_tracking_evidence_passes_without_authorizing_motion():
    samples = [_sample(float(i), i, yaw_deg=0.01 * i) for i in range(6)]

    report = evaluate_inspection_localization_readiness(
        samples,
        evidence_window=EvidenceWindow(0.0, 5.0),
        stationary_window=EvidenceWindow(0.0, 5.0),
        annotated_degeneracy_windows=[EvidenceWindow(2.0, 3.0)],
        config=InspectionLocalizationReadinessConfig(
            min_status_samples=6,
            min_stationary_duration_s=5.0,
            max_stationary_yaw_drift_deg_per_min=1.0,
            min_annotated_degeneracy_detection_rate=0.0,
        ),
    )

    assert report["status"] == "LOCALIZATION_PASS"
    assert report["localization_status"] == PASS
    assert report["patrol_readiness"]["status"] == INCOMPLETE
    assert report["patrol_readiness"]["reason"] == "motion_surfaces_not_evaluated"
    assert report["motion_authorization"] is False
    assert set(report["does_not_prove"]) == {
        "global_planner",
        "local_planner",
        "path_follower",
        "driver",
        "motor",
        "field_thermal",
        "camera_action",
    }


def test_missing_explicit_stationary_window_is_incomplete_not_pass():
    samples = [_sample(float(i), i) for i in range(3)]

    report = evaluate_inspection_localization_readiness(
        samples,
        evidence_window=EvidenceWindow(0.0, 2.0),
    )

    assert report["localization_status"] == INCOMPLETE
    assert report["checks"]["stationary_yaw_drift"]["status"] == INCOMPLETE


def test_stationary_yaw_drift_is_estimated_from_quaternion_slope():
    samples = [_sample(float(i), i, yaw_deg=0.02 * i) for i in range(61)]

    report = evaluate_inspection_localization_readiness(
        samples,
        evidence_window=EvidenceWindow(0.0, 60.0),
        stationary_window=EvidenceWindow(0.0, 60.0),
        config=InspectionLocalizationReadinessConfig(
            max_stationary_yaw_drift_deg_per_min=0.5,
        ),
    )

    assert report["localization_status"] == FAIL
    assert report["checks"]["stationary_yaw_drift"]["metrics"]["drift_deg_per_min"] == 1.2
    assert "stationary_yaw_drift: stationary yaw drift exceeds threshold" in report["blockers"]


def test_continuity_fails_on_runtime_split_sequence_gap_map_jump_and_bad_state():
    samples = [
        _sample(0.0, 0),
        _sample(1.0, 5, runtime_instance_id="slam-runtime-2"),
        _sample(2.0, 4, state="LOST"),
        _sample(3.0, 6, map_frame_jump=True),
        _sample(4.0, 7, pose={"x": math.nan}),
    ]

    report = evaluate_inspection_localization_readiness(
        samples,
        evidence_window=EvidenceWindow(0.0, 4.0),
        stationary_window=EvidenceWindow(0.0, 1.0),
    )

    continuity = report["checks"]["continuity"]
    assert report["localization_status"] == FAIL
    assert continuity["status"] == FAIL
    assert continuity["metrics"]["sequence_drop_count"] == 5
    assert continuity["metrics"]["sequence_rollback_count"] == 1
    assert "multiple runtime_instance_id values observed" in continuity["reasons"]
    assert "non-tracking localization states observed" in continuity["reasons"]
    assert "map_frame_jump observed" in continuity["reasons"]
    assert "non-finite or missing pose observed" in continuity["reasons"]


def test_annotated_degenerate_window_requires_detector_hits():
    samples = [_sample(float(i), i) for i in range(6)]

    report = evaluate_inspection_localization_readiness(
        samples,
        evidence_window=EvidenceWindow(0.0, 5.0),
        stationary_window=EvidenceWindow(0.0, 5.0),
        annotated_degeneracy_windows=[EvidenceWindow(2.0, 4.0)],
        config=InspectionLocalizationReadinessConfig(
            min_annotated_degeneracy_detection_rate=0.5,
        ),
    )

    assert report["localization_status"] == FAIL
    assert report["checks"]["degeneracy_diagnostics"]["status"] == FAIL
    assert report["checks"]["degeneracy_diagnostics"]["metrics"]["annotated_detection_rate"] == 0.0


def test_relocalization_window_requires_stable_tracking_tail():
    samples = [
        _sample(0.0, 0),
        _sample(1.0, 1, state="LOST", relocalization_state="SEARCHING"),
        _sample(2.0, 2, relocalization_state="TRACKING"),
        _sample(3.0, 3, relocalization_state="TRACKING"),
    ]

    report = evaluate_inspection_localization_readiness(
        samples,
        evidence_window=EvidenceWindow(0.0, 3.0),
        stationary_window=EvidenceWindow(2.0, 3.0),
        relocalization_windows=[
            RelocalizationRecoveryWindow(
                1.0,
                3.0,
                min_stable_s=2.0,
                initial_offset_m=2.0,
                initial_yaw_offset_rad=0.52,
                expected_map_sha256="map-a",
                observed_map_sha256="map-a",
            )
        ],
        config=InspectionLocalizationReadinessConfig(
            min_status_samples=4,
            min_stationary_duration_s=1.0,
        ),
    )

    assert report["localization_status"] == FAIL
    assert report["checks"]["relocalization_recovery"]["metrics"]["recoveries"] == [{"index": 0, "stable_s": 1.0}]


def test_raw_cpp_snapshot_shape_and_transport_counters_are_supported():
    samples = []
    for i in range(3):
        sample = _sample(float(i), i)
        sample["odometry"] = {"pose": sample.pop("pose")}
        sample.update(
            dropped_lidar_frames=0,
            dropped_imu_frames=0,
            imu_rollback_count=0,
            lidar_rollback_count=0,
        )
        samples.append(sample)

    report = evaluate_inspection_localization_readiness(
        samples,
        evidence_window=EvidenceWindow(0.0, 2.0),
        stationary_window=EvidenceWindow(0.0, 2.0),
        config=InspectionLocalizationReadinessConfig(
            min_status_samples=3,
            min_stationary_duration_s=2.0,
        ),
    )

    assert report["localization_status"] == PASS


def test_transport_drop_or_timestamp_rollback_counter_fails_continuity():
    samples = [_sample(float(i), i) for i in range(3)]
    samples[-1]["dropped_lidar_frames"] = 1
    samples[-1]["imu_rollback_count"] = 1

    report = evaluate_inspection_localization_readiness(
        samples,
        evidence_window=EvidenceWindow(0.0, 2.0),
        stationary_window=EvidenceWindow(0.0, 2.0),
        config=InspectionLocalizationReadinessConfig(
            min_status_samples=3,
            min_stationary_duration_s=2.0,
        ),
    )

    continuity = report["checks"]["continuity"]
    assert continuity["status"] == FAIL
    assert "sensor frame drops observed" in continuity["reasons"]
    assert "sensor timestamp rollback observed" in continuity["reasons"]


def test_annotated_relocalization_can_recover_without_failing_continuity():
    samples = [_sample(float(i), i) for i in range(7)]
    samples[1].update(state="LOST", relocalization_state="SEARCHING")

    report = evaluate_inspection_localization_readiness(
        samples,
        evidence_window=EvidenceWindow(0.0, 6.0),
        stationary_window=EvidenceWindow(2.0, 6.0),
        relocalization_windows=[
            RelocalizationRecoveryWindow(
                1.0,
                6.0,
                min_stable_s=4.0,
                initial_offset_m=2.0,
                initial_yaw_offset_rad=0.52,
                expected_map_sha256="map-a",
                observed_map_sha256="map-a",
            )
        ],
        config=InspectionLocalizationReadinessConfig(
            min_status_samples=7,
            min_stationary_duration_s=4.0,
        ),
    )

    assert report["localization_status"] == PASS


def test_zero_perturbation_relocalization_is_incomplete_not_pass():
    samples = [_sample(float(i), i) for i in range(7)]

    report = evaluate_inspection_localization_readiness(
        samples,
        evidence_window=EvidenceWindow(0.0, 6.0),
        stationary_window=EvidenceWindow(0.0, 6.0),
        relocalization_windows=[RelocalizationRecoveryWindow(1.0, 6.0, min_stable_s=4.0)],
        config=InspectionLocalizationReadinessConfig(
            min_status_samples=7,
            min_stationary_duration_s=6.0,
        ),
    )

    assert report["localization_status"] == INCOMPLETE
    assert "non-zero initial perturbation is required" in " ".join(
        report["checks"]["relocalization_recovery"]["reasons"]
    )


def _write_jsonl(path: Path, records: list[dict]) -> None:
    path.write_text(
        "".join(json.dumps(record) + "\n" for record in records),
        encoding="utf-8",
    )


def test_readiness_cli_writes_report_and_returns_pass(tmp_path: Path):
    snapshots = tmp_path / "slam_status.jsonl"
    report_path = tmp_path / "readiness.json"
    _write_jsonl(
        snapshots,
        [_sample(float(i), i, yaw_deg=0.001 * i) for i in range(6)],
    )

    rc = CLI.main(
        [
            str(snapshots),
            "--evidence",
            "0",
            "5",
            "--stationary",
            "0",
            "5",
            "--min-status-samples",
            "6",
            "--min-stationary-duration",
            "5",
            "--max-yaw-drift-deg-per-min",
            "0.5",
            "--write",
            str(report_path),
        ]
    )

    assert rc == 0
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["status"] == "LOCALIZATION_PASS"
    assert payload["localization_status"] == PASS
    assert payload["patrol_readiness"]["status"] == INCOMPLETE
    assert payload["motion_authorization"] is False


def test_readiness_cli_returns_incomplete_without_stationary_window(tmp_path: Path):
    snapshots = tmp_path / "slam_status.jsonl"
    _write_jsonl(snapshots, [_sample(float(i), i) for i in range(3)])

    rc = CLI.main(
        [
            str(snapshots),
            "--evidence",
            "0",
            "2",
            "--min-status-samples",
            "3",
        ]
    )

    assert rc == 2


def test_readiness_cli_can_use_captured_file_as_explicit_full_window(tmp_path: Path):
    snapshots = tmp_path / "slam_status.jsonl"
    _write_jsonl(
        snapshots,
        [_sample(float(i), i, yaw_deg=0.001 * i) for i in range(6)],
    )

    rc = CLI.main(
        [
            str(snapshots),
            "--full-evidence-window",
            "--stationary-full-window",
            "--min-status-samples",
            "6",
            "--min-stationary-duration",
            "5",
        ]
    )

    assert rc == 0


def test_readiness_cli_can_capture_atomic_snapshot_before_evaluation(
    tmp_path: Path,
    monkeypatch,
):
    source = tmp_path / "slam_status.json"
    snapshots = tmp_path / "slam_status.jsonl"
    report_path = tmp_path / "readiness.json"
    source.write_text(
        json.dumps(
            {
                **_sample(0.0, 0),
                "schema_version": "lingtu.slam.status_snapshot.v1",
            }
        ),
        encoding="utf-8",
    )

    def fake_capture(_source: Path, output: Path, **_kwargs) -> dict:
        _write_jsonl(output, [_sample(0.0, 0), _sample(1.0, 1)])
        return {
            "schema_version": "lingtu.slam.status_capture.v1",
            "status": "INCOMPLETE",
            "motion_authorization": False,
            "blockers": ["insufficient_observed_duration"],
            "unique_samples": 2,
        }

    monkeypatch.setattr(CLI, "capture_snapshot_history", fake_capture)

    rc = CLI.main(
        [
            str(snapshots),
            "--capture-source",
            str(source),
            "--capture-duration-s",
            "0",
            "--capture-min-samples",
            "1",
            "--capture-min-observed-duration-s",
            "0",
            "--full-evidence-window",
            "--stationary-full-window",
            "--min-status-samples",
            "2",
            "--min-stationary-duration",
            "1",
            "--write",
            str(report_path),
        ]
    )

    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert rc == 2
    assert report["capture"]["status"] == "INCOMPLETE"
    assert report["capture"]["unique_samples"] == 2
    assert "capture: insufficient_observed_duration" in report["blockers"]
    assert report["motion_authorization"] is False


def test_readiness_cli_accepts_relocalization_case_file(tmp_path: Path):
    snapshots = tmp_path / "slam_status.jsonl"
    case_path = tmp_path / "relocalization.json"
    _write_jsonl(
        snapshots,
        [
            _sample(0.0, 0),
            _sample(1.0, 1, state="LOST", relocalization_state="SEARCHING"),
            _sample(2.0, 2, relocalization_state="TRACKING"),
            _sample(3.0, 3, relocalization_state="TRACKING"),
            _sample(4.0, 4, relocalization_state="TRACKING"),
            _sample(5.0, 5, relocalization_state="TRACKING"),
        ],
    )
    case_path.write_text(
        json.dumps(
            {
                "relocalization_windows": [
                    {
                        "start_s": 1.0,
                        "end_s": 5.0,
                        "min_stable_s": 3.0,
                        "initial_offset_m": 1.5,
                        "initial_yaw_offset_rad": 0.5,
                        "expected_map_sha256": "map-a",
                        "observed_map_sha256": "map-a",
                    }
                ]
            }
        ),
        encoding="utf-8",
    )

    rc = CLI.main(
        [
            str(snapshots),
            "--evidence",
            "0",
            "5",
            "--stationary",
            "2",
            "5",
            "--min-status-samples",
            "6",
            "--min-stationary-duration",
            "3",
            "--relocalization-case",
            str(case_path),
        ]
    )

    assert rc == 0


def test_relocalization_stable_tail_requires_reported_tracking_state():
    samples = [
        _sample(0.0, 0),
        _sample(1.0, 1, state="LOST", reported_state="LOST", relocalization_state="SEARCHING"),
        _sample(2.0, 2, reported_state="LOST", relocalization_state="TRACKING"),
        _sample(3.0, 3, reported_state="LOST", relocalization_state="TRACKING"),
    ]

    report = evaluate_inspection_localization_readiness(
        samples,
        evidence_window=EvidenceWindow(0.0, 3.0),
        stationary_window=EvidenceWindow(2.0, 3.0),
        relocalization_windows=[
            RelocalizationRecoveryWindow(
                1.0,
                3.0,
                min_stable_s=1.0,
                initial_offset_m=1.0,
                expected_map_sha256="map-a",
                observed_map_sha256="map-a",
            )
        ],
        config=InspectionLocalizationReadinessConfig(
            min_status_samples=4,
            min_stationary_duration_s=1.0,
        ),
    )

    assert report["localization_status"] == FAIL
    assert "lacks stable tracking tail" in " ".join(report["checks"]["relocalization_recovery"]["reasons"])


def test_transport_counters_use_window_delta_not_lifetime_total():
    samples = [
        _sample(
            float(i),
            i,
            dropped_lidar_frames=5,
            dropped_imu_frames=2,
            imu_rollback_count=1,
            lidar_rollback_count=3,
        )
        for i in range(3)
    ]

    report = evaluate_inspection_localization_readiness(
        samples,
        evidence_window=EvidenceWindow(0.0, 2.0),
        stationary_window=EvidenceWindow(0.0, 2.0),
        config=InspectionLocalizationReadinessConfig(
            min_status_samples=3,
            min_stationary_duration_s=2.0,
        ),
    )

    assert report["localization_status"] == PASS
    continuity = report["checks"]["continuity"]
    assert continuity["metrics"]["sensor_drop_delta"] == 0
    assert continuity["metrics"]["sensor_rollback_delta"] == 0


def test_transport_counter_increase_or_reset_fails_continuity():
    increased = [_sample(float(i), i) for i in range(3)]
    increased[-1]["dropped_lidar_frames"] = 1
    reset = [_sample(float(i), i, dropped_imu_frames=4) for i in range(3)]
    reset[-1]["dropped_imu_frames"] = 0

    increased_report = evaluate_inspection_localization_readiness(
        increased,
        evidence_window=EvidenceWindow(0.0, 2.0),
        stationary_window=EvidenceWindow(0.0, 2.0),
        config=InspectionLocalizationReadinessConfig(
            min_status_samples=3,
            min_stationary_duration_s=2.0,
        ),
    )
    reset_report = evaluate_inspection_localization_readiness(
        reset,
        evidence_window=EvidenceWindow(0.0, 2.0),
        stationary_window=EvidenceWindow(0.0, 2.0),
        config=InspectionLocalizationReadinessConfig(
            min_status_samples=3,
            min_stationary_duration_s=2.0,
        ),
    )

    assert increased_report["localization_status"] == FAIL
    assert "sensor frame drops observed" in increased_report["checks"]["continuity"]["reasons"]
    assert reset_report["localization_status"] == FAIL
    assert "sensor transport counter rollback observed" in reset_report["checks"]["continuity"]["reasons"]


def test_status_timestamp_rollback_fails_continuity():
    samples = [_sample(0.0, 0), _sample(2.0, 1), _sample(1.0, 2)]

    report = evaluate_inspection_localization_readiness(
        samples,
        evidence_window=EvidenceWindow(0.0, 2.0),
        stationary_window=EvidenceWindow(0.0, 2.0),
        config=InspectionLocalizationReadinessConfig(
            min_status_samples=3,
            min_stationary_duration_s=1.0,
        ),
    )

    assert report["localization_status"] == FAIL
    assert "status timestamp rollback observed" in report["checks"]["continuity"]["reasons"]


def test_raw_cpp_relocalization_without_reported_state_can_prove_stable_tail():
    samples = [_sample(float(i), i) for i in range(6)]
    for sample in samples:
        sample.pop("reported_state")
    samples[1].update(state="LOST", relocalization_state="SEARCHING")

    report = evaluate_inspection_localization_readiness(
        samples,
        evidence_window=EvidenceWindow(0.0, 5.0),
        stationary_window=EvidenceWindow(2.0, 5.0),
        relocalization_windows=[
            RelocalizationRecoveryWindow(
                1.0,
                5.0,
                min_stable_s=3.0,
                initial_offset_m=1.0,
                expected_map_sha256="map-a",
                observed_map_sha256="map-a",
            )
        ],
        config=InspectionLocalizationReadinessConfig(
            min_status_samples=6,
            min_stationary_duration_s=3.0,
        ),
    )

    assert report["localization_status"] == PASS
