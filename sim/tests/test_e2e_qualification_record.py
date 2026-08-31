# ruff: noqa: S101

from __future__ import annotations

import hashlib
import json
import os
import subprocess
from pathlib import Path
from typing import Any

import pytest

from sim.runtime.qualification import (
    QualificationRecordError,
    build_e2e_qualification_record,
)


def _write_json(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, sort_keys=True), encoding="utf-8")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _maneuver(
    name: str,
    *,
    linear_x: float = 0.0,
    linear_y: float = 0.0,
    angular_z: float = 0.0,
) -> dict[str, Any]:
    rotation = angular_z != 0.0
    translation = linear_x != 0.0 or linear_y != 0.0
    document: dict[str, Any] = {
        "name": name,
        "command": {"linear_x": linear_x, "linear_y": linear_y, "angular_z": angular_z},
        "expectation_met": True,
        "motion_verified": True,
        "rotation_commanded": rotation,
        "translation_commanded": translation,
        "translation_met": True,
        "signed_translation_m": 0.09 if translation else 0.0,
        "minimum_displacement_m": 0.08,
    }
    if rotation:
        document.update(
            {
                "signed_yaw_rad": 0.41,
                "horizontal_drift_m": 0.04,
                "minimum_rotation_rad": 0.35,
                "maximum_turn_drift_m": 0.1,
                "rotation_met": True,
                "turn_drift_met": True,
            }
        )
    else:
        document.update(
            {
                "signed_yaw_rad": 0.0,
                "horizontal_drift_m": 0.09,
                "minimum_rotation_rad": 0.35,
                "maximum_turn_drift_m": 0.1,
                "rotation_met": True,
                "turn_drift_met": True,
            }
        )
    return document


def _completed_run(tmp_path: Path) -> tuple[Path, Path]:
    bundle_dir = tmp_path / "bundle"
    run_dir = tmp_path / "run"
    session_id = "factory-six-motion"
    run_id = "factory-six-motion"
    bundle_dir.mkdir()
    run_dir.mkdir()
    _write_json(
        bundle_dir / "session.yaml",
        {
            "schema": "lingtu.sim.session.v1",
            "session_id": session_id,
            "runtime": {
                "required_bindings": ["physics", "visual", "sensors", "control"]
            },
        },
    )
    _write_json(
        bundle_dir / "transport.intent.json",
        {
            "schema": "lingtu.sim.transport-intent.v1",
            "session_id": session_id,
            "dds": {"domain": 79},
        },
    )
    (run_dir / "simulation-timeline.jsonl").write_text(
        '{"schema":"lingtu.sim.timeline-frame.v1","frame_index":0}\n',
        encoding="utf-8",
    )
    _write_json(
        run_dir / "simulation-recording.json",
        {
            "schema": "lingtu.sim.recording.v1",
            "run_id": run_id,
            "session_id": session_id,
            "model_generation": 0,
            "reset_generation": {"start": 0, "end": 0},
            "timeline": {
                "path": "simulation-timeline.jsonl",
                "bytes": (run_dir / "simulation-timeline.jsonl").stat().st_size,
                "frame_count": 1,
                "sha256": _sha256(run_dir / "simulation-timeline.jsonl"),
            },
        },
    )
    _write_json(
        run_dir / "run-allocation.json",
        {
            "schema": "lingtu.sim.run-allocation.v1",
            "run_id": run_id,
            "session_id": session_id,
            "dds_domain": 79,
            "ports": {"visual_snapshot_udp": 25565},
            "shm": {},
            "log_dir": str((run_dir / "logs").resolve()),
        },
    )
    _write_json(
        run_dir / "session.runtime.json",
        {
            "schema": "lingtu.sim.session-runtime.v1",
            "run_id": run_id,
            "session_id": session_id,
            "model_generation": 0,
            "reset_generation": 0,
            "state": "STOPPED",
            "allocation": {
                "run_dir": str(run_dir.resolve()),
                "log_dir": str((run_dir / "logs").resolve()),
                "dds_domain": 79,
                "ports": {"visual_snapshot_udp": 25565},
                "shm": {},
            },
            "bindings": {
                "physics": {"required": True, "state": "ACTIVE"},
                "visual": {"required": True, "state": "ACTIVE"},
                "sensors": {"required": True, "state": "ACTIVE"},
                "control": {"required": True, "state": "ACTIVE"},
            },
            "sensor_streams": {
                "is_ready": True,
                "required_stream_ids": ["thunder_01.truth_odom"],
                "summary": {
                    "schema": "lingtu.sim.sensor-stream-summary.v1",
                    "session_id": session_id,
                    "model_generation": 0,
                    "reset_generation": 0,
                    "streams": {
                        "thunder_01.truth_odom": {
                            "required": True,
                            "state": "ACTIVE",
                            "sample_count": 3,
                        }
                    },
                },
                "streams": {
                    "thunder_01.truth_odom": {
                        "required": True,
                        "state": "ACTIVE",
                        "stream_kind": "truth_odom",
                        "transport": "typed_dds",
                    }
                },
            },
        },
    )
    _write_json(
        run_dir / "episode_result.json",
        {
            "schema": "lingtu.sim.episode-result.v1",
            "run_id": run_id,
            "session_id": session_id,
            "model_generation": 0,
            "reset_generation": 0,
            "status": "SUCCEEDED",
            "artifact_references": {
                "runtime_manifest": "session.runtime.json",
                "run_allocation": "run-allocation.json",
                "simulation_recording": "simulation-recording.json",
                "simulation_timeline": "simulation-timeline.jsonl",
                "scenario_visual_evidence": "scenario-visual-evidence.json",
            },
        },
    )
    _write_json(
        run_dir / "scenario-visual-evidence.json",
        {
            "schema": "lingtu.sim.scenario-visual-evidence.v1",
            "source": "ue_registry_applied",
            "input_source": "canonical_scenario_snapshot",
            "basis": "snapshot_pose_applied_to_unreal_actor",
            "run_id": run_id,
            "session_id": session_id,
            "model_generation": 0,
            "reset_generation": 0,
            "sequence": 7,
            "sim_time_ns": 5_000_000_000,
            "position_tolerance_m": 0.02,
            "maximum_error_m": 0.01,
            "within_tolerance": True,
            "actors": [
                {
                    "entity_id": "pedestrian_01",
                    "expected": {"x": 4.0, "y": 0.0, "z": 0.0},
                    "observed": {"x": 4.0, "y": 0.01, "z": 0.0},
                    "error_m": 0.01,
                }
            ],
        },
    )
    _write_json(
        run_dir / "motion-evidence.json",
        {
            "schema": "lingtu.sim.motion-recording-evidence.v2",
            "run_id": run_id,
            "session_id": session_id,
            "acceptance_profile": {"name": "factory_park_motion", "version": 1},
            "runtime_manifest": str((run_dir / "session.runtime.json").resolve()),
            "recording_manifest": str((run_dir / "simulation-recording.json").resolve()),
            "recording_timeline": str((run_dir / "simulation-timeline.jsonl").resolve()),
            "frames": {"captured_count": 72, "minimum_frames": 60},
            "maneuvers": [
                _maneuver("forward", linear_x=0.1),
                _maneuver("backward", linear_x=-0.1),
                _maneuver("left", linear_y=0.1),
                _maneuver("right", linear_y=-0.1),
                _maneuver("turn_left", angular_z=0.45),
                _maneuver("turn_right", angular_z=-0.45),
            ],
        },
    )
    return bundle_dir, run_dir


def _add_scenario_dispatch_proof(bundle_dir: Path, run_dir: Path) -> None:
    session_id = "factory-six-motion"
    run_id = "factory-six-motion"
    _write_json(
        bundle_dir / "scenario.plan.json",
        {
            "schema": "lingtu.sim.scenario-plan.v1",
            "session_id": session_id,
            "model_generation": 0,
            "reset_generation": 0,
            "entities": [
                {
                    "entity_id": "pedestrian_01",
                    "authority": "scenario",
                    "physics_proxy": {
                        "mode": "kinematic",
                        "body_stable_id": "pedestrian_01/proxy_root",
                    },
                }
            ],
        },
    )
    _write_json(
        bundle_dir / "physics.plan.json",
        {
            "schema": "lingtu.sim.physics-plan.v1",
            "session_id": session_id,
            "kinematic_entities": [{"entity_id": "pedestrian_01"}],
        },
    )
    episode = json.loads((run_dir / "episode_result.json").read_text(encoding="utf-8"))
    episode["artifact_references"]["scenario_physics_evidence"] = (
        "scenario-physics-evidence.json"
    )
    _write_json(run_dir / "episode_result.json", episode)
    _write_json(
        run_dir / "scenario-visual-evidence.json",
        {
            "schema": "lingtu.sim.scenario-visual-evidence.v1",
            "source": "ue_registry_applied",
            "input_source": "canonical_scenario_snapshot",
            "run_id": run_id,
            "session_id": session_id,
            "model_generation": 0,
            "reset_generation": 0,
            "sequence": 8,
            "sim_time_ns": 6_000_000_000,
            "position_tolerance_m": 0.02,
            "maximum_position_error_m": 0.01,
            "within_tolerance": True,
            "expected_actor_count": 1,
            "actor_count": 1,
            "complete_actor_set": True,
            "all_actors_visible": True,
            "actors": [
                {
                    "entity_id": "pedestrian_01",
                    "stable_id": "pedestrian_01",
                    "visible": True,
                    "position_error_m": 0.01,
                }
            ],
        },
    )
    _write_json(
        run_dir / "scenario-physics-evidence.json",
        {
            "schema": "lingtu.sim.scenario-physics-evidence.v1",
            "source": "mujoco_applied_kinematic_proxy",
            "input_source": "canonical_scenario_snapshot",
            "run_id": run_id,
            "session_id": session_id,
            "model_generation": 0,
            "reset_generation": 0,
            "sequence": 8,
            "sim_time_ns": 6_000_000_000,
            "position_tolerance_m": 0.02,
            "maximum_position_error_m": 0.01,
            "within_tolerance": True,
            "expected_proxy_count": 1,
            "proxy_count": 1,
            "complete_proxy_set": True,
            "pose_applied": True,
            "raycast_observations": [
                {
                    "source": "mujoco_readback",
                    "applied": True,
                    "session_id": session_id,
                    "model_generation": 0,
                    "reset_generation": 0,
                    "sequence": 8,
                    "sim_time_ns": 6_000_000_000,
                    "query": {
                        "sensor_frame_id": "thunder_01.mid360",
                        "direction_sensor": [0.0, 1.0, 0.0],
                        "range_min_m": 0.01,
                        "range_max_m": 10.0,
                        "offset_time_ns": 0,
                        "origin_world_m": [4.0, -7.0, 1.0],
                        "direction_world": [0.0, 1.0, 0.0],
                    },
                    "result": {
                        "hit": True,
                        "entity_id": "pedestrian_01",
                        "body_stable_id": "pedestrian_01/proxy_root",
                        "distance_m": 1.0,
                        "position_world_m": [4.0, -6.0, 0.0],
                    },
                }
            ],
            "contact_observations": [],
            "proxies": [
                {
                    "entity_id": "pedestrian_01",
                    "body_stable_id": "pedestrian_01/proxy_root",
                    "position_error_m": 0.01,
                }
            ],
        },
    )


def test_build_e2e_qualification_record_binds_completed_run_artifacts(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)

    record = build_e2e_qualification_record(bundle_dir, run_dir)

    assert record["schema"] == "lingtu.sim.e2e-qualification-record.v1"
    assert record["qualified"] is True
    assert record["identity"] == {
        "run_id": "factory-six-motion",
        "session_id": "factory-six-motion",
        "model_generation": 0,
        "reset_generation": 0,
    }
    assert record["episode"]["status"] == "SUCCEEDED"
    assert record["required_facets"] == {
        "control": "ACTIVE",
        "physics": "ACTIVE",
        "sensors": "ACTIVE",
        "visual": "ACTIVE",
    }
    assert record["sensor_streams"]["required"] == {
        "thunder_01.truth_odom": "ACTIVE"
    }
    assert record["transport_allocation"]["dds_domain"] == 79
    assert record["recording"]["motion_evidence"]["qualified"] is True
    assert {artifact["path"] for artifact in record["artifacts"]} >= {
        "session.runtime.json",
        "episode_result.json",
        "motion-evidence.json",
        "scenario-visual-evidence.json",
        "simulation-recording.json",
        "simulation-timeline.jsonl",
    }
    for artifact in record["artifacts"]:
        assert artifact["sha256"] == _sha256(run_dir / artifact["path"])


def test_build_e2e_qualification_record_fails_closed_for_required_sensor(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    runtime = json.loads((run_dir / "session.runtime.json").read_text(encoding="utf-8"))
    runtime["sensor_streams"]["streams"]["thunder_01.truth_odom"]["state"] = "PREPARING"
    _write_json(run_dir / "session.runtime.json", runtime)

    record = build_e2e_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is False
    assert "required sensor thunder_01.truth_odom is PREPARING" in record["reasons"]


def test_build_e2e_qualification_record_fails_closed_for_zero_sample_summary(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    runtime = json.loads((run_dir / "session.runtime.json").read_text(encoding="utf-8"))
    runtime["sensor_streams"]["summary"]["streams"]["thunder_01.truth_odom"][
        "sample_count"
    ] = 0
    _write_json(run_dir / "session.runtime.json", runtime)

    record = build_e2e_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is False
    assert "required sensor thunder_01.truth_odom has no samples" in record["reasons"]


def test_build_e2e_qualification_record_rejects_stale_timeline_digest(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    timeline = run_dir / "simulation-timeline.jsonl"
    timeline.write_text("x" * timeline.stat().st_size, encoding="utf-8")

    with pytest.raises(QualificationRecordError, match="timeline sha256 mismatch"):
        build_e2e_qualification_record(bundle_dir, run_dir)


def test_build_e2e_qualification_record_fails_closed_for_failed_episode(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    episode = json.loads((run_dir / "episode_result.json").read_text(encoding="utf-8"))
    episode["status"] = "FAILED"
    episode["failure_reason"] = "frame capture did not reach minimum"
    _write_json(run_dir / "episode_result.json", episode)

    record = build_e2e_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is False
    assert "episode status is FAILED" in record["reasons"]
    assert record["episode"]["failure_reason"] == "frame capture did not reach minimum"


def test_build_e2e_qualification_record_rejects_duplicate_json_key(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    (run_dir / "episode_result.json").write_text(
        '{"schema":"lingtu.sim.episode-result.v1","schema":"duplicate"}',
        encoding="utf-8",
    )

    with pytest.raises(QualificationRecordError, match="duplicate key"):
        build_e2e_qualification_record(bundle_dir, run_dir)


def test_build_e2e_qualification_record_fails_closed_for_incomplete_v2_turn_gate(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    motion = json.loads((run_dir / "motion-evidence.json").read_text(encoding="utf-8"))
    del motion["maneuvers"][1]["maximum_turn_drift_m"]
    _write_json(run_dir / "motion-evidence.json", motion)

    record = build_e2e_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is False
    assert "motion evidence is not qualified" in record["reasons"]


def test_build_e2e_qualification_record_fails_closed_for_empty_maneuvers(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    motion = json.loads((run_dir / "motion-evidence.json").read_text(encoding="utf-8"))
    motion["maneuvers"] = []
    _write_json(run_dir / "motion-evidence.json", motion)

    record = build_e2e_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is False
    assert "motion evidence is not qualified" in record["reasons"]


@pytest.mark.parametrize(
    "mutation",
    ["missing_expected", "duplicate_name", "wrong_order"],
)
def test_build_e2e_qualification_record_requires_factory_park_exact_maneuvers(
    tmp_path: Path, mutation: str
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    motion = json.loads((run_dir / "motion-evidence.json").read_text(encoding="utf-8"))
    if mutation == "missing_expected":
        motion["maneuvers"].pop(2)
    elif mutation == "duplicate_name":
        motion["maneuvers"][1]["name"] = "forward"
    elif mutation == "wrong_order":
        motion["maneuvers"][0], motion["maneuvers"][1] = (
            motion["maneuvers"][1],
            motion["maneuvers"][0],
        )
    _write_json(run_dir / "motion-evidence.json", motion)

    record = build_e2e_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is False
    assert "motion evidence is not qualified" in record["reasons"]


@pytest.mark.parametrize(
    "mutation",
    ["missing_translation_threshold", "translation_gate_inconsistent"],
)
def test_build_e2e_qualification_record_requires_complete_translation_gate(
    tmp_path: Path, mutation: str
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    motion = json.loads((run_dir / "motion-evidence.json").read_text(encoding="utf-8"))
    if mutation == "missing_translation_threshold":
        del motion["maneuvers"][0]["minimum_displacement_m"]
    elif mutation == "translation_gate_inconsistent":
        motion["maneuvers"][0]["signed_translation_m"] = 0.01
    _write_json(run_dir / "motion-evidence.json", motion)

    record = build_e2e_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is False
    assert "motion evidence is not qualified" in record["reasons"]


def test_build_e2e_qualification_record_accepts_legacy_profile_only_when_strict_nonempty(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    motion = json.loads((run_dir / "motion-evidence.json").read_text(encoding="utf-8"))
    motion["acceptance_profile"] = {"name": "legacy_motion", "version": 1}
    motion["maneuvers"] = [
        {
            "name": "legacy_forward",
            "command": {"linear_x": 0.1, "linear_y": 0.0, "angular_z": 0.0},
            "expectation_met": True,
            "motion_verified": True,
            "rotation_commanded": False,
            "translation_commanded": True,
            "translation_met": True,
            "signed_translation_m": 0.09,
            "minimum_displacement_m": 0.08,
            "signed_yaw_rad": 0.0,
            "horizontal_drift_m": 0.09,
            "minimum_rotation_rad": 0.35,
            "maximum_turn_drift_m": 0.1,
            "rotation_met": True,
            "turn_drift_met": True,
        }
    ]
    _write_json(run_dir / "motion-evidence.json", motion)

    assert build_e2e_qualification_record(bundle_dir, run_dir)["qualified"] is True

    motion["maneuvers"][0]["translation_met"] = True
    motion["maneuvers"][0]["signed_translation_m"] = 0.01
    _write_json(run_dir / "motion-evidence.json", motion)

    record = build_e2e_qualification_record(bundle_dir, run_dir)
    assert record["qualified"] is False


def test_factory_park_profile_uses_verifier_owned_thresholds_not_self_report(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    motion = json.loads((run_dir / "motion-evidence.json").read_text(encoding="utf-8"))
    turn = motion["maneuvers"][4]
    turn["signed_yaw_rad"] = 0.05
    turn["minimum_rotation_rad"] = 0.01
    turn["horizontal_drift_m"] = 1.5
    turn["maximum_turn_drift_m"] = 2.0
    turn["rotation_met"] = True
    turn["turn_drift_met"] = True
    _write_json(run_dir / "motion-evidence.json", motion)

    record = build_e2e_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is False
    assert "motion evidence is not qualified" in record["reasons"]


def test_factory_park_profile_version_must_be_one(tmp_path: Path) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    motion = json.loads((run_dir / "motion-evidence.json").read_text(encoding="utf-8"))
    motion["acceptance_profile"]["version"] = 2
    _write_json(run_dir / "motion-evidence.json", motion)

    record = build_e2e_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is False
    assert "motion evidence is not qualified" in record["reasons"]


def test_required_sensors_need_nonempty_required_streams_and_summary(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    runtime = json.loads((run_dir / "session.runtime.json").read_text(encoding="utf-8"))
    runtime["sensor_streams"]["required_stream_ids"] = []
    runtime["sensor_streams"]["streams"] = {}
    runtime["sensor_streams"]["summary"]["streams"] = {}
    _write_json(run_dir / "session.runtime.json", runtime)

    record = build_e2e_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is False
    assert "required sensors have no required_stream_ids" in record["reasons"]


@pytest.mark.parametrize(
    ("mutation", "expected_reason"),
    [
        ("missing_reference", "scenario visual evidence is missing"),
        ("sender_source", "scenario visual evidence source is not ue_registry_applied"),
        ("wrong_input", "scenario visual evidence input_source is not canonical_scenario_snapshot"),
        ("stale_identity", "scenario visual evidence session_id mismatch"),
        ("too_wide_tolerance", "scenario visual evidence tolerance is wider than 0.02 m"),
        ("outside_tolerance", "scenario visual evidence is outside tolerance"),
        ("empty_actors", "scenario visual evidence has no actors"),
    ],
)
def test_visual_qualification_requires_ue_registry_applied_scenario_evidence(
    tmp_path: Path, mutation: str, expected_reason: str
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    _add_scenario_dispatch_proof(bundle_dir, run_dir)
    episode = json.loads((run_dir / "episode_result.json").read_text(encoding="utf-8"))
    evidence_path = run_dir / "scenario-visual-evidence.json"
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    if mutation == "missing_reference":
        del episode["artifact_references"]["scenario_visual_evidence"]
        _write_json(run_dir / "episode_result.json", episode)
    elif mutation == "sender_source":
        evidence["source"] = "udp_sender"
        _write_json(evidence_path, evidence)
    elif mutation == "wrong_input":
        evidence["input_source"] = "sender_ack"
        _write_json(evidence_path, evidence)
    elif mutation == "stale_identity":
        evidence["session_id"] = "b" * 64
        _write_json(evidence_path, evidence)
    elif mutation == "too_wide_tolerance":
        evidence["position_tolerance_m"] = 0.5
        _write_json(evidence_path, evidence)
    elif mutation == "outside_tolerance":
        evidence["within_tolerance"] = False
        evidence["maximum_error_m"] = 0.05
        _write_json(evidence_path, evidence)
    elif mutation == "empty_actors":
        evidence["actors"] = []
        _write_json(evidence_path, evidence)

    record = build_e2e_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is False
    assert expected_reason in record["reasons"]


@pytest.mark.parametrize("evidence_name", ["visual", "physics"])
@pytest.mark.parametrize("mutation", ["delete", "none", "whitespace", "wrong"])
def test_scenario_evidence_requires_canonical_run_id(
    tmp_path: Path,
    evidence_name: str,
    mutation: str,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    _add_scenario_dispatch_proof(bundle_dir, run_dir)
    filename = f"scenario-{evidence_name}-evidence.json"
    evidence_path = run_dir / filename
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    if mutation == "delete":
        del evidence["run_id"]
    elif mutation == "none":
        evidence["run_id"] = None
    elif mutation == "whitespace":
        evidence["run_id"] = " factory-six-motion "
    elif mutation == "wrong":
        evidence["run_id"] = "wrong-run"
    _write_json(evidence_path, evidence)

    record = build_e2e_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is False
    assert f"scenario {evidence_name} evidence run_id mismatch" in record["reasons"]


def test_scenario_bearing_bundle_requires_visual_and_physics_applied_proof(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    _add_scenario_dispatch_proof(bundle_dir, run_dir)

    record = build_e2e_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is True
    assert record["scenario_visual"]["actor_count"] == 1
    assert record["scenario_physics"]["proxy_count"] == 1
    assert {artifact["path"] for artifact in record["artifacts"]} >= {
        "scenario-visual-evidence.json",
        "scenario-physics-evidence.json",
    }


@pytest.mark.parametrize(
    ("mutation", "expected_reason"),
    [
        ("missing_physics_reference", "scenario physics evidence is missing"),
        ("wrong_physics_source", "scenario physics evidence source is not mujoco_applied_kinematic_proxy"),
        ("stale_physics_digest", "scenario physics evidence session_id mismatch"),
        ("physics_sequence_mismatch", "scenario visual/physics sequence mismatch"),
        ("wrong_proxy", "scenario physics proxy set does not match scenario plan"),
        ("physics_error", "scenario physics evidence is outside tolerance"),
        ("missing_readback", "scenario physics readback proof is missing"),
        ("minimal_readback_stub", "scenario physics readback proof is missing"),
        ("missing_readback_stamp", "scenario physics readback proof is missing"),
        ("wrong_readback_sequence", "scenario physics readback proof is missing"),
        ("wrong_readback_body", "scenario physics readback proof is missing"),
        ("missing_readback_geometry", "scenario physics readback proof is missing"),
        ("outside_raycast_range", "scenario physics readback proof is missing"),
    ],
)
def test_physics_qualification_requires_mujoco_applied_scenario_evidence(
    tmp_path: Path, mutation: str, expected_reason: str
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    _add_scenario_dispatch_proof(bundle_dir, run_dir)
    episode = json.loads((run_dir / "episode_result.json").read_text(encoding="utf-8"))
    evidence_path = run_dir / "scenario-physics-evidence.json"
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    if mutation == "missing_physics_reference":
        del episode["artifact_references"]["scenario_physics_evidence"]
        _write_json(run_dir / "episode_result.json", episode)
    elif mutation == "wrong_physics_source":
        evidence["source"] = "mujoco_queued"
        _write_json(evidence_path, evidence)
    elif mutation == "stale_physics_digest":
        evidence["session_id"] = "b" * 64
        _write_json(evidence_path, evidence)
    elif mutation == "physics_sequence_mismatch":
        evidence["sequence"] = 9
        _write_json(evidence_path, evidence)
    elif mutation == "wrong_proxy":
        evidence["proxies"][0]["entity_id"] = "wrong"
        _write_json(evidence_path, evidence)
    elif mutation == "physics_error":
        evidence["maximum_position_error_m"] = 0.03
        _write_json(evidence_path, evidence)
    elif mutation == "missing_readback":
        evidence["raycast_observations"] = []
        evidence["contact_observations"] = []
        _write_json(evidence_path, evidence)
    elif mutation == "minimal_readback_stub":
        evidence["raycast_observations"] = [
            {"source": "mujoco_readback", "applied": True}
        ]
        _write_json(evidence_path, evidence)
    elif mutation == "missing_readback_stamp":
        del evidence["raycast_observations"][0]["sim_time_ns"]
        _write_json(evidence_path, evidence)
    elif mutation == "wrong_readback_sequence":
        evidence["raycast_observations"][0]["sequence"] = 7
        _write_json(evidence_path, evidence)
    elif mutation == "wrong_readback_body":
        evidence["raycast_observations"][0]["result"]["body_stable_id"] = "wrong/body"
        _write_json(evidence_path, evidence)
    elif mutation == "missing_readback_geometry":
        del evidence["raycast_observations"][0]["query"]["origin_world_m"]
        _write_json(evidence_path, evidence)
    elif mutation == "outside_raycast_range":
        evidence["raycast_observations"][0]["result"]["distance_m"] = 11.0
        _write_json(evidence_path, evidence)

    record = build_e2e_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is False
    assert expected_reason in record["reasons"]


def test_physics_qualification_rejects_nonfinite_readback_geometry(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    _add_scenario_dispatch_proof(bundle_dir, run_dir)
    evidence_path = run_dir / "scenario-physics-evidence.json"
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    evidence["raycast_observations"][0]["result"]["distance_m"] = float("nan")
    evidence_path.write_text(json.dumps(evidence, sort_keys=True), encoding="utf-8")

    with pytest.raises(QualificationRecordError, match="non-finite"):
        build_e2e_qualification_record(bundle_dir, run_dir)


def test_build_e2e_qualification_record_rejects_artifact_symlink_before_hash(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    timeline = run_dir / "simulation-timeline.jsonl"
    timeline.unlink()
    try:
        timeline.symlink_to(run_dir / "episode_result.json")
    except OSError as exc:
        pytest.skip(f"symlink creation is unavailable: {exc}")

    with pytest.raises(QualificationRecordError, match=r"reparse|symlink"):
        build_e2e_qualification_record(bundle_dir, run_dir)


def test_build_e2e_qualification_record_rejects_windows_junction_before_read(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_run(tmp_path)
    actual_run = tmp_path / "actual-run"
    run_dir.rename(actual_run)
    cmd = os.environ.get("ComSpec", r"C:\Windows\System32\cmd.exe")
    result = subprocess.run(  # noqa: S603
        [cmd, "/c", "mklink", "/J", str(run_dir), str(actual_run)],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        pytest.skip(f"junction creation is unavailable: {result.stderr or result.stdout}")

    with pytest.raises(QualificationRecordError, match=r"reparse|symlink"):
        build_e2e_qualification_record(bundle_dir, run_dir)
