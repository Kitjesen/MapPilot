
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest
from sim.catalog.resolver import CatalogResolver
from sim.runtime.qualification.production import (
    G007_PRODUCTION_RECORD_SCHEMA,
    build_g007_production_qualification_record,
    write_g007_production_qualification_result,
)
from sim.runtime.recording import SimulationRecordingWriter
from sim.runtime.qualification.thunderv4 import THUNDERV4_NAVIGATION_STREAM_IDS

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION = (
    REPO_ROOT
    / "sim" / "sessions" / "examples"
    / "thunderv4_open_field_pedestrian_unreal"
    / "session.yaml"
)


def _write_json(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, sort_keys=True), encoding="utf-8")


def _maneuver(
    name: str,
    *,
    linear_x: float = 0.0,
    linear_y: float = 0.0,
    angular_z: float = 0.0,
) -> dict[str, Any]:
    rotation = angular_z != 0.0
    translation = linear_x != 0.0 or linear_y != 0.0
    return {
        "name": name,
        "command": {"linear_x": linear_x, "linear_y": linear_y, "angular_z": angular_z},
        "expectation_met": True,
        "motion_verified": True,
        "rotation_commanded": rotation,
        "translation_commanded": translation,
        "translation_met": True,
        "rotation_met": True,
        "turn_drift_met": True,
        "signed_translation_m": 0.10 if translation else 0.0,
        "minimum_displacement_m": 0.08,
        "signed_yaw_rad": 0.41 if rotation else 0.0,
        "horizontal_drift_m": 0.04,
        "minimum_rotation_rad": 0.35,
        "maximum_turn_drift_m": 0.10,
    }


def _completed_production_run(tmp_path: Path) -> tuple[Path, Path]:
    bundle_dir = tmp_path / "bundle"
    run_dir = tmp_path / "run"
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION)
    resolved.write_bundle(bundle_dir)
    _write_json(bundle_dir / "session.yaml", resolved.session)
    session_id = resolved.session_id
    run_id = "g007-production"
    run_dir.mkdir()
    allocation = {
        "schema": "lingtu.sim.run-allocation.v1",
        "run_id": run_id,
        "session_id": session_id,
        "dds_domain": 79,
        "ports": {"visual_snapshot_udp": 25565},
        "shm": {
            "thunder_01.front_depth": "Local\\lingtu-g007-front-depth",
            "thunder_01.front_rgb": "Local\\lingtu-g007-front-rgb",
        },
    }
    _write_json(run_dir / "run-allocation.json", allocation)
    sensor_streams = {
        stream_id: {
            "required": True,
            "state": "ACTIVE",
            "sample_count": 4,
        }
        for stream_id in THUNDERV4_NAVIGATION_STREAM_IDS
    }
    _write_json(
        run_dir / "session.runtime.json",
        {
            "schema": "lingtu.sim.session-runtime.v1",
            "run_id": run_id,
            "session_id": session_id,
            "model_generation": 3,
            "reset_generation": 4,
            "state": "STOPPED",
            "allocation": {
                "run_dir": str(run_dir.resolve()),
                "dds_domain": 79,
                "ports": {"visual_snapshot_udp": 25565},
                "shm": allocation["shm"],
            },
            "bindings": {
                "physics": {"required": True, "state": "ACTIVE"},
                "visual": {"required": True, "state": "ACTIVE"},
                "sensors": {"required": True, "state": "ACTIVE"},
                "control": {"required": True, "state": "ACTIVE"},
            },
            "sensor_streams": {
                "is_ready": True,
                "required_stream_ids": list(THUNDERV4_NAVIGATION_STREAM_IDS),
                "streams": sensor_streams,
                "summary": {
                    "schema": "lingtu.sim.sensor-stream-summary.v1",
                    "run_id": run_id,
                    "session_id": session_id,
                    "model_generation": 3,
                    "reset_generation": 4,
                    "is_ready": True,
                    "required_stream_ids": list(THUNDERV4_NAVIGATION_STREAM_IDS),
                    "blocking_reasons": {},
                    "streams": sensor_streams,
                },
            },
        },
    )
    with SimulationRecordingWriter(
        run_dir,
        run_id=run_id,
        session_id=session_id,
        run_allocation=allocation,
        required_content=(
            "command",
            "truth_snapshot",
            "scenario_event",
            "sensor_metadata",
            "lifecycle_evidence",
        ),
        continuous_tolerances={"/bodies/*/position_m/*": 0.02},
    ) as recorder:
        base_snapshot = {
            "event": "snapshot",
            "session_id": session_id,
            "model_generation": 3,
            "reset_generation": 4,
            "bodies": [
                {
                    "stable_id": "pedestrian_01/proxy_root",
                    "position_m": [4.0, -6.0, 0.0],
                    "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                }
            ],
            "joints": [],
            "actuators": [],
        }
        recorder.append(
            {
                **base_snapshot,
                "sequence": 0,
                "physics_step": 0,
                "sim_time_ns": 0,
            },
            command={"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0},
            scenario_events=[
                {
                    "session_id": session_id,
                    "model_generation": 3,
                    "reset_generation": 4,
                    "event": "scenario_proxy_applied",
                    "entity_id": "pedestrian_01",
                }
            ],
            sensor_metadata=[
                {
                    "session_id": session_id,
                    "model_generation": 3,
                    "reset_generation": 4,
                    "sensor_id": "thunder_01.front_rgb",
                    "sample_count": 1,
                }
            ],
        )
        recorder.append(
            {
                **base_snapshot,
                "sequence": 1,
                "physics_step": 1,
                "sim_time_ns": 2_000_000,
            },
            command={"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0},
            lifecycle_evidence=[
                {
                    "session_id": session_id,
                    "model_generation": 3,
                    "reset_generation": 4,
                    "state": "STOPPED",
                }
            ],
        )
    _write_json(
        run_dir / "motion-evidence.json",
        {
            "schema": "lingtu.sim.motion-recording-evidence.v2",
            "run_id": run_id,
            "session_id": session_id,
            "acceptance_profile": {"name": "factory_park_motion", "version": 1},
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
    _write_json(
        run_dir / "scenario-visual-evidence.json",
        {
            "schema": "lingtu.sim.scenario-visual-evidence.v1",
            "source": "ue_registry_applied",
            "input_source": "canonical_scenario_snapshot",
            "basis": "snapshot_pose_applied_to_unreal_actor",
            "run_id": run_id,
            "session_id": session_id,
            "model_generation": 3,
            "reset_generation": 4,
            "sequence": 1,
            "sim_time_ns": 2_000_000,
            "position_tolerance_m": 0.02,
            "maximum_error_m": 0.01,
            "maximum_position_error_m": 0.01,
            "complete_actor_set": True,
            "all_actors_visible": True,
            "expected_actor_count": 2,
            "actor_count": 2,
            "within_tolerance": True,
            "actors": [
                {
                    "entity_id": "thunder_01",
                    "stable_id": "thunder_01",
                    "visible": True,
                    "expected_position_m": [0.0, 0.0, 0.0],
                    "observed_position_m": [0.0, 0.0, 0.0],
                    "expected": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "observed": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "position_error_m": 0.0,
                },
                {
                    "entity_id": "pedestrian_01",
                    "stable_id": "pedestrian_01",
                    "visible": True,
                    "expected_position_m": [4.0, -5.99, 0.0],
                    "observed_position_m": [4.0, -5.98, 0.0],
                    "expected": {"x": 4.0, "y": -5.99, "z": 0.0},
                    "observed": {"x": 4.0, "y": -5.98, "z": 0.0},
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
            "model_generation": 3,
            "reset_generation": 4,
            "sequence": 1,
            "sim_time_ns": 2_000_000,
            "position_tolerance_m": 0.02,
            "maximum_position_error_m": 0.01,
            "complete_proxy_set": True,
            "pose_applied": True,
            "expected_proxy_count": 1,
            "proxy_count": 1,
            "within_tolerance": True,
            "proxies": [
                {
                    "entity_id": "pedestrian_01",
                    "body_stable_id": "pedestrian_01/proxy_root",
                    "expected": {"x": 4.0, "y": -5.99, "z": 0.0},
                    "observed": {"x": 4.0, "y": -5.98, "z": 0.0},
                    "position_error_m": 0.01,
                }
            ],
            "raycast_observations": [
                {
                    "source": "mujoco_readback",
                    "applied": True,
                    "session_id": session_id,
                    "model_generation": 3,
                    "reset_generation": 4,
                    "sequence": 1,
                    "sim_time_ns": 2_000_000,
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
        },
    )
    _write_json(
        run_dir / "episode_result.json",
        {
            "schema": "lingtu.sim.episode-result.v1",
            "run_id": run_id,
            "session_id": session_id,
            "model_generation": 3,
            "reset_generation": 4,
            "status": "SUCCEEDED",
            "failure_reason": None,
            "artifact_references": {
                "runtime_manifest": "session.runtime.json",
                "run_allocation": "run-allocation.json",
                "simulation_recording": "simulation-recording.json",
                "simulation_timeline": "simulation-timeline.jsonl",
                "scenario_visual_evidence": "scenario-visual-evidence.json",
                "scenario_physics_evidence": "scenario-physics-evidence.json",
            },
        },
    )
    return bundle_dir, run_dir


def test_g007_session_combines_open_field_pedestrian_unreal_full_stack() -> None:
    resolved = CatalogResolver.from_repository(REPO_ROOT).resolve(SESSION)

    assert resolved.session["runtime"] == {
        "backend": "mujoco",
        "mode": "unreal",
        "required_bindings": ["physics", "visual", "sensors", "control"],
    }
    assert resolved.session["world"] == "open_field@1.1.0"
    assert resolved.session["scenario"] == "open_field_pedestrian_crossing@1.1.0"
    assert resolved.session["robots"][0]["sensor_rig"] == "thunderv4_navigation@1.0.0"
    assert {stream["sensor_id"] for streams in resolved.sensor_plan["streams"].values() for stream in streams} == set(
        THUNDERV4_NAVIGATION_STREAM_IDS
    )
    assert resolved.scenario_plan is not None
    assert resolved.visual_plan["backends"] == {"physics": "mujoco", "visual": "unreal"}


def test_g007_production_qualification_accepts_complete_same_session_evidence(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_production_run(tmp_path)

    record = build_g007_production_qualification_record(bundle_dir, run_dir)

    assert record["schema"] == G007_PRODUCTION_RECORD_SCHEMA
    assert record["qualified"] is True
    assert record["reasons"] == []
    assert record["checks"]["sensor_streams"]["required_stream_ids"] == list(
        THUNDERV4_NAVIGATION_STREAM_IDS
    )
    assert record["checks"]["scenario_visual"]["entity_ids"] == [
        "pedestrian_01",
        "thunder_01",
    ]
    assert record["checks"]["scenario_physics"]["expected_entities"] == {
        "pedestrian_01": "pedestrian_01/proxy_root"
    }
    assert record["checks"]["recording_replay"]["first"]["event_order"] == record[
        "checks"
    ]["recording_replay"]["second"]["event_order"]


def test_g007_production_result_is_separate_from_episode_result(tmp_path: Path) -> None:
    bundle_dir, run_dir = _completed_production_run(tmp_path)
    episode_before = (run_dir / "episode_result.json").read_bytes()

    path = write_g007_production_qualification_result(bundle_dir, run_dir)

    result = json.loads(path.read_text(encoding="utf-8"))
    assert path == run_dir / "qualification_result.json"
    assert result["schema"] == "lingtu.sim.qualification-result.v1"
    assert result["qualified"] is True
    assert result["record"]["schema"] == G007_PRODUCTION_RECORD_SCHEMA
    assert (run_dir / "episode_result.json").read_bytes() == episode_before


@pytest.mark.parametrize(
    ("mutation", "expected_reason"),
    [
        ("missing_physics_reference", "scenario physics evidence is missing"),
        ("self_certified_physics", "scenario physics evidence source is invalid"),
        ("no_observability", "scenario physics evidence has no MuJoCo readback/raycast/contact proof"),
        ("minimal_raycast_stub", "scenario physics evidence has no MuJoCo readback/raycast/contact proof"),
        ("raycast_missing_time", "scenario physics evidence has no MuJoCo readback/raycast/contact proof"),
        ("visual_sender_ack", "scenario visual evidence source is invalid"),
        ("sensor_missing", "production sensors must be the exact ThunderV4 navigation five-stream set"),
    ],
)
def test_g007_production_qualification_fails_closed_for_missing_or_untrusted_evidence(
    tmp_path: Path,
    mutation: str,
    expected_reason: str,
) -> None:
    bundle_dir, run_dir = _completed_production_run(tmp_path)
    if mutation == "missing_physics_reference":
        episode = json.loads((run_dir / "episode_result.json").read_text(encoding="utf-8"))
        del episode["artifact_references"]["scenario_physics_evidence"]
        _write_json(run_dir / "episode_result.json", episode)
    elif mutation == "self_certified_physics":
        evidence = json.loads((run_dir / "scenario-physics-evidence.json").read_text(encoding="utf-8"))
        evidence["source"] = "scenario_dispatcher_self_report"
        _write_json(run_dir / "scenario-physics-evidence.json", evidence)
    elif mutation == "no_observability":
        evidence = json.loads((run_dir / "scenario-physics-evidence.json").read_text(encoding="utf-8"))
        evidence["raycast_observations"] = []
        _write_json(run_dir / "scenario-physics-evidence.json", evidence)
    elif mutation == "minimal_raycast_stub":
        evidence = json.loads((run_dir / "scenario-physics-evidence.json").read_text(encoding="utf-8"))
        evidence["raycast_observations"] = [
            {
                "source": "mujoco_readback",
                "applied": True,
                "session_id": evidence["session_id"],
                "model_generation": evidence["model_generation"],
                "reset_generation": evidence["reset_generation"],
                "sequence": evidence["sequence"],
                "sim_time_ns": evidence["sim_time_ns"],
            }
        ]
        _write_json(run_dir / "scenario-physics-evidence.json", evidence)
    elif mutation == "raycast_missing_time":
        evidence = json.loads((run_dir / "scenario-physics-evidence.json").read_text(encoding="utf-8"))
        del evidence["raycast_observations"][0]["sim_time_ns"]
        _write_json(run_dir / "scenario-physics-evidence.json", evidence)
    elif mutation == "visual_sender_ack":
        evidence = json.loads((run_dir / "scenario-visual-evidence.json").read_text(encoding="utf-8"))
        evidence["source"] = "udp_sender"
        _write_json(run_dir / "scenario-visual-evidence.json", evidence)
    elif mutation == "sensor_missing":
        runtime = json.loads((run_dir / "session.runtime.json").read_text(encoding="utf-8"))
        runtime["sensor_streams"]["required_stream_ids"].remove("thunder_01.mid360")
        del runtime["sensor_streams"]["streams"]["thunder_01.mid360"]
        del runtime["sensor_streams"]["summary"]["streams"]["thunder_01.mid360"]
        _write_json(run_dir / "session.runtime.json", runtime)

    record = build_g007_production_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is False
    assert expected_reason in record["reasons"]


@pytest.mark.parametrize("evidence_name", ["visual", "physics"])
@pytest.mark.parametrize("mutation", ["delete", "none", "whitespace", "wrong"])
def test_g007_production_qualification_requires_canonical_evidence_run_id(
    tmp_path: Path,
    evidence_name: str,
    mutation: str,
) -> None:
    bundle_dir, run_dir = _completed_production_run(tmp_path)
    evidence_path = run_dir / f"scenario-{evidence_name}-evidence.json"
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    if mutation == "delete":
        del evidence["run_id"]
    elif mutation == "none":
        evidence["run_id"] = None
    elif mutation == "whitespace":
        evidence["run_id"] = " g007-production "
    elif mutation == "wrong":
        evidence["run_id"] = "wrong-run"
    _write_json(evidence_path, evidence)

    record = build_g007_production_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is False
    assert f"scenario {evidence_name} evidence run_id mismatch" in record["reasons"]


def test_g007_production_qualification_accepts_concrete_contact_readback(
    tmp_path: Path,
) -> None:
    bundle_dir, run_dir = _completed_production_run(tmp_path)
    evidence_path = run_dir / "scenario-physics-evidence.json"
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    evidence["raycast_observations"] = []
    evidence["contact_observations"] = [
        {
            "source": "mujoco_readback",
            "applied": True,
            "session_id": evidence["session_id"],
            "model_generation": evidence["model_generation"],
            "reset_generation": evidence["reset_generation"],
            "sequence": evidence["sequence"],
            "sim_time_ns": evidence["sim_time_ns"],
            "entity_id": "pedestrian_01",
            "body_stable_id": "pedestrian_01/proxy_root",
            "other_entity_id": "ground",
            "other_body_stable_id": "ground",
            "contact_count": 1,
            "contact_position_m": [4.0, -6.0, 0.0],
            "penetration_m": 0.001,
        }
    ]
    _write_json(evidence_path, evidence)

    record = build_g007_production_qualification_record(bundle_dir, run_dir)

    assert record["qualified"] is True


def test_g007_production_result_rejects_truncated_recording(tmp_path: Path) -> None:
    bundle_dir, run_dir = _completed_production_run(tmp_path)
    timeline = run_dir / "simulation-timeline.jsonl"
    timeline.write_bytes(timeline.read_bytes()[:-5])

    path = write_g007_production_qualification_result(bundle_dir, run_dir)

    result = json.loads(path.read_text(encoding="utf-8"))
    assert result["qualified"] is False
    assert result["verdict"] == "EVIDENCE_REJECTED"
    assert result["record"] is None
    assert "timeline" in result["error"]["message"]
