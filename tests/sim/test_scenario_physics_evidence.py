# ruff: noqa: S101

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from sim.runtime.qualification.scenario_physics_evidence import (
    SCENARIO_PHYSICS_EVIDENCE_FILENAME,
    SCENARIO_PHYSICS_EVIDENCE_SCHEMA,
    ScenarioPhysicsEvidenceError,
    ScenarioPhysicsEvidenceSink,
    ScenarioPhysicsRaycastProbe,
    build_scenario_physics_evidence_dispatcher,
)
from sim.runtime.scenario import (
    EntitySnapshot,
    GenerationStamp,
    ScenarioSnapshot,
    Transform,
)

DIGEST = "b" * 64


def _snapshot(*, sequence: int = 4, sim_time_ns: int = 5_000_000) -> ScenarioSnapshot:
    return ScenarioSnapshot(
        session_id=DIGEST,
        model_generation=2,
        reset_generation=3,
        sequence=sequence,
        sim_time_ns=sim_time_ns,
        entities=(
            EntitySnapshot(
                entity_id="pedestrian_01",
                transform=Transform(
                    position_m=(4.0, 1.0, 0.0),
                    quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
                ),
                authority="scenario",
                source_epoch=0,
                semantic_class="pedestrian",
                motion_state="moving",
                physics_proxy_mode="kinematic",
                body_stable_id="pedestrian_01/proxy_root",
            ),
        ),
    )


def _probe() -> ScenarioPhysicsRaycastProbe:
    return ScenarioPhysicsRaycastProbe(
        sensor_frame_id="thunder_01/base_link",
        direction_sensor=(1.0, 0.0, 0.0),
        expected_entity_id="pedestrian_01",
        expected_body_stable_id="pedestrian_01/proxy_root",
        range_min_m=0.01,
        range_max_m=10.0,
    )


class _Host:
    def __init__(self, *, ray_hit: dict[str, Any] | None = None) -> None:
        self.applied: list[ScenarioSnapshot] = []
        self.ray_hit = ray_hit if ray_hit is not None else {
            "entity_id": "pedestrian_01",
            "body_stable_id": "pedestrian_01/proxy_root",
            "origin_world_m": [0.0, 1.0, 0.0],
            "direction_world": [1.0, 0.0, 0.0],
            "position_world_m": [4.0, 1.0, 0.0],
            "distance_m": 4.0,
            "xyz_sensor": [4.0, 0.0, 0.0],
            "offset_time_ns": 0,
            "reflectivity": 15,
            "tag": 0,
            "line": 0,
        }

    def apply_kinematic_poses(self, snapshot: ScenarioSnapshot) -> dict[str, Any]:
        self.applied.append(snapshot)
        return {
            "event": "kinematic_poses",
            "result": "applied",
            "session_id": snapshot.session_id,
            "model_generation": snapshot.model_generation,
            "reset_generation": snapshot.reset_generation,
            "sequence": snapshot.sequence,
            "sim_time_ns": snapshot.sim_time_ns,
        }

    def snapshot(self) -> dict[str, Any]:
        snapshot = self.applied[-1]
        return {
            "event": "snapshot",
            "session_id": snapshot.session_id,
            "model_generation": snapshot.model_generation,
            "reset_generation": snapshot.reset_generation,
            "sequence": snapshot.sequence,
            "sim_time_ns": snapshot.sim_time_ns,
            "bodies": [
                {
                    "stable_id": "pedestrian_01/proxy_root",
                    "position_m": [4.0, 1.0, 0.0],
                    "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                }
            ],
        }

    def raycast(self, **_kwargs: Any) -> dict[str, Any]:
        snapshot = self.applied[-1]
        hits = [] if self.ray_hit == {} else [self.ray_hit]
        return {
            "event": "raycast",
            "sensor_frame_id": "thunder_01/base_link",
            "session_id": snapshot.session_id,
            "model_generation": snapshot.model_generation,
            "reset_generation": snapshot.reset_generation,
            "sequence": snapshot.sequence,
            "sim_time_ns": snapshot.sim_time_ns,
            "hit_count": len(hits),
            "hits": hits,
        }


class _StaleHost(_Host):
    def snapshot(self) -> dict[str, Any]:
        value = super().snapshot()
        value["sequence"] = 99
        return value


class _StaleRaycastHost(_Host):
    def raycast(self, **kwargs: Any) -> dict[str, Any]:
        value = super().raycast(**kwargs)
        value["sequence"] = 99
        return value


class _MinimalStub:
    def apply_kinematic_poses(self, snapshot: ScenarioSnapshot) -> dict[str, Any]:
        return {"result": "applied", "session_id": snapshot.session_id}


def test_valid_readback_and_raycast_writes_and_registers_artifact(tmp_path: Path) -> None:
    registrations: list[tuple[str, str]] = []
    sink = ScenarioPhysicsEvidenceSink(
        host=_Host(),
        run_dir=tmp_path,
        raycast_probes=(_probe(),),
        run_id="g007-run",
        register_episode_artifact=lambda name, path: registrations.append((name, path)),
    )

    ack = sink.apply_kinematic_poses(_snapshot())

    assert ack["result"] == "applied"
    assert registrations == [
        ("scenario_physics_evidence", SCENARIO_PHYSICS_EVIDENCE_FILENAME)
    ]
    evidence = json.loads(
        (tmp_path / SCENARIO_PHYSICS_EVIDENCE_FILENAME).read_text(encoding="utf-8")
    )
    assert evidence["schema"] == SCENARIO_PHYSICS_EVIDENCE_SCHEMA
    assert evidence["source"] == "mujoco_applied_kinematic_proxy"
    assert evidence["input_source"] == "canonical_scenario_snapshot"
    assert evidence["run_id"] == "g007-run"
    assert evidence["session_id"] == DIGEST
    assert evidence["sequence"] == 4
    assert evidence["sim_time_ns"] == 5_000_000
    assert evidence["complete_proxy_set"] is True
    assert evidence["pose_applied"] is True
    assert evidence["expected_proxy_count"] == 1
    assert evidence["proxy_count"] == 1
    assert evidence["maximum_position_error_m"] == pytest.approx(0.0)
    assert evidence["within_tolerance"] is True
    assert evidence["raycast_observations"][0]["result"]["body_stable_id"] == (
        "pedestrian_01/proxy_root"
    )


def test_dispatcher_wiring_uses_evidence_sink_as_authoritative_physics_sink(
    tmp_path: Path,
) -> None:
    registrations: list[tuple[str, str]] = []
    sink = ScenarioPhysicsEvidenceSink(
        host=_Host(),
        run_dir=tmp_path,
        raycast_probes=(_probe(),),
        register_episode_artifact=lambda name, path: registrations.append((name, path)),
    )
    dispatcher = build_scenario_physics_evidence_dispatcher(
        session_id=DIGEST,
        initial_generation=GenerationStamp(2, 3),
        physics_sink=sink,
    )

    dispatcher.dispatch(_snapshot(sequence=0, sim_time_ns=0))

    assert dispatcher.last_sequence == 0
    assert registrations == [
        ("scenario_physics_evidence", SCENARIO_PHYSICS_EVIDENCE_FILENAME)
    ]
    assert (tmp_path / SCENARIO_PHYSICS_EVIDENCE_FILENAME).is_file()


def test_minimal_stub_without_snapshot_and_raycast_is_rejected(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="snapshot"):
        ScenarioPhysicsEvidenceSink(
            host=_MinimalStub(),  # type: ignore[arg-type]
            run_dir=tmp_path,
            raycast_probes=(_probe(),),
        )


def test_stale_readback_generation_fails_closed(tmp_path: Path) -> None:
    sink = ScenarioPhysicsEvidenceSink(
        host=_StaleHost(),
        run_dir=tmp_path,
        raycast_probes=(_probe(),),
    )

    with pytest.raises(ScenarioPhysicsEvidenceError, match="sequence mismatch"):
        sink.apply_kinematic_poses(_snapshot())

    assert not (tmp_path / SCENARIO_PHYSICS_EVIDENCE_FILENAME).exists()


def test_ray_miss_fails_closed(tmp_path: Path) -> None:
    sink = ScenarioPhysicsEvidenceSink(
        host=_Host(ray_hit={}),
        run_dir=tmp_path,
        raycast_probes=(_probe(),),
    )

    with pytest.raises(ScenarioPhysicsEvidenceError, match="did not hit"):
        sink.apply_kinematic_poses(_snapshot())


def test_raycast_wrong_body_fails_closed(tmp_path: Path) -> None:
    wrong_hit = {
        "entity_id": "barrel_01",
        "body_stable_id": "barrel_01/proxy_root",
        "origin_world_m": [0.0, 1.0, 0.0],
        "direction_world": [1.0, 0.0, 0.0],
        "position_world_m": [4.0, 1.0, 0.0],
        "distance_m": 4.0,
    }
    sink = ScenarioPhysicsEvidenceSink(
        host=_Host(ray_hit=wrong_hit),
        run_dir=tmp_path,
        raycast_probes=(_probe(),),
    )

    with pytest.raises(ScenarioPhysicsEvidenceError, match="wrong entity"):
        sink.apply_kinematic_poses(_snapshot())


def test_stale_raycast_sequence_fails_closed(tmp_path: Path) -> None:
    sink = ScenarioPhysicsEvidenceSink(
        host=_StaleRaycastHost(),
        run_dir=tmp_path,
        raycast_probes=(_probe(),),
    )

    with pytest.raises(ScenarioPhysicsEvidenceError, match="sequence mismatch"):
        sink.apply_kinematic_poses(_snapshot())

    assert not (tmp_path / SCENARIO_PHYSICS_EVIDENCE_FILENAME).exists()
