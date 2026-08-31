"""Validated recording read models for SimStudio runs."""

# ruff: noqa: D103, S101

from __future__ import annotations

import json
from pathlib import Path

import pytest
from sim.runtime.recording import SensorPayloadSample, SimulationRecordingWriter
from sim.runtime.replay import SimulationReplay
from tools.simstudio.http import API_PREFIX, create_app
from tools.simstudio.service.application import SimulationStudioService
from tools.simstudio.service.artifact_service import ArtifactService
from tools.simstudio.service.recording_service import RecordingService
from tools.simstudio.service.run_service import RunService
from tools.simstudio.service.store import StudioStore


def _snapshot(sequence: int, sim_time_ns: int, x: float) -> dict[str, object]:
    return {
        "event": "snapshot",
        "session_id": "studio-session",
        "model_generation": 2,
        "reset_generation": 3,
        "sequence": sequence,
        "physics_step": sequence * 8,
        "sim_time_ns": sim_time_ns,
        "bodies": [
            {
                "stable_id": "robot_01/base_link",
                "instance_id": "robot_01",
                "frame_id": "base_link",
                "position_m": [x, 0.0, 0.5],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            }
        ],
        "joints": [],
        "actuators": [],
    }


def _studio_run(tmp_path: Path) -> tuple[StudioStore, str, Path]:
    ids = iter(["b" * 32, "c" * 32])
    store = StudioStore(tmp_path / "studio", id_factory=lambda: next(ids))
    bundle = store.create_bundle(
        {
            "bundle_path": "bundles/demo",
            "session_id": "studio-session",
        }
    )
    run = store.create_run(
        {
            "schema": "lingtu.sim.studio.run-payload.v1",
            "bundle_id": bundle.id,
            "launch_profile": "visual",
            "artifact_path": f"artifacts/runs/{'c' * 32}",
        },
        status="STOPPED",
    )
    root = store.root / "artifacts" / "runs" / run.id
    root.mkdir(parents=True)
    return store, run.id, root


def test_valid_recording_is_bound_to_its_run_and_bundle(tmp_path: Path) -> None:
    store, run_id, root = _studio_run(tmp_path)
    with SimulationRecordingWriter(
        root,
        run_id=run_id,
        session_id="studio-session",
    ) as writer:
        writer.append(_snapshot(10, 1_000_000_000, 0.0))
        writer.append(_snapshot(11, 1_020_000_000, 0.01))

    service = RecordingService(store, ArtifactService(store))

    recording = service.inspect(run_id)

    assert recording == {
        "schema": "lingtu.sim.studio.recording.v1",
        "recording_id": recording["recording_id"],
        "run_id": run_id,
        "state": "VALID",
        "source_run_id": run_id,
        "session_id": "studio-session",
        "model_generation": 2,
        "reset_generation": {"start": 3, "end": 3},
        "frame_count": 2,
        "duration_ns": 20_000_000,
        "terminal_state": None,
        "event_order": ["truth_snapshot", "truth_snapshot"],
        "replay": {
            "deterministic": True,
            "visual": True,
            "clock_authority": "recorded_mujoco",
        },
        "artifacts": {
            "manifest": "simulation-recording.json",
            "timeline": "simulation-timeline.jsonl",
        },
        "diagnostics": [],
    }
    assert len(recording["recording_id"]) == 32
    assert service.list_recordings()["recordings"] == [recording]


def test_recording_timeline_is_pageable_by_validated_frame_index(
    tmp_path: Path,
) -> None:
    store, run_id, root = _studio_run(tmp_path)
    with SimulationRecordingWriter(
        root,
        run_id=run_id,
        session_id="studio-session",
    ) as writer:
        writer.append(_snapshot(10, 1_000_000_000, 0.0))
        writer.append(_snapshot(11, 1_020_000_000, 0.01))
        writer.append(_snapshot(12, 1_040_000_000, 0.02))

    service = RecordingService(store, ArtifactService(store))

    timeline = service.timeline(run_id, offset=1, limit=1)

    assert timeline == {
        "schema": "lingtu.sim.studio.recording-timeline.v1",
        "recording_id": timeline["recording_id"],
        "run_id": run_id,
        "session_id": "studio-session",
        "frame_count": 3,
        "duration_ns": 40_000_000,
        "page": {
            "offset": 1,
            "limit": 1,
            "returned": 1,
            "next_offset": 2,
        },
        "frames": [
            {
                "frame_index": 1,
                "relative_time_ns": 20_000_000,
                "sim_time_ns": 1_020_000_000,
                "sequence": 11,
                "physics_step": 88,
                "model_generation": 2,
                "reset_generation": 3,
                "body_count": 1,
                "joint_count": 0,
                "actuator_count": 0,
                "sensor_count": 0,
                "has_command": False,
                "scenario_event_count": 0,
                "sensor_metadata_count": 0,
                "sensor_payload_count": 0,
                "sensor_payload_bytes": 0,
                "lifecycle_evidence_count": 0,
            }
        ],
    }


def test_recording_frame_returns_one_validated_truth_snapshot(tmp_path: Path) -> None:
    store, run_id, root = _studio_run(tmp_path)
    with SimulationRecordingWriter(
        root,
        run_id=run_id,
        session_id="studio-session",
    ) as writer:
        writer.append(_snapshot(10, 1_000_000_000, 0.0))
        writer.append(_snapshot(11, 1_020_000_000, 0.25))

    service = RecordingService(store, ArtifactService(store))

    frame = service.frame(run_id, frame_index=1)

    assert frame["schema"] == "lingtu.sim.studio.recording-frame.v1"
    assert frame["recording_id"]
    assert frame["run_id"] == run_id
    assert frame["session_id"] == "studio-session"
    assert frame["frame_index"] == 1
    assert frame["relative_time_ns"] == 20_000_000
    assert frame["snapshot"]["sequence"] == 11
    assert frame["snapshot"]["sim_time_ns"] == 1_020_000_000
    assert frame["snapshot"]["bodies"][0]["stable_id"] == "robot_01/base_link"
    assert frame["snapshot"]["bodies"][0]["position_m"] == [0.25, 0.0, 0.5]
    assert frame["command"] is None
    assert frame["scenario_events"] == []
    assert frame["sensor_metadata"] == []
    assert frame["lifecycle_evidence"] == []


def test_recording_browse_exposes_sensor_payload_evidence_without_blob_paths(
    tmp_path: Path,
) -> None:
    store, run_id, root = _studio_run(tmp_path)
    sample = SensorPayloadSample(
        sensor_id="robot_01.front_rgb",
        stream_kind="rgb",
        encoding="rgb8",
        media_type="application/vnd.lingtu.rgb8",
        sample_sequence=4,
        sample_time_ns=1_000_000_000,
        payload=b"\x10\x20\x30\x40\x50\x60",
        metadata={"width": 2, "height": 1, "stride_bytes": 6},
    )
    with SimulationRecordingWriter(
        root,
        run_id=run_id,
        session_id="studio-session",
    ) as writer:
        writer.append(
            _snapshot(10, 1_000_000_000, 0.0),
            sensor_payloads=[sample],
        )
    service = RecordingService(store, ArtifactService(store))

    summary = service.timeline(run_id)["frames"][0]
    frame = service.frame(run_id, frame_index=0)

    assert summary["sensor_payload_count"] == 1
    assert summary["sensor_payload_bytes"] == 6
    assert frame["sensor_payloads"] == [
        {
            "schema": "lingtu.sim.sensor-payload-ref.v1",
            "payload_index": 0,
            "session_id": "studio-session",
            "model_generation": 2,
            "reset_generation": 3,
            "sensor_id": "robot_01.front_rgb",
            "stream_kind": "rgb",
            "encoding": "rgb8",
            "media_type": "application/vnd.lingtu.rgb8",
            "sample_sequence": 4,
            "sample_time_ns": 1_000_000_000,
            "sha256": "23a15a0f97c0d5eb4ef78610cad63efcf7c14979d145edfc2aa980672188c727",
            "bytes": 6,
            "metadata": {"width": 2, "height": 1, "stride_bytes": 6},
        }
    ]
    assert "path" not in frame["sensor_payloads"][0]


def test_recording_browse_rejects_unbounded_pages_and_unknown_frames(
    tmp_path: Path,
) -> None:
    store, run_id, root = _studio_run(tmp_path)
    with SimulationRecordingWriter(
        root,
        run_id=run_id,
        session_id="studio-session",
    ) as writer:
        writer.append(_snapshot(10, 1_000_000_000, 0.0))
    service = RecordingService(store, ArtifactService(store))

    with pytest.raises(ValueError, match="limit must be between 1 and 200"):
        service.timeline(run_id, limit=201)
    with pytest.raises(ValueError, match=r"outside 0\.\.0"):
        service.frame(run_id, frame_index=1)


def test_http_recording_browse_reports_corrupt_timeline_as_invalid(
    tmp_path: Path,
) -> None:
    from fastapi.testclient import TestClient

    store, run_id, root = _studio_run(tmp_path)
    with SimulationRecordingWriter(
        root,
        run_id=run_id,
        session_id="studio-session",
    ) as writer:
        writer.append(_snapshot(10, 1_000_000_000, 0.0))
    manifest = json.loads(
        root.joinpath("simulation-recording.json").read_text(encoding="utf-8")
    )
    manifest["schema"] = "corrupt"
    root.joinpath("simulation-recording.json").write_text(
        json.dumps(manifest),
        encoding="utf-8",
    )
    artifacts = ArtifactService(store)
    service = SimulationStudioService(
        store=store,
        artifact_service=artifacts,
        recording_service=RecordingService(store, artifacts),
    )

    with TestClient(create_app(service)) as client:
        response = client.get(
            f"{API_PREFIX}/runs/{run_id}/recording/timeline"
        )

    assert response.status_code == 422
    assert response.json()["error"]["code"] == "SIMSTUDIO_RECORDING_INVALID"
    assert "schema" in response.json()["error"]["message"]


def test_missing_and_corrupt_recordings_have_explicit_states(tmp_path: Path) -> None:
    store, run_id, root = _studio_run(tmp_path)
    service = RecordingService(store, ArtifactService(store))

    missing = service.inspect(run_id)

    assert missing["state"] == "MISSING"
    assert missing["recording_id"] is None
    assert missing["replay"] == {
        "deterministic": False,
        "visual": False,
        "clock_authority": None,
    }
    assert service.list_recordings()["recordings"] == []

    root.joinpath("simulation-recording.json").write_text(
        json.dumps({"schema": "lingtu.sim.recording.v1"}),
        encoding="utf-8",
    )
    invalid = service.inspect(run_id)

    assert invalid["state"] == "INVALID"
    assert invalid["recording_id"] is not None
    assert invalid["replay"]["deterministic"] is False
    assert invalid["diagnostics"][0]["code"] == "SIMSTUDIO_RECORDING_INVALID"
    assert service.list_recordings()["recordings"] == [invalid]


def test_recording_with_a_different_run_or_session_identity_is_invalid(
    tmp_path: Path,
) -> None:
    store, run_id, root = _studio_run(tmp_path)
    with SimulationRecordingWriter(
        root,
        run_id="source-recording",
        session_id="studio-session",
    ) as writer:
        writer.append(_snapshot(10, 1_000_000_000, 0.0))

    service = RecordingService(store, ArtifactService(store))
    wrong_run = service.inspect(run_id)
    assert wrong_run["state"] == "INVALID"
    assert "run_id" in wrong_run["diagnostics"][0]["message"]

    manifest_path = root / "simulation-recording.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["run_id"] = run_id
    manifest["session_id"] = "other-session"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    wrong_session = service.inspect(run_id)
    assert wrong_session["state"] == "INVALID"
    assert "session_id" in wrong_session["diagnostics"][0]["message"]


def test_http_exposes_recording_status_and_recording_library(tmp_path: Path) -> None:
    from fastapi.testclient import TestClient

    store, run_id, root = _studio_run(tmp_path)
    with SimulationRecordingWriter(
        root,
        run_id=run_id,
        session_id="studio-session",
    ) as writer:
        writer.append(_snapshot(10, 1_000_000_000, 0.0))
    artifacts = ArtifactService(store)
    service = SimulationStudioService(
        store=store,
        artifact_service=artifacts,
        recording_service=RecordingService(store, artifacts),
    )

    with TestClient(create_app(service)) as client:
        inspected = client.get(f"{API_PREFIX}/runs/{run_id}/recording")
        library = client.get(f"{API_PREFIX}/recordings")
        timeline = client.get(
            f"{API_PREFIX}/runs/{run_id}/recording/timeline",
            params={"offset": 0, "limit": 50},
        )
        frame = client.get(f"{API_PREFIX}/runs/{run_id}/recording/frames/0")
        capabilities = client.get(f"{API_PREFIX}/capabilities")

    assert inspected.status_code == 200
    assert inspected.json()["result"]["state"] == "VALID"
    assert library.status_code == 200
    assert [item["run_id"] for item in library.json()["result"]["recordings"]] == [
        run_id
    ]
    assert timeline.status_code == 200
    assert timeline.json()["result"]["frames"][0]["frame_index"] == 0
    assert timeline.json()["result"]["page"] == {
        "offset": 0,
        "limit": 50,
        "returned": 1,
        "next_offset": None,
    }
    assert frame.status_code == 200
    assert frame.json()["result"]["snapshot"]["sequence"] == 10
    assert capabilities.json()["result"]["read_models"]["recordings"] == {
        "list": f"{API_PREFIX}/recordings",
        "inspect": f"{API_PREFIX}/runs/{{run_id}}/recording",
        "timeline": f"{API_PREFIX}/runs/{{run_id}}/recording/timeline",
        "frame": f"{API_PREFIX}/runs/{{run_id}}/recording/frames/{{frame_index}}",
        "start": f"{API_PREFIX}/runs/{{run_id}}/recording/start",
        "stop": f"{API_PREFIX}/runs/{{run_id}}/recording/stop",
    }


class _ObservableSession:
    def __init__(self, **_: object) -> None:
        self.observers: dict[int, object] = {}
        self.next_token = 1
        self.sequence = 0
        self.sensor_payloads: tuple[SensorPayloadSample, ...] = ()

    def prepare(self) -> dict[str, object]:
        return {"readiness": {"physics": "ACTIVE"}}

    def start(self) -> dict[str, object]:
        return {"readiness": {"physics": "ACTIVE"}}

    def pause(self) -> dict[str, object]:
        return {"readiness": {"physics": "ACTIVE"}}

    def reset(self) -> dict[str, object]:
        return {"reset_generation": 1}

    def stop(self) -> dict[str, object]:
        return {"stopped": True}

    def attach_event_observer(
        self,
        observer: object,
        *,
        replay_latest_snapshot: bool = False,
    ) -> int:
        assert callable(observer)
        assert replay_latest_snapshot is True
        token = self.next_token
        self.next_token += 1
        self.observers[token] = observer
        return token

    def detach_event_observer(self, token: int) -> bool:
        return self.observers.pop(token, None) is not None

    def capture_sensor_payloads(
        self,
        _event: object,
    ) -> tuple[SensorPayloadSample, ...]:
        return self.sensor_payloads

    def emit_snapshot(self) -> None:
        self.sequence += 1
        event = _snapshot(
            self.sequence,
            1_000_000_000 + self.sequence * 20_000_000,
            self.sequence * 0.01,
        )
        for observer in tuple(self.observers.values()):
            assert callable(observer)
            observer(event)


def _recordable_run(
    tmp_path: Path,
) -> tuple[RunService, _ObservableSession, dict[str, object], Path]:
    store = StudioStore(tmp_path / "recordable-studio")
    bundle = store.create_bundle(
        {"bundle_path": "bundles/demo", "session_id": "studio-session"}
    )
    sessions: list[_ObservableSession] = []

    def factory(**kwargs: object) -> _ObservableSession:
        session = _ObservableSession(**kwargs)
        sessions.append(session)
        return session

    service = RunService(store, factory)
    run = service.create_run(bundle.id, "visual")
    prepared = service.prepare(run["id"], expected_revision=run["revision"])
    running = service.start(
        run["id"], expected_revision=prepared["revision"]
    )
    root = service.artifact_root / run["id"]
    return service, sessions[0], running, root


def test_run_can_capture_and_commit_a_bounded_truth_recording(
    tmp_path: Path,
) -> None:
    service, session, running, root = _recordable_run(tmp_path)

    capturing = service.start_recording(
        running["id"], expected_revision=running["revision"]
    )
    session.emit_snapshot()
    session.emit_snapshot()
    committed = service.stop_recording(
        running["id"], expected_revision=capturing["revision"]
    )

    assert capturing["status"] == "RUNNING"
    assert capturing["payload"]["recording"]["state"] == "CAPTURING"
    assert committed["payload"]["recording"] == {
        "state": "COMMITTED",
        "manifest": "simulation-recording.json",
        "timeline": "simulation-timeline.jsonl",
    }
    assert SimulationReplay.open(root).frame_count == 2
    service.stop(committed["id"], expected_revision=committed["revision"])


def test_run_recording_attaches_runtime_sensor_payloads_to_the_same_frame(
    tmp_path: Path,
) -> None:
    service, session, running, root = _recordable_run(tmp_path)
    session.sensor_payloads = (
        SensorPayloadSample(
            sensor_id="robot_01.front_rgb",
            stream_kind="rgb",
            encoding="rgb8",
            media_type="application/vnd.lingtu.rgb8",
            sample_sequence=7,
            sample_time_ns=1_020_000_000,
            payload=b"rendered-rgb",
            metadata={"width": 2, "height": 2},
        ),
    )

    capturing = service.start_recording(
        running["id"], expected_revision=running["revision"]
    )
    session.emit_snapshot()
    committed = service.stop_recording(
        running["id"], expected_revision=capturing["revision"]
    )

    replay = SimulationReplay.open(root)
    assert len(replay.frames[0].sensor_payloads) == 1
    assert replay.read_sensor_payload(
        replay.frames[0].sensor_payloads[0]
    ) == b"rendered-rgb"
    service.stop(committed["id"], expected_revision=committed["revision"])


def test_stopping_a_run_commits_its_active_recording(tmp_path: Path) -> None:
    service, session, running, root = _recordable_run(tmp_path)
    capturing = service.start_recording(
        running["id"], expected_revision=running["revision"]
    )
    session.emit_snapshot()

    stopped = service.stop(
        running["id"], expected_revision=capturing["revision"]
    )

    assert stopped["status"] == "STOPPED"
    assert stopped["payload"]["recording"]["state"] == "COMMITTED"
    assert SimulationReplay.open(root).frame_count == 1


def test_http_controls_a_bounded_recording_window(tmp_path: Path) -> None:
    from fastapi.testclient import TestClient

    run_service, session, running, root = _recordable_run(tmp_path)
    artifacts = ArtifactService(run_service.store)
    service = SimulationStudioService(
        store=run_service.store,
        run_service=run_service,
        artifact_service=artifacts,
        recording_service=RecordingService(run_service.store, artifacts),
    )

    with TestClient(create_app(service)) as client:
        capture_response = client.post(
            f"{API_PREFIX}/runs/{running['id']}/recording/start",
            json={"revision": running["revision"]},
        )
        assert capture_response.status_code == 200
        capturing = capture_response.json()["result"]
        session.emit_snapshot()
        commit_response = client.post(
            f"{API_PREFIX}/runs/{running['id']}/recording/stop",
            json={"revision": capturing["revision"]},
        )

    assert commit_response.status_code == 200
    assert commit_response.json()["result"]["payload"]["recording"]["state"] == (
        "COMMITTED"
    )
    assert SimulationReplay.open(root).frame_count == 1
    run_service.stop(running["id"])


def test_recording_operations_are_idempotent_across_response_retries(
    tmp_path: Path,
) -> None:
    service, session, running, _ = _recordable_run(tmp_path)

    capturing = service.start_recording(
        running["id"],
        expected_revision=running["revision"],
        idempotency_key="capture-once",
    )
    retried_capture = service.start_recording(
        running["id"],
        expected_revision=running["revision"],
        idempotency_key="capture-once",
    )
    assert retried_capture == capturing
    assert len(session.observers) == 1
    session.emit_snapshot()

    committed = service.stop_recording(
        running["id"],
        expected_revision=capturing["revision"],
        idempotency_key="commit-once",
    )
    retried_commit = service.stop_recording(
        running["id"],
        expected_revision=capturing["revision"],
        idempotency_key="commit-once",
    )
    assert retried_commit == committed
    assert session.observers == {}
    service.stop(committed["id"], expected_revision=committed["revision"])
