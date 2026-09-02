"""Public recording-to-replay behavior for simulation truth timelines."""

# ruff: noqa: S101

from __future__ import annotations

import hashlib
import json
import socket
from dataclasses import replace
from pathlib import Path

import pytest

from sim.runtime.recording import (
    SensorPayloadSample,
    SimulationRecordingError,
    SimulationRecordingWriter,
)
from sim.runtime.replay import (
    SimulationReplay,
    SimulationReplayError,
    compare_replays,
    replay_sensor_payloads,
    replay_snapshots,
)
from sim.runtime.replay.__main__ import main as replay_main


def _snapshot(sequence: int, sim_time_ns: int, x: float) -> dict[str, object]:
    return {
        "event": "snapshot",
        "session_id": "session-a",
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
        "joints": [{"stable_id": "robot_01/wheel", "qpos": [x], "qvel": [0.1]}],
        "actuators": [{"stable_id": "robot_01/wheel_motor", "control": 0.2}],
    }


def test_complete_recording_replays_full_snapshots_without_rewriting_sim_time(
    tmp_path: Path,
) -> None:
    with SimulationRecordingWriter(
        tmp_path,
        run_id="recording-01",
        session_id="session-a",
    ) as recorder:
        recorder.append(
            _snapshot(10, 1_000_000_000, 0.0),
            command={"channel_id": "robot_01.control.base_twist", "linear_x": 0.2},
        )
        recorder.append(
            _snapshot(11, 1_020_000_000, 0.004),
            command={"channel_id": "robot_01.control.base_twist", "linear_x": 0.2},
        )

    replay = SimulationReplay.open(tmp_path)
    received: list[dict[str, object]] = []
    report = replay_snapshots(replay, received.append, pace=False)

    assert replay.session_id == "session-a"
    assert replay.frame_count == 2
    assert replay.duration_ns == 20_000_000
    assert [frame.relative_time_ns for frame in replay.frames] == [0, 20_000_000]
    assert [frame.command["linear_x"] for frame in replay.frames] == [0.2, 0.2]
    assert [snapshot["sim_time_ns"] for snapshot in received] == [1_000_000_000, 1_020_000_000]
    assert received[1]["bodies"][0]["position_m"] == [0.004, 0.0, 0.5]
    assert report.frames_presented == 2
    assert report.frames_dropped == 0

    manifest = json.loads((tmp_path / "simulation-recording.json").read_text(encoding="utf-8"))
    assert manifest["schema"] == "lingtu.sim.recording.v1"
    assert manifest["timeline"]["path"] == "simulation-timeline.jsonl"
    assert len(manifest["timeline"]["sha256"]) == 64


def test_recording_manifest_binds_all_ordered_replay_inputs(tmp_path: Path) -> None:
    allocation = {
        "schema": "lingtu.sim.run-allocation.v1",
        "run_id": "recording-complete",
        "session_id": "session-a",
        "artifact_root": str(tmp_path),
        "boot_id": "boot-recording-complete",
        "dds_domain": 79,
        "ports": {"visual_snapshot_udp": 25123},
        "shm": {"front_rgb": "lingtu-recording-complete-rgb"},
        "log_dir": str(tmp_path / "logs"),
    }
    descriptors = {
        "physics.plan.json": {
            "schema": "lingtu.sim.physics-plan.v1",
            "session_id": "session-a",
        },
        "sensor.plan.json": {
            "schema": "lingtu.sim.sensor-plan.v1",
            "session_id": "session-a",
            "model_generation": 2,
            "reset_generation": 3,
        },
    }
    identity = {
        "session_id": "session-a",
        "model_generation": 2,
        "reset_generation": 3,
    }

    with SimulationRecordingWriter(
        tmp_path,
        run_id="recording-complete",
        session_id="session-a",
        run_allocation=allocation,
        descriptors=descriptors,
        required_content={"scenario_event", "sensor_metadata", "lifecycle_evidence"},
        continuous_tolerances={"/bodies/*/position_m/*": 1e-6},
    ) as recorder:
        recorder.append(
            _snapshot(10, 1_000_000_000, 0.0),
            command={"channel_id": "robot_01.control.base_twist", "linear_x": 0.2},
            scenario_events=[
                {
                    **identity,
                    "schema": "lingtu.sim.scenario-snapshot.v1",
                    "sequence": 10,
                    "sim_time_ns": 1_000_000_000,
                    "entities": [{"entity_id": "pedestrian_01", "state": "ACTIVE"}],
                }
            ],
            sensor_metadata=[
                {
                    **identity,
                    "schema": "lingtu.sim.sensor-sample-metadata.v1",
                    "sensor_id": "robot_01.front_rgb",
                    "sequence": 10,
                    "sim_time_ns": 1_000_000_000,
                    "sample_count": 1,
                }
            ],
            lifecycle_evidence=[
                {
                    **identity,
                    "schema": "lingtu.sim.lifecycle-evidence.v1",
                    "state": "RUNNING",
                    "sequence": 10,
                    "sim_time_ns": 1_000_000_000,
                }
            ],
        )
        recorder.append(
            _snapshot(11, 1_020_000_000, 0.004),
            lifecycle_evidence=[
                {
                    **identity,
                    "schema": "lingtu.sim.lifecycle-evidence.v1",
                    "state": "STOPPED",
                    "sequence": 11,
                    "sim_time_ns": 1_020_000_000,
                }
            ],
        )

    manifest = json.loads((tmp_path / "simulation-recording.json").read_text(encoding="utf-8"))
    assert manifest["run_allocation"]["document"] == allocation
    assert manifest["descriptors"]["documents"] == descriptors
    assert len(manifest["run_allocation"]["sha256"]) == 64
    assert len(manifest["descriptors"]["sha256"]) == 64
    assert manifest["content"]["required_streams"] == [
        "lifecycle_evidence",
        "scenario_event",
        "sensor_metadata",
    ]
    assert manifest["content"]["streams"]["command"]["count"] == 1
    assert manifest["content"]["streams"]["truth_snapshot"]["count"] == 2
    assert manifest["content"]["streams"]["scenario_event"]["count"] == 1
    assert manifest["content"]["streams"]["sensor_metadata"]["count"] == 1
    assert manifest["content"]["streams"]["lifecycle_evidence"]["count"] == 2

    replay = SimulationReplay.open(tmp_path)
    assert replay.event_order == (
        "command",
        "scenario_event",
        "truth_snapshot",
        "sensor_metadata",
        "lifecycle_evidence",
        "truth_snapshot",
        "lifecycle_evidence",
    )
    assert replay.terminal_state == "STOPPED"
    assert dict(replay.continuous_tolerances) == {"/bodies/*/position_m/*": 1e-6}


def test_two_replays_match_discrete_state_exactly_and_physics_within_tolerance(
    tmp_path: Path,
) -> None:
    identity = {
        "session_id": "session-a",
        "model_generation": 2,
        "reset_generation": 3,
    }
    for directory, x in (
        (tmp_path / "first", 0.0),
        (tmp_path / "second", 5e-7),
        (tmp_path / "outside-tolerance", 2e-6),
    ):
        with SimulationRecordingWriter(
            directory,
            run_id="deterministic-recording",
            session_id="session-a",
            required_content={"lifecycle_evidence"},
            continuous_tolerances={
                "/bodies/*/position_m/*": 1e-6,
                "/joints/*/qpos/*": 1e-6,
            },
        ) as recorder:
            recorder.append(
                _snapshot(10, 1_000_000_000, x),
                lifecycle_evidence=[
                    {
                        **identity,
                        "schema": "lingtu.sim.lifecycle-evidence.v1",
                        "state": "STOPPED",
                        "sequence": 10,
                        "sim_time_ns": 1_000_000_000,
                    }
                ],
            )

    first = SimulationReplay.open(tmp_path / "first")
    second = SimulationReplay.open(tmp_path / "second")
    first_report = replay_snapshots(first, lambda _snapshot: True, pace=False)
    second_report = replay_snapshots(second, lambda _snapshot: True, pace=False)
    comparison = compare_replays(first, second)

    assert first_report.event_order == second_report.event_order
    assert first_report.generations == second_report.generations
    assert first_report.terminal_state == second_report.terminal_state == "STOPPED"
    assert first_report.clock_authority == second_report.clock_authority == "recorded_mujoco"
    assert comparison.discrete_match is True
    assert comparison.continuous_match is True
    assert comparison.equivalent is True
    assert comparison.continuous_fields_compared > 0
    assert comparison.clock_authority == "recorded_mujoco"

    outside = compare_replays(
        first,
        SimulationReplay.open(tmp_path / "outside-tolerance"),
    )
    assert outside.discrete_match is True
    assert outside.continuous_match is False
    assert outside.equivalent is False
    assert any("position_m" in mismatch for mismatch in outside.mismatches)

def test_self_reported_wide_tolerances_cannot_authorize_nondeterminism(
    tmp_path: Path,
) -> None:
    for directory, x in ((tmp_path / "first", 0.0), (tmp_path / "second", 999.0)):
        with SimulationRecordingWriter(
            directory,
            run_id="untrusted-tolerance",
            session_id="session-a",
            continuous_tolerances={"/bodies/*/position_m/*": 1000.0},
        ) as recorder:
            recorder.append(_snapshot(10, 1_000_000_000, x))

    for directory in (tmp_path / "first", tmp_path / "second"):
        with pytest.raises(
            SimulationReplayError,
            match="trusted maximum",
        ):
            SimulationReplay.open(directory)

        manifest_path = directory / "simulation-recording.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["continuous_tolerances"] = {}
        manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")

    first = SimulationReplay.open(tmp_path / "first")
    second = SimulationReplay.open(tmp_path / "second")
    comparison = compare_replays(
        replace(
            first,
            continuous_tolerances={"/bodies/*/position_m/*": 1000.0},
        ),
        replace(
            second,
            continuous_tolerances={"/bodies/*/position_m/*": 1000.0},
        ),
    )

    assert comparison.equivalent is False
    assert comparison.continuous_match is False
    assert any("trusted maximum" in mismatch for mismatch in comparison.mismatches)


def test_unknown_continuous_tolerance_path_is_not_trusted(tmp_path: Path) -> None:
    with SimulationRecordingWriter(
        tmp_path,
        run_id="unknown-tolerance",
        session_id="session-a",
        continuous_tolerances={"/metadata/operator_override": 1e-12},
    ) as recorder:
        recorder.append(_snapshot(10, 1_000_000_000, 0.0))

    with pytest.raises(SimulationReplayError, match="path is not trusted"):
        SimulationReplay.open(tmp_path)


def test_replay_rejects_rehashed_command_that_breaks_the_content_manifest(
    tmp_path: Path,
) -> None:
    with SimulationRecordingWriter(
        tmp_path,
        run_id="recording-content-hash",
        session_id="session-a",
    ) as recorder:
        recorder.append(
            _snapshot(10, 1_000_000_000, 0.0),
            command={"channel_id": "robot_01.control.base_twist", "linear_x": 0.2},
        )

    timeline_path = tmp_path / "simulation-timeline.jsonl"
    frame = json.loads(timeline_path.read_text(encoding="utf-8"))
    frame["command"]["linear_x"] = 0.9
    payload = (
        json.dumps(frame, sort_keys=True, separators=(",", ":")) + "\n"
    ).encode("utf-8")
    timeline_path.write_bytes(payload)
    manifest_path = tmp_path / "simulation-recording.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["timeline"].update(
        {"bytes": len(payload), "sha256": hashlib.sha256(payload).hexdigest()}
    )
    manifest_path.write_text(json.dumps(manifest, sort_keys=True), encoding="utf-8")

    with pytest.raises(SimulationReplayError, match="command content hash"):
        SimulationReplay.open(tmp_path)


def test_recording_required_content_must_be_nonempty_and_terminal(
    tmp_path: Path,
) -> None:
    with pytest.raises(SimulationRecordingError, match="scenario_event"):
        with SimulationRecordingWriter(
            tmp_path / "missing-scenario",
            run_id="missing-scenario",
            session_id="session-a",
            required_content={"scenario_event"},
        ) as recorder:
            recorder.append(_snapshot(10, 1_000_000_000, 0.0))

    identity = {
        "session_id": "session-a",
        "model_generation": 2,
        "reset_generation": 3,
    }
    with pytest.raises(SimulationRecordingError, match="terminal STOPPED or FAILED"):
        with SimulationRecordingWriter(
            tmp_path / "nonterminal",
            run_id="nonterminal",
            session_id="session-a",
            required_content={"lifecycle_evidence"},
        ) as recorder:
            recorder.append(
                _snapshot(10, 1_000_000_000, 0.0),
                lifecycle_evidence=[
                    {
                        **identity,
                        "schema": "lingtu.sim.lifecycle-evidence.v1",
                        "state": "RUNNING",
                        "sequence": 10,
                        "sim_time_ns": 1_000_000_000,
                    }
                ],
            )

    assert not (tmp_path / "missing-scenario" / "simulation-recording.json").exists()
    assert not (tmp_path / "nonterminal" / "simulation-recording.json").exists()


def test_recording_rejects_scenario_generation_mismatch_before_commit(
    tmp_path: Path,
) -> None:
    recorder = SimulationRecordingWriter(
        tmp_path,
        run_id="stale-scenario",
        session_id="session-a",
    )
    with pytest.raises(SimulationRecordingError, match="model_generation"):
        recorder.append(
            _snapshot(10, 1_000_000_000, 0.0),
            scenario_events=[
                {
                    "schema": "lingtu.sim.scenario-snapshot.v1",
                    "session_id": "session-a",
                    "model_generation": 1,
                    "reset_generation": 3,
                    "sequence": 10,
                    "sim_time_ns": 1_000_000_000,
                    "entities": [],
                }
            ],
        )
    recorder.abort()

    assert not (tmp_path / "simulation-recording.json").exists()
    assert not (tmp_path / "simulation-timeline.jsonl").exists()


def test_replay_rejects_unknown_manifest_controls(tmp_path: Path) -> None:
    with SimulationRecordingWriter(
        tmp_path,
        run_id="recording-01",
        session_id="session-a",
    ) as recorder:
        recorder.append(_snapshot(10, 1_000_000_000, 0.0))

    manifest_path = tmp_path / "simulation-recording.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["executable"] = "C:/untrusted/player.exe"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(SimulationReplayError, match="unknown"):
        SimulationReplay.open(tmp_path)


def test_replay_revalidates_snapshot_clock_even_if_attacker_rehashes_timeline(
    tmp_path: Path,
) -> None:
    with SimulationRecordingWriter(
        tmp_path,
        run_id="recording-01",
        session_id="session-a",
    ) as recorder:
        recorder.append(_snapshot(10, 1_000_000_000, 0.0))
        recorder.append(_snapshot(11, 1_020_000_000, 0.004))

    timeline_path = tmp_path / "simulation-timeline.jsonl"
    frames = [json.loads(line) for line in timeline_path.read_text(encoding="utf-8").splitlines()]
    frames[1]["snapshot"]["sim_time_ns"] = 900_000_000
    payload = "".join(
        json.dumps(frame, sort_keys=True, separators=(",", ":")) + "\n"
        for frame in frames
    ).encode("utf-8")
    timeline_path.write_bytes(payload)

    manifest_path = tmp_path / "simulation-recording.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["clock"]["end_sim_time_ns"] = 900_000_000
    manifest["timeline"]["bytes"] = len(payload)
    manifest["timeline"]["sha256"] = hashlib.sha256(payload).hexdigest()
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(SimulationReplayError, match="backwards"):
        SimulationReplay.open(tmp_path)


def test_replay_cli_streams_recorded_truth_to_the_loopback_visual_transport(
    tmp_path: Path,
) -> None:
    with SimulationRecordingWriter(
        tmp_path,
        run_id="recording-01",
        session_id="session-a",
    ) as recorder:
        recorder.append(_snapshot(10, 1_000_000_000, 0.0))
        recorder.append(_snapshot(11, 1_020_000_000, 0.004))

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(2.0)
        port = receiver.getsockname()[1]

        assert replay_main([str(tmp_path), "--port", str(port), "--no-pace"]) == 0
        documents = [json.loads(receiver.recv(60_000)) for _ in range(2)]

    assert [document["sequence"] for document in documents] == [10, 11]
    assert all(document["schema"] == "lingtu.sim.truth-snapshot.v1" for document in documents)


def test_sensor_payloads_are_content_addressed_and_replayed_without_json_embedding(
    tmp_path: Path,
) -> None:
    payload = bytes([12, 34, 56, 78, 90, 123])
    first = SensorPayloadSample(
        sensor_id="robot_01.front_rgb",
        stream_kind="rgb",
        encoding="rgb8",
        media_type="application/vnd.lingtu.rgb8",
        sample_sequence=41,
        sample_time_ns=1_000_000_000,
        payload=payload,
        metadata={"width": 2, "height": 1, "stride_bytes": 6},
    )
    second = replace(first, sample_sequence=42, sample_time_ns=1_020_000_000)

    with SimulationRecordingWriter(
        tmp_path,
        run_id="sensor-payload-recording",
        session_id="session-a",
        required_content={"sensor_payload"},
    ) as recorder:
        recorder.append(
            _snapshot(10, 1_000_000_000, 0.0),
            sensor_payloads=[first],
        )
        recorder.append(
            _snapshot(11, 1_020_000_000, 0.004),
            sensor_payloads=[second],
        )

    manifest = json.loads(
        (tmp_path / "simulation-recording.json").read_text(encoding="utf-8")
    )
    assert manifest["content"]["streams"]["sensor_payload"]["count"] == 2
    assert manifest["sensor_payloads"] == {
        "schema": "lingtu.sim.sensor-payload-store.v1",
        "root": "sensor-payloads/sha256",
        "reference_count": 2,
        "unique_blob_count": 1,
        "referenced_bytes": len(payload) * 2,
        "unique_bytes": len(payload),
    }

    frames = [
        json.loads(line)
        for line in (tmp_path / "simulation-timeline.jsonl")
        .read_text(encoding="utf-8")
        .splitlines()
    ]
    references = [frame["evidence"]["sensor_payloads"][0] for frame in frames]
    assert references[0]["path"] == references[1]["path"]
    assert all("payload" not in reference for reference in references)

    replay = SimulationReplay.open(tmp_path)
    assert replay.read_sensor_payload(replay.frames[0].sensor_payloads[0]) == payload
    presented: list[tuple[str, bytes]] = []
    report = replay_sensor_payloads(
        replay,
        lambda reference, content: presented.append((reference["sensor_id"], content)),
        pace=False,
    )
    assert presented == [
        ("robot_01.front_rgb", payload),
        ("robot_01.front_rgb", payload),
    ]
    assert report.samples_presented == 2
    assert report.samples_dropped == 0
    assert report.bytes_presented == len(payload) * 2


def test_sensor_payload_replay_rejects_tampered_or_escaping_blob_references(
    tmp_path: Path,
) -> None:
    sample = SensorPayloadSample(
        sensor_id="robot_01.front_depth",
        stream_kind="depth",
        encoding="16UC1",
        media_type="application/vnd.lingtu.depth16",
        sample_sequence=8,
        sample_time_ns=1_000_000_000,
        payload=b"\x01\x02\x03\x04",
        metadata={"width": 1, "height": 2, "stride_bytes": 2},
    )
    tampered = tmp_path / "tampered"
    with SimulationRecordingWriter(
        tampered,
        run_id="tampered-payload",
        session_id="session-a",
    ) as recorder:
        recorder.append(_snapshot(10, 1_000_000_000, 0.0), sensor_payloads=[sample])

    frame = json.loads(
        (tampered / "simulation-timeline.jsonl").read_text(encoding="utf-8")
    )
    payload_path = tampered / Path(frame["evidence"]["sensor_payloads"][0]["path"])
    payload_path.write_bytes(b"\x04\x03\x02\x01")
    with pytest.raises(SimulationReplayError, match="sensor payload SHA-256"):
        SimulationReplay.open(tampered)

    escaping = tmp_path / "escaping"
    with SimulationRecordingWriter(
        escaping,
        run_id="escaping-payload",
        session_id="session-a",
    ) as recorder:
        recorder.append(_snapshot(10, 1_000_000_000, 0.0), sensor_payloads=[sample])
    timeline_path = escaping / "simulation-timeline.jsonl"
    escaped_frame = json.loads(timeline_path.read_text(encoding="utf-8"))
    escaped_frame["evidence"]["sensor_payloads"][0]["path"] = "../outside.bin"
    timeline_payload = (
        json.dumps(escaped_frame, sort_keys=True, separators=(",", ":")) + "\n"
    ).encode("utf-8")
    timeline_path.write_bytes(timeline_payload)
    manifest_path = escaping / "simulation-recording.json"
    escaped_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    escaped_manifest["timeline"]["bytes"] = len(timeline_payload)
    escaped_manifest["timeline"]["sha256"] = hashlib.sha256(timeline_payload).hexdigest()
    manifest_path.write_text(json.dumps(escaped_manifest), encoding="utf-8")

    with pytest.raises(SimulationReplayError, match="sensor payload path"):
        SimulationReplay.open(escaping)
