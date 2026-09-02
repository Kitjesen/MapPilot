"""Internal contract tests for typed-DDS payload recording and replay."""

# ruff: noqa: S101

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from sim.runtime.recording import (
    SimulationRecordingError,
    SimulationRecordingWriter,
    TypedDdsPayloadSample,
)
from sim.runtime.replay import (
    SimulationReplay,
    SimulationReplayError,
    compare_replays,
    replay_typed_dds_payloads,
)


def _snapshot(sequence: int, sim_time_ns: int) -> dict[str, object]:
    return {
        "event": "snapshot",
        "session_id": "b" * 64,
        "model_generation": 7,
        "reset_generation": 2,
        "sequence": sequence,
        "physics_step": sequence,
        "sim_time_ns": sim_time_ns,
        "bodies": [],
    }


def _sample(sequence: int, sim_time_ns: int, payload: bytes) -> TypedDdsPayloadSample:
    return TypedDdsPayloadSample(
        topic="rt/sim/imu",
        type_name="lingtu::sim::ImuSample",
        encoding="cdr",
        model_generation=7,
        reset_generation=2,
        sequence=sequence,
        sim_time_ns=sim_time_ns,
        payload=payload,
        metadata={"sensor_id": "thunder_01.imu", "frame_id": "imu_link"},
    )


def test_typed_dds_payloads_are_content_addressed_with_replay_metadata(
    tmp_path: Path,
) -> None:
    payload = b"cdr-imu-payload"
    with SimulationRecordingWriter(
        tmp_path,
        run_id="typed-dds-recording",
        session_id="b" * 64,
        required_content={"typed_dds_payload"},
    ) as recorder:
        recorder.append(
            _snapshot(10, 1_000_000_000),
            typed_dds_payloads=[_sample(101, 1_000_000_000, payload)],
        )
        recorder.append(
            _snapshot(11, 1_020_000_000),
            typed_dds_payloads=[_sample(102, 1_020_000_000, payload)],
        )

    manifest = json.loads(
        (tmp_path / "simulation-recording.json").read_text(encoding="utf-8")
    )
    assert manifest["content"]["streams"]["typed_dds_payload"]["count"] == 2
    assert manifest["typed_dds_payloads"] == {
        "schema": "lingtu.sim.typed-dds-payload-store.v1",
        "root": "typed-dds-payloads/sha256",
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
    references = [frame["evidence"]["typed_dds_payloads"][0] for frame in frames]
    assert references[0]["path"] == references[1]["path"]
    assert references[0]["sha256"] == hashlib.sha256(payload).hexdigest()
    assert references[0]["topic"] == "rt/sim/imu"
    assert references[0]["type_name"] == "lingtu::sim::ImuSample"
    assert references[0]["model_generation"] == 7
    assert references[0]["sim_time_ns"] == 1_000_000_000
    assert references[0]["sequence"] == 101
    assert "payload" not in references[0]

    replay = SimulationReplay.open(tmp_path)
    assert replay.read_typed_dds_payload(replay.frames[0].typed_dds_payloads[0]) == payload
    presented: list[tuple[str, str, int, int, bytes]] = []
    report = replay_typed_dds_payloads(
        replay,
        lambda reference, content: presented.append(
            (
                reference["topic"],
                reference["type_name"],
                reference["sequence"],
                reference["sim_time_ns"],
                content,
            )
        ),
        pace=False,
    )
    assert presented == [
        ("rt/sim/imu", "lingtu::sim::ImuSample", 101, 1_000_000_000, payload),
        ("rt/sim/imu", "lingtu::sim::ImuSample", 102, 1_020_000_000, payload),
    ]
    assert report.samples_presented == 2
    assert report.bytes_presented == len(payload) * 2


def test_typed_dds_replay_rejects_payload_tampering(tmp_path: Path) -> None:
    with SimulationRecordingWriter(
        tmp_path,
        run_id="typed-dds-tamper",
        session_id="b" * 64,
    ) as recorder:
        recorder.append(
            _snapshot(10, 1_000_000_000),
            typed_dds_payloads=[_sample(101, 1_000_000_000, b"original")],
        )

    frame = json.loads(
        (tmp_path / "simulation-timeline.jsonl").read_text(encoding="utf-8")
    )
    payload_path = tmp_path / Path(frame["evidence"]["typed_dds_payloads"][0]["path"])
    payload_path.write_bytes(b"tampered")
    with pytest.raises(SimulationReplayError, match="typed DDS payload SHA-256"):
        SimulationReplay.open(tmp_path)


def test_independent_replay_comparison_includes_typed_dds_payloads(
    tmp_path: Path,
) -> None:
    for name, payload in (("first", b"cdr-a"), ("second", b"cdr-b")):
        with SimulationRecordingWriter(
            tmp_path / name,
            run_id="typed-dds-determinism",
            session_id="b" * 64,
        ) as recorder:
            recorder.append(
                _snapshot(10, 1_000_000_000),
                typed_dds_payloads=[_sample(101, 1_000_000_000, payload)],
            )

    comparison = compare_replays(
        SimulationReplay.open(tmp_path / "first"),
        SimulationReplay.open(tmp_path / "second"),
    )

    assert comparison.equivalent is False
    assert comparison.discrete_match is False
    assert "discrete:frames/0/typed_dds_payloads" in comparison.mismatches


def test_writer_rejects_preexisting_typed_dds_store_without_deleting_it(
    tmp_path: Path,
) -> None:
    sentinel = tmp_path / "typed-dds-payloads" / "sha256" / "sentinel.bin"
    sentinel.parent.mkdir(parents=True)
    sentinel.write_bytes(b"keep")

    with pytest.raises(SimulationRecordingError, match="artifacts already exist"):
        with SimulationRecordingWriter(
            tmp_path,
            run_id="typed-dds-existing-store",
            session_id="b" * 64,
        ):
            pass

    assert sentinel.read_bytes() == b"keep"


def test_writer_rejects_future_or_non_monotonic_typed_dds_samples(
    tmp_path: Path,
) -> None:
    future = _sample(101, 1_000_000_001, b"future")
    with pytest.raises(SimulationRecordingError, match="after its truth snapshot"):
        with SimulationRecordingWriter(
            tmp_path / "future",
            run_id="typed-dds-future",
            session_id="b" * 64,
        ) as recorder:
            recorder.append(
                _snapshot(10, 1_000_000_000),
                typed_dds_payloads=[future],
            )

    with pytest.raises(SimulationRecordingError, match="sequence must increase"):
        with SimulationRecordingWriter(
            tmp_path / "duplicate",
            run_id="typed-dds-duplicate",
            session_id="b" * 64,
        ) as recorder:
            recorder.append(
                _snapshot(10, 1_000_000_000),
                typed_dds_payloads=[_sample(101, 1_000_000_000, b"first")],
            )
            recorder.append(
                _snapshot(11, 1_020_000_000),
                typed_dds_payloads=[_sample(101, 1_020_000_000, b"duplicate")],
            )
