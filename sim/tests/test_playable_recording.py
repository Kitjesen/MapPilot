# ruff: noqa: S101

from __future__ import annotations

import json
import threading
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any

import pytest

from sim.runtime.control.contracts import CommandSubmitResult, ControllerCommand
from sim.runtime.coordinator.control_intent_udp import (
    BoundedRuntimeRequestInbox,
    LatestOperatorIntentInbox,
    OperatorIntentIdentity,
    OperatorRuntimeRequest,
)
from sim.runtime.coordinator.controlled_run import BaseTwistTarget
from sim.runtime.coordinator.interactive_session import InteractiveControlRequest
from sim.runtime.coordinator.playable_control import PlayableControlPump
from sim.runtime.coordinator.playable_recording import (
    InteractiveRecordingController,
    PlayableRecordingError,
    RecordingLifecycleState,
)
from sim.runtime.recording import RECORDING_FILENAME, SensorPayloadSample


def _snapshot(sequence: int, sim_time_ns: int) -> dict[str, Any]:
    return {
        "event": "snapshot",
        "session_id": "session-a",
        "model_generation": 3,
        "reset_generation": 2,
        "sequence": sequence,
        "physics_step": sequence,
        "sim_time_ns": sim_time_ns,
        "bodies": [],
    }


class _OrderedSession:
    def __init__(self, latest: Mapping[str, Any] | None) -> None:
        self.model_generation = 3
        self.reset_generation = 2
        self._latest = None if latest is None else dict(latest)
        self._observers: dict[int, Callable[[Mapping[str, Any]], object]] = {}
        self._next_token = 1
        self.capture_calls: list[int] = []
        self.background_thread_alive = False

    def attach_event_observer(
        self,
        observer: Callable[[Mapping[str, Any]], object],
        *,
        replay_latest_snapshot: bool = False,
    ) -> int:
        if replay_latest_snapshot and self._latest is not None:
            observer(dict(self._latest))
        token = self._next_token
        self._next_token += 1
        self._observers[token] = observer
        return token

    def detach_event_observer(self, token: int) -> bool:
        return self._observers.pop(token, None) is not None

    def capture_sensor_payloads(
        self,
        snapshot: Mapping[str, Any],
    ) -> Sequence[SensorPayloadSample]:
        sequence = int(snapshot["sequence"])
        self.capture_calls.append(sequence)
        return (
            SensorPayloadSample(
                sensor_id="thunder_01.front_rgb",
                stream_kind="rgb",
                encoding="rgb8",
                media_type="application/octet-stream",
                sample_sequence=sequence,
                sample_time_ns=int(snapshot["sim_time_ns"]),
                payload=f"frame-{sequence}".encode(),
            ),
        )

    def emit(self, event: Mapping[str, Any]) -> None:
        self._latest = dict(event)
        for observer in tuple(self._observers.values()):
            observer(dict(event))

    @property
    def observer_count(self) -> int:
        return len(self._observers)


class _MemoryWriter:
    def __init__(
        self,
        manifest_path: Path,
        *,
        fail_append_at: int | None = None,
        close_error: BaseException | None = None,
    ) -> None:
        self.manifest_path = manifest_path
        self.appended: list[dict[str, Any]] = []
        self.abort_count = 0
        self.fail_append_at = fail_append_at
        self.close_error = close_error

    def append(
        self,
        snapshot_event: Mapping[str, Any],
        *,
        sensor_payloads: Sequence[SensorPayloadSample] | None = None,
    ) -> None:
        assert sensor_payloads
        if self.fail_append_at == len(self.appended):
            raise OSError("append exploded")
        self.appended.append(dict(snapshot_event))

    def close(self) -> Path:
        if self.close_error is not None:
            raise self.close_error
        return self.manifest_path

    def abort(self) -> None:
        self.abort_count += 1


class _MemoryWriterFactory:
    def __init__(self, writer: _MemoryWriter) -> None:
        self.writer = writer

    def __call__(self, *_: Any, **__: Any) -> _MemoryWriter:
        return self.writer


class _FailingCaptureSession(_OrderedSession):
    def capture_sensor_payloads(
        self,
        snapshot: Mapping[str, Any],
    ) -> Sequence[SensorPayloadSample]:
        del snapshot
        raise OSError("capture exploded")


def _allocation(run_dir: Path) -> dict[str, Any]:
    return {
        "schema": "lingtu.sim.run-allocation.v1",
        "run_id": "run-playable-001",
        "session_id": "session-a",
        "artifact_root": str(run_dir),
        "boot_id": "boot-playable-001",
        "dds_domain": 17,
        "ports": {},
        "shm": {},
        "log_dir": str(run_dir / "logs"),
    }


def _runtime_request(request: str, sequence: int) -> OperatorRuntimeRequest:
    return OperatorRuntimeRequest(
        identity=OperatorIntentIdentity(
            run_id="run-playable-001",
            session_id="session-a",
            boot_id="boot-playable-001",
            model_generation=3,
            reset_generation=2,
            source_id="robotsimue.local_player.0",
        ),
        source_epoch=1,
        source_sequence=sequence,
        event_id=f"boot-playable-001:1:{sequence}",
        request=request,
        source_monotonic_ns=sequence,
        arrival_monotonic_ns=sequence,
        datagram_sha256=f"{sequence % 16:x}" * 64,
        ui_mode="menu",
        camera_mode="follow",
    )


class _PumpHost:
    def submit_controller_command(
        self,
        _controller_id: str,
        _command: ControllerCommand,
    ) -> CommandSubmitResult:
        return CommandSubmitResult.ACCEPTED

    def control_status_authority_snapshot(
        self,
        snapshot: Mapping[str, Any],
        *,
        recording_snapshot: Mapping[str, Any] | None = None,
    ) -> Mapping[str, Any]:
        unavailable_facet = {
            "state": "UNAVAILABLE",
            "required": True,
            "source_id": "unavailable",
            "blocker": "test fixture does not own this runtime facet",
        }
        return {
            "run_id": "run-playable-001",
            "session_id": "session-a",
            "boot_id": "boot-playable-001",
            "model_generation": 3,
            "reset_generation": 2,
            "runtime_state": "RUNNING",
            "sim_time_ns": snapshot["sim_time_ns"],
            "truth_sequence": snapshot["sequence"],
            "observed_base_velocity_mps_radps": {
                "available": False,
                "linear_x": 0.0,
                "linear_y": 0.0,
                "angular_z": 0.0,
            },
            "readiness": {
                name: dict(unavailable_facet)
                for name in ("physics", "control", "visual", "sensors")
            },
            "sensors": [
                {
                    "stream_id": stream_id,
                    "state": "UNAVAILABLE",
                    "sample_count": 0,
                    "blocker": "test fixture has no sensor source",
                }
                for stream_id in (
                    "thunder_01.front_depth",
                    "thunder_01.front_rgb",
                    "thunder_01.imu",
                    "thunder_01.mid360",
                    "thunder_01.truth_odom",
                )
            ],
            "recording": (
                {
                    "state": "unavailable",
                    "elapsed_sim_time_ns": 0,
                    "artifact_id": "",
                    "blocker": "recording status source unavailable",
                }
                if recording_snapshot is None
                else {
                    key: recording_snapshot[key]
                    for key in (
                        "state",
                        "elapsed_sim_time_ns",
                        "artifact_id",
                        "blocker",
                    )
                }
            ),
        }


class _AckPublisher:
    def __init__(self) -> None:
        self.documents: list[dict[str, Any]] = []

    def publish(self, document: Mapping[str, Any]) -> int:
        self.documents.append(dict(document))
        return 1


def test_start_replays_latest_snapshot_and_commit_publishes_real_manifest(
    tmp_path: Path,
) -> None:
    run_dir = tmp_path / "run-playable-001"
    session = _OrderedSession(_snapshot(10, 1_000_000_000))
    controller = InteractiveRecordingController(
        run_dir=run_dir,
        run_id="run-playable-001",
        session_id="session-a",
        allocation_provider=lambda: _allocation(run_dir),
    )
    controller.bind_session(session)

    started = controller.start(model_generation=3, reset_generation=2)
    assert started.lifecycle_state is RecordingLifecycleState.CAPTURING
    assert started.state == "recording"
    assert started.artifact_id == RECORDING_FILENAME
    assert not (run_dir / RECORDING_FILENAME).exists()

    session.emit(_snapshot(11, 1_020_000_000))
    committed = controller.commit(model_generation=3, reset_generation=2)

    assert committed.lifecycle_state is RecordingLifecycleState.COMMITTED
    assert committed.state == "committed"
    assert committed.elapsed_sim_time_ns == 20_000_000
    assert committed.artifact_id == RECORDING_FILENAME
    assert committed.blocker == ""
    assert session.capture_calls == [10, 11]

    manifest_path = run_dir / RECORDING_FILENAME
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema"] == "lingtu.sim.recording.v1"
    assert manifest["run_id"] == "run-playable-001"
    assert manifest["session_id"] == "session-a"
    assert manifest["timeline"]["frame_count"] == 2
    assert manifest["sensor_payloads"]["reference_count"] == 2

    session.emit(_snapshot(12, 1_040_000_000))
    assert controller.status_snapshot() == committed


def test_duplicate_snapshot_fails_detaches_aborts_and_blocks_later_append(
    tmp_path: Path,
) -> None:
    run_dir = tmp_path / "run-playable-001"
    session = _OrderedSession(_snapshot(10, 1_000_000_000))
    writer = _MemoryWriter(run_dir / RECORDING_FILENAME)
    controller = InteractiveRecordingController(
        run_dir=run_dir,
        run_id="run-playable-001",
        session_id="session-a",
        allocation_provider=lambda: _allocation(run_dir),
        writer_factory=_MemoryWriterFactory(writer),
    )
    controller.bind_session(session)
    controller.start(model_generation=3, reset_generation=2)

    with pytest.raises(PlayableRecordingError, match="stale or duplicated"):
        session.emit(_snapshot(10, 1_000_000_000))

    failed = controller.status_snapshot()
    assert failed.lifecycle_state is RecordingLifecycleState.FAILED
    assert failed.state == "failed"
    assert "recording_snapshot_not_ordered" not in failed.blocker
    assert "stale or duplicated" in failed.blocker
    assert failed.artifact_id == ""
    assert writer.abort_count == 1
    assert session.observer_count == 0
    assert len(writer.appended) == 1

    session.emit(_snapshot(11, 1_020_000_000))
    assert len(writer.appended) == 1


def test_writer_append_failure_is_authoritative_and_detaches_before_more_truth(
    tmp_path: Path,
) -> None:
    run_dir = tmp_path / "run-playable-001"
    session = _OrderedSession(_snapshot(10, 1_000_000_000))
    writer = _MemoryWriter(
        run_dir / RECORDING_FILENAME,
        fail_append_at=1,
    )
    controller = InteractiveRecordingController(
        run_dir=run_dir,
        run_id="run-playable-001",
        session_id="session-a",
        allocation_provider=lambda: _allocation(run_dir),
        writer_factory=_MemoryWriterFactory(writer),
    )
    controller.bind_session(session)
    controller.start(model_generation=3, reset_generation=2)

    with pytest.raises(PlayableRecordingError, match="append exploded"):
        session.emit(_snapshot(11, 1_020_000_000))

    failed = controller.status_snapshot()
    assert failed.state == "failed"
    assert failed.artifact_id == ""
    assert "append exploded" in failed.blocker
    assert session.observer_count == 0
    assert writer.abort_count == 1
    assert len(writer.appended) == 1
    session.emit(_snapshot(12, 1_040_000_000))
    assert len(writer.appended) == 1


def test_replayed_capture_failure_aborts_exactly_once_and_never_attaches(
    tmp_path: Path,
) -> None:
    run_dir = tmp_path / "run-playable-001"
    session = _FailingCaptureSession(_snapshot(10, 1_000_000_000))
    writer = _MemoryWriter(run_dir / RECORDING_FILENAME)
    controller = InteractiveRecordingController(
        run_dir=run_dir,
        run_id="run-playable-001",
        session_id="session-a",
        allocation_provider=lambda: _allocation(run_dir),
        writer_factory=_MemoryWriterFactory(writer),
    )
    controller.bind_session(session)

    with pytest.raises(PlayableRecordingError, match="capture exploded"):
        controller.start(model_generation=3, reset_generation=2)

    failed = controller.status_snapshot()
    assert failed.state == "failed"
    assert failed.artifact_id == ""
    assert writer.abort_count == 1
    assert session.observer_count == 0
    assert writer.appended == []


def test_close_failure_occurs_after_detach_and_never_claims_committed(
    tmp_path: Path,
) -> None:
    run_dir = tmp_path / "run-playable-001"
    session = _OrderedSession(_snapshot(10, 1_000_000_000))
    writer = _MemoryWriter(
        run_dir / RECORDING_FILENAME,
        close_error=OSError("close exploded"),
    )
    controller = InteractiveRecordingController(
        run_dir=run_dir,
        run_id="run-playable-001",
        session_id="session-a",
        allocation_provider=lambda: _allocation(run_dir),
        writer_factory=_MemoryWriterFactory(writer),
    )
    controller.bind_session(session)
    controller.start(model_generation=3, reset_generation=2)

    with pytest.raises(PlayableRecordingError, match="close exploded"):
        controller.commit(model_generation=3, reset_generation=2)

    failed = controller.status_snapshot()
    assert failed.lifecycle_state is RecordingLifecycleState.FAILED
    assert failed.state == "failed"
    assert failed.artifact_id == ""
    assert "close exploded" in failed.blocker
    assert session.observer_count == 0
    assert writer.abort_count == 1
    session.emit(_snapshot(11, 1_020_000_000))
    assert len(writer.appended) == 1


def test_duplicate_and_out_of_order_lifecycle_requests_are_rejected_without_lies(
    tmp_path: Path,
) -> None:
    run_dir = tmp_path / "run-playable-001"
    session = _OrderedSession(_snapshot(10, 1_000_000_000))
    writer = _MemoryWriter(run_dir / RECORDING_FILENAME)
    controller = InteractiveRecordingController(
        run_dir=run_dir,
        run_id="run-playable-001",
        session_id="session-a",
        allocation_provider=lambda: _allocation(run_dir),
        writer_factory=_MemoryWriterFactory(writer),
    )
    controller.bind_session(session)

    with pytest.raises(PlayableRecordingError) as before_start:
        controller.commit(model_generation=3, reset_generation=2)
    assert before_start.value.code == "recording_not_capturing"
    assert controller.status_snapshot().state == "idle"

    controller.start(model_generation=3, reset_generation=2)
    with pytest.raises(PlayableRecordingError) as duplicate_start:
        controller.start(model_generation=3, reset_generation=2)
    assert duplicate_start.value.code == "recording_already_capturing"
    assert controller.status_snapshot().state == "recording"

    controller.commit(model_generation=3, reset_generation=2)
    with pytest.raises(PlayableRecordingError) as duplicate_commit:
        controller.commit(model_generation=3, reset_generation=2)
    assert duplicate_commit.value.code == "recording_already_committed"
    assert controller.status_snapshot().state == "committed"


def test_status_document_is_exact_copy_safe_and_generation_bound(tmp_path: Path) -> None:
    run_dir = tmp_path / "run-playable-001"
    session = _OrderedSession(_snapshot(10, 1_000_000_000))
    controller = InteractiveRecordingController(
        run_dir=run_dir,
        run_id="run-playable-001",
        session_id="session-a",
        allocation_provider=lambda: _allocation(run_dir),
        writer_factory=_MemoryWriterFactory(
            _MemoryWriter(run_dir / RECORDING_FILENAME)
        ),
    )

    assert controller.status_snapshot().state == "unavailable"
    with pytest.raises(PlayableRecordingError) as unavailable_document:
        controller.status_document()
    assert unavailable_document.value.code == "recording_status_generation_unavailable"
    controller.bind_session(session)
    idle = controller.status_document()
    assert idle == {
        "schema": "lingtu.sim.recording-status.v1",
        "run_id": "run-playable-001",
        "session_id": "session-a",
        "model_generation": 3,
        "reset_generation": 2,
        "state": "idle",
        "elapsed_sim_time_ns": 0,
        "artifact_id": "",
        "blocker": "",
    }
    idle["state"] = "forged"
    assert controller.status_document()["state"] == "idle"

    controller.start(model_generation=3, reset_generation=2)
    capturing = controller.status_document()
    assert capturing["state"] == "recording"
    assert capturing["artifact_id"] == RECORDING_FILENAME
    assert not (run_dir / RECORDING_FILENAME).exists()

    failed = controller.abort_failed("test cleanup before commit")
    assert failed.state == "failed"
    assert failed.artifact_id == ""


def test_pump_routes_record_actions_and_blocks_exit_until_actual_commit(
    tmp_path: Path,
) -> None:
    run_dir = tmp_path / "run-playable-001"
    session = _OrderedSession(_snapshot(10, 1_000_000_000))
    writer = _MemoryWriter(run_dir / RECORDING_FILENAME)
    controller = InteractiveRecordingController(
        run_dir=run_dir,
        run_id="run-playable-001",
        session_id="session-a",
        allocation_provider=lambda: _allocation(run_dir),
        writer_factory=_MemoryWriterFactory(writer),
    )
    controller.bind_session(session)
    requests = BoundedRuntimeRequestInbox()
    ack = _AckPublisher()
    traces: list[dict[str, Any]] = []
    pump = PlayableControlPump(
        session_host=_PumpHost(),
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=LatestOperatorIntentInbox(),
        request_inbox=requests,
        ack_publisher=ack,
        recording_controller=controller,
        trace_sink=lambda document: traces.append(dict(document)),
    )

    requests.put(_runtime_request("record_start", 1))
    assert pump.process_runtime_requests() is None
    assert controller.status_snapshot().state == "recording"
    assert ack.documents[-1]["status"] == "accepted"
    assert traces[-1]["event"] == "runtime_request_accepted"
    assert len(writer.appended) == 1

    requests.put(_runtime_request("record_start", 2))
    assert pump.process_runtime_requests() is None
    assert ack.documents[-1]["status"] == "rejected"
    assert ack.documents[-1]["reason"] == "recording_already_capturing"
    assert controller.status_snapshot().state == "recording"

    requests.put(_runtime_request("exit", 3))
    assert pump.process_runtime_requests() is None
    assert ack.documents[-1]["status"] == "rejected"
    assert ack.documents[-1]["reason"] == "recording_commit_required_before_exit"

    requests.put(_runtime_request("record_stop_commit", 4))
    assert pump.process_runtime_requests() is None
    assert controller.status_snapshot().state == "committed"
    assert ack.documents[-1]["status"] == "accepted"

    requests.put(_runtime_request("exit", 5))
    assert pump.process_runtime_requests() is InteractiveControlRequest.EXIT
    assert ack.documents[-1]["status"] == "accepted"


def test_exceptional_abort_after_owner_quiesces_detaches_and_never_commits(
    tmp_path: Path,
) -> None:
    run_dir = tmp_path / "run-playable-001"
    session = _OrderedSession(_snapshot(10, 1_000_000_000))
    writer = _MemoryWriter(run_dir / RECORDING_FILENAME)
    controller = InteractiveRecordingController(
        run_dir=run_dir,
        run_id="run-playable-001",
        session_id="session-a",
        allocation_provider=lambda: _allocation(run_dir),
        writer_factory=_MemoryWriterFactory(writer),
    )
    controller.bind_session(session)
    controller.start(model_generation=3, reset_generation=2)

    failed = controller.abort_failed("runtime failed before explicit commit")

    assert failed.lifecycle_state is RecordingLifecycleState.FAILED
    assert failed.state == "failed"
    assert failed.artifact_id == ""
    assert "runtime failed before explicit commit" in failed.blocker
    assert writer.abort_count == 1
    assert session.observer_count == 0


def test_owner_thread_cannot_be_stolen_while_capture_is_live(tmp_path: Path) -> None:
    run_dir = tmp_path / "run-playable-001"
    session = _OrderedSession(_snapshot(10, 1_000_000_000))
    session.background_thread_alive = True
    controller = InteractiveRecordingController(
        run_dir=run_dir,
        run_id="run-playable-001",
        session_id="session-a",
        allocation_provider=lambda: _allocation(run_dir),
        writer_factory=_MemoryWriterFactory(
            _MemoryWriter(run_dir / RECORDING_FILENAME)
        ),
    )
    controller.bind_session(session)
    started = threading.Event()
    release = threading.Event()
    errors: list[BaseException] = []

    def owner() -> None:
        try:
            controller.start(model_generation=3, reset_generation=2)
            started.set()
            release.wait(timeout=1.0)
        except BaseException as exc:  # pragma: no cover - diagnostic capture
            errors.append(exc)

    thread = threading.Thread(target=owner)
    thread.start()
    assert started.wait(timeout=1.0)
    try:
        with pytest.raises(PlayableRecordingError) as wrong_owner:
            controller.commit(model_generation=3, reset_generation=2)
        assert wrong_owner.value.code == "recording_owner_thread_mismatch"
        with pytest.raises(PlayableRecordingError) as live_abort:
            controller.abort_failed("foreign cleanup while owner is live")
        assert live_abort.value.code == "recording_owner_not_quiesced"
        assert controller.status_snapshot().state == "recording"
    finally:
        session.background_thread_alive = False
        controller.abort_failed("test cleanup after owner quiesced")
        release.set()
        thread.join(timeout=1.0)
    assert not errors
