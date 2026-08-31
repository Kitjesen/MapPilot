# ruff: noqa: S101

from __future__ import annotations

import json
import math
import threading
import time
from collections.abc import Callable, Mapping
from typing import Any

import pytest

from sim.runtime.control.contracts import CommandSubmitResult, ControllerCommand
from sim.runtime.coordinator.control_intent_udp import (
    CONTROL_ACK_SCHEMA,
    BoundedRuntimeRequestInbox,
    LatestOperatorIntentInbox,
    OperatorIntentIdentity,
    OperatorMotionAxes,
    OperatorMotionIntent,
    OperatorRuntimeRequest,
)
from sim.runtime.coordinator.control_status import CONTROL_STATUS_SCHEMA
from sim.runtime.coordinator.controlled_run import BaseTwistTarget
from sim.runtime.coordinator.coordinator import RuntimeState
from sim.runtime.coordinator.interactive_session import (
    InteractiveControlRequest,
    InteractiveSimulationSession,
)
from sim.runtime.coordinator.playable_control import (
    PlayableControlError,
    PlayableControlPump,
)


def _truthful_status_authority(
    snapshot: Mapping[str, Any],
    *,
    runtime_state: str,
    recording_snapshot: Mapping[str, Any] | None,
) -> Mapping[str, Any]:
    inactive_facet = {
        "state": "UNAVAILABLE",
        "required": True,
        "source_id": "unavailable",
        "blocker": "test fixture does not own this runtime facet",
    }
    return {
        "run_id": "run-playable-001",
        "session_id": "a" * 64,
        "boot_id": "boot-playable-001",
        "model_generation": 3,
        "reset_generation": 2,
        "runtime_state": runtime_state,
        "sim_time_ns": snapshot["sim_time_ns"],
        "truth_sequence": snapshot["sequence"],
        "observed_base_velocity_mps_radps": {
            "available": False,
            "linear_x": 0.0,
            "linear_y": 0.0,
            "angular_z": 0.0,
        },
        "readiness": {
            "physics": dict(inactive_facet),
            "control": dict(inactive_facet),
            "visual": dict(inactive_facet),
            "sensors": dict(inactive_facet),
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


class _RecordingSessionHost:
    def __init__(
        self,
        result: CommandSubmitResult = CommandSubmitResult.ACCEPTED,
    ) -> None:
        self.submissions: list[tuple[str, ControllerCommand, int]] = []
        self.result = result

    def submit_controller_command(
        self,
        controller_id: str,
        command: ControllerCommand,
    ) -> CommandSubmitResult:
        self.submissions.append((controller_id, command, threading.get_ident()))
        return self.result

    def control_status_authority_snapshot(
        self,
        snapshot: Mapping[str, Any],
        *,
        recording_snapshot: Mapping[str, Any] | None = None,
    ) -> Mapping[str, Any]:
        return _truthful_status_authority(
            snapshot,
            runtime_state="RUNNING",
            recording_snapshot=recording_snapshot,
        )


class _RecordingAckPublisher:
    def __init__(
        self,
        *,
        on_publish: Callable[[Mapping[str, Any]], None] | None = None,
        fail_terminal_status: bool = False,
    ) -> None:
        self.documents: list[dict[str, Any]] = []
        self.successful_documents: list[dict[str, Any]] = []
        self._on_publish = on_publish
        self._fail_terminal_status = fail_terminal_status

    def publish(self, document: Mapping[str, Any]) -> int:
        copied = dict(document)
        self.documents.append(copied)
        if (
            self._fail_terminal_status
            and copied.get("schema") == CONTROL_STATUS_SCHEMA
            and copied.get("status") == "confirmed"
        ):
            return 0
        self.successful_documents.append(copied)
        if self._on_publish is not None:
            self._on_publish(copied)
        return len(
            json.dumps(
                copied,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            ).encode("utf-8")
        )


class _InteractiveCoordinator:
    def __init__(self) -> None:
        self._state = RuntimeState.NEW
        self._sequence = 0
        self._start_count = 0
        self.first_advance = threading.Event()
        self.resumed_advance = threading.Event()
        self.command_submitted = threading.Event()
        self.submissions: list[ControllerCommand] = []
        self.calls: list[str] = []

    @property
    def state(self) -> RuntimeState:
        return self._state

    def prepare(self) -> Mapping[str, Any]:
        self.calls.append("prepare")
        self._state = RuntimeState.READY
        return self._event("ready")

    def start(self) -> Mapping[str, Any]:
        self.calls.append("start")
        self._start_count += 1
        self._state = RuntimeState.RUNNING
        return self._event("running")

    def advance(self, steps: int = 1) -> Mapping[str, Any]:
        assert steps == 1
        self.calls.append("advance")
        self._sequence += 1
        self.first_advance.set()
        if self._start_count >= 2:
            self.resumed_advance.set()
        return self._event("snapshot")

    def pause(self) -> Mapping[str, Any]:
        self.calls.append("pause")
        self._state = RuntimeState.PAUSED
        return self._event("paused")

    def reset(self) -> Mapping[str, Any]:
        self.calls.append("reset")
        return self._event("snapshot")

    def hold_controller_commands(self) -> None:
        self.calls.append("hold")
        return None

    def submit_controller_command(
        self,
        controller_id: str,
        command: ControllerCommand,
    ) -> CommandSubmitResult:
        del controller_id
        self.calls.append("submit")
        self.submissions.append(command)
        self.command_submitted.set()
        return CommandSubmitResult.ACCEPTED

    def control_status_authority_snapshot(
        self,
        snapshot: Mapping[str, Any],
        *,
        recording_snapshot: Mapping[str, Any] | None = None,
    ) -> Mapping[str, Any]:
        return _truthful_status_authority(
            snapshot,
            runtime_state=self._state.value,
            recording_snapshot=recording_snapshot,
        )

    def stop(self, *, failure_reason: str | None = None) -> Mapping[str, Any]:
        del failure_reason
        self.calls.append("stop")
        self._state = RuntimeState.STOPPED
        return self._event("stopped")

    def stop_runtime_before_visual(
        self,
        *,
        failure_reason: str | None = None,
    ) -> Mapping[str, Any]:
        del failure_reason
        self.calls.append("stop_runtime_before_visual")
        self._state = RuntimeState.STOPPED
        return self._event("stopped")

    def finalize_visual_after_terminal(self) -> int:
        self.calls.append("finalize_visual_after_terminal")
        return 0

    def _event(self, event: str) -> dict[str, Any]:
        return {
            "event": event,
            "model_generation": 3,
            "reset_generation": 2,
            "sequence": self._sequence,
            "sim_time_ns": self._sequence * 2_000_000,
        }


def _identity() -> OperatorIntentIdentity:
    return OperatorIntentIdentity(
        run_id="run-playable-001",
        session_id="a" * 64,
        boot_id="boot-playable-001",
        model_generation=3,
        reset_generation=2,
        source_id="robotsimue.local_player.0",
    )


def _motion_intent(**overrides: Any) -> OperatorMotionIntent:
    values: dict[str, Any] = {
        "identity": _identity(),
        "source_epoch": 1,
        "source_sequence": 42,
        "event_id": "boot-playable-001:1:42",
        "input_mode": "drive",
        "input_device": "keyboard",
        "viewport_focused": True,
        "deadman": True,
        "axes": OperatorMotionAxes(forward=1.0, left=1.0, yaw_left=-0.5),
        "active_controls": ("keyboard.left_shift", "keyboard.w", "keyboard.a"),
        "source_monotonic_ns": 950_000_000,
        "arrival_monotonic_ns": 1_000_000_000,
        "datagram_sha256": "b" * 64,
    }
    values.update(overrides)
    return OperatorMotionIntent(**values)


def _current_event(**overrides: Any) -> dict[str, Any]:
    event: dict[str, Any] = {
        "event": "snapshot",
        "model_generation": 3,
        "reset_generation": 2,
        "sequence": 17,
        "sim_time_ns": 900_000_000,
    }
    event.update(overrides)
    return event


def _runtime_request(
    request: str,
    *,
    source_sequence: int,
) -> OperatorRuntimeRequest:
    return OperatorRuntimeRequest(
        identity=_identity(),
        source_epoch=1,
        source_sequence=source_sequence,
        event_id=f"boot-playable-001:1:{source_sequence}",
        request=request,
        source_monotonic_ns=950_000_000 + source_sequence,
        arrival_monotonic_ns=1_000_000_000 + source_sequence,
        datagram_sha256=f"{source_sequence % 16:x}" * 64,
    )


def test_fresh_drive_intent_is_scaled_and_submitted_with_hash_join() -> None:
    host = _RecordingSessionHost()
    motion_inbox = LatestOperatorIntentInbox()
    motion_inbox.put(_motion_intent())
    ack = _RecordingAckPublisher()
    traces: list[dict[str, Any]] = []
    pump = PlayableControlPump(
        session_host=host,
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=motion_inbox,
        request_inbox=BoundedRuntimeRequestInbox(),
        ack_publisher=ack,
        trace_sink=lambda record: traces.append(dict(record)),
        monotonic_ns=lambda: 1_050_000_000,
    )

    owner_thread_id = threading.get_ident()
    pump.process_before_advance(_current_event())

    assert len(host.submissions) == 1
    controller_id, command, submit_thread_id = host.submissions[0]
    assert controller_id == "thunder_01.thunderv4_locomotion"
    assert submit_thread_id == owner_thread_id
    assert command.channel_id == "thunder_01.control.base_twist"
    assert command.instance_id == "thunder_01"
    assert command.generation.model_generation == 3
    assert command.generation.reset_generation == 2
    assert command.sequence == 18
    assert command.apply_time_ns == 900_000_000
    assert command.payload == {
        "linear_x": 0.1 / math.sqrt(2.0),
        "linear_y": 0.1 / math.sqrt(2.0),
        "angular_z": -0.175,
    }

    assert [record["event"] for record in traces] == [
        "control_intent_received",
        "control_command_accepted",
    ]
    accepted = traces[-1]
    assert set(accepted) == {
        "schema",
        "event",
        "run_id",
        "session_id",
        "boot_id",
        "model_generation",
        "reset_generation",
        "source_id",
        "source_epoch",
        "source_sequence",
        "event_id",
        "source_monotonic_ns",
        "arrival_monotonic_ns",
        "datagram_sha256",
        "controller_id",
        "channel_id",
        "controller_sequence",
        "apply_time_ns",
        "submit_result",
        "admitted_twist",
    }
    assert accepted["datagram_sha256"] == "b" * 64
    assert accepted["controller_id"] == "thunder_01.thunderv4_locomotion"
    assert accepted["channel_id"] == "thunder_01.control.base_twist"
    assert accepted["controller_sequence"] == 18
    assert accepted["apply_time_ns"] == 900_000_000
    assert accepted["submit_result"] == "accepted"
    assert accepted["admitted_twist"] == command.payload

    assert len(ack.documents) == 1
    assert ack.documents[0]["status"] == "accepted"
    assert ack.documents[0]["reason"] == ""
    assert ack.documents[0]["intent_datagram_sha256"] == "b" * 64


def test_active_intent_times_out_to_one_zero_without_old_input_revival() -> None:
    host = _RecordingSessionHost()
    motion_inbox = LatestOperatorIntentInbox()
    motion_inbox.put(
        _motion_intent(
            axes=OperatorMotionAxes(forward=1.0, left=0.0, yaw_left=0.0),
        )
    )
    ack = _RecordingAckPublisher()
    traces: list[dict[str, Any]] = []
    now_ns = [1_050_000_000]
    pump = PlayableControlPump(
        session_host=host,
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=motion_inbox,
        request_inbox=BoundedRuntimeRequestInbox(),
        ack_publisher=ack,
        trace_sink=lambda record: traces.append(dict(record)),
        monotonic_ns=lambda: now_ns[0],
    )

    pump.process_before_advance(_current_event())
    now_ns[0] = 1_100_000_001
    pump.process_before_advance(_current_event(sequence=18, sim_time_ns=902_000_000))
    pump.process_before_advance(_current_event(sequence=19, sim_time_ns=904_000_000))

    assert [command.payload for _, command, _ in host.submissions] == [
        {"linear_x": 0.1, "linear_y": 0.0, "angular_z": 0.0},
        {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0},
    ]
    assert [record["event"] for record in traces] == [
        "control_intent_received",
        "control_command_accepted",
        "control_command_zero",
    ]
    zero = traces[-1]
    assert zero["reason"] == "intent_stale"
    assert zero["datagram_sha256"] == "b" * 64
    assert zero["submit_result"] == "accepted"
    assert [document["status"] for document in ack.documents] == [
        "accepted",
        "timeout_zero",
    ]
    assert ack.documents[-1]["intent_datagram_sha256"] == "b" * 64


@pytest.mark.parametrize(
    ("overrides", "now_ns", "status", "reason"),
    [
        ({"deadman": False}, 1_050_000_000, "released", "deadman_released"),
        (
            {"viewport_focused": False},
            1_050_000_000,
            "released",
            "viewport_unfocused",
        ),
        ({"input_mode": "tactical"}, 1_050_000_000, "released", "input_mode_not_drive"),
        ({}, 1_100_000_001, "timeout_zero", "intent_stale"),
    ],
)
def test_unsafe_or_stale_sample_is_admitted_only_as_zero(
    overrides: dict[str, Any],
    now_ns: int,
    status: str,
    reason: str,
) -> None:
    host = _RecordingSessionHost()
    motion_inbox = LatestOperatorIntentInbox()
    motion_inbox.put(_motion_intent(**overrides))
    ack = _RecordingAckPublisher()
    traces: list[dict[str, Any]] = []
    pump = PlayableControlPump(
        session_host=host,
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=motion_inbox,
        request_inbox=BoundedRuntimeRequestInbox(),
        ack_publisher=ack,
        trace_sink=lambda record: traces.append(dict(record)),
        monotonic_ns=lambda: now_ns,
    )

    pump.process_before_advance(_current_event())
    pump.process_before_advance(_current_event(sequence=18, sim_time_ns=902_000_000))

    assert len(host.submissions) == 1
    assert host.submissions[0][1].payload == {
        "linear_x": 0.0,
        "linear_y": 0.0,
        "angular_z": 0.0,
    }
    assert [record["event"] for record in traces] == [
        "control_intent_received",
        "control_command_zero",
    ]
    assert traces[-1]["reason"] == reason
    assert traces[-1]["datagram_sha256"] == "b" * 64
    assert ack.documents[-1]["status"] == status
    assert ack.documents[-1]["reason"] == reason


@pytest.mark.parametrize(
    "current_event",
    [
        _current_event(model_generation=4),
        _current_event(reset_generation=3),
    ],
)
def test_generation_race_rejects_motion_and_uses_current_event_for_safe_zero(
    current_event: dict[str, Any],
) -> None:
    host = _RecordingSessionHost()
    motion_inbox = LatestOperatorIntentInbox()
    motion_inbox.put(_motion_intent())
    ack = _RecordingAckPublisher()
    traces: list[dict[str, Any]] = []
    pump = PlayableControlPump(
        session_host=host,
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=motion_inbox,
        request_inbox=BoundedRuntimeRequestInbox(),
        ack_publisher=ack,
        trace_sink=lambda record: traces.append(dict(record)),
        monotonic_ns=lambda: 1_050_000_000,
    )

    pump.process_before_advance(current_event)

    assert len(host.submissions) == 1
    command = host.submissions[0][1]
    assert command.payload == {
        "linear_x": 0.0,
        "linear_y": 0.0,
        "angular_z": 0.0,
    }
    assert command.generation.model_generation == current_event["model_generation"]
    assert command.generation.reset_generation == current_event["reset_generation"]
    assert traces[-1]["event"] == "control_command_zero"
    assert traces[-1]["reason"] == "intent_generation_mismatch"
    assert ack.documents[-1]["status"] == "rejected"
    assert ack.documents[-1]["reason"] == "intent_generation_mismatch"


def test_clear_discards_unread_motion_and_emits_one_correlated_zero() -> None:
    host = _RecordingSessionHost()
    motion_inbox = LatestOperatorIntentInbox()
    motion_inbox.put(
        _motion_intent(
            axes=OperatorMotionAxes(forward=1.0, left=0.0, yaw_left=0.0),
        )
    )
    ack = _RecordingAckPublisher()
    traces: list[dict[str, Any]] = []
    pump = PlayableControlPump(
        session_host=host,
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=motion_inbox,
        request_inbox=BoundedRuntimeRequestInbox(),
        ack_publisher=ack,
        trace_sink=lambda record: traces.append(dict(record)),
        monotonic_ns=lambda: 1_050_000_000,
    )
    pump.process_before_advance(_current_event())
    motion_inbox.put(
        _motion_intent(
            source_sequence=43,
            event_id="boot-playable-001:1:43",
            datagram_sha256="c" * 64,
        )
    )

    pump.clear(reason="pause")
    pump.process_before_advance(_current_event(sequence=18, sim_time_ns=902_000_000))
    pump.process_before_advance(_current_event(sequence=19, sim_time_ns=904_000_000))

    assert len(motion_inbox) == 0
    assert [command.payload for _, command, _ in host.submissions] == [
        {"linear_x": 0.1, "linear_y": 0.0, "angular_z": 0.0},
        {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0},
    ]
    assert traces[-1]["event"] == "control_command_zero"
    assert traces[-1]["reason"] == "cleared:pause"
    assert traces[-1]["datagram_sha256"] == "b" * 64
    assert ack.documents[-1]["status"] == "released"
    assert ack.documents[-1]["intent_datagram_sha256"] == "b" * 64


def test_lifecycle_runtime_requests_are_consumed_in_fifo_order() -> None:
    request_inbox = BoundedRuntimeRequestInbox()
    for sequence, request in enumerate(("pause", "resume", "exit"), start=1):
        request_inbox.put(_runtime_request(request, source_sequence=sequence))
    ack = _RecordingAckPublisher()
    traces: list[dict[str, Any]] = []
    pump = PlayableControlPump(
        session_host=_RecordingSessionHost(),
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=LatestOperatorIntentInbox(),
        request_inbox=request_inbox,
        ack_publisher=ack,
        trace_sink=lambda record: traces.append(dict(record)),
    )

    assert pump.process_runtime_requests() is InteractiveControlRequest.PAUSE
    assert pump.process_runtime_requests() is InteractiveControlRequest.RESUME
    assert pump.process_runtime_requests() is InteractiveControlRequest.EXIT
    assert pump.process_runtime_requests() is None

    assert len(request_inbox) == 0
    assert [document["status"] for document in ack.documents] == [
        "accepted",
        "accepted",
        "accepted",
    ]
    assert [record["event"] for record in traces] == [
        "runtime_request_received",
        "runtime_request_accepted",
        "runtime_request_received",
        "runtime_request_accepted",
        "runtime_request_received",
        "runtime_request_accepted",
    ]


@pytest.mark.parametrize(
    ("request_kind", "status", "reason", "trace_event"),
    [
        ("control_claim", "accepted", "", "runtime_request_accepted"),
        (
            "record_start",
            "rejected",
            "recording_controller_unavailable",
            "runtime_request_rejected",
        ),
        (
            "record_stop_commit",
            "rejected",
            "recording_controller_unavailable",
            "runtime_request_rejected",
        ),
    ],
)
def test_non_lifecycle_runtime_requests_are_explicitly_acked_and_traced(
    request_kind: str,
    status: str,
    reason: str,
    trace_event: str,
) -> None:
    request_inbox = BoundedRuntimeRequestInbox()
    request_inbox.put(_runtime_request(request_kind, source_sequence=1))
    ack = _RecordingAckPublisher()
    traces: list[dict[str, Any]] = []
    pump = PlayableControlPump(
        session_host=_RecordingSessionHost(),
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=LatestOperatorIntentInbox(),
        request_inbox=request_inbox,
        ack_publisher=ack,
        trace_sink=lambda record: traces.append(dict(record)),
    )

    assert pump.process_runtime_requests() is None

    assert len(request_inbox) == 0
    assert [record["event"] for record in traces] == [
        "runtime_request_received",
        trace_event,
    ]
    assert traces[-1]["status"] == status
    assert traces[-1]["reason"] == reason
    assert ack.documents[-1]["status"] == status
    assert ack.documents[-1]["reason"] == reason
    assert ack.documents[-1]["intent_datagram_sha256"] == "1" * 64


@pytest.mark.parametrize("request_kind", ["control_release", "safe_stop"])
def test_release_requests_clear_motion_and_submit_a_hash_correlated_zero(
    request_kind: str,
) -> None:
    host = _RecordingSessionHost()
    motion_inbox = LatestOperatorIntentInbox()
    motion_inbox.put(_motion_intent())
    request_inbox = BoundedRuntimeRequestInbox()
    request_inbox.put(_runtime_request(request_kind, source_sequence=1))
    ack = _RecordingAckPublisher()
    traces: list[dict[str, Any]] = []
    pump = PlayableControlPump(
        session_host=host,
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=motion_inbox,
        request_inbox=request_inbox,
        ack_publisher=ack,
        trace_sink=lambda record: traces.append(dict(record)),
    )

    assert pump.process_runtime_requests() is None
    assert len(motion_inbox) == 0
    pump.process_before_advance(_current_event())
    pump.process_before_advance(_current_event(sequence=18, sim_time_ns=902_000_000))

    assert len(host.submissions) == 1
    assert host.submissions[0][1].payload == {
        "linear_x": 0.0,
        "linear_y": 0.0,
        "angular_z": 0.0,
    }
    assert [record["event"] for record in traces] == [
        "runtime_request_received",
        "runtime_request_accepted",
        "control_command_zero",
    ]
    assert traces[-1]["reason"] == f"runtime_request:{request_kind}"
    assert traces[-1]["datagram_sha256"] == "1" * 64
    assert [document["status"] for document in ack.documents] == [
        "pending",
        "released",
    ]
    assert all(document["intent_datagram_sha256"] == "1" * 64 for document in ack.documents)


def test_clear_motion_preserves_queued_runtime_requests() -> None:
    motion_inbox = LatestOperatorIntentInbox()
    motion_inbox.put(_motion_intent())
    request_inbox = BoundedRuntimeRequestInbox()
    request_inbox.put(_runtime_request("resume", source_sequence=1))
    pump = PlayableControlPump(
        session_host=_RecordingSessionHost(),
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=motion_inbox,
        request_inbox=request_inbox,
    )

    pump.clear_motion(reason="resume")

    assert len(motion_inbox) == 0
    assert len(request_inbox) == 1
    assert pump.process_runtime_requests() is InteractiveControlRequest.RESUME


def test_full_clear_explicitly_rejects_every_queued_runtime_request() -> None:
    request_inbox = BoundedRuntimeRequestInbox()
    request_inbox.put(_runtime_request("resume", source_sequence=1))
    request_inbox.put(_runtime_request("record_start", source_sequence=2))
    ack = _RecordingAckPublisher()
    traces: list[dict[str, Any]] = []
    pump = PlayableControlPump(
        session_host=_RecordingSessionHost(),
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=LatestOperatorIntentInbox(),
        request_inbox=request_inbox,
        ack_publisher=ack,
        trace_sink=lambda record: traces.append(dict(record)),
    )

    pump.clear(reason="reset")

    assert len(request_inbox) == 0
    assert [record["event"] for record in traces] == [
        "runtime_request_rejected",
        "runtime_request_rejected",
    ]
    assert all(record["reason"] == "request_cleared:reset" for record in traces)
    assert [document["status"] for document in ack.documents] == [
        "rejected",
        "rejected",
    ]
    assert all(document["reason"] == "request_cleared:reset" for document in ack.documents)


def test_real_interactive_session_resumes_from_runtime_request() -> None:
    coordinator = _InteractiveCoordinator()
    request_inbox = BoundedRuntimeRequestInbox()
    ack = _RecordingAckPublisher()
    traces: list[dict[str, Any]] = []
    pump = PlayableControlPump(
        session_host=coordinator,
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=LatestOperatorIntentInbox(),
        request_inbox=request_inbox,
        ack_publisher=ack,
        trace_sink=lambda record: traces.append(dict(record)),
    )
    session = InteractiveSimulationSession(
        coordinator,
        control_pump=pump,
        advance_wait_s=0.001,
    )
    session.prepare()
    session.start()
    assert coordinator.first_advance.wait(timeout=1.0)
    session.pause()
    request_inbox.put(_runtime_request("resume", source_sequence=1))

    assert coordinator.resumed_advance.wait(timeout=1.0)
    deadline = time.monotonic() + 1.0
    while session.state is not RuntimeState.RUNNING and time.monotonic() < deadline:
        time.sleep(0.001)

    assert session.state is RuntimeState.RUNNING
    assert session.last_error is None
    assert len(request_inbox) == 0
    assert any(record["event"] == "runtime_request_accepted" and record["request"] == "resume" for record in traces)
    ack_only = [
        document
        for document in ack.documents
        if document.get("schema") == CONTROL_ACK_SCHEMA
    ]
    assert [document["status"] for document in ack_only[-2:]] == [
        "accepted",
        "released",
    ]
    session.stop()


def test_runtime_exit_flushes_correlated_zero_before_hold_and_stop() -> None:
    coordinator = _InteractiveCoordinator()
    motion_inbox = LatestOperatorIntentInbox()
    motion_inbox.put(
        _motion_intent(
            axes=OperatorMotionAxes(forward=1.0, left=0.0, yaw_left=0.0),
        )
    )
    request_inbox = BoundedRuntimeRequestInbox()
    ack = _RecordingAckPublisher(
        on_publish=lambda document: (
            coordinator.calls.append("terminal_status_send")
            if document.get("schema") == CONTROL_STATUS_SCHEMA
            and document.get("status") == "confirmed"
            else None
        )
    )
    traces: list[dict[str, Any]] = []
    pump = PlayableControlPump(
        session_host=coordinator,
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=motion_inbox,
        request_inbox=request_inbox,
        ack_publisher=ack,
        trace_sink=lambda record: traces.append(dict(record)),
        monotonic_ns=lambda: 1_050_000_000,
    )
    session = InteractiveSimulationSession(
        coordinator,
        control_pump=pump,
        advance_wait_s=0.001,
    )
    session.prepare()
    session.start()
    assert coordinator.command_submitted.wait(timeout=1.0)
    request_inbox.put(_runtime_request("exit", source_sequence=43))

    deadline = time.monotonic() + 1.0
    while session.state is not RuntimeState.STOPPED and time.monotonic() < deadline:
        time.sleep(0.001)

    assert session.state is RuntimeState.STOPPED
    assert session.last_error is None
    assert len(coordinator.submissions) == 2
    final_zero = coordinator.submissions[-1]
    assert final_zero.payload == {
        "linear_x": 0.0,
        "linear_y": 0.0,
        "angular_z": 0.0,
    }
    event_traces = [record for record in traces if "event" in record]
    assert [record["event"] for record in event_traces[-3:]] == [
        "runtime_request_received",
        "runtime_request_accepted",
        "control_command_zero",
    ]
    assert event_traces[-1]["reason"] == "cleared:exit"
    assert set(event_traces[-1]) == {
        "schema",
        "event",
        "run_id",
        "session_id",
        "boot_id",
        "model_generation",
        "reset_generation",
        "source_id",
        "source_epoch",
        "source_sequence",
        "event_id",
        "source_monotonic_ns",
        "arrival_monotonic_ns",
        "datagram_sha256",
        "controller_sequence",
        "apply_time_ns",
        "submit_result",
        "admitted_twist",
        "reason",
    }
    assert event_traces[-1]["event_id"] == "boot-playable-001:1:43"
    assert event_traces[-1]["datagram_sha256"] == "b" * 64
    ack_only = [
        document
        for document in ack.documents
        if document.get("schema") == CONTROL_ACK_SCHEMA
    ]
    assert ack_only[-2]["status"] == "accepted"
    assert ack_only[-1]["status"] == "released"
    assert ack_only[-1]["event_id"] == "boot-playable-001:1:43"
    assert ack_only[-1]["intent_datagram_sha256"] == "b" * 64

    terminal = [
        document
        for document in ack.successful_documents
        if document.get("schema") == CONTROL_STATUS_SCHEMA
        and document.get("status") == "confirmed"
    ]
    assert len(terminal) == 1
    assert terminal[0]["event_id"] == "boot-playable-001:1:43"
    assert terminal[0]["intent_datagram_sha256"] == "b" * 64
    assert terminal[0]["runtime"]["runtime_state"] == "STOPPED"
    assert terminal[0]["runtime"]["safe_stop_state"] == "zeroed"
    assert terminal[0]["motion"]["admitted_twist_mps_radps"] == {
        "available": True,
        "linear_x": 0.0,
        "linear_y": 0.0,
        "angular_z": 0.0,
    }

    final_submit_index = max(index for index, name in enumerate(coordinator.calls) if name == "submit")
    hold_index = coordinator.calls.index("hold", final_submit_index)
    core_stop_index = coordinator.calls.index("stop_runtime_before_visual", hold_index)
    terminal_index = coordinator.calls.index("terminal_status_send", core_stop_index)
    visual_finalize_index = coordinator.calls.index(
        "finalize_visual_after_terminal",
        terminal_index,
    )
    assert (
        final_submit_index
        < hold_index
        < core_stop_index
        < terminal_index
        < visual_finalize_index
    )
    assert "advance" not in coordinator.calls[core_stop_index + 1 :]
    session.stop()


def test_runtime_exit_status_send_failure_never_confirms_normal_stop() -> None:
    coordinator = _InteractiveCoordinator()
    motion_inbox = LatestOperatorIntentInbox()
    motion_inbox.put(
        _motion_intent(
            axes=OperatorMotionAxes(forward=1.0, left=0.0, yaw_left=0.0),
        )
    )
    request_inbox = BoundedRuntimeRequestInbox()
    publisher = _RecordingAckPublisher(fail_terminal_status=True)
    pump = PlayableControlPump(
        session_host=coordinator,
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=motion_inbox,
        request_inbox=request_inbox,
        ack_publisher=publisher,
        monotonic_ns=lambda: 1_050_000_000,
    )
    session = InteractiveSimulationSession(
        coordinator,
        control_pump=pump,
        advance_wait_s=0.001,
    )
    session.prepare()
    session.start()
    assert coordinator.command_submitted.wait(timeout=1.0)
    request_inbox.put(_runtime_request("exit", source_sequence=43))

    deadline = time.monotonic() + 1.0
    while session.state is not RuntimeState.FAILED and time.monotonic() < deadline:
        time.sleep(0.001)

    assert session.state is RuntimeState.FAILED
    assert session.last_error is not None
    assert "complete encoded datagram" in session.last_error
    assert coordinator.calls.count("stop_runtime_before_visual") == 1
    assert "finalize_visual_after_terminal" not in coordinator.calls
    assert not any(
        document.get("schema") == CONTROL_STATUS_SCHEMA
        and document.get("status") == "confirmed"
        for document in publisher.successful_documents
    )


def test_command_submission_is_bound_to_the_first_owner_thread() -> None:
    host = _RecordingSessionHost()
    motion_inbox = LatestOperatorIntentInbox()
    motion_inbox.put(
        _motion_intent(
            axes=OperatorMotionAxes(forward=1.0, left=0.0, yaw_left=0.0),
        )
    )
    pump = PlayableControlPump(
        session_host=host,
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=motion_inbox,
        request_inbox=BoundedRuntimeRequestInbox(),
        monotonic_ns=lambda: 1_050_000_000,
    )
    owner = threading.Thread(
        target=pump.process_before_advance,
        args=(_current_event(),),
    )
    owner.start()
    owner.join(timeout=1.0)
    assert not owner.is_alive()
    motion_inbox.put(
        _motion_intent(
            source_sequence=43,
            event_id="boot-playable-001:1:43",
            datagram_sha256="c" * 64,
        )
    )

    with pytest.raises(PlayableControlError, match="owner thread"):
        pump.process_before_advance(_current_event(sequence=18, sim_time_ns=902_000_000))

    assert len(host.submissions) == 1
    assert len(motion_inbox) == 1


@pytest.mark.parametrize(
    ("field", "invalid_value"),
    [
        ("model_generation", True),
        ("reset_generation", -1),
        ("sequence", "17"),
        ("sim_time_ns", None),
    ],
)
def test_invalid_current_event_fails_before_consuming_or_submitting(
    field: str,
    invalid_value: object,
) -> None:
    host = _RecordingSessionHost()
    motion_inbox = LatestOperatorIntentInbox()
    motion_inbox.put(_motion_intent())
    pump = PlayableControlPump(
        session_host=host,
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=motion_inbox,
        request_inbox=BoundedRuntimeRequestInbox(),
    )

    with pytest.raises(PlayableControlError, match=field):
        pump.process_before_advance(_current_event(**{field: invalid_value}))

    assert len(host.submissions) == 0
    assert len(motion_inbox) == 1


def test_controller_rejection_is_traced_and_acked_before_failing_closed() -> None:
    host = _RecordingSessionHost(CommandSubmitResult.REJECTED_OUT_OF_ORDER)
    motion_inbox = LatestOperatorIntentInbox()
    motion_inbox.put(_motion_intent())
    ack = _RecordingAckPublisher()
    traces: list[dict[str, Any]] = []
    pump = PlayableControlPump(
        session_host=host,
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=motion_inbox,
        request_inbox=BoundedRuntimeRequestInbox(),
        ack_publisher=ack,
        trace_sink=lambda record: traces.append(dict(record)),
        monotonic_ns=lambda: 1_050_000_000,
    )

    with pytest.raises(PlayableControlError, match="rejected_out_of_order"):
        pump.process_before_advance(_current_event())

    assert [record["event"] for record in traces] == [
        "control_intent_received",
        "control_command_rejected",
    ]
    assert traces[-1]["datagram_sha256"] == "b" * 64
    assert traces[-1]["submit_result"] == "rejected_out_of_order"
    assert ack.documents[-1]["status"] == "rejected"
    assert ack.documents[-1]["reason"] == "rejected_out_of_order"


def test_rejected_safety_zero_fails_the_owner_loop_after_evidence() -> None:
    host = _RecordingSessionHost(CommandSubmitResult.REJECTED_OUT_OF_ORDER)
    motion_inbox = LatestOperatorIntentInbox()
    motion_inbox.put(_motion_intent(deadman=False))
    ack = _RecordingAckPublisher()
    traces: list[dict[str, Any]] = []
    pump = PlayableControlPump(
        session_host=host,
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=motion_inbox,
        request_inbox=BoundedRuntimeRequestInbox(),
        ack_publisher=ack,
        trace_sink=lambda record: traces.append(dict(record)),
        monotonic_ns=lambda: 1_050_000_000,
    )

    with pytest.raises(PlayableControlError, match="rejected_out_of_order"):
        pump.process_before_advance(_current_event())

    assert traces[-1]["event"] == "control_command_zero"
    assert traces[-1]["reason"] == "deadman_released"
    assert traces[-1]["submit_result"] == "rejected_out_of_order"
    assert ack.documents[-1]["status"] == "rejected"
    assert ack.documents[-1]["reason"] == ("deadman_released:rejected_out_of_order")


def test_server_caps_out_of_range_axes_at_its_speed_limits() -> None:
    host = _RecordingSessionHost()
    motion_inbox = LatestOperatorIntentInbox()
    motion_inbox.put(
        _motion_intent(
            axes=OperatorMotionAxes(forward=2.0, left=0.0, yaw_left=-2.0),
        )
    )
    pump = PlayableControlPump(
        session_host=host,
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=motion_inbox,
        request_inbox=BoundedRuntimeRequestInbox(),
        monotonic_ns=lambda: 1_050_000_000,
    )

    pump.process_before_advance(_current_event())

    assert host.submissions[0][1].payload == {
        "linear_x": 0.1,
        "linear_y": 0.0,
        "angular_z": -0.35,
    }
