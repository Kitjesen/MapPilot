# ruff: noqa: S101

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from sim.runtime.control.contracts import CommandSubmitResult, ControllerCommand
from sim.runtime.coordinator.control_intent_udp import (
    BoundedRuntimeRequestInbox,
    LatestOperatorIntentInbox,
    OperatorIntentIdentity,
    OperatorMotionAxes,
    OperatorMotionIntent,
    OperatorRuntimeRequest,
)
from sim.runtime.coordinator.controlled_run import BaseTwistTarget
from sim.runtime.coordinator.multiplayer_control import (
    RoomMotionAdmission,
    RoomRuntimeRequestAdmission,
)
from sim.runtime.coordinator.multiplayer_room import (
    MultiplayerJoinRequest,
    MultiplayerRole,
    MultiplayerRoomAuthority,
    MultiplayerRoomIdentity,
)
from sim.runtime.coordinator.playable_control import PlayableControlPump


class _Host:
    def __init__(self) -> None:
        self.submissions: list[ControllerCommand] = []

    def submit_controller_command(
        self,
        controller_id: str,
        command: ControllerCommand,
    ) -> CommandSubmitResult:
        assert controller_id == "thunder_01.thunderv4_locomotion"
        self.submissions.append(command)
        return CommandSubmitResult.ACCEPTED

    def control_status_authority_snapshot(
        self,
        snapshot: Mapping[str, Any],
        *,
        recording_snapshot: Mapping[str, Any] | None = None,
    ) -> Mapping[str, Any]:
        del snapshot, recording_snapshot
        raise AssertionError("status publication is outside this control-admission test")


class _AckPublisher:
    def __init__(self) -> None:
        self.documents: list[dict[str, Any]] = []

    def publish(self, document: Mapping[str, Any]) -> int:
        self.documents.append(dict(document))
        return 1


class _Clock:
    def __init__(self, now_ns: int = 1_050_000_000) -> None:
        self.now_ns = now_ns

    def __call__(self) -> int:
        return self.now_ns


def _room(*, monotonic_ns: _Clock | None = None) -> MultiplayerRoomAuthority:
    return MultiplayerRoomAuthority(
        MultiplayerRoomIdentity(
            run_id="run-multiplayer-001",
            session_id="a" * 64,
            boot_id="boot-multiplayer-001",
        ),
        monotonic_ns=monotonic_ns or (lambda: 1_050_000_000),
    )


def _join(
    room: MultiplayerRoomAuthority,
    *,
    member_id: str,
    source_id: str,
    role: MultiplayerRole,
) -> None:
    room.join(
        MultiplayerJoinRequest(
            member_id=member_id,
            role=role,
            identity=room.identity,
            source_id=source_id,
            source_epoch=1,
        )
    )


def _intent(source_id: str, *, sequence: int = 1) -> OperatorMotionIntent:
    return OperatorMotionIntent(
        identity=OperatorIntentIdentity(
            run_id="run-multiplayer-001",
            session_id="a" * 64,
            boot_id="boot-multiplayer-001",
            model_generation=3,
            reset_generation=2,
            source_id=source_id,
        ),
        source_epoch=1,
        source_sequence=sequence,
        event_id=f"boot-multiplayer-001:1:{sequence}",
        input_mode="drive",
        input_device="keyboard",
        viewport_focused=True,
        deadman=True,
        axes=OperatorMotionAxes(forward=1.0, left=0.0, yaw_left=0.0),
        active_controls=("keyboard.left_shift", "keyboard.w"),
        source_monotonic_ns=990_000_000,
        arrival_monotonic_ns=1_000_000_000,
        datagram_sha256=f"{(sequence + 10) % 16:x}" * 64,
        ui_mode="drive",
        camera_mode="follow",
    )


def _request(
    source_id: str,
    request: str,
    *,
    sequence: int,
) -> OperatorRuntimeRequest:
    return OperatorRuntimeRequest(
        identity=OperatorIntentIdentity(
            run_id="run-multiplayer-001",
            session_id="a" * 64,
            boot_id="boot-multiplayer-001",
            model_generation=3,
            reset_generation=2,
            source_id=source_id,
        ),
        source_epoch=1,
        source_sequence=sequence,
        event_id=f"boot-multiplayer-001:1:{sequence}",
        request=request,
        source_monotonic_ns=990_000_000 + sequence,
        arrival_monotonic_ns=1_000_000_000 + sequence,
        datagram_sha256=f"{sequence % 16:x}" * 64,
        ui_mode="drive",
        camera_mode="follow",
    )


def _pump(
    *,
    host: _Host,
    inbox: LatestOperatorIntentInbox,
    ack: _AckPublisher,
    room: MultiplayerRoomAuthority,
    traces: list[dict[str, Any]],
    request_inbox: BoundedRuntimeRequestInbox | None = None,
    monotonic_ns: _Clock | None = None,
) -> PlayableControlPump:
    return PlayableControlPump(
        session_host=host,
        target=BaseTwistTarget(
            controller_id="thunder_01.thunderv4_locomotion",
            instance_id="thunder_01",
            channel_id="thunder_01.control.base_twist",
        ),
        motion_inbox=inbox,
        request_inbox=request_inbox or BoundedRuntimeRequestInbox(),
        ack_publisher=ack,
        trace_sink=lambda record: traces.append(dict(record)),
        motion_admission=RoomMotionAdmission(room),
        runtime_request_admission=RoomRuntimeRequestAdmission(
            room,
            lease_ttl_ms=500,
        ),
        monotonic_ns=monotonic_ns or (lambda: 1_050_000_000),
    )


def _current_event() -> dict[str, Any]:
    return {
        "event": "snapshot",
        "model_generation": 3,
        "reset_generation": 2,
        "sequence": 17,
        "sim_time_ns": 900_000_000,
    }


def test_observer_motion_is_rejected_without_submitting_zero_or_nonzero() -> None:
    room = _room()
    _join(
        room,
        member_id="observer.alpha",
        source_id="robotsimue.observer.alpha",
        role=MultiplayerRole.OBSERVER,
    )
    host = _Host()
    inbox = LatestOperatorIntentInbox()
    inbox.put(_intent("robotsimue.observer.alpha"))
    ack = _AckPublisher()
    traces: list[dict[str, Any]] = []

    _pump(host=host, inbox=inbox, ack=ack, room=room, traces=traces).process_before_advance(
        _current_event()
    )

    assert host.submissions == []
    assert ack.documents[-1]["status"] == "rejected"
    assert ack.documents[-1]["reason"] == "multiplayer:role_denied"
    assert [record["event"] for record in traces] == [
        "control_intent_received",
        "control_intent_rejected",
    ]


def test_only_current_driver_lease_enters_existing_playable_command_path() -> None:
    room = _room()
    _join(
        room,
        member_id="operator.alpha",
        source_id="robotsimue.operator.alpha",
        role=MultiplayerRole.DRIVER,
    )
    room.claim_control(
        "operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=1,
        ttl_ms=500,
    )
    host = _Host()
    inbox = LatestOperatorIntentInbox()
    inbox.put(_intent("robotsimue.operator.alpha"))
    ack = _AckPublisher()
    traces: list[dict[str, Any]] = []

    _pump(host=host, inbox=inbox, ack=ack, room=room, traces=traces).process_before_advance(
        _current_event()
    )

    assert len(host.submissions) == 1
    assert host.submissions[0].payload == {
        "linear_x": 0.1,
        "linear_y": 0.0,
        "angular_z": 0.0,
    }
    assert ack.documents[-1]["status"] == "accepted"
    assert ack.documents[-1]["reason"] == ""


def test_observer_control_claim_is_rejected_without_creating_lease() -> None:
    room = _room()
    _join(
        room,
        member_id="observer.alpha",
        source_id="robotsimue.observer.alpha",
        role=MultiplayerRole.OBSERVER,
    )
    requests = BoundedRuntimeRequestInbox()
    requests.put(_request("robotsimue.observer.alpha", "control_claim", sequence=1))
    ack = _AckPublisher()
    traces: list[dict[str, Any]] = []

    result = _pump(
        host=_Host(),
        inbox=LatestOperatorIntentInbox(),
        ack=ack,
        room=room,
        traces=traces,
        request_inbox=requests,
    ).process_runtime_requests()

    assert result is None
    assert room.control_lease is None
    assert ack.documents[-1]["status"] == "rejected"
    assert ack.documents[-1]["reason"] == "multiplayer:role_denied"
    assert traces[-1]["event"] == "runtime_request_rejected"


def test_driver_claim_motion_and_release_are_one_authoritative_safe_sequence() -> None:
    room = _room()
    _join(
        room,
        member_id="operator.alpha",
        source_id="robotsimue.operator.alpha",
        role=MultiplayerRole.DRIVER,
    )
    requests = BoundedRuntimeRequestInbox()
    requests.put(_request("robotsimue.operator.alpha", "control_claim", sequence=1))
    motion = LatestOperatorIntentInbox()
    host = _Host()
    ack = _AckPublisher()
    traces: list[dict[str, Any]] = []
    pump = _pump(
        host=host,
        inbox=motion,
        ack=ack,
        room=room,
        traces=traces,
        request_inbox=requests,
    )

    assert pump.process_runtime_requests() is None
    assert room.control_lease is not None
    motion.put(_intent("robotsimue.operator.alpha"))
    pump.process_before_advance(_current_event())
    requests.put(_request("robotsimue.operator.alpha", "control_release", sequence=2))
    assert pump.process_runtime_requests() is None
    assert room.control_lease is None
    pump.process_before_advance(
        {**_current_event(), "sequence": 18, "sim_time_ns": 920_000_000}
    )

    assert [command.payload for command in host.submissions] == [
        {"linear_x": 0.1, "linear_y": 0.0, "angular_z": 0.0},
        {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0},
    ]
    assert [document["status"] for document in ack.documents] == [
        "accepted",
        "accepted",
        "pending",
        "released",
    ]


def test_expired_active_driver_is_zeroed_once_while_later_packets_only_reject() -> None:
    clock = _Clock()
    room = _room(monotonic_ns=clock)
    _join(
        room,
        member_id="operator.alpha",
        source_id="robotsimue.operator.alpha",
        role=MultiplayerRole.DRIVER,
    )
    room.claim_control(
        "operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=1,
        ttl_ms=100,
    )
    motion = LatestOperatorIntentInbox()
    motion.put(_intent("robotsimue.operator.alpha", sequence=1))
    host = _Host()
    ack = _AckPublisher()
    traces: list[dict[str, Any]] = []
    pump = _pump(
        host=host,
        inbox=motion,
        ack=ack,
        room=room,
        traces=traces,
        monotonic_ns=clock,
    )
    pump.process_before_advance(_current_event())
    clock.now_ns += 101_000_000
    motion.put(_intent("robotsimue.operator.alpha", sequence=2))
    pump.process_before_advance(
        {**_current_event(), "sequence": 18, "sim_time_ns": 920_000_000}
    )
    motion.put(_intent("robotsimue.operator.alpha", sequence=3))
    pump.process_before_advance(
        {**_current_event(), "sequence": 19, "sim_time_ns": 940_000_000}
    )

    assert [command.payload for command in host.submissions] == [
        {"linear_x": 0.1, "linear_y": 0.0, "angular_z": 0.0},
        {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0},
    ]
    assert [document["status"] for document in ack.documents] == [
        "accepted",
        "released",
        "rejected",
    ]
    assert ack.documents[1]["reason"] == "multiplayer:lease_required"
    assert ack.documents[2]["reason"] == "multiplayer:lease_required"
