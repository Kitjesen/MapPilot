
from __future__ import annotations

import json
import threading
from collections.abc import Mapping
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from sim.runtime.control.contracts import CommandSubmitResult
from sim.runtime.coordinator.control_intent_udp import (
    BoundedRuntimeRequestInbox,
    ControlIntentValidationError,
    LatestOperatorIntentInbox,
    OperatorIntentIdentity,
    OperatorMotionAxes,
    OperatorMotionIntent,
    UdpLoopbackControlAckPublisher,
)
from sim.runtime.coordinator.control_status import (
    CONTROL_STATUS_SCHEMA,
    ControlStatusControlSnapshot,
    ControlStatusReporter,
    build_control_status_authority,
    encode_control_status,
)
from sim.runtime.coordinator.controlled_run import BaseTwistTarget
from sim.runtime.coordinator.coordinator import CoordinatorError, RuntimeState
from sim.runtime.coordinator.playable_control import PlayableControlPump
from sim.runtime.coordinator.session_host import SessionHost


def _identity() -> OperatorIntentIdentity:
    return OperatorIntentIdentity(
        run_id="run-playable-001",
        session_id="a" * 64,
        boot_id="boot-playable-001",
        model_generation=3,
        reset_generation=2,
        source_id="robotsimue.local_player.0",
    )


def _control(**overrides: Any) -> ControlStatusControlSnapshot:
    values: dict[str, Any] = {
        "identity": _identity(),
        "source_epoch": 1,
        "source_sequence": 42,
        "event_id": "boot-playable-001:1:42",
        "intent_datagram_sha256": "b" * 64,
        "status": "accepted",
        "reason": "",
        "control_owner": "robotsimue.local_player.0",
        "deadman": True,
        "sample_age_ns": 10_000_000,
        "safe_stop_state": "clear",
        "requested_axes": (1.0, 0.0, 0.0),
        "requested_available": True,
        "admitted_twist": (0.10, 0.0, 0.0),
        "admitted_available": True,
        "ui_mode": "drive",
        "camera_mode": "unavailable",
    }
    values.update(overrides)
    return ControlStatusControlSnapshot(**values)


def _authority(**overrides: Any) -> dict[str, Any]:
    facets = {
        name: {
            "state": "ACTIVE",
            "required": True,
            "source_id": f"runtime.{name}",
            "blocker": "",
        }
        for name in ("physics", "control", "visual", "sensors")
    }
    sensors = [
        {
            "stream_id": stream_id,
            "state": "ACTIVE",
            "sample_count": count,
            "blocker": "",
        }
        for count, stream_id in enumerate(
            (
                "thunder_01.front_depth",
                "thunder_01.front_rgb",
                "thunder_01.imu",
                "thunder_01.mid360",
                "thunder_01.truth_odom",
            ),
            start=7,
        )
    ]
    authority: dict[str, Any] = {
        "run_id": "run-playable-001",
        "session_id": "a" * 64,
        "boot_id": "boot-playable-001",
        "model_generation": 3,
        "reset_generation": 2,
        "runtime_state": "RUNNING",
        "sim_time_ns": 2_000_000_000,
        "truth_sequence": 600,
        "observed_base_velocity_mps_radps": {
            "available": True,
            "linear_x": 0.08,
            "linear_y": 0.01,
            "angular_z": 0.02,
        },
        "readiness": facets,
        "sensors": sensors,
        "recording": {
            "state": "unavailable",
            "elapsed_sim_time_ns": 0,
            "artifact_id": "",
            "blocker": "recording status source unavailable",
        },
    }
    authority.update(overrides)
    return authority


def _status_document() -> dict[str, Any]:
    reporter = ControlStatusReporter(
        publisher=lambda _document: 1,
        monotonic_ns=lambda: 123_456_789,
    )
    return reporter.build_document(_control(), _authority())


def test_full_status_is_exact_truthful_and_keeps_three_motion_layers_separate() -> None:
    document = _status_document()
    encoded = encode_control_status(document)
    decoded = json.loads(encoded)

    assert decoded["schema"] == CONTROL_STATUS_SCHEMA
    assert decoded["server_status_sequence"] == 1
    assert decoded["server_monotonic_ns"] == 123_456_789
    assert decoded["motion"] == {
        "requested_axes": {
            "available": True,
            "forward": 1.0,
            "left": 0.0,
            "yaw_left": 0.0,
        },
        "admitted_twist_mps_radps": {
            "available": True,
            "linear_x": 0.1,
            "linear_y": 0.0,
            "angular_z": 0.0,
        },
        "observed_base_velocity_mps_radps": {
            "available": True,
            "linear_x": 0.08,
            "linear_y": 0.01,
            "angular_z": 0.02,
        },
    }
    assert [stream["stream_id"] for stream in decoded["sensors"]] == [
        "thunder_01.front_depth",
        "thunder_01.front_rgb",
        "thunder_01.imu",
        "thunder_01.mid360",
        "thunder_01.truth_odom",
    ]
    assert decoded["recording"] == {
        "artifact_id": "",
        "blocker": "recording status source unavailable",
        "elapsed_sim_time_ns": 0,
        "state": "unavailable",
    }
    assert len(encoded) <= 4096


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda value: value.update(unexpected=True), "unknown field"),
        (
            lambda value: value["readiness"].pop("visual"),
            "readiness.*missing",
        ),
        (
            lambda value: value["sensors"].append(dict(value["sensors"][0])),
            "unique stream ids",
        ),
        (
            lambda value: value["motion"]["requested_axes"].update(forward=float("nan")),
            "finite",
        ),
        (
            lambda value: value.update(event_id="wrong"),
            "event_id",
        ),
        (
            lambda value: value["recording"].update(
                state="unavailable", artifact_id="invented"
            ),
            "unavailable recording",
        ),
    ],
)
def test_full_status_rejects_unknown_nested_malformed_or_fabricated_state(
    mutate: Any,
    message: str,
) -> None:
    document = _status_document()
    mutate(document)
    with pytest.raises(ControlIntentValidationError, match=message):
        encode_control_status(document)


def test_reporter_publishes_immediately_on_state_change_and_otherwise_at_ten_hz() -> None:
    sent: list[dict[str, Any]] = []
    now = iter((1_000_000_000, 1_010_000_000, 1_050_000_000, 1_100_000_000))

    def publish(document: Mapping[str, Any]) -> int:
        sent.append(dict(document))
        return len(encode_control_status(document))

    reporter = ControlStatusReporter(
        publisher=publish,
        monotonic_ns=lambda: next(now),
    )

    assert reporter.publish_after_advance(_control(), _authority()) > 0
    assert reporter.publish_after_advance(_control(), _authority()) == 0
    assert (
        reporter.publish_after_advance(
            _control(status="released", reason="deadman_released", deadman=False),
            _authority(),
        )
        > 0
    )
    assert (
        reporter.publish_after_advance(
            _control(status="released", reason="deadman_released", deadman=False),
            _authority(),
        )
        > 0
    )
    assert [item["server_status_sequence"] for item in sent] == [1, 2, 3]


def test_reporter_force_publishes_terminal_state_inside_period() -> None:
    sent: list[dict[str, Any]] = []

    def publish(document: Mapping[str, Any]) -> int:
        sent.append(dict(document))
        return len(encode_control_status(document))

    reporter = ControlStatusReporter(
        publisher=publish,
        monotonic_ns=iter((1_000_000_000, 1_001_000_000)).__next__,
    )

    assert reporter.publish_after_advance(_control(), _authority()) > 0
    assert reporter.publish_after_advance(_control(), _authority(), force=True) > 0
    assert [item["server_status_sequence"] for item in sent] == [1, 2]


def test_reporter_is_owner_thread_only_and_rejects_mixed_authority_generation() -> None:
    reporter = ControlStatusReporter(
        publisher=lambda document: len(encode_control_status(document)),
        monotonic_ns=lambda: 1,
    )
    assert reporter.publish_after_advance(_control(), _authority()) > 0

    errors: list[BaseException] = []

    def publish_from_other_thread() -> None:
        try:
            reporter.publish_after_advance(_control(), _authority())
        except BaseException as exc:  # pragma: no branch - exact assertion below
            errors.append(exc)

    thread = threading.Thread(target=publish_from_other_thread)
    thread.start()
    thread.join()
    assert len(errors) == 1
    assert "owner thread" in str(errors[0])

    other = ControlStatusReporter(
        publisher=lambda document: len(encode_control_status(document)),
        monotonic_ns=lambda: 1,
    )
    with pytest.raises(ControlIntentValidationError, match="reset_generation"):
        other.publish_after_advance(
            _control(),
            _authority(reset_generation=9),
        )


def test_published_sink_receives_exact_status_only_after_complete_send() -> None:
    order: list[str] = []
    sent: list[dict[str, Any]] = []
    persisted: list[dict[str, Any]] = []

    def publish(document: Mapping[str, Any]) -> int:
        order.append("udp")
        sent.append(dict(document))
        return len(encode_control_status(document))

    def persist(document: Mapping[str, Any]) -> None:
        order.append("persist")
        persisted.append(dict(document))

    reporter = ControlStatusReporter(
        publisher=publish,
        published_sink=persist,
        monotonic_ns=lambda: 1_000_000_000,
    )
    assert reporter.publish_after_advance(_control(), _authority()) > 0
    assert order == ["udp", "persist"]
    assert persisted == sent

    incomplete = ControlStatusReporter(
        publisher=lambda _document: 1,
        published_sink=lambda _document: order.append("must_not_persist"),
        monotonic_ns=lambda: 1_000_000_000,
    )
    with pytest.raises(ControlIntentValidationError, match="complete encoded"):
        incomplete.publish_after_advance(_control(), _authority())
    assert "must_not_persist" not in order


class _FakeSendSocket:
    def __init__(self, *_: object) -> None:
        self.blocking: bool | None = None
        self.sent: list[tuple[bytes, tuple[str, int]]] = []

    def setblocking(self, value: bool) -> None:
        self.blocking = value

    def sendto(self, payload: bytes, destination: tuple[str, int]) -> int:
        self.sent.append((payload, destination))
        return len(payload)

    def close(self) -> None:
        pass


def test_existing_ack_publisher_schema_switches_without_weakening_ack_v1() -> None:
    fake = _FakeSendSocket()
    publisher = UdpLoopbackControlAckPublisher(
        25125,
        socket_factory=lambda *_: fake,  # type: ignore[arg-type]
    )
    document = _status_document()

    assert publisher.publish(document) == len(encode_control_status(document))
    assert json.loads(fake.sent[0][0])["schema"] == CONTROL_STATUS_SCHEMA
    publisher.close()


def test_playable_pump_publishes_full_status_only_after_admission_and_truth() -> None:
    class Host:
        def __init__(self) -> None:
            self.authority_calls: list[dict[str, Any]] = []

        def submit_controller_command(self, _controller_id: str, _command: Any) -> Any:
            return CommandSubmitResult.ACCEPTED

        def control_status_authority_snapshot(
            self,
            snapshot: Mapping[str, Any],
            *,
            recording_snapshot: Mapping[str, Any] | None = None,
        ) -> dict[str, Any]:
            del recording_snapshot
            self.authority_calls.append(dict(snapshot))
            return _authority(
                sim_time_ns=snapshot["sim_time_ns"],
                truth_sequence=snapshot["sequence"],
            )

    host = Host()
    class Publisher:
        def __init__(self) -> None:
            self.documents: list[dict[str, Any]] = []

        def publish(self, document: Mapping[str, Any]) -> int:
            self.documents.append(dict(document))
            if document.get("schema") == CONTROL_STATUS_SCHEMA:
                return len(encode_control_status(document))
            return 1

    publisher = Publisher()
    motion_inbox = LatestOperatorIntentInbox()
    motion_inbox.put(
        OperatorMotionIntent(
            identity=_identity(),
            source_epoch=1,
            source_sequence=42,
            event_id="boot-playable-001:1:42",
            input_mode="drive",
            input_device="keyboard",
            viewport_focused=True,
            deadman=True,
            axes=OperatorMotionAxes(forward=1.0, left=0.0, yaw_left=0.0),
            active_controls=("keyboard.left_shift", "keyboard.w"),
            source_monotonic_ns=900_000_000,
            arrival_monotonic_ns=1_000_000_000,
            datagram_sha256="b" * 64,
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
        ack_publisher=publisher,
        monotonic_ns=lambda: 1_050_000_000,
    )
    before = {
        "event": "snapshot",
        "model_generation": 3,
        "reset_generation": 2,
        "sequence": 17,
        "sim_time_ns": 900_000_000,
    }
    after = {**before, "sequence": 18, "sim_time_ns": 902_000_000}

    assert pump.publish_status_after_advance(after, runtime_state="RUNNING") == 0
    pump.process_before_advance(before)
    assert publisher.documents[-1]["schema"] == "lingtu.sim.ue-control-ack.v1"
    assert pump.publish_status_after_advance(after, runtime_state="RUNNING") > 0
    assert publisher.documents[-1]["schema"] == CONTROL_STATUS_SCHEMA
    assert publisher.documents[-1]["source_sequence"] == 42
    assert publisher.documents[-1]["motion"]["requested_axes"]["forward"] == 1.0
    assert publisher.documents[-1]["motion"]["admitted_twist_mps_radps"] == {
        "available": True,
        "linear_x": 0.1,
        "linear_y": 0.0,
        "angular_z": 0.0,
    }
    assert host.authority_calls == [after]
    assert pump.publish_status_after_advance(after, runtime_state="RUNNING") == 0
    assert host.authority_calls == [after]


def test_authority_builder_uses_coordinator_manifest_counts_and_mujoco_truth() -> None:
    identity = {
        "run_id": "run-playable-001",
        "session_id": "a" * 64,
        "boot_id": "boot-playable-001",
        "model_generation": 3,
        "reset_generation": 2,
    }
    bindings = {
        name: {
            "required": True,
            "state": "ACTIVE",
            "source_id": f"runtime.{name}",
            "failure_reason": None,
            "model_generation": 3,
            "reset_generation": 2,
        }
        for name in ("physics", "control", "visual", "sensors")
    }
    summary_streams = {
        stream_id: {
            "state": "ACTIVE",
            "sample_count": count,
        }
        for count, stream_id in enumerate(
            (
                "thunder_01.front_depth",
                "thunder_01.front_rgb",
                "thunder_01.imu",
                "thunder_01.mid360",
                "thunder_01.truth_odom",
            ),
            start=1,
        )
    }
    manifest = {
        "schema": "lingtu.sim.session-runtime.v1",
        "run_id": identity["run_id"],
        "session_id": identity["session_id"],
        "model_generation": 3,
        "reset_generation": 2,
        "state": "RUNNING",
        "bindings": bindings,
        "sensor_streams": {
            "summary": {
                "schema": "lingtu.sim.sensor-stream-summary.v1",
                "session_id": identity["session_id"],
                "model_generation": 3,
                "reset_generation": 2,
                "required_stream_ids": list(summary_streams),
                "blocking_reasons": {},
                "streams": summary_streams,
            }
        },
    }
    snapshot = {
        "event": "snapshot",
        "session_id": identity["session_id"],
        "model_generation": 3,
        "reset_generation": 2,
        "sequence": 600,
        "sim_time_ns": 2_000_000_000,
        "bodies": [
            {
                "stable_id": "thunder_01/base_link",
                "linear_velocity_mps": [0.08, 0.01, 0.0],
                "angular_velocity_rps": [0.0, 0.0, -0.02],
            }
        ],
    }

    authority = build_control_status_authority(
        identity=identity,
        runtime_state="RUNNING",
        snapshot=snapshot,
        runtime_manifest=manifest,
    )

    assert authority["observed_base_velocity_mps_radps"] == {
        "available": True,
        "linear_x": 0.08,
        "linear_y": 0.01,
        "angular_z": -0.02,
    }
    assert [item["sample_count"] for item in authority["sensors"]] == [1, 2, 3, 4, 5]
    assert authority["recording"] == {
        "state": "unavailable",
        "elapsed_sim_time_ns": 0,
        "artifact_id": "",
        "blocker": "recording status source unavailable",
    }

    recording = {
        "schema": "lingtu.sim.recording-status.v1",
        "run_id": identity["run_id"],
        "session_id": identity["session_id"],
        "model_generation": 3,
        "reset_generation": 2,
        "state": "recording",
        "elapsed_sim_time_ns": 900_000_000,
        "artifact_id": "simulation-recording.json",
        "blocker": "",
    }
    authoritative_recording = build_control_status_authority(
        identity=identity,
        runtime_state="RUNNING",
        snapshot=snapshot,
        runtime_manifest=manifest,
        recording_snapshot=recording,
    )
    assert authoritative_recording["recording"]["state"] == "recording"

    unavailable_recording = dict(recording)
    unavailable_recording.update(
        state="unavailable",
        elapsed_sim_time_ns=0,
        artifact_id="",
        blocker="recording controller is not bound",
    )
    unavailable_authority = build_control_status_authority(
        identity=identity,
        runtime_state="RUNNING",
        snapshot=snapshot,
        runtime_manifest=manifest,
        recording_snapshot=unavailable_recording,
    )
    assert unavailable_authority["recording"] == {
        "state": "unavailable",
        "elapsed_sim_time_ns": 0,
        "artifact_id": "",
        "blocker": "recording controller is not bound",
    }

    projected_recording = dict(recording)
    projected_recording.update(state="requested")
    with pytest.raises(ControlIntentValidationError, match="state"):
        build_control_status_authority(
            identity=identity,
            runtime_state="RUNNING",
            snapshot=snapshot,
            runtime_manifest=manifest,
            recording_snapshot=projected_recording,
        )

    manifest_without_sensor_source = dict(manifest)
    manifest_without_sensor_source["sensor_streams"] = None
    unavailable_sensors = build_control_status_authority(
        identity=identity,
        runtime_state="RUNNING",
        snapshot=snapshot,
        runtime_manifest=manifest_without_sensor_source,
    )
    assert unavailable_sensors["sensors"] == []

    stale = dict(manifest)
    stale["reset_generation"] = 1
    with pytest.raises(ControlIntentValidationError, match="reset_generation"):
        build_control_status_authority(
            identity=identity,
            runtime_state="RUNNING",
            snapshot=snapshot,
            runtime_manifest=stale,
        )


def test_session_host_reads_atomic_manifest_and_injects_recording_snapshot(
    tmp_path: Path,
) -> None:
    manifest_path = tmp_path / "session.runtime.json"
    streams = {
        stream_id: {"state": "ACTIVE", "sample_count": index}
        for index, stream_id in enumerate(
            (
                "thunder_01.front_depth",
                "thunder_01.front_rgb",
                "thunder_01.imu",
                "thunder_01.mid360",
                "thunder_01.truth_odom",
            ),
            start=1,
        )
    }
    manifest = {
        "schema": "lingtu.sim.session-runtime.v1",
        "run_id": "run-playable-001",
        "session_id": "a" * 64,
        "model_generation": 3,
        "reset_generation": 2,
        "state": "RUNNING",
        "bindings": {
            name: {
                "required": True,
                "state": "ACTIVE",
                "source_id": f"runtime.{name}",
                "failure_reason": None,
                "model_generation": 3,
                "reset_generation": 2,
            }
            for name in ("physics", "control", "visual", "sensors")
        },
        "sensor_streams": {
            "summary": {
                "schema": "lingtu.sim.sensor-stream-summary.v1",
                "session_id": "a" * 64,
                "model_generation": 3,
                "reset_generation": 2,
                "required_stream_ids": list(streams),
                "blocking_reasons": {},
                "streams": streams,
            }
        },
    }
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    coordinator = SimpleNamespace(
        state=RuntimeState.RUNNING,
        manifest_path=manifest_path,
        allocation=SimpleNamespace(
            run_id="run-playable-001",
            session_id="a" * 64,
            boot_id="boot-playable-001",
        ),
    )
    host = SessionHost(
        coordinator=coordinator,
        unreal_process=SimpleNamespace(),
        publisher=SimpleNamespace(),
    )
    snapshot = {
        "event": "snapshot",
        "session_id": "a" * 64,
        "model_generation": 3,
        "reset_generation": 2,
        "sequence": 600,
        "sim_time_ns": 2_000_000_000,
        "bodies": [
            {
                "stable_id": "thunder_01/base_link",
                "linear_velocity_mps": [0.08, 0.01, 0.0],
                "angular_velocity_rps": [0.0, 0.0, 0.02],
            }
        ],
    }
    recording = {
        "schema": "lingtu.sim.recording-status.v1",
        "run_id": "run-playable-001",
        "session_id": "a" * 64,
        "model_generation": 3,
        "reset_generation": 2,
        "state": "recording",
        "elapsed_sim_time_ns": 900_000_000,
        "artifact_id": "simulation-recording.json",
        "blocker": "",
    }

    authority = host.control_status_authority_snapshot(
        snapshot,
        recording_snapshot=recording,
    )
    assert authority["recording"]["state"] == "recording"
    assert authority["truth_sequence"] == 600

    manifest_path.write_text("not-json", encoding="utf-8")
    with pytest.raises(CoordinatorError, match="current runtime manifest"):
        host.control_status_authority_snapshot(snapshot)


def test_session_host_prefers_owner_thread_manifest_snapshot_over_disk(
    tmp_path: Path,
) -> None:
    manifest_path = tmp_path / "session.runtime.json"
    manifest_path.write_text("not-json", encoding="utf-8")
    streams = {
        stream_id: {"state": "ACTIVE", "sample_count": index}
        for index, stream_id in enumerate(
            (
                "thunder_01.front_depth",
                "thunder_01.front_rgb",
                "thunder_01.imu",
                "thunder_01.mid360",
                "thunder_01.truth_odom",
            ),
            start=1,
        )
    }
    manifest = {
        "schema": "lingtu.sim.session-runtime.v1",
        "run_id": "run-playable-001",
        "session_id": "a" * 64,
        "model_generation": 3,
        "reset_generation": 2,
        "state": "RUNNING",
        "bindings": {
            name: {
                "required": True,
                "state": "ACTIVE",
                "source_id": f"runtime.{name}",
                "failure_reason": None,
                "model_generation": 3,
                "reset_generation": 2,
            }
            for name in ("physics", "control", "visual", "sensors")
        },
        "sensor_streams": {
            "summary": {
                "schema": "lingtu.sim.sensor-stream-summary.v1",
                "session_id": "a" * 64,
                "model_generation": 3,
                "reset_generation": 2,
                "required_stream_ids": list(streams),
                "blocking_reasons": {},
                "streams": streams,
            }
        },
    }

    class Coordinator:
        state = RuntimeState.RUNNING
        allocation = SimpleNamespace(
            run_id="run-playable-001",
            session_id="a" * 64,
            boot_id="boot-playable-001",
        )

        def __init__(self) -> None:
            self.manifest_path = manifest_path
            self.snapshot_calls = 0

        def runtime_manifest_snapshot(self) -> Mapping[str, Any]:
            self.snapshot_calls += 1
            return manifest

    coordinator = Coordinator()
    host = SessionHost(
        coordinator=coordinator,  # type: ignore[arg-type]
        unreal_process=SimpleNamespace(),
        publisher=SimpleNamespace(),
    )
    snapshot = {
        "event": "snapshot",
        "session_id": "a" * 64,
        "model_generation": 3,
        "reset_generation": 2,
        "sequence": 601,
        "sim_time_ns": 2_100_000_000,
        "bodies": [],
    }

    authority = host.control_status_authority_snapshot(snapshot)

    assert authority["truth_sequence"] == 601
    assert coordinator.snapshot_calls == 1
