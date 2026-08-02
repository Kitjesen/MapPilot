from __future__ import annotations

import json
import threading
import time

from fastapi.testclient import TestClient

from gateway.gateway_module import GatewayModule
from runtime.msgs.nav import OperatorMotionAction, OperatorMotionReceipt


def _receipt(
    action: OperatorMotionAction,
    source_id: str,
    source_epoch: int,
    sequence: int,
    request_id: str | None,
    *,
    final_output_sequence: int = 0,
    reason: str = "accepted",
) -> OperatorMotionReceipt:
    return OperatorMotionReceipt(
        accepted=True,
        action=int(action),
        request_id=str(request_id or f"{source_id}:{action.name.lower()}:{sequence}"),
        source_id=source_id,
        source_epoch=source_epoch,
        source_sequence=sequence,
        accepted_sequence=sequence,
        final_output_sequence=final_output_sequence,
        endpoint_timestamp_s=time.time() or 1.0,
        reason=reason,
    )

class RecordingCommands:
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []
        self.events: list[tuple[str, str | None]] = []
        self.called = threading.Event()
        self.hold_called = threading.Event()
        self.stop_called = threading.Event()
        self.resume_called = threading.Event()
        self.release_called = threading.Event()

    def claim(
        self,
        source_id: str,
        source_epoch: int,
        sequence: int,
        *,
        lease_ttl_ms: int,
        request_id: str | None = None,
    ) -> OperatorMotionReceipt:
        self.calls.append(
            ("claim", source_id, source_epoch, sequence, lease_ttl_ms, request_id)
        )
        self.events.append(("claim", request_id))
        return _receipt(OperatorMotionAction.CLAIM, source_id, source_epoch, sequence, request_id)

    def sample(
        self,
        source_id: str,
        source_epoch: int,
        sequence: int,
        vx: float,
        vy: float,
        wz: float,
        *,
        deadman: bool = True,
        freshness_budget_ms: int = 350,
        request_id: str | None = None,
    ) -> bool:
        self.calls.append(
            (
                "sample",
                source_id,
                source_epoch,
                sequence,
                vx,
                vy,
                wz,
                deadman,
                freshness_budget_ms,
                request_id,
            )
        )
        self.events.append(("sample", request_id))
        self.called.set()
        return True

    def hold(
        self,
        source_id: str,
        source_epoch: int,
        sequence: int,
        *,
        reason: str = "operator_hold",
        request_id: str | None = None,
    ) -> OperatorMotionReceipt:
        self.calls.append(("hold", source_id, source_epoch, sequence, reason, request_id))
        self.events.append(("hold", request_id))
        self.hold_called.set()
        return _receipt(
            OperatorMotionAction.HOLD,
            source_id,
            source_epoch,
            sequence,
            request_id,
            final_output_sequence=sequence,
            reason=reason,
        )

    def release(
        self,
        source_id: str,
        source_epoch: int,
        sequence: int,
        *,
        reason: str = "operator_release",
        request_id: str | None = None,
    ) -> OperatorMotionReceipt:
        self.calls.append(
            ("release", source_id, source_epoch, sequence, reason, request_id)
        )
        self.events.append(("release", request_id))
        self.release_called.set()
        return _receipt(
            OperatorMotionAction.RELEASE,
            source_id,
            source_epoch,
            sequence,
            request_id,
            final_output_sequence=sequence,
            reason=reason,
        )

    def stop_motion(self, reason: str = "stop", request_id: str | None = None) -> bool:
        self.events.append(("stop", request_id))
        self.stop_called.set()
        return True

    def resume_autonomy(
        self,
        reason: str = "resume_autonomy",
        request_id: str | None = None,
    ) -> bool:
        self.events.append(("resume_autonomy", request_id))
        self.resume_called.set()
        return True


class RecordingTeleopLifecycle:
    def __init__(self) -> None:
        self.connects = 0
        self.disconnects = 0

    def on_client_connect(self) -> None:
        self.connects += 1

    def on_client_disconnect(self) -> None:
        self.disconnects += 1


def test_websocket_joy_reports_ingress_only_and_uses_native_command_path(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    commands = RecordingCommands()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.commands": commands})
    media_lifecycle = RecordingTeleopLifecycle()
    gateway._camera_module = media_lifecycle

    try:
        client = TestClient(gateway._app)
        with client.websocket_connect("/ws/teleop?client_id=operator-a") as ws:
            ws.send_text(
                json.dumps(
                    {
                        "type": "joy",
                        "lx": 0.4,
                        "ly": -0.2,
                        "az": 0.3,
                        "deadman": True,
                        "sequence": 1,
                        "request_id": "ws-joy-1",
                    }
                )
            )
            receipt = json.loads(ws.receive_text())

            assert receipt["type"] == "ingress_ack"
            assert receipt["action"] == "queued"
            assert receipt["stage"] == "gateway_queue_accepted"
            assert receipt["request_id"] == "ws-joy-1"
            assert receipt["source_sequence"] == 2
            assert receipt["replaceable"] is True
            assert receipt["sample_ack_expected"] is False
            assert receipt["final_cmd_vel_confirmed"] is False
            assert receipt["motor_confirmed"] is False
            assert commands.called.wait(1.0)
            assert commands.calls[1][0] == "sample"
            assert commands.calls[1][4:] == (0.2, -0.1, 0.3, True, 350, "ws-joy-1")

        assert commands.hold_called.wait(3.0)
        assert commands.release_called.wait(3.0)
        assert commands.calls[-2][0] == "hold"
        assert commands.calls[-1][0] == "release"
        assert [call[3] for call in commands.calls] == [1, 2, 3, 4]
        assert media_lifecycle.connects == 1
        assert media_lifecycle.disconnects == 1
    finally:
        gateway.stop()


def test_websocket_manual_hold_reports_final_logical_zero_not_motor(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    commands = RecordingCommands()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.commands": commands})

    try:
        client = TestClient(gateway._app)
        with client.websocket_connect("/ws/teleop?client_id=operator-hold") as ws:
            ws.send_text(
                json.dumps(
                    {
                        "type": "joy",
                        "lx": 0.0,
                        "ly": 0.0,
                        "az": 0.0,
                        "deadman": False,
                        "request_id": "hold-1",
                    }
                )
            )
            receipt = json.loads(ws.receive_text())

            assert receipt["type"] == "control_ack"
            assert receipt["action"] == "manual_hold"
            assert receipt["accepted"] is True
            assert receipt["request_id"] == "hold-1"
            assert receipt["source_sequence"] == 2
            assert receipt["stage"] == "final_zero_published"
            assert receipt["final_cmd_vel_confirmed"] is True
            assert receipt["motor_confirmed"] is False
            assert receipt["native_receipt"]["action_name"] == "HOLD"
            assert receipt["native_receipt"]["final_output_published"] is True
    finally:
        gateway.stop()


def test_websocket_quiesces_native_teleop_before_stop_and_resume(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    commands = RecordingCommands()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.commands": commands})

    try:
        client = TestClient(gateway._app)
        with client.websocket_connect("/ws/teleop?client_id=operator-a") as ws:
            ws.send_text(
                json.dumps(
                    {
                        "type": "joy",
                        "lx": 0.4,
                        "ly": 0.0,
                        "az": 0.0,
                        "deadman": True,
                        "sequence": 1,
                        "request_id": "joy-before-stop",
                    }
                )
            )
            assert json.loads(ws.receive_text())["stage"] == "gateway_queue_accepted"
            assert commands.called.wait(1.0)

            ws.send_text(json.dumps({"type": "stop", "request_id": "stop-1"}))
            stop_receipt = json.loads(ws.receive_text())
            assert stop_receipt["type"] == "control_ack"
            assert stop_receipt["action"] == "stop"
            assert stop_receipt["accepted"] is True
            assert stop_receipt["request_id"] == "stop-1"
            assert stop_receipt["source_sequence"] == 3
            assert stop_receipt["stage"] == "native_stop_acknowledged"
            assert stop_receipt["native_command_acknowledged"] is True
            assert stop_receipt["final_cmd_vel_confirmed"] is True
            assert stop_receipt["motor_confirmed"] is False
            assert stop_receipt["native_receipt"]["action_name"] == "HOLD"
            assert commands.stop_called.wait(3.0)
            assert [event[0] for event in commands.events[:4]] == ["claim", "sample", "hold", "stop"]
            assert commands.events[3] == ("stop", "stop-1")

            commands.called.clear()
            ws.send_text(
                json.dumps(
                    {
                        "type": "joy",
                        "lx": 0.2,
                        "ly": 0.0,
                        "az": 0.0,
                        "deadman": True,
                        "sequence": 2,
                        "request_id": "joy-before-resume",
                    }
                )
            )
            assert json.loads(ws.receive_text())["stage"] == "gateway_queue_accepted"
            assert commands.called.wait(1.0)

            ws.send_text(json.dumps({"type": "resume_autonomy", "request_id": "resume-1"}))
            resume_receipt = json.loads(ws.receive_text())
            assert resume_receipt["action"] == "resume_autonomy"
            assert resume_receipt["accepted"] is True
            assert commands.resume_called.wait(3.0)
            assert [event[0] for event in commands.events[4:7]] == [
                "sample",
                "hold",
                "resume_autonomy",
            ]
            assert commands.events[6] == ("resume_autonomy", "resume-1")
    finally:
        gateway.stop()


def test_websocket_local_stop_reports_logical_zero_without_native_or_motor_ack(
    monkeypatch,
) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "local_driver")
    gateway = GatewayModule()
    gateway.setup()

    try:
        client = TestClient(gateway._app)
        with client.websocket_connect("/ws/teleop?client_id=operator-local-stop") as ws:
            ws.send_text(json.dumps({"type": "stop", "request_id": "local-stop-1"}))
            receipt = json.loads(ws.receive_text())

            assert receipt == {
                "type": "control_ack",
                "action": "stop",
                "accepted": True,
                "request_id": "local-stop-1",
                "source_sequence": 1,
                "stage": "local_zero_requested",
                "native_command_acknowledged": False,
                "final_cmd_vel_confirmed": False,
                "motor_confirmed": False,
            }
    finally:
        gateway.stop()
