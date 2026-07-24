from __future__ import annotations

import json
import threading
import time

from fastapi.testclient import TestClient

from gateway.gateway_module import GatewayModule
from nav.commands.operator_motion import OperatorMotion


class RecordingCommands:
    def __init__(self) -> None:
        self.calls: list[tuple[float, float, float, str | None]] = []
        self.events: list[tuple[str, str | None]] = []
        self.called = threading.Event()
        self.zero_called = threading.Event()
        self.stop_called = threading.Event()
        self.resume_called = threading.Event()

    def send_teleop(
        self,
        vx: float,
        vy: float,
        wz: float,
        request_id: str | None = None,
    ) -> bool:
        self.calls.append((vx, vy, wz, request_id))
        self.events.append(("zero" if (vx, vy, wz) == (0.0, 0.0, 0.0) else "teleop", request_id))
        self.called.set()
        if (vx, vy, wz) == (0.0, 0.0, 0.0):
            self.zero_called.set()
        return True

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


def wait_until(predicate, timeout_s: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return bool(predicate())


def test_websocket_is_a_thin_operator_motion_adapter(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    commands = RecordingCommands()
    motion = OperatorMotion()
    motion.on_system_modules({"nav.commands": commands})
    motion.start()
    gateway = GatewayModule(manage_session_services=False)
    gateway.setup()
    gateway.on_system_modules(
        {
            "nav.commands": commands,
            "operator.motion": motion,
        }
    )
    teleop_lifecycle = RecordingTeleopLifecycle()
    gateway._teleop_module = teleop_lifecycle

    try:
        client = TestClient(gateway._app)
        with client.websocket_connect("/ws/teleop?client_id=operator-a") as ws:
            assert motion.health()["session"]["active"] is True

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

            assert receipt == {
                "type": "control_ack",
                "action": "queued",
                "accepted": True,
                "stage": "queued",
                "request_id": "ws-joy-1",
            }
            assert commands.called.wait(1.0)
            assert commands.calls[0] == (0.2, -0.1, 0.3, "ws-joy-1")

        assert commands.zero_called.wait(3.0)
        assert wait_until(lambda: motion.health()["session"]["active"] is False, timeout_s=3.0)
        assert commands.calls[-1][:3] == (0.0, 0.0, 0.0)
        assert teleop_lifecycle.connects == 1
        assert teleop_lifecycle.disconnects == 1
    finally:
        gateway.stop()
        motion.stop()


def test_websocket_quiesces_operator_motion_before_stop_and_resume(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    commands = RecordingCommands()
    motion = OperatorMotion()
    motion.on_system_modules({"nav.commands": commands})
    motion.start()
    gateway = GatewayModule(manage_session_services=False)
    gateway.setup()
    gateway.on_system_modules({"nav.commands": commands, "operator.motion": motion})

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
            assert json.loads(ws.receive_text())["stage"] == "queued"
            assert commands.called.wait(1.0)

            ws.send_text(json.dumps({"type": "stop", "request_id": "stop-1"}))
            assert commands.stop_called.wait(3.0)
            assert [event[0] for event in commands.events[:3]] == ["teleop", "zero", "stop"]
            assert commands.events[2] == ("stop", "stop-1")

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
            assert json.loads(ws.receive_text())["stage"] == "queued"
            assert commands.called.wait(1.0)

            ws.send_text(json.dumps({"type": "resume_autonomy", "request_id": "resume-1"}))
            resume_receipt = json.loads(ws.receive_text())
            assert resume_receipt["action"] == "resume_autonomy"
            assert resume_receipt["accepted"] is True
            assert commands.resume_called.wait(3.0)
            assert [event[0] for event in commands.events[3:6]] == [
                "teleop",
                "zero",
                "resume_autonomy",
            ]
            assert commands.events[5] == ("resume_autonomy", "resume-1")
    finally:
        gateway.stop()
        motion.stop()
