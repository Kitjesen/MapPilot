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
        self.sample_manual_modes: list[bool] = []
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
        manual_mode: bool = False,
        freshness_budget_ms: int = 350,
        request_id: str | None = None,
    ) -> bool:
        self.sample_manual_modes.append(manual_mode)
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

    def resume_autonomy_with_receipt(
        self,
        reason: str = "resume_autonomy",
        request_id: str | None = None,
    ) -> dict[str, object]:
        self.events.append(("resume_autonomy_with_receipt", request_id))
        self.resume_called.set()
        return {
            "accepted": True,
            "kind": 7,
            "task_id": "",
            "request_id": str(request_id or "native-generated"),
            "reason": "teleop_resume_ready_reassert_command",
            "endpoint_timestamp_s": 123.5,
        }


class UnconfirmedHoldCommands(RecordingCommands):
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
            final_output_sequence=0,
            reason="final_output_not_published",
        )


class ExpiredLeaseCommands(RecordingCommands):
    """Model navd's zero barrier before a new source epoch can reclaim control."""

    def __init__(self) -> None:
        super().__init__()
        self.claim_epochs: list[int] = []

    def claim(
        self,
        source_id: str,
        source_epoch: int,
        sequence: int,
        *,
        lease_ttl_ms: int,
        request_id: str | None = None,
    ) -> OperatorMotionReceipt:
        self.claim_epochs.append(source_epoch)
        if len(self.claim_epochs) != 2:
            return super().claim(
                source_id,
                source_epoch,
                sequence,
                lease_ttl_ms=lease_ttl_ms,
                request_id=request_id,
            )
        self.calls.append(
            ("claim", source_id, source_epoch, sequence, lease_ttl_ms, request_id)
        )
        self.events.append(("claim", request_id))
        return OperatorMotionReceipt(
            accepted=False,
            action=int(OperatorMotionAction.CLAIM),
            request_id=str(request_id or f"{source_id}:claim:{sequence}"),
            source_id=source_id,
            source_epoch=source_epoch,
            source_sequence=sequence,
            accepted_sequence=0,
            final_output_sequence=0,
            endpoint_timestamp_s=time.time() or 1.0,
            reason="authority_lease_expired",
        )


class BusyCommands(RecordingCommands):
    def __init__(self) -> None:
        super().__init__()
        self.active_source_id: str | None = None

    def claim(
        self,
        source_id: str,
        source_epoch: int,
        sequence: int,
        *,
        lease_ttl_ms: int,
        request_id: str | None = None,
    ) -> OperatorMotionReceipt:
        if self.active_source_id not in (None, source_id):
            self.calls.append(
                ("claim", source_id, source_epoch, sequence, lease_ttl_ms, request_id)
            )
            self.events.append(("claim", request_id))
            return OperatorMotionReceipt(
                accepted=False,
                action=int(OperatorMotionAction.CLAIM),
                request_id=str(request_id or f"{source_id}:claim:{sequence}"),
                source_id=source_id,
                source_epoch=source_epoch,
                source_sequence=sequence,
                accepted_sequence=0,
                final_output_sequence=0,
                endpoint_timestamp_s=time.time() or 1.0,
                reason="authority_busy",
            )
        self.active_source_id = source_id
        return super().claim(
            source_id,
            source_epoch,
            sequence,
            lease_ttl_ms=lease_ttl_ms,
            request_id=request_id,
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
        receipt = super().release(
            source_id,
            source_epoch,
            sequence,
            reason=reason,
            request_id=request_id,
        )
        if self.active_source_id == source_id:
            self.active_source_id = None
        return receipt


class RecordingTeleopLifecycle:
    def __init__(self) -> None:
        self.connects = 0
        self.disconnects = 0

    def on_client_connect(self) -> None:
        self.connects += 1

    def on_client_disconnect(self) -> None:
        self.disconnects += 1


def test_websocket_velocity_reports_ingress_only_and_uses_native_command_path(monkeypatch) -> None:
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
                        "type": "velocity",
                        "vx_mps": 0.4,
                        "vy_mps": -0.2,
                        "yaw_rps": 0.3,
                        "deadman": True,
                        "manual_mode": True,
                        "sequence": 1,
                        "request_id": "ws-velocity-1",
                    }
                )
            )
            receipt = json.loads(ws.receive_text())

            assert receipt["type"] == "ingress_ack"
            assert receipt["action"] == "queued"
            assert receipt["stage"] == "gateway_queue_accepted"
            assert receipt["request_id"] == "ws-velocity-1"
            assert receipt["replaceable"] is True
            assert receipt["final_cmd_vel_confirmed"] is False
            assert receipt["motor_confirmed"] is False
            assert "source_sequence" not in receipt
            assert "sample_ack_expected" not in receipt
            assert commands.called.wait(1.0)
            assert commands.calls[1][0] == "sample"
            assert commands.calls[1][4:] == (0.4, -0.2, 0.3, True, 350, "ws-velocity-1")

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
                        "type": "velocity",
                        "vx_mps": 0.2,
                        "vy_mps": 0.0,
                        "yaw_rps": 0.0,
                        "deadman": True,
                        "request_id": "move-before-hold",
                    }
                )
            )
            assert json.loads(ws.receive_text())["type"] == "ingress_ack"
            assert commands.called.wait(1.0)
            assert commands.sample_manual_modes == [False]

            ws.send_text(
                json.dumps(
                    {
                        "type": "velocity",
                        "vx_mps": 0.0,
                        "vy_mps": 0.0,
                        "yaw_rps": 0.0,
                        "deadman": False,
                        "request_id": "hold-1",
                    }
                )
            )
            receipt = json.loads(ws.receive_text())

            assert receipt["type"] == "control_ack"
            assert receipt["action"] == "hold"
            assert receipt["accepted"] is True
            assert receipt["request_id"] == "hold-1"
            assert receipt["stage"] == "final_zero_published"
            assert receipt["final_cmd_vel_confirmed"] is True
            assert receipt["motor_confirmed"] is False
            assert "source_sequence" not in receipt
            assert "native_receipt" not in receipt
    finally:
        gateway.stop()


def test_websocket_hold_then_velocity_reclaims_control_without_resume(monkeypatch) -> None:
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
                        "type": "velocity",
                        "vx_mps": 0.4,
                        "vy_mps": 0.0,
                        "yaw_rps": 0.0,
                        "deadman": True,
                        "request_id": "velocity-before-hold",
                    }
                )
            )
            assert json.loads(ws.receive_text())["stage"] == "gateway_queue_accepted"
            assert commands.called.wait(1.0)

            ws.send_text(
                json.dumps(
                    {
                        "type": "velocity",
                        "vx_mps": 0.0,
                        "vy_mps": 0.0,
                        "yaw_rps": 0.0,
                        "deadman": False,
                        "request_id": "hold-1",
                    }
                )
            )
            hold_receipt = json.loads(ws.receive_text())
            assert hold_receipt["type"] == "control_ack"
            assert hold_receipt["action"] == "hold"
            assert commands.hold_called.wait(1.0)

            commands.called.clear()
            ws.send_text(
                json.dumps(
                    {
                        "type": "velocity",
                        "vx_mps": 0.2,
                        "vy_mps": 0.0,
                        "yaw_rps": 0.0,
                        "deadman": True,
                        "request_id": "velocity-after-hold",
                    }
                )
            )
            assert json.loads(ws.receive_text())["stage"] == "gateway_queue_accepted"
            assert commands.called.wait(1.0)
            assert [event[0] for event in commands.events[:5]] == [
                "claim",
                "sample",
                "hold",
                "claim",
                "sample",
            ]
            assert commands.resume_called.is_set() is False
    finally:
        gateway.stop()


def test_websocket_does_not_consume_the_rest_control_lease(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    commands = RecordingCommands()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.commands": commands})
    assert gateway._lease.acquire("rest-owner", 30.0) is True

    try:
        client = TestClient(gateway._app)
        with client.websocket_connect("/ws/teleop?client_id=operator-a") as ws:
            ws.send_text(
                json.dumps(
                    {
                        "type": "velocity",
                        "vx_mps": 0.2,
                        "vy_mps": 0.0,
                        "yaw_rps": 0.0,
                        "deadman": True,
                        "request_id": "move-with-rest-lease",
                    }
                )
            )
            receipt = json.loads(ws.receive_text())

            assert receipt["type"] == "ingress_ack"
            assert commands.called.wait(1.0)
            assert gateway._lease.to_dict()["holder"] == "rest-owner"
    finally:
        gateway.stop()


def test_websocket_recovers_expired_native_authority_inside_gateway(monkeypatch) -> None:
    """The browser sends a fresh move; native epoch/claim recovery stays internal."""

    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    commands = ExpiredLeaseCommands()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.commands": commands})

    try:
        client = TestClient(gateway._app)
        with client.websocket_connect("/ws/teleop?client_id=operator-idle") as ws:
            ws.send_text(
                json.dumps(
                    {
                        "type": "velocity",
                        "vx_mps": 0.2,
                        "vy_mps": 0.0,
                        "yaw_rps": 0.0,
                        "deadman": True,
                        "request_id": "velocity-before-idle",
                    }
                )
            )
            assert json.loads(ws.receive_text())["type"] == "ingress_ack"
            assert commands.called.wait(1.0)
            commands.called.clear()

            ws.send_text(
                json.dumps(
                    {
                        "type": "velocity",
                        "vx_mps": 0.0,
                        "vy_mps": 0.0,
                        "yaw_rps": 0.0,
                        "deadman": False,
                        "request_id": "idle-hold",
                    }
                )
            )
            assert json.loads(ws.receive_text())["action"] == "hold"

            ws.send_text(
                json.dumps(
                    {
                        "type": "velocity",
                        "vx_mps": 0.35,
                        "vy_mps": 0.0,
                        "yaw_rps": 0.0,
                        "deadman": True,
                        "request_id": "velocity-after-idle",
                    }
                )
            )
            receipt = json.loads(ws.receive_text())

            assert receipt["type"] == "ingress_ack"
            assert receipt["action"] == "queued"
            assert receipt["request_id"] == "velocity-after-idle"
            assert "source_epoch" not in receipt
            assert "source_sequence" not in receipt
            assert "native_receipt" not in receipt
            assert commands.called.wait(1.0)
            assert [event[0] for event in commands.events[:6]] == [
                "claim",
                "sample",
                "hold",
                "claim",
                "claim",
                "sample",
            ]
            assert commands.claim_epochs[1] == commands.claim_epochs[0]
            assert commands.claim_epochs[2] > commands.claim_epochs[1]
    finally:
        gateway.stop()


def test_websocket_hides_native_hold_failure_details(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    commands = UnconfirmedHoldCommands()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.commands": commands})

    try:
        client = TestClient(gateway._app)
        with client.websocket_connect("/ws/teleop?client_id=operator-a") as ws:
            ws.send_text(
                json.dumps(
                    {
                        "type": "velocity",
                        "vx_mps": 0.2,
                        "vy_mps": 0.0,
                        "yaw_rps": 0.0,
                        "deadman": True,
                        "request_id": "move-before-hold",
                    }
                )
            )
            assert json.loads(ws.receive_text())["type"] == "ingress_ack"
            assert commands.called.wait(1.0)

            ws.send_text(
                json.dumps(
                    {
                        "type": "velocity",
                        "vx_mps": 0.0,
                        "vy_mps": 0.0,
                        "yaw_rps": 0.0,
                        "deadman": False,
                        "request_id": "hold-1",
                    }
                )
            )
            response = json.loads(ws.receive_text())

            assert response["type"] == "control_rejected"
            assert response["error"] == "hold_unconfirmed"
            assert response["final_cmd_vel_confirmed"] is False
            assert "native_receipt" not in response
            assert "reason" not in response
            assert "final_output_not_published" not in json.dumps(response)
            assert commands.hold_called.wait(1.0)
    finally:
        gateway.stop()


def test_websocket_rejects_second_connected_controller_without_touching_first(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    commands = RecordingCommands()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.commands": commands})

    try:
        client = TestClient(gateway._app)
        with client.websocket_connect("/ws/teleop?client_id=operator-a") as first:
            with client.websocket_connect("/ws/teleop?client_id=operator-b") as second:
                rejected = json.loads(second.receive_text())
                assert rejected == {
                    "type": "control_rejected",
                    "error": "control_in_use",
                    "message": "Another operator is connected.",
                }

            first.send_text(
                json.dumps(
                    {
                        "type": "velocity",
                        "vx_mps": 0.2,
                        "vy_mps": 0.0,
                        "yaw_rps": 0.0,
                        "deadman": True,
                        "request_id": "first-still-controls",
                    }
                )
            )
            assert json.loads(first.receive_text())["type"] == "ingress_ack"
            assert commands.called.wait(1.0)
    finally:
        gateway.stop()


def test_websocket_maps_native_busy_to_stable_public_error(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    commands = BusyCommands()
    commands.active_source_id = "native-controller"
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.commands": commands})

    try:
        client = TestClient(gateway._app)
        with client.websocket_connect("/ws/teleop?client_id=operator-a") as ws:
            ws.send_text(
                json.dumps(
                    {
                        "type": "velocity",
                        "vx_mps": 0.2,
                        "vy_mps": 0.0,
                        "yaw_rps": 0.0,
                        "deadman": True,
                        "request_id": "busy-1",
                    }
                )
            )
            response = json.loads(ws.receive_text())

            assert response == {
                "type": "control_rejected",
                "error": "control_in_use",
                "message": "Another controller currently owns robot motion.",
                "request_id": "busy-1",
            }
            assert "authority_busy" not in json.dumps(response)
    finally:
        gateway.stop()


def test_websocket_rejects_removed_heartbeat_stop_and_resume_protocol(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    commands = RecordingCommands()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.commands": commands})

    try:
        client = TestClient(gateway._app)
        with client.websocket_connect("/ws/teleop?client_id=operator-a") as ws:
            for message_type in ("heartbeat", "stop", "resume_control"):
                ws.send_text(json.dumps({"type": message_type, "request_id": message_type}))
                response = json.loads(ws.receive_text())
                assert response["type"] == "control_rejected"
                assert response["error"] == "unsupported_message"
            assert commands.calls == []
    finally:
        gateway.stop()
