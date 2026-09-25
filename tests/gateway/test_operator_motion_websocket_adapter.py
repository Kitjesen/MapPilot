from __future__ import annotations

import asyncio
import json
import threading
import time
from contextlib import suppress
from types import SimpleNamespace

from fastapi.testclient import TestClient

from gateway.gateway_module import GatewayModule
from gateway.routes import realtime
from gateway.services import teleop
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



def _input_window(ws) -> str:
    ws.send_json({"type": "input_request", "request_id": "input-test"})
    response = ws.receive_json()
    assert response["type"] == "input_ack"
    return response["input_window"]


def test_websocket_expired_or_reused_input_cannot_reach_native_motion(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    clock = {"offset": 0.0}
    fake_time = SimpleNamespace(
        monotonic=lambda: time.monotonic() + clock["offset"],
        monotonic_ns=time.monotonic_ns,
    )
    monkeypatch.setattr(realtime, "time", fake_time)
    commands = RecordingCommands()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.commands": commands})
    try:
        with TestClient(gateway._app).websocket_connect("/ws/teleop") as ws:
            window = _input_window(ws)
            clock["offset"] += 1.0
            ws.send_json({"type": "velocity", "deadman": True, "vx_mps": 0.5,
                          "input_window": window, "request_id": "delayed"})
            assert ws.receive_json()["error"] == "input_expired"
            assert not commands.calls
            clock["offset"] = 0.0
            window = _input_window(ws)
            sample = {"type": "velocity", "deadman": True, "vx_mps": 0.5,
                      "input_window": window, "request_id": "fresh"}
            ws.send_json(sample)
            admitted = ws.receive_json()
            assert admitted["type"] == "ingress_ack"
            assert admitted["input_window"] != window
            assert commands.called.wait(1.0)
            ws.send_json(sample)
            rejected = ws.receive_json()
            assert rejected.get("error") == "input_expired", rejected
            assert len([call for call in commands.calls if call[0] == "sample"]) == 1
            assert commands.hold_called.is_set()
    finally:
        gateway.stop()


def test_slow_claim_cannot_refresh_an_expired_browser_input(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    clock = {"offset": 0.0}
    fake_time = SimpleNamespace(
        monotonic=lambda: time.monotonic() + clock["offset"],
        monotonic_ns=time.monotonic_ns,
    )
    monkeypatch.setattr(realtime, "time", fake_time)
    monkeypatch.setattr(teleop, "time", fake_time)

    class SlowClaim(RecordingCommands):
        def claim(self, *args, **kwargs):
            receipt = super().claim(*args, **kwargs)
            clock["offset"] += 1.0
            return receipt

    commands = SlowClaim()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.commands": commands})
    try:
        with TestClient(gateway._app).websocket_connect("/ws/teleop") as ws:
            window = _input_window(ws)
            ws.send_json({"type": "velocity", "deadman": True, "vx_mps": 0.5,
                          "input_window": window, "request_id": "slow-claim"})
            assert ws.receive_json()["error"] == "input_expired"
            assert [call[0] for call in commands.calls] == ["claim"]
            ws.send_json({"type": "velocity", "deadman": False, "request_id": "release"})
            assert ws.receive_json()["action"] == "hold"
    finally:
        gateway.stop()


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
                        "input_window": _input_window(ws),
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
            assert commands.calls[1][4:8] == (0.4, -0.2, 0.3, True)
            assert 0 < commands.calls[1][8] <= 350
            assert commands.calls[1][9] == "ws-velocity-1"

        assert commands.hold_called.wait(3.0)
        assert commands.release_called.wait(3.0)
        assert commands.calls[-2][0] == "hold"
        assert commands.calls[-1][0] == "release"
        assert [call[3] for call in commands.calls] == [1, 2, 3, 4]
        assert media_lifecycle.connects == 1
        assert media_lifecycle.disconnects == 1
    finally:
        gateway.stop()


def test_websocket_disconnect_waits_for_zero_without_blocking_other_requests(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    hold_entered = threading.Event()
    allow_hold = threading.Event()
    disconnected = threading.Event()
    probe_done = threading.Event()
    probe_results = []

    class WaitingCommands(RecordingCommands):
        def hold(self, *args, **kwargs):
            if kwargs.get("reason") == "disconnect":
                hold_entered.set()
                assert allow_hold.wait(5.0)
            return super().hold(*args, **kwargs)

    class DisconnectLifecycle(RecordingTeleopLifecycle):
        def on_client_disconnect(self):
            super().on_client_disconnect()
            disconnected.set()

    commands = WaitingCommands()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.commands": commands})
    gateway._camera_module = DisconnectLifecycle()

    def request_probe(client):
        try:
            probe_results.append(client.get("/health"))
        finally:
            probe_done.set()

    close_thread = None
    probe_thread = None
    try:
        with TestClient(gateway._app) as client:
            with client.websocket_connect("/ws/teleop") as ws:
                ws.send_json({
                    "type": "velocity", "deadman": True, "vx_mps": 0.2,
                    "input_window": _input_window(ws), "request_id": "before-disconnect",
                })
                assert ws.receive_json()["type"] == "ingress_ack"
                assert commands.called.wait(1.0)
                try:
                    close_thread = threading.Thread(target=ws.close)
                    close_thread.start()
                    assert hold_entered.wait(1.0)
                    assert not commands.release_called.is_set()
                    assert not disconnected.is_set()
                    assert gateway._web_teleop_owner is not None

                    probe_thread = threading.Thread(target=request_probe, args=(client,))
                    probe_thread.start()
                    assert probe_done.wait(1.0), "disconnect hold blocked the Gateway event loop"
                    assert probe_results[0].json()["status"] == "ok"
                    assert not commands.release_called.is_set()
                finally:
                    allow_hold.set()
                    if close_thread is not None:
                        close_thread.join(timeout=2.0)
                    if probe_thread is not None:
                        probe_thread.join(timeout=2.0)
                assert disconnected.wait(1.0)
                assert commands.release_called.is_set()
                assert [call[0] for call in commands.calls] == ["claim", "sample", "hold", "release"]
                assert gateway._web_teleop_owner is None

            with client.websocket_connect("/ws/teleop") as reconnected:
                assert _input_window(reconnected)
    finally:
        allow_hold.set()
        gateway.stop()


def test_silent_broken_socket_releases_slot_without_disconnect_event(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setattr(realtime, "TELEOP_RECEIVE_TIMEOUT_S", 0.03)
    gateway = GatewayModule()
    gateway.setup()

    async def scenario():
        incoming = asyncio.Queue()
        await incoming.put({"type": "websocket.connect"})
        sent = []

        async def send(message):
            sent.append(message)

        websocket = realtime.StarletteWebSocket(
            {"type": "websocket", "path": "/ws/teleop", "query_string": b"", "headers": []},
            receive=incoming.get, send=send,
        )
        endpoint = next(route.endpoint for route in gateway._app.routes if route.path == "/ws/teleop")
        await asyncio.wait_for(endpoint(websocket), 1.0)
        assert gateway._web_teleop_owner is None
        assert any(message["type"] == "websocket.close" for message in sent)
        replacement = teleop.NativeTeleopSession(gateway, "replacement")
        assert replacement.open().accepted
        replacement.disconnect(request_id="cleanup")

    try:
        asyncio.run(scenario())
    finally:
        gateway.stop()


def test_stalled_websocket_ack_releases_control_after_send_timeout(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setattr(realtime, "REALTIME_SEND_TIMEOUT_S", 0.02)
    commands = RecordingCommands()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.commands": commands})
    lifecycle = RecordingTeleopLifecycle()
    gateway._camera_module = lifecycle

    async def scenario():
        incoming = asyncio.Queue()
        stalled = asyncio.Event()
        closed = []
        await incoming.put({"type": "websocket.connect"})
        await incoming.put({"type": "websocket.receive", "text": json.dumps({"type": "input_request"})})

        async def send(message):
            if message["type"] == "websocket.close":
                closed.append(message)
            if message["type"] != "websocket.send":
                return
            payload = json.loads(message["text"])
            if payload["type"] == "input_ack":
                await incoming.put({"type": "websocket.receive", "text": json.dumps({
                    "type": "velocity", "deadman": True, "vx_mps": 0.2,
                    "input_window": payload["input_window"], "request_id": "move-before-stalled-ack",
                })})
            elif payload["type"] == "ingress_ack":
                stalled.set()
                await asyncio.Event().wait()

        websocket = realtime.StarletteWebSocket(
            {"type": "websocket", "path": "/ws/teleop", "query_string": b"", "headers": []},
            receive=incoming.get,
            send=send,
        )
        endpoint = next(route.endpoint for route in gateway._app.routes if route.path == "/ws/teleop")
        task = asyncio.create_task(endpoint(websocket))
        try:
            await asyncio.wait_for(stalled.wait(), 1.0)
            assert await asyncio.to_thread(commands.called.wait, 1.0)
            await incoming.put({"type": "websocket.disconnect", "code": 1006})
            done, _ = await asyncio.wait({task}, timeout=0.5)
            assert task in done, "stalled ACK prevented disconnect hold and controller-slot release"
            await task
            assert [call[0] for call in commands.calls] == ["claim", "sample", "hold", "release"]
            assert gateway._web_teleop_owner is None
            assert lifecycle.disconnects == 1
            assert closed
        finally:
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task

    try:
        asyncio.run(scenario())
    finally:
        gateway.stop()


def test_paused_sse_observer_does_not_block_or_release_teleop(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    commands = RecordingCommands()
    gateway = GatewayModule()
    gateway.setup()
    gateway.on_system_modules({"nav.commands": commands})
    session = teleop.NativeTeleopSession(gateway, "web:active-operator")

    async def scenario():
        endpoint = next(route.endpoint for route in gateway._app.routes if route.path == "/api/v1/events")
        response = await endpoint()
        stream = response.body_iterator
        try:
            await anext(stream)
            queue = gateway._sse_queues[-1]
            # A slow network sender pauses here, after obtaining its current SSE chunk.
            for index in range(gateway._sse_queue_maxsize + 3):
                gateway.push_event({"type": "tick", "seq": index})
            await asyncio.sleep(0)
            assert queue.qsize() == gateway._sse_queue_maxsize

            assert session.open().accepted
            moved = await asyncio.to_thread(session.move, 0.2, 0.0, 0.0, request_id="while-observer-paused")
            assert moved.accepted
            assert await asyncio.to_thread(commands.called.wait, 1.0)
            await stream.aclose()
            assert queue not in gateway._sse_queues
            assert gateway._web_teleop_owner == "web:active-operator"
            assert [call[0] for call in commands.calls] == ["claim", "sample"]
        finally:
            await stream.aclose()
            await asyncio.to_thread(session.disconnect, request_id="operator-test-finished")

    try:
        asyncio.run(scenario())
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
                        "input_window": _input_window(ws),
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
                        "input_window": _input_window(ws),
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
                        "input_window": _input_window(ws),
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
                        "input_window": _input_window(ws),
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
                        "input_window": _input_window(ws),
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
                        "input_window": _input_window(ws),
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
                        "input_window": _input_window(ws),
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
            first.send_json({
                "type": "velocity", "vx_mps": 0.2, "deadman": True,
                "input_window": _input_window(first), "request_id": "first-already-controls",
            })
            assert first.receive_json()["type"] == "ingress_ack"
            assert commands.called.wait(1.0)
            commands.called.clear()
            with client.websocket_connect("/ws/teleop?client_id=operator-b") as second:
                rejected = json.loads(second.receive_text())
                assert rejected == {
                    "type": "control_rejected",
                    "error": "control_in_use",
                    "message": "Another operator is connected.",
                }

            assert [call[0] for call in commands.calls] == ["claim", "sample"]
            first.send_text(
                json.dumps(
                    {
                        "type": "velocity",
                        "vx_mps": 0.2,
                        "vy_mps": 0.0,
                        "yaw_rps": 0.0,
                        "deadman": True,
                        "input_window": _input_window(first),
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
                        "input_window": _input_window(ws),
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
