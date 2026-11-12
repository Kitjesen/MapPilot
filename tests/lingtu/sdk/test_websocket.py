"""Optional WebSocket clients against real Gateway routes on a local ASGI server."""

from __future__ import annotations

import asyncio
import importlib
import io
import json
import socket
import threading
import time
from contextlib import contextmanager
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

import pytest
import uvicorn
from fastapi import FastAPI

from gateway.media_access import MediaAccessMiddleware
from gateway.routes.camera import register_camera_routes
from gateway.routes.realtime import register_realtime_routes
from lingtu.sdk import AsyncLingTuClient, Feed, LingTuClient, sockets
from runtime.msgs.nav import OperatorMotionAction, OperatorMotionReceipt


class Viewer:
    def __init__(self):
        self.subscribers = set()
        self.payload = b"\x00\xffcloud\x00"

    def cloud_subscribe(self):
        queue = asyncio.Queue()
        self.subscribers.add(queue)
        return queue, self.payload

    scan_subscribe = cloud_subscribe

    def cloud_unsubscribe(self, queue):
        self.subscribers.remove(queue)

    scan_unsubscribe = cloud_unsubscribe


class Gateway:
    def __init__(self):
        self._state_lock = threading.Lock()
        self._web_teleop_owner_lock = threading.Lock()
        self._web_teleop_owner = None
        self._navigation_state = {}
        self._teleop_native_publisher = None
        self._camera_module = None
        self._cloud_viewer = Viewer()
        self._jpeg_lock = threading.Lock()
        self._latest_jpeg = b"\xff\xd8camera\x00\xff\xd9"
        self._latest_jpeg_seq = 1
        self._all_modules = {}
        self.claims = []
        self.samples = []
        self.releases = []
        self.connections = []
        self.reject_claim = False
        self.reject_release = False
        self.stale_ack = False
        self.claim_delay = 0.0
        self.claim_entered = threading.Event()

    def _teleop_client_connected(self):
        return 1

    def _teleop_client_disconnected(self):
        return 0

    def _teleop_claim(self, **values):
        self.claims.append(values)
        self.claim_entered.set()
        time.sleep(self.claim_delay)
        return self._receipt(OperatorMotionAction.CLAIM, values, accepted=not self.reject_claim)

    def _teleop_on_velocity(self, vx, vy, yaw, **values):
        self.samples.append((vx, vy, yaw, values))
        return True

    def _teleop_release(self, **values):
        self.releases.append(values)
        action = OperatorMotionAction.RELEASE if "release_sequence" in values else OperatorMotionAction.HOLD
        return self._receipt(action, values, accepted=not self.reject_release)

    @staticmethod
    def _receipt(action, values, *, accepted=True):
        sequence = values.get("release_sequence", values["sequence"])
        return OperatorMotionReceipt(
            accepted=accepted,
            action=action,
            request_id=values["request_id"],
            source_id=values["source_id"],
            source_epoch=values["source_epoch"],
            source_sequence=sequence,
            accepted_sequence=sequence if accepted else 0,
            final_output_sequence=sequence if accepted and action != OperatorMotionAction.CLAIM else 0,
            endpoint_timestamp_s=time.time(),
            reason="accepted" if accepted else "authority_busy",
        )


def wait_for(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("Local Gateway did not finish the expected lifecycle transition")


@contextmanager
def _gateway_server():
    gateway = Gateway()
    app = FastAPI()
    register_realtime_routes(app, gateway)
    register_camera_routes(app, gateway)
    app.add_middleware(MediaAccessMiddleware, required=True, viewer_token="viewer-secret")

    async def audited(scope, receive, send):
        if scope["type"] in {"websocket", "http"}:
            gateway.connections.append((scope["path"], dict(scope["headers"])))

        async def observed_send(message):
            if gateway.stale_ack and message["type"] == "websocket.send" and message.get("text"):
                payload = json.loads(message["text"])
                if payload.get("type") == "input_ack":
                    stale = {**payload, "request_id": "another-request", "input_window": "stale-window"}
                    await send({"type": "websocket.send", "text": json.dumps(stale)})
            await send(message)

        await app(scope, receive, observed_send)

    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    port = listener.getsockname()[1]
    config = uvicorn.Config(
        audited, log_level="critical", lifespan="off", ws="websockets-sansio", timeout_graceful_shutdown=0.1
    )
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, kwargs={"sockets": [listener]}, daemon=True)
    thread.start()
    try:
        wait_for(lambda: server.started)
        yield gateway, LingTuClient(port=port, api_key="operator-secret", timeout=1), port
    finally:
        server.should_exit = True
        thread.join(timeout=5)
        listener.close()
        assert not thread.is_alive()


@pytest.fixture
def live_gateway():
    with _gateway_server() as gateway:
        yield gateway


def test_feed_construction_is_lazy_and_optional_import_has_actionable_error(monkeypatch):
    def missing(name):
        raise ModuleNotFoundError(name)

    monkeypatch.setattr(importlib, "import_module", missing)
    robot = LingTuClient()
    feed = robot.watch_cloud()
    assert isinstance(feed, Feed)
    with pytest.raises(ImportError, match=r"lingtu-sdk\[websocket\]"):
        feed.open()


@pytest.mark.parametrize("channel", ["cloud", "scan"])
def test_binary_feeds_and_early_close(live_gateway, channel):
    gateway, robot, _ = live_gateway
    assert gateway.connections == []
    with getattr(robot, f"watch_{channel}")() as feed:
        assert next(feed) == gateway._cloud_viewer.payload
    assert feed._socket is None
    wait_for(lambda: not gateway._cloud_viewer.subscribers)
    assert gateway.connections[0][1][b"x-api-key"] == b"operator-secret"


def test_camera_requires_independent_viewer_token(live_gateway):
    from websockets.exceptions import InvalidStatus

    gateway, robot, _ = live_gateway
    with pytest.raises(InvalidStatus):
        with robot.watch_camera():
            pytest.fail("camera opened without media credential")
    with robot.watch_camera(viewer_token="viewer-secret") as feed:
        assert next(feed) == gateway._latest_jpeg
    assert robot.camera_snapshot(viewer_token="viewer-secret") == gateway._latest_jpeg
    assert gateway.connections[-1][1][b"authorization"] == b"Bearer viewer-secret"


def test_camera_disconnect_releases_idle_viewer(live_gateway):
    gateway, robot, _ = live_gateway
    tracker = SimpleNamespace(clients=0)

    def connected():
        tracker.clients += 1

    def disconnected():
        tracker.clients -= 1

    tracker.on_camera_client_connect = connected
    tracker.on_camera_client_disconnect = disconnected
    gateway._camera_module = tracker
    with robot.watch_camera(viewer_token="viewer-secret") as feed:
        assert next(feed) == gateway._latest_jpeg
        assert tracker.clients == 1
    wait_for(lambda: tracker.clients == 0)


def test_feed_timeout_and_large_frame(live_gateway):
    gateway, robot, _ = live_gateway
    with robot.watch_camera(viewer_token="viewer-secret", timeout=0.05) as camera:
        assert next(camera) == gateway._latest_jpeg
        with pytest.raises(TimeoutError):
            next(camera)
        assert camera._socket is None
    gateway._cloud_viewer.payload = b"x" * (10 * 1024 * 1024 + 1)
    with robot.watch_cloud() as cloud:
        assert next(cloud) == gateway._cloud_viewer.payload


def test_feed_clean_end_stops_iteration():
    from websockets.exceptions import ConnectionClosedOK

    feed = LingTuClient().watch_cloud()
    connection = Mock()
    connection.recv.side_effect = ConnectionClosedOK(None, None)
    feed._socket = connection
    assert list(feed) == []
    assert feed._socket is None
    connection.close.assert_called_once_with()


def test_teleop_correlates_windows_preserves_receipts_and_releases(live_gateway):
    gateway, robot, _ = live_gateway
    gateway.stale_ack = True
    session = robot.teleop(client_id="operator +1")
    assert gateway._web_teleop_owner is None
    with session:
        receipt = session.move(0.1, 0.2, 0.3, manual_mode=True, request_id="move-1")
        assert receipt == {
            "type": "ingress_ack",
            "action": "queued",
            "ingress_accepted": True,
            "stage": "gateway_queue_accepted",
            "request_id": "move-1",
            "replaceable": True,
            "final_cmd_vel_confirmed": False,
            "motor_confirmed": False,
            "input_window": "2",
        }
        held = session.hold(request_id="hold-1")
        assert held["type"] == "control_ack" and held["request_id"] == "hold-1"
        assert held["final_cmd_vel_confirmed"] is True and held["motor_confirmed"] is False
        session.move(0.2, 0.0, 0.0, request_id="move-2")
    wait_for(lambda: gateway._web_teleop_owner is None)
    assert len(gateway.samples) == 2
    assert gateway.samples[0][:3] == (0.1, 0.2, 0.3)
    assert gateway.samples[0][3]["manual_mode"] is True
    assert gateway.samples[0][3]["request_id"] == "move-1"
    assert gateway.releases[-1]["reason"] == "disconnect"


def test_teleop_rejections_and_connection_ownership(live_gateway):
    gateway, robot, _ = live_gateway
    with robot.teleop() as owner:
        with robot.teleop() as other:
            rejected = other.move(0, 0, 0)
            assert rejected["type"] == "control_rejected" and rejected["error"] == "control_in_use"
        assert gateway._web_teleop_owner is not None
        gateway.reject_claim = True
        rejected = owner.move(0.1, 0, 0, request_id="rejected-move")
        assert rejected["type"] == "control_rejected" and rejected["request_id"] == "rejected-move"
        assert gateway.samples == []
    wait_for(lambda: gateway._web_teleop_owner is None)


def test_teleop_timeout_closes_and_releases_owner(live_gateway):
    gateway, robot, _ = live_gateway
    gateway.claim_delay = 0.15
    with robot.teleop(timeout=0.05) as session:
        with pytest.raises(TimeoutError):
            session.move(0.1, 0, 0, request_id="timed-out")
        assert session._socket is None
    wait_for(lambda: gateway._web_teleop_owner is None)
    assert gateway.releases[-1]["reason"] == "disconnect"


@pytest.mark.asyncio
async def test_async_feeds_close_on_cancellation(live_gateway):
    gateway, _, port = live_gateway
    robot = AsyncLingTuClient(port=port, api_key="operator-secret", timeout=1)
    async with robot.watch_scan() as feed:
        assert await anext(feed) == gateway._cloud_viewer.payload
    await asyncio.to_thread(wait_for, lambda: not gateway._cloud_viewer.subscribers)
    async with robot.watch_camera(viewer_token="viewer-secret") as camera:
        assert await anext(camera) == gateway._latest_jpeg
        task = asyncio.create_task(anext(camera))
        await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert camera._socket is None


@pytest.mark.asyncio
async def test_async_teleop_move_hold_and_cancel(live_gateway):
    gateway, _, port = live_gateway
    robot = AsyncLingTuClient(port=port, timeout=1)
    async with robot.teleop() as session:
        receipt = await session.move(0.1, 0, 0, request_id="async-move")
        assert receipt["type"] == "ingress_ack" and receipt["request_id"] == "async-move"
        held = await session.hold(request_id="async-hold")
        assert held["type"] == "control_ack" and held["final_cmd_vel_confirmed"] is True
        gateway.claim_delay = 0.15
        gateway.claim_entered.clear()
        task = asyncio.create_task(session.move(0.2, 0, 0))
        await asyncio.to_thread(gateway.claim_entered.wait, 2)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert session._socket is None
    await asyncio.to_thread(wait_for, lambda: gateway._web_teleop_owner is None)
    assert gateway.releases[-1]["reason"] == "disconnect"


@pytest.mark.parametrize("method", ["camera_status", "media_status", "camera_diagnostics"])
def test_media_status_uses_bearer_and_inherited_api_key(monkeypatch, method):
    transport = Mock(return_value=io.BytesIO(b'{"available":false}'))
    monkeypatch.setattr("urllib.request.urlopen", transport)
    robot = LingTuClient(api_key="operator-secret")
    assert getattr(robot.observations, method)(viewer_token="viewer-secret")["available"] is False
    request = transport.call_args.args[0]
    assert request.get_header("Authorization") == "Bearer viewer-secret"
    assert request.get_header("X-api-key") == "operator-secret"


@pytest.mark.asyncio
async def test_async_media_viewer_token_reaches_http_transport(monkeypatch):
    transport = Mock(side_effect=lambda *args, **kwargs: io.BytesIO(b'{"available":false}'))
    monkeypatch.setattr("urllib.request.urlopen", transport)
    robot = AsyncLingTuClient()
    await robot.observations.camera_status(viewer_token="viewer-secret")
    await robot.observations.media_status(viewer_token="viewer-secret")
    await robot.observations.camera_diagnostics(viewer_token="viewer-secret")
    for call in transport.call_args_list:
        assert call.args[0].get_header("Authorization") == "Bearer viewer-secret"


@pytest.mark.asyncio
async def test_async_clean_feed_end():
    from websockets.exceptions import ConnectionClosedOK

    feed = AsyncLingTuClient().watch_cloud()
    connection = AsyncMock()
    connection.recv.side_effect = ConnectionClosedOK(None, None)
    feed._socket = connection
    assert [message async for message in feed] == []
    assert feed._socket is None
    connection.close.assert_awaited_once_with()
