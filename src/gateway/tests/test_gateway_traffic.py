from __future__ import annotations

import asyncio
import json
import threading
import time

import pytest


pytest.importorskip("fastapi")
from runtime.tests.numpy_guard import numpy_safe_skip_mark


def test_sse_slow_client_keeps_latest_events_and_drops_oldest():
    from gateway.gateway_module import GatewayModule
    from gateway.services.traffic import DROP_OLDEST_POLICY, SSE_EVENT_SCHEMA_VERSION

    gateway = GatewayModule()
    queue = gateway._sse_subscribe()

    for seq in range(gateway._sse_queue_maxsize + 3):
        gateway.push_event({"type": "tick", "seq": seq})

    assert queue.qsize() == gateway._sse_queue_maxsize
    retained = queue.get_nowait()
    assert retained["seq"] == 3
    assert retained["event_id"] == 4
    assert retained["schema_version"] == SSE_EVENT_SCHEMA_VERSION
    assert retained["ts"] > 0

    stats = gateway._traffic_stats_snapshot()
    assert stats["sse"]["clients"] == 1
    assert stats["sse"]["queue_maxsize"] == gateway._sse_queue_maxsize
    assert stats["sse"]["latest_event_id"] == gateway._sse_queue_maxsize + 3
    assert stats["sse"]["published_events"] == gateway._sse_queue_maxsize + 3
    assert stats["sse"]["dropped_events"] == 3
    assert stats["sse"]["drop_policy"] == DROP_OLDEST_POLICY


@numpy_safe_skip_mark()
def test_sse_raster_events_are_client_bound_and_throttled():
    import numpy as np

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._sse_raster_min_interval_s = 60.0
    grid = np.full((2, 3), 42, dtype=np.uint8)

    gateway._costmap_throttle = 4
    gateway._on_costmap({"grid": grid, "resolution": 0.1, "origin": [1.0, 2.0]})
    assert gateway._traffic_stats_snapshot()["sse"]["published_events"] == 0

    queue = gateway._sse_subscribe()
    gateway._costmap_throttle = 4
    gateway._on_costmap({"grid": grid, "resolution": 0.1, "origin": [1.0, 2.0]})

    event = queue.get_nowait()
    assert event["type"] == "costmap"
    assert event["rows"] == 2
    assert event["cols"] == 3
    assert event["grid_b64"]

    gateway._costmap_throttle = 4
    gateway._on_costmap({"grid": grid, "resolution": 0.1, "origin": [1.0, 2.0]})
    assert queue.empty()

    stats = gateway._traffic_stats_snapshot()
    assert stats["sse"]["suppressed_events"]["costmap"] == 1
    assert stats["sse"]["raster_min_interval_s"] == 60.0


@numpy_safe_skip_mark()
def test_slope_grid_defaults_to_metadata_and_can_enable_inline_payload():
    import numpy as np

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._sse_raster_min_interval_s = 0.0
    queue = gateway._sse_subscribe()

    gateway._on_slope_grid({"grid": np.ones((2, 2)), "resolution": 0.2, "origin": [0.0, 0.0]})
    event = queue.get_nowait()

    assert event["type"] == "slope_grid"
    assert event["payload"] == "omitted"
    assert event["available"] is True
    assert event["rows"] == 2
    assert event["cols"] == 2
    assert "grid_b64" not in event

    gateway._sse_slope_payload_enabled = True
    gateway._on_slope_grid({"grid": np.ones((2, 2)), "resolution": 0.2, "origin": [0.0, 0.0]})
    event = queue.get_nowait()

    assert event["type"] == "slope_grid"
    assert event["payload"] == "inline"
    assert event["grid_b64"]


def test_gateway_visual_raster_inputs_keep_latest_only():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    assert gateway.costmap._policy == "latest"
    assert gateway.slope_grid._policy == "latest"


def test_cloud_slow_client_keeps_latest_frames_and_drops_oldest():
    from gateway.gateway_module import GatewayModule
    from gateway.services.traffic import DROP_OLDEST_POLICY

    gateway = GatewayModule()
    queue, latest = gateway._cloud_subscribe()

    assert latest is None

    for seq in range(gateway._cloud_queue_maxsize + 2):
        gateway._publish_cloud_frame(bytes([seq]))

    assert queue.qsize() == gateway._cloud_queue_maxsize
    assert queue.get_nowait() == bytes([2])

    stats = gateway._traffic_stats_snapshot()
    assert stats["cloud"]["clients"] == 1
    assert stats["cloud"]["queue_maxsize"] == gateway._cloud_queue_maxsize
    assert stats["cloud"]["published_frames"] == gateway._cloud_queue_maxsize + 2
    assert stats["cloud"]["dropped_frames"] == 2
    assert stats["cloud"]["drop_policy"] == DROP_OLDEST_POLICY
    assert stats["cloud"]["latest_seq"] == gateway._cloud_queue_maxsize + 2


def test_gateway_health_and_bootstrap_expose_traffic_policy():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_bootstrap, build_app_traffic

    gateway = GatewayModule()
    gateway._sse_subscribe()
    gateway._cloud_subscribe()

    health = gateway.health()
    bootstrap = build_app_bootstrap(gateway)
    traffic = build_app_traffic(gateway)

    assert health["gateway"]["traffic"]["sse"]["clients"] == 1
    assert health["gateway"]["traffic"]["cloud"]["clients"] == 1
    assert bootstrap["traffic"]["sse"]["queue_maxsize"] == gateway._sse_queue_maxsize
    assert bootstrap["traffic"]["cloud"]["queue_maxsize"] == gateway._cloud_queue_maxsize
    assert bootstrap["traffic"]["recommended_client_rates_hz"]["state"] == 1.0
    assert bootstrap["traffic"]["client_policy"]["large_event_policy"]["slope_grid_payload"] == "metadata_sse"
    assert traffic["schema_version"] == 1
    assert traffic["status"] == "ok"
    assert traffic["sse"]["clients"] == 1
    assert traffic["cloud"]["clients"] == 1
    assert traffic["client_policy"]["usage"] == "low_frequency_monitoring"
    assert traffic["client_policy"]["traffic_endpoint"] == "/api/v1/app/traffic"


def test_app_traffic_reports_backpressure_warnings():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_traffic

    gateway = GatewayModule()
    gateway._sse_queue_maxsize = 2
    gateway._cloud_queue_maxsize = 2
    gateway._sse_subscribe()
    gateway._cloud_subscribe()

    for seq in range(5):
        gateway.push_event({"type": "tick", "seq": seq})
        gateway._publish_cloud_frame(bytes([seq]))

    traffic = build_app_traffic(gateway)

    assert traffic["status"] == "degraded"
    assert traffic["sse"]["dropped_events"] == 3
    assert traffic["cloud"]["dropped_frames"] == 3
    assert "sse_events_dropped" in traffic["warnings"]
    assert "sse_queue_pressure" in traffic["warnings"]
    assert "cloud_frames_dropped_latest_only" in traffic["warnings"]
    assert "cloud_queue_pressure" in traffic["warnings"]


def test_sse_message_format_keeps_eventsource_onmessage_contract():
    from gateway.services.traffic import (
        SSE_EVENT_SCHEMA_VERSION,
        format_sse_message,
        normalize_sse_event,
    )

    event = normalize_sse_event({"type": "mission_status", "data": {"state": "IDLE"}}, event_id=7, now=123.0)
    text = format_sse_message(event, retry_ms=3000)

    assert text.startswith("retry: 3000\nid: 7\ndata: ")
    assert "\nevent:" not in text
    assert '"schema_version":1' in text
    assert '"event_id":7' in text
    assert '"ts":123.0' in text
    assert event["schema_version"] == SSE_EVENT_SCHEMA_VERSION


def test_sse_subscribe_with_event_id_reserves_snapshot_before_push():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    queue, snapshot_id = gateway._sse_subscribe_with_event_id()

    gateway.push_event({"type": "later"})

    event = queue.get_nowait()
    assert snapshot_id == 1
    assert event["type"] == "later"
    assert event["event_id"] == 2
    assert gateway._traffic_stats_snapshot()["sse"]["latest_event_id"] == 2


def test_sse_subscriber_receives_threaded_publish_on_endpoint_loop():
    from gateway.gateway_module import GatewayModule

    async def _run():
        gateway = GatewayModule()
        queue = gateway._sse_subscribe()

        thread = threading.Thread(
            target=lambda: gateway.push_event({"type": "threaded", "data": 1})
        )
        thread.start()
        try:
            event = await asyncio.wait_for(queue.get(), timeout=1.0)
        finally:
            thread.join(timeout=1.0)
            gateway._sse_unsubscribe(queue)

        assert event["type"] == "threaded"
        assert event["event_id"] == 1
        assert gateway._traffic_stats_snapshot()["sse"]["clients"] == 0

    asyncio.run(_run())


def test_cloud_subscriber_receives_threaded_publish_on_endpoint_loop():
    from gateway.gateway_module import GatewayModule

    async def _run():
        gateway = GatewayModule()
        queue, latest = gateway._cloud_subscribe()
        assert latest is None

        thread = threading.Thread(
            target=lambda: gateway._publish_cloud_frame(b"PCL-threaded")
        )
        thread.start()
        try:
            frame = await asyncio.wait_for(queue.get(), timeout=1.0)
        finally:
            thread.join(timeout=1.0)
            gateway._cloud_unsubscribe(queue)

        assert frame == b"PCL-threaded"
        assert gateway._traffic_stats_snapshot()["cloud"]["clients"] == 0

    asyncio.run(_run())


def test_camera_websocket_is_camera_only_and_cleans_up():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    class TeleopTracker:
        def __init__(self):
            self.camera_clients = 0
            self.teleop_clients = 0

        def on_camera_client_connect(self):
            self.camera_clients += 1

        def on_camera_client_disconnect(self):
            self.camera_clients -= 1

        def on_client_connect(self):
            self.teleop_clients += 1

        def on_client_disconnect(self):
            self.teleop_clients -= 1

    gateway = GatewayModule()
    gateway.setup()
    tracker = TeleopTracker()
    gateway._teleop_module = tracker
    gateway.push_jpeg(b"\xff\xd8\xffcamera")

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/camera") as ws:
        assert ws.receive_bytes() == b"\xff\xd8\xffcamera"
        assert tracker.camera_clients == 1
        assert tracker.teleop_clients == 0
        assert gateway._teleop_clients == 0

    assert tracker.camera_clients == 0
    assert tracker.teleop_clients == 0


def test_cloud_websocket_sends_latest_and_cleans_up():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._publish_cloud_frame(b"PCL0")

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/cloud") as ws:
        assert ws.receive_bytes() == b"PCL0"
        assert gateway._traffic_stats_snapshot()["cloud"]["clients"] == 1
        gateway._publish_cloud_frame(b"PCL1")
        assert ws.receive_bytes() == b"PCL1"

    assert gateway._traffic_stats_snapshot()["cloud"]["clients"] == 0


def test_teleop_websocket_ignores_malformed_frames_without_motion():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    class TeleopTracker:
        def __init__(self):
            self.clients = 0

        def on_client_connect(self):
            self.clients += 1

        def on_client_disconnect(self):
            self.clients -= 1

        def force_release(self):
            pass

    gateway = GatewayModule()
    gateway.setup()
    tracker = TeleopTracker()
    gateway._teleop_module = tracker
    sent_cmds = []
    gateway.cmd_vel._add_callback(sent_cmds.append)

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/teleop") as ws:
        ws.send_bytes(b"\xff")
        ws.send_text("[1, 2, 3]")
        ws.send_text("not json")
        ws.send_text('{"type":"joy","lx":"bad","ly":0,"az":0}')
        ws.send_text('{"type":"unknown"}')
        assert gateway._teleop_clients == 1
        assert tracker.clients == 1
        assert sent_cmds == []

    assert gateway._teleop_clients == 0
    assert tracker.clients == 0
    assert sent_cmds == []


def test_teleop_websocket_rejects_joy_when_safety_stop_active():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    class TeleopTracker:
        def __init__(self):
            self.clients = 0
            self.joy_calls = 0

        def on_client_connect(self):
            self.clients += 1

        def on_client_disconnect(self):
            self.clients -= 1

        def force_release(self):
            pass

    gateway = GatewayModule()
    gateway.setup()
    tracker = TeleopTracker()
    gateway._teleop_module = tracker
    with gateway._state_lock:
        gateway._safety = {"level": 2}
    sent_cmds = []
    gateway.cmd_vel._add_callback(sent_cmds.append)

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/teleop") as ws:
        ws.send_text('{"type":"joy","lx":0.2,"ly":0,"az":0.1}')
        payload = json.loads(ws.receive_text())

        assert payload["type"] == "control_rejected"
        assert payload["error"] == "safety_stop"
        assert gateway._teleop_clients == 1
        assert tracker.clients == 1
        assert tracker.joy_calls == 0
        assert sent_cmds == []

    assert gateway._teleop_clients == 0
    assert tracker.clients == 0
    assert tracker.joy_calls == 0
    assert sent_cmds == []


def test_teleop_camera_frame_task_is_awaited_on_disconnect(monkeypatch):
    from fastapi.testclient import TestClient

    import gateway.routes.realtime as realtime
    from gateway.gateway_module import GatewayModule

    class FakeTask:
        def __init__(self):
            self.cancelled = False
            self.awaited = False

        def cancel(self):
            self.cancelled = True

        def __await__(self):
            async def _complete():
                self.awaited = True
                raise asyncio.CancelledError()

            return _complete().__await__()

    fake_task = FakeTask()

    def fake_create_task(coro):
        coro.close()
        return fake_task

    monkeypatch.setattr(realtime.asyncio, "create_task", fake_create_task)

    gateway = GatewayModule()
    gateway.setup()

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/teleop?camera=1"):
        assert gateway._teleop_clients == 1

    assert fake_task.cancelled is True
    assert fake_task.awaited is True
    assert gateway._teleop_clients == 0


def test_teleop_camera_frame_task_error_still_releases_client(monkeypatch):
    from fastapi.testclient import TestClient

    import gateway.routes.realtime as realtime
    from gateway.gateway_module import GatewayModule

    class FakeTask:
        def __init__(self):
            self.cancelled = False
            self.awaited = False

        def cancel(self):
            self.cancelled = True

        def __await__(self):
            async def _complete():
                self.awaited = True
                raise RuntimeError("camera stream failed during cleanup")

            return _complete().__await__()

    fake_task = FakeTask()

    def fake_create_task(coro):
        coro.close()
        return fake_task

    monkeypatch.setattr(realtime.asyncio, "create_task", fake_create_task)

    gateway = GatewayModule()
    gateway.setup()

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/teleop?camera=1"):
        assert gateway._teleop_clients == 1

    assert fake_task.cancelled is True
    assert fake_task.awaited is True
    assert gateway._teleop_clients == 0


def test_teleop_client_counter_helpers_are_thread_safe():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()

    def connect_and_disconnect_many():
        for _ in range(1000):
            gateway._teleop_client_connected()
            gateway._teleop_client_disconnected()

    threads = [
        threading.Thread(target=connect_and_disconnect_many)
        for _ in range(8)
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert gateway._teleop_client_count() == 0
    assert gateway._teleop_client_disconnected() == 0


def test_gateway_run_server_reports_failure_without_configured_app():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()

    assert gateway._run_server() is False


def test_gateway_run_server_reports_clean_uvicorn_shutdown(monkeypatch):
    import sys
    import types

    from gateway.gateway_module import GatewayModule

    class FakeConfig:
        def __init__(self, *args, **kwargs):
            pass

    class FakeServer:
        def __init__(self, config):
            self.should_exit = False
            self.force_exit = False

        def run(self):
            self.should_exit = True

    fake_uvicorn = types.ModuleType("uvicorn")
    fake_uvicorn.Config = FakeConfig
    fake_uvicorn.Server = FakeServer
    monkeypatch.setitem(sys.modules, "uvicorn", fake_uvicorn)

    gateway = GatewayModule()
    gateway.setup()

    assert gateway._run_server() is True
    assert gateway._server is None


def test_gateway_run_server_reports_unexpected_uvicorn_return(monkeypatch):
    import sys
    import types

    from gateway.gateway_module import GatewayModule

    class FakeConfig:
        def __init__(self, *args, **kwargs):
            pass

    class FakeServer:
        def __init__(self, config):
            self.should_exit = False
            self.force_exit = False

        def run(self):
            pass

    fake_uvicorn = types.ModuleType("uvicorn")
    fake_uvicorn.Config = FakeConfig
    fake_uvicorn.Server = FakeServer
    monkeypatch.setitem(sys.modules, "uvicorn", fake_uvicorn)

    gateway = GatewayModule()
    gateway.setup()

    assert gateway._run_server() is False
    assert gateway._server is None


def test_gateway_stop_signals_background_threads_without_server_start():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._defer_server = True
    gateway._drift_watchdog_enabled = True
    gateway._drift_watchdog_interval = 60.0
    gateway.setup()
    gateway.start()

    saved_thread = gateway._saved_map_loader_thread
    drift_thread = gateway._drift_watchdog_thread
    assert saved_thread is not None
    assert drift_thread is not None
    assert saved_thread.is_alive()
    assert drift_thread.is_alive()

    gateway.stop()

    assert gateway._saved_map_loader_thread is None
    assert gateway._drift_watchdog_thread is None
    assert not saved_thread.is_alive()
    assert not drift_thread.is_alive()


def test_gateway_start_runs_client_http_prewarm_in_background(monkeypatch):
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._drift_watchdog_enabled = False
    gateway.setup()

    server_started = threading.Event()
    prewarm_started = threading.Event()

    def fake_run_server(stop_event=None):
        server_started.set()
        (stop_event or gateway._stop_event).wait(1.0)
        return True

    def fake_prewarm(stop_event=None, *, timeout_s=15.0):
        assert timeout_s == 15.0
        prewarm_started.set()
        (stop_event or gateway._stop_event).wait(1.0)
        return False

    monkeypatch.setattr(gateway, "_run_server", fake_run_server)
    monkeypatch.setattr(gateway, "_prewarm_client_http_routes", fake_prewarm)

    started_at = time.perf_counter()
    gateway.start()
    elapsed_s = time.perf_counter() - started_at

    assert elapsed_s < 0.5
    assert server_started.wait(timeout=0.5)
    assert prewarm_started.wait(timeout=0.5)
    assert gateway._client_http_prewarm_thread is not None
    assert gateway._client_http_prewarm_thread.is_alive()

    gateway.stop()

    assert gateway._client_http_prewarm_thread is None


def test_gateway_deferred_mode_can_start_client_http_prewarm(monkeypatch):
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._defer_server = True
    gateway._drift_watchdog_enabled = False
    gateway.setup()

    prewarm_started = threading.Event()

    def fake_prewarm(stop_event=None, *, timeout_s=30.0):
        assert timeout_s == 30.0
        prewarm_started.set()
        (stop_event or gateway._stop_event).wait(1.0)
        return False

    monkeypatch.setattr(gateway, "_prewarm_client_http_routes", fake_prewarm)

    assert gateway._start_client_http_prewarm(timeout_s=30.0) is True
    assert prewarm_started.wait(timeout=0.5)
    assert gateway._client_http_prewarm_thread is not None
    assert gateway._client_http_prewarm_thread.is_alive()

    gateway.stop()

    assert gateway._client_http_prewarm_thread is None


def test_gateway_http_prewarm_does_not_skip_deferred_server(monkeypatch):
    import urllib.request

    from gateway.gateway_module import GatewayModule

    class FakeResponse:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def read(self):
            return b"{}"

    calls = []

    def fake_urlopen(request, timeout=None):
        calls.append((request.full_url, timeout))
        return FakeResponse()

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)

    gateway = GatewayModule()
    gateway._defer_server = True
    gateway.setup()

    assert gateway._prewarm_client_http_routes(threading.Event(), timeout_s=1.0) is True
    assert calls
    assert calls[0][0].endswith("/api/v1/app/capabilities")


def test_gateway_stop_retains_background_thread_when_join_times_out():
    from gateway.gateway_module import GatewayModule

    class StuckThread:
        name = "saved_map_loader"

        def __init__(self):
            self.join_timeouts = []

        def is_alive(self):
            return True

        def join(self, timeout=None):
            self.join_timeouts.append(timeout)

    gateway = GatewayModule()
    old_event = gateway._stop_event
    stuck = StuckThread()
    gateway._saved_map_loader_thread = stuck

    gateway.stop()

    assert old_event.is_set()
    assert stuck.join_timeouts == [2.0]
    assert gateway._saved_map_loader_thread is stuck


def test_gateway_start_replaces_stopped_event_without_duplicate_stuck_loader():
    from gateway.gateway_module import GatewayModule

    class StuckThread:
        name = "saved_map_loader"

        def is_alive(self):
            return True

        def join(self, timeout=None):
            pass

    gateway = GatewayModule()
    gateway._defer_server = True
    gateway._drift_watchdog_enabled = False
    gateway.setup()
    old_event = gateway._stop_event
    old_event.set()
    stuck = StuckThread()
    gateway._saved_map_loader_thread = stuck

    gateway.start()

    assert gateway._stop_event is not old_event
    assert old_event.is_set()
    assert gateway._saved_map_loader_thread is stuck

    gateway.stop()


def test_gateway_start_does_not_duplicate_background_threads():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._defer_server = True
    gateway._drift_watchdog_enabled = True
    gateway._drift_watchdog_interval = 60.0
    gateway.setup()
    gateway.start()

    saved_thread = gateway._saved_map_loader_thread
    drift_thread = gateway._drift_watchdog_thread

    gateway.start()

    assert gateway._saved_map_loader_thread is saved_thread
    assert gateway._drift_watchdog_thread is drift_thread

    gateway.stop()


def test_gateway_stop_signals_uvicorn_server():
    from gateway.gateway_module import GatewayModule

    class FakeServer:
        should_exit = False

    gateway = GatewayModule()
    fake_server = FakeServer()
    gateway._server = fake_server

    gateway.stop()

    assert fake_server.should_exit is True
