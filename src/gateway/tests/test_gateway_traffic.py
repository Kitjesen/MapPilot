from __future__ import annotations

import asyncio
import json
import threading
import time

import pytest

pytest.importorskip("fastapi")
from gateway.services.sse import subscribe, subscribe_with_event_id, unsubscribe
from runtime.msgs.nav import OperatorMotionAction, OperatorMotionReceipt
from runtime.tests.numpy_guard import numpy_safe_skip_mark


def operator_motion_receipt(
    action: OperatorMotionAction,
    source_id: str,
    source_epoch: int,
    sequence: int,
    request_id: str | None,
    *,
    final_output_sequence: int = 0,
    accepted: bool = True,
    reason: str = "accepted",
) -> OperatorMotionReceipt:
    return OperatorMotionReceipt(
        accepted=accepted,
        action=int(action),
        request_id=str(request_id or f"{source_id}:{action.name.lower()}:{sequence}"),
        source_id=source_id,
        source_epoch=source_epoch,
        source_sequence=sequence,
        accepted_sequence=sequence if accepted else 0,
        final_output_sequence=final_output_sequence if accepted else 0,
        endpoint_timestamp_s=time.time() or 1.0,
        reason=reason,
    )


def configure_map_viewer(
    gateway,
    *,
    map_voxel_size: float = 0.1,
    voxel_min_hits: int | None = None,
    map_viewer_stale_grace: int | None = None,
    cloud_viewer_min_interval_s: float | None = None,
    cloud_viewer_force_interval_s: float | None = None,
    cloud_viewer_min_point_delta: int | None = None,
    scan_viewer_min_interval_s: float | None = None,
) -> None:
    kwargs = {
        "map_voxel_size": map_voxel_size,
        "voxel_min_hits": voxel_min_hits,
        "map_viewer_stale_grace": map_viewer_stale_grace,
        "cloud_viewer_min_interval_s": cloud_viewer_min_interval_s,
        "cloud_viewer_force_interval_s": cloud_viewer_force_interval_s,
        "cloud_viewer_min_point_delta": cloud_viewer_min_point_delta,
        "scan_viewer_min_interval_s": scan_viewer_min_interval_s,
    }
    gateway._cloud_viewer.configure(**{k: v for k, v in kwargs.items() if v is not None})


def test_sse_slow_client_keeps_latest_events_and_drops_oldest():
    from gateway.gateway_module import GatewayModule
    from gateway.services.traffic import DROP_OLDEST_POLICY, SSE_EVENT_SCHEMA_VERSION

    gateway = GatewayModule()
    queue = subscribe(gateway)

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


def test_cloud_slow_client_keeps_latest_frames_and_drops_oldest():
    from gateway.gateway_module import GatewayModule
    from gateway.services.traffic import DROP_OLDEST_POLICY

    gateway = GatewayModule()
    queue, latest = gateway._cloud_viewer.cloud_subscribe()

    assert latest is None

    cloud_queue_maxsize = gateway._cloud_viewer.cloud_queue_maxsize()
    for seq in range(cloud_queue_maxsize + 2):
        gateway._cloud_viewer.publish_cloud_frame(bytes([seq]))

    assert queue.qsize() == cloud_queue_maxsize
    assert queue.get_nowait() == bytes([2])

    stats = gateway._traffic_stats_snapshot()
    assert stats["cloud"]["clients"] == 1
    assert stats["cloud"]["queue_maxsize"] == cloud_queue_maxsize
    assert stats["cloud"]["published_frames"] == cloud_queue_maxsize + 2
    assert stats["cloud"]["dropped_frames"] == 2
    assert stats["cloud"]["drop_policy"] == DROP_OLDEST_POLICY
    assert stats["cloud"]["latest_seq"] == cloud_queue_maxsize + 2


def test_cloud_stats_include_latest_frame_metadata():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    seq = gateway._cloud_viewer.publish_cloud_frame(
        b"PCLD",
        metadata={
            "point_count": 12,
            "source": "slam_map_cloud",
            "z_min": -6.5,
            "z_max": -2.8,
        },
    )

    latest = gateway._traffic_stats_snapshot()["cloud"]["latest_frame"]
    assert latest["seq"] == seq
    assert latest["bytes"] == 4
    assert latest["point_count"] == 12
    assert latest["source"] == "slam_map_cloud"
    assert latest["z_min"] == pytest.approx(-6.5)
    assert latest["z_max"] == pytest.approx(-2.8)
    assert latest["age_s"] >= 0.0


def test_scan_slow_client_keeps_latest_frames_and_drops_oldest():
    from gateway.gateway_module import GatewayModule
    from gateway.services.traffic import DROP_OLDEST_POLICY

    gateway = GatewayModule()
    queue, latest = gateway._cloud_viewer.scan_subscribe()

    assert latest is None

    scan_queue_maxsize = gateway._cloud_viewer.scan_queue_maxsize()
    for seq in range(scan_queue_maxsize + 2):
        gateway._cloud_viewer.publish_scan_frame(bytes([seq]))

    assert queue.qsize() == scan_queue_maxsize
    assert queue.get_nowait() == bytes([2])

    stats = gateway._traffic_stats_snapshot()
    assert stats["scan"]["clients"] == 1
    assert stats["scan"]["queue_maxsize"] == scan_queue_maxsize
    assert stats["scan"]["published_frames"] == scan_queue_maxsize + 2
    assert stats["scan"]["dropped_frames"] == 2
    assert stats["scan"]["drop_policy"] == DROP_OLDEST_POLICY
    assert stats["scan"]["latest_seq"] == scan_queue_maxsize + 2


def test_scan_stats_include_latest_frame_metadata():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    seq = gateway._cloud_viewer.publish_scan_frame(
        b"PCLD",
        metadata={
            "point_count": 8,
            "source": "slam_map_cloud",
            "z_min": -1.0,
            "z_max": 2.0,
        },
    )

    latest = gateway._traffic_stats_snapshot()["scan"]["latest_frame"]
    assert latest["seq"] == seq
    assert latest["bytes"] == 4
    assert latest["point_count"] == 8
    assert latest["source"] == "slam_map_cloud"
    assert latest["z_min"] == pytest.approx(-1.0)
    assert latest["z_max"] == pytest.approx(2.0)
    assert latest["age_s"] >= 0.0


def test_map_viewer_voxel_size_can_be_configured(monkeypatch):
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_MAP_VIEWER_VOXEL_SIZE", "0.05")
    gateway = GatewayModule()

    config = gateway._cloud_viewer.viewer_config()
    assert config["map_voxel_size"] == pytest.approx(0.05)
    assert config["inv_map_voxel_size"] == pytest.approx(20.0)


@numpy_safe_skip_mark()
def test_current_scan_allows_kilometer_scale_odom_frame_coordinates():
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    gateway = GatewayModule()
    pts = np.array(
        [
            [-2421.8, 1072.8, -21.8],
            [-2421.4, 1073.1, -21.6],
        ],
        dtype=np.float32,
    )

    gateway._cloud_viewer.configure(scan_viewer_min_interval_s=0.0)
    gateway._on_lidar_scan(PointCloud2(points=pts, frame_id="map"))

    traffic = gateway._traffic_stats_snapshot()
    assert traffic["scan"]["latest_frame"]["point_count"] == 2
    assert traffic["scan"]["latest_frame"]["source"] == "lidar_scan"


def test_gateway_health_and_bootstrap_expose_traffic_policy():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_bootstrap, build_app_traffic

    gateway = GatewayModule()
    subscribe(gateway)
    gateway._cloud_viewer.cloud_subscribe()

    health = gateway.health()
    bootstrap = build_app_bootstrap(gateway)
    traffic = build_app_traffic(gateway)

    assert health["gateway"]["traffic"]["sse"]["clients"] == 1
    assert health["gateway"]["traffic"]["cloud"]["clients"] == 1
    assert bootstrap["traffic"]["sse"]["queue_maxsize"] == gateway._sse_queue_maxsize
    assert bootstrap["traffic"]["cloud"]["queue_maxsize"] == gateway._cloud_viewer.cloud_queue_maxsize()
    assert bootstrap["traffic"]["recommended_client_rates_hz"]["state"] == 1.0
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
    gateway._cloud_viewer.configure(cloud_queue_maxsize=2)
    subscribe(gateway)
    gateway._cloud_viewer.cloud_subscribe()

    for seq in range(5):
        gateway.push_event({"type": "tick", "seq": seq})
        gateway._cloud_viewer.publish_cloud_frame(bytes([seq]))

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
    queue, snapshot_id = subscribe_with_event_id(gateway)

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
        queue = subscribe(gateway)

        thread = threading.Thread(target=lambda: gateway.push_event({"type": "threaded", "data": 1}))
        thread.start()
        try:
            event = await asyncio.wait_for(queue.get(), timeout=1.0)
        finally:
            thread.join(timeout=1.0)
            unsubscribe(gateway, queue)

        assert event["type"] == "threaded"
        assert event["event_id"] == 1
        assert gateway._traffic_stats_snapshot()["sse"]["clients"] == 0

    asyncio.run(_run())


def test_cloud_subscriber_receives_threaded_publish_on_endpoint_loop():
    from gateway.gateway_module import GatewayModule

    async def _run():
        gateway = GatewayModule()
        queue, latest = gateway._cloud_viewer.cloud_subscribe()
        assert latest is None

        thread = threading.Thread(target=lambda: gateway._cloud_viewer.publish_cloud_frame(b"PCL-threaded"))
        thread.start()
        try:
            frame = await asyncio.wait_for(queue.get(), timeout=1.0)
        finally:
            thread.join(timeout=1.0)
            gateway._cloud_viewer.cloud_unsubscribe(queue)

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
    gateway._camera_module = tracker
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
    gateway._cloud_viewer.publish_cloud_frame(b"PCL0")

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/cloud") as ws:
        assert ws.receive_bytes() == b"PCL0"
        assert gateway._traffic_stats_snapshot()["cloud"]["clients"] == 1
        gateway._cloud_viewer.publish_cloud_frame(b"PCL1")
        assert ws.receive_bytes() == b"PCL1"

    assert gateway._traffic_stats_snapshot()["cloud"]["clients"] == 0


def test_scan_websocket_sends_latest_and_cleans_up():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._cloud_viewer.publish_scan_frame(b"SCAN0")

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/scan") as ws:
        assert ws.receive_bytes() == b"SCAN0"
        assert gateway._traffic_stats_snapshot()["scan"]["clients"] == 1
        gateway._cloud_viewer.publish_scan_frame(b"SCAN1")
        assert ws.receive_bytes() == b"SCAN1"

    assert gateway._traffic_stats_snapshot()["scan"]["clients"] == 0


def test_teleop_websocket_reports_unconfirmed_manual_hold(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule
    from nav.adapters.native.commands import NavigationClientError

    class FailingPublisher:
        def claim(self, **kwargs):
            return operator_motion_receipt(
                OperatorMotionAction.CLAIM,
                kwargs["source_id"],
                kwargs["source_epoch"],
                kwargs["sequence"],
                kwargs.get("request_id"),
            )

        def quiesce_and_send_zero(self, **_kwargs):
            raise NavigationClientError("zero command rejected")

        def submit(self, *_args, **_kwargs):
            return True

    gateway = GatewayModule()
    gateway.setup()
    gateway._teleop_native_publisher = FailingPublisher()
    events = []
    gateway.push_event = events.append
    client = TestClient(gateway._app)

    with client.websocket_connect("/ws/teleop?client_id=operator-deadman") as ws:
        ws.send_text('{"type":"velocity","vx_mps":0.2,"vy_mps":0,"yaw_rps":0,"deadman":true}')
        assert json.loads(ws.receive_text())["type"] == "ingress_ack"
        ws.send_text('{"type":"velocity","vx_mps":0,"vy_mps":0,"yaw_rps":0,"deadman":false}')
        payload = json.loads(ws.receive_text())

        assert payload["type"] == "control_rejected"
        assert payload["error"] == "hold_unconfirmed"

    assert any(
        event["type"] == "control_rejected" and event["data"]["error"] == "disconnect_unconfirmed"
        for event in events
    )


def test_teleop_websocket_reports_disconnect_zero_when_release_raises():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._teleop_native_publisher = type(
        "ClaimingPublisher",
        (),
        {
            "claim": lambda self, **kwargs: operator_motion_receipt(
                OperatorMotionAction.CLAIM,
                kwargs["source_id"],
                kwargs["source_epoch"],
                kwargs["sequence"],
                kwargs.get("request_id"),
            ),
            "submit": lambda self, *_args, **_kwargs: True,
        },
    )()
    gateway._teleop_release = lambda **_kwargs: (_ for _ in ()).throw(RuntimeError("release crashed"))
    events = []
    gateway.push_event = events.append
    client = TestClient(gateway._app)

    with client.websocket_connect("/ws/teleop?client_id=operator-release-error") as ws:
        ws.send_text('{"type":"velocity","vx_mps":0.2,"vy_mps":0,"yaw_rps":0,"deadman":true}')
        assert json.loads(ws.receive_text())["type"] == "ingress_ack"

    assert any(
        event["type"] == "control_rejected" and event["data"]["error"] == "disconnect_unconfirmed"
        for event in events
    )


def test_teleop_websocket_rejects_truthy_non_boolean_disconnect_ack():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._teleop_native_publisher = type(
        "ClaimingPublisher",
        (),
        {
            "claim": lambda self, **kwargs: operator_motion_receipt(
                OperatorMotionAction.CLAIM,
                kwargs["source_id"],
                kwargs["source_epoch"],
                kwargs["sequence"],
                kwargs.get("request_id"),
            ),
            "submit": lambda self, *_args, **_kwargs: True,
        },
    )()
    gateway._teleop_release = lambda **_kwargs: "true"
    events = []
    gateway.push_event = events.append
    client = TestClient(gateway._app)

    with client.websocket_connect("/ws/teleop?client_id=operator-malformed-release") as ws:
        ws.send_text('{"type":"velocity","vx_mps":0.2,"vy_mps":0,"yaw_rps":0,"deadman":true}')
        assert json.loads(ws.receive_text())["type"] == "ingress_ack"

    assert any(
        event["type"] == "control_rejected" and event["data"]["error"] == "disconnect_unconfirmed"
        for event in events
    )


def test_teleop_websocket_reports_unavailable_native_command_queue():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._teleop_native_publisher = None
    client = TestClient(gateway._app)

    with client.websocket_connect("/ws/teleop?client_id=operator-native") as ws:
        ws.send_text('{"type":"velocity","vx_mps":0.2,"vy_mps":0,"yaw_rps":0,"deadman":true}')
        payload = json.loads(ws.receive_text())

        assert payload["type"] == "control_rejected"
        assert payload["error"] == "control_unavailable"


def test_teleop_websocket_rejects_truthy_non_boolean_claim_ack():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._teleop_native_publisher = type(
        "MalformedClaimPublisher",
        (),
        {"claim": lambda self, **_kwargs: "true"},
    )()
    client = TestClient(gateway._app)

    with client.websocket_connect("/ws/teleop?client_id=operator-malformed-claim") as ws:
        ws.send_text('{"type":"velocity","vx_mps":0.2,"vy_mps":0,"yaw_rps":0,"deadman":true}')
        payload = json.loads(ws.receive_text())

    assert payload["type"] == "control_rejected"
    assert payload["error"] == "control_unavailable"


def test_teleop_websocket_success_is_ingress_ack_not_control_ack():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    class NativePublisher:
        last_error = None

        def __init__(self) -> None:
            self.submitted = []
            self.zeroed = False
            self.claimed = False

        def claim(self, **kwargs):
            self.claimed = True
            return operator_motion_receipt(
                OperatorMotionAction.CLAIM,
                kwargs["source_id"],
                kwargs["source_epoch"],
                kwargs["sequence"],
                kwargs.get("request_id"),
            )

        def submit(self, vx, vy, wz, **kwargs):
            request_id = kwargs.get("request_id")
            self.submitted.append((vx, vy, wz, request_id))
            return True

        def quiesce_and_send_zero(self, **kwargs):
            self.zeroed = True
            return operator_motion_receipt(
                OperatorMotionAction.HOLD,
                kwargs["source_id"],
                kwargs["source_epoch"],
                kwargs["sequence"],
                kwargs.get("request_id"),
                final_output_sequence=kwargs["sequence"],
            )

        def release_source(self, **kwargs):
            return operator_motion_receipt(
                OperatorMotionAction.RELEASE,
                kwargs["source_id"],
                kwargs["source_epoch"],
                kwargs["sequence"],
                kwargs.get("request_id"),
                final_output_sequence=kwargs["sequence"],
            )

    gateway = GatewayModule()
    gateway.setup()
    publisher = NativePublisher()
    gateway._teleop_native_publisher = publisher
    events = []
    gateway.push_event = events.append
    client = TestClient(gateway._app)

    with client.websocket_connect("/ws/teleop?client_id=operator-native") as ws:
        ws.send_text('{"type":"velocity","vx_mps":0.2,"vy_mps":0,"yaw_rps":0,"deadman":true,"request_id":"velocity-1"}')
        payload = json.loads(ws.receive_text())

        assert payload == {
            "type": "ingress_ack",
            "action": "queued",
            "ingress_accepted": True,
            "stage": "gateway_queue_accepted",
            "request_id": "velocity-1",
            "replaceable": True,
            "final_cmd_vel_confirmed": False,
            "motor_confirmed": False,
        }
        assert publisher.submitted
        assert publisher.submitted[-1][3] == "velocity-1"
        assert publisher.claimed is True

    assert publisher.zeroed is True
    assert not any(
        event.get("type") == "control_rejected" and event.get("data", {}).get("error") == "disconnect_unconfirmed"
        for event in events
    )


def test_teleop_client_counter_helpers_are_thread_safe():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()

    def connect_and_disconnect_many():
        for _ in range(1000):
            gateway._teleop_client_connected()
            gateway._teleop_client_disconnected()

    threads = [threading.Thread(target=connect_and_disconnect_many) for _ in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert gateway._teleop_client_count() == 0
    assert gateway._teleop_client_disconnected() == 0


def test_websocket_registry_does_not_advertise_unimplemented_heartbeat_metrics():
    from gateway.services.ws_registry import WebSocketRegistry

    registry = WebSocketRegistry()
    registry.register("teleop-1", "/ws/teleop", client_id="operator-a")

    snapshot = registry.snapshot()
    detail = registry.connections_detail()[0]

    assert snapshot["active_connections"] == 1
    assert snapshot["by_endpoint"] == {"/ws/teleop": 1}
    assert "heartbeat_interval_s" not in snapshot
    assert "heartbeat_timeout_s" not in snapshot
    assert "timeout_disconnects" not in snapshot
    assert "since_pong_s" not in detail
    assert "messages_rx" not in detail


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

    drift_thread = gateway._drift_watchdog_thread
    assert drift_thread is not None
    assert drift_thread.is_alive()

    gateway.stop()

    assert gateway._drift_watchdog_thread is None
    assert not drift_thread.is_alive()


def test_gateway_stop_retains_background_thread_when_join_times_out():
    from gateway.gateway_module import GatewayModule

    class StuckThread:
        name = "drift_watchdog"

        def __init__(self):
            self.join_timeouts = []

        def is_alive(self):
            return True

        def join(self, timeout=None):
            self.join_timeouts.append(timeout)

    gateway = GatewayModule()
    old_event = gateway._stop_event
    stuck = StuckThread()
    gateway._drift_watchdog_thread = stuck

    gateway.stop()

    assert old_event.is_set()
    assert stuck.join_timeouts == [2.0]
    assert gateway._drift_watchdog_thread is stuck


def test_gateway_start_replaces_stopped_event():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._defer_server = True
    gateway._drift_watchdog_enabled = False
    gateway.setup()
    old_event = gateway._stop_event
    old_event.set()
    gateway.start()

    assert gateway._stop_event is not old_event
    assert old_event.is_set()

    gateway.stop()


def test_gateway_start_does_not_duplicate_background_threads():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._defer_server = True
    gateway._drift_watchdog_enabled = True
    gateway._drift_watchdog_interval = 60.0
    gateway.setup()
    gateway.start()

    drift_thread = gateway._drift_watchdog_thread

    gateway.start()

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
