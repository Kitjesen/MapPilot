from __future__ import annotations

import asyncio
import threading
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("fastapi")


@pytest.mark.parametrize("endpoint", ["/api/v1/state", "/api/v1/events", "/api/v1/navigation/status"])
def test_state_reads_do_not_block_the_teleop_event_loop(monkeypatch, endpoint):
    from gateway.gateway_module import GatewayModule
    from gateway.navigation import routes as navigation_routes
    from gateway.routes import realtime, status

    gateway = GatewayModule()
    gateway.setup()
    threads = []

    def blocking_snapshot(_gateway):
        threads.append(threading.get_ident())
        return {"probe": "state"}

    monkeypatch.setattr(status, "build_state_snapshot", blocking_snapshot)
    monkeypatch.setattr(realtime, "build_state_snapshot", blocking_snapshot)
    monkeypatch.setattr(navigation_routes, "build_navigation_status", blocking_snapshot)
    route = next(route for route in gateway._app.routes if route.path == endpoint)

    async def read():
        loop_thread = threading.get_ident()
        response = await route.endpoint()
        if endpoint.endswith("events"):
            try:
                first = await anext(response.body_iterator)
                assert '"probe":"state"' in first
            finally:
                await response.body_iterator.aclose()
        else:
            assert response == {"probe": "state"}
        assert len(threads) == 1
        assert threads[0] != loop_thread

    asyncio.run(read())


def test_state_snapshot_exposes_native_navigation_state_and_client_contract():
    from gateway.gateway_module import GatewayModule
    from gateway.services.state_snapshot import build_state_snapshot

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0}
        gateway._navigation_state = {
            "ts": time.time(),
            "boot_id": "boot-1",
            "lifecycle_state_name": "EXECUTING",
            "active_task_id": "task-1",
            "active_request_id": "request-1",
            "authority": "autonomy",
        }
        gateway._navigation_goal_status_by_task["task-1"] = {
            "boot_id": "boot-1",
            "task_id": "task-1",
            "request_id": "request-1",
            "state_name": "EXECUTING",
        }
        gateway._mode = "autonomous"
        gateway._teleop_clients = 2
        gateway._sg_json = '{"nodes":[]}'
        gateway._last_path = [{"x": 0.0}, {"x": 1.0}]
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.8}
        gateway._visual_servo_status = {
            "mode": "idle",
            "target": "",
            "follow_available": True,
        }
    monkeypatch = pytest.MonkeyPatch()
    monkeypatch.setattr("gateway.gateway_module.native_teleop_active", lambda: True)

    payload = build_state_snapshot(gateway)
    monkeypatch.undo()

    assert payload["schema_version"] == 4
    assert payload["ts"] > 0
    assert payload["server"]["time"] == payload["ts"]
    assert payload["server"]["api_version"] == "v1"
    assert payload["localization"]["odometry"] == {"x": 1.0, "y": 2.0}
    assert payload["navigation"]["task"]["state"] == "EXECUTING"
    assert set(payload["navigation"]) == {
        "schema_version",
        "task",
        "goal_admission",
        "control",
        "motion",
        "ts",
    }
    assert payload["session"]["mode"] == "idle"
    assert payload["lease"]["holder"] is None
    assert payload["teleop"] == {"active": True, "clients": 2}
    assert payload["localization"]["reported_state"] == "TRACKING"
    assert payload["localization"]["confidence"] == 0.8
    assert payload["visual_servo"]["follow_available"] is True
    assert payload["scene"]["available"] is True
    assert payload["path"]["points"] == 2
    assert payload["links"]["capabilities"] == "/api/v1/app/capabilities"
    assert payload["links"]["localization_status"] == "/api/v1/localization/status"
    assert payload["links"]["navigation_status"] == "/api/v1/navigation/status"


def test_state_route_returns_stable_snapshot():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    route = next(route for route in gateway._app.routes if route.path == "/api/v1/state")
    payload = asyncio.run(route.endpoint())

    assert payload["schema_version"] == 4
    assert payload["ts"] > 0
    assert "odometry" in payload["localization"]
    assert "teleop" in payload
    assert "navigation" in payload
    assert payload["links"]["events"] == "/api/v1/events"


def test_state_localization_preserves_bound_runtime_identity():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status
    from gateway.services.state_snapshot import build_state_snapshot

    gateway = GatewayModule()
    gateway._compiled_env = "sim"
    gateway._compiled_product = "nav"
    gateway._compiled_product_session_id = "session-1"
    gateway._compiled_run_plan = SimpleNamespace(required_topics=(), has_process=lambda _role: False)

    snapshot = build_state_snapshot(gateway)
    status = build_localization_status(gateway)

    assert snapshot["localization"]["runtime"] == status["runtime"]
    assert snapshot["localization"]["runtime"]["env"] == "sim"
    assert snapshot["localization"]["runtime"]["product"] == "nav"
    assert snapshot["localization"]["runtime"]["state"] == "active"


def test_state_keeps_one_navigation_sample_when_callback_arrives(monkeypatch):
    from gateway.gateway_module import GatewayModule
    from gateway.services.state_snapshot import build_state_snapshot

    gateway = GatewayModule()
    gateway._navigation_state = {
        "ts": time.time(),
        "boot_id": "boot-1",
        "lifecycle_state_name": "EXECUTING",
        "active_task_id": "task-1",
        "active_request_id": "request-1",
        "authority": "autonomy",
    }
    gateway._navigation_goal_status_by_task["task-1"] = {
        "boot_id": "boot-1",
        "task_id": "task-1",
        "request_id": "request-1",
        "state_name": "EXECUTING",
    }

    def receive_new_state():
        with gateway._state_lock:
            gateway._navigation_state = None
            gateway._navigation_goal_status_by_task.clear()
        return 0

    monkeypatch.setattr(gateway, "_teleop_client_count", receive_new_state)
    snapshot = build_state_snapshot(gateway)

    assert snapshot["navigation"]["task"]["task_id"] == "task-1"
    assert snapshot["navigation"]["task"]["state"] == "EXECUTING"
    assert gateway._navigation_state is None


def test_state_snapshot_includes_camera_media_status():
    from gateway.gateway_module import GatewayModule
    from gateway.services.state_snapshot import build_state_snapshot

    class Camera:
        def health(self):
            return {
                "backend": "dds",
                "ports_out": {
                    "color_image": {
                        "msg_count": 0,
                        "rate_hz": 0.0,
                        "stale_ms": -1.0,
                    },
                    "depth_image": {
                        "msg_count": 0,
                        "rate_hz": 0.0,
                        "stale_ms": -1.0,
                    },
                    "camera_info": {
                        "msg_count": 0,
                    },
                },
            }

    gateway = GatewayModule()
    gateway._all_modules = {"camera": Camera()}

    payload = build_state_snapshot(gateway)
    camera = payload["media"]["camera"]

    assert payload["media"]["camera_ws"] == "/ws/camera"
    assert payload["media"]["camera_snapshot"] == "/api/v1/camera/snapshot"
    assert camera["available"] is False
    assert camera["status"] == "idle"
    assert camera["reason"] == "no_color_frames"
    assert camera["jpeg"]["cached"] is False
    assert camera["jpeg"]["seq"] == 0


def test_camera_media_status_reports_not_loaded_without_camera():
    from gateway.services.media_status import build_camera_status

    payload = build_camera_status(SimpleNamespace(_all_modules={}))

    assert payload["available"] is False
    assert payload["status"] == "not_loaded"
    assert payload["reason"] == "camera_not_loaded"
    assert payload["frames"] == 0
    assert payload["jpeg"]["cached"] is False


def test_camera_media_status_reports_cached_jpeg_without_camera():
    from gateway.services.media_status import build_camera_status

    payload = build_camera_status(
        SimpleNamespace(
            _all_modules={},
            _latest_jpeg=b"\xff\xd8\xffcamera",
            _latest_jpeg_seq=7,
            _jpeg_lock=None,
        )
    )

    assert payload["available"] is False
    assert payload["status"] == "not_loaded"
    assert payload["reason"] == "camera_not_loaded"
    assert payload["jpeg"]["cached"] is True
    assert payload["jpeg"]["seq"] == 7
    assert payload["jpeg"]["bytes"] == len(b"\xff\xd8\xffcamera")


def test_camera_media_status_accepts_canonical_camera_role():
    from gateway.services.media_status import build_camera_status

    class Camera:
        def health(self):
            return {
                "backend": "dds",
                "ports_out": {
                    "color_image": {
                        "msg_count": 4,
                        "rate_hz": 9.8,
                        "stale_ms": 20.0,
                    },
                    "depth_image": {
                        "msg_count": 4,
                        "rate_hz": 9.7,
                        "stale_ms": 22.0,
                    },
                    "camera_info": {"msg_count": 1},
                },
            }

    payload = build_camera_status(SimpleNamespace(_all_modules={"camera": Camera()}))

    assert payload["role"] == "camera"
    assert payload["available"] is True
    assert payload["status"] == "streaming"
    assert payload["frames"] == 4


@pytest.mark.parametrize("removed_role", ["CameraBridgeModule", "CameraModule"])
def test_camera_media_status_rejects_removed_graph_aliases(removed_role):
    from gateway.services.media_status import build_camera_status

    class Camera:
        def health(self):
            raise AssertionError("removed camera role must not be resolved")

    payload = build_camera_status(
        SimpleNamespace(_all_modules={removed_role: Camera()})
    )

    assert payload["role"] == "camera"
    assert payload["status"] == "not_loaded"
    assert payload["reason"] == "camera_not_loaded"


def test_camera_media_status_marks_old_or_legacy_frames_stale():
    from gateway.services.media_status import build_camera_status

    class Camera:
        def health(self):
            return {
                "backend": "dds",
                "ports_out": {
                    "color_image": {"msg_count": 12, "rate_hz": 7.2},
                    "depth_image": {
                        "msg_count": 10,
                        "rate_hz": 6.8,
                        "stale_ms": 6200.0,
                    },
                    "camera_info": {"msg_count": 1},
                },
            }

    payload = build_camera_status(SimpleNamespace(_all_modules={"camera": Camera()}))

    assert payload["available"] is False
    assert payload["status"] == "stale"
    assert payload["reason"] == "camera_frames_stale"
    assert payload["frames"] == 12
    assert payload["color"]["stale_ms"] is None
    assert payload["depth"]["stale_ms"] == 6200.0


def test_camera_media_status_reports_health_errors_without_crashing():
    from gateway.services.media_status import build_camera_status

    class Camera:
        def health(self):
            raise RuntimeError("camera unavailable")

    payload = build_camera_status(SimpleNamespace(_all_modules={"camera": Camera()}))

    assert payload["available"] is False
    assert payload["status"] == "error"
    assert payload["reason"] == "camera_health_error"
    assert "camera unavailable" in payload["error"]
