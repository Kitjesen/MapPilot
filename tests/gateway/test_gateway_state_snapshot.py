from __future__ import annotations

import asyncio
from types import SimpleNamespace

import pytest

pytest.importorskip("fastapi")


def test_state_snapshot_exposes_native_navigation_state_and_client_contract():
    from gateway.gateway_module import GatewayModule
    from gateway.services.state_snapshot import build_state_snapshot

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0}
        gateway._navigation_state = {
            "lifecycle_state_name": "EXECUTING",
            "authority": "autonomy",
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

    assert payload["schema_version"] == 2
    assert payload["ts"] > 0
    assert payload["server"]["time"] == payload["ts"]
    assert payload["server"]["api_version"] == "v1"
    assert payload["localization"]["odometry"] == {"x": 1.0, "y": 2.0}
    assert payload["navigation"]["diagnostics"]["safety"]["stop_active"] is False
    assert payload["navigation"]["navigation_state"]["lifecycle_state_name"] == "EXECUTING"
    assert payload["session"]["mode"] == "idle"
    assert payload["lease"]["holder"] is None
    assert payload["teleop"] == {"active": True, "clients": 2}
    assert payload["localization"]["reported_state"] == "TRACKING"
    assert payload["localization"]["confidence"] == 0.8
    assert payload["navigation"]["state"] == "EXECUTING"
    assert payload["navigation"]["path"]["points"] == 2
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

    assert payload["schema_version"] == 2
    assert payload["ts"] > 0
    assert "odometry" in payload["localization"]
    assert "teleop" in payload
    assert "navigation" in payload
    assert payload["links"]["events"] == "/api/v1/events"


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
