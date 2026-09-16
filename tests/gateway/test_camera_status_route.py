from types import SimpleNamespace

from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.routes.camera import register_camera_routes


def test_camera_status_reports_not_loaded_without_probing_hardware():
    app = FastAPI()
    register_camera_routes(app, SimpleNamespace(_all_modules={}))
    with TestClient(app) as client:
        response = client.get("/api/v1/camera/status")
    assert response.status_code == 200
    assert response.json()["reason"] == "camera_not_loaded"
    assert not response.json()["available"]


def test_camera_status_distinguishes_live_frames_from_stale_frames():
    port = {"msg_count": 30, "rate_hz": 30, "stale_ms": 20}
    camera = SimpleNamespace(health=lambda: {"ports_out": {
        "color_image": port, "depth_image": port, "camera_info": port,
    }})
    app = FastAPI()
    register_camera_routes(app, SimpleNamespace(_all_modules={"camera": camera}))
    with TestClient(app) as client:
        live = client.get("/api/v1/camera/status").json()
        assert live["available"] and live["color"]["fps"] == 30
        port["stale_ms"] = 6000
        stale = client.get("/api/v1/camera/status").json()
        assert not stale["available"]
        assert stale["reason"] == "camera_frames_stale"
