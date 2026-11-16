from __future__ import annotations

import json
import struct
import time

import httpx
import pytest
from fastapi.testclient import TestClient

from gateway.gateway_module import GatewayModule
from gateway.video_probe import create_app
from runtime.transport.camera_shm import ShmFrameWriter, StreamKind


@pytest.fixture
def video_environment(monkeypatch, tmp_path):
    for name in ("LINGTU_PRODUCT", "LINGTU_PRODUCT_SESSION_ID", "LINGTU_RUN_PLAN"):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv("LINGTU_ROBOT", "unitree/go2")
    monkeypatch.setenv("LINGTU_ENV", "real")
    monkeypatch.setenv("LINGTU_WEBRTC_URL", "http://127.0.0.1:19090/whep/endpoint")
    monkeypatch.setenv("LINGTU_VIDEO_STATUS_FILE", str(tmp_path / "video.json"))
    monkeypatch.setenv("LINGTU_CAMERA_STATUS_FILE", str(tmp_path / "camera.json"))
    monkeypatch.delenv("LINGTU_MEDIA_REQUIRE_TOKEN", raising=False)
    monkeypatch.delenv("LINGTU_MEDIA_VIEWER_TOKEN", raising=False)
    return tmp_path / "video.json"


def test_unstarted_gateway_has_no_control_routes_or_advertised_control(video_environment, monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("video gateway must not load or start a Host")

    monkeypatch.setattr(GatewayModule, "setup", forbidden)
    monkeypatch.setattr(GatewayModule, "start", forbidden)
    app = create_app()
    client = TestClient(app)
    for path, version in (
        ("/api/v1/app/bootstrap", 4), ("/api/v1/app/capabilities", 2),
        ("/api/v1/state", 4), ("/api/v1/readiness", 1),
    ):
        response = client.get(path)
        assert response.status_code == 200
        assert response.json()["schema_version"] == version
        assert response.json()["ts"] <= time.time()
    readiness = client.get("/api/v1/readiness").json()
    assert readiness["ready"] is False and readiness["motion_ready"] is False
    assert readiness["reasons"] == ["no_modules_loaded"]
    assert readiness["module_count"] == 0
    bootstrap = client.get("/api/v1/app/bootstrap").json()
    assert bootstrap["session"]["product"] is None
    assert bootstrap["session"]["product_session_id"] is None
    assert bootstrap["robot"]["online"] is None
    capabilities = client.get("/api/v1/app/capabilities").json()
    assert capabilities["features"]["teleop"] is False
    assert capabilities["features"]["navigation"] is False
    assert capabilities["features"]["whep"] is True
    assert capabilities["endpoints"]["control"] == {}
    assert "lease" not in capabilities["links"]
    writes = {
        (method, route.path) for route in app.routes
        for method in getattr(route, "methods", ()) if method in {"POST", "PUT", "PATCH", "DELETE"}
    }
    assert writes == {
        ("POST", "/api/v1/webrtc/whep"),
        ("DELETE", "/api/v1/webrtc/sessions/{resource_path:path}"),
    }
    assert client.post("/api/v1/goal", json={}).status_code == 404


def test_real_video_routes_use_configured_status_and_upstream(video_environment, monkeypatch):
    video_environment.write_text(json.dumps({"ready": True, "status": "running"}))
    requests = []

    def upstream(request):
        requests.append(request)
        if request.method == "OPTIONS":
            return httpx.Response(204)
        return httpx.Response(200, text="answer", headers={"Content-Type": "application/sdp"})

    original = httpx.AsyncClient
    monkeypatch.setattr(
        httpx, "AsyncClient", lambda **kw: original(transport=httpx.MockTransport(upstream), **kw)
    )
    client = TestClient(create_app())
    assert client.get("/api/v1/webrtc/status").json()["available"] is True
    assert client.post("/api/v1/webrtc/whep", content="offer").text == "answer"
    assert [request.method for request in requests] == ["OPTIONS", "POST"]
    assert all(str(request.url) == "http://127.0.0.1:19090/whep/endpoint" for request in requests)


def test_probe_projects_file_backed_camera_status(video_environment, tmp_path):
    camera_file = tmp_path / "camera.json"
    camera_file.write_text(
        json.dumps(
            {
                "status": "running",
                "color_frames": 42,
                "depth_frames": 41,
                "info_frames": 2,
                "last_error": "",
            }
        )
    )
    client = TestClient(create_app())
    payload = client.get("/api/v1/camera/status").json()
    assert payload["available"] is True
    assert payload["status"] == "streaming"
    assert payload["color"]["frames"] == 42
    assert payload["depth"]["frames"] == 41


def test_probe_reports_rate_after_two_status_samples(video_environment, tmp_path):
    camera_file = tmp_path / "camera.json"
    camera_file.write_text(json.dumps({"status": "running", "color_frames": 42, "depth_frames": 41}))
    client = TestClient(create_app())
    assert client.get("/api/v1/camera/status").json()["color"]["fps"] == 0.0
    camera_file.write_text(json.dumps({"status": "running", "color_frames": 72, "depth_frames": 71}))
    payload = client.get("/api/v1/camera/status").json()
    assert payload["color"]["fps"] > 0.0
    assert payload["depth"]["fps"] > 0.0


def test_optional_camera_mcp_exposes_only_read_tools(video_environment, monkeypatch):
    monkeypatch.setenv("LINGTU_CAMERA_MCP_ENABLED", "1")
    client = TestClient(create_app())
    tools = client.post("/mcp", json={"jsonrpc": "2.0", "id": 1, "method": "tools/list"}).json()
    assert [tool["name"] for tool in tools["result"]["tools"]] == ["get_camera_status", "capture_camera", "capture_depth"]
    assert client.post("/mcp", json={"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {"name": "set_mode", "arguments": {}}}).json()["error"]["code"] == -32602


def test_depth_snapshot_reports_unavailable_when_shm_is_missing(video_environment, tmp_path, monkeypatch):
    monkeypatch.setenv("LINGTU_CAMERA_DEPTH_SHM", str(tmp_path / "missing-depth"))
    response = TestClient(create_app()).get("/api/v1/camera/depth/snapshot")
    assert response.status_code == 503
    assert response.json()["sensor"] == "depth"


@pytest.mark.parametrize("age_s", [0, 5])
def test_depth_snapshot_reads_fresh_shm_and_rejects_stale_frames(video_environment, tmp_path, monkeypatch, age_s):
    path = tmp_path / "depth.shm"
    monkeypatch.setenv("LINGTU_CAMERA_DEPTH_SHM", str(path))
    timestamp_ns = time.time_ns() - age_s * 1_000_000_000
    with ShmFrameWriter(path, stream_kind=StreamKind.DEPTH, slot_capacity=12) as writer:
        writer.publish(
            timestamp_ns=timestamp_ns,
            width=2,
            height=2,
            stride=6,
            encoding="16UC1",
            frame_id="camera_depth_link",
            payload=struct.pack("<6H", 1000, 1500, 60000, 2000, 2500, 60000),
            depth_scale=0.001,
        )
        response = TestClient(create_app()).get("/api/v1/camera/depth/snapshot")

    if age_s:
        payload = response.json()
        assert response.status_code == 503
        assert payload["success"] is False
        assert "stale threshold" in payload["error"]
    else:
        assert response.status_code == 200
        assert response.headers["content-type"] == "image/jpeg"
        assert response.headers["cache-control"] == "no-store"
        assert response.headers["x-depth-width"] == "2"
        assert response.headers["x-depth-height"] == "2"
        assert 1.0 <= float(response.headers["x-depth-near-m"]) < float(response.headers["x-depth-far-m"]) <= 2.5
        assert response.content.startswith(b"\xff\xd8\xff")


def test_media_authorization_is_kept(video_environment, monkeypatch):
    monkeypatch.setenv("LINGTU_MEDIA_REQUIRE_TOKEN", "1")
    client = TestClient(create_app())
    assert client.get("/api/v1/app/bootstrap").status_code == 200
    assert client.get("/api/v1/webrtc/status").status_code == 503


def test_probe_serves_only_declared_assets(video_environment, tmp_path):
    for name in ("index.html", "probe.mjs", "probe-core.mjs", "whepReceiver.mjs"):
        (tmp_path / name).write_text(name)
    (tmp_path / "private.txt").write_text("not served")
    client = TestClient(create_app(tmp_path))
    assert client.get("/video-probe/").text == "index.html"
    assert client.get("/video-probe/probe.mjs").text == "probe.mjs"
    assert client.get("/video-probe/private.txt").status_code == 404
    assert client.post("/video-probe/probe.mjs").status_code == 405


@pytest.mark.parametrize("name", ["LINGTU_PRODUCT", "LINGTU_PRODUCT_SESSION_ID", "LINGTU_RUN_PLAN"])
def test_managed_runtime_cannot_be_misrepresented_as_isolated(video_environment, monkeypatch, name):
    monkeypatch.setenv(name, "existing-runtime")
    with pytest.raises(ValueError, match=name):
        create_app()
