from __future__ import annotations

import asyncio
import json
import os
import time
from types import SimpleNamespace

import httpx
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.routes import video
from gateway.routes.video import register_video_routes


def client_for(monkeypatch, handler, *, native="", ice=None, status_file="", camera=None):
    original = httpx.AsyncClient
    transport = httpx.MockTransport(handler)
    monkeypatch.setattr(httpx, "AsyncClient", lambda **kwargs: original(transport=transport, **kwargs))
    app = FastAPI()
    gateway = SimpleNamespace(
        _go2rtc_upstream="http://127.0.0.1:1984",
        _webrtc_upstream=native,
        _webrtc_ice_servers=ice or [],
        _webrtc_rotation=180,
        _webrtc_status_file=str(status_file),
        _all_modules={"camera": camera} if camera is not None else {},
    )
    register_video_routes(app, gateway)
    return TestClient(app)


def test_existing_go2rtc_stream_and_query_are_preserved(monkeypatch):
    calls = []

    def upstream(request):
        calls.append(request)
        if request.url.path == "/api/streams":
            return httpx.Response(200, json={"cam": {}})
        assert request.url.path == "/api/webrtc"
        assert dict(request.url.params) == {"src": "cam&other=1"}
        assert request.content == b"offer"
        return httpx.Response(200, text="answer", headers={"Content-Type": "application/sdp"})

    client = client_for(monkeypatch, upstream)
    assert client.get("/api/v1/webrtc/go2rtc/status").json()["streams"] == ["cam"]
    status = client.get("/api/v1/webrtc/status").json()
    assert status["available"] and status["backend"] == "go2rtc"
    answer = client.post("/api/v1/webrtc/whep", params={"src": "cam&other=1"}, content="offer")
    assert answer.status_code == 200 and answer.text == "answer"
    assert len(calls) == 3


def test_native_status_and_resource_deletion(monkeypatch):
    deleted = []
    ice = [{"urls": "turn:relay.example:3478", "username": "viewer", "credential": "temporary"}]

    def upstream(request):
        if request.method == "OPTIONS":
            assert request.url.path == "/whep/endpoint"
            return httpx.Response(204)
        if request.method == "POST":
            assert request.url.path == "/whep/endpoint" and not request.url.query
            assert request.extensions["timeout"]["read"] == 35.0
            return httpx.Response(
                201,
                text="native-answer",
                headers={"Location": "/whep/resource/session-1", "ETag": '"version-1"'},
            )
        deleted.append(request)
        return httpx.Response(204)

    client = client_for(
        monkeypatch, upstream, native="http://127.0.0.1:9090/whep/endpoint", ice=ice
    )
    status = client.get("/api/v1/webrtc/status").json()
    assert status["available"] and status["backend"] == "gstreamer"
    assert status["ice_servers"] == ice
    assert status["rotation_degrees"] == 180
    answer = client.post("/api/v1/webrtc/whep?src=cam", content="offer")
    assert answer.status_code == 201
    assert answer.headers["location"] == "/api/v1/webrtc/sessions/whep/resource/session-1"
    response = client.delete(answer.headers["location"], headers={"If-Match": answer.headers["etag"]})
    assert response.status_code == 204
    assert str(deleted[0].url) == "http://127.0.0.1:9090/whep/resource/session-1"
    assert deleted[0].headers["if-match"] == '"version-1"'


def test_native_negotiation_has_a_total_budget_beyond_server_ice_gathering(monkeypatch):
    deadlines = []

    async def short_wait_for(operation, *, timeout):
        deadlines.append(timeout)
        return await asyncio.wait_for(operation, timeout=0.01)

    async def waiting_for_ice(request):
        await asyncio.Event().wait()

    monkeypatch.setattr(video, "wait_for", short_wait_for)
    client = client_for(monkeypatch, waiting_for_ice, native="http://127.0.0.1:9090/whep/endpoint")
    response = client.post("/api/v1/webrtc/whep", content="offer")

    assert deadlines == [35.0]
    assert response.status_code == 503
    assert response.json()["error"] == "media_unreachable"


@pytest.mark.parametrize("location", ["http://different.example/session", "/unrelated/session"])
def test_resource_stays_with_configured_media_endpoint(monkeypatch, location):
    client = client_for(
        monkeypatch,
        lambda request: httpx.Response(201, headers={"Location": location}),
        native="http://127.0.0.1:9090/whep/endpoint",
    )
    response = client.post("/api/v1/webrtc/whep", content="offer")
    assert response.status_code == 502
    assert response.json()["error"] == "invalid_media_resource"
    assert client.delete("/api/v1/webrtc/sessions/unrelated/service").status_code == 400


def test_native_endpoint_unavailable_is_an_explicit_status(monkeypatch):
    def unavailable(request):
        raise httpx.ConnectError("offline", request=request)

    client = client_for(monkeypatch, unavailable, native="http://127.0.0.1:9090/whep/endpoint")
    status = client.get("/api/v1/webrtc/status").json()
    assert not status["available"] and status["reason"] == "ConnectError"
    assert client.post("/api/v1/webrtc/whep", content="offer").status_code == 503


def test_upstream_negotiation_error_is_preserved(monkeypatch):
    client = client_for(
        monkeypatch,
        lambda request: httpx.Response(415, text="unsupported codec"),
        native="http://127.0.0.1:9090/whep/endpoint",
    )
    response = client.post("/api/v1/webrtc/whep", content="offer")
    assert response.status_code == 415 and response.text == "unsupported codec"


def test_native_listener_alone_does_not_claim_fresh_video(monkeypatch, tmp_path):
    path = tmp_path / "video.json"
    client = client_for(
        monkeypatch, lambda request: httpx.Response(204),
        native="http://127.0.0.1:9090/whep/endpoint", status_file=path,
    )
    status = client.get("/api/v1/webrtc/status").json()
    assert status["whep_available"] is True
    assert status["available"] is False and status["source_ready"] is False

    path.write_text(json.dumps({"ready": True, "status": "running", "source_generation": 3}))
    status = client.get("/api/v1/webrtc/status").json()
    assert status["available"] is True and status["source_ready"] is True
    assert status["source_status"]["source_generation"] == 3

    old = time.time() - 5
    os.utime(path, (old, old))
    assert client.get("/api/v1/webrtc/status").json()["available"] is False

    path.write_text(json.dumps({"ready": False, "status": "stopped"}))
    assert client.get("/api/v1/webrtc/status").json()["available"] is False


def video_snapshot(**updates):
    return {
        "schema_version": "lingtu.camera_video.status.v1",
        "status": "running", "ready": True, "source_fresh": True,
        "heartbeat_timestamp_s": time.time(), "source_generation": 8, "source_sequence": 15,
        "frame_age_ms": 12, "encoder": "x264enc", "last_error": "",
        **updates,
    }


def test_unified_diagnostics_uses_observations_without_opening_a_camera_or_media_session(monkeypatch, tmp_path):
    methods = []
    path = tmp_path / "video.json"
    path.write_text(json.dumps(video_snapshot(endpoint="http://secret:credential@internal/whep?token=value")))
    port = {"msg_count": 30, "rate_hz": 30, "stale_ms": 20}
    camera = SimpleNamespace(health=lambda: {"ports_out": {"color_image": port}})

    def upstream(request):
        methods.append(request.method)
        return httpx.Response(204)

    client = client_for(monkeypatch, upstream, native="http://127.0.0.1:9090/whep/endpoint", status_file=path, camera=camera)
    response = client.get("/api/v1/camera/diagnostics")
    status = response.json()
    assert response.status_code == 200 and methods == ["OPTIONS"]
    assert status["schema_version"] == "lingtu.camera_video.diagnostics.v1"
    assert status["camera"]["frames_fresh"] is True
    assert status["camera"]["physical_online"] is None
    assert status["video"]["pipeline_ready"] is True and status["video"]["encoder"] == "x264enc"
    assert status["whep"]["available"] is True and status["ready_for_receiver"] is True
    assert status["receiver"]["decoded_frames_advancing"] is None
    assert status["video"]["encoder_output_verified"] is None
    assert "credential" not in response.text and "internal" not in response.text


@pytest.mark.parametrize("updates,reason", [
    ({"source_fresh": False}, "video_source_not_ready"),
    ({"schema_version": "old"}, "video_status_invalid"),
    ({"heartbeat_timestamp_s": time.time() - 10}, "video_status_stale"),
    ({"heartbeat_timestamp_s": time.time() + 3600}, "video_status_stale"),
])
def test_diagnostics_rejects_stale_or_invalid_ready_snapshots(monkeypatch, tmp_path, updates, reason):
    path = tmp_path / "video.json"
    path.write_text(json.dumps(video_snapshot(**updates)))
    client = client_for(monkeypatch, lambda request: httpx.Response(204), native="http://127.0.0.1:9090/whep/endpoint", status_file=path)
    status = client.get("/api/v1/camera/diagnostics").json()
    assert status["video"]["pipeline_ready"] is False
    assert status["video"]["reason"] == reason
    assert status["ready_for_receiver"] is False


def test_diagnostics_separates_camera_frames_pipeline_and_whep_failure(monkeypatch, tmp_path):
    path = tmp_path / "video.json"
    path.write_text(json.dumps(video_snapshot(last_error="failed http://user:password@internal/path")))
    port = {"msg_count": 30, "rate_hz": 0, "stale_ms": 6000}
    camera = SimpleNamespace(health=lambda: {"ports_out": {"color_image": port}})
    client = client_for(monkeypatch, lambda request: httpx.Response(503), native="http://127.0.0.1:9090/whep/endpoint", status_file=path, camera=camera)
    response = client.get("/api/v1/camera/diagnostics")
    status = response.json()
    assert status["camera"]["frames_fresh"] is False
    assert status["camera"]["reason"] == "camera_frames_stale"
    assert status["video"]["pipeline_ready"] is True
    assert status["video"]["last_error"] == "native_video_error"
    assert status["whep"]["available"] is False and status["ready_for_receiver"] is False
    assert "password" not in response.text


def test_diagnostics_keeps_unobserved_native_pipeline_unknown_for_go2rtc(monkeypatch):
    client = client_for(monkeypatch, lambda request: httpx.Response(200, json={"cam": {}}))
    status = client.get("/api/v1/camera/diagnostics").json()
    assert status["camera"]["status"] == "not_loaded"
    assert status["camera"]["frames_fresh"] is None
    assert status["video"]["pipeline_ready"] is None
    assert status["video"]["reason"] == "native_video_status_not_supported"
    assert status["whep"]["available"] is True
    assert status["ready_for_receiver"] is None
