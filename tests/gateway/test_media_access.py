from __future__ import annotations

import pytest
from fastapi import FastAPI, WebSocket
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from gateway.gateway_module import GatewayModule
from gateway.media_access import MediaAccessMiddleware

TOKEN = "test-only-independent-media-viewer-token"
MEDIA_REQUESTS = [
    ("GET", "/api/v1/camera/status"),
    ("GET", "/api/v1/camera/snapshot"),
    ("GET", "/api/v1/camera/diagnostics"),
    ("GET", "/api/v1/webrtc/status"),
    ("GET", "/api/v1/webrtc/go2rtc/status"),
    ("POST", "/api/v1/webrtc/whep"),
    ("DELETE", "/api/v1/webrtc/sessions/whep/resource/session-1"),
]


def media_client(*, required=True, token=TOKEN):
    app = FastAPI()
    calls = []
    app.add_middleware(MediaAccessMiddleware, required=required, viewer_token=token)

    async def media():
        calls.append("media")
        return {"reached": True}

    for method, path in MEDIA_REQUESTS:
        app.add_api_route(path, media, methods=[method])

    @app.get("/api/v1/state")
    async def state():
        return {"state": "idle"}

    @app.websocket("/ws/camera")
    async def camera(socket: WebSocket):
        calls.append("websocket")
        await socket.accept()
        await socket.send_bytes(b"test-frame")
        await socket.close()

    return TestClient(app), calls


@pytest.mark.parametrize("method,path", MEDIA_REQUESTS)
def test_media_requests_require_the_independent_bearer(method, path, caplog):
    client, calls = media_client()
    missing = client.request(method, path)
    assert missing.status_code == 401
    assert missing.headers["cache-control"] == "no-store"
    assert missing.headers["www-authenticate"] == 'Bearer realm="lingtu-media"'
    wrong = client.request(method, path, headers={"Authorization": "Bearer wrong-token"})
    assert wrong.status_code == 403
    assert calls == []
    accepted = client.request(method, path, headers={"Authorization": f"Bearer {TOKEN}"})
    assert accepted.status_code == 200
    assert calls == ["media"]
    assert TOKEN not in caplog.text


@pytest.mark.parametrize("token", [None, ""])
def test_required_media_without_a_viewer_credential_fails_closed(token):
    client, calls = media_client(token=token)
    response = client.get("/api/v1/camera/snapshot", headers={"Authorization": f"Bearer {TOKEN}"})
    assert response.status_code == 503
    assert response.json() == {"error": "media_access_unconfigured"}
    assert calls == []


def test_cookie_query_and_operator_credentials_do_not_authorize_viewing():
    client, calls = media_client()
    response = client.get(
        "/api/v1/camera/snapshot",
        params={"api_key": TOKEN, "viewer_token": TOKEN},
        headers={"Cookie": f"lingtu_api_key={TOKEN}", "X-API-Key": TOKEN},
    )
    assert response.status_code == 401
    assert calls == []


@pytest.mark.parametrize("headers,code", [({}, 4401), ({"Authorization": "Bearer wrong-token"}, 4403)])
def test_camera_websocket_rejects_unauthorized_viewers(headers, code):
    client, calls = media_client()
    with pytest.raises(WebSocketDisconnect) as error, client.websocket_connect("/ws/camera", headers=headers):
        pass
    assert error.value.code == code
    assert calls == []


def test_camera_websocket_accepts_bearer_but_missing_configuration_fails_closed():
    client, calls = media_client()
    with client.websocket_connect("/ws/camera", headers={"Authorization": f"Bearer {TOKEN}"}) as socket:
        assert socket.receive_bytes() == b"test-frame"
    assert calls == ["websocket"]
    unconfigured, _ = media_client(token=None)
    with pytest.raises(WebSocketDisconnect) as error, unconfigured.websocket_connect("/ws/camera"):
        pass
    assert error.value.code == 4403


def test_policy_disabled_keeps_development_media_and_nonmedia_routes_open():
    client, calls = media_client(required=False)
    assert client.get("/api/v1/camera/snapshot").status_code == 200
    assert client.get("/api/v1/state").status_code == 200
    protected, _ = media_client()
    assert protected.get("/api/v1/state").status_code == 200
    assert calls == ["media"]


def test_gateway_assembly_installs_media_policy_without_reusing_operator_key(monkeypatch):
    monkeypatch.setenv("LINGTU_MEDIA_REQUIRE_TOKEN", "1")
    monkeypatch.delenv("LINGTU_MEDIA_VIEWER_TOKEN", raising=False)
    monkeypatch.setenv("LINGTU_API_KEY", TOKEN)
    gateway = GatewayModule()
    gateway.setup()
    try:
        with TestClient(gateway._app) as client:
            response = client.get("/api/v1/camera/snapshot", headers={"Authorization": f"Bearer {TOKEN}"})
            assert response.status_code == 503
            assert response.json()["error"] == "media_access_unconfigured"
            assert client.get("/api/v1/state").status_code == 200
    finally:
        gateway.stop()


def test_gateway_media_cors_preflight_does_not_require_a_viewer_token(monkeypatch):
    monkeypatch.setenv("LINGTU_MEDIA_REQUIRE_TOKEN", "1")
    monkeypatch.setenv("LINGTU_MEDIA_VIEWER_TOKEN", TOKEN)
    gateway = GatewayModule()
    gateway.setup()
    try:
        with TestClient(gateway._app) as client:
            headers = {"Origin": "http://localhost:5050"}
            preflight = client.options(
                "/api/v1/webrtc/whep",
                headers={**headers, "Access-Control-Request-Method": "POST", "Access-Control-Request-Headers": "authorization,content-type"},
            )
            assert preflight.status_code == 200
            denied = client.get("/api/v1/camera/status", headers=headers)
            assert denied.status_code == 401
            assert denied.headers["access-control-allow-origin"] == "http://localhost:5050"
            accepted = client.get("/api/v1/camera/status", headers={**headers, "Authorization": f"Bearer {TOKEN}"})
            assert accepted.status_code == 200
            assert accepted.headers["access-control-expose-headers"] == "Location, ETag"
    finally:
        gateway.stop()
