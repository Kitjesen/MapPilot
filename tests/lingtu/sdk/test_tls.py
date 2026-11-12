"""Authenticated HTTPS/WSS with public test-only certificates, valid through 2040."""

from __future__ import annotations

import os
import socket
import ssl
import threading
import time
import urllib.error
from pathlib import Path

import pytest
import uvicorn
from fastapi import FastAPI, Request, WebSocket
from fastapi.responses import Response
from websockets.exceptions import InvalidStatus

from lingtu.sdk import AsyncLingTuClient, LingTuClient


@pytest.mark.parametrize("origin", ["http://localhost:5050", "https://robot.example", "https://[::1]:5050/"])
def test_base_url_preserves_origin(origin):
    assert LingTuClient(base_url=origin).base_url == origin.rstrip("/")
    assert AsyncLingTuClient(base_url=origin)._client.base_url == origin.rstrip("/")
    assert LingTuClient("robot-host", 6060, "key", 2).base_url == "http://robot-host:6060"


@pytest.mark.parametrize("origin", [
    "", "robot.example", "ftp://robot.example", "https:///missing", "https://user:password@robot.example",
    "https://robot.example/prefix", "https://robot.example?key=value", "https://robot.example#fragment",
    "https://localhost:0", "https://localhost:65536", "https://localhost:bad", "https://[::1",
])
def test_base_url_rejects_invalid_origin(origin):
    with pytest.raises(ValueError):
        LingTuClient(base_url=origin)


@pytest.fixture
def tls_gateway(monkeypatch):
    certificates = Path(__file__).parent / "fixtures" / "tls"
    monkeypatch.setenv("SSL_CERT_FILE", str(certificates / "ca.pem"))
    bypass = ",".join(filter(None, [
        os.environ.get("NO_PROXY", os.environ.get("no_proxy", "")),
        "localhost", "127.0.0.1",
    ]))
    monkeypatch.setenv("NO_PROXY", bypass)
    payload = b"\xff\xd8tls-camera\x00\xff\xd9"
    app = FastAPI()

    def authorized(headers):
        return headers.get("x-api-key") == "test-key" and headers.get("authorization") == "Bearer viewer-secret"

    @app.get("/api/v1/camera/snapshot")
    async def snapshot(request: Request):
        return Response(payload, media_type="image/jpeg") if authorized(request.headers) else Response(status_code=403)

    @app.websocket("/ws/camera")
    async def camera(websocket: WebSocket):
        if not authorized(websocket.headers):
            await websocket.close()
            return
        await websocket.accept()
        await websocket.send_bytes(payload)
        await websocket.receive()

    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    port = listener.getsockname()[1]
    server = uvicorn.Server(uvicorn.Config(
        app, log_level="critical", lifespan="off", ws="websockets-sansio",
        ssl_certfile=str(certificates / "server.pem"), ssl_keyfile=str(certificates / "server.pem"),
        timeout_graceful_shutdown=0.1,
    ))
    thread = threading.Thread(target=server.run, kwargs={"sockets": [listener]}, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 3
        while not server.started and thread.is_alive() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert server.started
        yield f"https://127.0.0.1:{port}", payload
    finally:
        server.should_exit = True
        thread.join(timeout=5)
        listener.close()
        assert not thread.is_alive()


def test_https_snapshot_and_authenticated_wss(tls_gateway):
    origin, payload = tls_gateway
    robot = LingTuClient(base_url=origin, api_key="test-key", timeout=2)
    assert robot.observations.camera_snapshot(viewer_token="viewer-secret") == payload
    with robot.watch_camera(viewer_token="viewer-secret") as camera:
        assert next(camera) == payload
    with pytest.raises(urllib.error.HTTPError) as rejected:
        robot.observations.camera_snapshot(viewer_token="wrong-token")
    assert rejected.value.code == 403
    with pytest.raises(InvalidStatus) as rejected:
        with robot.watch_camera(viewer_token="wrong-token"):
            pytest.fail("Invalid media token was accepted")
    assert rejected.value.response.status_code == 403


def test_wss_checks_certificate_hostname(tls_gateway, monkeypatch):
    origin, _ = tls_gateway
    resolve = socket.getaddrinfo

    def local_address(host, *args, **kwargs):
        return resolve("127.0.0.1" if host == "mismatch.example" else host, *args, **kwargs)

    monkeypatch.setattr(socket, "getaddrinfo", local_address)
    monkeypatch.setenv("NO_PROXY", os.environ["NO_PROXY"] + ",mismatch.example")
    robot = LingTuClient(base_url=origin.replace("127.0.0.1", "mismatch.example"), api_key="test-key", timeout=2)
    with pytest.raises(ssl.SSLCertVerificationError):
        with robot.watch_camera(viewer_token="viewer-secret"):
            pytest.fail("Mismatched certificate hostname was accepted")


@pytest.mark.asyncio
async def test_async_https_snapshot_and_wss(tls_gateway):
    origin, payload = tls_gateway
    async with AsyncLingTuClient(base_url=origin, api_key="test-key", timeout=2) as robot:
        assert await robot.observations.camera_snapshot(viewer_token="viewer-secret") == payload
        async with robot.watch_camera(viewer_token="viewer-secret") as camera:
            assert await anext(camera) == payload
