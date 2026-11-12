from __future__ import annotations

import asyncio
import gzip
import socket
import threading
import time

import httpx
import pytest
import uvicorn
from fastapi import APIRouter, FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, Response, StreamingResponse
from fastapi.testclient import TestClient
from starlette.testclient import WebSocketDenialResponse

from gateway.entry import build_entry_app
from lingtu.control_server import ControlOperations, create_server, public_routes
from lingtu.sdk import LingTuClient


class RunningServer:
    def __init__(self, app):
        self.app = app
        self.port = 0

    def start(self):
        self.socket = socket.socket()
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(("127.0.0.1", self.port))
        self.port = self.socket.getsockname()[1]
        self.socket.listen()
        self.server = uvicorn.Server(uvicorn.Config(self.app, log_level="error"))
        self.thread = threading.Thread(target=self.server.run, kwargs={"sockets": [self.socket]})
        self.thread.start()
        deadline = time.monotonic() + 5
        while not self.server.started and self.thread.is_alive() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert self.server.started, "test server did not start"
        return self

    def stop(self):
        self.server.should_exit = True
        self.thread.join(5)
        self.socket.close()
        assert not self.thread.is_alive(), "test server did not stop"


@pytest.fixture
def host():
    app = FastAPI()
    closed = threading.Event()
    release_stream = threading.Event()
    payload = b"point-cloud-data\x00" * 200_000

    @app.get("/api/echo")
    @app.post("/api/echo")
    @app.patch("/api/echo")
    async def echo(request: Request):
        return Response(await request.body(), headers={
            "X-Query": request.url.query,
            "X-Auth": request.headers.get("authorization", ""),
            "X-Host": request.headers["host"],
        })

    @app.get("/api/download")
    async def download():
        response = Response(gzip.compress(payload), headers={"Content-Encoding": "gzip", "ETag": '"cloud"'})
        response.set_cookie("first", "one")
        response.set_cookie("second", "two")
        return response

    @app.get("/map/{name:path}")
    async def map_path(request: Request):
        return Response(request.scope["raw_path"])

    @app.get("/api/events")
    async def events():
        async def chunks():
            yield b"data: first\n\n"
            await asyncio.to_thread(release_stream.wait)
            yield b"data: second\n\n"
        return StreamingResponse(chunks(), media_type="text/event-stream")

    @app.get("/health")
    @app.get("/ready")
    async def health():
        return {"status": "ok"}

    @app.websocket("/ws/teleop")
    async def teleop(ws: WebSocket):
        await ws.accept()
        try:
            while True:
                message = await ws.receive_text()
                if message == "cloud":
                    await ws.send_bytes(payload)
                else:
                    await ws.send_text(message)
        except WebSocketDisconnect:
            pass
        finally:
            closed.set()

    @app.websocket("/ws/camera")
    async def camera(ws: WebSocket):
        await ws.send_denial_response(JSONResponse({"error": "media_authorization_required"}, status_code=401))

    @app.websocket("/ws/close/{code}")
    async def close(ws: WebSocket, code: int):
        await ws.accept()
        await ws.close(code=code, reason="Host closed")

    running = RunningServer(app).start()
    try:
        yield running, payload, closed, release_stream
    finally:
        release_stream.set()
        running.stop()


def test_proxy_preserves_body_headers_and_compressed_download(host):
    running, payload, _, _ = host
    with TestClient(build_entry_app(running.port, control_routes=APIRouter())) as client:
        response = client.patch("/api/echo?map=office&part=2", content=payload, headers={"Authorization": "Bearer test"})
        assert response.content == payload
        assert response.headers["x-query"] == "map=office&part=2"
        assert response.headers["x-auth"] == "Bearer test"
        assert response.headers["x-host"] == "testserver"
        downloaded = client.get("/api/download")
        assert downloaded.content == payload
        assert downloaded.headers["etag"] == '"cloud"'
        assert len(downloaded.headers.get_list("set-cookie")) == 2
        assert client.get("/map/office%2520glass.pcd").content == b"/map/office%2520glass.pcd"
        assert client.get("/openapi.json").json()["info"]["title"] == "FastAPI"
        assert "swagger-ui" in client.get("/docs").text
        assert client.get("/health").json() == {"status": "ok"}
        assert client.get("/ready").json() == {"status": "ok"}


def test_proxy_keeps_sse_streaming(host):
    running, _, _, release = host
    entry = RunningServer(build_entry_app(running.port, control_routes=APIRouter())).start()
    try:
        with httpx.stream("GET", f"http://127.0.0.1:{entry.port}/api/events", timeout=2, trust_env=False) as response:
            lines = response.iter_lines()
            assert next(lines) == "data: first"
            assert not release.is_set()
            release.set()
            assert list(lines) == ["", "data: second", ""]
    finally:
        release.set()
        entry.stop()


def test_proxy_relays_websocket_and_closes_upstream(host):
    running, payload, closed, _ = host
    with TestClient(build_entry_app(running.port, control_routes=APIRouter())) as client:
        with client.websocket_connect("/ws/teleop?client_id=test") as ws:
            ws.send_text("fresh input")
            assert ws.receive_text() == "fresh input"
            ws.send_text("cloud")
            assert ws.receive_bytes() == payload
        assert closed.wait(2), "client disconnect did not reach the Host"
        with pytest.raises(WebSocketDenialResponse) as rejected:
            with client.websocket_connect("/ws/camera"):
                pass
        assert rejected.value.status_code == 401
        assert rejected.value.json() == {"error": "media_authorization_required"}
        for code in [1000, 4409]:
            with client.websocket_connect(f"/ws/close/{code}") as ws:
                with pytest.raises(WebSocketDisconnect) as closed_by_host:
                    ws.receive_text()
                assert closed_by_host.value.code == code
                assert closed_by_host.value.reason == "Host closed"


@pytest.mark.parametrize("restart_product", ["map", "nav"])
def test_public_entry_survives_product_stop_and_restart(host, tmp_path, monkeypatch, restart_product):
    running, _, _, _ = host
    web = tmp_path / "web"
    web.mkdir()
    (web / "index.html").write_text("persistent dashboard", encoding="utf-8")
    monkeypatch.setenv("LINGTU_WEB_DIST", str(web))

    class Control:
        robot = "unitree/go2"
        env = "real"
        product = "map"
        session = "first"

        def status(self, **_kwargs):
            return {"status": "active" if self.product else "stopped", "product": self.product,
                    "product_session_id": self.session}

        def stop(self, **kwargs):
            assert kwargs["expected_product_session_id"] == "first"
            running.stop()
            self.product, self.session = None, ""
            return {"ok": True, "status": "stopped"}

        def switch(self, product, **kwargs):
            assert kwargs["expected_product_session_id"] == ""
            running.start()
            self.product, self.session = product, "second"
            return {"ok": True, "status": "active", "product_session_id": self.session}

    operations = ControlOperations(Control(), tmp_path)
    server = create_server(operations, 0)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    entry = RunningServer(build_entry_app(running.port, control_routes=public_routes(operations))).start()
    url = f"http://127.0.0.1:{entry.port}"
    robot = LingTuClient(base_url=url)
    try:
        with httpx.Client(base_url=url, trust_env=False) as client:
            assert client.get("/api/echo").status_code == 200
            accepted = robot.system.stop_product("first", request_id="stop")
            assert accepted["request_id"] == "stop"
            for request_id in ["stop", "restart"]:
                if request_id == "restart":
                    assert robot.system.switch_product(
                        restart_product, expected_product_session_id="", request_id="restart",
                        map_name="office" if restart_product == "nav" else None,
                    )["request_id"] == "restart"
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    result = robot.system.product_operation(request_id)
                    if result["state"] != "running":
                        break
                    time.sleep(0.01)
                assert result["state"] == "succeeded", result
                assert client.get("/").text == "persistent dashboard"
                status = robot.system.product_status()["current"]
                if request_id == "stop":
                    assert status["status"] == "stopped"
                    assert client.get("/api/echo").status_code == 503
                else:
                    assert status["product_session_id"] == "second"
                    assert client.get("/api/echo").status_code == 200
    finally:
        entry.stop()
        server.shutdown()
        thread.join()
        server.server_close()
        operations.close()
