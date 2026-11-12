"""Persistent HTTP entry while the Product-owned Gateway starts and stops."""

from __future__ import annotations

import asyncio
import os
from contextlib import asynccontextmanager

import httpx
from fastapi import APIRouter, FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, Response, StreamingResponse
from starlette.background import BackgroundTask
from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosed, InvalidStatus

from gateway.routes.assets import mount_dashboard_assets, register_asset_routes

_HOP_HEADERS = {
    b"connection", b"keep-alive", b"proxy-authenticate", b"proxy-authorization",
    b"te", b"trailer", b"transfer-encoding", b"upgrade",
}


def _request_target(scope) -> str:
    path = scope["raw_path"].decode("ascii")
    query = scope["query_string"].decode("ascii")
    return path + "?" + query if query else path


def build_entry_app(gateway_port: int, *, control_routes: APIRouter) -> FastAPI:
    gateway_url = f"http://127.0.0.1:{gateway_port}"

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        # SSE and downloads remain streaming. Local proxy requests ignore HTTP_PROXY.
        async with httpx.AsyncClient(
            base_url=gateway_url, trust_env=False, timeout=httpx.Timeout(10.0, read=None),
        ) as client:
            app.state.gateway = client
            yield

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None, openapi_url=None)
    app.add_middleware(
        CORSMiddleware,
        allow_origins=os.environ.get("LINGTU_CORS_ORIGINS", "http://localhost:5050,http://127.0.0.1:5050").split(","),
        allow_methods=["*"], allow_headers=["*"], expose_headers=["Location", "ETag"],
    )
    app.include_router(control_routes)
    register_asset_routes(app)

    async def forward(request: Request):
        headers = [
            (key, value) for key, value in request.headers.raw
            if key.lower() not in _HOP_HEADERS | {b"x-forwarded-for", b"x-forwarded-proto"}
        ]
        if request.client:
            headers.append((b"x-forwarded-for", request.client.host.encode("ascii")))
        headers.append((b"x-forwarded-proto", request.url.scheme.encode("ascii")))
        content = request.stream() if "content-length" in request.headers or "transfer-encoding" in request.headers else None
        outgoing = app.state.gateway.build_request(request.method, _request_target(request.scope), headers=headers, content=content)
        try:
            upstream = await app.state.gateway.send(outgoing, stream=True)
        except httpx.HTTPError:
            return JSONResponse(
                {"error": "runtime_unavailable", "message": "运行服务未连接，请查看当前工作模式"},
                status_code=503,
            )
        response = StreamingResponse(
            upstream.aiter_raw(), status_code=upstream.status_code,
            background=BackgroundTask(upstream.aclose),
        )
        response.raw_headers = [
            (key, value) for key, value in upstream.headers.raw if key.lower() not in _HOP_HEADERS
        ]
        return response

    for path in (
        "/api/{path:path}", "/map/{path:path}", "/mcp", "/health", "/ready",
        "/docs", "/docs/{path:path}", "/redoc", "/openapi.json",
    ):
        app.add_api_route(path, forward, methods=["GET", "HEAD", "POST", "PUT", "PATCH", "DELETE", "OPTIONS"])

    @app.websocket("/ws/{path:path}")
    async def forward_socket(socket: WebSocket):
        close_code = 1013
        close_reason = ""
        headers = [
            (key.decode("latin-1"), value.decode("latin-1"))
            for key, value in socket.headers.raw
            if key.lower() not in _HOP_HEADERS | {b"host", b"x-forwarded-for", b"x-forwarded-proto"}
            and not key.lower().startswith(b"sec-websocket-")
        ]
        if socket.client:
            headers.append(("x-forwarded-for", socket.client.host))
        headers.append(("x-forwarded-proto", "https" if socket.url.scheme == "wss" else "http"))
        path = _request_target(socket.scope)
        try:
            async with connect(
                f"ws://127.0.0.1:{gateway_port}{path}", additional_headers=headers,
                subprotocols=socket.scope.get("subprotocols") or None,
                compression=None, max_size=None, max_queue=1, ping_interval=None, proxy=None,
            ) as upstream:
                await socket.accept(subprotocol=upstream.subprotocol)

                async def to_gateway():
                    while True:
                        message = await socket.receive()
                        if message["type"] == "websocket.disconnect":
                            return
                        await upstream.send(message.get("text") if message.get("text") is not None else message["bytes"])

                async def to_client():
                    async for message in upstream:
                        if isinstance(message, bytes):
                            await socket.send_bytes(message)
                        else:
                            await socket.send_text(message)

                tasks = [asyncio.create_task(to_gateway()), asyncio.create_task(to_client())]
                try:
                    await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                finally:
                    for task in tasks:
                        task.cancel()
                    await asyncio.gather(*tasks, return_exceptions=True)
                if upstream.close_code not in {None, 1005, 1006, 1015}:
                    close_code = upstream.close_code
                    close_reason = upstream.close_reason or ""
        except InvalidStatus as exc:
            await socket.send_denial_response(Response(
                bytes(exc.response.body), status_code=exc.response.status_code,
                headers={key: value for key, value in exc.response.headers.raw_items()
                         if key.lower().encode("ascii") not in _HOP_HEADERS},
            ))
        except (OSError, ConnectionClosed, WebSocketDisconnect):
            pass
        finally:
            try:
                await socket.close(code=close_code, reason=close_reason)
            except (RuntimeError, WebSocketDisconnect):
                pass

    mount_dashboard_assets(app)
    return app
