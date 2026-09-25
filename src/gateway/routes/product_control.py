"""Forward lifecycle requests to the Host-independent ProductControl transport."""

from __future__ import annotations

import asyncio
import json
import os
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import ProxyHandler, Request, build_opener

from fastapi import FastAPI
from fastapi import Request as HttpRequest
from fastapi.responses import JSONResponse


def _forward(path: str, body: dict | None = None) -> JSONResponse:
    port = int(os.environ.get("LINGTU_CONTROL_PORT", "5051"))
    request = Request(
        f"http://127.0.0.1:{port}{path}",
        data=json.dumps(body).encode("utf-8") if body is not None else None,
        headers={"Content-Type": "application/json"},
    )
    try:
        # ProductControl is always loopback, including on hosts with an HTTP proxy.
        with build_opener(ProxyHandler({})).open(request, timeout=3) as response:
            return JSONResponse(json.loads(response.read()), status_code=response.status)
    except HTTPError as exc:
        return JSONResponse(json.loads(exc.read()), status_code=exc.code)
    except (URLError, TimeoutError, OSError, ValueError):
        return JSONResponse(
            {"available": False, "message": "模式切换服务未连接，请检查 ProductControl 服务"},
            status_code=503,
        )


def register_product_control_routes(app: FastAPI) -> None:
    @app.get("/api/v1/product-control")
    async def control_status():
        return await asyncio.to_thread(_forward, "/status")

    @app.get("/api/v1/product-control/operations/{request_id}")
    async def control_operation(request_id: str):
        return await asyncio.to_thread(_forward, f"/operations/{quote(request_id, safe='')}")

    @app.post("/api/v1/product-control/switch")
    async def control_switch(request: HttpRequest):
        try:
            body = await request.json()
        except ValueError:
            return JSONResponse({"message": "Invalid JSON request"}, status_code=400)
        if not isinstance(body, dict):
            return JSONResponse({"message": "Expected a JSON object"}, status_code=400)
        return await asyncio.to_thread(_forward, "/switch", body)
