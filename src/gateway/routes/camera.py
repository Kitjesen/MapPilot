"""Camera snapshot routes for GatewayModule."""

from __future__ import annotations

import contextlib
from urllib.parse import urlencode
from typing import Any

import httpx

from fastapi.responses import JSONResponse
from starlette.responses import Response

from gateway.schemas import GatewayErrorResponse
from gateway.services.media_status import build_camera_status
from runtime.registry import auto_select, get


def _error_response(
    error: str,
    *,
    status_code: int,
    message: str | None = None,
    detail: Any = None,
) -> JSONResponse:
    return JSONResponse(
        status_code=status_code,
        content=GatewayErrorResponse(
            error=error,
            message=message or error,
            detail=detail,
        ).model_dump(),
    )


def _cached_gateway_jpeg(gw: Any | None) -> bytes | None:
    if gw is None:
        return None
    lock = getattr(gw, "_jpeg_lock", None)
    cm = lock if lock is not None else contextlib.nullcontext()
    with cm:
        frame = getattr(gw, "_latest_jpeg", None)
    if isinstance(frame, (bytes, bytearray)) and len(frame) > 0:
        return bytes(frame)
    return None


def _teleop_snapshot_jpeg(gw: Any | None) -> bytes | None:
    if gw is None:
        return None
    media = getattr(gw, "_camera_module", None)
    snapshot = getattr(media, "snapshot_jpeg", None)
    if not callable(snapshot):
        return None
    try:
        frame = snapshot()
    except Exception:
        return None
    if isinstance(frame, (bytes, bytearray)) and len(frame) > 0:
        return bytes(frame)
    return None


def _camera_unavailable(gw: Any | None) -> JSONResponse | None:
    if gw is None:
        return None
    status = build_camera_status(gw)
    if status.get("available"):
        return None
    reason = str(status.get("reason") or status.get("status") or "camera_unavailable")
    return _error_response(
        "camera_unavailable",
        status_code=503,
        message=f"Camera snapshot unavailable: {reason}",
        detail={"camera": status},
    )


def _registered_snapshot_adapter_jpeg() -> bytes | None:
    adapter_name = auto_select("camera_snapshot_adapter")
    if not adapter_name:
        return None
    adapter = get("camera_snapshot_adapter", adapter_name)
    capture = getattr(adapter, "capture_jpeg", None)
    if not callable(capture):
        return None
    frame = capture()
    if isinstance(frame, (bytes, bytearray)) and len(frame) > 0:
        return bytes(frame)


def _go2rtc_snapshot_jpeg(gw: Any | None) -> bytes | None:
    """Read one fresh JPEG from the configured sidecar without owning the camera."""
    upstream = str(getattr(gw, "_go2rtc_upstream", "") or "").rstrip("/")
    if not upstream:
        return None
    url = f"{upstream}/api/frame.jpeg?{urlencode({'src': 'cam'})}"
    try:
        response = httpx.get(url, headers={"Accept": "image/jpeg", "Connection": "close"}, timeout=5.0)
        if response.status_code != 200 or not str(response.headers.get("Content-Type", "")).lower().startswith("image/"):
            return None
        data = response.content
        if len(data) > 8 * 1024 * 1024:
            return None
    except Exception:
        return None
    return data if 2 <= len(data) <= 8 * 1024 * 1024 and data[:2] == b"\xff\xd8" else None


def snapshot_jpeg(gw: Any | None) -> Response:
    frame = _teleop_snapshot_jpeg(gw) or _cached_gateway_jpeg(gw)
    if frame:
        return Response(content=frame, media_type="image/jpeg")

    unavailable = _camera_unavailable(gw)
    if unavailable is not None:
        return unavailable

    try:
        data = _registered_snapshot_adapter_jpeg()
        data = data or _go2rtc_snapshot_jpeg(gw)
        if data:
            return Response(content=data, media_type="image/jpeg")
        return _error_response(
            "no_frame",
            status_code=503,
            message="Camera snapshot unavailable: no frame",
        )
    except Exception as e:
        return _error_response(
            "camera_snapshot_failed",
            status_code=500,
            message=str(e),
        )


def register_camera_routes(app, gw=None) -> None:
    @app.get("/api/v1/camera/status", summary="Camera frame availability and rates")
    async def camera_status():
        return build_camera_status(gw)

    @app.get(
        "/api/v1/camera/snapshot",
        summary="Camera JPEG snapshot",
        responses={
            200: {
                "content": {
                    "image/jpeg": {"schema": {"type": "string", "format": "binary"}}
                }
            },
            500: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def camera_snapshot():
        """Return the current JPEG frame without blocking cold-start clients."""
        return snapshot_jpeg(gw)
