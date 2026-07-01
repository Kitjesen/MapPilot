"""Camera snapshot routes for GatewayModule."""

from __future__ import annotations

import contextlib
from typing import Any

from fastapi.responses import JSONResponse
from starlette.responses import Response

from runtime.plugin_seed import seed_registered_plugins
from runtime.registry import auto_select, get
from gateway.schemas import GatewayErrorResponse
from gateway.services.media_status import build_camera_status


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
    teleop = getattr(gw, "_teleop_module", None) if gw is not None else None
    snapshot = getattr(teleop, "snapshot_jpeg", None)
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


def _seed_registered_camera_snapshot_plugins() -> None:
    try:
        seed_registered_plugins(groups=("camera_ros2",), reload_loaded=False)
    except ValueError as exc:
        if "camera_ros2" not in str(exc):
            raise


def _seed_camera_snapshot_plugins() -> None:
    try:
        from lingtu.plugin_seed import seed_builtin_plugins
    except ModuleNotFoundError as exc:
        if exc.name not in {"lingtu", "lingtu.plugin_seed"}:
            raise
        _seed_registered_camera_snapshot_plugins()
        return

    try:
        seed_builtin_plugins(groups=("camera_ros2",), reload_loaded=False)
    except ValueError as exc:
        if "camera_ros2" not in str(exc):
            raise
        _seed_registered_camera_snapshot_plugins()


def _registered_snapshot_adapter_jpeg() -> bytes | None:
    _seed_camera_snapshot_plugins()
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
    return None


def register_camera_routes(app, gw=None) -> None:
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
        frame = _cached_gateway_jpeg(gw) or _teleop_snapshot_jpeg(gw)
        if frame:
            return Response(content=frame, media_type="image/jpeg")

        unavailable = _camera_unavailable(gw)
        if unavailable is not None:
            return unavailable

        try:
            data = _registered_snapshot_adapter_jpeg()
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
