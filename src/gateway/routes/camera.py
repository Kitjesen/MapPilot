"""Camera snapshot routes for GatewayModule."""

from __future__ import annotations

import contextlib
import os
import subprocess
import tempfile
from typing import Any

from fastapi.responses import JSONResponse
from starlette.responses import Response

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

        # Compatibility fallback for standalone route tests or legacy deployments
        # where no Gateway instance was passed into this route module. This is
        # not the product runtime communication boundary.
        out = os.path.join(tempfile.gettempdir(), "lingtu_cam_snap.jpg")
        script = os.path.join(tempfile.gettempdir(), "lingtu_cam_snap.py")
        # Write script to avoid shell escaping issues.
        with open(script, "w") as f:
            f.write(
                (
                    "import rclpy, sys\n"
                    "from sensor_msgs.msg import CompressedImage\n"
                    "rclpy.init()\n"
                    "n=rclpy.create_node('cam_snap')\n"
                    "msg=[None]\n"
                    "n.create_subscription(CompressedImage,"
                    "'/camera/color/image_raw/compressed',"
                    "lambda m:msg.__setitem__(0,m),1)\n"
                    "import time; t=time.time()\n"
                    "while msg[0] is None and time.time()-t<2:"
                    " rclpy.spin_once(n,timeout_sec=0.1)\n"
                    "n.destroy_node(); rclpy.shutdown()\n"
                    f"open('{out}','wb').write(msg[0].data)"
                    " if msg[0] else None\n"
                )
            )
        try:
            # Unset RMW to use default fastrtps (camera uses fastrtps).
            env = os.environ.copy()
            env.pop("RMW_IMPLEMENTATION", None)
            subprocess.run(
                ["bash", "-c", f"source /opt/ros/humble/setup.bash && python3 {script}"],
                capture_output=True,
                timeout=6,
                env=env,
            )
            if os.path.isfile(out) and os.path.getsize(out) > 100:
                with open(out, "rb") as f:
                    data = f.read()
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
