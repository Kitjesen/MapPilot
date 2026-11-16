"""Foreground video acceptance Gateway; owns no Product or device processes."""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import os
import threading
import time
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from starlette.responses import Response

from gateway.gateway_module import GatewayModule
from gateway.media_access import MediaAccessMiddleware
from gateway.routes.camera import register_camera_routes
from gateway.routes.video import register_video_routes
from gateway.services.app_bootstrap import build_app_bootstrap, build_app_capabilities
from gateway.services.readiness import build_readiness_snapshot
from gateway.services.media_status import build_camera_status
from gateway.services.robot_tools import camera_snapshot
from gateway.services.state_snapshot import build_state_snapshot
from runtime.contracts import CAMERA_ROLE


_CAMERA_MCP_TOOLS = [
    {
        "name": "get_camera_status",
        "description": "Read camera freshness and video readiness without changing hardware.",
        "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
        "annotations": {"readOnlyHint": True, "destructiveHint": False, "openWorldHint": False},
    },
    {
        "name": "capture_camera",
        "description": "Capture one fresh JPEG from the configured robot camera.",
        "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
        "annotations": {"readOnlyHint": True, "destructiveHint": False, "openWorldHint": False},
    },
    {
        "name": "capture_depth",
        "description": "Capture a colorized depth preview and depth range from the robot camera.",
        "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
        "annotations": {"readOnlyHint": True, "destructiveHint": False, "openWorldHint": False},
    },
]


def _camera_mcp_ok(req_id: Any, value: Any) -> dict[str, Any]:
    image = None
    display = value
    if isinstance(value, dict):
        display = dict(value)
        image_data_url = display.pop("image_data_url", None)
        if isinstance(image_data_url, str) and image_data_url.startswith("data:image/") and "," in image_data_url:
            header, encoded = image_data_url.split(",", 1)
            mime = header[5:].split(";", 1)[0]
            if mime in {"image/jpeg", "image/png", "image/webp"} and encoded:
                try:
                    base64.b64decode(encoded, validate=True)
                    image = {"type": "image", "data": encoded, "mimeType": mime}
                except ValueError:
                    image = None
    content = [{"type": "text", "text": json.dumps(display, ensure_ascii=False)}]
    if image is not None:
        content.append(image)
    return {"jsonrpc": "2.0", "id": req_id, "result": {"content": content, "structuredContent": display}}


def _camera_mcp_error(req_id: Any, code: int, message: str) -> dict[str, Any]:
    return {"jsonrpc": "2.0", "id": req_id, "error": {"code": code, "message": message}}


def _depth_preview(gateway: GatewayModule) -> dict[str, Any]:
    """Read one depth SHM frame and return a bounded false-color JPEG preview."""
    import base64 as _base64

    import cv2
    import numpy as np

    from runtime.camera_shm import ShmFrameReader, posix_shm_path

    path = os.environ.get("LINGTU_CAMERA_DEPTH_SHM", "/lingtu_camera_depth")
    with ShmFrameReader(posix_shm_path(path), max_age_s=2.0) as reader:
        frame = reader.read_latest()
    if frame is None:
        raise RuntimeError("depth_frame_unavailable")
    if frame.encoding != "16UC1" or frame.width <= 0 or frame.height <= 0:
        raise RuntimeError(f"unsupported_depth_frame:{frame.encoding}")
    values = np.frombuffer(frame.payload, dtype=np.uint16)
    row_stride = max(frame.width, frame.stride // 2)
    if values.size < row_stride * frame.height:
        raise RuntimeError("depth_frame_payload_truncated")
    depth_mm = values[: row_stride * frame.height].reshape(frame.height, row_stride)[:, : frame.width]
    scale = float(frame.depth_scale or 0.001)
    depth_m = depth_mm.astype(np.float32) * scale
    valid = (depth_mm > 0) & np.isfinite(depth_m)
    if not bool(valid.any()):
        raise RuntimeError("depth_frame_has_no_valid_points")
    finite = depth_m[valid]
    near_m = float(np.percentile(finite, 2))
    far_m = float(np.percentile(finite, 98))
    near_m = max(0.1, near_m)
    far_m = max(near_m + 0.1, far_m)
    normalized = np.clip((far_m - depth_m) / (far_m - near_m) * 255.0, 0.0, 255.0).astype(np.uint8)
    preview = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    preview[~valid] = 0
    ok, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 88])
    if not ok:
        raise RuntimeError("depth_preview_encode_failed")
    captured_at_s = float(frame.timestamp_ns) / 1_000_000_000.0
    return {
        "success": True,
        "sensor": "depth",
        "width": int(frame.width),
        "height": int(frame.height),
        "depth_scale_m": scale,
        "near_m": near_m,
        "far_m": far_m,
        "source_timestamp_s": captured_at_s,
        "received_at_s": time.time(),
        "image_data_url": "data:image/jpeg;base64," + _base64.b64encode(encoded.tobytes()).decode("ascii"),
        "color_map": "turbo-near-bright",
    }


def _register_camera_mcp(app: FastAPI, gateway: GatewayModule) -> None:
    """Expose only camera read tools when explicitly enabled on an edge host."""
    if os.environ.get("LINGTU_CAMERA_MCP_ENABLED") != "1":
        return

    @app.post("/mcp")
    async def camera_mcp(request: dict[str, Any]):
        method = request.get("method", "")
        req_id = request.get("id")
        if req_id is None:
            return JSONResponse(status_code=204, content=None)
        if method == "initialize":
            return JSONResponse({
                "jsonrpc": "2.0",
                "id": req_id,
                "result": {
                    "protocolVersion": "2025-11-25",
                    "capabilities": {"tools": {}},
                    "serverInfo": {"name": "lingtu-camera-edge", "version": "1.0.0"},
                },
            })
        if method == "notifications/initialized":
            return JSONResponse(status_code=204, content=None)
        if method == "tools/list":
            return JSONResponse({"jsonrpc": "2.0", "id": req_id, "result": {"tools": _CAMERA_MCP_TOOLS}})
        if method != "tools/call":
            return JSONResponse(_camera_mcp_error(req_id, -32601, f"Unknown method: {method}"))
        params = request.get("params")
        if not isinstance(params, dict) or not isinstance(params.get("name"), str):
            return JSONResponse(_camera_mcp_error(req_id, -32602, "tools/call requires a tool name"))
        if params.get("arguments") not in (None, {}):
            return JSONResponse(_camera_mcp_error(req_id, -32602, "camera tools take no arguments"))
        name = params["name"]
        if name == "get_camera_status":
            return JSONResponse(_camera_mcp_ok(req_id, {"success": True, "status": build_camera_status(gateway)}))
        if name == "capture_camera":
            result = await asyncio.to_thread(camera_snapshot, gateway)
            return JSONResponse(_camera_mcp_ok(req_id, result))
        if name == "capture_depth":
            try:
                result = await asyncio.to_thread(_depth_preview, gateway)
            except Exception as exc:
                result = {"success": False, "sensor": "depth", "error": str(exc)}
            return JSONResponse(_camera_mcp_ok(req_id, result))
        return JSONResponse(_camera_mcp_error(req_id, -32602, f"Unknown camera tool: {name}"))

    @app.get("/mcp/capabilities")
    async def camera_mcp_capabilities():
        return {"server": "lingtu-camera-edge", "tools": [tool["name"] for tool in _CAMERA_MCP_TOOLS], "read_only": True}


class _CameraStatusProbe:
    """Read-only camera health projection for an unmanaged edge gateway."""

    def __init__(self, status_file: str) -> None:
        self._path = Path(status_file)
        self._rate_lock = threading.Lock()
        self._last_rate_sample: tuple[float, int, int] | None = None

    def _rates(self, now: float, color_frames: int, depth_frames: int) -> tuple[float, float]:
        with self._rate_lock:
            previous = self._last_rate_sample
            self._last_rate_sample = (now, color_frames, depth_frames)
        if previous is None:
            return 0.0, 0.0
        elapsed = now - previous[0]
        if elapsed <= 0.0 or color_frames < previous[1] or depth_frames < previous[2]:
            return 0.0, 0.0
        return round((color_frames - previous[1]) / elapsed, 1), round((depth_frames - previous[2]) / elapsed, 1)

    def health(self) -> dict[str, Any]:
        try:
            payload = json.loads(self._path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("camera status is not an object")
            mtime = self._path.stat().st_mtime
        except (OSError, ValueError) as exc:
            return {
                "role": CAMERA_ROLE,
                "backend": "orbbec_native",
                "ports_out": {},
                "error": f"camera_status_unavailable: {exc}",
            }

        age_ms = max(0.0, (time.time() - mtime) * 1000.0)
        color_frames = int(payload.get("color_frames") or 0)
        depth_frames = int(payload.get("depth_frames") or 0)
        info_frames = int(payload.get("info_frames") or 0)
        stale_ms = int(age_ms)
        color_rate, depth_rate = self._rates(time.monotonic(), color_frames, depth_frames)
        return {
            "role": CAMERA_ROLE,
            "backend": "orbbec_native",
            "ports_out": {
                "color_image": {
                    "msg_count": color_frames,
                    "rate_hz": color_rate,
                    "stale_ms": stale_ms,
                },
                "depth_image": {
                    "msg_count": depth_frames,
                    "rate_hz": depth_rate,
                    "stale_ms": stale_ms,
                },
                "camera_info": {
                    "msg_count": info_frames,
                    "rate_hz": 0.0,
                    "stale_ms": stale_ms,
                },
            },
            "camera_info_active_topic": payload.get("info_topic"),
            "camera_info_preferred_topic": payload.get("info_topic"),
            "camera_info_topics": [payload.get("info_topic")]
            if payload.get("info_topic")
            else [],
            "reconnect_count": 0,
            "service_recovery_allowed": False,
            "service_recovery_suppressed": False,
            "status": payload.get("status"),
            "last_error": payload.get("last_error") or None,
        }


def create_app(probe_dir: Path | None = None) -> FastAPI:
    """Expose actual, unstarted Gateway state and the configured media backend."""
    for name in ("LINGTU_PRODUCT", "LINGTU_PRODUCT_SESSION_ID", "LINGTU_RUN_PLAN"):
        if os.environ.get(name):
            raise ValueError(f"video probe requires an unmanaged environment: unset {name}")
    for name in ("LINGTU_ROBOT",):
        if not os.environ.get(name):
            raise ValueError(f"video probe requires {name}")

    # Construction supplies status projection only: never call setup() or start().
    gateway = GatewayModule(product="", product_session_id="")
    app = FastAPI(title="LingTu video acceptance Gateway", docs_url=None, redoc_url=None)
    gateway._app = app
    camera_status_file = os.environ.get("LINGTU_CAMERA_STATUS_FILE", "").strip()
    if camera_status_file:
        gateway._camera_status_probe = _CameraStatusProbe(camera_status_file)
    app.state.gateway = gateway
    app.add_middleware(
        MediaAccessMiddleware,
        required=os.environ.get("LINGTU_MEDIA_REQUIRE_TOKEN") == "1",
        viewer_token=os.environ.get("LINGTU_MEDIA_VIEWER_TOKEN"),
    )
    app.add_middleware(
        CORSMiddleware,
        allow_origins=os.environ.get(
            "LINGTU_CORS_ORIGINS", "http://localhost:5050,http://127.0.0.1:5050"
        ).split(","),
        allow_methods=["GET", "POST", "DELETE", "OPTIONS"],
        allow_headers=["*"],
        expose_headers=["Location", "ETag"],
    )

    def snapshot(payload: dict) -> dict:
        paths = {route.path for route in app.routes}
        payload["links"] = {
            name: path for name, path in payload["links"].items() if path in paths
        }
        flag_key = "features" if "features" in payload else "capabilities"
        if flag_key in payload:
            payload[flag_key] = {
                name: name in {"state", "whep"} for name in payload[flag_key]
            }
        if "endpoints" in payload:
            payload["endpoints"] = {
                group: {name: spec for name, spec in specs.items() if spec["path"] in paths}
                for group, specs in payload["endpoints"].items()
            }
            payload["probes"] = {}
            payload["realtime"] = {}
            payload["validation_gates"] = {}
        # These standard snapshots contain links to streams absent from this app.
        if "media" in payload:
            payload["media"] = {
                name: value for name, value in payload["media"].items()
                if not isinstance(value, str) or value in paths
            }
        if "traffic" in payload:
            payload.pop("traffic")
        if "robot" in payload:
            payload["robot"]["online"] = None
        return payload

    @app.get("/api/v1/app/bootstrap")
    async def bootstrap():
        return snapshot(await asyncio.to_thread(build_app_bootstrap, gateway))

    @app.get("/api/v1/app/capabilities")
    async def capabilities():
        return snapshot(await asyncio.to_thread(build_app_capabilities, gateway))

    @app.get("/api/v1/state")
    async def state():
        return snapshot(await asyncio.to_thread(build_state_snapshot, gateway))

    @app.get("/api/v1/readiness")
    async def readiness():
        payload, _ = build_readiness_snapshot(gateway)
        return payload

    register_camera_routes(app, gateway)
    register_video_routes(app, gateway)
    _register_camera_mcp(app, gateway)

    @app.get("/api/v1/camera/depth/snapshot", summary="Colorized depth preview")
    async def depth_snapshot():
        try:
            payload = await asyncio.to_thread(_depth_preview, gateway)
            header, encoded = payload["image_data_url"].split(",", 1)
            import base64 as _base64
            return Response(
                content=_base64.b64decode(encoded),
                media_type="image/jpeg",
                headers={
                    "Cache-Control": "no-store",
                    "X-Depth-Near-M": f"{payload['near_m']:.3f}",
                    "X-Depth-Far-M": f"{payload['far_m']:.3f}",
                    "X-Depth-Width": str(payload["width"]),
                    "X-Depth-Height": str(payload["height"]),
                },
            )
        except Exception as exc:
            return JSONResponse({"success": False, "sensor": "depth", "error": str(exc)}, status_code=503)
    if probe_dir is not None:
        assets = {name: probe_dir / name for name in (
            "index.html", "probe.mjs", "probe-core.mjs", "whepReceiver.mjs",
        )}
        if not all(path.is_file() for path in assets.values()):
            raise ValueError("video probe directory must contain the four declared assets")

        @app.get("/video-probe/")
        async def probe_index():
            return FileResponse(assets["index.html"], media_type="text/html")

        @app.get("/video-probe/{asset}")
        async def probe_asset(asset: str):
            if asset not in assets:
                raise HTTPException(status_code=404)
            return FileResponse(
                assets[asset], media_type="text/html" if asset == "index.html" else "text/javascript"
            )

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=5051, type=int)
    parser.add_argument("--probe-dir", type=Path)
    args = parser.parse_args()
    import uvicorn

    uvicorn.run(create_app(args.probe_dir), host=args.host, port=args.port)


if __name__ == "__main__":
    main()
