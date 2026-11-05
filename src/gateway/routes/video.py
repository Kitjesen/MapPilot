"""WebRTC signalling only; the native media endpoint owns video and peers."""

from __future__ import annotations

import json
import time
from asyncio import TimeoutError, wait_for
from pathlib import Path
from typing import Any
from urllib.parse import urljoin, urlsplit, urlunsplit

from fastapi import Request
from fastapi.responses import JSONResponse, Response

from gateway.schemas import Go2RTCStatusResponse, WebRTCStatusResponse
from gateway.services.media_status import build_camera_video_diagnostics


def _native_source_status(path: str) -> dict[str, Any]:
    if not path:
        return {"source_ready": None, "source_status": None}
    try:
        file = Path(path)
        age_s = max(0.0, time.time() - file.stat().st_mtime)
        snapshot = json.loads(file.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {"source_ready": False, "source_status": None, "source_reason": "video_status_unavailable"}
    if not isinstance(snapshot, dict):
        return {"source_ready": False, "source_status": None, "source_reason": "video_status_invalid"}
    ready = age_s <= 3.0 and snapshot.get("ready") is True
    return {
        "source_ready": ready,
        "source_status": snapshot,
        "source_status_age_s": age_s,
        "source_reason": None if ready else "video_status_stale" if age_s > 3.0 else "video_source_not_ready",
    }


def _origin(url: str) -> tuple[str, str]:
    parsed = urlsplit(url)
    return parsed.scheme, parsed.netloc


def _resource_url(endpoint: str, path: str, query: str = "") -> str:
    parsed = urlsplit(endpoint)
    prefix = parsed.path.rsplit("/", 1)[0] + "/"
    resource = urlunsplit((parsed.scheme, parsed.netloc, "/" + path, query, ""))
    normalized = urlsplit(urljoin(endpoint, resource))
    if not normalized.path.startswith(prefix) or any(
        part in {".", ".."} for part in normalized.path.split("/")
    ):
        raise ValueError("resource is outside the configured media endpoint")
    return resource


def _response(upstream: Any, endpoint: str) -> Response:
    headers = {"Cache-Control": "no-store"}
    for name in ("etag", "accept-patch", "link"):
        if name in upstream.headers:
            headers[name] = upstream.headers[name]
    location = upstream.headers.get("location")
    if location:
        resource = urljoin(endpoint, location)
        parsed = urlsplit(resource)
        if _origin(resource) != _origin(endpoint):
            raise ValueError("media endpoint returned a resource on another origin")
        _resource_url(endpoint, parsed.path.lstrip("/"), parsed.query)
        headers["Location"] = "/api/v1/webrtc/sessions/" + parsed.path.lstrip("/")
        if parsed.query:
            headers["Location"] += "?" + parsed.query
    return Response(
        content=upstream.content,
        status_code=upstream.status_code,
        headers=headers,
        media_type=upstream.headers.get("content-type", "application/sdp"),
    )


def register_video_routes(app: Any, gw: Any) -> None:
    try:
        import httpx
    except ImportError:
        httpx = None

    go2rtc = gw._go2rtc_upstream.rstrip("/")
    native_endpoint = getattr(gw, "_webrtc_upstream", "")
    endpoint = native_endpoint or go2rtc + "/api/webrtc"
    backend = "gstreamer" if native_endpoint else "go2rtc"
    ice_servers = getattr(gw, "_webrtc_ice_servers", [])
    rotation = getattr(gw, "_webrtc_rotation", 0)
    status_file = getattr(gw, "_webrtc_status_file", "")

    @app.get(
        "/api/v1/webrtc/go2rtc/status",
        summary="Probe the optional go2rtc endpoint",
        response_model=Go2RTCStatusResponse,
    )
    async def go2rtc_status() -> dict[str, Any]:
        if httpx is None:
            return {"available": False, "reason": "httpx_missing"}
        try:
            async with httpx.AsyncClient(timeout=0.5) as client:
                response = await client.get(go2rtc + "/api/streams")
            if response.status_code != 200:
                return {"available": False, "status": response.status_code}
            data = response.json()
            return {"available": True, "streams": list(data) if isinstance(data, dict) else []}
        except (httpx.HTTPError, ValueError) as error:
            return {"available": False, "reason": type(error).__name__}

    @app.get(
        "/api/v1/webrtc/status",
        summary="Probe the selected media endpoint and camera display configuration",
        response_model=WebRTCStatusResponse,
    )
    async def media_status() -> dict[str, Any]:
        if httpx is None:
            status = {"available": False, "reason": "httpx_missing"}
        elif not native_endpoint:
            status = await go2rtc_status()
        else:
            try:
                async with httpx.AsyncClient(timeout=0.5) as client:
                    response = await client.options(endpoint)
                status = {
                    "available": response.is_success,
                    "streams": ["cam"] if response.is_success else [],
                    "status": response.status_code,
                }
            except httpx.HTTPError as error:
                status = {"available": False, "reason": type(error).__name__}
        if native_endpoint:
            source = _native_source_status(status_file)
            status["whep_available"] = status["available"]
            if source["source_ready"] is False:
                status["available"] = False
                status.setdefault("reason", source["source_reason"])
            status.update(source)
        return {
            **status,
            "backend": backend,
            "ice_servers": ice_servers,
            "rotation_degrees": rotation,
        }

    @app.get("/api/v1/camera/diagnostics", summary="Read camera, video pipeline and WHEP diagnostic observations")
    async def camera_video_diagnostics() -> dict[str, Any]:
        return build_camera_video_diagnostics(gw, await media_status())

    @app.post("/api/v1/webrtc/whep", summary="Negotiate a video receiver with the media endpoint")
    async def offer(request: Request) -> Response:
        if httpx is None:
            return JSONResponse({"error": "httpx_missing"}, status_code=503)
        parameters = None if native_endpoint else {"src": request.query_params.get("src", "cam")}
        try:
            # Native WHEP waits up to 30 seconds for complete ICE gathering.
            async with httpx.AsyncClient(timeout=35.0) as client:
                response = await wait_for(
                    client.post(
                        endpoint,
                        params=parameters,
                        content=await request.body(),
                        headers={"content-type": "application/sdp"},
                    ),
                    timeout=35.0,
                )
            return _response(response, endpoint)
        except (httpx.HTTPError, TimeoutError):
            return JSONResponse({"error": "media_unreachable"}, status_code=503)
        except ValueError:
            return JSONResponse({"error": "invalid_media_resource"}, status_code=502)

    @app.delete("/api/v1/webrtc/sessions/{resource_path:path}", summary="Release a video receiver")
    async def delete_session(resource_path: str, request: Request) -> Response:
        if httpx is None:
            return JSONResponse({"error": "httpx_missing"}, status_code=503)
        try:
            resource = _resource_url(endpoint, resource_path, request.url.query)
        except ValueError:
            return JSONResponse({"error": "invalid_media_resource"}, status_code=400)
        headers = {}
        if "if-match" in request.headers:
            headers["If-Match"] = request.headers["if-match"]
        try:
            async with httpx.AsyncClient(timeout=3.0) as client:
                response = await client.delete(resource, headers=headers)
            return Response(content=response.content, status_code=response.status_code)
        except httpx.HTTPError:
            return JSONResponse({"error": "media_unreachable"}, status_code=503)
