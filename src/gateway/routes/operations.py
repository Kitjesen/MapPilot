"""Operational Gateway routes.

These endpoints talk to optional services or subprocess-backed capabilities.
Keeping them out of GatewayModule makes the runtime shell smaller without
changing the external API shape.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Any

from core.runtime_policy import (
    is_supported_slam_profile,
    normalize_slam_profile,
    slam_switch_plan,
)
from gateway.services.map_paths import map_dir_for
from gateway.services.map_safety import safe_map_name
from gateway.schemas import (
    BagStartRequest,
    BagOperationResponse,
    BagStatusResponse,
    BitrateRequest,
    ExplorationCommandResponse,
    ExplorationStatusResponse,
    GatewayErrorResponse,
    Go2RTCStatusResponse,
    SlamOperationResponse,
    SlamRelocalizeRequest,
    SlamStatusResponse,
    SlamSwitchRequest,
    TemporalSemanticRequest,
    TemporalMemoryResponse,
    WebRTCOfferRequest,
    WebRTCControlResponse,
    WebRTCStatsResponse,
)

logger = logging.getLogger(__name__)

_ACTIVE_SERVICE_STATES = {"running", "active"}
_SLAM_STATUS_SERVICES = (
    "lidar",
    "slam",
    "slam_pgo",
    "localizer",
    "super_lio",
    "super_lio_relocation",
)
_EXPLORER_UNAVAILABLE_DETAIL = {
    "reason": "explorer_backend_not_running",
    "required_profile": "explore_or_tare_explore",
    "supported_profiles": ["explore", "tare_explore"],
    "action": (
        "restart LingTu with the explore or tare_explore profile before "
        "starting exploration"
    ),
}

try:
    from fastapi import Request as FastAPIRequest
except ImportError:  # FastAPI remains optional until routes are registered.
    FastAPIRequest = Any


def _parse_since(since: str) -> float:
    """Parse a human-readable duration into a Unix timestamp."""
    import re as _re

    now = time.time()
    m = _re.match(
        r"^(\d+(?:\.\d+)?)\s*(s|sec|m|min|h|hour|d|day)?",
        since.strip().lower(),
    )
    if m:
        value = float(m.group(1))
        unit = m.group(2) or "s"
        if unit in ("h", "hour"):
            return now - value * 3600
        if unit in ("m", "min"):
            return now - value * 60
        if unit in ("d", "day"):
            return now - value * 86400
        return now - value
    try:
        return now - float(since)
    except ValueError:
        return now - 3600


def _normalize_slam_profile(profile: Any) -> str:
    return normalize_slam_profile(profile)


def _body_mapping(body: Any) -> dict[str, Any]:
    """Normalise request body to a plain dict.

    Why raw dicts are accepted:
      Pydantic models define fixed fields, but ROS2 frontends / WebSocket
      messages send JSON with backend-variant keys (e.g. "map_name" vs.
      "map", "slam_profile" vs. "slam_backend"). Accepting raw dicts avoids
      per-endpoint model proliferation and keeps route handlers flexible.

    Trade-off:
      Pydantic type coercion (float, int, str trim) is bypassed at the
      boundary. Call sites MUST manually coerce numeric fields with
      float() / int() — see slam_relocalize (x, y, yaw) and bag_start
      (duration) for examples.
    """
    if hasattr(body, "model_dump"):
        return body.model_dump(exclude_none=True)
    assert isinstance(body, dict), f"expected dict or Pydantic model, got {type(body).__name__}"
    return body


def slam_operation_payload(success: bool, **fields: Any) -> dict[str, Any]:
    payload = {
        "schema_version": 1,
        "ok": bool(success),
        "success": bool(success),
        "ts": time.time(),
    }
    payload.update({key: value for key, value in fields.items() if value is not None})
    return payload


def _slam_operation_response(
    success: bool,
    *,
    status_code: int,
    **fields: Any,
) -> Any:
    from fastapi.responses import JSONResponse

    return JSONResponse(
        slam_operation_payload(success, **fields),
        status_code=status_code,
    )


def _unsupported_saved_map_relocalization_response(gw) -> Any | None:

    from gateway.services.runtime_status import build_localization_status

    try:
        status = build_localization_status(gw)
    except Exception:
        status = {}

    raw = status.get("raw") if isinstance(status.get("raw"), dict) else {}
    backend = (
        raw.get("localization_backend")
        or raw.get("backend")
        or status.get("localization_backend")
        or status.get("backend")
    )
    backend_name = str(backend or "").strip().lower()
    saved_map_supported = raw.get(
        "saved_map_relocalization_supported",
        status.get("saved_map_relocalization_supported"),
    )
    if (
        backend_name not in {"super_lio", "super_lio_relocation"}
        or saved_map_supported is not False
    ):
        return None

    recovery_method = status.get("recovery_method") or raw.get("recovery_method")
    recovery_hint = (
        f"; recovery_method={recovery_method}" if recovery_method else ""
    )
    return _slam_operation_response(
        False,
        message=(
            "unsupported: saved map relocalization is not supported by "
            f"{backend_name}{recovery_hint}"
        ),
        status_code=409,
    )


def _relocalization_service_unavailable_response(gw) -> Any | None:
    service = getattr(gw, "_relocalization_service", None)
    if service is not None:
        return None
    return _slam_operation_response(
        False,
        message="relocalization service unavailable",
        status_code=503,
    )


def register_operation_routes(app, gw) -> None:
    from fastapi import Body
    from fastapi.responses import JSONResponse, Response

    @app.get(
        "/api/v1/webrtc/stats",
        summary="WebRTC peer telemetry (bitrate, fps, encode time)",
        response_model=WebRTCStatsResponse,
        responses={500: {"model": WebRTCStatsResponse}},
    )
    async def get_webrtc_stats():
        if gw._webrtc is None:
            return {"enabled": False, "active_peers": 0}
        try:
            return await gw._webrtc.collect_stats()
        except Exception as e:
            logger.debug("webrtc stats error: %s", e)
            return JSONResponse(
                {"enabled": True, "error": str(e)},
                status_code=500,
            )

    go2rtc_upstream = gw._go2rtc_upstream

    @app.get(
        "/api/v1/webrtc/go2rtc/status",
        summary="Probe the go2rtc sidecar (image transmission fast path)",
        response_model=Go2RTCStatusResponse,
    )
    async def get_go2rtc_status():
        try:
            import httpx
        except ImportError:
            return {"available": False, "reason": "httpx_missing"}
        try:
            async with httpx.AsyncClient(timeout=0.5) as client:
                r = await client.get(f"{go2rtc_upstream}/api/streams")
                if r.status_code != 200:
                    return {"available": False, "status": r.status_code}
                data = r.json()
                streams = list(data.keys()) if isinstance(data, dict) else []
                return {"available": True, "streams": streams}
        except Exception as e:
            return {"available": False, "reason": type(e).__name__}

    @app.post(
        "/api/v1/webrtc/whep",
        summary="WHEP signalling proxy to go2rtc (image transmission path)",
        responses={
            200: {
                "content": {"application/sdp": {"schema": {"type": "string"}}}
            },
            503: {"model": GatewayErrorResponse},
        },
    )
    async def post_webrtc_whep(request: FastAPIRequest):
        try:
            import httpx
        except ImportError:
            return JSONResponse({"error": "httpx_missing"}, status_code=503)
        src = request.query_params.get("src", "cam")
        body = await request.body()
        try:
            async with httpx.AsyncClient(timeout=3.0) as client:
                r = await client.post(
                    f"{go2rtc_upstream}/api/webrtc?src={src}",
                    content=body,
                    headers={"content-type": "application/sdp"},
                )
        except Exception as e:
            logger.info("go2rtc unreachable: %s", e)
            return JSONResponse({"error": "go2rtc_unreachable"}, status_code=503)
        return Response(
            content=r.content,
            status_code=r.status_code,
            media_type=r.headers.get("content-type", "application/sdp"),
        )

    @app.post(
        "/api/v1/webrtc/offer",
        summary="WebRTC SDP offer/answer exchange for low-latency camera",
        response_model=WebRTCControlResponse,
        responses={
            400: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def post_webrtc_offer(body: WebRTCOfferRequest = Body(...)):
        if gw._webrtc is None:
            return JSONResponse({"error": "webrtc_unavailable"}, status_code=503)
        try:
            return await gw._webrtc.handle_offer(_body_mapping(body))
        except ValueError as e:
            return JSONResponse({"error": str(e)}, status_code=400)

    @app.post(
        "/api/v1/webrtc/bitrate",
        summary="Live-tune WebRTC max bitrate without reconnect",
        response_model=WebRTCControlResponse,
        responses={
            400: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def post_webrtc_bitrate(body: BitrateRequest):
        if gw._webrtc is None:
            return JSONResponse({"error": "webrtc_unavailable"}, status_code=503)
        try:
            return await gw._webrtc.set_max_bitrate(body.bps)
        except ValueError as e:
            return JSONResponse({"error": str(e)}, status_code=400)

    def _temporal_store():
        if gw._temporal_store is None:
            try:
                import os

                from memory.storage.temporal_store import TemporalStore

                mem_dir = os.environ.get(
                    "LINGTU_MEMORY_DIR",
                    os.path.join(os.path.expanduser("~"), ".nova", "semantic"),
                )
                gw._temporal_store = TemporalStore(
                    os.path.join(mem_dir, "temporal_memory.db")
                )
            except Exception as exc:
                logger.warning("GatewayModule: TemporalStore unavailable: %s", exc)
        return gw._temporal_store

    @app.get(
        "/api/v1/memory/temporal",
        summary="Query temporal entity observations",
        response_model=TemporalMemoryResponse,
        responses={503: {"model": GatewayErrorResponse}},
    )
    async def get_temporal_memory(
        label: str | None = None,
        since: str | None = None,
        near_x: float | None = None,
        near_y: float | None = None,
        radius: float | None = None,
        limit: int = 100,
    ):
        since_ts = _parse_since(since) if since else None
        store = _temporal_store()
        if store is None:
            return JSONResponse(
                status_code=503,
                content={
                    "error": "temporal_store_unavailable",
                    "detail": "TemporalMemoryModule not running or save_dir not set",
                },
            )
        loop = asyncio.get_event_loop()
        rows = await loop.run_in_executor(
            None,
            lambda: store.query(
                label=label,
                since_ts=since_ts,
                near_x=near_x,
                near_y=near_y,
                radius=radius,
                limit=max(1, min(limit, 1000)),
            ),
        )
        return {"observations": rows, "count": len(rows)}

    @app.post(
        "/api/v1/memory/temporal/semantic",
        summary="Semantic similarity search over temporal observations",
        response_model=TemporalMemoryResponse,
        responses={
            422: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def post_temporal_semantic(body: TemporalSemanticRequest):
        payload = _body_mapping(body)
        raw_emb = payload.get("embedding")
        if not raw_emb:
            return JSONResponse(
                status_code=422,
                content={"error": "embedding required"},
            )
        try:
            query_vec = [float(value) for value in raw_emb]
        except Exception as exc:
            return JSONResponse(
                status_code=422,
                content={"error": f"invalid embedding: {exc}"},
            )

        since_ts = _parse_since(payload["since"]) if payload.get("since") else None
        store = _temporal_store()
        if store is None:
            return JSONResponse(
                status_code=503,
                content={"error": "temporal_store_unavailable"},
            )

        loop = asyncio.get_event_loop()
        rows = await loop.run_in_executor(
            None,
            lambda: store.query_semantic(
                query_vec,
                top_k=int(payload.get("top_k", 10)),
                since_ts=since_ts,
                label=payload.get("label") or None,
            ),
        )
        return {"observations": rows, "count": len(rows)}

    @app.post(
        "/api/v1/explore/start",
        summary="Start autonomous frontier exploration",
        response_model=ExplorationCommandResponse,
        responses={
            409: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def explore_start():
        if not gw._explorer_available():
            return JSONResponse(
                status_code=503,
                content={
                    "schema_version": 1,
                    "ok": False,
                    "error": "Exploration backend not running",
                    "message": "Exploration is unavailable in the current runtime profile.",
                    "detail": dict(_EXPLORER_UNAVAILABLE_DETAIL),
                },
            )
        readiness = gw._exploration_start_readiness()
        if not readiness.get("can_start", False):
            blockers = readiness.get("blockers") or ["navigation_not_ready"]
            return JSONResponse(
                status_code=409,
                content={
                    "schema_version": 1,
                    "ok": False,
                    "error": "exploration_not_ready",
                    "message": (
                        "Exploration cannot start until navigation readiness "
                        f"blockers clear: {', '.join(map(str, blockers))}"
                    ),
                    "detail": readiness,
                },
            )
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(None, gw._begin_exploration)
        gw._exploring = True
        gw.push_event({"type": "exploring", "active": True})
        return {"status": result}

    @app.post(
        "/api/v1/explore/stop",
        summary="Stop autonomous frontier exploration",
        response_model=ExplorationCommandResponse,
        responses={503: {"model": GatewayErrorResponse}},
    )
    async def explore_stop():
        if not gw._explorer_available():
            return JSONResponse(
                status_code=503,
                content={
                    "schema_version": 1,
                    "ok": False,
                    "error": "Exploration backend not running",
                    "message": "Exploration is unavailable in the current runtime profile.",
                    "detail": dict(_EXPLORER_UNAVAILABLE_DETAIL),
                },
            )
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(None, gw._end_exploration)
        gw._exploring = False
        gw.push_event({"type": "exploring", "active": False})
        return {"status": result}

    @app.get(
        "/api/v1/explore/status",
        summary="Exploration status",
        response_model=ExplorationStatusResponse,
    )
    async def explore_status():
        return gw._exploration_status_payload()

    @app.get(
        "/api/v1/slam/status",
        summary="SLAM service status",
        response_model=SlamStatusResponse,
    )
    async def slam_status():
        try:
            from core.service_manager import get_service_manager

            svc = get_service_manager()
            services = svc.status(*_SLAM_STATUS_SERVICES)
        except Exception:
            services = {
                "lidar": "unknown",
                "slam": "unknown",
                "slam_pgo": "unknown",
                "localizer": "unknown",
                "super_lio": "unknown",
                "super_lio_relocation": "unknown",
            }

        if services.get("super_lio_relocation") in _ACTIVE_SERVICE_STATES:
            mode = "super_lio_relocation"
        elif services.get("super_lio") in _ACTIVE_SERVICE_STATES:
            mode = "super_lio"
        elif services.get("slam_pgo") in _ACTIVE_SERVICE_STATES:
            mode = "fastlio2"
        elif services.get("localizer") in _ACTIVE_SERVICE_STATES:
            mode = "localizer"
        elif services.get("slam") in _ACTIVE_SERVICE_STATES:
            mode = "fastlio2"
        else:
            mode = "stopped"
        return {"mode": mode, "services": services}

    @app.post(
        "/api/v1/slam/switch",
        summary="Hot-switch SLAM profile",
        response_model=SlamOperationResponse,
        responses={
            400: {"model": SlamOperationResponse},
            500: {"model": SlamOperationResponse},
        },
    )
    async def slam_switch(body: SlamSwitchRequest):
        payload = _body_mapping(body)
        requested_profile = payload.get("profile", "")
        profile = _normalize_slam_profile(requested_profile)
        if not is_supported_slam_profile(profile, allow_stop=True):
            return _slam_operation_response(
                False,
                message=f"Unknown profile: {requested_profile}",
                status_code=400,
            )
        try:
            from core.service_manager import get_service_manager

            svc = get_service_manager()
            plan = slam_switch_plan(profile)
            svc.stop(*plan.stop)
            if plan.ensure:
                svc.ensure(*plan.ensure)
            ok = (
                svc.wait_ready(*plan.wait_ready, timeout=10.0)
                if plan.wait_ready
                else True
            )
            if ok:
                gw._cached_slam_profile = "stopped" if profile == "stop" else profile
                gw._slam_profile_ts = time.time()
            return slam_operation_payload(
                ok,
                profile=profile,
                message=(
                    f"Switched to {profile}" if ok else "Services not ready after 10s"
                ),
            )
        except Exception as e:
            return _slam_operation_response(False, message=str(e), status_code=500)

    @app.post(
        "/api/v1/slam/auto_relocalize",
        summary="Global relocalize via 3D-BBS (no guess required)",
        response_model=SlamOperationResponse,
        responses={
            409: {"model": SlamOperationResponse},
            500: {"model": SlamOperationResponse},
            503: {"model": SlamOperationResponse},
            504: {"model": SlamOperationResponse},
        },
    )
    async def slam_auto_relocalize():
        unsupported_response = _unsupported_saved_map_relocalization_response(gw)
        if unsupported_response is not None:
            return unsupported_response
        unavailable_response = _relocalization_service_unavailable_response(gw)
        if unavailable_response is not None:
            return unavailable_response

        try:
            result = gw._relocalization_service.trigger_global_relocalize(timeout_s=10.0)
            if result.timed_out:
                return _slam_operation_response(
                    False,
                    message=result.message,
                    status_code=504,
                )
            return slam_operation_payload(result.success, message=result.message)
        except Exception as e:
            return _slam_operation_response(False, message=str(e), status_code=500)

    @app.post(
        "/api/v1/slam/relocalize",
        summary="Relocalize against a saved map",
        response_model=SlamOperationResponse,
        responses={
            400: {"model": SlamOperationResponse},
            404: {"model": SlamOperationResponse},
            409: {"model": SlamOperationResponse},
            500: {"model": SlamOperationResponse},
            503: {"model": SlamOperationResponse},
            504: {"model": SlamOperationResponse},
        },
    )
    async def slam_relocalize(body: SlamRelocalizeRequest):
        unsupported_response = _unsupported_saved_map_relocalization_response(gw)
        if unsupported_response is not None:
            return unsupported_response

        payload = _body_mapping(body)
        map_name = payload.get("map_name", "")
        x = float(payload.get("x", 0.0))
        y = float(payload.get("y", 0.0))
        yaw = float(payload.get("yaw", 0.0))
        if not map_name:
            return _slam_operation_response(
                False,
                message="map_name required",
                status_code=400,
            )
        name_error = safe_map_name(map_name)
        if name_error is not None:
            return _slam_operation_response(
                False,
                message=name_error,
                status_code=400,
            )
        pcd_path = map_dir_for(map_name) / "map.pcd"
        if not pcd_path.is_file():
            return _slam_operation_response(
                False,
                message=f"Map not found: {pcd_path}",
                status_code=404,
            )
        unavailable_response = _relocalization_service_unavailable_response(gw)
        if unavailable_response is not None:
            return unavailable_response
        try:
            result = gw._relocalization_service.relocalize_saved_map(
                pcd_path,
                x,
                y,
                yaw,
                timeout_s=30.0,
            )
            if result.timed_out:
                return _slam_operation_response(
                    False,
                    message=result.message,
                    status_code=504,
                )
            ok = result.success
            quality = result.quality
            if quality is None and ok:
                quality = float(getattr(gw, "_icp_quality", 0.0))
            if ok:
                gw._persist_last_nav_pose(map_name, x, y, yaw, quality)
            msg = result.message if not ok else f"Relocalized to {map_name}"
            return slam_operation_payload(ok, message=msg, quality=quality)
        except Exception as e:
            return _slam_operation_response(False, message=str(e), status_code=500)

    @app.post(
        "/api/v1/bag/start",
        summary="Start rosbag recording",
        response_model=BagOperationResponse,
        responses={
            409: {"model": BagOperationResponse},
            500: {"model": BagOperationResponse},
        },
    )
    async def bag_start(body: BagStartRequest = BagStartRequest()):
        import os
        import pathlib
        import subprocess

        payload = _body_mapping(body)
        duration = int(payload.get("duration", 600))
        prefix = str(payload.get("prefix", "web"))[:40]
        prefix = "".join(c for c in prefix if c.isalnum() or c in "-_") or "web"

        with gw._bag_lock:
            if gw._bag_proc is not None and gw._bag_proc.poll() is None:
                return JSONResponse(
                    status_code=409,
                    content={
                        "error": "recording_in_progress",
                        "path": gw._bag_path,
                        "pid": gw._bag_proc.pid,
                    },
                )

            repo_root = pathlib.Path(__file__).resolve().parents[3]
            script = repo_root / "scripts" / "record_bag.sh"
            if not script.exists():
                return JSONResponse(
                    status_code=500,
                    content={"error": "script_not_found", "path": str(script)},
                )

            stamp = time.strftime("%Y%m%d_%H%M%S")
            bag_dir = os.path.expanduser(f"~/data/bags/{prefix}_{stamp}")
            try:
                proc = subprocess.Popen(
                    ["bash", str(script), str(duration), prefix],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
            except FileNotFoundError as e:
                return JSONResponse(
                    status_code=500,
                    content={"error": "bash_not_found", "detail": str(e)},
                )

            gw._bag_proc = proc
            gw._bag_path = bag_dir
            gw._bag_started_ts = time.time()
            return {
                "status": "started",
                "path": bag_dir,
                "pid": proc.pid,
                "duration": duration,
                "prefix": prefix,
            }

    @app.post(
        "/api/v1/bag/stop",
        summary="Stop rosbag recording",
        response_model=BagOperationResponse,
        responses={404: {"model": BagOperationResponse}},
    )
    async def bag_stop():
        import os
        import signal

        with gw._bag_lock:
            proc = gw._bag_proc
            if proc is None or proc.poll() is not None:
                return JSONResponse(
                    status_code=404,
                    content={"error": "not_recording"},
                )
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except (ProcessLookupError, PermissionError, AttributeError):
                try:
                    proc.terminate()
                except OSError:
                    pass
            return {"status": "stopping", "path": gw._bag_path, "pid": proc.pid}

    @app.get(
        "/api/v1/bag/status",
        summary="rosbag recording status",
        response_model=BagStatusResponse,
    )
    async def bag_status():
        import os
        import shutil

        with gw._bag_lock:
            proc = gw._bag_proc
            path = gw._bag_path
            started_ts = gw._bag_started_ts

        recording = proc is not None and proc.poll() is None
        size_bytes = 0
        if path and os.path.isdir(path):
            try:
                for root, _dirs, files in os.walk(path):
                    for filename in files:
                        fp = os.path.join(root, filename)
                        try:
                            size_bytes += os.path.getsize(fp)
                        except OSError:
                            pass
            except OSError:
                pass

        bag_disk = os.path.expanduser("~/data")
        if not os.path.isdir(bag_disk):
            bag_disk = os.path.expanduser("~")
        try:
            du = shutil.disk_usage(bag_disk)
            disk_free, disk_total = du.free, du.total
        except OSError:
            disk_free, disk_total = 0, 0
        return {
            "recording": recording,
            "path": path,
            "duration_s": (time.time() - started_ts) if started_ts else 0.0,
            "size_bytes": size_bytes,
            "pid": proc.pid if proc else None,
            "exit_code": (
                proc.returncode if proc and proc.poll() is not None else None
            ),
            "disk_free": disk_free,
            "disk_total": disk_total,
        }
