"""Operational Gateway routes.

These endpoints talk to optional services or subprocess-backed capabilities.
Keeping them out of GatewayModule makes the runtime shell smaller without
changing the external API shape.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import time
from pathlib import Path
from typing import Any

from gateway.schemas import (
    BagOperationResponse,
    BagStartRequest,
    BagStatusResponse,
    ExplorationCommandResponse,
    ExplorationStatusResponse,
    GatewayErrorResponse,
    Go2RTCStatusResponse,
    ServiceStatusResponse,
    SlamOperationResponse,
    SlamRelocalizeRequest,
    SlamStatusResponse,
    SlamSwitchRequest,
    TemporalMemoryResponse,
    TemporalSemanticRequest,
)
from gateway.services.map_service import ensure_maps_service
from maps.services.storage import safe_map_name
from runtime.runtime_policy import (
    is_supported_slam_profile,
    normalize_slam_profile,
    slam_switch_plan,
)
from runtime.service_catalogs.thunder import (
    thunder_field_readiness_services,
    thunder_service_groups,
    thunder_service_metadata,
    thunder_slam_status_services,
)

logger = logging.getLogger(__name__)

_ACTIVE_SERVICE_STATES = {"running", "active"}
_SERVICE_STATUS_DEFAULTS = tuple(dict.fromkeys((*thunder_field_readiness_services(), "gateway")))
_FIELD_SERVICE_READINESS_DEFAULT_PATH = "/tmp/lingtu_service_readiness.json"
_SLAM_STATUS_SERVICES = thunder_slam_status_services()
_SLAM_SERVICE_GROUPS = thunder_service_groups()
_SLAM_SERVICE_METADATA = thunder_service_metadata()
_EXPLORER_UNAVAILABLE_DETAIL = {
    "reason": "explorer_backend_not_running",
    "required_profile": "explore_or_tare_explore",
    "supported_profiles": ["explore", "tare_explore"],
    "action": ("restart LingTu with the explore or tare_explore profile before starting exploration"),
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


def _parse_service_names(names: str | None) -> tuple[str, ...]:
    if not names:
        return _SERVICE_STATUS_DEFAULTS
    parsed = tuple(item.strip() for item in names.split(",") if item.strip())
    return parsed or _SERVICE_STATUS_DEFAULTS


def _mark_gateway_http_observed(service_details: dict[str, Any]) -> None:
    """The current route response proves the Gateway HTTP surface is alive."""
    for name in ("gateway", "lingtu"):
        detail = service_details.get(name)
        if not isinstance(detail, dict):
            continue
        contract = detail.get("contract")
        checks = contract.get("checks") if isinstance(contract, dict) else []
        if "http" not in checks:
            continue
        observed = detail.setdefault("observed", {})
        if isinstance(observed, dict):
            observed["http"] = {
                "ok": True,
                "checked": True,
                "enabled": True,
                "source": "current_gateway_route",
                "path": "/api/v1/services/status",
                "blockers": [],
            }
        blockers = [blocker for blocker in list(detail.get("blockers") or []) if blocker != "http_unchecked"]
        detail["blockers"] = blockers
        detail["ready"] = not blockers


def _service_readiness_summary(
    services: dict[str, str],
    service_details: dict[str, Any],
    selected: tuple[str, ...],
    field_readiness: dict[str, Any] | None = None,
) -> dict[str, Any]:
    blockers: list[str] = []
    ready_services: list[str] = []
    not_ready_services: list[str] = []

    for name in selected:
        detail = service_details.get(name)
        if isinstance(detail, dict):
            ready = bool(detail.get("ready", False))
            detail_blockers = list(detail.get("blockers") or [])
            if ready:
                ready_services.append(name)
                continue
            not_ready_services.append(name)
            if detail_blockers:
                blockers.extend(f"{name}:{blocker}" for blocker in detail_blockers)
            else:
                blockers.append(f"{name}:not_ready")
            continue

        status = services.get(name, "unknown")
        if status in _ACTIVE_SERVICE_STATES:
            ready_services.append(name)
        else:
            not_ready_services.append(name)
            blockers.append(f"{name}:status_{status}")

    if field_readiness and field_readiness.get("required"):
        blockers.extend(f"field_readiness:{blocker}" for blocker in field_readiness.get("blockers", []))

    return {
        "ok": not blockers,
        "ready": not blockers,
        "selected": list(selected),
        "ready_services": ready_services,
        "not_ready_services": not_ready_services,
        "blockers": blockers,
        "blocker_count": len(blockers),
    }


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None or value.strip() == "":
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _load_field_service_readiness(now: float | None = None) -> dict[str, Any]:
    """Load optional field collector evidence written by the read-only gate."""
    path_text = os.environ.get(
        "LINGTU_SERVICE_READINESS_JSON",
        _FIELD_SERVICE_READINESS_DEFAULT_PATH,
    ).strip()
    required = _env_bool("LINGTU_REQUIRE_FIELD_SERVICE_READINESS", False)
    max_age_s = _env_float("LINGTU_SERVICE_READINESS_MAX_AGE_S", 300.0)
    evidence: dict[str, Any] = {
        "path": path_text,
        "required": required,
        "available": False,
        "checked": False,
        "fresh": False,
        "ok": not required,
        "max_age_s": max_age_s,
        "blockers": [],
    }
    if not path_text:
        if required:
            evidence["ok"] = False
            evidence["blockers"] = ["field_readiness_path_unset"]
        return evidence

    path = Path(path_text)
    if not path.exists():
        if required:
            evidence["ok"] = False
            evidence["blockers"] = [f"field_readiness_missing:{path_text}"]
        return evidence

    evidence["available"] = True
    try:
        stat = path.stat()
        age_s = max(0.0, (time.time() if now is None else now) - stat.st_mtime)
        evidence["age_s"] = age_s
        evidence["fresh"] = age_s <= max_age_s
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        evidence["checked"] = True
        evidence["ok"] = False
        evidence["blockers"] = [f"field_readiness_unreadable:{type(exc).__name__}"]
        return evidence

    summary = payload.get("summary", {}) if isinstance(payload, dict) else {}
    evidence["checked"] = True
    evidence["schema"] = payload.get("schema") if isinstance(payload, dict) else None
    evidence["stamp_s"] = payload.get("stamp_s") if isinstance(payload, dict) else None
    evidence["summary"] = summary
    blockers: list[str] = []
    if not evidence["fresh"]:
        blockers.append(f"field_readiness_stale:{path_text}")
    if isinstance(summary, dict) and not summary.get("ok", False):
        blockers.extend(str(item) for item in summary.get("blockers", []))
    evidence["blockers"] = blockers
    evidence["ok"] = not blockers
    return evidence


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
    if backend_name not in {"genz", "super_lio", "super_lio_relocation"} or saved_map_supported is not False:
        return None

    recovery_method = status.get("recovery_method") or raw.get("recovery_method")
    recovery_hint = f"; recovery_method={recovery_method}" if recovery_method else ""
    return _slam_operation_response(
        False,
        message=(f"unsupported: saved map relocalization is not supported by {backend_name}{recovery_hint}"),
        status_code=409,
    )


def _relocalization_service_unavailable_response(gw) -> Any | None:
    if gw.localization.available:
        return None
    return _slam_operation_response(
        False,
        message="relocalization service unavailable",
        status_code=503,
    )


def register_operation_routes(app, gw) -> None:
    from fastapi.responses import JSONResponse, Response

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
            200: {"content": {"application/sdp": {"schema": {"type": "string"}}}},
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

    def _temporal_store():
        if gw._temporal_store is None:
            try:
                import os

                from memory.storage.temporal_store import TemporalStore

                mem_dir = os.environ.get(
                    "LINGTU_MEMORY_DIR",
                    os.path.join(os.path.expanduser("~"), ".nova", "semantic"),
                )
                gw._temporal_store = TemporalStore(os.path.join(mem_dir, "temporal_memory.db"))
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
        try:
            result = await loop.run_in_executor(None, gw._begin_exploration)
        except RuntimeError as exc:
            message = str(exc)
            rejected = "rejected:" in message.lower()
            return JSONResponse(
                status_code=409 if rejected else 503,
                content={
                    "schema_version": 1,
                    "ok": False,
                    "error": ("exploration_command_rejected" if rejected else "exploration_command_unavailable"),
                    "message": message,
                    "detail": {"source": ("native_exploration_ack" if rejected else "exploration_command_boundary")},
                },
            )
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
        if not gw._explorer_stop_available():
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
        try:
            result = await loop.run_in_executor(None, gw._end_exploration)
        except RuntimeError as exc:
            message = str(exc)
            rejected = "rejected:" in message.lower()
            return JSONResponse(
                status_code=409 if rejected else 503,
                content={
                    "schema_version": 1,
                    "ok": False,
                    "error": ("exploration_command_rejected" if rejected else "exploration_command_unavailable"),
                    "message": message,
                    "detail": {"source": ("native_exploration_ack" if rejected else "exploration_command_boundary")},
                },
            )
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
        "/api/v1/services/status",
        summary="Product service status",
        response_model=ServiceStatusResponse,
    )
    async def service_status(names: str | None = None, dds_check: bool = False):
        selected = _parse_service_names(names)
        service_details: dict[str, Any] = {}
        try:
            from lingtu.control import ProductControl

            services, service_details = ProductControl().status(
                tuple(selected),
                dds_check=dds_check,
            )
            _mark_gateway_http_observed(service_details)
        except Exception:
            services = {name: "unknown" for name in selected}
        field_readiness = _load_field_service_readiness()
        return {
            "schema_version": 1,
            "services": services,
            "service_details": service_details,
            "readiness": _service_readiness_summary(
                services,
                service_details,
                selected,
                field_readiness,
            ),
            "field_readiness": field_readiness,
            "service_groups": _SLAM_SERVICE_GROUPS,
            "service_metadata": _SLAM_SERVICE_METADATA,
            "product_runtime": "native_dds",
            "control_entrypoint": "lingtu svc status",
        }

    @app.get(
        "/api/v1/slam/status",
        summary="SLAM service status",
        response_model=SlamStatusResponse,
    )
    async def slam_status():
        service_details: dict[str, Any] = {}
        try:
            from lingtu.control import ProductControl

            services, service_details = ProductControl().status(
                tuple(_SLAM_STATUS_SERVICES),
            )
        except Exception:
            services = {
                "lidar": "unknown",
                "slam": "unknown",
                "traversability": "unknown",
                "nav": "unknown",
                "explore": "unknown",
                "slam_pgo": "unknown",
                "localizer": "unknown",
                "genz_icp": "unknown",
                "hba": "unknown",
                "super_lio": "unknown",
                "super_lio_relocation": "unknown",
            }

        live_mode = ""
        native_mode = None
        try:
            status_snapshot = getattr(gw, "_localization_status", None) or {}
            live_mode = str(gw._slam_profile_from_status(status_snapshot) or "").strip().lower()
            if live_mode == "native_dds":
                native_mode = str(status_snapshot.get("mode") or "").strip() or None
        except Exception:
            live_mode = ""

        slam_detail = service_details.get("slam") if isinstance(service_details, dict) else {}
        slam_active_units = slam_detail.get("active_units", []) if isinstance(slam_detail, dict) else []
        native_slam_active = "lingtu-slam-dds.service" in slam_active_units

        if live_mode in {
            "native_dds",
            "fastlio2",
            "genz",
            "localizer",
            "super_lio",
            "super_lio_relocation",
            "none",
        }:
            mode = live_mode
        elif services.get("super_lio_relocation") in _ACTIVE_SERVICE_STATES:
            mode = "super_lio_relocation"
        elif services.get("super_lio") in _ACTIVE_SERVICE_STATES:
            mode = "super_lio"
        elif services.get("genz_icp") in _ACTIVE_SERVICE_STATES:
            mode = "genz"
        elif native_slam_active:
            mode = "native_dds"
        elif services.get("slam_pgo") in _ACTIVE_SERVICE_STATES:
            mode = "fastlio2"
        elif services.get("localizer") in _ACTIVE_SERVICE_STATES:
            mode = "localizer"
        elif services.get("slam") in _ACTIVE_SERVICE_STATES:
            mode = "fastlio2"
        else:
            mode = "stopped"
        return {
            "mode": mode,
            "native_mode": native_mode,
            "services": services,
            "service_details": service_details,
            "service_groups": _SLAM_SERVICE_GROUPS,
            "service_metadata": _SLAM_SERVICE_METADATA,
            "product_runtime": "native_dds",
            "ros2_required": False,
            "manual_systemctl_required": False,
            "control_entrypoint": "lingtu svc restart slam",
        }

    @app.post(
        "/api/v1/slam/switch",
        summary="Hot-switch SLAM profile",
        response_model=SlamOperationResponse,
        responses={
            400: {"model": SlamOperationResponse},
            409: {"model": SlamOperationResponse},
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
        if not gw._manage_session_services:
            return _slam_operation_response(
                False,
                message=(
                    "SLAM is owned by the active Profile/Endpoint; switch the "
                    "product instead of hot-switching Gateway services"
                ),
                status_code=409,
            )
        try:
            from lingtu.control import ProductControl

            plan = slam_switch_plan(profile)
            ok = ProductControl().legacy_transition(plan, timeout_s=10.0)
            if ok:
                gw._cached_slam_profile = "stopped" if profile == "stop" else profile
                gw._slam_profile_ts = time.time()
            return slam_operation_payload(
                ok,
                profile=profile,
                message=(f"Switched to {profile}" if ok else "Services not ready after 10s"),
            )
        except Exception as e:
            return _slam_operation_response(False, message=str(e), status_code=500)

    @app.post(
        "/api/v1/slam/restart",
        summary="Force-restart native SLAM localization service",
        response_model=SlamOperationResponse,
        responses={
            500: {"model": SlamOperationResponse},
            504: {"model": SlamOperationResponse},
        },
    )
    async def slam_restart():
        if not gw._manage_session_services:
            try:
                from lingtu.control import ProductControl

                report = ProductControl().restart("slam")
                clear_cache = getattr(gw, "clear_localization_runtime_cache", None)
                if callable(clear_cache):
                    clear_cache(reason="manual_localization_restart")
                gw._cached_slam_profile = "native_dds"
                gw._slam_profile_ts = time.time()
                return slam_operation_payload(
                    report.ok,
                    profile="native_dds",
                    message="Native SLAM restarted",
                    details=report.as_dict(),
                )
            except Exception as e:
                return _slam_operation_response(False, message=str(e), status_code=500)
        try:
            from lingtu.control import ProductControl

            control = ProductControl()
            clear_cache = getattr(gw, "clear_localization_runtime_cache", None)
            if callable(clear_cache):
                clear_cache(reason="manual_localization_restart")
            ok = control.legacy_restart("slam", timeout_s=20.0)
            if ok:
                gw._cached_slam_profile = "native_dds"
                gw._slam_profile_ts = time.time()
            return slam_operation_payload(
                ok,
                profile="native_dds",
                message=("Native SLAM restarted" if ok else "SLAM service not ready after restart"),
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
            result = gw.localization.trigger_global_relocalize(timeout_s=10.0)
            if result.timed_out:
                return _slam_operation_response(
                    False,
                    message=result.message,
                    status_code=504,
                )
            return slam_operation_payload(
                result.success,
                message=result.message,
                quality=result.quality,
                details=dict(result.details),
            )
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
        try:
            ensure_maps_service(gw)
        except RuntimeError as exc:
            return _slam_operation_response(False, message=str(exc), status_code=503)
        pcd_path = gw._map_artifact_path_from_maps_service(
            map_name,
            "source_pointcloud",
        )
        if pcd_path is None:
            return _slam_operation_response(
                False,
                message=f"Map source point cloud unavailable: {map_name}",
                status_code=404,
            )
        unavailable_response = _relocalization_service_unavailable_response(gw)
        if unavailable_response is not None:
            return unavailable_response
        try:
            result = gw.localization.relocalize_saved_map(
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
            return slam_operation_payload(
                ok,
                message=msg,
                quality=quality,
                details=dict(result.details),
            )
        except Exception as e:
            return _slam_operation_response(False, message=str(e), status_code=500)

    @app.post(
        "/api/v1/slam/track_against_map",
        summary="Start continuous saved-map tracking",
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
    async def slam_track_against_map(body: SlamRelocalizeRequest):
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
        try:
            ensure_maps_service(gw)
        except RuntimeError as exc:
            return _slam_operation_response(False, message=str(exc), status_code=503)
        pcd_path = gw._map_artifact_path_from_maps_service(
            map_name,
            "source_pointcloud",
        )
        if pcd_path is None:
            return _slam_operation_response(
                False,
                message=f"Map source point cloud unavailable: {map_name}",
                status_code=404,
            )
        unavailable_response = _relocalization_service_unavailable_response(gw)
        if unavailable_response is not None:
            return unavailable_response
        try:
            result = gw.localization.track_against_map(
                pcd_path,
                x,
                y,
                yaw,
                timeout_s=10.0,
            )
            if result.timed_out:
                return _slam_operation_response(
                    False,
                    message=result.message,
                    status_code=504,
                )
            return slam_operation_payload(
                result.success,
                message=result.message,
                quality=result.quality,
                details=dict(result.details),
            )
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
            script = repo_root / "scripts" / "hardware" / "record_bag.sh"
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
            "exit_code": (proc.returncode if proc and proc.poll() is not None else None),
            "disk_free": disk_free,
            "disk_total": disk_total,
        }
