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
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from gateway.schemas import (
    DirectedExplorationClearRequest,
    DirectedExplorationResponse,
    DirectedExplorationTargetRequest,
    ExplorationCommandResponse,
    ExplorationRunCommandRequest,
    ExplorationRunListResponse,
    ExplorationRunResponse,
    ExplorationStartRequest,
    ExplorationStatusResponse,
    GatewayErrorResponse,
    Go2RTCStatusResponse,
    LocalizationMapTrackingRequest,
    LocalizationOperationResponse,
    LocalizationRelocalizationRequest,
    RecordingOperationResponse,
    RecordingStartRequest,
    RecordingStatusResponse,
    ServiceStatusResponse,
    SlamStatusResponse,
    TemporalMemoryResponse,
    TemporalSemanticRequest,
)
from gateway.services.exploration import (
    DirectedExplorationError,
    ExplorationRunError,
    clear_directed_exploration_target,
    command_exploration_run,
    directed_exploration_target,
    product_control_owns_explore,
    start_exploration_run,
)
from gateway.services.explore_runs import ensure_explore_runs
from gateway.services.mapd_transport import mapd_query, safe_map_name
from gateway.services.recording import NativeRecordingError
from gateway.services.runtime_status import runtime_identity
from runtime.service_catalogs.thunder import (
    thunder_runtime_services,
    thunder_service_groups,
    thunder_service_metadata,
    thunder_service_spec,
    thunder_slam_status_services,
)

logger = logging.getLogger(__name__)

_ACTIVE_SERVICE_STATES = {"running", "active"}
_SERVICE_STATUS_DEFAULTS = tuple(dict.fromkeys((*thunder_runtime_services(), "gateway")))
_FIELD_SERVICE_READINESS_DEFAULT_PATH = "/tmp/lingtu_service_readiness.json"
_SLAM_STATUS_SERVICES = thunder_slam_status_services()
_SLAM_SERVICE_GROUPS = thunder_service_groups()
_SLAM_SERVICE_METADATA = thunder_service_metadata()
_EXPLORER_UNAVAILABLE_DETAIL = {
    "reason": "explorer_backend_not_running",
    "required_product": "explore",
    "supported_products": ["explore"],
    "action": "switch LingTu to the explore Product before starting exploration",
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
    evidence["_report"] = payload if isinstance(payload, dict) else {}
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


def _run_plan_process(gw: Any, service_name: str) -> Any | None:
    plan = getattr(gw, "_compiled_run_plan", None)
    logical_name = "host" if service_name in {"gateway", "lingtu"} else service_name
    for process in tuple(getattr(plan, "processes", ()) or ()):
        if str(getattr(process, "name", "") or "") == logical_name:
            return process
    return None


def _field_status_for_service(
    service_name: str,
    *,
    process: Any | None,
    report: Mapping[str, Any],
) -> tuple[str | None, dict[str, Any]]:
    systemd = report.get("systemd")
    systemd = systemd if isinstance(systemd, Mapping) else {}
    if process is not None:
        units = (str(getattr(process, "target", "") or ""),)
    else:
        spec = thunder_service_spec(service_name)
        units = tuple(spec.units) if spec is not None else ()
    unit_evidence = {unit: dict(systemd[unit]) for unit in units if unit and isinstance(systemd.get(unit), Mapping)}
    if unit_evidence:
        active = any(
            str(item.get("active_state") or "").lower() == "active"
            or str(item.get("sub_state") or "").lower() == "running"
            for item in unit_evidence.values()
        )
        known = any(str(item.get("active_state") or "").strip() for item in unit_evidence.values())
        status = "running" if active else ("stopped" if known else "unknown")
        return status, {"source": "field_readiness", "systemd": unit_evidence}

    dds = report.get("dds")
    dds = dds if isinstance(dds, Mapping) else {}
    dds_services = dds.get("services")
    dds_services = dds_services if isinstance(dds_services, Mapping) else {}
    dds_evidence = dds_services.get(service_name)
    if dds.get("checked") is True and isinstance(dds_evidence, Mapping):
        return (
            "active" if dds_evidence.get("ok") is True else "unknown",
            {"source": "field_readiness", "dds": dict(dds_evidence)},
        )
    return None, {}


def _service_status_snapshot(
    gw: Any,
    selected: tuple[str, ...],
    field_readiness: dict[str, Any],
) -> tuple[dict[str, str], dict[str, dict[str, Any]]]:
    plan = getattr(gw, "_compiled_run_plan", None)
    report = field_readiness.pop("_report", {})
    report = report if field_readiness.get("fresh") and isinstance(report, Mapping) else {}
    live_slam_profile = (
        str(gw._slam_profile_from_status(getattr(gw, "_localization_status", None)) or "").strip().lower()
    )
    backend_service = {
        "native_dds": "slam",
        "fastlio2": "slam",
    }.get(live_slam_profile)
    session_mode = str(getattr(gw, "_session_mode", "idle") or "idle").strip().lower()
    exploring = bool(getattr(gw, "_exploring", False))

    services: dict[str, str] = {}
    details: dict[str, dict[str, Any]] = {}
    for name in selected:
        process = _run_plan_process(gw, name)
        declared = process is not None
        status = "declared" if declared else ("not_declared" if plan is not None else "unknown")
        observed: dict[str, Any] = {}
        if plan is not None:
            observed["runtime"] = runtime_identity(gw)
            observed["process"] = {
                "declared": declared,
                "target": str(getattr(process, "target", "") or "") if process is not None else None,
            }

        field_status, field_observed = _field_status_for_service(
            name,
            process=process,
            report=report,
        )
        if field_status is not None:
            status = field_status
            observed["field_readiness"] = field_observed

        if backend_service == name:
            status = "running"
            observed["native_telemetry"] = {
                "backend": live_slam_profile,
                "source": "localization_status",
            }

        if name == "explore" and exploring:
            status = "running"
        observed["product_view"] = {
            "mode": session_mode,
            "exploring": exploring,
        }

        if name in {"gateway", "lingtu"}:
            status = "running"

        ready = status in _ACTIVE_SERVICE_STATES
        metadata = _SLAM_SERVICE_METADATA.get(name, {})
        details[name] = {
            "status": status,
            "ready": ready,
            "blockers": [] if ready else [f"status_{status}"],
            "observed": observed,
            "contract": {
                "checks": list(metadata.get("checks") or []),
                "topics": list(metadata.get("topics") or []),
                "dds_topics": list(metadata.get("dds_topics") or []),
                "files": list(metadata.get("files") or []),
                "binaries": list(metadata.get("binaries") or []),
            },
        }
        services[name] = status

    _mark_gateway_http_observed(details)
    return services, details


def _operation_payload(success: bool, **fields: Any) -> dict[str, Any]:
    payload = {
        "schema_version": 1,
        "ok": bool(success),
        "success": bool(success),
        "ts": time.time(),
    }
    payload.update({key: value for key, value in fields.items() if value is not None})
    return payload


def _localization_operation_response(
    success: bool,
    *,
    status_code: int,
    **fields: Any,
) -> Any:
    from fastapi.responses import JSONResponse

    return JSONResponse(
        _operation_payload(success, **fields),
        status_code=status_code,
    )


def _relocalization_service_unavailable_response(gw) -> Any | None:
    if gw.localization.available:
        return None
    return _localization_operation_response(
        False,
        message="relocalization service unavailable",
        status_code=503,
    )


def _field_product_map_guard_response(
    gw: Any,
    map_name: str,
) -> tuple[str, Any | None]:
    """Keep field SLAM control aligned with ProductControl's active map."""

    from gateway.routes.session import (
        _external_product_map_guard,
        _externally_owned_product,
    )

    current_product = _externally_owned_product(gw)
    if current_product is None:
        return map_name, None
    resolved_map, blocker = _external_product_map_guard(
        gw,
        current_product=current_product,
        requested_map=map_name,
    )
    if blocker is None:
        return resolved_map, None
    return resolved_map, _localization_operation_response(
        False,
        message="ProductControl must switch the active map before field relocalization",
        details=blocker,
        status_code=409,
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
        payload = body.model_dump(exclude_none=True)
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
    async def explore_start(body: ExplorationStartRequest | None = None):
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
        if product_control_owns_explore(gw):
            loop = asyncio.get_event_loop()
            try:
                result = await loop.run_in_executor(
                    None,
                    lambda: start_exploration_run(
                        gw,
                        request_id=(body.request_id if body is not None else None),
                    ),
                )
            except ExplorationRunError as exc:
                return JSONResponse(
                    status_code=exc.status_code,
                    content={
                        "schema_version": 1,
                        "ok": False,
                        "error": exc.code,
                        "message": str(exc),
                        "detail": exc.detail,
                    },
                )
            if result.get("accepted") is not True:
                return JSONResponse(
                    status_code=409,
                    content={
                        "schema_version": 1,
                        "ok": False,
                        "error": "exploration_command_rejected",
                        "message": str(result.get("reason") or "Native Explore start was rejected."),
                        "detail": result,
                    },
                )
            gw.push_event({"type": "exploration_run_admitted", "data": result})
            return result

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

    @app.get(
        "/api/v1/explore/runs/{exploration_run_id}",
        summary="Query one durable native Explore execution",
        response_model=ExplorationRunResponse,
        responses={404: {"model": GatewayErrorResponse}},
    )
    async def explore_run_get(exploration_run_id: str):
        try:
            result = ensure_explore_runs(gw).query(exploration_run_id)
        except ValueError as exc:
            return JSONResponse(
                status_code=400,
                content={
                    "schema_version": 1,
                    "ok": False,
                    "error": "exploration_run_id_invalid",
                    "message": str(exc),
                    "detail": {},
                },
            )
        if not result.get("found"):
            return JSONResponse(
                status_code=404,
                content={
                    "schema_version": 1,
                    "ok": False,
                    "error": "exploration_run_not_found",
                    "message": "The requested Explore run is not retained on this robot.",
                    "detail": result,
                },
            )
        return result

    @app.get(
        "/api/v1/explore/runs",
        summary="List recent durable native Explore executions",
        response_model=ExplorationRunListResponse,
    )
    async def explore_runs_list(limit: int = 20):
        runs = ensure_explore_runs(gw)
        return {
            "schema_version": "lingtu.explore.run.list.v1",
            "runs": runs.list_recent(limit=limit),
            "health": runs.health(),
        }

    def _register_explore_run_command(path_action: str) -> None:
        async def command(
            exploration_run_id: str,
            body: ExplorationRunCommandRequest | None = None,
        ):
            loop = asyncio.get_event_loop()
            try:
                return await loop.run_in_executor(
                    None,
                    lambda: command_exploration_run(
                        gw,
                        exploration_run_id,
                        action=path_action,
                        request_id=(body.request_id if body is not None else None),
                        reason=(body.reason if body is not None else None),
                    ),
                )
            except ExplorationRunError as exc:
                return JSONResponse(
                    status_code=exc.status_code,
                    content={
                        "schema_version": 1,
                        "ok": False,
                        "error": exc.code,
                        "message": str(exc),
                        "detail": exc.detail,
                    },
                )

        command.__name__ = f"explore_run_{path_action}"
        app.post(
            f"/api/v1/explore/runs/{{exploration_run_id}}/{path_action}",
            summary=f"{path_action.title()} one native Explore execution",
            response_model=ExplorationRunResponse,
            responses={
                404: {"model": GatewayErrorResponse},
                409: {"model": GatewayErrorResponse},
                503: {"model": GatewayErrorResponse},
            },
        )(command)

    for _explore_run_action in ("pause", "resume", "finish"):
        _register_explore_run_command(_explore_run_action)

    @app.post(
        "/api/v1/explore/stop",
        summary="Stop autonomous frontier exploration",
        response_model=ExplorationCommandResponse,
        responses={
            409: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def explore_stop():
        if product_control_owns_explore(gw):
            return JSONResponse(
                status_code=409,
                content={
                    "schema_version": 1,
                    "ok": False,
                    "error": "product_control_stop_required",
                    "message": (
                        "Field exploration stop must use ProductControl so the "
                        "robot is not reported stopped before parking evidence."
                    ),
                    "detail": {
                        "reason": "parking_evidence_required",
                        "operator_command": (
                            "python -m lingtu.control stop "
                            "--expected-product explore"
                        ),
                    },
                },
            )
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

    @app.post(
        "/api/v1/explore/directed",
        summary="Set an explicit native TARE exploration direction intent",
        response_model=DirectedExplorationResponse,
        responses={
            409: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def explore_directed(body: DirectedExplorationTargetRequest):
        """Set a TARE preference without converting a click into a nav goal."""

        try:
            result = await asyncio.to_thread(
                directed_exploration_target,
                gw,
                x=body.x,
                y=body.y,
                ttl_s=body.ttl_s,
                reason=body.reason,
                request_id=body.request_id,
            )
        except DirectedExplorationError as exc:
            return JSONResponse(
                status_code=exc.status_code,
                content={
                    "schema_version": 1,
                    "ok": False,
                    "error": exc.code,
                    "message": str(exc),
                    "detail": exc.detail,
                },
            )
        return {
            "schema_version": 1,
            "ok": True,
            "accepted": True,
            "status": "accepted",
            **result,
        }

    @app.post(
        "/api/v1/explore/directed/clear",
        summary="Clear the explicit native TARE exploration direction intent",
        response_model=DirectedExplorationResponse,
        responses={
            409: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def clear_explore_directed(body: DirectedExplorationClearRequest):
        try:
            result = await asyncio.to_thread(
                clear_directed_exploration_target,
                gw,
                reason=body.reason,
                request_id=body.request_id,
            )
        except DirectedExplorationError as exc:
            return JSONResponse(
                status_code=exc.status_code,
                content={
                    "schema_version": 1,
                    "ok": False,
                    "error": exc.code,
                    "message": str(exc),
                    "detail": exc.detail,
                },
            )
        return {
            "schema_version": 1,
            "ok": True,
            "accepted": True,
            "status": "cleared",
            **result,
        }

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
        field_readiness = _load_field_service_readiness()
        services, service_details = _service_status_snapshot(
            gw,
            selected,
            field_readiness,
        )
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
        }

    @app.get(
        "/api/v1/slam/status",
        summary="SLAM service status",
        response_model=SlamStatusResponse,
    )
    async def slam_status():
        field_readiness = _load_field_service_readiness()
        services, service_details = _service_status_snapshot(
            gw,
            _SLAM_STATUS_SERVICES,
            field_readiness,
        )

        live_mode = ""
        native_mode = None
        try:
            status_snapshot = getattr(gw, "_localization_status", None) or {}
            live_mode = str(gw._slam_profile_from_status(status_snapshot) or "").strip().lower()
            if live_mode == "native_dds":
                native_mode = str(status_snapshot.get("mode") or "").strip() or None
        except Exception:
            live_mode = ""

        if live_mode in {
            "native_dds",
            "fastlio2",
            "none",
        }:
            mode = live_mode
        elif services.get("slam") in _ACTIVE_SERVICE_STATES:
            mode = "native_dds"
        else:
            cached_mode = str(gw._get_slam_profile() or "").strip().lower()
            mode = cached_mode if cached_mode not in {"", "none"} else "stopped"
        return {
            "mode": mode,
            "native_mode": native_mode,
            "services": services,
            "service_details": service_details,
            "service_groups": _SLAM_SERVICE_GROUPS,
            "service_metadata": _SLAM_SERVICE_METADATA,
            "product_runtime": "native_dds",
            "manual_systemctl_required": False,
        }

    def localization_map_id(map_name: str) -> tuple[str, Any | None]:
        name_error = safe_map_name(map_name)
        if name_error is not None:
            return map_name, _localization_operation_response(
                False,
                message=name_error,
                status_code=400,
            )
        resolved_map, map_guard_response = _field_product_map_guard_response(gw, map_name)
        if map_guard_response is not None:
            return resolved_map, map_guard_response
        try:
            record = mapd_query(
                gw,
                {"action": "get_record", "map_id": resolved_map},
            )
        except Exception as exc:
            return resolved_map, _localization_operation_response(
                False,
                message=str(exc),
                status_code=503,
            )
        if record.get("success") is not True:
            return resolved_map, _localization_operation_response(
                False,
                map_name=resolved_map,
                message=f"Map unavailable: {resolved_map}",
                status_code=404,
            )
        return resolved_map, None

    @app.post(
        "/api/v1/localization/relocalizations",
        summary="Relocalize against the running Product's active map",
        response_model=LocalizationOperationResponse,
        responses={
            400: {"model": LocalizationOperationResponse},
            404: {"model": LocalizationOperationResponse},
            409: {"model": LocalizationOperationResponse},
            500: {"model": LocalizationOperationResponse},
            503: {"model": LocalizationOperationResponse},
            504: {"model": LocalizationOperationResponse},
        },
    )
    async def localization_relocalize(body: LocalizationRelocalizationRequest):
        request = (
            body
            if isinstance(body, LocalizationRelocalizationRequest)
            else LocalizationRelocalizationRequest.model_validate(body)
        )
        map_id, map_response = localization_map_id(request.map_name)
        if map_response is not None:
            return map_response
        unavailable_response = _relocalization_service_unavailable_response(gw)
        if unavailable_response is not None:
            return unavailable_response

        try:
            if request.mode == "global":
                result = gw.localization.trigger_global_relocalize(timeout_s=10.0)
            else:
                if request.initial_pose is None:
                    return _localization_operation_response(
                        False,
                        map_name=map_id,
                        mode=request.mode,
                        request_id=request.request_id,
                        message="seeded relocalization requires an initial pose",
                        status_code=400,
                    )
                result = gw.localization.relocalize_saved_map(
                    map_id,
                    request.initial_pose.x,
                    request.initial_pose.y,
                    request.initial_pose.yaw,
                    timeout_s=30.0,
                )
            if result.timed_out:
                return _localization_operation_response(
                    False,
                    map_name=map_id,
                    mode=request.mode,
                    request_id=request.request_id,
                    message=result.message,
                    status_code=504,
                )
            ok = result.success
            quality = result.quality
            if quality is None and ok:
                quality = float(getattr(gw, "_icp_quality", 0.0))
            return _operation_payload(
                ok,
                map_name=map_id,
                mode=request.mode,
                request_id=request.request_id,
                message=result.message if not ok else f"Relocalized to {map_id}",
                quality=quality,
                details=dict(result.details),
            )
        except Exception as exc:
            return _localization_operation_response(
                False,
                map_name=map_id,
                mode=request.mode,
                request_id=request.request_id,
                message=str(exc),
                status_code=500,
            )

    @app.post(
        "/api/v1/localization/map-tracking",
        summary="Start continuous tracking against the running Product's active map",
        response_model=LocalizationOperationResponse,
        responses={
            400: {"model": LocalizationOperationResponse},
            404: {"model": LocalizationOperationResponse},
            409: {"model": LocalizationOperationResponse},
            500: {"model": LocalizationOperationResponse},
            503: {"model": LocalizationOperationResponse},
            504: {"model": LocalizationOperationResponse},
        },
    )
    async def localization_map_tracking(body: LocalizationMapTrackingRequest):
        request = (
            body
            if isinstance(body, LocalizationMapTrackingRequest)
            else LocalizationMapTrackingRequest.model_validate(body)
        )
        map_name, map_response = localization_map_id(request.map_name)
        if map_response is not None:
            return map_response
        unavailable_response = _relocalization_service_unavailable_response(gw)
        if unavailable_response is not None:
            return unavailable_response
        try:
            result = gw.localization.start_map_tracking(timeout_s=10.0)
            if result.timed_out:
                return _localization_operation_response(
                    False,
                    map_name=map_name,
                    mode="tracking",
                    request_id=request.request_id,
                    message=result.message,
                    status_code=504,
                )
            return _operation_payload(
                result.success,
                map_name=map_name,
                mode="tracking",
                request_id=request.request_id,
                message=result.message,
                quality=result.quality,
                details=dict(result.details),
            )
        except Exception as exc:
            return _localization_operation_response(
                False,
                map_name=map_name,
                mode="tracking",
                request_id=request.request_id,
                message=str(exc),
                status_code=500,
            )

    def _recording_error_response(exc: NativeRecordingError):
        return JSONResponse(
            status_code=exc.status_code,
            content={"error": exc.code, "detail": None},
        )

    @app.post(
        "/api/v1/recordings/start",
        summary="Start native MCAP recording",
        response_model=RecordingOperationResponse,
        responses={
            409: {"model": RecordingOperationResponse},
            503: {"model": RecordingOperationResponse},
        },
    )
    async def recording_start(body: RecordingStartRequest = RecordingStartRequest()):
        duration = body.duration
        prefix = body.prefix
        capture_profile = body.capture_profile
        task_id = body.task_id
        camera = body.camera
        minimum_free_gib = body.minimum_free_gib
        try:
            snapshot = await asyncio.to_thread(
                gw._recording.start,
                duration=duration,
                prefix=prefix,
                capture_profile=capture_profile,
                task_id=task_id,
                camera=camera,
                minimum_free_gib=minimum_free_gib,
            )
        except NativeRecordingError as exc:
            return _recording_error_response(exc)
        return {
            "status": "started",
            "state": snapshot.state,
            "backend": "native_mcap",
            "session_id": snapshot.session_id,
            "path": None,
            "pid": None,
            "duration": duration,
            "prefix": prefix,
            "capture_profile": capture_profile,
            "task_id": task_id,
            "camera": camera,
            "minimum_free_gib": minimum_free_gib,
        }

    @app.post(
        "/api/v1/recordings/stop",
        summary="Stop native MCAP recording",
        response_model=RecordingOperationResponse,
        responses={404: {"model": RecordingOperationResponse}},
    )
    async def recording_stop():
        try:
            snapshot = await asyncio.to_thread(gw._recording.stop)
        except NativeRecordingError as exc:
            return _recording_error_response(exc)
        return {
            "status": snapshot.state,
            "state": snapshot.state,
            "backend": "native_mcap",
            "session_id": snapshot.session_id,
            "path": None,
            "pid": None,
        }

    @app.get(
        "/api/v1/recordings/status",
        summary="Native MCAP recording status",
        response_model=RecordingStatusResponse,
    )
    async def recording_status():
        try:
            snapshot = await asyncio.to_thread(gw._recording.status)
        except NativeRecordingError as exc:
            return _recording_error_response(exc)
        payload = snapshot.as_payload()
        payload["path"] = None
        payload["pid"] = None
        payload["exit_code"] = None
        return payload
