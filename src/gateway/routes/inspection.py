"""Inspection route product APIs backed by native C++ adapters."""

from __future__ import annotations

import asyncio
import errno
import json
import math
import os
import stat
import time
import uuid
from pathlib import Path
from typing import Any

from fastapi.responses import JSONResponse, Response

from gateway.schemas import (
    GatewayErrorResponse,
    InspectionCommandResponse,
    InspectionRouteListResponse,
    InspectionRouteRequest,
    InspectionRouteResponse,
    InspectionStatusResponse,
    InspectionTaskCommandResponse,
    InspectionTaskControlRequest,
    InspectionTaskListResponse,
    InspectionTaskReportResponse,
    InspectionTaskStartRequest,
    InspectionTaskStatusResponse,
)
from gateway.services.inspection_boundary import (
    InspectionBoundaryError,
    InspectionCommandRejected,
    invoke_inspection,
)
from gateway.services.inspection_report import build_inspection_task_report
from gateway.services.inspection_task_lifecycle import (
    InspectionTaskJournalUnavailable,
    ensure_inspection_task_timeline,
)
from gateway.services.map_service import map_service_query
from maps.paths import active_map_name, nav_map_root
from runtime.contracts.inspection_evidence import (
    EvidenceIntegrityError,
    EvidenceValidationError,
    InspectionEvidenceResult,
    InspectionEvidenceStore,
)

_DEFAULT_EVIDENCE_STATUS_FILE = "/dev/shm/lingtu/inspection_evidence_status.json"
_DEFAULT_EVIDENCE_HEARTBEAT_MAX_AGE_S = 3.0
_ARTIFACT_INTEGRITY_ERRNOS = {errno.ENOENT, errno.ENOTDIR, errno.EISDIR, errno.ELOOP}
_DEFAULT_EVIDENCE_ROOT = "~/data/lingtu/inspection_evidence"
_EVIDENCE_ARTIFACT_KINDS = {"rgb", "pose", "detections"}


def _is_safe_artifact_name(value: str) -> bool:
    return (
        bool(value)
        and value not in {".", ".."}
        and "/" not in value
        and "\\" not in value
    )


def _error(
    status_code: int,
    code: str,
    message: str,
    *,
    detail: dict[str, Any] | None = None,
) -> JSONResponse:
    return JSONResponse(
        status_code=status_code,
        content=GatewayErrorResponse(
            error=code,
            message=message,
            detail=detail,
        ).model_dump(mode="json"),
    )


def _native_error(exc: Exception) -> JSONResponse:
    message = str(exc) or "native inspection adapter failed"
    lowered = message.lower()
    if "not found" in lowered and "library" not in lowered:
        return _error(404, "inspection_route_not_found", message)
    if "invalid" in lowered or "must" in lowered or "route point" in lowered:
        return _error(400, "inspection_route_invalid", message)
    return _error(503, "inspection_native_unavailable", message)


def _task_command_rejected(
    exc: Exception,
    *,
    action: str,
    task_id: str,
    request_id: str,
) -> JSONResponse:
    """Return a retry-safe conflict when the live endpoint declines a task command."""

    return _error(
        409,
        "inspection_task_rejected",
        "native inspection endpoint rejected the task command",
        detail={
            "action": action,
            "task_id": task_id,
            "request_id": request_id,
            "native_reason": str(exc) or "inspection_task_rejected",
        },
    )


def _resolve_map_id(value: str | None) -> str | JSONResponse:
    requested = str(value or "").strip()
    if requested:
        return requested
    resolved = active_map_name(nav_map_root())
    if resolved:
        return resolved
    return _error(
        400,
        "inspection_map_required",
        "map_id is required when no active map is selected",
    )


def _evidence_root() -> Path:
    return Path(os.environ.get("LINGTU_INSPECTION_EVIDENCE_DIR", _DEFAULT_EVIDENCE_ROOT))


def _evidence_store() -> InspectionEvidenceStore:
    return InspectionEvidenceStore(_evidence_root())


def _run_store_operation(gw: Any, method: str, *args) -> Any:
    """Run one route-store operation through the assembled domain service."""

    try:
        if method == "list":
            operation, kwargs = "list_routes", {"map_id": args[0]}
        elif method == "put":
            operation, kwargs = "put_route", {"route": args[0]}
        elif method == "get":
            operation, kwargs = "get_route", {"map_id": args[0], "route_id": args[1]}
        elif method == "delete":
            operation, kwargs = "delete_route", {"map_id": args[0], "route_id": args[1]}
        elif method == "status":
            operation, kwargs = "status", {}
        else:
            raise KeyError(method)
    except (KeyError, IndexError) as exc:
        raise InspectionBoundaryError(f"unsupported inspection store operation: {method}") from exc
    return invoke_inspection(gw, operation, **kwargs)


def _evidence_not_found() -> JSONResponse:
    return _error(
        404,
        "inspection_evidence_not_found",
        "inspection evidence was not found",
    )


def _evidence_integrity_error(exc: Exception) -> JSONResponse:
    return _error(
        409,
        "inspection_evidence_integrity_failed",
        str(exc) or "inspection evidence integrity check failed",
    )


def _evidence_unavailable(exc: Exception) -> JSONResponse:
    return _error(
        503,
        "inspection_evidence_unavailable",
        "inspection evidence store is unavailable",
        detail={"reason": str(exc)},
    )


def _evidence_result(evidence_id: str) -> InspectionEvidenceResult | JSONResponse:
    try:
        return _evidence_store().get(evidence_id)
    except FileNotFoundError:
        return _evidence_not_found()
    except EvidenceValidationError:
        return _evidence_not_found()
    except EvidenceIntegrityError as exc:
        return _evidence_integrity_error(exc)
    except OSError as exc:
        return _evidence_unavailable(exc)


def _artifact_records(result: InspectionEvidenceResult) -> list[dict[str, Any]]:
    persistence = result.manifest.get("persistence")
    if not isinstance(persistence, dict):
        raise EvidenceIntegrityError("manifest persistence section is invalid")
    artifacts = persistence.get("artifacts")
    if not isinstance(artifacts, list):
        raise EvidenceIntegrityError("manifest artifacts must be an array")
    records: list[dict[str, Any]] = []
    for record in artifacts:
        if not isinstance(record, dict):
            raise EvidenceIntegrityError("artifact record must be an object")
        kind = record.get("kind")
        if not isinstance(kind, str):
            raise EvidenceIntegrityError("artifact kind is invalid")
        records.append(record)
    return records


def _public_artifacts(result: InspectionEvidenceResult) -> list[dict[str, Any]]:
    public: list[dict[str, Any]] = []
    for record in _artifact_records(result):
        item = {
            "kind": record.get("kind"),
            "media_type": record.get("media_type"),
            "bytes": record.get("bytes"),
            "sha256": record.get("sha256"),
        }
        public.append({key: value for key, value in item.items() if value is not None})
    return public


def _evidence_summary(result: InspectionEvidenceResult) -> dict[str, Any]:
    return {
        "evidence_id": result.request.request_id,
        "manifest_sha256": result.manifest_sha256,
        "request": result.manifest.get("request", {}),
        "analysis": result.manifest.get("analysis", {}),
        "persistence": {
            key: value
            for key, value in dict(result.manifest.get("persistence", {})).items()
            if key != "artifacts"
        },
        "artifacts": _public_artifacts(result),
    }


def _report_evidence_by_id(task: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Verify every evidence ID referenced by retained native task facts."""

    evidence: dict[str, dict[str, Any]] = {}
    timeline = task.get("timeline")
    if not isinstance(timeline, list):
        return evidence
    for event in timeline:
        if not isinstance(event, dict) or int(event.get("kind", 0) or 0) != 5:
            continue
        evidence_id = str(event.get("evidence_id") or "").strip()
        if not evidence_id or evidence_id in evidence:
            continue
        result = _evidence_result(evidence_id)
        if isinstance(result, InspectionEvidenceResult):
            evidence[evidence_id] = {
                "status": "VERIFIED",
                "summary": _evidence_summary(result),
            }
        elif result.status_code == 409:
            evidence[evidence_id] = {
                "status": "INVALID",
                "reason": "evidence_integrity_failed",
            }
        elif result.status_code == 503:
            evidence[evidence_id] = {
                "status": "UNAVAILABLE",
                "reason": "evidence_store_unavailable",
            }
        else:
            evidence[evidence_id] = {
                "status": "MISSING",
                "reason": "evidence_not_found",
            }
    return evidence


def _read_evidence_artifact(
    result: InspectionEvidenceResult,
    kind: str,
) -> tuple[bytes, str] | JSONResponse:
    if kind not in _EVIDENCE_ARTIFACT_KINDS:
        return _error(
            400,
            "inspection_evidence_artifact_kind_invalid",
            "inspection evidence artifact kind is invalid",
            detail={"kind": kind, "allowed": sorted(_EVIDENCE_ARTIFACT_KINDS)},
        )
    try:
        matching = [record for record in _artifact_records(result) if record.get("kind") == kind]
        if not matching:
            return _evidence_not_found()
        record = matching[0]
        relative_path = record.get("path")
        if not isinstance(relative_path, str) or not _is_safe_artifact_name(relative_path):
            raise EvidenceIntegrityError("artifact path is invalid")
        artifact_path = result.evidence_dir / relative_path
        evidence_root = result.evidence_dir.resolve()
        try:
            artifact_stat = artifact_path.lstat()
        except OSError as exc:
            raise EvidenceIntegrityError(f"cannot stat artifact {kind}: {exc}") from exc
        if stat.S_ISLNK(artifact_stat.st_mode) or not stat.S_ISREG(artifact_stat.st_mode):
            raise EvidenceIntegrityError(f"artifact is not a regular file: {kind}")
        if artifact_path.resolve().parent != evidence_root:
            raise EvidenceIntegrityError("artifact path escaped the evidence directory")
        expected_bytes = record.get("bytes")
        expected_sha256 = record.get("sha256")
        if isinstance(expected_bytes, bool) or not isinstance(expected_bytes, int) or expected_bytes < 0:
            raise EvidenceIntegrityError(f"artifact byte count is invalid: {kind}")
        if not isinstance(expected_sha256, str) or not expected_sha256:
            raise EvidenceIntegrityError(f"artifact sha256 is invalid: {kind}")
        payload = _read_regular_file_bytes(artifact_path)
        if len(payload) != expected_bytes:
            raise EvidenceIntegrityError(f"artifact byte count mismatch: {kind}")
        if _sha256_bytes(payload) != expected_sha256:
            raise EvidenceIntegrityError(f"artifact sha256 mismatch: {kind}")
        media_type = record.get("media_type")
        if not isinstance(media_type, str) or not media_type:
            media_type = "application/octet-stream"
        return payload, media_type
    except EvidenceIntegrityError as exc:
        return _evidence_integrity_error(exc)
    except OSError as exc:
        if exc.errno in _ARTIFACT_INTEGRITY_ERRNOS:
            return _evidence_integrity_error(exc)
        return _evidence_unavailable(exc)


def _sha256_bytes(payload: bytes) -> str:
    import hashlib

    return hashlib.sha256(payload).hexdigest()


def _read_regular_file_bytes(path: Path) -> bytes:
    flags = os.O_RDONLY
    if hasattr(os, "O_BINARY"):
        flags |= os.O_BINARY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(path, flags)
    try:
        file_stat = os.fstat(descriptor)
        if not stat.S_ISREG(file_stat.st_mode):
            raise EvidenceIntegrityError(f"artifact is not a regular file: {path.name}")
        with os.fdopen(descriptor, "rb") as handle:
            descriptor = -1
            return handle.read()
    finally:
        if descriptor >= 0:
            os.close(descriptor)


def _list_verified_evidence(limit: int) -> dict[str, Any] | JSONResponse:
    bounded_limit = min(100, max(1, int(limit)))
    try:
        store = _evidence_store()
        requests_dir = store.requests_dir
    except OSError as exc:
        return _evidence_unavailable(exc)
    evidence: list[dict[str, Any]] = []
    integrity_failures = 0
    candidates: list[Path] = []
    try:
        if requests_dir.exists():
            candidates = sorted(
                [path for path in requests_dir.iterdir() if not path.name.startswith(".")],
                key=lambda path: path.lstat().st_mtime,
                reverse=True,
            )
    except OSError as exc:
        return _evidence_unavailable(exc)
    for candidate in candidates:
        if len(evidence) >= bounded_limit:
            break
        try:
            result = store.get(candidate.name)
        except (FileNotFoundError, EvidenceValidationError):
            continue
        except EvidenceIntegrityError:
            integrity_failures += 1
            continue
        except OSError as exc:
            return _evidence_unavailable(exc)
        evidence.append(_evidence_summary(result))
    return {
        "schema_version": "lingtu.inspection.evidence.list.v1",
        "ok": True,
        "count": len(evidence),
        "limit": bounded_limit,
        "integrity_failures": integrity_failures,
        "evidence": evidence,
        "ts": time.time(),
    }


def _route_payload(body: InspectionRouteRequest) -> dict[str, Any]:
    return {
        "id": body.id,
        "name": body.name or body.id,
        "map_id": body.map_id,
        "map_version": body.map_version,
        "revision": body.revision,
        "loop_count": body.loop_count,
        "failure_policy": body.failure_policy,
        "max_retries": body.max_retries,
        "points": [
            {
                "id": point.id,
                "x": point.x,
                "y": point.y,
                "z": point.z,
                "yaw": point.yaw,
                "position_tolerance_m": point.tolerance,
                "dwell_s": point.dwell,
                "action": point.action,
                "enabled": point.enabled,
            }
            for point in body.points
        ],
    }


def _map_version(gw: Any, map_id: str) -> int | JSONResponse:
    try:
        response = map_service_query(gw, {"action": "get_record", "name": map_id})
    except Exception as exc:
        return _error(503, "inspection_map_query_failed", str(exc))
    record = response.get("record") if isinstance(response, dict) else None
    if not isinstance(record, dict) or response.get("success") is not True:
        return _error(404, "inspection_map_not_found", f"map not found: {map_id}")
    try:
        return int(record["version"])
    except (KeyError, TypeError, ValueError):
        return _error(
            503,
            "inspection_map_version_unavailable",
            f"map record has no valid version: {map_id}",
        )


def _normalize_route(route: dict[str, Any]) -> dict[str, Any]:
    normalized = dict(route)
    points = route.get("points")
    if isinstance(points, list):
        normalized["points"] = [
            {
                **point,
                "tolerance": point.get("tolerance", point.get("position_tolerance_m", 0.35)),
                "dwell": point.get("dwell", point.get("dwell_s", 0.0)),
            }
            for point in points
            if isinstance(point, dict)
        ]
    return normalized


def _routes_from_native(payload: dict[str, Any]) -> list[dict[str, Any]]:
    routes = payload.get("routes")
    if isinstance(routes, list):
        return [_normalize_route(item) for item in routes if isinstance(item, dict)]
    route = payload.get("route")
    if isinstance(route, dict):
        return [_normalize_route(route)]
    return []


def _new_task_request_id(value: str | None) -> str:
    normalized = str(value or "").strip()
    if normalized:
        return normalized
    return f"inspection-request-{uuid.uuid4().hex}"


def _task_id_for_request(request_id: str) -> str:
    """Derive a stable product task id from an idempotent start request."""

    return f"inspection-task-{uuid.uuid5(uuid.NAMESPACE_URL, f'lingtu-inspection:{request_id}').hex}"


def _run_native_task_command(
    gw: Any,
    method: str,
    task_id: str,
    *,
    route_id: str | None = None,
    revision: int = 0,
    reason: str | None = None,
    request_id: str,
) -> None:
    if method == "start":
        operation = "start_task"
        parameters = {
            "task_id": task_id,
            "route_id": str(route_id or ""),
            "revision": revision,
            "request_id": request_id,
        }
    else:
        operation = f"{method}_task"
        parameters = {
            "task_id": task_id,
            "reason": str(reason or f"operator_{method}"),
            "request_id": request_id,
        }
    accepted = invoke_inspection(gw, operation, **parameters)
    if accepted is not True:
        raise InspectionBoundaryError(
            f"inspection service returned an invalid acknowledgement for {operation}"
        )


def _evidence_status_file() -> Path:
    return Path(
        os.environ.get(
            "LINGTU_INSPECTION_EVIDENCE_STATUS_FILE",
            _DEFAULT_EVIDENCE_STATUS_FILE,
        )
    )


def _evidence_heartbeat_max_age_s() -> float:
    raw = os.environ.get("LINGTU_INSPECTION_EVIDENCE_HEARTBEAT_MAX_AGE_S")
    if raw is None:
        return _DEFAULT_EVIDENCE_HEARTBEAT_MAX_AGE_S
    try:
        value = float(raw)
    except ValueError:
        return _DEFAULT_EVIDENCE_HEARTBEAT_MAX_AGE_S
    return value if math.isfinite(value) and value > 0 else _DEFAULT_EVIDENCE_HEARTBEAT_MAX_AGE_S


def _unready_evidence_worker(path: Path, reason: str, **extra: Any) -> dict[str, Any]:
    return {
        "ready": False,
        "reason": reason,
        "path": str(path),
        "supported_actions": [],
        **extra,
    }


def _read_evidence_worker_status(now_s: float | None = None) -> dict[str, Any]:
    path = _evidence_status_file()
    now = time.time() if now_s is None else now_s
    try:
        file_stat = path.lstat()
    except FileNotFoundError:
        return _unready_evidence_worker(path, "status_file_missing")
    except OSError as exc:
        return _unready_evidence_worker(path, "status_file_unreadable", error=str(exc))
    if not stat.S_ISREG(file_stat.st_mode):
        return _unready_evidence_worker(path, "status_file_not_regular")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        return _unready_evidence_worker(path, "status_file_invalid_json", error=str(exc))
    except OSError as exc:
        return _unready_evidence_worker(path, "status_file_unreadable", error=str(exc))
    if not isinstance(payload, dict):
        return _unready_evidence_worker(path, "status_file_not_object")

    heartbeat_raw = payload.get("heartbeat_ts", payload.get("heartbeat_s", payload.get("ts")))
    try:
        heartbeat_ts = float(heartbeat_raw)
    except (TypeError, ValueError):
        return _unready_evidence_worker(path, "heartbeat_missing")
    if not math.isfinite(heartbeat_ts) or heartbeat_ts <= 0:
        return _unready_evidence_worker(path, "heartbeat_missing")
    heartbeat_age_s = max(0.0, now - heartbeat_ts)
    max_age_s = _evidence_heartbeat_max_age_s()
    raw_actions = payload.get("supported_actions", [])
    supported_actions = [
        item for item in raw_actions
        if isinstance(item, str) and item.strip()
    ] if isinstance(raw_actions, list) else []
    base = {
        "path": str(path),
        "supported_actions": supported_actions,
        "heartbeat_ts": heartbeat_ts,
        "heartbeat_age_s": heartbeat_age_s,
        "max_heartbeat_age_s": max_age_s,
    }
    for key in (
        "worker_id",
        "status",
        "state",
        "error",
        "readiness_reason",
        "evidence_root",
        "last_error",
    ):
        value = payload.get(key)
        if isinstance(value, str) and value:
            base[key] = value
    analyzers = payload.get("analyzers")
    if isinstance(analyzers, dict):
        base["analyzers"] = {
            str(key): str(value)
            for key, value in analyzers.items()
            if isinstance(key, str) and isinstance(value, (str, int, float, bool))
        }
    if heartbeat_age_s > max_age_s:
        return {
            "ready": False,
            "reason": "heartbeat_stale",
            **base,
        }
    if payload.get("ready") is not True:
        reason = payload.get("readiness_reason")
        return {
            "ready": False,
            "reason": reason if isinstance(reason, str) and reason else "worker_not_ready",
            **base,
        }
    return {
        "ready": True,
        "reason": "ready",
        **base,
    }


def _public_evidence_worker_status(worker: dict[str, Any]) -> dict[str, Any]:
    public_fields = (
        "ready",
        "reason",
        "supported_actions",
        "heartbeat_age_s",
        "max_heartbeat_age_s",
        "worker_id",
        "status",
        "state",
        "readiness_reason",
        "analyzers",
        "last_error",
    )
    return {key: worker[key] for key in public_fields if key in worker}


def _route_evidence_actions(route: dict[str, Any]) -> list[str]:
    actions: list[str] = []
    points = route.get("points")
    if not isinstance(points, list):
        return actions
    for point in points:
        if not isinstance(point, dict) or point.get("enabled") is False:
            continue
        action = str(point.get("action") or "").strip()
        if action:
            actions.append(action)
    return sorted(set(actions))


def _validate_route_evidence_worker(route: dict[str, Any]) -> JSONResponse | None:
    actions = _route_evidence_actions(route)
    if not actions:
        return None
    worker = _read_evidence_worker_status()
    public_worker = _public_evidence_worker_status(worker)
    if worker.get("ready") is not True:
        return _error(
            503,
            "inspection_evidence_worker_unavailable",
            "inspection route has evidence actions but the evidence worker is not ready",
            detail={
                "actions": actions,
                "evidence_worker": public_worker,
            },
        )
    supported = {
        item for item in worker.get("supported_actions", [])
        if isinstance(item, str)
    }
    unsupported = [action for action in actions if action not in supported]
    if unsupported:
        return _error(
            503,
            "inspection_evidence_action_unsupported",
            "inspection route uses evidence actions unsupported by the evidence worker",
            detail={
                "actions": actions,
                "unsupported_actions": unsupported,
                "supported_actions": sorted(supported),
                "evidence_worker": public_worker,
            },
        )
    return None


def register_inspection_routes(app, gw) -> None:
    @app.get(
        "/api/v1/inspection/evidence",
        summary="List recent verified inspection evidence",
        responses={409: {"model": GatewayErrorResponse}},
    )
    async def list_inspection_evidence(limit: int = 20):
        return await asyncio.to_thread(_list_verified_evidence, limit)

    @app.get(
        "/api/v1/inspection/evidence/{evidence_id}",
        summary="Read one verified inspection evidence manifest",
        responses={404: {"model": GatewayErrorResponse}, 409: {"model": GatewayErrorResponse}},
    )
    async def get_inspection_evidence(evidence_id: str):
        result = await asyncio.to_thread(_evidence_result, evidence_id)
        if isinstance(result, JSONResponse):
            return result
        return {
            "schema_version": "lingtu.inspection.evidence.detail.v1",
            "ok": True,
            "evidence": _evidence_summary(result),
            "ts": time.time(),
        }

    @app.get(
        "/api/v1/inspection/evidence/{evidence_id}/artifacts/{kind}",
        summary="Read one verified inspection evidence artifact",
        responses={
            400: {"model": GatewayErrorResponse},
            404: {"model": GatewayErrorResponse},
            409: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def get_inspection_evidence_artifact(evidence_id: str, kind: str):
        result = await asyncio.to_thread(_evidence_result, evidence_id)
        if isinstance(result, JSONResponse):
            return result
        artifact = await asyncio.to_thread(_read_evidence_artifact, result, kind)
        if isinstance(artifact, JSONResponse):
            return artifact
        payload, media_type = artifact
        return Response(content=payload, media_type=media_type)

    @app.get(
        "/api/v1/inspection/routes",
        summary="List native inspection routes for a map",
        response_model=InspectionRouteListResponse,
        responses={400: {"model": GatewayErrorResponse}, 503: {"model": GatewayErrorResponse}},
    )
    async def list_inspection_routes(map_id: str | None = None):
        resolved = _resolve_map_id(map_id)
        if isinstance(resolved, JSONResponse):
            return resolved
        try:
            payload = await asyncio.to_thread(_run_store_operation, gw, "list", resolved)
        except InspectionBoundaryError as exc:
            return _native_error(exc)
        routes = _routes_from_native(payload)
        return {
            "schema_version": "lingtu.inspection.v1",
            "ok": True,
            "map_id": resolved,
            "routes": routes,
            "count": len(routes),
            "ts": time.time(),
        }

    @app.post(
        "/api/v1/inspection/routes",
        summary="Create or update a native inspection route",
        response_model=InspectionRouteResponse,
        responses={400: {"model": GatewayErrorResponse}, 503: {"model": GatewayErrorResponse}},
    )
    async def put_inspection_route(body: InspectionRouteRequest):
        version = await asyncio.to_thread(_map_version, gw, body.map_id)
        if isinstance(version, JSONResponse):
            return version
        if body.map_version is not None and body.map_version != version:
            return _error(
                409,
                "inspection_map_version_mismatch",
                "route map_version does not match the current map record",
                detail={"requested": body.map_version, "current": version},
            )
        payload = _route_payload(body)
        payload["map_version"] = version
        try:
            stored = await asyncio.to_thread(_run_store_operation, gw, "put", payload)
            route = _normalize_route(stored)
        except InspectionBoundaryError as exc:
            return _native_error(exc)
        return {
            "schema_version": "lingtu.inspection.v1",
            "ok": True,
            "route": route,
            "ts": time.time(),
        }

    @app.get(
        "/api/v1/inspection/routes/{route_id}",
        summary="Read one native inspection route",
        response_model=InspectionRouteResponse,
        responses={
            400: {"model": GatewayErrorResponse},
            404: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def get_inspection_route(route_id: str, map_id: str | None = None):
        resolved = _resolve_map_id(map_id)
        if isinstance(resolved, JSONResponse):
            return resolved
        try:
            route = await asyncio.to_thread(
                _run_store_operation,
                gw,
                "get",
                resolved,
                route_id,
            )
        except InspectionBoundaryError as exc:
            return _native_error(exc)
        return {
            "schema_version": "lingtu.inspection.v1",
            "ok": True,
            "route": _normalize_route(route),
            "ts": time.time(),
        }

    @app.delete(
        "/api/v1/inspection/routes/{route_id}",
        summary="Delete one native inspection route",
        response_model=InspectionCommandResponse,
        responses={
            400: {"model": GatewayErrorResponse},
            404: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def delete_inspection_route(route_id: str, map_id: str | None = None):
        resolved = _resolve_map_id(map_id)
        if isinstance(resolved, JSONResponse):
            return resolved
        try:
            await asyncio.to_thread(
                _run_store_operation,
                gw,
                "delete",
                resolved,
                route_id,
            )
        except InspectionBoundaryError as exc:
            return _native_error(exc)
        return InspectionCommandResponse(
            action="delete",
            route_id=route_id,
            map_id=resolved,
        )

    @app.get(
        "/api/v1/inspection/tasks",
        summary="List retained inspection task projections",
        response_model=InspectionTaskListResponse,
    )
    async def list_inspection_tasks(
        map_id: str | None = None,
        route_id: str | None = None,
        include_terminal: bool = False,
        limit: int = 20,
    ):
        try:
            return ensure_inspection_task_timeline(gw).list_tasks(
                map_id=map_id,
                route_id=route_id,
                include_terminal=include_terminal,
                limit=limit,
            )
        except InspectionTaskJournalUnavailable as exc:
            return _error(503, "inspection_task_journal_unavailable", str(exc))

    @app.post(
        "/api/v1/inspection/tasks",
        status_code=202,
        summary="Submit a task-addressed native inspection route",
        response_model=InspectionTaskCommandResponse,
        responses={
            400: {"model": GatewayErrorResponse},
            409: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def start_inspection_task(body: InspectionTaskStartRequest):
        task_timeline = ensure_inspection_task_timeline(gw)
        try:
            task_timeline.require_available()
        except InspectionTaskJournalUnavailable as exc:
            return _error(503, "inspection_task_journal_unavailable", str(exc))
        resolved = _resolve_map_id(None)
        if isinstance(resolved, JSONResponse):
            return resolved
        if body.map_id and body.map_id != resolved:
            return _error(
                409,
                "inspection_active_map_mismatch",
                "inspection can only start on the active map",
                detail={"requested": body.map_id, "active": resolved},
            )
        try:
            route = await asyncio.to_thread(
                _run_store_operation,
                gw,
                "get",
                resolved,
                body.route_id,
            )
        except InspectionBoundaryError as exc:
            return _native_error(exc)
        try:
            current_revision = int(route["revision"])
        except (KeyError, TypeError, ValueError):
            return _error(
                503,
                "inspection_route_revision_unavailable",
                "stored inspection route has no valid revision",
            )
        if body.revision not in (0, current_revision):
            return _error(
                409,
                "inspection_route_revision_mismatch",
                "inspection start revision does not match the stored route",
                detail={
                    "requested": body.revision,
                    "current": current_revision,
                    "route_id": body.route_id,
                },
            )
        try:
            route_snapshot = task_timeline.prepare_route_snapshot(route)
        except (TypeError, ValueError) as exc:
            return _error(
                503,
                "inspection_route_snapshot_unavailable",
                "stored inspection route cannot be preserved for task reporting",
                detail={"reason": str(exc), "route_id": body.route_id},
            )
        request_id = _new_task_request_id(body.request_id)
        task_id = _task_id_for_request(request_id)
        try:
            task_timeline.require_route_snapshot_compatible(
                task_id,
                route_snapshot,
            )
        except ValueError as exc:
            return _error(
                409,
                "inspection_task_route_snapshot_mismatch",
                "this start request already identifies a different route revision",
                detail={
                    "task_id": task_id,
                    "request_id": request_id,
                    "reason": str(exc),
                },
            )
        except InspectionTaskJournalUnavailable as exc:
            return _error(503, "inspection_task_journal_unavailable", str(exc))
        evidence_error = _validate_route_evidence_worker(route)
        if evidence_error is not None:
            return evidence_error
        try:
            task_timeline.reserve_submission(
                task_id=task_id,
                action="start",
                request_id=request_id,
                route_snapshot=route_snapshot,
            )
        except ValueError as exc:
            return _error(
                409,
                "inspection_request_id_conflict",
                "this request_id already identifies a different inspection command",
                detail={
                    "task_id": task_id,
                    "request_id": request_id,
                    "reason": str(exc),
                },
            )
        except InspectionTaskJournalUnavailable as exc:
            return _error(503, "inspection_task_journal_unavailable", str(exc))
        try:
            await asyncio.to_thread(
                _run_native_task_command,
                gw,
                "start",
                task_id,
                route_id=body.route_id,
                revision=current_revision,
                request_id=request_id,
            )
        except InspectionCommandRejected as exc:
            return _task_command_rejected(
                exc,
                action="start",
                task_id=task_id,
                request_id=request_id,
            )
        except InspectionBoundaryError as exc:
            return _error(503, "inspection_native_unavailable", str(exc))
        try:
            task_timeline.record_submission(
                task_id=task_id,
                action="start",
                request_id=request_id,
                route_id=body.route_id,
                map_id=resolved,
                map_version=int(route.get("map_version", 0) or 0),
                route_revision=current_revision,
                route_snapshot=route_snapshot,
            )
        except ValueError as exc:
            return _error(
                409,
                "inspection_task_route_snapshot_mismatch",
                "the accepted native command conflicts with the retained task identity",
                detail={
                    "task_id": task_id,
                    "request_id": request_id,
                    "reason": str(exc),
                    "native_command_accepted": True,
                    "retry_safe": False,
                },
            )
        except InspectionTaskJournalUnavailable as exc:
            return _error(
                503,
                "inspection_task_journal_commit_failed",
                str(exc),
                detail={
                    "task_id": task_id,
                    "request_id": request_id,
                    "native_command_accepted": True,
                    "retry_safe": False,
                },
            )
        # This is a business ACK only. Execution and terminal truth arrive on
        # the native task-event stream, so the receipt must never say RUNNING.
        return InspectionTaskCommandResponse(
            action="start",
            task_id=task_id,
            request_id=request_id,
            route_id=body.route_id,
            map_id=resolved,
            revision=current_revision,
        )

    async def _control_inspection_task(
        task_id: str,
        body: InspectionTaskControlRequest | None,
        *,
        action: str,
    ) -> InspectionTaskCommandResponse | JSONResponse:
        normalized_task_id = str(task_id or "").strip()
        if not normalized_task_id:
            return _error(400, "inspection_task_id_required", "inspection task_id is required")
        task_timeline = ensure_inspection_task_timeline(gw)
        try:
            task_timeline.require_available()
        except InspectionTaskJournalUnavailable as exc:
            return _error(503, "inspection_task_journal_unavailable", str(exc))
        request = body or InspectionTaskControlRequest(reason=f"operator_{action}")
        request_id = _new_task_request_id(request.request_id)
        default_reason = f"operator_{action}"
        reason = str(request.reason or default_reason).strip() or default_reason
        try:
            task_timeline.reserve_submission(
                task_id=normalized_task_id,
                action=action,
                request_id=request_id,
                reason=reason,
            )
        except ValueError as exc:
            return _error(
                409,
                "inspection_request_id_conflict",
                "this request_id already identifies a different inspection command",
                detail={
                    "task_id": normalized_task_id,
                    "request_id": request_id,
                    "action": action,
                    "reason": str(exc),
                },
            )
        except InspectionTaskJournalUnavailable as exc:
            return _error(503, "inspection_task_journal_unavailable", str(exc))
        try:
            await asyncio.to_thread(
                _run_native_task_command,
                gw,
                action,
                normalized_task_id,
                reason=reason,
                request_id=request_id,
            )
        except InspectionCommandRejected as exc:
            return _task_command_rejected(
                exc,
                action=action,
                task_id=normalized_task_id,
                request_id=request_id,
            )
        except InspectionBoundaryError as exc:
            return _error(503, "inspection_native_unavailable", str(exc))
        try:
            task_timeline.record_submission(
                task_id=normalized_task_id,
                action=action,
                request_id=request_id,
                reason=reason,
            )
        except ValueError as exc:
            return _error(
                409,
                "inspection_request_id_conflict",
                "the accepted native command conflicts with its reserved request_id",
                detail={
                    "task_id": normalized_task_id,
                    "request_id": request_id,
                    "action": action,
                    "reason": str(exc),
                    "native_command_accepted": True,
                    "retry_safe": False,
                },
            )
        except InspectionTaskJournalUnavailable as exc:
            return _error(
                503,
                "inspection_task_journal_commit_failed",
                str(exc),
                detail={
                    "task_id": normalized_task_id,
                    "request_id": request_id,
                    "native_command_accepted": True,
                    "retry_safe": False,
                },
            )
        return InspectionTaskCommandResponse(
            action=action,
            task_id=normalized_task_id,
            request_id=request_id,
        )

    @app.post(
        "/api/v1/inspection/tasks/{task_id}/pause",
        status_code=202,
        summary="Request pause for one native inspection task",
        response_model=InspectionTaskCommandResponse,
        responses={
            400: {"model": GatewayErrorResponse},
            409: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def pause_inspection_task(
        task_id: str,
        body: InspectionTaskControlRequest | None = None,
    ):
        return await _control_inspection_task(task_id, body, action="pause")

    @app.post(
        "/api/v1/inspection/tasks/{task_id}/resume",
        status_code=202,
        summary="Request resume for one native inspection task",
        response_model=InspectionTaskCommandResponse,
        responses={
            400: {"model": GatewayErrorResponse},
            409: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def resume_inspection_task(
        task_id: str,
        body: InspectionTaskControlRequest | None = None,
    ):
        return await _control_inspection_task(task_id, body, action="resume")

    @app.post(
        "/api/v1/inspection/tasks/{task_id}/cancel",
        status_code=202,
        summary="Request cancellation for one native inspection task",
        response_model=InspectionTaskCommandResponse,
        responses={
            400: {"model": GatewayErrorResponse},
            409: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def cancel_inspection_task(
        task_id: str,
        body: InspectionTaskControlRequest | None = None,
    ):
        return await _control_inspection_task(task_id, body, action="cancel")

    @app.get(
        "/api/v1/inspection/tasks/{task_id}/report",
        summary="Read the business result of one inspection task",
        response_model=InspectionTaskReportResponse,
        responses={
            404: {"model": GatewayErrorResponse},
            409: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def get_inspection_task_report(task_id: str):
        task = ensure_inspection_task_timeline(gw).query(task_id)
        if task.get("found") is not True:
            reason = str(task.get("reason") or "task_status_unknown")
            if reason in {"task_journal_corrupt", "task_journal_write_failed"}:
                return _error(503, "inspection_task_journal_unavailable", reason)
            return _error(
                404,
                "inspection_task_not_found",
                "inspection task was not found",
                detail={"task_id": task_id},
            )
        identity = task.get("identity")
        if not isinstance(identity, dict):
            identity = {}
        map_id = str(identity.get("map_id") or "")
        route_id = str(identity.get("route_id") or "")
        if not map_id or not route_id:
            return _error(
                409,
                "inspection_task_identity_incomplete",
                "inspection task has no immutable route identity",
                detail={"task_id": task_id},
            )
        route = task.get("route_snapshot")
        if not isinstance(route, dict):
            return _error(
                409,
                "inspection_task_route_snapshot_unavailable",
                "inspection task has no immutable route requirements snapshot",
                detail={
                    "task_id": task_id,
                    "route_id": route_id,
                    "route_revision": int(identity.get("route_revision") or 0),
                },
            )
        expected_revision = int(identity.get("route_revision") or 0)
        actual_revision = int(route.get("revision") or 0)
        if expected_revision <= 0 or actual_revision != expected_revision:
            return _error(
                409,
                "inspection_task_route_revision_unavailable",
                "the immutable route revision used by this task is unavailable",
                detail={
                    "task_id": task_id,
                    "route_id": route_id,
                    "expected_revision": expected_revision,
                    "available_revision": actual_revision,
                },
            )
        evidence = await asyncio.to_thread(_report_evidence_by_id, task)
        return build_inspection_task_report(task, route, evidence)

    @app.get(
        "/api/v1/inspection/tasks/{task_id}",
        summary="Read the fact-backed state of one inspection task",
        response_model=InspectionTaskStatusResponse,
    )
    async def get_inspection_task(task_id: str):
        return ensure_inspection_task_timeline(gw).query(task_id)

    @app.get(
        "/api/v1/inspection/status",
        summary="Read native inspection store/status snapshot",
        response_model=InspectionStatusResponse,
        responses={503: {"model": GatewayErrorResponse}},
    )
    async def inspection_status():
        try:
            status = await asyncio.to_thread(_run_store_operation, gw, "status")
        except InspectionBoundaryError as exc:
            return _native_error(exc)
        if not isinstance(status, dict):
            status = {"native_status": status}
        status = {
            **status,
            "evidence_worker": _public_evidence_worker_status(
                _read_evidence_worker_status()
            ),
        }
        return {
            "schema_version": "lingtu.inspection.v1",
            "ok": True,
            "status": status,
            "ts": time.time(),
        }
