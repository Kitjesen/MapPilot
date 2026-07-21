"""Inspection route product APIs backed by native C++ adapters."""

from __future__ import annotations

import asyncio
import errno
import json
import math
import os
import stat
import time
from pathlib import Path
from typing import Any

from fastapi.responses import JSONResponse, Response

from gateway.schemas import (
    GatewayErrorResponse,
    InspectionCommandResponse,
    InspectionRouteListResponse,
    InspectionRouteRequest,
    InspectionRouteResponse,
    InspectionRunControlRequest,
    InspectionStartRequest,
    InspectionStatusResponse,
)
from gateway.services.map_service import map_service_query
from gateway.services.inspection_boundary import (
    InspectionBoundaryError,
    invoke_inspection,
)
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


def _run_native_command(gw: Any, method: str, *args, **kwargs) -> None:
    parameters = dict(kwargs)
    if method == "start":
        parameters.update(route_id=args[0])
        operation = "start_route"
    else:
        parameters.update(reason=args[0])
        operation = method
    invoke_inspection(gw, operation, **parameters)


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

    @app.post(
        "/api/v1/inspection/routes/{route_id}/start",
        summary="Start native C++ inspection execution",
        response_model=InspectionCommandResponse,
        responses={400: {"model": GatewayErrorResponse}, 503: {"model": GatewayErrorResponse}},
    )
    async def start_inspection_route(route_id: str, body: InspectionStartRequest | None = None):
        request = body or InspectionStartRequest()
        resolved = _resolve_map_id(None)
        if isinstance(resolved, JSONResponse):
            return resolved
        if request.map_id and request.map_id != resolved:
            return _error(
                409,
                "inspection_active_map_mismatch",
                "inspection can only start on the active map",
                detail={"requested": request.map_id, "active": resolved},
            )
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
        try:
            current_revision = int(route["revision"])
        except (KeyError, TypeError, ValueError):
            return _error(
                503,
                "inspection_route_revision_unavailable",
                "stored inspection route has no valid revision",
            )
        if request.revision not in (0, current_revision):
            return _error(
                409,
                "inspection_route_revision_mismatch",
                "inspection start revision does not match the stored route",
                detail={
                    "requested": request.revision,
                    "current": current_revision,
                    "route_id": route_id,
                },
            )
        evidence_error = _validate_route_evidence_worker(route)
        if evidence_error is not None:
            return evidence_error
        try:
            await asyncio.to_thread(
                _run_native_command,
                gw,
                "start",
                route_id,
                revision=current_revision,
                request_id=request.request_id,
            )
        except InspectionBoundaryError as exc:
            return _error(503, "inspection_native_unavailable", str(exc))
        return InspectionCommandResponse(
            action="start",
            route_id=route_id,
            map_id=resolved,
            revision=current_revision,
            request_id=request.request_id,
        )

    @app.post(
        "/api/v1/inspection/run/pause",
        summary="Pause native C++ inspection execution",
        response_model=InspectionCommandResponse,
        responses={400: {"model": GatewayErrorResponse}, 503: {"model": GatewayErrorResponse}},
    )
    async def pause_inspection_run(body: InspectionRunControlRequest | None = None):
        request = body or InspectionRunControlRequest(reason="operator_pause")
        resolved = _resolve_map_id(None)
        if isinstance(resolved, JSONResponse):
            return resolved
        try:
            await asyncio.to_thread(
                _run_native_command,
                gw,
                "pause",
                request.reason,
                request_id=request.request_id,
            )
        except InspectionBoundaryError as exc:
            return _error(503, "inspection_native_unavailable", str(exc))
        return InspectionCommandResponse(
            action="pause",
            map_id=resolved,
            request_id=request.request_id,
        )

    @app.post(
        "/api/v1/inspection/run/resume",
        summary="Resume native C++ inspection execution",
        response_model=InspectionCommandResponse,
        responses={400: {"model": GatewayErrorResponse}, 503: {"model": GatewayErrorResponse}},
    )
    async def resume_inspection_run(body: InspectionRunControlRequest | None = None):
        request = body or InspectionRunControlRequest(reason="operator_resume")
        resolved = _resolve_map_id(None)
        if isinstance(resolved, JSONResponse):
            return resolved
        try:
            await asyncio.to_thread(
                _run_native_command,
                gw,
                "resume",
                request.reason,
                request_id=request.request_id,
            )
        except InspectionBoundaryError as exc:
            return _error(503, "inspection_native_unavailable", str(exc))
        return InspectionCommandResponse(
            action="resume",
            map_id=resolved,
            request_id=request.request_id,
        )

    @app.post(
        "/api/v1/inspection/run/cancel",
        summary="Cancel native C++ inspection execution",
        response_model=InspectionCommandResponse,
        responses={400: {"model": GatewayErrorResponse}, 503: {"model": GatewayErrorResponse}},
    )
    async def cancel_inspection_run(body: InspectionRunControlRequest | None = None):
        request = body or InspectionRunControlRequest(reason="operator_cancel")
        resolved = _resolve_map_id(None)
        if isinstance(resolved, JSONResponse):
            return resolved
        try:
            await asyncio.to_thread(
                _run_native_command,
                gw,
                "cancel",
                request.reason,
                request_id=request.request_id,
            )
        except InspectionBoundaryError as exc:
            return _error(503, "inspection_native_unavailable", str(exc))
        return InspectionCommandResponse(
            action="cancel",
            map_id=resolved,
            request_id=request.request_id,
        )

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
