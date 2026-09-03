"""Map lifecycle and viewer routes for GatewayModule."""

from __future__ import annotations

import asyncio
import logging
import pathlib
import re
import time
from datetime import datetime
from typing import Any

from fastapi import HTTPException
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse

from gateway.schemas import (
    GatewayErrorResponse,
    MapLifecycleResponse,
    MapListResponse,
    MapPointsResponse,
    MapRenameRequest,
    MapSaveOperationResponse,
    MapSaveRequest,
    PlanPreviewRequest,
)
from gateway.services.control_commands import ControlCommandService
from gateway.services.exploration import (
    ExplorationRunError,
    exploration_map_save_readiness,
    product_control_owns_explore,
)
from gateway.services.mapd_transport import (
    mapd_command,
    open_artifact,
    safe_map_name,
)
from runtime.endpoints.map_paths import map_import_root, resolve_exchange_path
from runtime.endpoints.mapd import MapClientError
from runtime.runtime_interface import normalize_frame_id
from runtime.runtime_policy import backend_capability_defaults
from runtime.utils.sanitize import sanitize_dict

logger = logging.getLogger(__name__)
MAP_VIEWER_TEMPLATE = pathlib.Path(__file__).resolve().parents[1] / "templates" / "map_viewer.html"
_PUBLIC_MAP_STATES = frozenset(
    {
        "CREATED",
        "STALE",
        "READY",
        "ACTIVE",
        "RETIRED",
        "FAILED",
    }
)
_RESERVED_PUBLIC_MAP_NAMES = frozenset({"operations"})

MAP_OPERATION_ID_PATTERN = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_-]{0,127}$")
_INTERNAL_PATH_PATTERN = re.compile(
    r"(?<![A-Za-z0-9:])/(?:[^/\s\"'<>]+/)*[^/\s\"'<>]+"
    r"|(?<![A-Za-z0-9])[A-Za-z]:[\\/][^\s\"'<>]+",
    re.IGNORECASE,
)
_EXTERNAL_OPERATION_STATUS_FIELDS = frozenset(
    {
        "operation_id",
        "request_id",
        "map_id",
        "name",
        "state",
        "phase",
        "progress",
        "reason",
        "reason_code",
        "message",
        "created_at",
        "updated_at",
        "completed_at",
        "started_at",
        "finished_at",
        "created_at_ns",
        "updated_at_ns",
        "completed_at_ns",
        "started_at_ns",
        "finished_at_ns",
        "ts",
        "cancel_requested",
        "recovered",
        "replayed",
        "attempt",
    }
)
_EXTERNAL_OPERATION_RESPONSE_FIELDS = _EXTERNAL_OPERATION_STATUS_FIELDS | frozenset(
    {
        "schema_version",
        "ok",
        "success",
        "accepted",
        "status",
        "operation",
        "operations",
        "result",
        "count",
        "can_activate",
        "occupancy_ok",
        "octomap_ok",
        "metadata_ok",
        "semantic_ok",
        "point_count",
        "slam_profile",
        "map_save_source",
    }
)
_EXTERNAL_OPERATION_MESSAGES = {
    "artifact_failed": "Map artifact generation failed.",
    "cancelled": "Map save was cancelled.",
    "canceled": "Map save was cancelled.",
    "invalid_job_id": "Invalid map-save operation ID.",
    "invalid_map_name": "Invalid map name.",
    "job_not_found": "Map-save operation was not found.",
    "map_save_failed": "Map save failed.",
    "map_save_unsupported": "Map save is not supported by the active SLAM backend.",
    "map_service_unavailable": "Native mapd is unavailable.",
    "save_job_in_progress": "Map save is in progress.",
    "save_job_timeout": "Map save is still in progress.",
}
_PUBLIC_OPERATION_REASON_CODES = {
    "invalid_job_id": "invalid_operation_id",
    "job_not_found": "operation_not_found",
    "save_job_in_progress": "map_save_in_progress",
    "save_job_timeout": "map_save_in_progress",
}


def public_map_name_error(name: str) -> str | None:
    """Validate a map name at the HTTP boundary, including reserved routes."""

    error = safe_map_name(name)
    if error is not None:
        return error
    if name in _RESERVED_PUBLIC_MAP_NAMES:
        return f"map name is reserved by the public API: {name}"
    return None


def map_lifecycle_payload(success: bool, **fields: Any) -> dict[str, Any]:
    """Build the stable response envelope used by map lifecycle routes."""

    payload = {
        "schema_version": 1,
        "ok": bool(success),
        "success": bool(success),
        "ts": time.time(),
    }
    payload.update({key: value for key, value in fields.items() if value is not None})
    return payload


def _external_operation_scalar(value: Any) -> Any:
    """Return one JSON scalar after removing local filesystem details."""

    if value is None or isinstance(value, (bool, int, float)):
        return value
    if isinstance(value, str):
        return _INTERNAL_PATH_PATTERN.sub("[internal path]", value)
    return None


def _external_operation_message(payload: dict[str, Any]) -> str:
    """Derive a stable customer message without echoing native text."""

    reason_code = str(payload.get("reason_code") or "").strip().lower()
    if reason_code in _EXTERNAL_OPERATION_MESSAGES:
        return _EXTERNAL_OPERATION_MESSAGES[reason_code]
    state = str(payload.get("state") or payload.get("status") or "").strip().upper()
    if state in {"QUEUED", "RUNNING"} or payload.get("accepted") is True:
        return "Map save is in progress."
    if state == "SUCCEEDED" or payload.get("success") is True:
        return "Map save completed."
    if state in {"CANCELLED", "CANCELED"}:
        return "Map save was cancelled."
    if state in {"FAILED", "ERROR", "REJECTED"} or payload.get("success") is False:
        return "Map save failed."
    return "Map save status updated."


def _external_operation_reason_code(value: Any) -> str | None:
    reason_code = str(value or "").strip().lower()
    if not reason_code:
        return None
    return _PUBLIC_OPERATION_REASON_CODES.get(reason_code, reason_code)


def _sanitize_external_operation_status(value: Any) -> dict[str, Any] | None:
    """Project one native SaveMap status onto the customer-safe contract."""

    if not isinstance(value, dict):
        return None
    sanitized: dict[str, Any] = {}
    operation_id = _external_operation_scalar(value.get("operation_id") or value.get("job_id"))
    if isinstance(operation_id, str) and operation_id:
        sanitized["operation_id"] = operation_id
    for key in _EXTERNAL_OPERATION_STATUS_FIELDS:
        if key not in value:
            continue
        if key == "reason_code":
            reason_code = _external_operation_reason_code(value[key])
            if reason_code is not None:
                sanitized[key] = reason_code
            continue
        if key == "message":
            sanitized[key] = _external_operation_message(value)
            continue
        scalar = _external_operation_scalar(value[key])
        if scalar is not None:
            sanitized[key] = scalar
    if "message" not in sanitized and "reason_code" in sanitized:
        sanitized["message"] = _external_operation_message(value)
    return sanitized


def _sanitize_external_operation_payload(payload: dict[str, Any]) -> dict[str, Any]:
    """Recursively whitelist the public SaveMap response.

    Native SaveMap status deliberately contains local working directories and
    artifact reports used for recovery.  Those fields are implementation
    details and must never cross the customer HTTP seam.
    """

    sanitized: dict[str, Any] = {}
    operation_id = _external_operation_scalar(payload.get("operation_id") or payload.get("job_id"))
    if isinstance(operation_id, str) and operation_id:
        sanitized["operation_id"] = operation_id
    operation_source = (
        payload.get("operation")
        if "operation" in payload
        else payload.get("job")
        if "job" in payload
        else payload.get("status")
        if isinstance(payload.get("status"), dict)
        else None
    )
    operation = _sanitize_external_operation_status(operation_source)
    if operation is not None:
        sanitized["operation"] = operation
    operations = payload.get("operations") if "operations" in payload else payload.get("jobs")
    if isinstance(operations, list):
        sanitized["operations"] = [
            status for item in operations if (status := _sanitize_external_operation_status(item)) is not None
        ]
    for key in _EXTERNAL_OPERATION_RESPONSE_FIELDS:
        if key not in payload:
            continue
        value = payload[key]
        if value is None:
            sanitized[key] = None
            continue
        if key == "status" and isinstance(value, dict):
            # Native mapd names the operation record `status`. Public callers
            # receive one stable `operation` object instead.
            continue
        if key == "operation" and isinstance(value, dict):
            status = _sanitize_external_operation_status(value)
            if status is not None:
                sanitized[key] = status
            continue
        if key == "reason_code":
            reason_code = _external_operation_reason_code(value)
            if reason_code is not None:
                sanitized[key] = reason_code
            continue
        if key == "message":
            sanitized[key] = _external_operation_message(payload)
            continue
        scalar = _external_operation_scalar(value)
        if scalar is not None:
            sanitized[key] = scalar
    if "message" not in sanitized and "reason_code" in sanitized:
        sanitized["message"] = _external_operation_message(payload)
    return sanitized


def _external_operation_json_response(
    payload: dict[str, Any],
    *,
    status_code: int,
) -> JSONResponse:
    return JSONResponse(
        _sanitize_external_operation_payload(payload),
        status_code=status_code,
    )


def _external_operation_lifecycle_response(
    success: bool,
    *,
    status_code: int,
    **fields: Any,
) -> JSONResponse:
    return _external_operation_json_response(
        map_lifecycle_payload(success, **fields),
        status_code=status_code,
    )


def _map_lifecycle_response(
    success: bool,
    *,
    status_code: int,
    **fields: Any,
) -> JSONResponse:
    return JSONResponse(
        map_lifecycle_payload(success, **fields),
        status_code=status_code,
    )


def _customer_map_lifecycle_response(
    resp: dict[str, Any],
    *,
    name: str,
    status_code: int,
    success_message: str,
    failure_message: str,
    reason_code: str | None = None,
    **fields: Any,
) -> JSONResponse:
    """Project a native map mutation onto the public lifecycle envelope."""

    ok = resp.get("success") is True
    public_reason = str(reason_code or resp.get("reason_code") or "").strip()
    if re.fullmatch(r"[a-z][a-z0-9_]{0,63}", public_reason) is None:
        public_reason = ""
    return _map_lifecycle_response(
        ok,
        status_code=status_code,
        name=name,
        reason_code=public_reason or None,
        message=success_message if ok else failure_message,
        **fields,
    )


def _external_operation_service_response(
    resp: dict[str, Any],
    *,
    status_code: int,
    **defaults: Any,
) -> JSONResponse:
    payload = dict(resp)
    ok = payload.get("success") is True
    payload.pop("success", None)
    payload.pop("ok", None)
    payload.pop("schema_version", None)
    for key, value in defaults.items():
        payload.setdefault(key, value)
    return _external_operation_lifecycle_response(
        ok,
        status_code=status_code,
        **payload,
    )


def _mapd_command(gw: Any, cmd: dict[str, Any]) -> dict[str, Any]:
    try:
        return mapd_command(gw, cmd)
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc


def _register_save_operation_routes(app, gw) -> None:
    """Register the public map-save operation API."""

    @app.get(
        "/api/v1/maps/operations",
        summary="List durable map-save operations",
        response_model=MapSaveOperationResponse,
    )
    async def list_map_operations(limit: int = 100):
        resp = await asyncio.to_thread(
            _mapd_command,
            gw,
            {"action": "list_save_map_jobs", "limit": max(1, min(limit, 1000))},
        )
        return _external_operation_service_response(
            resp,
            status_code=200 if resp.get("success") is True else 500,
        )
    @app.get(
        "/api/v1/maps/operations/{operation_id}",
        summary="Get a durable map-save operation",
        response_model=MapSaveOperationResponse,
    )
    async def get_map_operation(operation_id: str):
        if MAP_OPERATION_ID_PATTERN.fullmatch(operation_id) is None:
            return _external_operation_lifecycle_response(
                False,
                status_code=400,
                reason_code="invalid_operation_id",
                message="invalid map operation id",
            )
        resp = await asyncio.to_thread(
            _mapd_command,
            gw,
            {"action": "get_save_map_status", "job_id": operation_id},
        )
        return _external_operation_service_response(
            resp,
            status_code=200 if resp.get("success") is True else 404,
        )

    @app.post(
        "/api/v1/maps/operations/{operation_id}/cancel",
        summary="Cancel a durable map-save operation",
        response_model=MapSaveOperationResponse,
    )
    async def cancel_map_operation(operation_id: str):
        if MAP_OPERATION_ID_PATTERN.fullmatch(operation_id) is None:
            return _external_operation_lifecycle_response(
                False,
                status_code=400,
                reason_code="invalid_operation_id",
                message="invalid map operation id",
            )
        resp = await asyncio.to_thread(
            _mapd_command,
            gw,
            {"action": "cancel_save_map", "job_id": operation_id},
        )
        accepted = resp.get("accepted") is True
        return _external_operation_service_response(
            {**resp, "success": accepted},
            status_code=200 if accepted else 409,
        )

    @app.post(
        "/api/v1/maps/operations/{operation_id}/retry",
        summary="Retry a failed durable map-save operation",
        response_model=MapSaveOperationResponse,
    )
    async def retry_map_operation(operation_id: str):
        if MAP_OPERATION_ID_PATTERN.fullmatch(operation_id) is None:
            return _external_operation_lifecycle_response(
                False,
                status_code=400,
                reason_code="invalid_operation_id",
                message="invalid map operation id",
            )
        resp = await asyncio.to_thread(
            _mapd_command,
            gw,
            {"action": "retry_save_map", "job_id": operation_id},
        )
        accepted = resp.get("accepted") is True
        return _external_operation_service_response(
            {**resp, "success": accepted},
            status_code=202 if accepted else 409,
        )


def register_map_routes(app, gw) -> None:
    """Register saved-map query and lifecycle routes on the Gateway app."""

    command_service = ControlCommandService(gw)
    _register_save_operation_routes(app, gw)

    @app.get(
        "/api/v1/slam/maps",
        summary="List maps through native mapd",
        response_model=MapListResponse,
        responses={503: {"model": GatewayErrorResponse}},
    )
    async def slam_maps():
        resp = await asyncio.to_thread(
            _mapd_command, gw, {"action": "list_maps"}
        )
        if resp.get("success") is not True:
            return JSONResponse(
                status_code=503,
                content=GatewayErrorResponse(
                    error="map_service_unavailable",
                    message="Native mapd is unavailable.",
                ).model_dump(mode="json"),
            )
        active_candidate = str(resp.get("active") or "")
        active_target = (
            active_candidate if public_map_name_error(active_candidate) is None else ""
        )
        maps = []
        for item in resp.get("maps") if isinstance(resp.get("maps"), list) else []:
            if not isinstance(item, dict):
                continue
            name = str(item.get("name") or "")
            if public_map_name_error(name) is not None:
                continue
            if type(item.get("can_activate")) is not bool:
                return JSONResponse(
                    status_code=503,
                    content=GatewayErrorResponse(
                        error="map_service_contract_invalid",
                        message="Native mapd returned invalid can_activate.",
                    ).model_dump(mode="json"),
                )
            maps.append(
                {
                    "name": name,
                    "has_pcd": bool(item.get("has_pcd")),
                    "has_occupancy": bool(item.get("has_occupancy")),
                    "has_octomap": bool(item.get("has_octomap")),
                    "can_activate": item["can_activate"],
                    "state": (
                        state
                        if (state := str(item.get("state") or "").strip().upper())
                        in _PUBLIC_MAP_STATES
                        else None
                    ),
                    "is_active": bool(item.get("is_active")) or name == active_target,
                    "size_mb": item.get("size_mb"),
                    "patch_count": int(item.get("patch_count") or 0),
                }
            )
        return {
            "schema_version": 1,
            "maps": maps,
            "count": len(maps),
            "active": active_target,
            "ts": time.time(),
        }

    @app.delete(
        "/api/v1/maps/{name}",
        summary="Delete a saved map",
        response_model=MapLifecycleResponse,
        responses={
            400: {"model": MapLifecycleResponse},
            404: {"model": MapLifecycleResponse},
            409: {"model": MapLifecycleResponse},
            503: {"model": MapLifecycleResponse},
        },
    )
    async def delete_saved_map(name: str):
        error = public_map_name_error(name)
        if error is not None:
            return _map_lifecycle_response(
                False,
                status_code=400,
                name=name,
                reason_code="invalid_map_name",
                message=error,
            )
        try:
            resp = await asyncio.to_thread(
                _mapd_command,
                gw,
                {"action": "delete_map", "map_id": name},
            )
        except HTTPException as exc:
            return _map_lifecycle_response(
                False,
                status_code=int(exc.status_code),
                name=name,
                reason_code="map_service_unavailable",
                message="Native mapd is unavailable.",
            )
        ok = resp.get("success") is True
        missing = not ok and (
            str(resp.get("reason_code") or "") == "map_not_found"
            or "not found" in str(resp.get("message") or "").lower()
        )
        return _customer_map_lifecycle_response(
            resp,
            name=name,
            status_code=200 if ok else 404 if missing else 409,
            success_message="Map deleted.",
            failure_message="Map was not found." if missing else "Map could not be deleted.",
            reason_code="map_not_found" if missing else "map_delete_failed" if not ok else None,
        )

    @app.post(
        "/api/v1/maps/import_pcd",
        summary="Import a PCD file into a LingTu map package",
        response_model=MapLifecycleResponse,
    )
    async def import_pcd_map(body: dict[str, Any]):
        payload = body
        name = str(payload.get("name") or "")
        err = public_map_name_error(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        try:
            source_path = resolve_exchange_path(
                payload.get("source_path") or payload.get("path") or "",
                root=map_import_root(),
                must_exist=True,
                require_file=True,
                suffixes=(".pcd",),
            )
        except ValueError as exc:
            return _map_lifecycle_response(False, message=str(exc), status_code=400)
        resp = _mapd_command(
            gw,
            {
                "action": "import_pcd",
                "map_id": name,
                "source_path": str(source_path),
                "voxel_size": payload.get("voxel_size", 0.0),
                "bounds": payload.get("bounds"),
            },
        )
        ok = resp.get("success") is True
        return _customer_map_lifecycle_response(
            resp,
            name=name,
            status_code=200 if ok else 400,
            success_message="Map imported.",
            failure_message="Map could not be imported.",
        )

    @app.post(
        "/api/v1/maps/{name}/crop",
        summary="Crop a saved map point cloud and invalidate derived artifacts",
        response_model=MapLifecycleResponse,
    )
    async def crop_saved_map(name: str, body: dict[str, Any]):
        err = public_map_name_error(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        payload = body
        resp = _mapd_command(
            gw,
            {
                "action": "crop_pcd",
                "map_id": name,
                "bounds": payload.get("bounds") or payload,
                "invert": payload.get("invert", False),
                "voxel_size": payload.get("voxel_size", 0.0),
            },
        )
        ok = resp.get("success") is True
        if ok:
            gw.clear_map_cloud_cache(reason="saved_map_crop")
        return _customer_map_lifecycle_response(
            resp,
            name=name,
            status_code=200 if ok else 400,
            success_message="Map cropped.",
            failure_message="Map could not be cropped.",
            live_cloud_reset=ok,
        )

    @app.post(
        "/api/v1/maps/{name}/mark_zone",
        summary="Mark occupied/free/preblocked/traversable zones in the saved OctoMap",
        response_model=MapLifecycleResponse,
    )
    async def mark_saved_map_zone(name: str, body: dict[str, Any]):
        err = public_map_name_error(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        cmd = dict(body or {})
        cmd["action"] = "edit_octomap_voxels"
        cmd["map_id"] = name
        resp = _mapd_command(gw, cmd)
        ok = resp.get("success") is True
        return _customer_map_lifecycle_response(
            resp,
            name=name,
            status_code=200 if ok else 400,
            success_message="Map zone updated.",
            failure_message="Map zone could not be updated.",
        )

    @app.post(
        "/api/v1/maps/{name}/build_occupancy",
        summary="Build a 2D occupancy artifact from a saved map",
        response_model=MapLifecycleResponse,
        responses={
            400: {"model": MapLifecycleResponse},
            404: {"model": MapLifecycleResponse},
            409: {"model": MapLifecycleResponse},
            503: {"model": MapLifecycleResponse},
        },
    )
    async def build_saved_map_occupancy(name: str):
        error = public_map_name_error(name)
        if error is not None:
            return _map_lifecycle_response(
                False,
                status_code=400,
                name=name,
                reason_code="invalid_map_name",
                message=error,
            )
        try:
            resp = await asyncio.to_thread(
                _mapd_command,
                gw,
                {"action": "build_occupancy_snapshot", "map_id": name},
            )
        except HTTPException as exc:
            return _map_lifecycle_response(
                False,
                status_code=int(exc.status_code),
                name=name,
                reason_code="map_service_unavailable",
                message="Native mapd is unavailable.",
                occupancy_ok=False,
            )
        ok = resp.get("success") is True
        missing = not ok and (
            str(resp.get("reason_code") or "") == "map_not_found"
            or "not found" in str(resp.get("message") or "").lower()
        )
        return _customer_map_lifecycle_response(
            resp,
            name=name,
            status_code=200 if ok else 404 if missing else 409,
            success_message="Occupancy map built.",
            failure_message=(
                "Map was not found." if missing else "Occupancy map could not be built."
            ),
            reason_code=(
                "map_not_found"
                if missing
                else "occupancy_build_failed"
                if not ok
                else None
            ),
            occupancy_ok=ok,
        )

    @app.post(
        "/api/v1/maps/{name}/build_octomap",
        summary="Build OctoPlanner3D octomap.ot from saved map.pcd",
        response_model=MapLifecycleResponse,
    )
    async def build_saved_map_octomap(name: str):
        err = public_map_name_error(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        resp = _mapd_command(gw, {"action": "build_octomap_artifact", "map_id": name})
        ok = resp.get("success") is True
        return _customer_map_lifecycle_response(
            resp,
            name=name,
            status_code=200 if ok else 400,
            success_message="OctoMap built.",
            failure_message="OctoMap could not be built.",
            octomap_ok=ok,
        )

    @app.post(
        "/api/v1/maps/{name}/validate_plan",
        summary="Preview a route on the active saved map without publishing a goal",
    )
    async def validate_saved_map_plan(name: str, body: PlanPreviewRequest):
        err = public_map_name_error(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        active_resp = await asyncio.to_thread(
            _mapd_command,
            gw,
            {"action": "get_active_map"},
        )
        active = str(active_resp.get("active") or "")
        if active != name:
            return _map_lifecycle_response(
                False,
                message=f"map must be active before validate_plan: {name}",
                active=active,
                motion_published=False,
                status_code=409,
            )
        payload = await asyncio.to_thread(command_service.preview_plan, body)
        payload.update({"map_id": name, "active": active, "motion_published": False})
        return sanitize_dict(payload)

    @app.get(
        "/api/v1/maps/{name}/pcd",
        summary="Serve raw PCD file for inline preview",
        responses={200: {"content": {"application/octet-stream": {"schema": {"type": "string", "format": "binary"}}}}},
    )
    async def get_map_pcd(name: str):
        error = public_map_name_error(name)
        if error is not None:
            raise HTTPException(
                status_code=400,
                detail={
                    "reason_code": "invalid_map_id",
                    "message": "Invalid map ID.",
                },
            )
        try:
            artifact = await asyncio.to_thread(
                open_artifact,
                gw,
                name,
                "source_pointcloud",
            )
        except MapClientError as exc:
            not_found = {
                "artifact_not_found",
                "map_not_found",
                "missing_artifact",
                "missing_capability",
            }
            unavailable = {
                "connection_closed",
                "fd_passing_unsupported",
                "mapd_unavailable",
            }
            if exc.reason_code in not_found:
                status = 404
                reason_code = exc.reason_code
                message = "Map artifact was not found."
            elif exc.reason_code == "map_write_in_progress":
                status = 409
                reason_code = "map_write_in_progress"
                message = "Map artifact is being updated."
            elif exc.reason_code in unavailable:
                status = 503
                reason_code = "mapd_unavailable"
                message = "Map artifact service is unavailable."
            else:
                status = 502
                reason_code = "map_artifact_unavailable"
                message = "Map artifact is unavailable."
            logger.warning(
                "mapd artifact open failed for map %r (%s): %s",
                name,
                exc.reason_code,
                exc,
            )
            raise HTTPException(
                status_code=status,
                detail={"reason_code": reason_code, "message": message},
            ) from exc
        except (OSError, RuntimeError) as exc:
            logger.warning("mapd artifact endpoint unavailable for map %r: %s", name, exc)
            raise HTTPException(
                status_code=503,
                detail={
                    "reason_code": "mapd_unavailable",
                    "message": "Map artifact service is unavailable.",
                },
            ) from exc

        headers = {
            "Content-Length": str(artifact.size_bytes),
            "Content-Disposition": 'attachment; filename="map.pcd"',
        }
        try:
            body = artifact.iter_bytes()
        except Exception:
            artifact.close()
            raise
        return StreamingResponse(
            body,
            media_type=artifact.media_type,
            headers=headers,
        )

    @app.get(
        "/api/v1/maps/{name}/points",
        summary="Saved map point cloud as JSON",
        response_model=MapPointsResponse,
    )
    async def get_saved_map_points(name: str, max_points: int = 30000):
        error = public_map_name_error(name)
        if error is not None:
            raise HTTPException(
                status_code=400,
                detail={
                    "reason_code": "invalid_map_id",
                    "message": "Invalid map ID.",
                },
            )
        active_map_before = gw._active_map_from_mapd()
        if active_map_before != name:
            raise HTTPException(
                status_code=409,
                detail=(
                    "Saved map is not the authoritative active map: "
                    f"requested {name}, active {active_map_before or 'none'}"
                ),
            )
        scene_identity_before = gw._cloud_viewer.scene_identity()
        if scene_identity_before.get("map_id") != name:
            raise HTTPException(
                status_code=409,
                detail="Live map scene is not bound to the requested saved map; retry",
            )
        resp = _mapd_command(
            gw,
            {
                "action": "get_map_points",
                "map_id": name,
                "max_points": max(0, int(max_points or 0)),
            },
        )
        if resp.get("success") is not True:
            reason = str(resp.get("reason_code") or "")
            status = 404 if reason in {"map_not_found", "map_pcd_not_found"} else 400
            raise HTTPException(
                status_code=status,
                detail=str(resp.get("message") or f"Map not found: {name}"),
            )
        response_map_id = str(resp.get("map_id") or "").strip()
        if response_map_id != name:
            raise HTTPException(
                status_code=409,
                detail=(f"Saved map identity mismatch: expected {name}, got {response_map_id or 'unknown'}"),
            )
        frame_id = normalize_frame_id(str(resp.get("frame_id") or ""))
        if frame_id is None:
            raise HTTPException(
                status_code=409,
                detail=f"Saved map frame_id unavailable: {name}",
            )
        content_epoch = resp.get("content_epoch")
        if isinstance(content_epoch, bool) or not isinstance(content_epoch, int) or content_epoch <= 0:
            raise HTTPException(
                status_code=409,
                detail=f"Saved map content_epoch unavailable: {name}",
            )
        scene_identity = gw._cloud_viewer.scene_identity()
        active_map_after = gw._active_map_from_mapd()
        if (
            active_map_after != name
            or scene_identity.get("map_id") != name
            or int(scene_identity_before["protocol_version"]) != int(scene_identity["protocol_version"])
            or int(scene_identity_before["epoch"]) != int(scene_identity["epoch"])
        ):
            raise HTTPException(
                status_code=409,
                detail="Map scene changed while saved map points were loading; retry",
            )
        return {
            "schema_version": 1,
            "protocol_version": int(scene_identity["protocol_version"]),
            "count": int(resp.get("returned") or 0),
            "layout": "xyz_rows",
            "frame_id": frame_id,
            "epoch": int(scene_identity["epoch"]),
            "sequence": int(scene_identity["sequence"]),
            "stamp_s": float(scene_identity["stamp_s"]),
            "stream_kind": "map",
            "source": "mapd",
            "name": response_map_id,
            "content_epoch": content_epoch,
            "points": resp.get("points") if isinstance(resp.get("points"), list) else [],
            "ts": time.time(),
        }

    @app.post(
        "/api/v1/maps/{name}/voxels/edit",
        summary="Edit saved OctoMap voxels for OctoPlanner3D",
        response_model=MapLifecycleResponse,
        responses={
            400: {"model": MapLifecycleResponse},
            404: {"model": MapLifecycleResponse},
            503: {"model": MapLifecycleResponse},
        },
    )
    async def edit_saved_map_voxels(name: str, body: dict[str, Any]):
        err = public_map_name_error(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        cmd = dict(body or {})
        cmd["action"] = "edit_octomap_voxels"
        cmd["map_id"] = name
        resp = _mapd_command(gw, cmd)
        if resp.get("success") is not True:
            message = str(resp.get("message") or "voxel edit failed")
            code = 404 if "not found" in message.lower() else 400
            return _map_lifecycle_response(
                False,
                message=message,
                detail=resp,
                status_code=code,
            )
        legacy = dict(resp)
        legacy.pop("success", None)
        return map_lifecycle_payload(True, **legacy)

    @app.get(
        "/api/v1/maps/{name}/voxels/edits",
        summary="Saved OctoMap voxel edit overlay",
    )
    async def get_saved_map_voxel_edits(name: str):
        err = public_map_name_error(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        resp = _mapd_command(
            gw,
            {"action": "get_voxel_edits", "map_id": name},
        )
        if resp.get("success") is not True:
            message = str(resp.get("message") or "voxel edit overlay unavailable")
            code = 404 if resp.get("reason_code") == "map_not_found" else 400
            return _map_lifecycle_response(
                False,
                message=message,
                status_code=code,
            )
        return {
            "schema_version": 1,
            "ok": True,
            "success": True,
            "map_id": name,
            "edits": resp.get("edits") if isinstance(resp.get("edits"), list) else [],
        }

    @app.get(
        "/api/v1/map/points",
        summary="Map point cloud as JSON (from ikd-tree snapshot)",
        response_model=MapPointsResponse,
    )
    async def get_map_points(max_points: int = 80000):
        return gw._cloud_viewer.map_points_snapshot(max_points=max_points)

    @app.post(
        "/api/v1/map_cloud/reset",
        summary="Clear accumulated map cloud (viz only, SLAM ikd-tree untouched)",
        response_model=MapLifecycleResponse,
    )
    async def reset_map_cloud():
        gw.clear_map_cloud_cache(reason="manual_map_cloud_reset")
        return map_lifecycle_payload(
            True,
            message="Accumulated map cloud cleared",
        )

    @app.get("/map/viewer", summary="Interactive 3D map viewer")
    async def map_viewer(map: str = ""):
        if map:
            error = public_map_name_error(map)
            if error is not None:
                raise HTTPException(status_code=400, detail=error)
        return FileResponse(
            MAP_VIEWER_TEMPLATE,
            media_type="text/html; charset=utf-8",
            headers={"Cache-Control": "public, max-age=300"},
        )

    @app.post(
        "/api/v1/map/rename",
        summary="Rename a saved map",
        response_model=MapLifecycleResponse,
        responses={
            400: {"model": MapLifecycleResponse},
            404: {"model": MapLifecycleResponse},
            409: {"model": MapLifecycleResponse},
            500: {"model": MapLifecycleResponse},
        },
    )
    async def rename_map(body: MapRenameRequest):
        payload = body
        old = payload.get("old_name", "")
        new = payload.get("new_name", "")
        err_old = public_map_name_error(old)
        err_new = public_map_name_error(new)
        if err_old or err_new:
            return _map_lifecycle_response(
                False,
                message=err_old or err_new,
                status_code=400,
            )
        try:
            resp = _mapd_command(
                gw,
                {"action": "rename_map", "map_id": old, "new_map_id": new},
            )
        except HTTPException as exc:
            return _map_lifecycle_response(
                False,
                message=str(exc.detail),
                status_code=int(exc.status_code),
            )
        message = str(resp.get("message") or "").lower()
        status_code = (
            200
            if resp.get("success") is True
            else (404 if "not found" in message else 409 if "exists" in message else 500)
        )
        return _customer_map_lifecycle_response(
            resp,
            name=new,
            status_code=status_code,
            success_message="Map renamed.",
            failure_message="Map could not be renamed.",
            old_name=old,
            new_name=new,
        )

    @app.post(
        "/api/v1/map/save",
        summary="Save current SLAM map",
        response_model=MapSaveOperationResponse,
        responses={
            202: {"model": MapSaveOperationResponse},
            400: {"model": MapSaveOperationResponse},
            503: {"model": MapSaveOperationResponse},
            409: {"model": MapSaveOperationResponse},
            500: {"model": MapSaveOperationResponse},
        },
    )
    async def save_map_now(body: MapSaveRequest = MapSaveRequest()):
        payload = body.model_dump()
        name = payload.get("name", "")
        if not name:
            name = "map_" + datetime.now().strftime("%Y%m%d_%H%M%S")
        err = public_map_name_error(name)
        if err is not None:
            return _external_operation_lifecycle_response(
                False,
                message=err,
                reason_code="invalid_map_name",
                status_code=400,
            )
        if product_control_owns_explore(gw):
            try:
                save_readiness = exploration_map_save_readiness(gw)
            except ExplorationRunError as exc:
                return _external_operation_lifecycle_response(
                    False,
                    status_code=exc.status_code,
                    name=name,
                    reason_code=exc.code,
                    message=str(exc),
                )
            if not save_readiness["can_save"]:
                return _external_operation_lifecycle_response(
                    False,
                    status_code=409,
                    name=name,
                    reason_code=str(save_readiness["reason"]),
                    message=str(save_readiness["message"]),
                    exploration_run_id=save_readiness.get("exploration_run_id"),
                    exploration_state=save_readiness.get("exploration_state"),
                )
        try:
            slam_profile = gw._get_slam_profile()
        except Exception:
            return _external_operation_lifecycle_response(
                False,
                status_code=503,
                name=name,
                reason_code="slam_profile_unavailable",
                message="Current SLAM profile is unavailable.",
            )
        capability = backend_capability_defaults(slam_profile)
        if capability.get("map_save_supported") is not True:
            return _external_operation_lifecycle_response(
                False,
                status_code=409,
                name=name,
                reason_code="map_save_unsupported",
                message=f"map save is not supported by SLAM backend: {slam_profile}",
                slam_profile=slam_profile,
                map_save_source=capability.get("map_save_source", "unknown"),
            )

        try:
            resp = await asyncio.to_thread(
                _mapd_command,
                gw,
                {
                    "action": "save_map",
                    "map_id": name,
                    "slam_profile": slam_profile,
                    "request_id": payload.get("request_id"),
                },
            )
        except HTTPException as exc:
            return _external_operation_lifecycle_response(
                False,
                status_code=int(exc.status_code),
                name=name,
                reason_code="map_service_unavailable",
                message=str(exc.detail),
            )
        ok = resp.get("success") is True
        native_status = resp.get("status") if isinstance(resp.get("status"), dict) else {}
        operation_id = str(
            resp.get("operation_id")
            or resp.get("job_id")
            or native_status.get("job_id")
            or ""
        ).strip()
        accepted = bool(operation_id) and (
            resp.get("accepted") is True
            or str(resp.get("status") or "").strip().lower() == "running"
        )
        if accepted:
            accepted_payload = dict(resp)
            accepted_payload.pop("success", None)
            accepted_payload.pop("ok", None)
            accepted_payload["accepted"] = True
            accepted_payload["operation_id"] = operation_id
            if accepted_payload.get("reason_code") == "save_job_timeout":
                accepted_payload["reason_code"] = "save_job_in_progress"
            payload = map_lifecycle_payload(
                True,
                name=name,
                **accepted_payload,
            )
            # Accepted is neither terminal success nor terminal failure.
            payload["success"] = None
            return _external_operation_json_response(payload, status_code=202)
        message_lower = str(resp.get("message", "")).lower()
        status_code = 200 if ok else 409 if "unsupported" in message_lower or "not supported" in message_lower else 500
        return _external_operation_service_response(
            resp,
            status_code=status_code,
            name=name,
        )
