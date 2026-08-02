"""Map lifecycle and viewer routes for GatewayModule."""

from __future__ import annotations

import asyncio
import logging
import math
import os
import pathlib
import re
import time
from collections.abc import Mapping
from datetime import datetime
from typing import Any

from fastapi import HTTPException
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse

from gateway.schemas import (
    GatewayErrorResponse,
    MapLifecycleResponse,
    MapListResponse,
    MapNameRequest,
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
from gateway.services.map_service import (
    activate_runtime_map,
    map_runtime_lock,
    map_service_command,
    open_artifact,
)
from gateway.services.runtime_switch_plan import build_operator_command
from maps.client import MapClientError
from maps.paths import map_import_root, resolve_exchange_path
from maps.services.storage import safe_map_name
from runtime.runtime_interface import TOPICS, normalize_frame_id, topic_default_frame_id
from runtime.runtime_policy import backend_capability_defaults
from runtime.utils.sanitize import sanitize_dict

logger = logging.getLogger(__name__)
MAX_EXECUTABLE_START_SNAP_M = 0.5
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
        "map_version",
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
        "compatibility_ready",
        "compatibility_message",
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
        "navigation_ready",
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
    "map_service_unavailable": "Map service is unavailable.",
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


def _robot_mesh_path(filename: str) -> pathlib.Path | None:
    """Resolve a robot mesh, preferring the field Thunder V4 asset set."""

    safe_name = os.path.basename(filename)
    configured = os.environ.get("DOG_MESH_DIR")
    repo_root = pathlib.Path(__file__).resolve().parents[3]
    candidates = [
        pathlib.Path(configured).expanduser() if configured else None,
        repo_root / "sim" / "robots" / "thunderv4" / "meshes",
        repo_root / "products" / "quadruped_ws" / "dog_arm" / "meshes",
    ]
    for mesh_dir in candidates:
        if mesh_dir is None:
            continue
        path = mesh_dir / safe_name
        if path.is_file():
            return path
    return None


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
      float() / int(); see slam_relocalize (x, y, yaw) and bag_start
      (duration) for examples.
    """
    if hasattr(body, "model_dump"):
        return body.model_dump(exclude_none=True)
    if not isinstance(body, dict):
        raise TypeError(f"expected dict or Pydantic model, got {type(body).__name__}")
    return body


def _clear_gateway_map_cloud(gw: Any, reason: str) -> None:
    clear_cache = getattr(gw, "clear_map_cloud_cache", None)
    if callable(clear_cache):
        clear_cache(reason=reason)
        return
    raise RuntimeError("gateway map cloud cache service is unavailable")


def _preview_path_xyz(preview: dict[str, Any]) -> list[list[float]]:
    out: list[list[float]] = []
    for point in preview.get("path") or []:
        if isinstance(point, dict):
            raw = (point.get("x"), point.get("y"), point.get("z", 0.0))
        else:
            raw = point
        try:
            x, y, z = float(raw[0]), float(raw[1]), float(raw[2])
        except (TypeError, ValueError, IndexError):
            continue
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            out.append([x, y, z])
    return out


def _evaluate_preview_live_safety(
    command_service: ControlCommandService,
    preview: dict[str, Any],
) -> dict[str, Any] | None:
    path = _preview_path_xyz(preview)
    if not path:
        return None
    try:
        return command_service.evaluate_navigation_path(path)
    except Exception as exc:
        logger.debug("live path safety preview failed: %s", exc, exc_info=True)
        return {
            "ok": False,
            "reason": "live_path_safety_error",
            "error": str(exc),
        }


def _preview_reaches_requested_goal(preview: dict[str, Any]) -> bool:
    if preview.get("feasible") is not True:
        return False
    if preview.get("reached_goal") is True:
        return True
    if preview.get("reached_goal") is False:
        return False
    global_plan = preview.get("global_plan")
    if isinstance(global_plan, dict) and global_plan.get("reached_goal") is True:
        return True
    if isinstance(global_plan, dict) and global_plan.get("reached_goal") is False:
        return False
    if preview.get("adjusted_goal") is not None:
        return False
    return True


def _mark_preview_target_unreached(preview: dict[str, Any]) -> None:
    reasons = [str(reason) for reason in (preview.get("reasons") or [])]
    if preview.get("adjusted_goal") is not None:
        reason = "goal_adjusted"
    else:
        reason = "goal_not_reached"
    if reason not in reasons:
        reasons.append(reason)
    preview["feasible"] = False
    preview["reasons"] = reasons
    preview["fallback_reason"] = preview.get("fallback_reason") or "planner did not reach the requested target"


def _append_preview_reason(preview: dict[str, Any], reason: str) -> None:
    reasons = [str(item) for item in (preview.get("reasons") or [])]
    if reason not in reasons:
        reasons.append(reason)
    preview["reasons"] = reasons


def _preview_start_snap_too_large(preview: dict[str, Any]) -> bool:
    snap = preview.get("snap_diagnostics")
    if not isinstance(snap, dict):
        return False
    try:
        distance = float(
            snap.get("start_snap_xy_distance_m")
            if snap.get("start_snap_xy_distance_m") is not None
            else snap.get("start_snap_distance_m")
        )
    except (TypeError, ValueError):
        return False
    return math.isfinite(distance) and distance > MAX_EXECUTABLE_START_SNAP_M


def _point_xyz(value: Any) -> tuple[float, float, float] | None:
    if isinstance(value, dict):
        raw = (value.get("x"), value.get("y"), value.get("z", 0.0))
    else:
        raw = value
    try:
        xyz = (float(raw[0]), float(raw[1]), float(raw[2]))
    except (TypeError, ValueError, IndexError):
        return None
    if all(math.isfinite(item) for item in xyz):
        return xyz
    return None


def _pose_map_mismatch_hint(preview: dict[str, Any]) -> dict[str, Any]:
    start = _point_xyz(preview.get("start"))
    goal = _point_xyz(preview.get("goal"))
    if start is None or goal is None:
        return {}
    dx = start[0] - goal[0]
    dy = start[1] - goal[1]
    delta_xy = math.hypot(dx, dy)
    max_abs = max(
        abs(start[0]),
        abs(start[1]),
        abs(start[2]),
        abs(goal[0]),
        abs(goal[1]),
        abs(goal[2]),
    )
    if delta_xy < 1000.0 and max_abs < 1000.0:
        return {}
    return {
        "diagnostic_codes": ["pose_map_mismatch"],
        "likely_cause": "live pose or requested goal is not in the active saved-map frame",
        "start_goal_xy_delta_m": round(delta_xy, 3),
    }


def _planner_failure_summary(preview: dict[str, Any]) -> dict[str, Any] | None:
    rejected = preview.get("rejected_plans") or []
    first_rejected = rejected[0] if rejected and isinstance(rejected[0], dict) else {}
    diagnostics = first_rejected.get("planner_diagnostics") or {}
    if not isinstance(diagnostics, dict):
        diagnostics = {}

    text = "\n".join(
        str(value or "")
        for value in (
            first_rejected.get("reason"),
            preview.get("fallback_reason"),
            preview.get("error"),
            diagnostics.get("error_message"),
            diagnostics.get("stdout"),
            diagnostics.get("stderr"),
        )
    )
    lowered = text.lower()
    reason = ""
    message = ""
    endpoint = ""
    reasons = {str(item) for item in (preview.get("reasons") or [])}
    path_safety = preview.get("path_safety")
    if "start_snap_too_large" in reasons:
        reason = "start_snap_too_large"
        message = "planner start snapped too far from the live robot pose"
        endpoint = "start"
        mismatch_hint = {}
    elif (
        "path_safety_failed" in reasons
        or "live path_safety failed" in lowered
        or (isinstance(path_safety, dict) and path_safety.get("ok") is False)
    ):
        reason = "live_path_safety_failed"
        message = "live path safety rejected the map-only path"
        endpoint = "path"
        mismatch_hint = {}
    elif "start is occupied/out of map" in lowered:
        reason = "start_occupied_or_out_of_map"
        message = "start is occupied/out of map and no nearby free cell"
        endpoint = "start"
        mismatch_hint = _pose_map_mismatch_hint(preview)
    elif "goal is occupied/out of map" in lowered:
        reason = "goal_occupied_or_out_of_map"
        message = "goal is occupied/out of map and no nearby free cell"
        endpoint = "goal"
        mismatch_hint = {}
    elif "a* planning failed" in lowered:
        reason = "astar_failed"
        message = "octoplanner3d a_star planning failed"
        endpoint = "path"
        mismatch_hint = {
            "diagnostic_codes": ["no_traversable_path"],
            "likely_cause": (
                "start and goal were accepted by OctoPlanner3D, but no traversable occupancy path connected them"
            ),
        }
    elif preview.get("fallback_reason") or preview.get("error"):
        reason = "planner_failed"
        message = str(preview.get("fallback_reason") or preview.get("error") or "")
        mismatch_hint = {}
    else:
        return None

    summary = {
        "reason": reason,
        "message": message,
        "planner": preview.get("selected_planner") or preview.get("planner"),
        "runtime_mode": diagnostics.get("runtime_mode"),
        "process_boundary": diagnostics.get("process_boundary"),
        "executable_path": diagnostics.get("executable_path"),
        "runtime_map_path": diagnostics.get("runtime_map_path") or diagnostics.get("map_path"),
        "returncode": diagnostics.get("returncode"),
        "start": preview.get("start"),
        "goal": preview.get("goal"),
        "endpoint": endpoint or None,
    }
    snap = preview.get("snap_diagnostics")
    if isinstance(snap, dict):
        summary["snap_diagnostics"] = snap
    if isinstance(path_safety, dict):
        summary["path_safety"] = path_safety
    summary.update(mismatch_hint)
    return summary


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
    map_version = _external_operation_scalar(
        value.get("map_version") if "map_version" in value else value.get("version")
    )
    if isinstance(map_version, int) and map_version >= 0:
        sanitized["map_version"] = map_version
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
    map_version = _external_operation_scalar(
        payload.get("map_version") if "map_version" in payload else payload.get("version")
    )
    if isinstance(map_version, int) and map_version >= 0:
        sanitized["map_version"] = map_version
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


def _map_service_lifecycle_response(
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
    return _map_lifecycle_response(ok, status_code=status_code, **payload)


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


def _map_service_command(gw: Any, cmd: dict[str, Any]) -> dict[str, Any]:
    try:
        return map_service_command(gw, cmd)
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc


def _map_navigation_ready(item: dict[str, Any]) -> bool:
    if "navigation_ready" in item:
        return bool(item.get("navigation_ready"))
    # Legacy providers predate the authoritative readiness field.  The active
    # field planner is OctoPlanner3D, so compatibility must stay conservative:
    # 2-D occupancy, ESDF, or traversability alone cannot satisfy its map gate.
    return bool(item.get("has_pcd")) and bool(item.get("has_octomap"))


def _register_save_operation_routes(app, gw) -> None:
    """Register the public map-save operation API."""

    @app.get(
        "/api/v1/maps/operations",
        summary="List durable map-save operations",
        response_model=MapSaveOperationResponse,
    )
    async def list_map_operations(limit: int = 100):
        resp = await asyncio.to_thread(
            _map_service_command,
            gw,
            {"action": "list_save_jobs", "limit": max(1, min(limit, 1000))},
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
            _map_service_command,
            gw,
            {"action": "save_status", "job_id": operation_id},
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
            _map_service_command,
            gw,
            {"action": "cancel_save", "job_id": operation_id},
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
            _map_service_command,
            gw,
            {"action": "retry_save", "job_id": operation_id},
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
        summary="List maps through the native maps service",
        response_model=MapListResponse,
        responses={503: {"model": GatewayErrorResponse}},
    )
    async def slam_maps():
        resp = await asyncio.to_thread(
            _map_service_command, gw, {"action": "list"}
        )
        if resp.get("success") is not True:
            return JSONResponse(
                status_code=503,
                content=GatewayErrorResponse(
                    error="map_service_unavailable",
                    message="Map service is unavailable.",
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
            maps.append(
                {
                    "name": name,
                    "has_pcd": bool(item.get("has_pcd")),
                    "has_occupancy": bool(item.get("has_occupancy")),
                    "has_octomap": bool(item.get("has_octomap")),
                    "navigation_ready": _map_navigation_ready(item),
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
                _map_service_command,
                gw,
                {"action": "delete", "name": name},
            )
        except HTTPException as exc:
            return _map_lifecycle_response(
                False,
                status_code=int(exc.status_code),
                name=name,
                reason_code="map_service_unavailable",
                message="Map service is unavailable.",
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
        payload = _body_mapping(body)
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
        resp = _map_service_command(
            gw,
            {
                "action": "import_pcd",
                "name": name,
                "source_path": str(source_path),
                "voxel_size": payload.get("voxel_size", 0.0),
                "bounds": payload.get("bounds"),
            },
        )
        ok = resp.get("success") is True
        return _map_service_lifecycle_response(resp, status_code=200 if ok else 400)

    @app.post(
        "/api/v1/maps/{name}/crop",
        summary="Crop a saved map point cloud and invalidate derived artifacts",
        response_model=MapLifecycleResponse,
    )
    async def crop_saved_map(name: str, body: dict[str, Any]):
        err = public_map_name_error(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        payload = _body_mapping(body)
        resp = _map_service_command(
            gw,
            {
                "action": "crop",
                "name": name,
                "bounds": payload.get("bounds") or payload,
                "invert": payload.get("invert", False),
                "voxel_size": payload.get("voxel_size", 0.0),
            },
        )
        ok = resp.get("success") is True
        if ok:
            _clear_gateway_map_cloud(gw, "saved_map_crop")
            resp["live_cloud_reset"] = True
        return _map_service_lifecycle_response(resp, status_code=200 if ok else 400)

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
        cmd["action"] = "edit_voxels"
        cmd["name"] = name
        resp = _map_service_command(gw, cmd)
        ok = resp.get("success") is True
        return _map_service_lifecycle_response(resp, status_code=200 if ok else 400)

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
                _map_service_command,
                gw,
                {"action": "build_occupancy", "name": name},
            )
        except HTTPException as exc:
            return _map_lifecycle_response(
                False,
                status_code=int(exc.status_code),
                name=name,
                reason_code="map_service_unavailable",
                message="Map service is unavailable.",
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
        resp = _map_service_command(gw, {"action": "build_octomap", "name": name})
        ok = resp.get("success") is True
        return _map_service_lifecycle_response(resp, status_code=200 if ok else 400)

    @app.post(
        "/api/v1/maps/{name}/validate_plan",
        summary="No-motion OctoPlanner3D route preview for the active saved map",
    )
    async def validate_saved_map_plan(name: str, body: PlanPreviewRequest):
        err = public_map_name_error(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        validation = _map_service_command(
            gw,
            {
                "action": "validate_artifacts",
                "name": name,
                "require_octomap": True,
                "expected_frame_id": topic_default_frame_id(TOPICS.saved_map_cloud),
            },
        )
        if validation.get("success") is not True:
            message = str(validation.get("message") or "map validation unavailable")
            code = 404 if validation.get("reason_code") == "map_not_found" else 400
            return _map_lifecycle_response(False, message=message, status_code=code)
        gate = dict(validation.get("gate") or {})
        gate["required"] = True
        active_resp = _map_service_command(gw, {"action": "get_active"})
        active = str(active_resp.get("active") or "")
        if gate.get("ok") is not True:
            return _map_lifecycle_response(
                False,
                message="saved map artifact gate failed",
                artifact_gate=gate,
                no_motion_gate={
                    "name": "saved_map_validate_plan",
                    "map_only": True,
                    "motion_published": False,
                    "blocked": True,
                    "blockers": list(gate.get("blockers") or []),
                    "required_artifacts": ["map_pcd", "octomap"],
                },
                motion_published=False,
                status_code=409,
            )
        if active != name:
            return _map_lifecycle_response(
                False,
                message=f"map must be active before validate_plan: {name}",
                active=active,
                no_motion_gate={
                    "name": "saved_map_validate_plan",
                    "map_only": True,
                    "motion_published": False,
                    "blocked": True,
                    "blockers": ["active_map_mismatch"],
                    "required_artifacts": ["map_pcd", "octomap"],
                },
                motion_published=False,
                status_code=409,
            )
        ignored_blockers = {
            "navigation_session_inactive",
            "real_runtime_evidence_missing_or_stale",
            "safety_stop",
        }
        preview = await asyncio.to_thread(
            command_service.preview_navigation_plan,
            body,
            ignore_blockers=ignored_blockers,
            map_only=True,
        )
        executable_preview = dict(preview)
        executable_preview["source"] = "map_only_path_with_live_safety_overlay"
        executable_path_safety = None
        target_reached = _preview_reaches_requested_goal(preview)
        if bool(preview.get("feasible", False)):
            if not target_reached:
                _mark_preview_target_unreached(executable_preview)
            if _preview_start_snap_too_large(preview):
                executable_preview["feasible"] = False
                _append_preview_reason(executable_preview, "start_snap_too_large")
                executable_preview["fallback_reason"] = (
                    executable_preview.get("fallback_reason") or "planner start snapped too far from live pose"
                )
            executable_path_safety = _evaluate_preview_live_safety(command_service, preview)
            executable_preview["path_safety"] = executable_path_safety
            if isinstance(executable_path_safety, dict) and executable_path_safety.get("ok") is False:
                executable_preview["feasible"] = False
                _append_preview_reason(executable_preview, "path_safety_failed")
                executable_preview["fallback_reason"] = (
                    executable_preview.get("fallback_reason") or "live path_safety failed"
                )
        else:
            executable_preview["path_safety"] = None
        executable_ok = bool(executable_preview.get("feasible", False))
        map_plan_ok = bool(preview.get("feasible", False)) and target_reached
        no_motion_blockers = list(executable_preview.get("reasons") or [])
        planner_failure = _planner_failure_summary(executable_preview)
        if planner_failure:
            failure_reason = str(planner_failure.get("reason") or "")
            if failure_reason and failure_reason not in no_motion_blockers:
                no_motion_blockers.append(failure_reason)
            for diagnostic_code in planner_failure.get("diagnostic_codes") or []:
                code = str(diagnostic_code)
                if code and code not in no_motion_blockers:
                    no_motion_blockers.append(code)
        payload = {
            "schema_version": 1,
            "ok": executable_ok,
            "success": executable_ok,
            "map_id": name,
            "active": active,
            "artifact_gate": gate,
            "preview": preview,
            "executable_preview": executable_preview,
            "map_plan_ok": map_plan_ok,
            "executable_feasible": executable_ok,
            "live_path_safety": executable_path_safety,
            "no_motion_gate": {
                "name": "saved_map_validate_plan",
                "map_only": True,
                "motion_published": False,
                "blocked": not executable_ok,
                "blockers": no_motion_blockers,
                "ignored_readiness_blockers": sorted(ignored_blockers),
                "required_artifacts": ["map_pcd", "octomap"],
                "selected_planner": preview.get("selected_planner"),
                "fallback_reason": executable_preview.get("fallback_reason", ""),
                "planner_failure": planner_failure,
                "snap_diagnostics": preview.get("snap_diagnostics"),
                "preview_feasible": bool(preview.get("feasible", False)),
                "target_reached": target_reached,
                "live_safety_blocked": map_plan_ok and not executable_ok,
                "executable_feasible": executable_ok,
            },
            "motion_published": False,
            "ts": time.time(),
        }
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
        if re.fullmatch(r"[0-9a-fA-F]{64}", artifact.sha256):
            headers["ETag"] = f'"sha256:{artifact.sha256.lower()}"'
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
        active_map_before = gw._active_map_from_maps_service()
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
        resp = _map_service_command(
            gw,
            {
                "action": "get_map_points",
                "name": name,
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
        scene_identity = gw._cloud_viewer.scene_identity()
        active_map_after = gw._active_map_from_maps_service()
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
            "source": "maps_service",
            "name": response_map_id,
            "version_id": str(resp.get("version_id") or "") or None,
            "map_pcd_sha256": str(resp.get("map_pcd_sha256") or "") or None,
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
        cmd["action"] = "edit_voxels"
        cmd["name"] = name
        resp = _map_service_command(gw, cmd)
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
        resp = _map_service_command(
            gw,
            {"action": "get_voxel_edits", "name": name},
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
        _clear_gateway_map_cloud(gw, "manual_map_cloud_reset")
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

    @app.get("/robot/meshes/{filename}", summary="Serve robot STL mesh files")
    async def serve_robot_mesh(filename: str):
        safe_name = os.path.basename(filename)
        path = _robot_mesh_path(safe_name)
        if path is None:
            return JSONResponse(
                status_code=404,
                content={"error": "mesh not found", "name": safe_name},
            )
        return FileResponse(
            str(path),
            media_type="application/octet-stream",
            headers={
                "Access-Control-Allow-Origin": "*",
                "Cache-Control": "public, max-age=3600",
            },
        )

    @app.post(
        "/api/v1/map/restore_predufo",
        summary="Restore map.pcd from pre-clean backup",
        response_model=MapLifecycleResponse,
        responses={
            400: {"model": MapLifecycleResponse},
            404: {"model": MapLifecycleResponse},
            500: {"model": MapLifecycleResponse},
        },
    )
    async def restore_predufo(body: MapNameRequest):
        payload = _body_mapping(body)
        name = payload.get("name", "")
        err = public_map_name_error(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        try:
            resp = _map_service_command(
                gw,
                {"action": "restore_source", "name": name},
            )
        except HTTPException as exc:
            return _map_lifecycle_response(
                False,
                message=str(exc.detail),
                status_code=int(exc.status_code),
            )
        reason = str(resp.get("reason_code") or "")
        status_code = (
            200
            if resp.get("success") is True
            else (404 if reason in {"map_not_found", "source_backup_missing"} else 500)
        )
        if resp.get("success") is True:
            _clear_gateway_map_cloud(gw, "saved_map_restore_source")
            resp["live_cloud_reset"] = True
        return _map_service_lifecycle_response(resp, status_code=status_code, name=name)

    @app.post(
        "/api/v1/map/activate",
        summary="Activate a saved map in a module-controlled runtime",
        description=(
            "Local/development compatibility route. ProductControl-managed Products "
            "must switch maps through ProductControl and receive HTTP 409 here."
        ),
        response_model=MapLifecycleResponse,
        responses={
            400: {"model": MapLifecycleResponse},
            404: {"model": MapLifecycleResponse},
            409: {"model": MapLifecycleResponse},
            500: {"model": MapLifecycleResponse},
        },
    )
    async def activate_map(body: MapNameRequest):
        payload = _body_mapping(body)
        name = payload.get("name", "")
        err = public_map_name_error(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        run_plan = getattr(gw, "_compiled_run_plan", None)
        process_control_value = (
            run_plan.get("process_control")
            if isinstance(run_plan, Mapping)
            else getattr(run_plan, "process_control", "")
        )
        process_control = str(process_control_value or "").strip()
        if process_control == "systemd":
            current_product_value = (
                run_plan.get("product")
                if isinstance(run_plan, Mapping)
                else getattr(run_plan, "product", "")
            )
            env_value = (
                run_plan.get("env")
                if isinstance(run_plan, Mapping)
                else getattr(run_plan, "env", "")
            )
            current_product = str(current_product_value or "").strip()
            env = str(env_value or "").strip()
            operator_command = None
            if env in {"real", "sim"}:
                switch_request: dict[str, Any] = {
                    "target_product": "nav",
                    "map_name": name,
                    "relocalize": True,
                }
                if current_product:
                    switch_request["current_product"] = current_product
                try:
                    operator_command = build_operator_command(switch_request, env=env)
                except ValueError:
                    operator_command = None
            return _map_lifecycle_response(
                False,
                status_code=409,
                reason_code="product_map_switch_required",
                message=(
                    "Field map selection is owned by ProductControl; preview with "
                    "/api/v1/runtime/switch-plan, then run the returned operator command."
                ),
                requested_map=name,
                switch_plan="/api/v1/runtime/switch-plan",
                operator_command=operator_command,
            )
        try:
            resp = activate_runtime_map(
                gw,
                name,
                command_service.reload_navigation_map,
            )
        except RuntimeError as exc:
            return _map_lifecycle_response(
                False,
                message=str(exc),
                status_code=503,
            )
        ok = resp.get("success") is True
        reason = str(resp.get("reason_code") or "")
        if not ok and reason == "map_not_found":
            status_code = 404
        elif not ok:
            status_code = 409
        else:
            status_code = 200
        transaction = resp.get("transaction")
        transaction_state = str(transaction.get("state") or "") if isinstance(transaction, dict) else ""
        if transaction_state not in {"", "rejected"} and resp.get("unchanged") is not True:
            _clear_gateway_map_cloud(gw, "map_activation")
            resp["live_cloud_reset"] = True
        return _map_service_lifecycle_response(resp, status_code=status_code)

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
        payload = _body_mapping(body)
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
            resp = _map_service_command(
                gw,
                {"action": "rename", "name": old, "new_name": new},
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
        return _map_service_lifecycle_response(
            resp,
            status_code=status_code,
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
        payload = _body_mapping(body)
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
                _map_service_command,
                gw,
                {
                    "action": "save",
                    "name": name,
                    "slam_profile": slam_profile,
                    "optimization": payload.get("optimization"),
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
        accepted_running = resp.get("status") == "running" and bool(str(resp.get("job_id") or "").strip())
        if accepted_running:
            accepted_payload = dict(resp)
            accepted_payload.pop("success", None)
            accepted_payload.pop("ok", None)
            accepted_payload["accepted"] = True
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

    @app.get(
        "/api/v1/maps/{name}/versions",
        summary="List verified immutable map versions",
        response_model=MapLifecycleResponse,
    )
    async def list_map_versions(name: str):
        err = public_map_name_error(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        resp = _map_service_command(
            gw,
            {"action": "list_map_versions", "name": name},
        )
        return _map_service_lifecycle_response(
            resp,
            status_code=200 if resp.get("success") is True else 409,
        )

    @app.post(
        "/api/v1/maps/{name}/versions/{version}/rollback",
        summary="Atomically roll a map back to a verified version",
        response_model=MapLifecycleResponse,
    )
    async def rollback_map_version(name: str, version: int):
        err = public_map_name_error(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        if version <= 0:
            return _map_lifecycle_response(
                False,
                message="version must be a positive integer",
                status_code=400,
            )
        with map_runtime_lock():
            active_response = _map_service_command(gw, {"action": "get_active"})
            if active_response.get("success") is not True:
                return _map_lifecycle_response(
                    False,
                    status_code=409,
                    reason_code="active_map_query_failed",
                    message=str(active_response.get("message") or "Failed to read active map."),
                )
            active_map = str(active_response.get("active") or "").strip()
            if active_map == name:
                return _map_lifecycle_response(
                    False,
                    status_code=409,
                    name=name,
                    active=active_map,
                    reason_code="active_map_version_transaction_required",
                    message=(
                        "The active map version cannot change independently of localization "
                        "and planning; activate another map first or use a full runtime transaction."
                    ),
                )
            resp = _map_service_command(
                gw,
                {"action": "rollback_map_version", "name": name, "version": version},
            )
        return _map_service_lifecycle_response(
            resp,
            status_code=200 if resp.get("success") is True else 409,
        )
