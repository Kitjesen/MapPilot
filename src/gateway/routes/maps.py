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
from fastapi.responses import FileResponse, JSONResponse

from gateway.schemas import (
    MapLifecycleResponse,
    MapListResponse,
    MapNameRequest,
    MapPointsResponse,
    MapRenameRequest,
    MapSaveRequest,
    PlanPreviewRequest,
)
from gateway.services.control_commands import ControlCommandService
from gateway.services.map_service import artifact_path, map_service_command
from maps.paths import map_import_root, resolve_exchange_path
from maps.services.storage import safe_map_name
from runtime.runtime_interface import TOPICS, normalize_frame_id, topic_default_frame_id
from runtime.runtime_policy import backend_capability_defaults
from runtime.utils.sanitize import sanitize_dict

logger = logging.getLogger(__name__)
MAX_EXECUTABLE_START_SNAP_M = 0.5
MAP_VIEWER_TEMPLATE = pathlib.Path(__file__).resolve().parents[1] / "templates" / "map_viewer.html"
SAVE_JOB_ID_PATTERN = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_-]{0,127}$")


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
    payload = {
        "schema_version": 1,
        "ok": bool(success),
        "success": bool(success),
        "ts": time.time(),
    }
    payload.update({key: value for key, value in fields.items() if value is not None})
    return payload


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


def register_map_routes(app, gw) -> None:
    command_service = ControlCommandService(gw)

    @app.get(
        "/api/v1/slam/maps",
        summary="List maps through the native maps service",
        response_model=MapListResponse,
    )
    async def slam_maps():
        resp = _map_service_command(gw, {"action": "list"})
        active_target = str(resp.get("active") or "")
        maps = []
        for item in resp.get("maps") if isinstance(resp.get("maps"), list) else []:
            if not isinstance(item, dict):
                continue
            name = str(item.get("name") or "")
            if not name:
                continue
            maps.append(
                {
                    "name": name,
                    "has_pcd": bool(item.get("has_pcd")),
                    "has_occupancy": bool(item.get("has_occupancy")),
                    "has_octomap": bool(item.get("has_octomap")),
                    "navigation_ready": _map_navigation_ready(item),
                    "state": item.get("state"),
                    "is_active": bool(item.get("is_active")) or name == active_target,
                    "size_mb": item.get("size_mb"),
                    "patch_count": int(item.get("patch_count") or 0),
                    "record": item.get("record"),
                    "artifacts": item.get("artifacts"),
                    "capabilities": item.get("capabilities"),
                    "health": item.get("health"),
                    "map_classes": item.get("map_classes"),
                    "has_esdf": bool(item.get("has_esdf")),
                    "has_traversability": bool(item.get("has_traversability")),
                    "source": "maps_service",
                }
            )
        return {
            "schema_version": 1,
            "maps": maps,
            "count": len(maps),
            "active": active_target,
            "map_dir": str(resp.get("map_dir") or ""),
            "source": "maps_service",
            "ts": time.time(),
        }

    @app.post(
        "/api/v1/maps/import_pcd",
        summary="Import a PCD file into a LingTu map package",
        response_model=MapLifecycleResponse,
    )
    async def import_pcd_map(body: dict[str, Any]):
        payload = _body_mapping(body)
        name = str(payload.get("name") or "")
        err = safe_map_name(name)
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
        err = safe_map_name(name)
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
        err = safe_map_name(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        cmd = dict(body or {})
        cmd["action"] = "edit_voxels"
        cmd["name"] = name
        resp = _map_service_command(gw, cmd)
        ok = resp.get("success") is True
        return _map_service_lifecycle_response(resp, status_code=200 if ok else 400)

    @app.post(
        "/api/v1/maps/{name}/build_octomap",
        summary="Build OctoPlanner3D octomap.ot from saved map.pcd",
        response_model=MapLifecycleResponse,
    )
    async def build_saved_map_octomap(name: str):
        err = safe_map_name(name)
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
        err = safe_map_name(name)
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
        resp = _map_service_command(
            gw,
            {
                "action": "get_map_bundle",
                "name": name,
                "capability": "source_pointcloud",
            },
        )
        if resp.get("success") is not True:
            reason = str(resp.get("reason_code") or "")
            status = 404 if reason in {"map_not_found", "missing_capability"} else 400
            raise HTTPException(
                status_code=status,
                detail=str(resp.get("message") or f"No PCD for map: {name}"),
            )
        pcd_path = artifact_path(gw, name, "source_pointcloud")
        if pcd_path is None:
            raise HTTPException(status_code=404, detail=f"No PCD for map: {name}")
        return FileResponse(
            str(pcd_path),
            media_type="application/octet-stream",
            filename="map.pcd",
        )

    @app.get(
        "/api/v1/maps/{name}/points",
        summary="Saved map point cloud as JSON",
        response_model=MapPointsResponse,
    )
    async def get_saved_map_points(name: str, max_points: int = 30000):
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
        err = safe_map_name(name)
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
        err = safe_map_name(name)
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
            safe_map_name(map)
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
        err = safe_map_name(name)
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
        summary="Set active map (symlink)",
        response_model=MapLifecycleResponse,
        responses={
            400: {"model": MapLifecycleResponse},
            404: {"model": MapLifecycleResponse},
            500: {"model": MapLifecycleResponse},
        },
    )
    async def activate_map(body: MapNameRequest):
        payload = _body_mapping(body)
        name = payload.get("name", "")
        err = safe_map_name(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        session_mode = str(getattr(gw, "_session_mode", "") or "").strip().lower()
        if session_mode in {"navigating", "exploring"}:
            active_map = str(getattr(gw, "_session_map", "") or "").strip()
            if not active_map or active_map != name:
                return _map_lifecycle_response(
                    False,
                    status_code=409,
                    name=name,
                    active=active_map or None,
                    message=(
                        "Cross-map activation is blocked during an active navigation "
                        "session; use the full runtime switch so localization and "
                        "planning change maps together."
                    ),
                )
            return _map_lifecycle_response(
                True,
                status_code=200,
                name=name,
                active=active_map,
                message="Map is already active for the current navigation session.",
            )
        try:
            previous_resp = _map_service_command(gw, {"action": "get_active"})
        except HTTPException as exc:
            return _map_lifecycle_response(
                False,
                message=str(exc.detail),
                status_code=int(exc.status_code),
            )
        previous_active = str(previous_resp.get("active") or "").strip()
        _clear_gateway_map_cloud(gw, "map_activation_begin")
        try:
            resp = _map_service_command(gw, {"action": "set_active", "name": name})
        except HTTPException as exc:
            return _map_lifecycle_response(
                False,
                message=str(exc.detail),
                status_code=int(exc.status_code),
            )
        ok = resp.get("success") is True
        message = str(resp.get("message") or "")
        if not ok and "not found" in message.lower():
            status_code = 404
        elif not ok:
            status_code = 409
        else:
            status_code = 200
            map_path = str(resp.get("octomap") or resp.get("occupancy") or resp.get("pcd") or "")
            reload_planner_map = command_service.reload_navigation_map
            try:
                resp["planner_reload"] = reload_planner_map(map_path)
            except Exception as exc:
                logger.warning("planner map reload after activation failed: %s", exc)
                resp["planner_reload"] = {
                    "ok": False,
                    "reason": "planner_reload_failed",
                    "message": str(exc),
                    "map_path": map_path,
                }
            planner_reload = resp.get("planner_reload")
            if not isinstance(planner_reload, Mapping) or planner_reload.get("ok") is not True:
                rollback: dict[str, Any] = {
                    "success": previous_active == name,
                    "skipped": previous_active == name,
                    "previous_active": previous_active or None,
                }
                if previous_active and previous_active != name:
                    try:
                        rollback = _map_service_command(
                            gw,
                            {"action": "set_active", "name": previous_active},
                        )
                    except HTTPException as exc:
                        rollback = {
                            "success": False,
                            "message": str(exc.detail),
                            "previous_active": previous_active,
                        }
                map_rollback_ok = rollback.get("success") is True
                if map_rollback_ok and previous_active:
                    rollback_path = str(
                        rollback.get("octomap") or rollback.get("occupancy") or rollback.get("pcd") or ""
                    )
                    if rollback_path:
                        try:
                            planner_rollback = reload_planner_map(rollback_path)
                        except Exception as exc:
                            planner_rollback = {
                                "ok": False,
                                "reason": "planner_rollback_failed",
                                "message": str(exc),
                                "map_path": rollback_path,
                            }
                    else:
                        planner_rollback = {
                            "ok": False,
                            "reason": "planner_rollback_path_missing",
                            "map_path": "",
                        }
                    rollback["planner_reload"] = planner_rollback
                    rollback["success"] = bool(
                        isinstance(planner_rollback, Mapping) and planner_rollback.get("ok") is True
                    )
                resp["rollback"] = rollback
                resp["success"] = False
                resp["message"] = (
                    "planner map reload failed; active map was rolled back"
                    if rollback.get("success") is True
                    else "planner map reload failed and active-map rollback failed"
                )
                status_code = 409
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
        err_old = safe_map_name(old)
        err_new = safe_map_name(new)
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
        response_model=MapLifecycleResponse,
        responses={
            202: {"model": MapLifecycleResponse},
            400: {"model": MapLifecycleResponse},
            409: {"model": MapLifecycleResponse},
            500: {"model": MapLifecycleResponse},
        },
    )
    async def save_map_now(body: MapSaveRequest = MapSaveRequest()):
        payload = _body_mapping(body)
        name = payload.get("name", "")
        if not name:
            name = "map_" + datetime.now().strftime("%Y%m%d_%H%M%S")
        err = safe_map_name(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        slam_profile = "unknown"
        try:
            slam_profile = gw._get_slam_profile()
        except Exception:
            slam_profile = getattr(gw, "_session_slam_profile", "unknown")
        capability = backend_capability_defaults(slam_profile)
        if capability.get("map_save_supported") is not True:
            return _map_lifecycle_response(
                False,
                status_code=409,
                name=name,
                message=f"map save is not supported by SLAM backend: {slam_profile}",
                slam_profile=slam_profile,
                map_save_source=capability.get("map_save_source", "unknown"),
            )

        try:
            resp = _map_service_command(
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
            return _map_lifecycle_response(
                False,
                status_code=int(exc.status_code),
                name=name,
                message=str(exc.detail),
            )
        ok = resp.get("success") is True
        accepted_running = resp.get("accepted") is True and resp.get("status") == "running"
        if accepted_running:
            accepted_payload = dict(resp)
            accepted_payload.pop("success", None)
            accepted_payload.pop("ok", None)
            payload = map_lifecycle_payload(
                True,
                name=name,
                **accepted_payload,
            )
            payload["success"] = False
            return JSONResponse(payload, status_code=202)
        message_lower = str(resp.get("message", "")).lower()
        status_code = 200 if ok else 409 if "unsupported" in message_lower or "not supported" in message_lower else 500
        return _map_service_lifecycle_response(resp, status_code=status_code, name=name)

    @app.get(
        "/api/v1/maps/save-jobs",
        summary="List durable SaveMap jobs",
        response_model=MapLifecycleResponse,
    )
    async def list_save_map_jobs(limit: int = 100):
        resp = _map_service_command(
            gw,
            {"action": "list_save_jobs", "limit": max(1, min(limit, 1000))},
        )
        return _map_service_lifecycle_response(
            resp,
            status_code=200 if resp.get("success") is True else 500,
        )

    @app.get(
        "/api/v1/maps/save-jobs/{job_id}",
        summary="Get durable SaveMap job status",
        response_model=MapLifecycleResponse,
    )
    async def get_save_map_job(job_id: str):
        if SAVE_JOB_ID_PATTERN.fullmatch(job_id) is None:
            return _map_lifecycle_response(
                False,
                status_code=400,
                message="invalid SaveMap job id",
            )
        resp = _map_service_command(
            gw,
            {"action": "save_status", "job_id": job_id},
        )
        return _map_service_lifecycle_response(
            resp,
            status_code=200 if resp.get("success") is True else 404,
        )

    @app.post(
        "/api/v1/maps/save-jobs/{job_id}/cancel",
        summary="Cancel a durable SaveMap job",
        response_model=MapLifecycleResponse,
    )
    async def cancel_save_map_job(job_id: str):
        if SAVE_JOB_ID_PATTERN.fullmatch(job_id) is None:
            return _map_lifecycle_response(
                False,
                status_code=400,
                message="invalid SaveMap job id",
            )
        resp = _map_service_command(
            gw,
            {"action": "cancel_save", "job_id": job_id},
        )
        accepted = resp.get("accepted") is True
        return _map_service_lifecycle_response(
            {**resp, "success": accepted},
            status_code=200 if accepted else 409,
        )

    @app.post(
        "/api/v1/maps/save-jobs/{job_id}/retry",
        summary="Retry a failed durable SaveMap job",
        response_model=MapLifecycleResponse,
    )
    async def retry_save_map_job(job_id: str):
        if SAVE_JOB_ID_PATTERN.fullmatch(job_id) is None:
            return _map_lifecycle_response(
                False,
                status_code=400,
                message="invalid SaveMap job id",
            )
        resp = _map_service_command(
            gw,
            {"action": "retry_save", "job_id": job_id},
        )
        accepted = resp.get("accepted") is True
        return _map_service_lifecycle_response(
            {**resp, "success": accepted},
            status_code=202 if accepted else 409,
        )

    @app.get(
        "/api/v1/maps/{name}/versions",
        summary="List verified immutable map versions",
        response_model=MapLifecycleResponse,
    )
    async def list_map_versions(name: str):
        err = safe_map_name(name)
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
        err = safe_map_name(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        if version <= 0:
            return _map_lifecycle_response(
                False,
                message="version must be a positive integer",
                status_code=400,
            )
        resp = _map_service_command(
            gw,
            {"action": "rollback_map_version", "name": name, "version": version},
        )
        return _map_service_lifecycle_response(
            resp,
            status_code=200 if resp.get("success") is True else 409,
        )
