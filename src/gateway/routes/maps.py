"""Map lifecycle and viewer routes for GatewayModule."""

from __future__ import annotations

import json
import logging
import math
import os
import pathlib
import shutil
import struct
import time
from datetime import datetime
from typing import Any

from fastapi import HTTPException
from fastapi.responses import FileResponse, JSONResponse
from starlette.responses import HTMLResponse

from gateway.schemas import (
    MapLifecycleResponse,
    MapListResponse,
    MapNameRequest,
    MapPointsResponse,
    MapRenameRequest,
    MapSaveRequest,
    PlanPreviewRequest,
)
from gateway.services.map_paths import active_map_name, nav_map_root_str
from gateway.services.map_safety import (
    safe_map_name,
)
from gateway.services.control_commands import ControlCommandService
from runtime.msgs.numpy_compat import is_numpy_array, np
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.same_source_map_artifacts import (
    validate_saved_map_artifact_dir,
)
from runtime.utils.sanitize import sanitize_dict

logger = logging.getLogger(__name__)
MAX_EXECUTABLE_START_SNAP_M = 0.5


def _apply_dynamic_filter_step1half(*args: Any, **kwargs: Any) -> Any:
    from runtime.dynamic_filter import apply_dynamic_filter_step1half

    return apply_dynamic_filter_step1half(*args, **kwargs)


apply_dynamic_filter_step1half = _apply_dynamic_filter_step1half


def _map_dir() -> str:
    return nav_map_root_str()


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
    assert isinstance(body, dict), f"expected dict or Pydantic model, got {type(body).__name__}"
    return body


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
    gw: Any,
    preview: dict[str, Any],
) -> dict[str, Any] | None:
    path = _preview_path_xyz(preview)
    if not path:
        return None
    nav = (getattr(gw, "_all_modules", {}) or {}).get("nav.mission")
    planner = getattr(nav, "_planner_svc", None)
    evaluator = getattr(planner, "evaluate_current_path_safety", None)
    if not callable(evaluator):
        return None
    try:
        return evaluator(path)
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
    preview["fallback_reason"] = (
        preview.get("fallback_reason")
        or "planner did not reach the requested target"
    )


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
        or (
            isinstance(path_safety, dict)
            and path_safety.get("ok") is False
        )
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
                "start and goal were accepted by OctoPlanner3D, but no "
                "traversable occupancy path connected them"
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
        "runtime_map_path": diagnostics.get("runtime_map_path")
        or diagnostics.get("map_path"),
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
    mgr = getattr(gw, "_map_mgr", None)
    if mgr is None:
        raise HTTPException(status_code=503, detail="MapService not running")
    result: list[dict[str, Any]] = []

    def _capture(resp: dict[str, Any]) -> None:
        result.append(resp)

    mgr.map_response._add_callback(_capture)
    try:
        mgr.map_command._deliver(json.dumps(cmd))
    finally:
        try:
            mgr.map_response._callbacks.remove(_capture)
        except (ValueError, AttributeError):
            pass
    return result[0] if result else {"success": False, "message": "no response"}


def _safe_map_file(name: str, filename: str) -> pathlib.Path:
    base = pathlib.Path(_map_dir()).resolve()
    path = (base / name / filename).resolve()
    try:
        path.relative_to(base)
    except ValueError as exc:
        raise HTTPException(status_code=403) from exc
    return path


def _write_binary_xyz_pcd(path: pathlib.Path, points: Any) -> int:
    if is_numpy_array(points):
        return _write_binary_xyz_pcd_array(path, points)

    rows = points.tolist() if hasattr(points, "tolist") else points
    pts: list[tuple[float, float, float]] = []
    for row in rows or []:
        try:
            x, y, z = float(row[0]), float(row[1]), float(row[2])
        except (TypeError, ValueError, IndexError):
            continue
        if (
            math.isfinite(x)
            and math.isfinite(y)
            and math.isfinite(z)
            and max(abs(x), abs(y), abs(z)) < 500.0
        ):
            pts.append((x, y, z))

    if not pts:
        raise ValueError("no valid points to save")
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(pts)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(pts)}\n"
        "DATA binary\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        f.write(header.encode("ascii"))
        for point in pts:
            f.write(struct.pack("<fff", *point))
    return len(pts)


def _write_binary_xyz_pcd_array(path: pathlib.Path, points: Any) -> int:
    arr = np.asarray(points, dtype=np.float32)
    if arr.ndim != 2 or arr.shape[1] < 3:
        raise ValueError("point cloud must have shape Nx3")
    xyz = arr[:, :3]
    finite = np.isfinite(xyz).all(axis=1)
    bounded = np.max(np.abs(xyz), axis=1) < 500.0
    valid = np.ascontiguousarray(xyz[finite & bounded], dtype=np.float32)
    if valid.size == 0:
        raise ValueError("no valid points to save")
    count = int(valid.shape[0])
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        "DATA binary\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        f.write(header.encode("ascii"))
        f.write(valid.astype("<f4", copy=False).tobytes())
    return count


def _median(values: list[float]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) * 0.5


def _sample_xyz_points(
    points: list[tuple[float, float, float]],
    max_points: int,
) -> list[tuple[float, float, float]]:
    if max_points <= 0:
        return []
    if len(points) <= max_points:
        return points
    step = len(points) / float(max_points)
    return [points[min(int(index * step), len(points) - 1)] for index in range(max_points)]


def _binary_pcd_xyz_payload(
    data: bytes,
    *,
    n_points: int,
    point_step: int,
    max_points: int,
) -> tuple[int, list[float]]:
    if n_points <= 0 or point_step < 12:
        return 0, []

    points: list[tuple[float, float, float]] = []
    available = min(n_points, len(data) // point_step)
    for index in range(available):
        x, y, z = struct.unpack_from("<fff", data, index * point_step)
        if all(math.isfinite(value) for value in (x, y, z)):
            points.append((float(x), float(y), float(z)))

    if points:
        med = (
            _median([point[0] for point in points]),
            _median([point[1] for point in points]),
            _median([point[2] for point in points]),
        )
        points = [
            point
            for point in points
            if max(abs(point[axis] - med[axis]) for axis in range(3)) < 100.0
        ]

    points = _sample_xyz_points(points, int(max_points))
    flat = [coord for point in points for coord in point]
    return len(points), flat


def register_map_routes(app, gw) -> None:
    command_service = ControlCommandService(gw)

    @app.get(
        "/api/v1/slam/maps",
        summary="List maps from filesystem",
        response_model=MapListResponse,
    )
    async def slam_maps():
        map_dir = _map_dir()
        maps = []
        active_target = active_map_name() or ""

        if os.path.isdir(map_dir):
            for d in sorted(os.listdir(map_dir)):
                full = os.path.join(map_dir, d)
                if not os.path.isdir(full) or d.startswith("_") or d == "active":
                    continue
                pcd = os.path.join(full, "map.pcd")
                has_pcd = os.path.isfile(pcd)
                patches_dir = os.path.join(full, "patches")
                patch_count = (
                    len(os.listdir(patches_dir)) if os.path.isdir(patches_dir) else 0
                )
                has_tomogram = os.path.isfile(os.path.join(full, "tomogram.pickle"))
                has_occupancy = os.path.isfile(os.path.join(full, "occupancy.npz"))
                has_octomap = os.path.isfile(os.path.join(full, "octomap.ot")) or os.path.isfile(
                    os.path.join(full, "octomap.bt")
                )
                has_metadata = os.path.isfile(os.path.join(full, "metadata.json"))
                metadata_path = os.path.join(full, "metadata.json")
                state: str | None = None
                for state_path in (os.path.join(full, "map_record.json"), metadata_path):
                    if not os.path.isfile(state_path):
                        continue
                    try:
                        with open(state_path, encoding="utf-8") as f:
                            state_data = json.load(f)
                        state_value = state_data.get("state")
                        if state_value is not None:
                            state = str(state_value)
                            break
                    except (OSError, json.JSONDecodeError, TypeError, ValueError):
                        continue
                size_mb: float | None = None
                if has_pcd:
                    sz = os.path.getsize(pcd)
                    size_mb = round(sz / 1024 / 1024, 1)
                maps.append(
                    {
                        "name": d,
                        "has_pcd": has_pcd,
                        "has_tomogram": has_tomogram,
                        "has_occupancy": has_occupancy,
                        "has_octomap": has_octomap,
                        "navigation_ready": bool(has_pcd and has_octomap and has_metadata),
                        "state": state,
                        "is_active": d == active_target,
                        "size_mb": size_mb,
                        "patch_count": patch_count,
                    }
                )
        return {
            "schema_version": 1,
            "maps": maps,
            "count": len(maps),
            "active": active_target,
            "map_dir": map_dir,
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
        resp = _map_service_command(
            gw,
            {
                "action": "import_pcd",
                "name": name,
                "source_path": payload.get("source_path") or payload.get("path"),
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
        map_dir = pathlib.Path(_map_dir()) / name
        gate = validate_saved_map_artifact_dir(
            map_dir,
            require_octomap=True,
            expected_frame_id=topic_default_frame_id(TOPICS.saved_map_cloud),
        )
        gate["required"] = True
        active = active_map_name() or ""
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
        preview = command_service.preview_navigation_plan(
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
                    executable_preview.get("fallback_reason")
                    or "planner start snapped too far from live pose"
                )
            executable_path_safety = _evaluate_preview_live_safety(gw, preview)
            executable_preview["path_safety"] = executable_path_safety
            if (
                isinstance(executable_path_safety, dict)
                and executable_path_safety.get("ok") is False
            ):
                executable_preview["feasible"] = False
                _append_preview_reason(executable_preview, "path_safety_failed")
                executable_preview["fallback_reason"] = (
                    executable_preview.get("fallback_reason")
                    or "live path_safety failed"
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
        responses={
            200: {
                "content": {
                    "application/octet-stream": {
                        "schema": {"type": "string", "format": "binary"}
                    }
                }
            }
        },
    )
    async def get_map_pcd(name: str):
        pcd_path = _safe_map_file(name, "map.pcd")
        if not pcd_path.is_file():
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
        pcd_path = _safe_map_file(name, "map.pcd")
        if not pcd_path.is_file():
            raise HTTPException(status_code=404, detail=f"Map not found: {name}")

        with open(pcd_path, "rb") as f:
            n_points, point_step = 0, 16
            while True:
                line = f.readline().decode("ascii", errors="ignore").strip()
                if "POINTS" in line:
                    n_points = int(line.split()[-1])
                if "SIZE" in line:
                    point_step = sum(int(s) for s in line.split()[1:])
                if line.startswith("DATA"):
                    break
            data = f.read(n_points * point_step)
        count, flat = _binary_pcd_xyz_payload(
            data[: n_points * point_step],
            n_points=n_points,
            point_step=point_step,
            max_points=max_points,
        )
        return {
            "schema_version": 1,
            "count": count,
            "layout": "flat_xyz",
            "frame_id": "map",
            "source": "saved_map_pcd",
            "name": name,
            "points": flat,
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
        path = _safe_map_file(name, "voxel_edits.json")
        if not path.parent.is_dir():
            return _map_lifecycle_response(
                False,
                message=f"map not found: {name}",
                status_code=404,
            )
        if not path.is_file():
            return {
                "schema_version": 1,
                "ok": True,
                "success": True,
                "map_id": name,
                "edits": [],
            }
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            return _map_lifecycle_response(
                False,
                message=f"invalid voxel edit overlay: {exc}",
                status_code=400,
            )
        edits = payload.get("edits") if isinstance(payload, dict) else None
        return {
            "schema_version": 1,
            "ok": True,
            "success": True,
            "map_id": name,
            "edits": edits if isinstance(edits, list) else [],
        }

    @app.get(
        "/api/v1/map/points",
        summary="Map point cloud as JSON (from ikd-tree snapshot)",
        response_model=MapPointsResponse,
    )
    async def get_map_points(max_points: int = 80000):
        with gw._map_cloud_lock:
            pts = gw._map_points
        if pts is None or len(pts) == 0:
            return {
                "schema_version": 1,
                "count": 0,
                "layout": "xyz_rows",
                "frame_id": "map",
                "source": "live_map_cloud",
                "points": [],
                "ts": time.time(),
            }
        if len(pts) > max_points:
            idx = np.random.choice(len(pts), max_points, replace=False)
            pts = pts[idx]
        return {
            "count": len(pts),
            "layout": "xyz_rows",
            "frame_id": "map",
            "source": "live_map_cloud",
            "bounds": {
                "x": [float(pts[:, 0].min()), float(pts[:, 0].max())],
                "y": [float(pts[:, 1].min()), float(pts[:, 1].max())],
                "z": [float(pts[:, 2].min()), float(pts[:, 2].max())],
            },
            "points": pts[:, :3].tolist(),
            "ts": time.time(),
        }

    @app.post(
        "/api/v1/map_cloud/reset",
        summary="Clear accumulated map cloud (viz only, SLAM ikd-tree untouched)",
        response_model=MapLifecycleResponse,
    )
    async def reset_map_cloud():
        with gw._map_cloud_lock:
            gw._map_points = None
            gw._map_cloud_count = 0
            gw._voxel_hits.clear()
        gw.push_event({"type": "map_cloud", "points": [], "count": 0})
        return map_lifecycle_payload(
            True,
            message="Accumulated map cloud cleared",
        )

    @app.get("/map/viewer", summary="Interactive 3D map viewer")
    async def map_viewer(map: str = ""):
        if map:
            html = gw._generate_viewer_from_pcd(map)
        else:
            html = gw._generate_viewer_live()
        return HTMLResponse(html)

    @app.get("/robot/meshes/{filename}", summary="Serve robot STL mesh files")
    async def serve_robot_mesh(filename: str):
        mesh_dir = os.environ.get(
            "DOG_MESH_DIR",
            os.path.join(
                os.path.dirname(__file__),
                "../../../../products/quadruped_ws/dog_arm/meshes",
            ),
        )
        safe_name = os.path.basename(filename)
        path = os.path.join(mesh_dir, safe_name)
        if not os.path.isfile(path):
            return JSONResponse(
                status_code=404,
                content={"error": "mesh not found", "name": safe_name},
            )
        return FileResponse(
            path,
            media_type="application/octet-stream",
            headers={
                "Access-Control-Allow-Origin": "*",
                "Cache-Control": "public, max-age=3600",
            },
        )

    @app.post(
        "/api/v1/map/restore_predufo",
        summary="Restore map.pcd from DUFOMap pre-filter backup",
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
        map_dir = _map_dir()
        target = pathlib.Path(map_dir) / name
        pcd = target / "map.pcd"
        backup = target / "map.pcd.predufo"
        if not backup.is_file():
            return _map_lifecycle_response(
                False,
                message=(
                    f"No predufo backup for {name}. "
                    "DUFOMap may not have run on this map."
                ),
                status_code=404,
            )
        try:
            import time as _t

            if pcd.is_file():
                shutil.copy(pcd, target / f"map.pcd.replaced-{_t.time_ns()}")
            tmp_new = pcd.with_suffix(".pcd.tmp")
            shutil.copy(backup, tmp_new)
            os.replace(tmp_new, pcd)

            replaced = sorted(
                target.glob("map.pcd.replaced-*"),
                key=lambda p: p.stat().st_mtime,
                reverse=True,
            )
            pruned = 0
            for old in replaced[3:]:
                try:
                    old.unlink()
                    pruned += 1
                except Exception as e:
                    logger.warning("cleanup old backup failed: %s", e)

            return map_lifecycle_payload(
                True,
                name=name,
                restored_size=pcd.stat().st_size,
                replaced_backups_kept=min(len(replaced), 3),
                replaced_backups_pruned=pruned,
                note="octomap/occupancy must be rebuilt before planner use",
            )
        except Exception as e:
            logger.exception("restore_predufo failed")
            return _map_lifecycle_response(False, message=str(e), status_code=500)

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
            map_path = str(
                resp.get("octomap")
                or resp.get("tomogram")
                or resp.get("occupancy")
                or ""
            )
            nav = (getattr(gw, "_all_modules", {}) or {}).get("nav.mission")
            reload_planner_map = getattr(nav, "reload_planner_map", None)
            if callable(reload_planner_map):
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
            else:
                resp["planner_reload"] = {
                    "ok": False,
                    "reason": "nav_mission_unavailable",
                    "map_path": map_path,
                }
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
        map_dir = _map_dir()
        old_path = os.path.join(map_dir, old)
        new_path = os.path.join(map_dir, new)
        if not os.path.isdir(old_path):
            return _map_lifecycle_response(
                False,
                message=f"Map does not exist: {old}",
                status_code=404,
            )
        if os.path.exists(new_path):
            return _map_lifecycle_response(
                False,
                message=f"Name already exists: {new}",
                status_code=409,
            )
        try:
            os.rename(old_path, new_path)
            active_link = pathlib.Path(map_dir) / "active"
            if active_link.is_symlink() and active_link.resolve().name == old:
                active_link.unlink()
                active_link.symlink_to(new)
            return map_lifecycle_payload(True, old_name=old, new_name=new)
        except Exception as e:
            return _map_lifecycle_response(False, message=str(e), status_code=500)

    @app.post(
        "/api/v1/map/save",
        summary="Save current SLAM map",
        response_model=MapLifecycleResponse,
        responses={
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

        try:
            resp = _map_service_command(
                gw,
                {
                    "action": "save",
                    "name": name,
                    "slam_profile": slam_profile,
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
        status_code = (
            200
            if ok
            else 409
            if "unsupported" in str(resp.get("message", ""))
            else 500
        )
        return _map_service_lifecycle_response(resp, status_code=status_code, name=name)
