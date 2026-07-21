"""Maps Module command routing."""

from __future__ import annotations

from typing import Any, Callable

from maps.services.storage import InvalidMapName

Handler = Callable[[Any, dict[str, Any]], dict[str, Any]]


def dispatch_map_command(service: Any, cmd: dict[str, Any]) -> dict[str, Any]:
    action = str(cmd.get("action", ""))
    handler = _ROUTES.get(action)
    if handler is None:
        return {
            "action": action,
            "success": False,
            "reason_code": "unknown_action",
            "message": f"unknown action: {action}",
        }
    try:
        return _with_reason_code(handler(service, cmd))
    except InvalidMapName as exc:
        return {
            "action": action,
            "success": False,
            "reason_code": "invalid_map_name",
            "message": str(exc),
        }


def _name(cmd: dict[str, Any]) -> str:
    return str(cmd.get("name", "") or cmd.get("map_id", ""))


def _map_id(cmd: dict[str, Any]) -> str:
    return str(cmd.get("map_id", "") or cmd.get("map", ""))


def _job_id(cmd: dict[str, Any]) -> str:
    return str(cmd.get("job_id", "") or cmd.get("request_id", ""))


def _save(service: Any, cmd: dict[str, Any]) -> dict[str, Any]:
    kwargs = {
        "slam_profile": cmd.get("slam_profile") or cmd.get("backend") or cmd.get("localization_backend"),
        "map_opt": cmd.get("map_opt") or cmd.get("optimization") or cmd.get("optimizer"),
    }
    request_id = cmd.get("request_id") or cmd.get("idempotency_key")
    if request_id:
        kwargs["request_id"] = request_id
    return service._map_save(str(cmd.get("name", "")), **kwargs)


def _artifact_type(cmd: dict[str, Any]) -> str:
    return str(
        cmd.get("artifact_type", "") or cmd.get("type", "") or cmd.get("capability", "") or cmd.get("map_class", "")
    )


def _float(cmd: dict[str, Any], key: str, default: float = 0.0) -> float:
    try:
        return float(cmd.get(key, default) or default)
    except (TypeError, ValueError):
        return default


def _optional_str(cmd: dict[str, Any], key: str) -> str | None:
    value = cmd.get(key)
    return None if value is None else str(value)


def _validate_artifacts(service: Any, cmd: dict[str, Any]) -> dict[str, Any]:
    return service._validate_map_artifacts(
        _name(cmd),
        require_octomap=bool(cmd.get("require_octomap", False)),
        require_occupancy=bool(cmd.get("require_occupancy", False)),
        expected_data_source=_optional_str(cmd, "expected_data_source"),
        expected_source_profile=_optional_str(cmd, "expected_source_profile"),
        expected_frame_id=_optional_str(cmd, "expected_frame_id"),
    )


def _with_reason_code(resp: dict[str, Any]) -> dict[str, Any]:
    if resp.get("success") is not False or resp.get("reason_code"):
        return resp
    message = str(resp.get("message") or "").lower()
    if "invalid map name" in message:
        reason = "invalid_map_name"
    elif "not found" in message:
        reason = "map_not_found"
    elif "artifact_type" in message:
        reason = "unsupported_artifact_type"
    elif "artifact gate" in message:
        reason = "artifact_gate_failed"
    elif "missing map name" in message:
        reason = "missing_map_name"
    elif "missing capability" in message:
        reason = "missing_capability"
    elif "write in progress" in message:
        reason = "map_write_in_progress"
    else:
        reason = "operation_failed"
    return {**resp, "reason_code": reason}


_ROUTES: dict[str, Handler] = {
    "list": lambda service, _cmd: service._map_list(),
    "get_map_types": lambda service, _cmd: service._get_map_types(),
    "map_types": lambda service, _cmd: service._get_map_types(),
    "describe_map_types": lambda service, _cmd: service._get_map_types(),
    "create": lambda service, cmd: service._map_create(_name(cmd)),
    "create_map": lambda service, cmd: service._map_create(_name(cmd)),
    "save": _save,
    "save_status": lambda service, cmd: service._get_save_map_status(_job_id(cmd)),
    "get_save_status": lambda service, cmd: service._get_save_map_status(_job_id(cmd)),
    "list_save_jobs": lambda service, cmd: service._list_save_map_jobs(
        max(1, min(int(_float(cmd, "limit", 100.0)), 1000))
    ),
    "cancel_save": lambda service, cmd: service._cancel_save_map(_job_id(cmd)),
    "retry_save": lambda service, cmd: service._retry_save_map(_job_id(cmd)),
    "list_versions": lambda service, cmd: service._list_map_versions(_name(cmd)),
    "list_map_versions": lambda service, cmd: service._list_map_versions(_name(cmd)),
    "rollback_version": lambda service, cmd: service._rollback_map_version(
        _name(cmd),
        int(_float(cmd, "version", 0.0)),
    ),
    "rollback_map_version": lambda service, cmd: service._rollback_map_version(
        _name(cmd),
        int(_float(cmd, "version", 0.0)),
    ),
    "delete": lambda service, cmd: service._map_delete(str(cmd.get("name", ""))),
    "retire": lambda service, cmd: service._map_retire(_name(cmd)),
    "retire_map": lambda service, cmd: service._map_retire(_name(cmd)),
    "rename": lambda service, cmd: service._map_rename(
        str(cmd.get("name", "")),
        str(cmd.get("new_name", "")),
    ),
    "restore_source": lambda service, cmd: service._map_restore_source_backup(_name(cmd)),
    "restore_source_backup": lambda service, cmd: service._map_restore_source_backup(_name(cmd)),
    "set_active": lambda service, cmd: service._map_set_active(str(cmd.get("name", ""))),
    "build_occupancy": lambda service, cmd: service._build_occupancy_snapshot(str(cmd.get("name", ""))),
    "build_occupancy_snapshot": lambda service, cmd: service._build_occupancy_snapshot(str(cmd.get("name", ""))),
    "build_octomap": lambda service, cmd: service._build_octomap_artifact(str(cmd.get("name", ""))),
    "build_octomap_artifact": lambda service, cmd: service._build_octomap_artifact(str(cmd.get("name", ""))),
    "build_esdf": lambda service, cmd: service._build_esdf_artifact(_name(cmd)),
    "build_esdf_artifact": lambda service, cmd: service._build_esdf_artifact(_name(cmd)),
    "build_traversability": lambda service, cmd: service._build_traversability_artifact(_name(cmd)),
    "build_traversability_artifact": lambda service, cmd: service._build_traversability_artifact(_name(cmd)),
    "build_artifact": lambda service, cmd: service._build_artifact(
        _name(cmd),
        _artifact_type(cmd),
    ),
    "build_map_artifact": lambda service, cmd: service._build_artifact(
        _name(cmd),
        _artifact_type(cmd),
    ),
    "import_pcd": lambda service, cmd: service._import_pcd(
        _name(cmd),
        str(cmd.get("source_path", "") or cmd.get("path", "")),
        voxel_size=_float(cmd, "voxel_size", 0.0),
        bounds=cmd.get("bounds") if isinstance(cmd.get("bounds"), dict) else None,
    ),
    "import_unity_semantics": lambda service, cmd: service._import_unity_semantic_artifact(
        _name(cmd),
        str(cmd.get("scene_dir", "") or cmd.get("source_path", "")),
        taxonomy_path=_optional_str(cmd, "taxonomy_path"),
        frame_id=str(cmd.get("frame_id", "map") or "map"),
        voxel_size_m=_float(cmd, "voxel_size_m", 0.20),
        occupied_probability=_float(cmd, "occupied_probability", 0.95),
        shell_thickness_voxels=_float(cmd, "shell_thickness_voxels", 0.75),
        generation=max(1, int(_float(cmd, "generation", 1.0))),
        max_objects=max(1, int(_float(cmd, "max_objects", 100_000.0))),
        max_voxels=max(1, int(_float(cmd, "max_voxels", 2_000_000.0))),
        max_voxel_checks=max(1, int(_float(cmd, "max_voxel_checks", 50_000_000.0))),
        include_unknown_geometry=bool(cmd.get("include_unknown_geometry", False)),
        exclude_dynamic_classes=bool(cmd.get("exclude_dynamic_classes", True)),
    ),
    "crop": lambda service, cmd: service._crop_map(
        _name(cmd),
        cmd.get("bounds") if isinstance(cmd.get("bounds"), dict) else {},
        invert=bool(cmd.get("invert", False)),
        voxel_size=_float(cmd, "voxel_size", 0.0),
    ),
    "crop_map": lambda service, cmd: service._crop_map(
        _name(cmd),
        cmd.get("bounds") if isinstance(cmd.get("bounds"), dict) else {},
        invert=bool(cmd.get("invert", False)),
        voxel_size=_float(cmd, "voxel_size", 0.0),
    ),
    "get_record": lambda service, cmd: service._get_record(_name(cmd)),
    "get_metadata": lambda service, cmd: service._get_record(_name(cmd)),
    "get_active": lambda service, _cmd: service._get_active_map(),
    "get_active_map": lambda service, _cmd: service._get_active_map(),
    "get_health": lambda service, cmd: service._get_map_health(_name(cmd)),
    "get_map_health": lambda service, cmd: service._get_map_health(_name(cmd)),
    "get_build_status": lambda service, cmd: service._get_pipeline_status(_name(cmd)),
    "build_status": lambda service, cmd: service._get_pipeline_status(_name(cmd)),
    "get_pipeline_status": lambda service, cmd: service._get_pipeline_status(_name(cmd)),
    "get_artifact": lambda service, cmd: service._get_map_bundle(
        _name(cmd),
        str(cmd.get("capability", "")),
    ),
    "get_map_bundle": lambda service, cmd: service._get_map_bundle(
        _name(cmd),
        str(cmd.get("capability", "")),
    ),
    "get_map_points": lambda service, cmd: service._get_map_points(
        _name(cmd),
        max_points=int(_float(cmd, "max_points", 0.0)),
    ),
    "get_saved_map_points": lambda service, cmd: service._get_map_points(
        _name(cmd),
        max_points=int(_float(cmd, "max_points", 0.0)),
    ),
    "validate_artifacts": _validate_artifacts,
    "validate_map_artifacts": _validate_artifacts,
    "get_voxel_edits": lambda service, cmd: service._get_voxel_edits(_name(cmd)),
    "edit_voxel": lambda service, cmd: service._map_edit_voxels(_name(cmd), cmd),
    "edit_voxels": lambda service, cmd: service._map_edit_voxels(_name(cmd), cmd),
    "voxel_edit": lambda service, cmd: service._map_edit_voxels(_name(cmd), cmd),
    "poi_set": lambda service, cmd: service._poi_set(cmd),
    "poi_delete": lambda service, cmd: service._poi_delete(
        str(cmd.get("name", "")),
        map_id=_map_id(cmd),
    ),
    "poi_list": lambda service, cmd: service._poi_list(map_id=_map_id(cmd)),
    "map_graph": lambda service, _cmd: service._map_graph(),
    "get_map_graph": lambda service, _cmd: service._map_graph(),
    "set_map_edge": lambda service, cmd: service._map_edge_set(cmd),
    "map_edge_set": lambda service, cmd: service._map_edge_set(cmd),
    "delete_map_edge": lambda service, cmd: service._map_edge_delete(cmd),
    "map_edge_delete": lambda service, cmd: service._map_edge_delete(cmd),
    "shortest_route": lambda service, cmd: service._shortest_route(cmd),
    "map_shortest_route": lambda service, cmd: service._shortest_route(cmd),
    "rollback_active": lambda service, _cmd: service._rollback_active_map(),
    "rollback_active_map": lambda service, _cmd: service._rollback_active_map(),
    "active_slots": lambda service, _cmd: service._active_slots(),
    "list_active_slots": lambda service, _cmd: service._active_slots(),
    "get_active_slot": lambda service, cmd: service._get_active_slot(str(cmd.get("slot", "") or cmd.get("name", ""))),
    "set_active_slot": lambda service, cmd: service._set_active_slot(cmd),
    "clear_active_slot": lambda service, cmd: service._clear_active_slot(
        str(cmd.get("slot", "") or cmd.get("name", ""))
    ),
    "build_queue": lambda service, _cmd: service._build_queue(),
    "get_build_queue": lambda service, _cmd: service._build_queue(),
    "enqueue_build": lambda service, cmd: service._enqueue_build(cmd),
    "pop_build_queue": lambda service, _cmd: service._pop_build_queue(),
    "get_artifact_job": lambda service, cmd: service._get_artifact_job(_job_id(cmd)),
    "cancel_artifact_job": lambda service, cmd: service._cancel_artifact_job(_job_id(cmd)),
    "retry_artifact_job": lambda service, cmd: service._retry_artifact_job(_job_id(cmd)),
    "audit_versions": lambda service, cmd: service._audit_versions(dry_run=bool(cmd.get("dry_run", True))),
    "quarantine_versions": lambda service, cmd: service._quarantine_versions(dry_run=bool(cmd.get("dry_run", True))),
    "gc_versions": lambda service, cmd: service._gc_versions(dry_run=bool(cmd.get("dry_run", True))),
    "migrate_versions": lambda service, cmd: service._migrate_versions(dry_run=bool(cmd.get("dry_run", True))),
    "export_version": lambda service, cmd: service._export_version(
        _name(cmd),
        int(_float(cmd, "version", 0.0)),
        str(cmd.get("package_dir", "") or cmd.get("path", "")),
        dry_run=bool(cmd.get("dry_run", False)),
    ),
    "import_package": lambda service, cmd: service._import_package(
        str(cmd.get("package_dir", "") or cmd.get("path", "")),
        requested_map_id=_name(cmd),
        dry_run=bool(cmd.get("dry_run", False)),
    ),
}
