"""MapService command routing."""

from __future__ import annotations

from typing import Any, Callable

from nav.services.map.storage import InvalidMapName

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


def _save(service: Any, cmd: dict[str, Any]) -> dict[str, Any]:
    return service._map_save(
        str(cmd.get("name", "")),
        slam_profile=cmd.get("slam_profile")
        or cmd.get("backend")
        or cmd.get("localization_backend"),
    )


def _artifact_type(cmd: dict[str, Any]) -> str:
    return str(
        cmd.get("artifact_type", "")
        or cmd.get("type", "")
        or cmd.get("capability", "")
        or cmd.get("map_class", "")
    )


def _float(cmd: dict[str, Any], key: str, default: float = 0.0) -> float:
    try:
        return float(cmd.get(key, default) or default)
    except (TypeError, ValueError):
        return default


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
    "delete": lambda service, cmd: service._map_delete(str(cmd.get("name", ""))),
    "retire": lambda service, cmd: service._map_retire(_name(cmd)),
    "retire_map": lambda service, cmd: service._map_retire(_name(cmd)),
    "rename": lambda service, cmd: service._map_rename(
        str(cmd.get("name", "")),
        str(cmd.get("new_name", "")),
    ),
    "set_active": lambda service, cmd: service._map_set_active(str(cmd.get("name", ""))),
    "build_tomogram": lambda service, cmd: service._build_tomogram(str(cmd.get("name", ""))),
    "build_occupancy": lambda service, cmd: service._build_occupancy_snapshot(str(cmd.get("name", ""))),
    "build_occupancy_snapshot": lambda service, cmd: service._build_occupancy_snapshot(str(cmd.get("name", ""))),
    "build_octomap": lambda service, cmd: service._build_octomap_artifact(str(cmd.get("name", ""))),
    "build_octomap_artifact": lambda service, cmd: service._build_octomap_artifact(str(cmd.get("name", ""))),
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
    "get_artifact": lambda service, cmd: service._get_map_bundle(
        _name(cmd),
        str(cmd.get("capability", "")),
    ),
    "get_map_bundle": lambda service, cmd: service._get_map_bundle(
        _name(cmd),
        str(cmd.get("capability", "")),
    ),
    "edit_voxel": lambda service, cmd: service._map_edit_voxels(_name(cmd), cmd),
    "edit_voxels": lambda service, cmd: service._map_edit_voxels(_name(cmd), cmd),
    "voxel_edit": lambda service, cmd: service._map_edit_voxels(_name(cmd), cmd),
    "poi_set": lambda service, cmd: service._poi_set(cmd),
    "poi_delete": lambda service, cmd: service._poi_delete(str(cmd.get("name", ""))),
    "poi_list": lambda service, _cmd: service._poi_list(),
}
