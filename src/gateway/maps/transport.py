"""Stateless Gateway transport helpers for native ``mapd``."""

from __future__ import annotations

import logging
import re
import time
import uuid
from typing import Any

from runtime.endpoints.mapd import ArtifactHandle, MapClient

logger = logging.getLogger(__name__)
_MANAGEMENT_TIMEOUT_S = 360.0
_QUERY_TIMEOUT_S = 0.2
_ACTIVE_MAP_HOLD_S = 5.0
_MAP_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")

_ACTIVE_MUTATIONS = frozenset({"set_active_map", "clear_active_map"})


def map_management_available(gw: Any) -> bool:
    """Return whether this RunPlan starts mapd's local management endpoint."""
    plan = getattr(gw, "_compiled_run_plan", None)
    if plan is None:
        return False
    try:
        command = plan.process("maps").command
    except (AttributeError, KeyError):
        return False
    return command is None or "--disable-query" not in command.argv


def safe_map_name(name: str) -> str | None:
    """Validate one public map id at the HTTP seam."""
    if not isinstance(name, str) or not name:
        return "empty name"
    if len(name) > 100:
        return "name too long (max 100)"
    if "/" in name or "\\" in name or ".." in name:
        return f"unsafe characters in name: {name!r}"
    if name[0] in ".-":
        return f"name cannot start with . or -: {name!r}"
    if not _MAP_ID_PATTERN.fullmatch(name):
        return f"only [A-Za-z0-9_.-] allowed: {name!r}"
    return None


def map_client(gw: Any, *, timeout_s: float | None = None) -> MapClient:
    """Return the same-host UDS transport; it owns no map state.

    Long map-management operations (notably save) use the management client.
    Read-only status queries use a separate short-deadline client so a stalled
    mapd query cannot block the Gateway state snapshot and its control UI.
    """
    # Tests and embedders may provide a lightweight injected client. Preserve
    # that seam for bounded queries instead of silently bypassing it.
    injected = getattr(gw, "_map_client", None)
    if timeout_s is not None and injected is not None and not isinstance(injected, MapClient):
        return injected
    attribute = "_map_client" if timeout_s is None else "_map_query_client"
    client = getattr(gw, attribute, None)
    if client is None:
        client = MapClient(timeout_s=_MANAGEMENT_TIMEOUT_S if timeout_s is None else timeout_s)
        try:
            setattr(gw, attribute, client)
        except (AttributeError, TypeError):
            pass
    return client


def open_artifact(gw: Any, map_name: str, capability: str) -> ArtifactHandle:
    """Open an artifact through ``mapd``'s validated descriptor endpoint."""
    open_native = getattr(map_client(gw), "open_artifact", None)
    if not callable(open_native):
        raise RuntimeError("mapd client has no artifact endpoint")
    return open_native(map_name, capability)


def _bounds_arguments(value: Any) -> dict[str, Any]:
    if not isinstance(value, dict) or not value:
        return {"has_bounds": False}

    def component(group: Any, axis: str, index: int, flat_key: str) -> float:
        if isinstance(group, dict):
            return float(group.get(axis, value.get(flat_key, 0.0)))
        if isinstance(group, (list, tuple)) and len(group) > index:
            return float(group[index])
        return float(value.get(flat_key, 0.0))

    minimum = value.get("min")
    maximum = value.get("max")
    return {
        "has_bounds": True,
        "min_x": component(minimum, "x", 0, "min_x"),
        "min_y": component(minimum, "y", 1, "min_y"),
        "min_z": component(minimum, "z", 2, "min_z"),
        "max_x": component(maximum, "x", 0, "max_x"),
        "max_y": component(maximum, "y", 1, "max_y"),
        "max_z": component(maximum, "z", 2, "max_z"),
    }


def voxel_edit_center(command: dict[str, Any]) -> tuple[float, float, float] | None:
    """Return the edit center from ``center`` ({x,y,z} or [x,y,z]) or flat x/y/z keys."""
    center = command.get("center")
    if isinstance(center, dict) and "x" in center and "y" in center:
        return float(center["x"]), float(center["y"]), float(center.get("z") or 0.0)
    if isinstance(center, (list, tuple)) and len(center) >= 2:
        z = center[2] if len(center) > 2 else 0.0
        return float(center[0]), float(center[1]), float(z or 0.0)
    x = command.get("x_m", command.get("x"))
    y = command.get("y_m", command.get("y"))
    if x is None or y is None:
        return None
    return float(x), float(y), float(command.get("z_m", command.get("z")) or 0.0)


def _canonical_request(command: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    requested = str(command.get("action") or "").strip()
    if requested in _ACTIVE_MUTATIONS:
        raise RuntimeError("active map mutation is owned by ProductControl")
    if not requested:
        raise RuntimeError("mapd action is required")
    action = requested

    arguments: dict[str, Any] = {}
    map_id = str(command.get("map_id") or "")
    map_actions = {
        "delete_map", "rename_map", "import_pcd", "crop_pcd", "get_record",
        "get_bundle", "get_map_points", "validate_artifacts",
        "get_voxel_edits", "edit_octomap_voxels", "build_occupancy_snapshot",
        "build_octomap_artifact",
    }
    if action in map_actions:
        arguments["map_id"] = map_id
    if action == "save_map":
        arguments.update(
            map_id=map_id,
            request_id=str(command.get("request_id") or uuid.uuid4()),
        )
    elif action == "rename_map":
        arguments["new_map_id"] = str(command.get("new_map_id") or "")
    elif action in {"get_save_map_status", "cancel_save_map", "retry_save_map"}:
        arguments["job_id"] = str(command.get("job_id") or "")
    elif action == "list_save_map_jobs":
        arguments["limit"] = max(1, min(int(command.get("limit") or 100), 1000))
    elif action in {"import_pcd", "crop_pcd"}:
        arguments.update(_bounds_arguments(command.get("bounds")))
        arguments["voxel_size"] = float(command.get("voxel_size") or 0.0)
        if action == "import_pcd":
            arguments["source_path"] = str(command.get("source_path") or command.get("path") or "")
        else:
            arguments["invert"] = bool(command.get("invert", False))
    elif action == "get_bundle":
        arguments["capability"] = str(command.get("capability") or "")
    elif action == "get_map_points":
        arguments["max_points"] = max(0, int(command.get("max_points") or 0))
    elif action == "validate_artifacts":
        for key in ("require_octomap", "require_occupancy", "expected_frame_id", "expected_data_source", "expected_source_profile"):
            arguments[key] = command.get(key, False if key.startswith("require_") else "")
    elif action == "edit_octomap_voxels":
        center = voxel_edit_center(command)
        if center is None:
            raise ValueError("voxel edit center is required")
        arguments.update(
            state=str(command.get("state") or ""),
            shape=str(command.get("shape") or "sphere"),
            x_m=center[0],
            y_m=center[1],
            z_m=center[2],
            radius_m=float(command.get("radius_m", command.get("radius", 0.2)) or 0.2),
            timeout_sec=float(command.get("timeout_sec") or 15.0),
        )
    return action, {key: value for key, value in arguments.items() if value is not None}


def mapd_request(gw: Any, cmd: dict[str, Any], *, timeout_s: float | None = None) -> dict[str, Any]:
    """Adapt one Gateway request to the canonical mapd action."""
    action, arguments = _canonical_request(dict(cmd))
    return map_client(gw, timeout_s=timeout_s).service(action, **arguments)


def map_query_request(gw: Any, cmd: dict[str, Any]) -> dict[str, Any]:
    """Run one bounded read-only mapd query for status/readiness paths."""
    return mapd_request(gw, cmd, timeout_s=_QUERY_TIMEOUT_S)


def active_map(gw: Any) -> str | None:
    """Return mapd's active map identity, or ``None`` when no map is active.

    The query has a short deadline. A failed or late answer says nothing about
    the active map, so it returns the identity mapd last reported within
    ``_ACTIVE_MAP_HOLD_S`` instead of ``None``: callers such as the scene cache
    and the goal gate treat ``None`` as "no map" and would otherwise flap.
    """
    try:
        resp = map_query_request(gw, {"action": "get_active_map"})
    except Exception as exc:
        logger.debug("mapd active-map query failed: %s", exc)
        resp = None
    now = time.monotonic()
    if isinstance(resp, dict) and resp.get("success") is not False:
        active = str(resp.get("active") or "").strip() or None
        try:
            gw._active_map_observation = (active, now)
        except (AttributeError, TypeError):
            pass
        return active
    observed = getattr(gw, "_active_map_observation", None)
    if isinstance(observed, tuple) and now - observed[1] <= _ACTIVE_MAP_HOLD_S:
        return observed[0]
    return None


def map_bundle(gw: Any, map_name: str, capability: str) -> dict[str, Any] | None:
    """Return mapd artifact metadata without resolving a local file path."""
    try:
        resp = map_query_request(gw, {"action": "get_bundle", "map_id": map_name, "capability": capability})
    except Exception as exc:
        logger.debug("mapd bundle query failed: %s", exc)
        return None
    return resp if isinstance(resp, dict) and resp.get("success") is True else None


def validate_map_artifacts(
    gw: Any,
    map_name: str,
    *,
    require_octomap: bool = False,
    require_occupancy: bool = False,
    expected_data_source: str | None = None,
    expected_source_profile: str | None = None,
    expected_frame_id: str | None = None,
) -> dict[str, Any] | None:
    """Ask mapd to validate one saved-map artifact set."""
    try:
        resp = map_query_request(gw, {
            "action": "validate_artifacts", "map_id": map_name,
            "require_octomap": require_octomap, "require_occupancy": require_occupancy,
            "expected_data_source": expected_data_source, "expected_source_profile": expected_source_profile,
            "expected_frame_id": expected_frame_id,
        })
    except Exception as exc:
        logger.debug("mapd artifact validation query failed: %s", exc)
        return None
    return resp if isinstance(resp, dict) and resp.get("success") is True else None
