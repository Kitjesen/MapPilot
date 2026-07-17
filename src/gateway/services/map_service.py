"""Thin client helpers for the maps service contract."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

from runtime.msgs.map import MapControlRequest
from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)


def maps_service(gw: Any) -> Any | None:
    manager = getattr(gw, "_map_mgr", None)
    if manager is not None:
        return manager
    modules = getattr(gw, "_all_modules", {}) or {}
    for name in ("maps.service", "MapsModule", "MapManagerModule"):
        manager = modules.get(name)
        if manager is not None:
            return manager
    return None


def ensure_maps_service(gw: Any) -> Any:
    """Return the injected maps service.

    Gateway does not own the MapsModule lifecycle. ProductGraph/Blueprint must
    inject ``maps.service`` so persistent map ownership stays in the maps domain.
    """
    manager = maps_service(gw)
    if manager is not None:
        return manager
    raise RuntimeError("maps.service is unavailable; ProductGraph/Blueprint must inject maps.service")


def map_service_command(gw: Any, cmd: dict[str, Any]) -> dict[str, Any]:
    direct_execute = getattr(gw, "execute", None)
    manager = gw if callable(direct_execute) else ensure_maps_service(gw)
    execute = getattr(manager, "execute", None)
    if not callable(execute):
        raise RuntimeError("maps.service has no typed control endpoint")
    return execute(MapControlRequest.from_mapping(cmd))


def map_service_query(gw: Any, query: dict[str, Any]) -> dict[str, Any]:
    direct_query = getattr(gw, "query", None)
    direct_execute = getattr(gw, "execute", None)
    manager = gw if callable(direct_query) or callable(direct_execute) else ensure_maps_service(gw)
    request = MapControlRequest.from_mapping(query)
    query_method = getattr(manager, "query", None)
    if callable(query_method):
        return query_method(request)
    execute = getattr(manager, "execute", None)
    if callable(execute):
        return execute(request)
    raise RuntimeError("maps.service has no typed query endpoint")


def active_map(gw: Any) -> str | None:
    try:
        resp = map_service_query(gw, {"action": "get_active"})
    except Exception as exc:
        logger.debug("maps active-map query failed: %s", exc)
        return None
    active = str(resp.get("active") or "").strip() if isinstance(resp, dict) else ""
    return active or None


def saved_map_points(
    gw: Any,
    map_name: str,
    *,
    max_points: int,
) -> np.ndarray | None:
    try:
        resp = map_service_query(
            gw,
            {
                "action": "get_map_points",
                "name": map_name,
                "max_points": max_points,
            },
        )
    except Exception as exc:
        logger.debug("maps saved-map points query failed: %s", exc)
        return None
    if not isinstance(resp, dict) or resp.get("success") is not True:
        return None
    rows = resp.get("points") if isinstance(resp.get("points"), list) else []
    if not rows:
        return None
    try:
        arr = np.asarray(rows, dtype=np.float32)
    except Exception as exc:
        logger.debug("maps saved-map points conversion failed: %s", exc)
        return None
    if arr.ndim != 2 or arr.shape[1] < 3:
        return None
    return np.ascontiguousarray(arr[:, :3], dtype=np.float32)


def map_bundle(gw: Any, map_name: str, capability: str) -> dict[str, Any] | None:
    try:
        resp = map_service_query(
            gw,
            {
                "action": "get_map_bundle",
                "name": map_name,
                "capability": capability,
            },
        )
    except Exception as exc:
        logger.debug("maps bundle query failed: %s", exc)
        return None
    if not isinstance(resp, dict) or resp.get("success") is not True:
        return None
    artifact = resp.get("artifact")
    if not isinstance(artifact, dict):
        return None
    uri = str(artifact.get("uri") or "").strip()
    path = str(artifact.get("path") or "").strip()
    if not uri and not path:
        return None
    return resp


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
    """Validate a saved-map package through the maps service query API."""
    try:
        resp = map_service_query(
            gw,
            {
                "action": "validate_artifacts",
                "name": map_name,
                "require_octomap": require_octomap,
                "require_occupancy": require_occupancy,
                "expected_data_source": expected_data_source,
                "expected_source_profile": expected_source_profile,
                "expected_frame_id": expected_frame_id,
            },
        )
    except Exception as exc:
        logger.debug("maps artifact validation query failed: %s", exc)
        return None
    return resp if isinstance(resp, dict) and resp.get("success") is True else None


def artifact_path(gw: Any, map_name: str, capability: str) -> Path | None:
    bundle = map_bundle(gw, map_name, capability)
    if bundle is None:
        return None
    artifact = bundle.get("artifact")
    if not isinstance(artifact, dict):
        return None
    raw = str(artifact.get("uri") or artifact.get("path") or "").strip()
    if not raw:
        return None
    base_raw = str(bundle.get("map_dir") or "").strip()
    if not base_raw:
        return None
    try:
        base = Path(base_raw).resolve(strict=True)
        path = Path(raw)
        candidate = path.resolve(strict=True) if path.is_absolute() else (base / path).resolve(strict=True)
        candidate.relative_to(base)
    except (OSError, ValueError):
        return None
    return candidate if candidate.is_file() else None
