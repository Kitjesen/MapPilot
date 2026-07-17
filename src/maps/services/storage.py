"""Native map storage boundary.

Map ids, active-map state, records, artifacts, and lifecycle operations are
owned by ``lingtu_maps``. This module only adapts those native services to the
Python runtime. There is intentionally no Python storage fallback.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

from maps.adapters.python.service import (
    MapsServiceNativeUnavailable,
    NativeMapsService,
)
from maps.adapters.python.store import MapStoreNativeUnavailable, NativeMapStore

MAP_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


class InvalidMapName(ValueError):
    """Raised when a map id is unsafe."""


def safe_map_name(name: str) -> str | None:
    """Return a validation error, or ``None`` for a safe map id."""

    if not isinstance(name, str) or not name:
        return "empty name"
    if len(name) > 100:
        return "name too long (max 100)"
    if "/" in name or "\\" in name or ".." in name:
        return f"unsafe characters in name: {name!r}"
    if name[0] in ".-":
        return f"name cannot start with . or -: {name!r}"
    if not MAP_NAME_RE.fullmatch(name):
        return f"only [A-Za-z0-9_.-] allowed: {name!r}"
    return None


def validate_map_name(name: str) -> str:
    """Validate and normalize a single map id component."""

    value = str(name or "").strip()
    error = safe_map_name(value)
    if error:
        raise InvalidMapName(f"invalid map name: {name!r} ({error})")
    path = Path(value)
    if path.is_absolute():
        raise InvalidMapName(f"invalid map name: {name}")
    if any(part in {"", ".", ".."} for part in path.parts):
        raise InvalidMapName(f"invalid map name: {name}")
    if len(path.parts) != 1:
        raise InvalidMapName(f"invalid map name: {name}")
    return value


class MapStorageService:
    """Thin runtime adapter over native map store and service APIs.

    Python cannot implement map lifecycle behavior when ``lingtu_maps`` is
    absent.
    """

    def __init__(self, *, data_dir: str | Path, map_dir: str | Path) -> None:
        self.data_dir = Path(data_dir)
        self.map_dir = Path(map_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.map_dir.mkdir(parents=True, exist_ok=True)

        try:
            self.native_store = NativeMapStore(self.map_dir)
            self.native_service = NativeMapsService(self.map_dir)
        except (MapStoreNativeUnavailable, MapsServiceNativeUnavailable) as exc:
            raise RuntimeError(
                "MapStorageService requires the native lingtu_maps library; the Python file fallback has been removed"
            ) from exc

        self.native_backend = "lingtu_maps"
        self.native_unavailable_reason = ""
        self.active_map = self.native_store.active_map_id()

    def close(self) -> None:
        self.native_service.close()
        self.native_store.close()

    def map_path(self, name: str) -> Path:
        return self.map_dir / validate_map_name(name)

    def list_map_ids(self) -> list[str]:
        return self.native_store.list_map_ids()

    def create_map(self, name: str) -> dict[str, Any]:
        value = validate_map_name(name)
        response = self.native_service.create_map(value)
        if response.get("success") is not True:
            raise RuntimeError(str(response.get("message") or f"failed to create map: {value}"))
        return response

    def rename_map(self, name: str, new_name: str) -> dict[str, Any]:
        value = validate_map_name(name)
        new_value = validate_map_name(new_name)
        response = self.native_service.rename_map(value, new_value)
        if response.get("success") is not True:
            raise RuntimeError(str(response.get("message") or f"failed to rename map: {value}"))
        if self.active_map == value:
            self.active_map = new_value
        return response

    def retire_map(self, name: str) -> dict[str, Any]:
        value = validate_map_name(name)
        response = self.native_service.retire_map(value)
        if response.get("success") is True and self.active_map == value:
            self.active_map = ""
        return response

    def delete_map(self, name: str) -> dict[str, Any]:
        value = validate_map_name(name)
        response = self.native_service.delete_map(value)
        if response.get("success") is True and self.active_map == value:
            self.active_map = ""
        return response

    def set_active_map(self, name: str, *, strict: bool = False) -> dict[str, Any]:
        value = validate_map_name(name)
        response = self.native_service.set_active_map(value, strict=strict)
        if response.get("success") is not True:
            raise RuntimeError(str(response.get("message") or f"failed to set active map: {value}"))
        self.active_map = value
        return response

    def clear_active_map(self) -> dict[str, Any]:
        response = self.native_service.clear_active_map()
        if response.get("success") is not True:
            raise RuntimeError(str(response.get("message") or "failed to clear active map"))
        self.active_map = ""
        return response

    def native_record(self, name: str) -> dict[str, Any] | None:
        return self.native_store.record(validate_map_name(name))

    def save_active_map(self) -> None:
        if self.active_map:
            self.set_active_map(self.active_map, strict=False)
        else:
            self.clear_active_map()

    def get_active_octomap(self) -> str | None:
        response = self.native_service.get_active_map()
        if response.get("success") is not True:
            return None
        return str((response.get("artifacts") or {}).get("octomap") or "") or None

    def get_active_occupancy(self) -> str | None:
        response = self.native_service.get_active_map()
        if response.get("success") is not True:
            return None
        return str((response.get("artifacts") or {}).get("occupancy") or "") or None

    def get_active_artifacts(self) -> dict[str, Any]:
        """Return the native active-map artifact view."""

        response = self.native_service.get_active_map()
        if response.get("success") is not True:
            return {"record": None, "artifacts": {}, "map_classes": []}
        active_artifacts = dict(response.get("artifacts") or {})
        record = response.get("record") if isinstance(response.get("record"), dict) else {}
        inventory: dict[str, dict[str, Any]] = {}
        map_classes: set[str] = set()
        for artifact in record.get("artifacts") or []:
            if not isinstance(artifact, dict):
                continue
            name = str(artifact.get("name") or artifact.get("type") or "")
            map_class = str(artifact.get("map_class") or "")
            if map_class:
                map_classes.add(map_class)
            inventory.setdefault(
                name,
                {
                    "path": str(artifact.get("uri") or ""),
                    "exists": True,
                    "map_class": map_class,
                    "role": str(artifact.get("role") or ""),
                },
            )
        return {
            **active_artifacts,
            "artifacts": inventory,
            "map_classes": sorted(map_classes),
            "record": record,
        }
