"""Query and POI API layer for MapService."""

from __future__ import annotations

from typing import Any

from nav.services.map.records import (
    MAP_BUNDLE_SCHEMA,
    artifact_for_capability,
    map_type_catalog,
)
from nav.services.map.storage import (
    InvalidMapName,
    MapStorageService,
    artifact_inventory,
    existing_map_classes,
)


class MapAPIService:
    """Read/query side of the Spatial Data Service."""

    def __init__(self, *, storage: MapStorageService) -> None:
        self.storage = storage

    def get_map_types(self) -> dict[str, Any]:
        """Return the stable map class, artifact, and capability catalog."""
        catalog = map_type_catalog()
        return {
            "action": "get_map_types",
            "success": True,
            "schema_version": catalog["schema_version"],
            "record_schema_version": catalog["record_schema_version"],
            "bundle_schema_version": catalog["bundle_schema_version"],
            "catalog": catalog,
            "states": catalog["states"],
            "classes": catalog["classes"],
            "artifacts": catalog["artifacts"],
            "aliases": catalog["aliases"],
            "capabilities": catalog["capabilities"],
        }

    def list_maps(self) -> dict[str, Any]:
        """List map directories and their available artifacts."""
        maps: list[dict[str, Any]] = []
        for entry in sorted(self.storage.map_dir.iterdir()):
            if not entry.is_dir() or entry.name == "active":
                continue
            artifacts = artifact_inventory(entry)
            record = self.storage.build_record_for_dir(entry, entry.name).to_dict()
            maps.append(
                {
                    "name": entry.name,
                    "has_pcd": (entry / "map.pcd").exists(),
                    "has_tomogram": (entry / "tomogram.pickle").exists(),
                    "has_occupancy": (entry / "occupancy.npz").exists(),
                    "has_octomap": (entry / "octomap.ot").exists()
                    or (entry / "octomap.bt").exists(),
                    "map_classes": existing_map_classes(artifacts),
                    "artifacts": artifacts,
                    "record": record,
                    "capabilities": record["capabilities"],
                    "health": record["health"],
                    "state": record["state"],
                }
            )
        return {
            "action": "list",
            "success": True,
            "maps": maps,
            "active": self.storage.active_map,
        }

    def get_record(self, name: str) -> dict[str, Any]:
        if not name:
            return {
                "action": "get_record",
                "success": False,
                "message": "missing map name",
            }
        try:
            map_dir = self.storage.map_path(name)
        except InvalidMapName as exc:
            return {
                "action": "get_record",
                "success": False,
                "message": str(exc),
            }
        if not map_dir.is_dir():
            return {
                "action": "get_record",
                "success": False,
                "message": f"map not found: {name}",
            }
        return {
            "action": "get_record",
            "success": True,
            "record": self.storage.build_record_for_dir(map_dir, name).to_dict(),
        }

    def get_active_map(self) -> dict[str, Any]:
        active_dir = self.storage.active_link()
        if not active_dir.exists() or not self.storage.active_map:
            return {
                "action": "get_active",
                "success": False,
                "message": "no active map",
            }
        return {
            "action": "get_active",
            "success": True,
            "active": self.storage.active_map,
            "artifacts": self.storage.get_active_artifacts(),
            "record": self.storage.build_record_for_dir(
                self.storage.map_path(self.storage.active_map),
                self.storage.active_map,
                state="ACTIVE",
            ).to_dict(),
        }

    def get_map_health(self, name: str) -> dict[str, Any]:
        if not name:
            name = self.storage.active_map
        if not name:
            return {
                "action": "get_health",
                "success": False,
                "message": "missing map name",
            }
        try:
            map_dir = self.storage.map_path(name)
        except InvalidMapName as exc:
            return {
                "action": "get_health",
                "success": False,
                "message": str(exc),
            }
        if not map_dir.is_dir():
            return {
                "action": "get_health",
                "success": False,
                "message": f"map not found: {name}",
            }
        record = self.storage.build_record_for_dir(
            map_dir,
            name,
        ).to_dict()
        return {
            "action": "get_health",
            "success": True,
            "map_id": record["map_id"],
            "health": record["health"],
        }

    def get_map_bundle(self, name: str, capability: str) -> dict[str, Any]:
        if not name:
            name = self.storage.active_map
        if not capability:
            return {
                "action": "get_map_bundle",
                "success": False,
                "message": "missing capability",
            }
        if not name:
            return {
                "action": "get_map_bundle",
                "success": False,
                "message": "missing map name",
            }
        try:
            map_dir = self.storage.map_path(name)
        except InvalidMapName as exc:
            return {
                "action": "get_map_bundle",
                "success": False,
                "message": str(exc),
            }
        if not map_dir.is_dir():
            return {
                "action": "get_map_bundle",
                "success": False,
                "message": f"map not found: {name}",
            }
        record = self.storage.build_record_for_dir(
            map_dir,
            name,
        ).to_dict()
        scope = record.get("scope") or {}
        artifact = artifact_for_capability(record, capability)
        if artifact is None:
            return {
                "action": "get_map_bundle",
                "success": False,
                "schema_version": MAP_BUNDLE_SCHEMA,
                "map_id": name,
                "version_id": record.get("version_id", ""),
                "state": record.get("state", ""),
                "capability": capability,
                "message": f"capability unavailable: {capability}",
                "available_capabilities": record.get("capabilities", []),
                "health": record["health"],
            }
        return {
            "action": "get_map_bundle",
            "success": True,
            "schema_version": MAP_BUNDLE_SCHEMA,
            "map_id": record["map_id"],
            "version_id": record.get("version_id", ""),
            "state": record["state"],
            "frame_id": scope.get("frame_id"),
            "map_dir": scope.get("map_dir"),
            "capability": capability,
            "artifact": artifact,
            "artifacts": record["artifacts"],
            "available_capabilities": record.get("capabilities", []),
            "health": record["health"],
            "record": record,
        }

    def poi_set(self, cmd: dict[str, Any]) -> dict[str, Any]:
        name = str(cmd.get("name", ""))
        if not name:
            return {"action": "poi_set", "success": False, "message": "missing name"}
        self.storage.pois[name] = {
            "x": float(cmd.get("x", 0.0)),
            "y": float(cmd.get("y", 0.0)),
            "z": float(cmd.get("z", 0.0)),
        }
        self.storage.save_pois()
        return {"action": "poi_set", "success": True, "message": f"POI set: {name}"}

    def poi_delete(self, name: str) -> dict[str, Any]:
        if name not in self.storage.pois:
            return {
                "action": "poi_delete",
                "success": False,
                "message": f"POI not found: {name}",
            }
        del self.storage.pois[name]
        self.storage.save_pois()
        return {"action": "poi_delete", "success": True, "message": f"POI deleted: {name}"}

    def poi_list(self) -> dict[str, Any]:
        return {
            "action": "poi_list",
            "success": True,
            "pois": dict(self.storage.pois),
        }
