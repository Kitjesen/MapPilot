"""File-backed map storage service and artifact inventory helpers."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

from nav.services.map.records import (
    build_map_record,
    load_map_record,
    map_artifact_catalog,
    write_map_record,
)
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.same_source_map_artifacts import (
    validate_saved_map_artifact_dir,
)
from runtime.yaml_helpers import load_yaml, save_yaml

MAP_ARTIFACT_CATALOG: dict[str, dict[str, str]] = map_artifact_catalog(
    include_aliases=True,
)

MAP_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


class InvalidMapName(ValueError):
    """Raised when a map name is unsafe for file-backed storage."""


def validate_map_name(name: str) -> str:
    value = str(name or "").strip()
    if not value:
        raise InvalidMapName("missing map name")
    path = Path(value)
    if path.is_absolute():
        raise InvalidMapName(f"invalid map name: {name}")
    if any(part in {"", ".", ".."} for part in path.parts):
        raise InvalidMapName(f"invalid map name: {name}")
    if len(path.parts) != 1:
        raise InvalidMapName(f"invalid map name: {name}")
    if value[0] in ".-":
        raise InvalidMapName(f"invalid map name: {name}")
    if not MAP_NAME_RE.fullmatch(value):
        raise InvalidMapName(f"invalid map name: {name}")
    return value


def artifact_inventory(map_dir: Path) -> dict[str, dict[str, Any]]:
    artifacts: dict[str, dict[str, Any]] = {}
    for name, spec in MAP_ARTIFACT_CATALOG.items():
        path = map_dir / spec["filename"]
        artifacts[name] = {
            "path": str(path),
            "exists": path.exists(),
            "map_class": spec["map_class"],
            "role": spec["role"],
        }
    return artifacts


def existing_map_classes(artifacts: dict[str, dict[str, Any]]) -> list[str]:
    return sorted(
        {
            str(artifact["map_class"])
            for artifact in artifacts.values()
            if artifact.get("exists")
        }
    )


class MapStorageService:
    """Owns file-backed MapService state.

    This service is intentionally small and synchronous.  It owns paths,
    active-map persistence, POI persistence, and MapRecord read/write helpers;
    it does not build artifacts or decide whether a map may become active.
    """

    def __init__(self, *, data_dir: str | Path, map_dir: str | Path) -> None:
        self.data_dir = Path(data_dir)
        self.map_dir = Path(map_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.map_dir.mkdir(parents=True, exist_ok=True)

        self.poi_file = self.data_dir / "pois.yaml"
        self.active_map_file = self.data_dir / "active_map.yaml"
        self.pois: dict[str, dict[str, float]] = load_yaml(
            self.poi_file, default={}
        )
        self.active_map: str = load_yaml(
            self.active_map_file, default={}
        ).get("active", "")

    def map_path(self, name: str) -> Path:
        return self.map_dir / validate_map_name(name)

    def active_link(self) -> Path:
        return self.map_dir / "active"

    def save_active_map(self) -> None:
        save_yaml(self.active_map_file, {"active": self.active_map})

    def save_pois(self) -> None:
        save_yaml(self.poi_file, self.pois)

    def build_record_for_dir(
        self,
        map_dir: Path,
        map_id: str,
        *,
        state: str | None = None,
        gate: dict[str, Any] | None = None,
    ) -> Any:
        if gate is None and map_dir.is_dir() and (map_dir / "metadata.json").exists():
            gate = validate_saved_map_artifact_dir(
                map_dir,
                expected_frame_id=topic_default_frame_id(TOPICS.saved_map_cloud),
            )
        loaded = load_map_record(map_dir) or {}
        resolved_state = state or str(loaded.get("state") or "READY")
        return build_map_record(
            map_dir,
            map_id=map_id,
            gate=gate,
            state=resolved_state,
        )

    def write_map_record(
        self,
        name: str,
        *,
        state: str = "READY",
        gate: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        map_dir = self.map_path(name)
        record = self.build_record_for_dir(
            map_dir,
            name,
            state=state,
            gate=gate,
        )
        path = write_map_record(map_dir, record)
        payload = record.to_dict()
        payload["path"] = str(path)
        return payload

    def get_active_tomogram(self) -> str | None:
        active = self.active_link() / "tomogram.pickle"
        if active.exists():
            return str(active)
        return None

    def get_active_octomap(self) -> str | None:
        active_dir = self.active_link()
        for filename in ("octomap.ot", "octomap.bt"):
            active = active_dir / filename
            if active.exists():
                return str(active)
        return None

    def get_active_occupancy(self) -> str | None:
        active = self.active_link() / "occupancy.npz"
        if active.exists():
            return str(active)
        return None

    def get_active_artifacts(self) -> dict[str, Any]:
        active_dir = self.active_link()
        artifacts = artifact_inventory(active_dir)
        record = (
            self.build_record_for_dir(
                self.map_path(self.active_map),
                self.active_map,
                state="ACTIVE",
            ).to_dict()
            if self.active_map
            else None
        )
        return {
            "map_dir": str(active_dir) if active_dir.exists() else None,
            "map_pcd": str(active_dir / "map.pcd") if (active_dir / "map.pcd").exists() else None,
            "octomap": self.get_active_octomap(),
            "tomogram": self.get_active_tomogram(),
            "occupancy": self.get_active_occupancy(),
            "metadata": str(active_dir / "metadata.json") if (active_dir / "metadata.json").exists() else None,
            "artifacts": artifacts,
            "map_classes": existing_map_classes(artifacts),
            "record": record,
        }
