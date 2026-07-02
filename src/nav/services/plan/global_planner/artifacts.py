"""Saved-map artifact lookup for global planning.

Global planning has two map inputs:

- saved-map artifacts under ``<map_dir>/active`` for the planner backend
- live costmap updates pushed later through ``GlobalPlanner.update_map``

This module owns only the saved-map artifact lookup. It deliberately does not
touch point clouds, costmaps, or backend logic.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from nav.services.map.records import (
    MAP_BUNDLE_SCHEMA,
    artifact_for_capability,
    load_map_record,
)
from runtime.profiles.planner_backends import normalize_planner_name

PLANNER_CAPABILITY_ORDER = {
    "octoplanner3d": ("navigation_safety_3d",),
    "pct": ("terrain_reasoning",),
    "astar": ("path_planning_2d", "terrain_reasoning"),
}


def default_map_dirs() -> tuple[Path, ...]:
    """Return map roots searched by runtime planning, in priority order."""

    candidates: list[Path] = []
    for value in (
        os.environ.get("NAV_MAP_DIR", ""),
        os.path.expanduser("~/data/nova/maps"),
        os.path.expanduser("~/data/inovxio/data/maps"),
    ):
        if not value:
            continue
        path = Path(value)
        if path not in candidates:
            candidates.append(path)
    return tuple(candidates)


@dataclass(frozen=True)
class SavedMapArtifacts:
    """Resolve active saved-map artifacts for a planner backend."""

    explicit_path: str = ""
    map_dirs: tuple[Path, ...] = ()

    @classmethod
    def from_runtime(cls, explicit_path: str = "") -> "SavedMapArtifacts":
        return cls(
            explicit_path=str(explicit_path or ""),
            map_dirs=default_map_dirs(),
        )

    def existing_explicit_path(self) -> str:
        path = str(self.explicit_path or "")
        if path and os.path.exists(path):
            return path
        return ""

    def active_artifact(self, filename: str) -> str:
        for map_dir in self.map_dirs:
            candidate = map_dir / "active" / filename
            if candidate.exists():
                return str(candidate)
        return ""

    def planner_map_bundle(self, planner_name: str) -> dict[str, Any]:
        planner = normalize_planner_name(planner_name)
        for capability in PLANNER_CAPABILITY_ORDER.get(planner, ()):
            bundle = self.active_bundle(capability)
            if bundle:
                return bundle
        return {}

    def active_bundle(self, capability: str) -> dict[str, Any]:
        for map_dir in self.map_dirs:
            active_dir = map_dir / "active"
            record = load_map_record(active_dir)
            if not record:
                continue
            artifact = artifact_for_capability(record, capability)
            if not artifact:
                continue
            artifact_path = self._artifact_path(active_dir, artifact)
            if not artifact_path:
                continue
            return {
                "schema_version": MAP_BUNDLE_SCHEMA,
                "map_id": str(record.get("map_id") or active_dir.name),
                "version_id": str(record.get("version_id") or ""),
                "state": str(record.get("state") or ""),
                "frame_id": str((record.get("scope") or {}).get("frame_id") or ""),
                "capability": capability,
                "artifact": {**artifact, "uri": artifact_path},
                "record": record,
            }
        return {}

    @staticmethod
    def _artifact_path(active_dir: Path, artifact: dict[str, Any]) -> str:
        uri = str(artifact.get("uri") or "")
        if not uri:
            return ""
        path = Path(uri)
        if not path.is_absolute():
            path = active_dir / path
        return str(path) if path.exists() else ""

    def planner_map_path(self, planner_name: str) -> str:
        """Return the saved-map artifact a backend should receive."""

        planner = normalize_planner_name(planner_name)
        if planner == "octoplanner3d":
            explicit = self.existing_explicit_path()
            if explicit and Path(explicit).suffix.lower() in {".bt", ".ot", ".octomap"}:
                return explicit
            octomap = (
                self.active_artifact("octomap.ot")
                or self.active_artifact("octomap.bt")
            )
            if octomap:
                return octomap
            if explicit:
                return explicit
        else:
            explicit = self.existing_explicit_path()
            if explicit:
                return explicit

        bundle = self.planner_map_bundle(planner)
        if bundle:
            return str((bundle.get("artifact") or {}).get("uri") or "")

        if planner == "octoplanner3d":
            return ""
        if planner == "pct":
            return self.active_artifact("tomogram.pickle")
        if planner == "astar":
            return (
                self.active_artifact("occupancy.npz")
                or self.active_artifact("tomogram.pickle")
            )
        return ""

    def static_occupancy_path(self, map_path: str = "") -> str:
        if map_path:
            sibling = Path(map_path).resolve().parent / "occupancy.npz"
            if sibling.exists():
                return str(sibling)
        bundle = self.active_bundle("path_planning_2d")
        if bundle:
            return str((bundle.get("artifact") or {}).get("uri") or "")
        return self.active_artifact("occupancy.npz")
