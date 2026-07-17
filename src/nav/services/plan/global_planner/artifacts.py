"""Planner-name to map-capability selection.

Active-map layout and record lookup are owned by :mod:`maps.services`.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from maps.services.active_assets import ActiveMapAssets, default_map_roots
from runtime.profiles.planner_backends import normalize_planner_name

PLANNER_CAPABILITY_ORDER = {
    "octoplanner3d": ("navigation_safety_3d",),
    "astar": ("path_planning_2d",),
}


def default_map_dirs() -> tuple[Path, ...]:
    """Return map roots exposed by the maps-domain resolver."""
    return default_map_roots()


@dataclass(frozen=True)
class SavedMapArtifacts:
    """Resolve active saved-map artifacts for a planner backend."""

    explicit_path: str = ""
    map_dirs: tuple[Path, ...] = ()

    @classmethod
    def from_runtime(cls, explicit_path: str = "") -> SavedMapArtifacts:
        return cls(
            explicit_path=str(explicit_path or ""),
            map_dirs=default_map_dirs(),
        )

    def existing_explicit_path(self) -> str:
        path = str(self.explicit_path or "")
        if path and os.path.exists(path):
            return path
        return ""

    def _assets(self) -> ActiveMapAssets:
        return ActiveMapAssets(map_roots=self.map_dirs)

    def active_artifact(self, filename: str) -> str:
        return self._assets().active_artifact(filename)

    def planner_map_bundle(self, planner_name: str) -> dict[str, Any]:
        planner = normalize_planner_name(planner_name)
        for capability in PLANNER_CAPABILITY_ORDER.get(planner, ()):
            bundle = self.active_bundle(capability)
            if bundle:
                return bundle
        return {}

    def active_bundle(self, capability: str) -> dict[str, Any]:
        return self._assets().active_bundle(capability)

    def validate_artifact_path(
        self,
        artifact_path: str,
        *,
        require_octomap: bool = False,
        require_occupancy: bool = False,
        expected_frame_id: str | None = None,
    ) -> dict[str, Any]:
        return self._assets().validate_artifact_path(
            artifact_path,
            require_octomap=require_octomap,
            require_occupancy=require_occupancy,
            expected_frame_id=expected_frame_id,
        )

    def planner_map_path(self, planner_name: str) -> str:
        """Return the saved-map artifact a backend should receive."""

        planner = normalize_planner_name(planner_name)
        if planner == "octoplanner3d":
            explicit = self.existing_explicit_path()
            if explicit and Path(explicit).suffix.lower() in {".bt", ".ot", ".octomap"}:
                return explicit
            octomap = self.active_artifact("octomap.ot") or self.active_artifact("octomap.bt")
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
        if planner == "astar":
            return self.active_artifact("occupancy.npz")
        return ""

    def static_occupancy_path(self, map_path: str = "") -> str:
        return self._assets().static_occupancy_path(map_path)
