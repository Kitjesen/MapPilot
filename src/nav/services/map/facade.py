"""MapService compatibility facade, public helpers, skills, and health."""

from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Any

from runtime import skill
from runtime.msgs.numpy_compat import np


class MapServiceFacadeMixin:
    @property
    def _pois(self) -> dict[str, dict[str, float]]:
        return self.storage.pois

    @_pois.setter
    def _pois(self, value: dict[str, dict[str, float]]) -> None:
        self.storage.pois = value

    @property
    def _poi_file(self) -> Path:
        return self.storage.poi_file

    @property
    def _active_map_file(self) -> Path:
        return self.storage.active_map_file

    @property
    def _active_map(self) -> str:
        return self.storage.active_map

    @_active_map.setter
    def _active_map(self, value: str) -> None:
        self.storage.active_map = value

    @property
    def _map_save_adapter(self) -> Any:
        return self.runtime_bridge.map_save_adapter

    @_map_save_adapter.setter
    def _map_save_adapter(self, value: Any) -> None:
        self.runtime_bridge.map_save_adapter = value

    @property
    def _map_save_timeout_sec(self) -> float:
        return self.runtime_bridge.map_save_timeout_sec

    @_map_save_timeout_sec.setter
    def _map_save_timeout_sec(self, value: float) -> None:
        self.runtime_bridge.map_save_timeout_sec = float(value)

    @property
    def _map_cloud_lock(self) -> threading.Lock:
        return self.runtime_bridge.map_cloud_lock

    @property
    def _latest_map_points(self) -> np.ndarray | None:
        return self.runtime_bridge.latest_map_points

    @_latest_map_points.setter
    def _latest_map_points(self, value: np.ndarray | None) -> None:
        self.runtime_bridge.latest_map_points = value

    @property
    def _map_artifact_converter_command(self) -> Any:
        return self.pipeline.map_artifact_converter_command

    @_map_artifact_converter_command.setter
    def _map_artifact_converter_command(self, value: Any) -> None:
        self.pipeline.map_artifact_converter_command = value

    @property
    def _octomap_build_mode(self) -> str:
        return self.pipeline.octomap_build_mode

    @_octomap_build_mode.setter
    def _octomap_build_mode(self, value: str) -> None:
        self.pipeline.octomap_build_mode = str(value)

    @property
    def _octomap_resolution(self) -> float:
        return self.pipeline.octomap_resolution

    @_octomap_resolution.setter
    def _octomap_resolution(self, value: float) -> None:
        self.pipeline.octomap_resolution = float(value)

    @property
    def _octomap_build_timeout_sec(self) -> float:
        return self.pipeline.octomap_build_timeout_sec

    @_octomap_build_timeout_sec.setter
    def _octomap_build_timeout_sec(self, value: float) -> None:
        self.pipeline.octomap_build_timeout_sec = float(value)

    @property
    def _build_octomap_on_save(self) -> bool:
        return self.pipeline.build_octomap_on_save

    @_build_octomap_on_save.setter
    def _build_octomap_on_save(self, value: bool) -> None:
        self.pipeline.build_octomap_on_save = bool(value)

    def get_active_tomogram(self) -> str | None:
        """Return the tomogram.pickle path for the active map, or None.

        Navigation uses this to locate the PCT planner index at startup.
        """
        return self.storage.get_active_tomogram()

    def get_active_octomap(self) -> str | None:
        """Return the active OctoMap path for the active map, or None."""
        return self.storage.get_active_octomap()

    def get_active_occupancy(self) -> str | None:
        """Return the occupancy.npz path for the active map, or None.

        A* backend and Dashboard may use this as a static planning grid.
        Load with: data = np.load(path); grid=data['grid'], res=data['resolution'],
        origin=data['origin'].
        """
        return self.storage.get_active_occupancy()

    def get_active_artifacts(self) -> dict[str, Any]:
        """Return active runtime artifact paths for map consumers."""
        return self.storage.get_active_artifacts()

    def get_map_types(self) -> dict[str, Any]:
        """Return the stable map class, artifact, and capability catalog."""
        return self.api.get_map_types()

    @skill
    def list_maps(self) -> str:
        """List saved maps and which one is active."""
        return json.dumps(self._map_list(), default=str)

    @skill
    def list_map_types(self) -> str:
        """List supported map classes, artifacts, and capabilities."""
        return json.dumps(self.get_map_types(), default=str)

    @skill
    def save_map(self, name: str, slam_profile: str | None = None) -> str:
        """Save current SLAM map as *name* and build all artifacts."""
        return json.dumps(self._map_save(name, slam_profile=slam_profile), default=str)

    @skill
    def use_map(self, name: str) -> str:
        """Activate *name* as the current map (symlink + planner index path)."""
        return json.dumps(self._map_set_active(name), default=str)

    @skill
    def build_tomogram(self, name: str) -> str:
        """Build tomogram.pickle from map.pcd for map *name*."""
        return json.dumps(self._build_tomogram(name), default=str)

    def _poi_set(self, cmd: dict[str, Any]) -> dict[str, Any]:
        return self.api.poi_set(cmd)

    def _poi_delete(self, name: str) -> dict[str, Any]:
        return self.api.poi_delete(name)

    def _poi_list(self) -> dict[str, Any]:
        return self.api.poi_list()

    def health(self) -> dict[str, Any]:
        info = self.port_summary()
        maps = [
            e.name for e in sorted(self._map_dir.iterdir())
            if e.is_dir() and e.name != "active"
        ] if self._map_dir.exists() else []
        info["map_count"] = len(maps)
        info["active_map"] = self._active_map
        info["poi_count"] = len(self._pois)
        info["map_dir"] = str(self._map_dir)
        return info
