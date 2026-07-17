"""Runtime path defaults shared by product and compatibility profile catalogs."""

from __future__ import annotations

import os
from pathlib import Path

from runtime.runtime_interface import map_frame_id, odom_frame_id

DEFAULT_GATEWAY_PORT = 5050
DEFAULT_SAMPLE_OCTOPLANNER3D_MAP = "third_party/OctoPlanner3D-ROS2/result_cleaned.bt"
_OCTOPLANNER3D_MAP_CANDIDATES = (
    "third_party/OctoPlanner3D-ROS2/result_cleaned.bt",
    "third_party/OctoPlanner3D-ROS2/build_official_pcl/result_cleaned.bt",
    "third_party/OctoPlanner3D/build/result_cleaned.bt",
    "artifacts/external/OctoPlanner3D/build/result_cleaned.bt",
    "artifacts/octoplanner3d_official_pcd_test/official_result_cleaned.bt",
)


def _maps_path_helpers():
    """Return canonical maps-domain path helpers when the maps package is present."""
    try:
        from maps.paths import active_map_dir, nav_map_root_str
    except ModuleNotFoundError as exc:
        if exc.name not in {"maps", "maps.paths"}:
            raise
        return None
    return active_map_dir, nav_map_root_str


def _default_map_dir() -> str:
    """Return the canonical persistent map root."""
    helpers = _maps_path_helpers()
    if helpers is None:
        return os.environ.get("NAV_MAP_DIR", "")
    _, nav_map_root_str = helpers
    return nav_map_root_str()


def _resolve_octoplanner3d_map() -> str:
    """Return an OctoPlanner3D-compatible OctoMap/PCD path."""

    for env_name in ("LINGTU_OCTOPLANNER3D_MAP", "NAV_OCTOMAP"):
        env_value = os.environ.get(env_name)
        if env_value:
            return env_value

    helpers = _maps_path_helpers()
    map_dir = _default_map_dir()
    if helpers is not None and map_dir:
        active_map_dir, _ = helpers
        selected = active_map_dir(Path(map_dir))
        if selected is not None:
            for filename in ("octomap.ot", "octomap.bt"):
                active = selected / filename
                if active.is_file():
                    return str(active)

    repo = Path(__file__).resolve().parents[4]
    for candidate in _OCTOPLANNER3D_MAP_CANDIDATES:
        path = repo.joinpath(*candidate.split("/"))
        if path.is_file():
            return str(path)
    return str(repo.joinpath(*DEFAULT_SAMPLE_OCTOPLANNER3D_MAP.split("/")))


RUNTIME_MAP_FRAME_ID = map_frame_id()
RUNTIME_ODOM_FRAME_ID = odom_frame_id()
DEFAULT_PLANNING_FRAME_ID = RUNTIME_MAP_FRAME_ID
