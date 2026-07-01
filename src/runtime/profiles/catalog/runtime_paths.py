"""Runtime path defaults shared by product and compatibility profile catalogs."""

from __future__ import annotations

import os
from pathlib import Path

from runtime.runtime_interface import map_frame_id, odom_frame_id


DEFAULT_GATEWAY_PORT = 5050
DEFAULT_SAMPLE_TOMOGRAM = (
    "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/rsc/tomogram/building2_9.pickle"
)
DEFAULT_SAMPLE_OCTOPLANNER3D_MAP = (
    "third_party/OctoPlanner3D-ROS2/result_cleaned.bt"
)
_OCTOPLANNER3D_MAP_CANDIDATES = (
    "third_party/OctoPlanner3D-ROS2/result_cleaned.bt",
    "third_party/OctoPlanner3D-ROS2/build_official_pcl/result_cleaned.bt",
    "third_party/OctoPlanner3D/build/result_cleaned.bt",
    "artifacts/external/OctoPlanner3D/build/result_cleaned.bt",
    "artifacts/octoplanner3d_official_pcd_test/official_result_cleaned.bt",
)


def _default_map_dir() -> str:
    """Return the default on-robot map storage directory.

    Maps are runtime data and should live outside the repo so they survive code
    updates and can be shared across profiles.

    Order:
      1) $NAV_MAP_DIR, if set
      2) legacy: ~/data/nova/maps
      3) new default: ~/data/lingtu/maps
    """

    env = os.environ.get("NAV_MAP_DIR")
    if env:
        return env

    legacy = os.path.expanduser("~/data/nova/maps")
    if os.path.isdir(legacy):
        return legacy

    return os.path.expanduser("~/data/lingtu/maps")


def _resolve_tomogram() -> str:
    """Return the active tomogram path, falling back to the built-in sample map."""

    active = os.path.join(
        _default_map_dir(),
        "active",
        "tomogram.pickle",
    )
    if os.path.isfile(active):
        return active

    repo = Path(__file__).resolve().parents[4]
    return str(repo.joinpath(*DEFAULT_SAMPLE_TOMOGRAM.split("/")))


def _resolve_octoplanner3d_map() -> str:
    """Return an OctoPlanner3D-compatible OctoMap/PCD path."""

    for env_name in ("LINGTU_OCTOPLANNER3D_MAP", "NAV_OCTOMAP"):
        env_value = os.environ.get(env_name)
        if env_value:
            return env_value

    active_dir = Path(_default_map_dir()) / "active"
    for filename in ("map.bt", "map.ot", "map.octomap", "map.pcd"):
        active = active_dir / filename
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
