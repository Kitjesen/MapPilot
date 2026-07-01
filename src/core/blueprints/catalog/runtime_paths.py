"""Runtime path defaults shared by product and compatibility profile catalogs."""

from __future__ import annotations

import os
from pathlib import Path

from core.runtime_interface import map_frame_id


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
    return str(repo.joinpath(
        "src",
        "global_planning",
        "pct_planner",
        "rsc",
        "tomogram",
        "building2_9.pickle",
    ))


RUNTIME_MAP_FRAME_ID = map_frame_id()
