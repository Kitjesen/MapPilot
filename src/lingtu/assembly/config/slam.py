"""Render the SLAM runtime settings for the native launcher."""

import math
from collections.abc import Mapping
from typing import Any

_FIELDS = (
    "tick_hz",
    "log_status_s",
    "status_json_hz",
    "cloud_snapshot_hz",
    "lidar_scan_snapshot_hz",
    "track_against_map_period_s",
)


def slam_environment(settings: Mapping[str, Any], *, simulation: bool) -> dict[str, str]:
    runtime = settings["runtime"]
    sim = runtime["sim"]
    values = {key: runtime[key] for key in _FIELDS}
    if simulation:
        values.update(sim)
    environment = {}
    for key, value in values.items():
        if key not in _FIELDS:
            raise ValueError(f"unknown slam.runtime setting: {key}")
        if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(value):
            raise ValueError(f"slam.runtime.{key} must be a finite number")
        if key == "tick_hz" and value < 1:
            raise ValueError("slam.runtime.tick_hz must be at least 1 Hz")
        if key == "track_against_map_period_s" and value != 0 and value < 1:
            raise ValueError("slam.runtime.track_against_map_period_s must be 0 or at least 1 second")
        if value < 0:
            raise ValueError(f"slam.runtime.{key} has an invalid rate or interval")
        environment[f"LINGTU_SLAM_{key.upper()}"] = format(value, ".15g")
    return environment
