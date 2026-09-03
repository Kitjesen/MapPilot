"""Sensor plans compiled from tracked simulation sessions."""

from __future__ import annotations

import json
from functools import cache
from pathlib import Path
from typing import Any, cast

from sim.catalog import CatalogResolver
from sim.runtime.sensors import SensorRuntime

_REPO_ROOT = Path(__file__).resolve().parents[3]
_THUNDERV4_UNREAL_SESSION = (
    _REPO_ROOT / "sim" / "sessions" / "examples" / "thunderv4_unreal" / "session.yaml"
)


@cache
def _thunderv4_unreal_sensor_json() -> str:
    return CatalogResolver.from_repository(_REPO_ROOT).resolve(
        _THUNDERV4_UNREAL_SESSION
    ).sensor_json


def thunderv4_unreal_sensor_plan() -> dict[str, Any]:
    """Return a fresh copy of the tracked ThunderV4 Unreal sensor plan."""

    return cast(dict[str, Any], json.loads(_thunderv4_unreal_sensor_json()))


def thunderv4_unreal_sensor_runtime() -> SensorRuntime:
    """Build a sensor runtime without relying on ignored build output."""

    return SensorRuntime.from_plan(thunderv4_unreal_sensor_plan())
