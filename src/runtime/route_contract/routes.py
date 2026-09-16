"""Built-in route presets."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from message.catalog import load_topics
from message.topics import TOPICS

from .model import RouteBackend, RouteSpec

_REPLAY_LCM_BINDINGS = {
    TOPICS.lidar_scan: {
        "channel": "LT_LIDAR_RAW_FRAME",
        "type": "lingtu_lidar_frame_t",
    },
    TOPICS.imu: {
        "channel": "LT_IMU_RAW",
        "type": "lingtu_imu_t",
    },
    TOPICS.odometry: {
        "channel": "LT_SLAM_ODOMETRY",
        "type": "lingtu_odometry_t",
    },
    TOPICS.map_cloud: {
        "channel": "LT_SLAM_MAP_CLOUD",
        "type": "lingtu_pointcloud2_t",
    },
    TOPICS.localization_health: {
        "channel": "LT_SLAM_LOCALIZATION_HEALTH",
        "type": "lingtu_text_t",
    },
    TOPICS.cmd_vel: {
        "channel": "LT_NAV_CMD_VEL",
        "type": "lingtu_twist_stamped_t",
        "single_writer": True,
    },
}


def robot() -> RouteSpec:
    """Physical robot route using typed DDS at native service boundaries."""

    bindings = {
        spec["topic"]: ({"single_writer": True} if spec.get("single_writer_per_product") else {})
        for spec in load_topics()
        if spec.get("transport") == RouteBackend.DDS.value
    }

    return RouteSpec(
        name="robot",
        description="Physical robot route. Native service boundaries use typed DDS.",
        default=RouteBackend.LOCAL.value,
        routes={topic: RouteBackend.DDS.value for topic in bindings},
        bindings={
            RouteBackend.DDS.value: bindings,
        },
    )


def replay() -> RouteSpec:
    """Replay/development route with typed LCM bindings where available."""

    return RouteSpec(
        name="replay",
        description=("Replay/development route. Canonical topics use typed LCM bindings where available."),
        default=RouteBackend.LOCAL.value,
        routes={topic: RouteBackend.LCM.value for topic in _REPLAY_LCM_BINDINGS},
        bindings={
            RouteBackend.LCM.value: _copy_bindings(_REPLAY_LCM_BINDINGS),
        },
    )


def sim() -> RouteSpec:
    """In-process simulation route."""

    return RouteSpec(
        name="sim",
        description=(
            "In-process simulation route. Module ports use local callback delivery unless explicitly overridden."
        ),
        default=RouteBackend.LOCAL.value,
    )


ROUTE_PRESETS: dict[str, Callable[[], RouteSpec]] = {
    "robot": robot,
    "replay": replay,
    "sim": sim,
}


def route_preset(name: str) -> RouteSpec:
    """Return one built-in route preset by name."""

    key = str(name).strip().lower()
    try:
        factory = ROUTE_PRESETS[key]
    except KeyError as exc:
        available = ", ".join(sorted(ROUTE_PRESETS))
        raise KeyError(f"Unknown route preset {key!r}; available: {available}") from exc
    return factory()


def _copy_bindings(bindings: dict[str, dict[str, Any]]) -> dict[str, dict[str, Any]]:
    return {topic: dict(binding) for topic, binding in bindings.items()}
