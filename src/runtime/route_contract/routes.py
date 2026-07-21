"""Built-in route presets."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from runtime.runtime_interface import TOPICS

from .model import RouteBackend, RouteSpec

_ROBOT_DDS_QOS = {
    "/tf": {"qos": "state"},
    "/tf_static": {"qos": "state", "transient_local": True},
    TOPICS.lidar_scan: {"qos": "sensor"},
    TOPICS.imu: {"qos": "sensor"},
    TOPICS.odometry: {"qos": "state"},
    TOPICS.registered_cloud: {"qos": "cloud"},
    TOPICS.map_cloud: {"qos": "cloud"},
    TOPICS.saved_map_cloud: {"qos": "cloud", "required": False},
    TOPICS.localization_health: {"qos": "state"},
    TOPICS.localization_quality: {"qos": "state"},
    TOPICS.nav_command_request: {"qos": "command"},
    TOPICS.nav_command_ack: {"qos": "event"},
    TOPICS.exploration_command: {"qos": "command"},
    TOPICS.exploration_ack: {"qos": "event"},
    TOPICS.inspection_command: {"qos": "command"},
    TOPICS.inspection_ack: {"qos": "event"},
    TOPICS.inspection_status: {"qos": "state"},
    TOPICS.inspection_evidence_request: {"qos": "command"},
    TOPICS.inspection_evidence_result: {"qos": "event"},
    TOPICS.goal_pose: {"qos": "command", "required": False},
    TOPICS.cancel: {"qos": "command", "required": False},
    TOPICS.teleop_cmd_vel: {"qos": "command", "required": False},
    TOPICS.traversability: {"qos": "state", "required": False},
    TOPICS.global_path: {"qos": "state"},
    TOPICS.local_path: {"qos": "state"},
    TOPICS.nav_way_point: {"qos": "state", "required": False},
    TOPICS.cmd_vel: {"qos": "command", "single_writer": True},
}

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

    return RouteSpec(
        name="robot",
        description="Physical robot route. Native service boundaries use typed DDS.",
        default=RouteBackend.LOCAL.value,
        endpoint_contract="thunder_field_dds_v1",
        routes={topic: RouteBackend.DDS.value for topic in _ROBOT_DDS_QOS},
        bindings={
            RouteBackend.DDS.value: _copy_bindings(_ROBOT_DDS_QOS),
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
