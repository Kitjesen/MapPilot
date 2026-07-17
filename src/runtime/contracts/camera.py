"""Generic camera contract.

This file defines the runtime-facing camera boundary. Hardware details such as
Orbbec stay below this contract.
"""

from __future__ import annotations

from dataclasses import dataclass

CAMERA_ROLE = "camera"
CAMERA_ALIAS = CAMERA_ROLE
CAMERA_COMPAT_ALIAS = "CameraBridgeModule"
CAMERA_CONFIG_FORCE = "force_camera"
CAMERA_COMPAT_CONFIG_FORCE = "force_camera_bridge"

CAMERA_BACKEND_ORBBEC = "orbbec"
CAMERA_BACKEND_REPLAY = "replay"
CAMERA_BACKEND_SIM = "sim"
CAMERA_BACKEND_DDS = "dds"
CAMERA_BACKENDS = (
    CAMERA_BACKEND_ORBBEC,
    CAMERA_BACKEND_REPLAY,
    CAMERA_BACKEND_SIM,
    CAMERA_BACKEND_DDS,
)

CAMERA_PORTS = ("color_image", "depth_image", "camera_info", "alive")
CAMERA_STREAM_PORTS = ("color_image", "depth_image", "camera_info")
CAMERA_HEALTH_FIELDS = (
    "status",
    "backend",
    "fps",
    "frames",
    "stale_ms",
    "device",
    "error",
)


@dataclass(frozen=True)
class CameraContract:
    role: str = CAMERA_ROLE
    alias: str = CAMERA_ALIAS
    compat_aliases: tuple[str, ...] = (CAMERA_COMPAT_ALIAS,)
    config_keys: tuple[str, ...] = (CAMERA_CONFIG_FORCE,)
    compat_config_keys: tuple[str, ...] = (CAMERA_COMPAT_CONFIG_FORCE,)
    backends: tuple[str, ...] = CAMERA_BACKENDS
    ports: tuple[str, ...] = CAMERA_PORTS
    stream_ports: tuple[str, ...] = CAMERA_STREAM_PORTS
    health_fields: tuple[str, ...] = CAMERA_HEALTH_FIELDS

    def to_dict(self) -> dict[str, object]:
        return {
            "role": self.role,
            "alias": self.alias,
            "compat_aliases": list(self.compat_aliases),
            "config_keys": list(self.config_keys),
            "compat_config_keys": list(self.compat_config_keys),
            "backends": list(self.backends),
            "ports": list(self.ports),
            "stream_ports": list(self.stream_ports),
            "health_fields": list(self.health_fields),
        }


CAMERA_CONTRACT = CameraContract()
