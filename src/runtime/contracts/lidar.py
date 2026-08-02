"""Generic LiDAR contract."""

from __future__ import annotations

from dataclasses import dataclass

LIDAR_ROLE = "lidar"
LIDAR_ALIAS = LIDAR_ROLE
LIDAR_CONFIG_ENABLE = "enable_lidar"
LIDAR_CONFIG_BACKEND = "lidar_backend"

LIDAR_BACKEND_MID360 = "mid360"
LIDAR_BACKEND_MUJOCO = "mujoco"
LIDAR_BACKEND_DDS = "dds"
LIDAR_BACKEND_REPLAY = "replay"
LIDAR_BACKENDS = (
    LIDAR_BACKEND_MID360,
    LIDAR_BACKEND_MUJOCO,
    LIDAR_BACKEND_DDS,
    LIDAR_BACKEND_REPLAY,
)

LIDAR_PORTS = ("scan", "raw_scan", "imu", "alive")
LIDAR_STREAM_PORTS = ("scan", "raw_scan", "imu")
LIDAR_HEALTH_FIELDS = (
    "status",
    "backend",
    "source",
    "frames",
    "points",
    "stale_ms",
    "error",
)


@dataclass(frozen=True)
class LidarContract:
    role: str = LIDAR_ROLE
    alias: str = LIDAR_ALIAS
    config_keys: tuple[str, ...] = (LIDAR_CONFIG_ENABLE, LIDAR_CONFIG_BACKEND)
    backends: tuple[str, ...] = LIDAR_BACKENDS
    ports: tuple[str, ...] = LIDAR_PORTS
    stream_ports: tuple[str, ...] = LIDAR_STREAM_PORTS
    health_fields: tuple[str, ...] = LIDAR_HEALTH_FIELDS

    def to_dict(self) -> dict[str, object]:
        return {
            "role": self.role,
            "alias": self.alias,
            "config_keys": list(self.config_keys),
            "backends": list(self.backends),
            "ports": list(self.ports),
            "stream_ports": list(self.stream_ports),
            "health_fields": list(self.health_fields),
        }


LIDAR_CONTRACT = LidarContract()
