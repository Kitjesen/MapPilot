"""Generic IMU contract."""

from __future__ import annotations

from dataclasses import dataclass

IMU_ROLE = "imu"
IMU_ALIAS = IMU_ROLE
IMU_CONFIG_ENABLE = "enable_imu"
IMU_CONFIG_BACKEND = "imu_backend"

IMU_BACKEND_LIVOX = "livox"
IMU_BACKEND_MUJOCO = "mujoco"
IMU_BACKEND_DDS = "dds"
IMU_BACKEND_REPLAY = "replay"
IMU_BACKENDS = (
    IMU_BACKEND_LIVOX,
    IMU_BACKEND_MUJOCO,
    IMU_BACKEND_DDS,
    IMU_BACKEND_REPLAY,
)

IMU_PORTS = ("imu", "alive")
IMU_STREAM_PORTS = ("imu",)
IMU_HEALTH_FIELDS = (
    "status",
    "backend",
    "source",
    "samples",
    "stale_ms",
    "error",
)


@dataclass(frozen=True)
class ImuContract:
    role: str = IMU_ROLE
    alias: str = IMU_ALIAS
    compat_aliases: tuple[str, ...] = ()
    config_keys: tuple[str, ...] = (IMU_CONFIG_ENABLE, IMU_CONFIG_BACKEND)
    compat_config_keys: tuple[str, ...] = ()
    backends: tuple[str, ...] = IMU_BACKENDS
    ports: tuple[str, ...] = IMU_PORTS
    stream_ports: tuple[str, ...] = IMU_STREAM_PORTS
    health_fields: tuple[str, ...] = IMU_HEALTH_FIELDS

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


IMU_CONTRACT = ImuContract()
