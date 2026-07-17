"""Generic GNSS contract."""

from __future__ import annotations

from dataclasses import dataclass

GNSS_ROLE = "gnss"
GNSS_ALIAS = GNSS_ROLE
GNSS_COMPAT_ALIAS = "GnssModule"
GNSS_CONFIG_ENABLE = "enable_gnss"
GNSS_CONFIG_BACKEND = "gnss_backend"

GNSS_BACKEND_WTRTK980 = "wtrtk980"
GNSS_BACKEND_HW = "hw"
GNSS_BACKEND_DDS = "dds"
GNSS_BACKEND_REPLAY = "replay"
GNSS_BACKENDS = (
    GNSS_BACKEND_WTRTK980,
    GNSS_BACKEND_HW,
    GNSS_BACKEND_DDS,
    GNSS_BACKEND_REPLAY,
)

GNSS_INPUT_PORTS = ("rtcm_bytes",)
GNSS_OUTPUT_PORTS = ("gnss_fix", "gnss_status", "gnss_odom", "alive")
GNSS_PORTS = GNSS_INPUT_PORTS + GNSS_OUTPUT_PORTS
GNSS_STREAM_PORTS = ("gnss_fix", "gnss_status", "gnss_odom")
GNSS_HEALTH_FIELDS = (
    "role",
    "device_model",
    "backend",
    "source",
    "dataflow_owner",
    "product_owner",
    "python_compat",
    "serial_port",
    "uses_hw_inventory",
    "requires_hw_bridge",
    "dds_compat_reader",
    "replay_source",
    "direct_serial",
    "link_ok",
    "origin_initialised",
    "stale_ms",
    "error",
)


@dataclass(frozen=True)
class GnssContract:
    role: str = GNSS_ROLE
    alias: str = GNSS_ALIAS
    compat_aliases: tuple[str, ...] = (GNSS_COMPAT_ALIAS,)
    config_keys: tuple[str, ...] = (GNSS_CONFIG_ENABLE, GNSS_CONFIG_BACKEND)
    compat_config_keys: tuple[str, ...] = ()
    backends: tuple[str, ...] = GNSS_BACKENDS
    input_ports: tuple[str, ...] = GNSS_INPUT_PORTS
    output_ports: tuple[str, ...] = GNSS_OUTPUT_PORTS
    ports: tuple[str, ...] = GNSS_PORTS
    stream_ports: tuple[str, ...] = GNSS_STREAM_PORTS
    health_fields: tuple[str, ...] = GNSS_HEALTH_FIELDS

    def to_dict(self) -> dict[str, object]:
        return {
            "role": self.role,
            "alias": self.alias,
            "compat_aliases": list(self.compat_aliases),
            "config_keys": list(self.config_keys),
            "compat_config_keys": list(self.compat_config_keys),
            "backends": list(self.backends),
            "input_ports": list(self.input_ports),
            "output_ports": list(self.output_ports),
            "ports": list(self.ports),
            "stream_ports": list(self.stream_ports),
            "health_fields": list(self.health_fields),
        }


GNSS_CONTRACT = GnssContract()
