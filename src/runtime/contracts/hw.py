"""Generic hardware inventory contract."""

from __future__ import annotations

from dataclasses import dataclass

HW_ROLE = "hw"
HW_ALIAS = HW_ROLE
HW_COMPAT_ALIAS = "DeviceManager"
HW_CONFIG_ENABLE = "enable_hw"
HW_COMPAT_CONFIG_ENABLE = "enable_device_manager"
HW_CONFIG_BRIDGE = "hw_bridge"
HW_COMPAT_CONFIG_BRIDGE = "device_manager_bridge"

HW_PORTS = ("device_status", "device_event", "alive")
HW_HEALTH_FIELDS = ("devices", "spec_count", "opened_count")


@dataclass(frozen=True)
class HwContract:
    role: str = HW_ROLE
    alias: str = HW_ALIAS
    compat_aliases: tuple[str, ...] = (HW_COMPAT_ALIAS,)
    config_keys: tuple[str, ...] = (HW_CONFIG_ENABLE, HW_CONFIG_BRIDGE)
    compat_config_keys: tuple[str, ...] = (
        HW_COMPAT_CONFIG_ENABLE,
        HW_COMPAT_CONFIG_BRIDGE,
    )
    ports: tuple[str, ...] = HW_PORTS
    health_fields: tuple[str, ...] = HW_HEALTH_FIELDS

    def to_dict(self) -> dict[str, object]:
        return {
            "role": self.role,
            "alias": self.alias,
            "compat_aliases": list(self.compat_aliases),
            "config_keys": list(self.config_keys),
            "compat_config_keys": list(self.compat_config_keys),
            "ports": list(self.ports),
            "health_fields": list(self.health_fields),
        }


HW_CONTRACT = HwContract()
