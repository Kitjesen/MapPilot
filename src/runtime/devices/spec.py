"""DeviceSpec — load and validate config/devices.yaml entries."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class UdevMatch:
    """udev attribute match for auto-discovery."""
    vendor_id: str | None = None
    product_id: str | None = None
    port_path: str | None = None
    serial: str | None = None


@dataclass
class TopicSpec:
    """A topic this device publishes."""
    name: str
    type: str           # GnssFix / PointCloud2 / Image / Imu / ...
    rate_hz: float = 10.0


@dataclass
class DeviceSpec:
    """One entry from config/devices.yaml — pure data, no behaviour."""
    id: str
    type: str
    driver: str
    description: str = ""
    enabled: bool = True
    udev_match: UdevMatch | None = None
    serial: dict[str, Any] = field(default_factory=dict)
    network: dict[str, Any] = field(default_factory=dict)
    config: dict[str, Any] = field(default_factory=dict)
    capabilities: list[str] = field(default_factory=list)
    publish_topics: list[TopicSpec] = field(default_factory=list)
    note: str = ""

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> DeviceSpec:
        udev = d.get("udev_match")
        if udev:
            udev = UdevMatch(
                vendor_id=udev.get("vendor_id"),
                product_id=udev.get("product_id"),
                port_path=udev.get("port_path"),
                serial=udev.get("serial"),
            )
        topics = [TopicSpec(**t) for t in d.get("publish_topics", [])]
        return cls(
            id=d["id"],
            type=d["type"],
            driver=d["driver"],
            description=d.get("description", ""),
            enabled=d.get("enabled", True),
            udev_match=udev,
            serial=d.get("serial", {}),
            network=d.get("network", {}),
            config=d.get("config", {}),
            capabilities=d.get("capabilities", []),
            publish_topics=topics,
            note=d.get("note", ""),
        )


def load_device_specs(path: str | Path) -> list[DeviceSpec]:
    """Load and parse devices.yaml — returns list of DeviceSpec."""
    import yaml
    p = Path(path)
    if not p.exists():
        return []
    with open(p, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return [DeviceSpec.from_dict(d) for d in data.get("devices", [])]
