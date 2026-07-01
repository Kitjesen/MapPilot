"""Device base class — common lifecycle for all hardware."""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class DeviceType(str, Enum):
    """Supported hardware device types in the sensor/actuator suite."""
    GNSS = "gnss"
    LIDAR = "lidar"
    CAMERA = "camera"
    IMU = "imu"
    MOTOR_CONTROLLER = "motor_controller"
    CAN_BUS = "can_bus"
    BATTERY = "battery"
    UNKNOWN = "unknown"


class DeviceStatus(str, Enum):
    """Lifecycle states for a hardware device from detection through streaming."""
    OFFLINE = "offline"        # not detected
    DETECTED = "detected"      # found but not opened
    READY = "ready"            # opened, ready to read
    STREAMING = "streaming"    # actively producing data
    ERROR = "error"            # error state, needs retry
    DISABLED = "disabled"      # marked enabled=false


@dataclass
class DeviceHealth:
    """Snapshot of a device's current health."""
    id: str
    type: DeviceType
    status: DeviceStatus
    description: str = ""
    last_data_ts: float = 0.0
    error_count: int = 0
    last_error: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)
    ts: float = field(default_factory=time.time)

    @property
    def age_s(self) -> float:
        return time.time() - self.last_data_ts if self.last_data_ts > 0 else float("inf")

    @property
    def is_healthy(self) -> bool:
        return self.status in (DeviceStatus.READY, DeviceStatus.STREAMING)

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "type": self.type.value,
            "status": self.status.value,
            "description": self.description,
            "last_data_ts": self.last_data_ts,
            "age_s": round(self.age_s, 2) if self.age_s != float("inf") else None,
            "error_count": self.error_count,
            "last_error": self.last_error,
            "is_healthy": self.is_healthy,
            "metadata": self.metadata,
        }


class Device(ABC):
    """Hardware device base class.

    Subclasses implement detect/open/read/close. The DeviceManager
    coordinates lifecycle and publishes health to Dashboard.
    """

    def __init__(self, spec: Any):  # type: DeviceSpec — avoid circular import
        self.spec = spec
        self.id: str = spec.id
        self.type: DeviceType = DeviceType(spec.type)
        self._status = DeviceStatus.DISABLED if not spec.enabled else DeviceStatus.OFFLINE
        self._error_count = 0
        self._last_error = ""
        self._last_data_ts = 0.0

    # -- lifecycle (subclass overrides) ---------------------------------

    @abstractmethod
    def detect(self) -> bool:
        """Check if hardware is physically present (udev, ping, lsusb)."""

    @abstractmethod
    def open(self) -> bool:
        """Open device for reading."""

    @abstractmethod
    def close(self) -> None:
        """Close device, release resources."""

    # -- common helpers -------------------------------------------------

    def mark_data(self) -> None:
        """Record that we received data (advances last_data_ts)."""
        self._last_data_ts = time.time()
        if self._status == DeviceStatus.READY:
            self._status = DeviceStatus.STREAMING

    def mark_error(self, msg: str) -> None:
        self._error_count += 1
        self._last_error = msg
        self._status = DeviceStatus.ERROR

    @property
    def status(self) -> DeviceStatus:
        return self._status

    def health(self) -> DeviceHealth:
        return DeviceHealth(
            id=self.id,
            type=self.type,
            status=self._status,
            description=self.spec.description,
            last_data_ts=self._last_data_ts,
            error_count=self._error_count,
            last_error=self._last_error,
            metadata=self._extra_metadata(),
        )

    def _extra_metadata(self) -> dict[str, Any]:
        """Subclass override to add type-specific metadata to health snapshot."""
        return {}
