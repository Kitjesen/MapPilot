"""LingTu Device Registry — unified hardware device framework.

Three-layer design:

    Device      — hardware abstraction (lifecycle: detect/open/read/close)
    Decoder     — protocol parser (NMEA / Livox custom / ROS2 NavSatFix / ...)
    DeviceManager — orchestrator (loads devices.yaml, hot-plug, status)

Usage::

    from runtime.devices import DeviceManager
    mgr = DeviceManager(config_path="config/devices.yaml")
    bp.add(mgr)
"""

from runtime.devices.base import (
    Device,
    DeviceStatus,
    DeviceType,
)
from runtime.devices.decoder import (
    Decoder,
    decoder_registry,
    register_decoder,
)
from runtime.devices.manager import DeviceManager
from runtime.devices.spec import DeviceSpec, load_device_specs

__all__ = [
    "Decoder",
    "Device",
    "DeviceManager",
    "DeviceSpec",
    "DeviceStatus",
    "DeviceType",
    "decoder_registry",
    "load_device_specs",
    "register_decoder",
]
