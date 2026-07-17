"""LingTu hardware device framework.

Use ``Hw`` as the runtime inventory/status module. ``DeviceManager`` remains a
compatibility alias for older imports.
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
from runtime.devices.manager import DeviceManager, Hw
from runtime.devices.spec import DeviceSpec, load_device_specs

__all__ = [
    "Decoder",
    "Device",
    "DeviceManager",
    "DeviceSpec",
    "DeviceStatus",
    "DeviceType",
    "Hw",
    "decoder_registry",
    "load_device_specs",
    "register_decoder",
]
