"""Unified transport layer for LingTu navigation system.

Module code talks to the simple Transport protocol. Concrete backends are
loaded lazily so optional IPC dependencies do not affect lightweight profiles.
"""

from .abc import (
    Publisher,
    Subscriber,
    TopicConfig,
    TransportABC,
    TransportStrategy,
)
from .adapter import TransportAdapter
from .factory import create_publisher, create_subscriber, create_transport
from .local import LocalTransport, Transport

_OPTIONAL_BACKENDS = {
    "SHMPublisher": ("shm", "SHMPublisher"),
    "SHMSubscriber": ("shm", "SHMSubscriber"),
    "SHMTransport": ("shm", "SHMTransport"),
    "LCMPublisher": ("lcm", "LCMPublisher"),
    "LCMSubscriber": ("lcm", "LCMSubscriber"),
    "LCMTransport": ("lcm", "LCMTransport"),
    "DDSPublisher": ("dds", "DDSPublisher"),
    "DDSSubscriber": ("dds", "DDSSubscriber"),
    "DDSTransport": ("dds", "DDSTransport"),
    "DualPublisher": ("dual", "DualPublisher"),
    "DualSubscriber": ("dual", "DualSubscriber"),
    "DualTransport": ("dual", "DualTransport"),
}


def __getattr__(name: str):
    if name not in _OPTIONAL_BACKENDS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, attr_name = _OPTIONAL_BACKENDS[name]
    import importlib

    module = importlib.import_module(f".{module_name}", package=__name__)
    value = getattr(module, attr_name)
    globals()[name] = value
    return value


__all__ = [
    "LocalTransport",
    "Publisher",
    "Subscriber",
    "TopicConfig",
    "Transport",
    "TransportABC",
    "TransportAdapter",
    "TransportStrategy",
    "create_publisher",
    "create_subscriber",
    "create_transport",
]
