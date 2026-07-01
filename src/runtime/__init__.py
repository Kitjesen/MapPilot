"""lingtu.core — Module orchestration framework core, built directly in src/.

Core components:
- transport — Pluggable transport layer (Transport, LocalTransport, SHM, DDS, Dual)
- msgs      — Unified message types (Vector3, Odometry, SceneGraph, ...)
- stream    — Typed data-flow ports (Out[T], In[T]) and transport abstraction
- module    — Module base class with automatic port scanning
- blueprint — Declarative orchestration blueprint (Blueprint, autoconnect, SystemHandle)
- config    — Unified configuration loader (config/robot_config.yaml)
- clock     — Switchable real-time / simulation clock
"""

from .blueprint import Blueprint, SystemHandle, WorkerSystemHandle, autoconnect
from .clock import Clock, clock
from .config import RobotConfig, get_config, load_config, reset_config
from .tf import (
    Buffer,
    ConnectivityException,
    ExtrapolationError,
    FrameError,
    FrameTree,
    LookupException,
    NoTransformError,
    StaticTransformBroadcaster,
    TfBus,
    TransformBroadcaster,
    TransformException,
    TransformListener,
    UnknownFrameError,
)
from .module import Module, SkillInfo, rpc, skill
from .remote_ports import RemoteIn, RemoteOut
from .rpc_client import RPCClient
from .stream import In, Out
from .transport.local import LocalTransport, Transport

__all__ = [
    # blueprint
    "Blueprint",
    "Buffer",
    "ConnectivityException",
    # clock
    "Clock",
    # frames
    "ExtrapolationError",
    "FrameError",
    "FrameTree",
    "In",
    "LocalTransport",
    "LookupException",
    # module
    "Module",
    "NoTransformError",
    "StaticTransformBroadcaster",
    # coordinator (imported lazily — requires WorkerManager)
    "ModuleCoordinator",
    # stream
    "Out",
    # rpc / remote
    "RPCClient",
    "RemoteIn",
    "RemoteOut",
    # resource monitor (imported lazily)
    "ResourceMonitor",
    # config
    "RobotConfig",
    "SkillInfo",
    "SystemHandle",
    # transport
    "Transport",
    "TransformException",
    "TfBus",
    "TransformBroadcaster",
    "TransformListener",
    "UnknownFrameError",
    "WorkerSystemHandle",
    "autoconnect",
    "clock",
    "get_config",
    "load_config",
    "render_dot",
    "render_png",
    "render_svg",
    # introspection (imported lazily)
    "render_text",
    "reset_config",
    "rpc",
    "skill",
]


def __getattr__(name: str):
    if name == "ModuleCoordinator":
        from .coordinator import ModuleCoordinator  # type: ignore[import]
        return ModuleCoordinator
    if name in ("render_text", "render_dot", "render_svg", "render_png",
                "render_connections"):
        import importlib
        mod = importlib.import_module(".introspection", package=__name__)
        return getattr(mod, name)
    if name == "ResourceMonitor":
        from .resource_monitor import ResourceMonitor
        return ResourceMonitor
    raise AttributeError(f"module 'core' has no attribute {name!r}")
