"""lingtu.core — lazily loaded Module orchestration framework facade.

Core components:
- transport — Pluggable transport layer (Transport, LocalTransport, SHM, DDS, Dual)
- msgs      — Unified message types (Vector3, Odometry, SceneGraph, ...)
- stream    — Typed data-flow ports (Out[T], In[T]) and transport abstraction
- module    — Module base class with automatic port scanning
- blueprint — Declarative orchestration blueprint (Blueprint, autoconnect, SystemHandle)
- config    — Unified configuration loader (config/robot_config.yaml)
- clock     — Switchable real-time / simulation clock
Importing a contract submodule such as :mod:`runtime.graph.processes` must not
load Blueprint or construct the Module framework.  Public facade symbols stay
source-compatible and are imported only when requested.
"""

from __future__ import annotations

import importlib
from typing import Any

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


_LAZY_EXPORTS = {
    "Blueprint": (".blueprint", "Blueprint"),
    "SystemHandle": (".blueprint", "SystemHandle"),
    "WorkerSystemHandle": (".blueprint", "WorkerSystemHandle"),
    "autoconnect": (".blueprint", "autoconnect"),
    "Clock": (".clock", "Clock"),
    "clock": (".clock", "clock"),
    "RobotConfig": (".config", "RobotConfig"),
    "get_config": (".config", "get_config"),
    "load_config": (".config", "load_config"),
    "reset_config": (".config", "reset_config"),
    "Buffer": (".tf", "Buffer"),
    "ConnectivityException": (".tf", "ConnectivityException"),
    "ExtrapolationError": (".tf", "ExtrapolationError"),
    "FrameError": (".tf", "FrameError"),
    "FrameTree": (".tf", "FrameTree"),
    "LookupException": (".tf", "LookupException"),
    "NoTransformError": (".tf", "NoTransformError"),
    "StaticTransformBroadcaster": (".tf", "StaticTransformBroadcaster"),
    "TfBus": (".tf", "TfBus"),
    "TransformBroadcaster": (".tf", "TransformBroadcaster"),
    "TransformException": (".tf", "TransformException"),
    "TransformListener": (".tf", "TransformListener"),
    "UnknownFrameError": (".tf", "UnknownFrameError"),
    "Module": (".module", "Module"),
    "SkillInfo": (".module", "SkillInfo"),
    "rpc": (".module", "rpc"),
    "skill": (".module", "skill"),
    "RemoteIn": (".remote_ports", "RemoteIn"),
    "RemoteOut": (".remote_ports", "RemoteOut"),
    "RPCClient": (".rpc_client", "RPCClient"),
    "In": (".stream", "In"),
    "Out": (".stream", "Out"),
    "LocalTransport": (".transport.local", "LocalTransport"),
    "Transport": (".transport.local", "Transport"),
    "ModuleCoordinator": (".coordinator", "ModuleCoordinator"),
    "ResourceMonitor": (".resource_monitor", "ResourceMonitor"),
}

_INTROSPECTION_EXPORTS = frozenset(
    {"render_text", "render_dot", "render_svg", "render_png", "render_connections"}
)


def __getattr__(name: str) -> Any:
    target = _LAZY_EXPORTS.get(name)
    if target is not None:
        module_name, attribute = target
        value = getattr(importlib.import_module(module_name, package=__name__), attribute)
        globals()[name] = value
        return value
    if name in _INTROSPECTION_EXPORTS:
        value = getattr(importlib.import_module(".introspection", package=__name__), name)
        globals()[name] = value
        return value
    raise AttributeError(f"module 'core' has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted({*globals(), *__all__, *_INTROSPECTION_EXPORTS})
