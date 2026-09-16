"""lingtu.core — lazily loaded Module orchestration framework facade.

Core components:
- transport — Process-local Module transport (Transport, LocalTransport)
- msgs      — Unified message types (Vector3, Odometry, SceneGraph, ...)
- stream    — Typed data-flow ports (Out[T], In[T]) and transport abstraction
- module    — Module base class with automatic port scanning
- blueprint — Declarative orchestration blueprint (Blueprint, autoconnect, SystemHandle)
- config    — Typed RobotConfig loader
- clock     — Switchable real-time / simulation clock
Importing a contract submodule such as :mod:`lingtu.assembly.graph.processes` must not
load Blueprint or construct the Module framework.  Public facade symbols stay
source-compatible and are imported only when requested.
"""

from __future__ import annotations

import importlib
from typing import Any

__all__ = [
    "Blueprint",
    "Buffer",
    "Clock",
    "ConnectivityException",
    "ExtrapolationError",
    "FrameError",
    "FrameTree",
    "In",
    "LocalTransport",
    "LookupException",
    "Module",
    "NoTransformError",
    "Out",
    "ResourceMonitor",
    "RobotConfig",
    "SkillInfo",
    "StaticTransformBroadcaster",
    "SystemHandle",
    "TfBus",
    "TransformBroadcaster",
    "TransformException",
    "TransformListener",
    "Transport",
    "UnknownFrameError",
    "autoconnect",
    "clock",
    "get_config",
    "load_config",
    "reset_config",
    "rpc",
    "skill",
]


_LAZY_EXPORTS = {
    "Blueprint": (".blueprint", "Blueprint"),
    "SystemHandle": (".blueprint", "SystemHandle"),
    "autoconnect": (".blueprint", "autoconnect"),
    "Clock": (".clock", "Clock"),
    "clock": (".clock", "clock"),
    "RobotConfig": (".config", "RobotConfig"),
    "get_config": (".config", "get_config"),
    "load_config": (".config", "load_config"),
    "reset_config": (".config", "reset_config"),
    "Buffer": (".tf.buffer", "Buffer"),
    "ConnectivityException": (".tf.tree", "ConnectivityException"),
    "ExtrapolationError": (".tf.tree", "ExtrapolationError"),
    "FrameError": (".tf.tree", "FrameError"),
    "FrameTree": (".tf.tree", "FrameTree"),
    "LookupException": (".tf.tree", "LookupException"),
    "NoTransformError": (".tf.tree", "NoTransformError"),
    "StaticTransformBroadcaster": (".tf.buffer", "StaticTransformBroadcaster"),
    "TfBus": (".tf.buffer", "TfBus"),
    "TransformBroadcaster": (".tf.buffer", "TransformBroadcaster"),
    "TransformException": (".tf.tree", "TransformException"),
    "TransformListener": (".tf.buffer", "TransformListener"),
    "UnknownFrameError": (".tf.tree", "UnknownFrameError"),
    "Module": (".module", "Module"),
    "SkillInfo": (".module", "SkillInfo"),
    "rpc": (".module", "rpc"),
    "skill": (".module", "skill"),
    "In": (".stream", "In"),
    "Out": (".stream", "Out"),
    "LocalTransport": (".transport.local", "LocalTransport"),
    "Transport": (".transport.local", "Transport"),
    "ResourceMonitor": (".resource_monitor", "ResourceMonitor"),
}

def __getattr__(name: str) -> Any:
    target = _LAZY_EXPORTS.get(name)
    if target is not None:
        module_name, attribute = target
        value = getattr(importlib.import_module(module_name, package=__name__), attribute)
        globals()[name] = value
        return value
    raise AttributeError(f"module 'core' has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted({*globals(), *__all__})
