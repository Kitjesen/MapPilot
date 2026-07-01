"""Adapter helpers for perception and gateway ROS-compatible boundaries."""

from __future__ import annotations

from importlib import import_module

from runtime.blueprints.stacks._registry import optional_stack_module, stack_module
from runtime.plugin_seed import seed_registered_plugins
from runtime.registry import get


def _is_ros2_module(cls: type) -> bool:
    module = str(getattr(cls, "__module__", ""))
    return ".adapters.ros2." in module or module == "gateway.visualization.rerun_bridge"


def camera_bridge_module(*, enable_ros2: bool = False) -> type | None:
    """Resolve the optional camera bridge adapter.

    Default stack resolution does not import ROS2 compatibility modules. Pass
    ``enable_ros2=True`` only for explicit ROS2 runtime profiles.
    """

    if enable_ros2:
        try:
            seed_registered_plugins(groups=("camera_ros2",), reload_loaded=False)
        except ValueError as exc:
            if "camera_ros2" not in str(exc):
                raise
            return None
        try:
            return get("camera_bridge", "default")
        except KeyError:
            return None

    try:
        registered = get("camera_bridge", "default")
        if _is_ros2_module(registered):
            return None
        return registered
    except KeyError:
        pass

    seed_registered_plugins(groups=("camera",), reload_loaded=False)
    try:
        registered = get("camera_bridge", "default")
        if _is_ros2_module(registered):
            return None
        return registered
    except KeyError:
        return None


def rerun_bridge_module(*, enable_ros2: bool = False) -> type | None:
    """Resolve the optional Rerun visualization adapter."""

    if enable_ros2:
        return optional_stack_module(
            "visualization",
            "rerun",
            seed_group="visualization_ros2",
            fallback="gateway.visualization.rerun_bridge.RerunBridgeModule",
        )

    def _non_ros2_registered() -> type | None:
        try:
            registered = get("visualization", "rerun")
        except KeyError:
            return None
        if _is_ros2_module(registered):
            return None
        return registered

    registered = _non_ros2_registered()
    if registered is not None:
        return registered

    seed_registered_plugins(groups=("visualization",), reload_loaded=False)
    registered = _non_ros2_registered()
    if registered is not None:
        return registered

    if not enable_ros2:
        try:
            module = import_module("runtime.rerun_module")
            return getattr(module, "RerunModule")
        except (ImportError, AttributeError):
            return None
