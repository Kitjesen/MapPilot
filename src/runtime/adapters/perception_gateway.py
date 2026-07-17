"""Adapter helpers for perception and gateway ROS-compatible boundaries."""

from __future__ import annotations

import importlib
import sys
from importlib import import_module

from runtime.blueprints.stacks._registry import optional_stack_module, stack_module
from runtime.contracts import CAMERA_BACKEND_ORBBEC, CAMERA_BACKEND_SIM, CAMERA_ROLE
from runtime.plugin_seed import seed_registered_plugins
from runtime.registry import get

# Candidate modules whose @register decorators populate the camera registry.
# Order: sim first because it is the most commonly needed fallback in tests
# after registry.clear(); real hardware backends follow.
_CAMERA_CANDIDATE_MODULES = (
    "drivers.sim.camera.module",
    "drivers.real.camera.module",
    "drivers.real.camera.dds_module",
)


def _is_ros2_module(cls: type) -> bool:
    module = str(getattr(cls, "__module__", ""))
    return ".adapters.ros2." in module or module == "gateway.visualization.rerun_bridge"


def _reload_camera_candidates() -> None:
    """Re-run @register decorators of camera candidate modules.

    This is a resilience path for tests that call ``registry.clear()`` without
    restoring camera registrations. In production the modules are imported
    once at startup and stay registered; this helper only re-imports modules
    that are already present in ``sys.modules``.
    """

    for module_name in _CAMERA_CANDIDATE_MODULES:
        mod = sys.modules.get(module_name)
        if mod is not None:
            try:
                importlib.reload(mod)
            except Exception:
                # Best-effort: a broken candidate must not block other backends.
                pass


def camera_module(*, enable_ros2: bool = False, backend: str = CAMERA_BACKEND_ORBBEC) -> type | None:
    """Resolve the optional camera adapter.

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

    registered = _camera_module(backend)
    if registered is not None:
        return registered

    if backend == CAMERA_BACKEND_ORBBEC:
        try:
            registered = get("camera_bridge", "default")
            if _is_ros2_module(registered):
                return None
            return registered
        except KeyError:
            pass

    group = "camera_sim" if backend == CAMERA_BACKEND_SIM else "camera"
    seed_registered_plugins(groups=(group,), reload_loaded=False)
    registered = _camera_module(backend)
    if registered is not None:
        return registered

    # Last-resort: re-run @register decorators of already-imported camera
    # candidate modules. This recovers from test-side registry clears.
    _reload_camera_candidates()
    registered = _camera_module(backend)
    if registered is not None:
        return registered

    if backend == CAMERA_BACKEND_ORBBEC:
        try:
            registered = get("camera_bridge", "default")
            if _is_ros2_module(registered):
                return None
            return registered
        except KeyError:
            pass
    return None


camera_bridge_module = camera_module


def _camera_module(backend: str) -> type | None:
    try:
        registered = get(CAMERA_ROLE, backend)
    except KeyError:
        return None
    if _is_ros2_module(registered):
        return None
    return registered


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
            return module.RerunModule
        except (ImportError, AttributeError):
            return None
