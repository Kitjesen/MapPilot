"""Adapter helpers for perception and gateway ROS-compatible boundaries."""

from __future__ import annotations

import importlib
import logging
import sys
from importlib import import_module

from runtime.contracts import CAMERA_BACKEND_ORBBEC, CAMERA_BACKEND_SIM, CAMERA_ROLE
from runtime.plugin_resolution import optional_stack_module
from runtime.plugin_seed import seed_registered_plugins
from runtime.registry import get

logger = logging.getLogger(__name__)

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


def _load_camera_candidates() -> None:
    """Load camera candidates or restore their registry decorators.

    Product assembly can run without the CLI bootstrap that installs plugin
    catalogs. Import candidates directly in that case; reload only modules
    already present after a test-side registry reset.
    """

    for module_name in _CAMERA_CANDIDATE_MODULES:
        try:
            mod = sys.modules.get(module_name)
            if mod is None:
                import_module(module_name)
            else:
                importlib.reload(mod)
        except Exception:
            # Best-effort: one unavailable backend must not block the others.
            logger.debug("Camera plugin candidate is unavailable: %s", module_name, exc_info=True)


def camera_module(*, backend: str = CAMERA_BACKEND_ORBBEC) -> type | None:
    """Resolve a camera backend through the canonical camera role."""

    registered = _camera_module(backend)
    if registered is not None:
        return registered

    group = "camera_sim" if backend == CAMERA_BACKEND_SIM else "camera"
    seed_registered_plugins(groups=(group,), reload_loaded=False)
    registered = _camera_module(backend)
    if registered is not None:
        return registered

    # Last-resort: load candidates or re-run their @register decorators after
    # a test-side registry clear.
    _load_camera_candidates()
    registered = _camera_module(backend)
    if registered is not None:
        return registered
    return None


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
