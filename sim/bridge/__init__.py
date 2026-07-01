"""Compatibility shim for legacy simulation bridge imports.

Uses a lazy finder so heavy deps such as mujoco and rclpy are only loaded when
someone actually imports ``sim.bridge.<module>``.
"""

import importlib
import importlib.abc
import sys

_REDIRECTS = {
    "sim.bridge.mujoco_ros2_bridge": "compat.ros2.mujoco_ros2_bridge",
    "sim.bridge.mujoco_viz_bridge": "compat.ros2.mujoco_viz_bridge",
    "sim.bridge.nova_nav_bridge": "compat.ros2.nova_nav_bridge",
}


class _RedirectFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def find_module(self, fullname, path=None):
        if fullname in _REDIRECTS:
            return self
        return None

    def load_module(self, fullname):
        if fullname in sys.modules:
            return sys.modules[fullname]
        real = importlib.import_module(_REDIRECTS[fullname])
        sys.modules[fullname] = real
        return real


sys.meta_path.append(_RedirectFinder())
