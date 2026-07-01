"""Colcon install-layout helpers for legacy ROS2 compatibility adapters.

This module is not a product runtime surface. Product planning and autonomy
code should use in-process kernels (nanobind/Python) and normal Module ports.
Keep this helper limited to explicit ``runtime.adapters.ros2`` adapters that still need
to locate an installed ROS2 executable or share file.

Usage::

    from runtime.native_install import exe, share, DDS_ENV
    from runtime.config import get_config

    cfg = get_config()
    binary = exe(cfg, "livox_ros_driver2", "livox_ros_driver2_node")
    config = share(cfg, "livox_ros_driver2", "config", "MID360_config.json")
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .config import RobotConfig

# Default install prefix: <lingtu_repo>/install
# (the colcon build output lives next to src/ inside the repo root)
# Overridable via cfg.raw["nav_install"] or env var LINGTU_NAV_INSTALL.
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
_DEFAULT_PREFIX = os.environ.get(
    "LINGTU_NAV_INSTALL",
    str(_REPO_ROOT / "install"),
)

def _build_ld_library_path(prefix: str) -> str:
    """Collect all install/*/lib dirs + CycloneDDS, append to current LD_LIBRARY_PATH."""
    lib_dirs: list[str] = []

    # Scan every package under the colcon install prefix
    prefix_path = Path(prefix)
    if prefix_path.exists():
        for pkg in prefix_path.iterdir():
            lib = pkg / "lib"
            if lib.is_dir():
                lib_dirs.append(str(lib))

    # CycloneDDS built from source (Unitree / S100P convention)
    cyclone_lib = Path.home() / "cyclonedds" / "install" / "lib"
    if cyclone_lib.exists():
        lib_dirs.append(str(cyclone_lib))

    existing = os.environ.get("LD_LIBRARY_PATH", "")
    all_dirs = lib_dirs + ([existing] if existing else [])
    return ":".join(all_dirs)


DDS_ENV: dict[str, str] = {
    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
    "LD_LIBRARY_PATH": _build_ld_library_path(_DEFAULT_PREFIX),
}


def _install_prefix(cfg: RobotConfig) -> str:
    return cfg.raw.get("nav_install", _DEFAULT_PREFIX)


def exe(cfg: RobotConfig, package: str, binary: str) -> str:
    """Return the absolute path to a compiled binary.

    Tries colcon full layout first (prefix/package/lib/package/binary),
    then falls back to flat layout (prefix/lib/package/binary).
    """
    prefix = _install_prefix(cfg)
    colcon = os.path.join(prefix, package, "lib", package, binary)
    if os.path.exists(colcon):
        return colcon
    return os.path.join(prefix, "lib", package, binary)


def share(cfg: RobotConfig, package: str, *sub: str) -> str:
    """Return the absolute path to a share file.

    Tries colcon full layout first (prefix/package/share/package/...),
    then falls back to flat layout (prefix/share/package/...).
    """
    prefix = _install_prefix(cfg)
    colcon = os.path.join(prefix, package, "share", package, *sub)
    if os.path.exists(colcon):
        return colcon
    return os.path.join(prefix, "share", package, *sub)
