"""LiDAR stack: Livox MID-360 stream as an independent Module."""

from __future__ import annotations

import logging

from runtime.blueprint import Blueprint
from runtime.blueprints.stacks._registry import stack_module
from runtime.contracts import LIDAR_BACKEND_MID360, LIDAR_BACKEND_MUJOCO, LIDAR_BACKENDS

logger = logging.getLogger(__name__)


def lidar(
    ip: str | None = None,
    enabled: bool = True,
    start_driver: bool = False,
    backend: str = "mid360",
) -> Blueprint:
    """LiDAR stream stack.

    Args:
        ip: LiDAR IP address override (default from robot_config.yaml).
        enabled: Set to False for stub/dev profiles that don't need hardware.
        start_driver: Compatibility-only flag for starting the legacy local
            livox_ros_driver2 process.
        backend: Registered LiDAR backend. Use ``mid360`` for real Livox and
            ``mujoco`` for same-process simulation.

    Returns:
        Blueprint with the canonical `lidar` role (or empty if disabled).
    """
    bp = Blueprint()

    if not enabled:
        return bp

    backend = (backend or LIDAR_BACKEND_MID360).strip()
    if backend not in LIDAR_BACKENDS:
        raise ValueError(f"Unsupported lidar backend: {backend!r}; expected one of {LIDAR_BACKENDS}")
    if backend not in {LIDAR_BACKEND_MID360, LIDAR_BACKEND_MUJOCO}:
        raise ValueError(f"LiDAR backend {backend!r} is declared but not implemented by this stack")
    try:
        LidarModule = stack_module(
            "lidar",
            backend,
            seed_group="lidar",
            fallback=(
                "drivers.sim.lidar.MujocoLidarModule"
                if backend == LIDAR_BACKEND_MUJOCO
                else "drivers.real.lidar.LidarModule"
            ),
        )
    except ImportError as e:
        logger.warning("LiDAR stack: lidar backend not available: %s", e)
        return bp

    kw = {}
    if ip:
        kw["ip"] = ip
    if start_driver and backend == "mid360":
        kw["start_driver"] = True
    bp.add(LidarModule, alias="lidar", **kw)
    logger.info(
        "LiDAR stack: lidar added (backend=%s, ip=%s, start_driver=%s)",
        backend,
        ip or "config default",
        start_driver,
    )

    return bp
