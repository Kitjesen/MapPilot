"""LiDAR stack: Livox MID-360 stream as an independent Module."""

from __future__ import annotations

import logging

from runtime.blueprint import Blueprint
from runtime.blueprints.stacks._registry import stack_module

logger = logging.getLogger(__name__)


def lidar(
    ip: str | None = None,
    enabled: bool = True,
    start_driver: bool = False,
) -> Blueprint:
    """LiDAR stream stack.

    Args:
        ip: LiDAR IP address override (default from robot_config.yaml).
        enabled: Set to False for stub/dev profiles that don't need hardware.
        start_driver: Compatibility-only flag for starting the legacy local
            livox_ros_driver2 process.

    Returns:
        Blueprint with LidarModule (or empty if disabled).
    """
    bp = Blueprint()

    if not enabled:
        return bp

    try:
        LidarModule = stack_module(
            "lidar",
            "mid360",
            seed_group="lidar",
            fallback="drivers.real.lidar.LidarModule",
        )
    except ImportError as e:
        logger.warning("LiDAR stack: LidarModule not available: %s", e)
        return bp

    kw = {}
    if ip:
        kw["ip"] = ip
    if start_driver:
        kw["start_driver"] = True
    bp.add(LidarModule, alias="LidarModule", **kw)
    logger.info(
        "LiDAR stack: LidarModule added (ip=%s, start_driver=%s)",
        ip or "config default",
        start_driver,
    )

    return bp
