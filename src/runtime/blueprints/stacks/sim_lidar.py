"""Legacy simulated map-cloud stack for pure-Python navigation tests.

New MuJoCo sensor profiles should use the canonical ``lidar`` role with backend
``mujoco``. This stack remains for ``sim_nav`` and older pure-Python tests that
need a direct map-cloud provider without running a LiDAR or SLAM role.
"""

from __future__ import annotations

import logging

from runtime.blueprint import Blueprint
from runtime.blueprints.stacks._registry import stack_module

logger = logging.getLogger(__name__)


def sim_lidar(scene_xml: str = "", **config) -> Blueprint:
    """Legacy simulated map-cloud provider from MuJoCo XML scene.

    Args:
        scene_xml: Path to MuJoCo XML world file.

    Returns:
        Blueprint with SimPointCloudProvider (or empty if no scene).
    """
    bp = Blueprint()

    if not scene_xml:
        return bp

    SimPointCloudProvider = stack_module(
        "sim_lidar",
        "pointcloud",
        seed_group="sim_lidar",
        fallback="drivers.sim.pointcloud.SimPointCloudProvider",
    )
    bp.add(
        SimPointCloudProvider,
        alias="SimPointCloudProvider",
        scene_xml=scene_xml,
        **config,
    )
    logger.info("sim_lidar stack: SimPointCloudProvider(%s)", scene_xml)

    return bp
