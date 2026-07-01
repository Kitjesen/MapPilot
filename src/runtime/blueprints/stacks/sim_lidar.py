"""Simulated LiDAR stack: generate PointCloud2 from MuJoCo XML scene geometry.

Replaces real LiDAR + SLAM for pure-Python simulation.  The output
``map_cloud`` is identical to the canonical SLAM map-cloud stream, so
OccupancyGridModule and the rest of the navigation pipeline work unchanged.
"""

from __future__ import annotations

import logging

from runtime.blueprint import Blueprint
from runtime.blueprints.stacks._registry import stack_module

logger = logging.getLogger(__name__)


def sim_lidar(scene_xml: str = "", **config) -> Blueprint:
    """Simulated LiDAR from MuJoCo XML scene.

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
