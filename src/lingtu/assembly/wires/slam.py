"""SLAM observation, map-frame, localization-health, and odometry wires."""

from __future__ import annotations

from runtime.runtime_interface import TOPICS
from runtime.wiring import WireSpec

from .context import (
    MAP_OBSERVATION_CONSUMERS,
    ODOMETRY_CONSUMERS,
    TOPIC_SLAM_LOCALIZATION_HEALTH,
    TOPIC_SLAM_LOCALIZATION_QUALITY,
    TOPIC_SLAM_ODOMETRY,
    WiringContext,
)

LOCALIZATION_STATUS_CONSUMERS = (
    "GatewayModule",
)

GNSS_FUSION_HEALTH_CONSUMERS = (
    "GatewayModule",
)

LIDAR_ROLE = "lidar"


def map_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    specs: list[WireSpec] = []
    if ctx.slam_module:
        specs.extend(
            WireSpec(
                ctx.slam_module,
                "map_observation",
                consumer,
                "map_observation",
                topic=TOPICS.map_observation,
            )
            for consumer in MAP_OBSERVATION_CONSUMERS
            if consumer in ctx.names
        )
    elif ctx.driver_module in {
        "MujocoDriverModule",
        "SimEndpointDriverModule",
    }:
        if ctx.driver_module == "MujocoDriverModule":
            specs.extend(
                WireSpec(ctx.driver_module, "map_observation", consumer, "map_observation")
                for consumer in MAP_OBSERVATION_CONSUMERS
                if consumer in ctx.names
            )
    return tuple(specs)


def scan_view_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    """Wire the Gateway live scan overlay to the raw LiDAR role when present.

    Real-env Product Hosts do not instantiate a Python LiDAR owner. In that case the
    native SLAM status adapter can expose the same scan as a file snapshot from
    the C++ runtime without using cyclonedds-python in the robot process.
    """
    if "GatewayModule" not in ctx.names:
        return ()
    source = ""
    source_port = "scan"
    if LIDAR_ROLE in ctx.names:
        source = LIDAR_ROLE
    elif ctx.slam_module:
        source = ctx.slam_module
        source_port = "lidar_scan"
    if not source:
        return ()
    return (
        WireSpec(
            source,
            source_port,
            "GatewayModule",
            "lidar_scan",
            delivery="local",
            topic=TOPICS.lidar_scan,
        ),
    )


def localization_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    adapter = ctx.slam_module
    if not adapter:
        return ()

    specs: list[WireSpec] = [
        WireSpec(
            adapter,
            "localization_status",
            consumer,
            "localization_status",
            topic=TOPIC_SLAM_LOCALIZATION_HEALTH,
        )
        for consumer in LOCALIZATION_STATUS_CONSUMERS
    ]
    specs.extend(
        (
            WireSpec(
                adapter,
                "localization_quality",
                "GatewayModule",
                "localization_quality",
                topic=TOPIC_SLAM_LOCALIZATION_QUALITY,
            ),
            WireSpec(adapter, "map_odom_tf", "GatewayModule", "map_odom_tf"),
            WireSpec(adapter, "map_odom_tf", "PerceptionModule", "map_odom_tf"),
        )
    )
    specs.extend(
        WireSpec(adapter, "gnss_fusion_health", consumer, "gnss_fusion_health")
        for consumer in GNSS_FUSION_HEALTH_CONSUMERS
    )
    return tuple(specs)


def odometry_fanout_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    topic = TOPIC_SLAM_ODOMETRY if ctx.slam_module and ctx.nav_odom_src == ctx.slam_module else None
    specs = [
        WireSpec(ctx.nav_odom_src, "odometry", consumer, "odometry", topic=topic) for consumer in ODOMETRY_CONSUMERS
    ]
    specs.append(WireSpec(ctx.nav_odom_src, "odometry", "GatewayModule", "odometry", topic=topic))
    return tuple(specs)
