"""SLAM, map-cloud, localization-health, and odometry fan-out wires."""

from __future__ import annotations

from runtime.contracts import GNSS_ROLE, LIDAR_ROLE
from runtime.runtime_interface import TOPICS

from .context import (
    MAP_CLOUD_CONSUMERS,
    MAP_CLOUD_FRAME_CONSUMERS,
    MAP_OBSERVATION_CONSUMERS,
    ODOMETRY_CONSUMERS,
    TOPIC_SLAM_LOCALIZATION_HEALTH,
    TOPIC_SLAM_LOCALIZATION_QUALITY,
    TOPIC_SLAM_MAP_CLOUD,
    TOPIC_SLAM_ODOMETRY,
    WiringContext,
)
from .types import WireSpec

LOCALIZATION_STATUS_CONSUMERS = (
    "nav.safety",
    "nav.mission",
    "nav.localization_monitor",
    "DepthVisualOdomModule",
    "GatewayModule",
    "maps.service",
)

GNSS_FUSION_HEALTH_CONSUMERS = (
    "nav.safety",
    "GatewayModule",
)

MAP_FRAME_JUMP_CONSUMERS = (
    "nav.mission",
    "nav.local_planner",
    "nav.path_follower",
)

DEPTH_VISUAL_ODOM_MODULE = "DepthVisualOdomModule"


def map_cloud_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    specs: list[WireSpec] = []
    if ctx.slam_module:
        if ctx.slam_module == "SlamModule":
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
            specs.extend(
                WireSpec(ctx.slam_module, "map_cloud_frame", consumer, "map_cloud_frame")
                for consumer in MAP_CLOUD_FRAME_CONSUMERS
                if consumer in ctx.names
            )
            legacy_consumers = ("RerunBridgeModule", "GatewayModule")
        else:
            legacy_consumers = MAP_CLOUD_CONSUMERS
            if ctx.slam_module == "SlamAdapterModule":
                legacy_consumers = tuple(
                    consumer
                    for consumer in MAP_CLOUD_CONSUMERS
                    if consumer not in MAP_OBSERVATION_CONSUMERS
                )
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
            if "maps.service" in ctx.names:
                specs.append(
                    WireSpec(
                        ctx.slam_module,
                        "map_cloud",
                        "maps.service",
                        "map_cloud",
                        topic=TOPIC_SLAM_MAP_CLOUD,
                    )
                )
        specs.extend(
            WireSpec(
                ctx.slam_module,
                "map_cloud",
                consumer,
                "map_cloud",
                topic=TOPIC_SLAM_MAP_CLOUD,
            )
            for consumer in legacy_consumers
            if consumer in ctx.names
        )
    elif ctx.scene_xml and "SimPointCloudProvider" in ctx.names:
        specs.extend(
            WireSpec("SimPointCloudProvider", "map_cloud", consumer, "map_cloud")
            for consumer in MAP_CLOUD_CONSUMERS
            if consumer in ctx.names
        )
        if "maps.service" in ctx.names:
            specs.append(WireSpec("SimPointCloudProvider", "map_cloud", "maps.service", "map_cloud"))
        specs.append(WireSpec(ctx.driver_module, "odometry", "SimPointCloudProvider", "odometry"))
    elif ctx.driver_module in {
        "MujocoDriverModule",
        "SimEndpointDriverModule",
    }:
        specs.extend(
            WireSpec(ctx.driver_module, "map_cloud", consumer, "map_cloud")
            for consumer in MAP_CLOUD_CONSUMERS
            if consumer in ctx.names
        )
        if "maps.service" in ctx.names:
            specs.append(WireSpec(ctx.driver_module, "map_cloud", "maps.service", "map_cloud"))
    return tuple(specs)


def sensor_feed_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    if ctx.slam_module != "SlamModule":
        return ()
    if LIDAR_ROLE not in ctx.names:
        return ()
    return (
        WireSpec(
            LIDAR_ROLE,
            "raw_scan",
            "SlamModule",
            "lidar_raw_scan",
            delivery="dds",
            topic=TOPICS.raw_lidar_points,
        ),
        WireSpec(
            LIDAR_ROLE,
            "imu",
            "SlamModule",
            "lidar_imu",
            delivery="dds",
            topic=TOPICS.raw_imu,
        ),
    )


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
    elif ctx.slam_module and ctx.slam_module != "SlamModule":
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


def gnss_feed_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    if not ctx.slam_module:
        return ()
    source = ""
    if GNSS_ROLE in ctx.names:
        source = GNSS_ROLE
    if not source:
        return ()
    return (
        WireSpec(
            source,
            "gnss_odom",
            ctx.slam_module,
            "gnss_odom",
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
            WireSpec(adapter, "saved_map", "GatewayModule", "saved_map"),
            WireSpec(
                adapter,
                "localization_quality",
                "GatewayModule",
                "localization_quality",
                topic=TOPIC_SLAM_LOCALIZATION_QUALITY,
            ),
            WireSpec(adapter, "map_odom_tf", "GatewayModule", "map_odom_tf"),
            WireSpec(adapter, "map_odom_tf", "nav.mission", "map_odom_tf"),
            WireSpec(adapter, "map_odom_tf", "nav.local_planner", "map_odom_tf"),
            WireSpec(adapter, "map_odom_tf", "nav.path_follower", "map_odom_tf"),
        )
    )
    specs.extend(
        WireSpec(adapter, "gnss_fusion_health", consumer, "gnss_fusion_health")
        for consumer in GNSS_FUSION_HEALTH_CONSUMERS
    )
    specs.extend(
        WireSpec(adapter, "map_frame_jump_event", consumer, "map_frame_jump_event")
        for consumer in MAP_FRAME_JUMP_CONSUMERS
    )
    specs.extend(
        (
            WireSpec(DEPTH_VISUAL_ODOM_MODULE, "visual_odometry", adapter, "visual_odom"),
            WireSpec(ctx.camera_src, ctx.color_out, DEPTH_VISUAL_ODOM_MODULE, "color_image"),
            WireSpec(ctx.camera_src, "depth_image", DEPTH_VISUAL_ODOM_MODULE, "depth_image"),
            WireSpec(ctx.camera_src, "camera_info", DEPTH_VISUAL_ODOM_MODULE, "camera_info"),
        )
    )
    return tuple(specs)


def odometry_fanout_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    topic = TOPIC_SLAM_ODOMETRY if ctx.slam_module and ctx.nav_odom_src == ctx.slam_module else None
    specs = [
        WireSpec(ctx.nav_odom_src, "odometry", consumer, "odometry", topic=topic) for consumer in ODOMETRY_CONSUMERS
    ]
    specs.append(WireSpec(ctx.nav_odom_src, "odometry", "GatewayModule", "odometry", topic=topic))
    return tuple(specs)
