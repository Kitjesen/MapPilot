"""SLAM, map-cloud, localization-health, and odometry fan-out wires."""

from __future__ import annotations

from runtime.runtime_interface import TOPICS

from .context import (
    MAP_CLOUD_CONSUMERS,
    MAP_CLOUD_FRAME_CONSUMERS,
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
    "DepthVisualOdomModule",
    "GatewayModule",
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
                WireSpec(ctx.slam_module, "map_cloud_frame", consumer, "map_cloud_frame")
                for consumer in MAP_CLOUD_FRAME_CONSUMERS
                if consumer in ctx.names
            )
            legacy_consumers = ("RerunBridgeModule", "GatewayModule")
        else:
            legacy_consumers = MAP_CLOUD_CONSUMERS
            if "nav.maps" in ctx.names:
                specs.append(
                    WireSpec(
                        ctx.slam_module,
                        "map_cloud",
                        "nav.maps",
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
        if "nav.maps" in ctx.names:
            specs.append(
                WireSpec("SimPointCloudProvider", "map_cloud", "nav.maps", "map_cloud")
            )
        specs.append(
            WireSpec(ctx.driver_module, "odometry", "SimPointCloudProvider", "odometry")
        )
    elif ctx.driver_module in {
        "MujocoDriverModule",
        "SimEndpointDriverModule",
    }:
        specs.extend(
            WireSpec(ctx.driver_module, "map_cloud", consumer, "map_cloud")
            for consumer in MAP_CLOUD_CONSUMERS
            if consumer in ctx.names
        )
        if "nav.maps" in ctx.names:
            specs.append(
                WireSpec(ctx.driver_module, "map_cloud", "nav.maps", "map_cloud")
            )
    return tuple(specs)


def sensor_feed_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    if ctx.slam_module != "SlamModule" or "LidarModule" not in ctx.names:
        return ()
    return (
        WireSpec(
            "LidarModule",
            "raw_scan",
            "SlamModule",
            "lidar_raw_scan",
            transport="dds",
            topic=TOPICS.raw_lidar_points,
        ),
        WireSpec(
            "LidarModule",
            "imu",
            "SlamModule",
            "lidar_imu",
            transport="dds",
            topic=TOPICS.raw_imu,
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
        WireSpec(ctx.nav_odom_src, "odometry", consumer, "odometry", topic=topic)
        for consumer in ODOMETRY_CONSUMERS
    ]
    specs.append(WireSpec(ctx.nav_odom_src, "odometry", "GatewayModule", "odometry", topic=topic))
    return tuple(specs)
