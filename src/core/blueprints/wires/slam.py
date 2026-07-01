"""SLAM, map-cloud, localization-health, and odometry fan-out wires."""

from __future__ import annotations

from .context import MAP_CLOUD_CONSUMERS, ODOMETRY_CONSUMERS, WiringContext
from .types import WireSpec


LOCALIZATION_STATUS_CONSUMERS = (
    "SafetyRingModule",
    "NavigationModule",
    "DepthVisualOdomModule",
    "GatewayModule",
)

MAP_FRAME_JUMP_CONSUMERS = (
    "NavigationModule",
    "LocalPlannerModule",
    "PathFollowerModule",
)

DEPTH_VISUAL_ODOM_MODULE = "DepthVisualOdomModule"


def map_cloud_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    specs: list[WireSpec] = []
    if ctx.slam_module:
        specs.extend(
            WireSpec(ctx.slam_module, "map_cloud", consumer, "map_cloud")
            for consumer in MAP_CLOUD_CONSUMERS
            if consumer in ctx.names
        )
    elif ctx.scene_xml and "SimPointCloudProvider" in ctx.names:
        specs.extend(
            WireSpec("SimPointCloudProvider", "map_cloud", consumer, "map_cloud")
            for consumer in MAP_CLOUD_CONSUMERS
            if consumer in ctx.names
        )
        specs.append(
            WireSpec(ctx.driver_module, "odometry", "SimPointCloudProvider", "odometry")
        )
    elif ctx.driver_module in {"ROS2SimDriverModule", "MujocoDriverModule"}:
        specs.extend(
            WireSpec(ctx.driver_module, "map_cloud", consumer, "map_cloud")
            for consumer in MAP_CLOUD_CONSUMERS
            if consumer in ctx.names
        )
    return tuple(specs)


def localization_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    adapter = ctx.slam_module
    if not adapter:
        return ()

    specs: list[WireSpec] = [
        WireSpec(adapter, "localization_status", consumer, "localization_status")
        for consumer in LOCALIZATION_STATUS_CONSUMERS
    ]
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
    specs = [
        WireSpec(ctx.nav_odom_src, "odometry", consumer, "odometry")
        for consumer in ODOMETRY_CONSUMERS
    ]
    specs.append(WireSpec(ctx.nav_odom_src, "odometry", "GatewayModule", "odometry"))
    return tuple(specs)
