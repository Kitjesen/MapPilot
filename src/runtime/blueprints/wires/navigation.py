"""Navigation, local planning, path following, and exploration wires."""

from __future__ import annotations

from .context import (
    TOPIC_SLAM_ODOMETRY,
    WiringContext,
)
from .types import WireSpec


def exploration_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    return (
        WireSpec("TAREExplorerModule", "exploration_goal", "nav.mission", "goal_pose"),
        WireSpec("TAREExplorerModule", "exploration_path", "nav.mission", "patrol_goals"),
        WireSpec(ctx.nav_odom_src, "odometry", "TAREExplorerModule", "odometry"),
        WireSpec("nav.mission", "mission_status", "TAREExplorerModule", "navigation_status"),
    )


def navigation_input_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    topic = TOPIC_SLAM_ODOMETRY if ctx.slam_module and ctx.nav_odom_src == ctx.slam_module else None
    return (
        WireSpec(ctx.nav_odom_src, "odometry", "nav.mission", "odometry", topic=topic),
        WireSpec(ctx.nav_odom_src, "odometry", "PerceptionModule", "odometry", topic=topic),
    )


def navigation_execution_specs() -> tuple[WireSpec, ...]:
    """Return Navigation -> LocalPlanner -> follower/safety execution wires.

    Local planner execution is an in-process Module chain. Standalone native
    planner processes are connected through typed endpoint adapters, not
    per-wire transport delivery.
    """

    return (
        WireSpec(
            "nav.mission",
            "global_path",
            "nav.local_planner",
            "global_path",
        ),
        # Navigation dispatches staged goals; local geometry comes from
        # Terrain and the local planner emits the trackable local_path.
        WireSpec(
            "nav.mission",
            "waypoint",
            "nav.local_planner",
            "waypoint",
        ),
        WireSpec(
            "nav.mission",
            "clear_path",
            "nav.local_planner",
            "clear_path",
        ),
        WireSpec(
            "nav.terrain",
            "terrain_map",
            "nav.local_planner",
            "terrain_map",
        ),
        WireSpec(
            "nav.terrain",
            "terrain_map_ext",
            "nav.local_planner",
            "terrain_map_ext",
        ),
        WireSpec(
            "nav.terrain",
            "traversability",
            "nav.local_planner",
            "traversability",
        ),
        WireSpec(
            "nav.local_planner",
            "local_path",
            "nav.path_follower",
            "local_path",
        ),
        WireSpec(
            "nav.local_planner",
            "local_path",
            "nav.safety",
            "path",
        ),
        WireSpec(
            "nav.local_planner",
            "control_hint",
            "nav.path_follower",
            "control_hint",
        ),
    )


def navigation_support_specs() -> tuple[WireSpec, ...]:
    """Wires for navigation interface, patrol, and localization support."""
    return (
        # AI/MCP commands always pass through the canonical goal service.
        WireSpec("nav.mission", "mission_status", "nav.skills", "mission_status"),
        WireSpec("nav.skills", "goal_command", "nav.goals", "goal_command"),
        WireSpec("nav.goals", "goal_status", "nav.skills", "goal_status"),
        # LocalizationMonitorModule <-> Navigation
        WireSpec("nav.localization_monitor", "speed_scale", "nav.mission", "speed_scale"),
        WireSpec(
            "nav.localization_monitor",
            "localization_state",
            "nav.mission",
            "localization_state",
        ),
    )


def navigation_service_specs() -> tuple[WireSpec, ...]:
    return (
        WireSpec("nav.goals", "goal_pose", "nav.mission", "goal_pose"),
        WireSpec("nav.goals", "cancel", "nav.mission", "cancel"),
    )
