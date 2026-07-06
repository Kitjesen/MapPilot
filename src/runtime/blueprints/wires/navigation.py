"""Navigation, local planning, path following, and exploration wires."""

from __future__ import annotations

from .context import (
    NAV_IN,
    NAV_OUT,
    TOPIC_NAV_CANCEL,
    TOPIC_NAV_CMD_VEL,
    TOPIC_NAV_GLOBAL_PATH,
    TOPIC_NAV_GOAL_POSE,
    TOPIC_NAV_INSTRUCTION,
    TOPIC_NAV_LOCAL_PATH,
    TOPIC_NAV_WAYPOINT,
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


def navigation_io_input_specs() -> tuple[WireSpec, ...]:
    return (
        WireSpec(NAV_IN, "goal_pose", "nav.mission", "goal_pose", topic=TOPIC_NAV_GOAL_POSE),
        WireSpec(NAV_IN, "cancel", "nav.mission", "cancel", topic=TOPIC_NAV_CANCEL),
        WireSpec(NAV_IN, "instruction", "nav.mission", "instruction", topic=TOPIC_NAV_INSTRUCTION),
    )


def navigation_output_specs() -> tuple[WireSpec, ...]:
    return (
        WireSpec(
            "nav.mission",
            "global_path",
            NAV_OUT,
            "global_path",
            topic=TOPIC_NAV_GLOBAL_PATH,
        ),
        WireSpec(
            "nav.local_planner",
            "local_path",
            NAV_OUT,
            "local_path",
            topic=TOPIC_NAV_LOCAL_PATH,
        ),
        WireSpec(
            "nav.mission",
            "waypoint",
            NAV_OUT,
            "waypoint",
            topic=TOPIC_NAV_WAYPOINT,
        ),
        WireSpec(
            "nav.velocity_mux",
            "driver_cmd_vel",
            NAV_OUT,
            "cmd_vel",
            topic=TOPIC_NAV_CMD_VEL,
        ),
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


def navigation_service_specs() -> tuple[WireSpec, ...]:
    return (
        WireSpec("nav.goals", "goal_pose", "nav.mission", "goal_pose"),
        WireSpec("nav.goals", "patrol_goals", "nav.mission", "patrol_goals"),
        WireSpec("nav.goals", "cancel", "nav.mission", "cancel"),
        WireSpec("PatrolManagerModule", "patrol_goals", "nav.mission", "patrol_goals"),
        WireSpec("TaskSchedulerModule", "patrol_command", "PatrolManagerModule", "patrol_command"),
    )
