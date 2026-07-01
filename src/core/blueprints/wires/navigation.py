"""Navigation, local planning, path following, and exploration wires."""

from __future__ import annotations

from .context import WiringContext
from .types import WireSpec


def exploration_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    return (
        WireSpec("TAREExplorerModule", "exploration_goal", "NavigationModule", "goal_pose"),
        WireSpec("TAREExplorerModule", "exploration_path", "NavigationModule", "patrol_goals"),
        WireSpec(ctx.nav_odom_src, "odometry", "TAREExplorerModule", "odometry"),
        WireSpec("NavigationModule", "mission_status", "TAREExplorerModule", "navigation_status"),
    )


def navigation_input_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    return (
        WireSpec(ctx.nav_odom_src, "odometry", "NavigationModule", "odometry"),
        WireSpec(ctx.nav_odom_src, "odometry", "PerceptionModule", "odometry"),
    )


def path_planning_specs() -> tuple[WireSpec, ...]:
    return (
        WireSpec("NavigationModule", "global_path", "LocalPlannerModule", "global_path"),
        WireSpec("NavigationModule", "global_path", "EndpointPathBridgeModule", "global_path"),
        WireSpec("LocalPlannerModule", "local_path", "EndpointPathBridgeModule", "local_path"),
        # Navigation dispatches staged goals; local geometry comes from
        # TerrainModule and the local planner emits the trackable local_path.
        WireSpec("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint"),
        WireSpec("NavigationModule", "clear_path", "LocalPlannerModule", "clear_path"),
        WireSpec("TerrainModule", "terrain_map", "LocalPlannerModule", "terrain_map"),
        WireSpec("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path"),
        WireSpec("LocalPlannerModule", "local_path", "SafetyRingModule", "path"),
        WireSpec("LocalPlannerModule", "control_hint", "PathFollowerModule", "control_hint"),
    )
