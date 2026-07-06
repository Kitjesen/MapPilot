"""Product mode contracts for module graph and switch planning."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ProductModeContract:
    profile: str
    label: str
    product_mode: str
    product_session: str
    required_modules: frozenset[str]
    forbidden_modules: frozenset[str]
    required_wires: frozenset[str]
    switch_policy: str
    hot_switch_candidates: frozenset[str] = frozenset()
    online_hot_switch_supported: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "profile": self.profile,
            "label": self.label,
            "product_mode": self.product_mode,
            "product_session": self.product_session,
            "required_modules": sorted(self.required_modules),
            "forbidden_modules": sorted(self.forbidden_modules),
            "required_wires": sorted(self.required_wires),
            "switch_policy": self.switch_policy,
            "hot_switch_candidates": sorted(self.hot_switch_candidates),
            "online_hot_switch_supported": self.online_hot_switch_supported,
        }


_COMMAND_OUTPUT_CHAIN = frozenset({
    "nav.velocity_mux.driver_cmd_vel->nav.safety.cmd_vel",
})

_TELEOP_CHAIN = frozenset({
    "GatewayModule.cmd_vel->nav.velocity_mux.teleop_cmd_vel",
    "MCPServerModule.cmd_vel->nav.velocity_mux.teleop_cmd_vel",
    "TeleopModule.cmd_vel->nav.velocity_mux.teleop_cmd_vel",
}) | _COMMAND_OUTPUT_CHAIN

_TELEOP_AVOID_CHAIN = frozenset({
    "SlamAdapterModule.odometry->nav.velocity_mux.collision_odometry@/slam/odometry",
    "TraversabilityCostModule.fused_cost->nav.velocity_mux.collision_costmap",
})

_MAP_CHAIN = frozenset({
    "SlamAdapterModule.odometry->OccupancyGridModule.odometry@/slam/odometry",
    "SlamAdapterModule.odometry->VoxelGridModule.odometry@/slam/odometry",
    "SlamAdapterModule.odometry->ElevationMapModule.odometry@/slam/odometry",
    "SlamAdapterModule.map_cloud->OccupancyGridModule.map_cloud@/slam/map_cloud",
    "SlamAdapterModule.map_cloud->VoxelGridModule.map_cloud@/slam/map_cloud",
    "SlamAdapterModule.map_cloud->ElevationMapModule.map_cloud@/slam/map_cloud",
    "SlamAdapterModule.map_cloud->nav.maps.map_cloud@/slam/map_cloud",
    "OccupancyGridModule.costmap->TraversabilityCostModule.costmap",
    "ESDFModule.esdf->TraversabilityCostModule.esdf",
    "TraversabilityCostModule.fused_cost->GatewayModule.costmap",
})

_SAFETY_CHAIN = frozenset({
    "nav.safety.stop_cmd->nav.mission.stop_signal",
    "GatewayModule.stop_cmd->nav.mission.stop_signal",
    "GatewayModule.cmd_vel->nav.velocity_mux.teleop_cmd_vel",
    "nav.mission.mission_status->GatewayModule.mission_status",
}) | _COMMAND_OUTPUT_CHAIN

_PYTHON_AUTONOMY_MODULES = frozenset({
    "nav.terrain",
    "nav.local_planner",
    "nav.path_follower",
})

_NATIVE_NAV_MISSION_CHAIN = frozenset({
    "SlamAdapterModule.odometry->nav.mission.odometry@/slam/odometry",
    "SlamAdapterModule.localization_status->nav.mission.localization_status@/slam/localization_health",
    "SlamAdapterModule.map_odom_tf->nav.mission.map_odom_tf",
    "SlamAdapterModule.map_frame_jump_event->nav.mission.map_frame_jump_event",
    "TraversabilityCostModule.fused_cost->nav.mission.costmap",
})

_GOAL_SERVICE_CHAIN = frozenset({
    "GatewayModule.goal_pose->nav.goals.goal_request",
    "GatewayModule.cancel->nav.goals.cancel_request",
    "nav.goals.goal_pose->nav.mission.goal_pose",
    "nav.goals.cancel->nav.mission.cancel",
})

_TARE_EXPLORATION_CHAIN = frozenset({
    "OccupancyGridModule.exploration_grid->TAREExplorerModule.exploration_grid",
    "SlamAdapterModule.odometry->TAREExplorerModule.odometry",
    "TAREExplorerModule.exploration_goal->nav.mission.goal_pose",
    "TAREExplorerModule.exploration_path->nav.mission.patrol_goals",
    "nav.mission.mission_status->TAREExplorerModule.navigation_status",
})

_NAV_HOT_CANDIDATES = frozenset({"tracking", "nav", "inspection"})

PRODUCT_MODE_CONTRACTS: dict[str, ProductModeContract] = {
    "teleop": ProductModeContract(
        profile="teleop",
        label="Teleop",
        product_mode="teleop",
        product_session="teleop",
        required_modules=frozenset({
            "GatewayModule",
            "TeleopModule",
            "nav.safety",
            "nav.velocity_mux",
        }),
        forbidden_modules=frozenset({
            "nav.mission",
            "SlamModule",
            "SlamAdapterModule",
            "SlamBridgeModule",
        }) | _PYTHON_AUTONOMY_MODULES,
        required_wires=_TELEOP_CHAIN,
        switch_policy="cold_restart",
    ),
    "teleop_avoid": ProductModeContract(
        profile="teleop_avoid",
        label="Teleop avoid",
        product_mode="teleop_avoid",
        product_session="teleop_avoid",
        required_modules=frozenset({
            "GatewayModule",
            "TeleopModule",
            "nav.safety",
            "nav.velocity_mux",
            "SlamAdapterModule",
            "OccupancyGridModule",
            "TraversabilityCostModule",
        }),
        forbidden_modules=frozenset({
            "nav.mission",
            "SemanticPlannerModule",
        }) | _PYTHON_AUTONOMY_MODULES,
        required_wires=_TELEOP_CHAIN | _MAP_CHAIN | _TELEOP_AVOID_CHAIN,
        switch_policy="cold_restart",
    ),
    "map": ProductModeContract(
        profile="map",
        label="Mapping",
        product_mode="mapping",
        product_session="mapping",
        required_modules=frozenset({
            "GatewayModule",
            "TeleopModule",
            "SlamAdapterModule",
            "OccupancyGridModule",
            "TraversabilityCostModule",
            "nav.maps",
        }),
        forbidden_modules=frozenset({
            "nav.mission",
            "SemanticPlannerModule",
        }) | _PYTHON_AUTONOMY_MODULES,
        required_wires=_TELEOP_CHAIN | _MAP_CHAIN,
        switch_policy="cold_restart",
    ),
    "tracking": ProductModeContract(
        profile="tracking",
        label="Tracking",
        product_mode="tracking",
        product_session="tracking",
        required_modules=frozenset({
            "GatewayModule",
            "SlamAdapterModule",
            "nav.goals",
            "nav.mission",
            "nav.velocity_mux",
        }),
        forbidden_modules=frozenset({"SemanticPlannerModule"}) | _PYTHON_AUTONOMY_MODULES,
        required_wires=_SAFETY_CHAIN | _NATIVE_NAV_MISSION_CHAIN | _GOAL_SERVICE_CHAIN,
        switch_policy="same_graph_candidate",
        hot_switch_candidates=_NAV_HOT_CANDIDATES,
        online_hot_switch_supported=True,
    ),
    "nav": ProductModeContract(
        profile="nav",
        label="Navigation",
        product_mode="navigation",
        product_session="navigation",
        required_modules=frozenset({
            "GatewayModule",
            "SlamAdapterModule",
            "nav.goals",
            "nav.mission",
            "SemanticPlannerModule",
        }),
        forbidden_modules=_PYTHON_AUTONOMY_MODULES,
        required_wires=_SAFETY_CHAIN | _NATIVE_NAV_MISSION_CHAIN | _GOAL_SERVICE_CHAIN,
        switch_policy="same_graph_candidate",
        hot_switch_candidates=_NAV_HOT_CANDIDATES,
        online_hot_switch_supported=True,
    ),
    "inspection": ProductModeContract(
        profile="inspection",
        label="Inspection",
        product_mode="inspection",
        product_session="inspection",
        required_modules=frozenset({
            "GatewayModule",
            "SlamAdapterModule",
            "nav.goals",
            "nav.mission",
            "SemanticPlannerModule",
            "TaskSchedulerModule",
        }),
        forbidden_modules=_PYTHON_AUTONOMY_MODULES,
        required_wires=_SAFETY_CHAIN | _NATIVE_NAV_MISSION_CHAIN | _GOAL_SERVICE_CHAIN,
        switch_policy="same_graph_candidate",
        hot_switch_candidates=_NAV_HOT_CANDIDATES,
        online_hot_switch_supported=True,
    ),
    "tare_explore": ProductModeContract(
        profile="tare_explore",
        label="TARE exploration",
        product_mode="exploration",
        product_session="exploration",
        required_modules=frozenset({
            "GatewayModule",
            "SlamAdapterModule",
            "OccupancyGridModule",
            "VoxelGridModule",
            "ESDFModule",
            "ElevationMapModule",
            "TraversabilityCostModule",
            "TAREExplorerModule",
            "nav.maps",
            "nav.goals",
            "nav.mission",
            "nav.velocity_mux",
        }),
        forbidden_modules=frozenset({"WavefrontFrontierExplorer"}) | _PYTHON_AUTONOMY_MODULES,
        required_wires=(
            _SAFETY_CHAIN
            | _MAP_CHAIN
            | _NATIVE_NAV_MISSION_CHAIN
            | _GOAL_SERVICE_CHAIN
            | _TARE_EXPLORATION_CHAIN
        ),
        switch_policy="cold_restart",
    ),
}


def product_mode_contract(profile: str) -> ProductModeContract:
    return PRODUCT_MODE_CONTRACTS[profile]


def product_mode_switch_plan(current_profile: str, target_profile: str) -> dict[str, Any]:
    current = product_mode_contract(current_profile)
    target = product_mode_contract(target_profile)
    same_graph_candidate = target_profile in current.hot_switch_candidates
    online_supported = (
        current.online_hot_switch_supported
        and target.online_hot_switch_supported
        and same_graph_candidate
    )
    return {
        "current": current.as_dict(),
        "target": target.as_dict(),
        "same_graph_candidate": same_graph_candidate,
        "online_hot_switch_supported": online_supported,
        "required_lifecycle": "hot_switch" if online_supported else "cold_restart",
        "reason": (
            "online hot switch is not enabled for product modes"
            if not online_supported
            else "same graph hot switch is supported"
        ),
    }
