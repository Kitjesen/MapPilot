"""Product mode contracts for module graph and switch planning."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ProductModeContract:
    profile: str
    label: str
    product_mode: str
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
            "required_modules": sorted(self.required_modules),
            "forbidden_modules": sorted(self.forbidden_modules),
            "required_wires": sorted(self.required_wires),
            "switch_policy": self.switch_policy,
            "hot_switch_candidates": sorted(self.hot_switch_candidates),
            "online_hot_switch_supported": self.online_hot_switch_supported,
        }


_COMMAND_OUTPUT_CHAIN = frozenset({
    "nav.velocity_mux.driver_cmd_vel->nav.out.cmd_vel@/nav/cmd_vel",
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

_NAV_EXECUTION_CHAIN = frozenset({
    "TraversabilityCostModule.fused_cost->nav.mission.costmap",
    "nav.terrain.traversability->nav.mission.traversability",
    "nav.mission.global_path->nav.local_planner.global_path",
    "nav.mission.waypoint->nav.local_planner.waypoint",
    "nav.mission.clear_path->nav.local_planner.clear_path",
    "nav.terrain.terrain_map->nav.local_planner.terrain_map",
    "nav.terrain.traversability->nav.local_planner.traversability",
    "nav.local_planner.local_path->nav.path_follower.local_path",
    "nav.local_planner.control_hint->nav.path_follower.control_hint",
    "nav.mission.global_path->nav.out.global_path@/nav/global_path",
    "nav.mission.waypoint->nav.out.waypoint@/nav/way_point",
    "nav.local_planner.local_path->nav.out.local_path@/nav/local_path",
    "nav.path_follower.cmd_vel->nav.velocity_mux.path_follower_cmd_vel",
})

_GOAL_SERVICE_CHAIN = frozenset({
    "GatewayModule.goal_pose->nav.goals.goal_request",
    "GatewayModule.cancel->nav.goals.cancel_request",
    "nav.goals.goal_pose->nav.mission.goal_pose",
    "nav.goals.cancel->nav.mission.cancel",
})

_NAV_HOT_CANDIDATES = frozenset({"tracking", "nav", "inspection"})

PRODUCT_MODE_CONTRACTS: dict[str, ProductModeContract] = {
    "teleop": ProductModeContract(
        profile="teleop",
        label="Teleop",
        product_mode="teleop",
        required_modules=frozenset({
            "GatewayModule",
            "TeleopModule",
            "nav.out",
            "nav.safety",
            "nav.velocity_mux",
        }),
        forbidden_modules=frozenset({
            "nav.mission",
            "nav.local_planner",
            "nav.path_follower",
            "SlamModule",
            "SlamAdapterModule",
            "SlamBridgeModule",
        }),
        required_wires=_TELEOP_CHAIN,
        switch_policy="cold_restart",
    ),
    "teleop_avoid": ProductModeContract(
        profile="teleop_avoid",
        label="Teleop avoid",
        product_mode="teleop_avoid",
        required_modules=frozenset({
            "GatewayModule",
            "TeleopModule",
            "nav.out",
            "nav.safety",
            "nav.velocity_mux",
            "SlamAdapterModule",
            "OccupancyGridModule",
            "TraversabilityCostModule",
        }),
        forbidden_modules=frozenset({
            "nav.mission",
            "nav.local_planner",
            "nav.path_follower",
            "SemanticPlannerModule",
        }),
        required_wires=_TELEOP_CHAIN | _MAP_CHAIN | _TELEOP_AVOID_CHAIN,
        switch_policy="cold_restart",
    ),
    "map": ProductModeContract(
        profile="map",
        label="Mapping",
        product_mode="mapping",
        required_modules=frozenset({
            "GatewayModule",
            "TeleopModule",
            "SlamAdapterModule",
            "OccupancyGridModule",
            "TraversabilityCostModule",
            "nav.maps",
            "nav.out",
        }),
        forbidden_modules=frozenset({
            "nav.mission",
            "nav.local_planner",
            "nav.path_follower",
            "SemanticPlannerModule",
        }),
        required_wires=_TELEOP_CHAIN | _MAP_CHAIN,
        switch_policy="cold_restart",
    ),
    "tracking": ProductModeContract(
        profile="tracking",
        label="Tracking",
        product_mode="tracking",
        required_modules=frozenset({
            "GatewayModule",
            "SlamAdapterModule",
            "nav.mission",
            "nav.local_planner",
            "nav.out",
            "nav.path_follower",
            "nav.velocity_mux",
        }),
        forbidden_modules=frozenset({"SemanticPlannerModule"}),
        required_wires=_SAFETY_CHAIN | _NAV_EXECUTION_CHAIN,
        switch_policy="same_graph_candidate",
        hot_switch_candidates=_NAV_HOT_CANDIDATES,
        online_hot_switch_supported=True,
    ),
    "nav": ProductModeContract(
        profile="nav",
        label="Navigation",
        product_mode="navigation",
        required_modules=frozenset({
            "GatewayModule",
            "SlamAdapterModule",
            "nav.mission",
            "nav.local_planner",
            "nav.out",
            "nav.path_follower",
            "SemanticPlannerModule",
        }),
        forbidden_modules=frozenset(),
        required_wires=_SAFETY_CHAIN | _NAV_EXECUTION_CHAIN,
        switch_policy="same_graph_candidate",
        hot_switch_candidates=_NAV_HOT_CANDIDATES,
        online_hot_switch_supported=True,
    ),
    "inspection": ProductModeContract(
        profile="inspection",
        label="Inspection",
        product_mode="inspection",
        required_modules=frozenset({
            "GatewayModule",
            "SlamAdapterModule",
            "nav.mission",
            "nav.local_planner",
            "nav.out",
            "nav.path_follower",
            "SemanticPlannerModule",
            "TaskSchedulerModule",
        }),
        forbidden_modules=frozenset(),
        required_wires=_SAFETY_CHAIN | _NAV_EXECUTION_CHAIN | _GOAL_SERVICE_CHAIN,
        switch_policy="same_graph_candidate",
        hot_switch_candidates=_NAV_HOT_CANDIDATES,
        online_hot_switch_supported=True,
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
