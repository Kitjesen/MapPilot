"""Product mode contracts for module graph and switch planning."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from runtime.runtime_interface import TOPICS


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


_TELEOP_CHAIN = frozenset()

_MAP_CHAIN = frozenset(
    {
        f"SlamAdapterModule.odometry->OccupancyGridModule.odometry@{TOPICS.odometry}",
        f"SlamAdapterModule.odometry->VoxelGridModule.odometry@{TOPICS.odometry}",
        f"SlamAdapterModule.odometry->ElevationMapModule.odometry@{TOPICS.odometry}",
        f"SlamAdapterModule.map_cloud->OccupancyGridModule.map_cloud@{TOPICS.map_cloud}",
        f"SlamAdapterModule.map_cloud->VoxelGridModule.map_cloud@{TOPICS.map_cloud}",
        f"SlamAdapterModule.map_cloud->ElevationMapModule.map_cloud@{TOPICS.map_cloud}",
        f"SlamAdapterModule.map_cloud->maps.service.map_cloud@{TOPICS.map_cloud}",
        f"VoxelGridModule.scene->GatewayModule.map_scene@{TOPICS.maps_scene}",
        "OccupancyGridModule.costmap->TraversabilityCostModule.costmap",
        "ESDFModule.esdf->TraversabilityCostModule.esdf",
        "TraversabilityCostModule.fused_cost->GatewayModule.costmap",
    }
)

_SAFETY_CHAIN = frozenset(
    {
        "nav.safety.stop_cmd->nav.mission.stop_signal",
        "GatewayModule.stop_cmd->nav.mission.stop_signal",
        "nav.mission.mission_status->GatewayModule.mission_status",
    }
)

_PYTHON_AUTONOMY_MODULES = frozenset(
    {
        "nav.terrain",
        "nav.local_planner",
        "nav.path_follower",
    }
)

_NATIVE_NAV_MISSION_CHAIN = frozenset(
    {
        f"SlamAdapterModule.odometry->nav.mission.odometry@{TOPICS.odometry}",
        f"SlamAdapterModule.localization_status->nav.mission.localization_status@{TOPICS.localization_health}",
        "SlamAdapterModule.map_odom_tf->nav.mission.map_odom_tf",
        "SlamAdapterModule.map_frame_jump_event->nav.mission.map_frame_jump_event",
        "TraversabilityCostModule.fused_cost->nav.mission.costmap",
    }
)

_GOAL_SERVICE_CHAIN = frozenset(
    {
        "GatewayModule.goal_pose->nav.goals.goal_request",
        "GatewayModule.cancel->nav.goals.cancel_request",
        "nav.goals.goal_pose->nav.mission.goal_pose",
        "nav.goals.cancel->nav.mission.cancel",
    }
)

_INSPECTION_EVIDENCE_CHAIN = frozenset(
    {
        f"SlamAdapterModule.odometry->InspectionEvidenceModule.odometry@{TOPICS.odometry}",
        "camera.color_image->InspectionEvidenceModule.color_image",
        "camera.depth_image->InspectionEvidenceModule.depth_image",
        "camera.camera_info->InspectionEvidenceModule.camera_info",
        "PerceptionModule.detections_3d->InspectionEvidenceModule.detections_3d",
    }
)

_TARE_EXPLORATION_CHAIN = frozenset(
    {
        "OccupancyGridModule.exploration_grid->TAREExplorerModule.exploration_grid",
        "SlamAdapterModule.odometry->TAREExplorerModule.odometry",
        "TAREExplorerModule.exploration_goal->nav.mission.goal_pose",
        "TAREExplorerModule.exploration_path->nav.mission.patrol_goals",
        "nav.mission.mission_status->TAREExplorerModule.navigation_status",
    }
)

_NAV_HOT_CANDIDATES = frozenset({"tracking", "nav", "inspection"})

PRODUCT_MODE_CONTRACTS: dict[str, ProductModeContract] = {
    "teleop": ProductModeContract(
        profile="teleop",
        label="Teleop",
        product_mode="teleop",
        product_session="teleop",
        required_modules=frozenset(
            {
                "GatewayModule",
                "TeleopModule",
                "nav.safety",
            }
        ),
        forbidden_modules=frozenset(
            {
                "nav.mission",
                "SlamModule",
                "SlamAdapterModule",
                "nav.velocity_mux",
            }
        )
        | _PYTHON_AUTONOMY_MODULES,
        required_wires=_TELEOP_CHAIN,
        switch_policy="cold_restart",
    ),
    "teleop_avoid": ProductModeContract(
        profile="teleop_avoid",
        label="Teleop avoid",
        product_mode="teleop_avoid",
        product_session="teleop_avoid",
        required_modules=frozenset(
            {
                "GatewayModule",
                "TeleopModule",
                "nav.safety",
                "SlamAdapterModule",
                "OccupancyGridModule",
                "TraversabilityCostModule",
            }
        ),
        forbidden_modules=frozenset(
            {
                "nav.mission",
                "SemanticPlannerModule",
                "nav.velocity_mux",
            }
        )
        | _PYTHON_AUTONOMY_MODULES,
        required_wires=_TELEOP_CHAIN | _MAP_CHAIN,
        switch_policy="cold_restart",
    ),
    "map": ProductModeContract(
        profile="map",
        label="Mapping",
        product_mode="mapping",
        product_session="mapping",
        required_modules=frozenset(
            {
                "GatewayModule",
                "TeleopModule",
                "SlamAdapterModule",
                "OccupancyGridModule",
                "TraversabilityCostModule",
                "maps.service",
            }
        ),
        forbidden_modules=frozenset(
            {
                "nav.mission",
                "SemanticPlannerModule",
                "nav.velocity_mux",
            }
        )
        | _PYTHON_AUTONOMY_MODULES,
        required_wires=_TELEOP_CHAIN | _MAP_CHAIN,
        switch_policy="cold_restart",
    ),
    "tracking": ProductModeContract(
        profile="tracking",
        label="Tracking",
        product_mode="tracking",
        product_session="tracking",
        required_modules=frozenset(
            {
                "GatewayModule",
                "SlamAdapterModule",
                "nav.goals",
                "nav.mission",
            }
        ),
        forbidden_modules=frozenset(
            {
                "SemanticPlannerModule",
                "nav.velocity_mux",
            }
        )
        | _PYTHON_AUTONOMY_MODULES,
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
        required_modules=frozenset(
            {
                "GatewayModule",
                "SlamAdapterModule",
                "nav.goals",
                "nav.mission",
                "SemanticPlannerModule",
            }
        ),
        forbidden_modules=frozenset({"nav.velocity_mux"}) | _PYTHON_AUTONOMY_MODULES,
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
        required_modules=frozenset(
            {
                "GatewayModule",
                "SlamAdapterModule",
                "nav.goals",
                "nav.mission",
                "SemanticPlannerModule",
                "PerceptionModule",
                "InspectionEvidenceModule",
            }
        ),
        forbidden_modules=frozenset({"nav.velocity_mux"}) | _PYTHON_AUTONOMY_MODULES,
        required_wires=(
            _SAFETY_CHAIN
            | _NATIVE_NAV_MISSION_CHAIN
            | _GOAL_SERVICE_CHAIN
            | _INSPECTION_EVIDENCE_CHAIN
        ),
        switch_policy="same_graph_candidate",
        hot_switch_candidates=_NAV_HOT_CANDIDATES,
        online_hot_switch_supported=True,
    ),
    "tare_explore": ProductModeContract(
        profile="tare_explore",
        label="TARE exploration",
        product_mode="exploration",
        product_session="exploration",
        required_modules=frozenset(
            {
                "GatewayModule",
                "SlamAdapterModule",
                "OccupancyGridModule",
                "VoxelGridModule",
                "ESDFModule",
                "ElevationMapModule",
                "TraversabilityCostModule",
                "TAREExplorerModule",
                "maps.service",
                "nav.goals",
                "nav.mission",
            }
        ),
        forbidden_modules=frozenset(
            {
                "WavefrontFrontierExplorer",
                "nav.velocity_mux",
            }
        )
        | _PYTHON_AUTONOMY_MODULES,
        required_wires=(
            _SAFETY_CHAIN | _MAP_CHAIN | _NATIVE_NAV_MISSION_CHAIN | _GOAL_SERVICE_CHAIN | _TARE_EXPLORATION_CHAIN
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
        current.online_hot_switch_supported and target.online_hot_switch_supported and same_graph_candidate
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
