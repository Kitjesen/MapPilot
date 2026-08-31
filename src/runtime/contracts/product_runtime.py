"""Code-owned topic and capability contracts for compiled Products."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

PRODUCT_CONTROL_SESSION_HEADER = "X-LingTu-Product-Session"
PRODUCT_SESSION_ID_ENV = "LINGTU_PRODUCT_SESSION_ID"


@dataclass(frozen=True)
class ProductRuntimeContract:
    """One versioned Product boundary interpreted by runtime code."""

    contract_id: str
    topics: tuple[str, ...]
    capabilities: tuple[str, ...]

    def as_dict(self) -> dict[str, Any]:
        """Return deterministic JSON-ready catalog data."""

        return {
            "contract_id": self.contract_id,
            "topics": list(self.topics),
            "capabilities": list(self.capabilities),
        }


@dataclass(frozen=True)
class ResolvedProductRuntimeContracts:
    """Ordered union of one or more named Product contracts."""

    contract_ids: tuple[str, ...]
    topics: tuple[str, ...]
    capabilities: tuple[str, ...]


def _contract(
    contract_id: str,
    *,
    topics: Sequence[str],
    capabilities: Sequence[str],
) -> ProductRuntimeContract:
    normalized_topics = tuple(topics)
    normalized_capabilities = tuple(capabilities)
    if (
        not contract_id
        or any(not value for value in normalized_topics)
        or any(not value for value in normalized_capabilities)
        or len(set(normalized_topics)) != len(normalized_topics)
        or len(set(normalized_capabilities)) != len(normalized_capabilities)
    ):
        raise RuntimeError(f"invalid Product runtime contract: {contract_id!r}")
    return ProductRuntimeContract(
        contract_id=contract_id,
        topics=normalized_topics,
        capabilities=normalized_capabilities,
    )


_CONTRACTS = (
    _contract(
        "lingtu.product.teleop.v1",
        topics=(
            "/nav/command/request",
            "/nav/command/ack",
            "/nav/state",
            "/nav/operator_motion/control",
            "/nav/operator_motion/sample",
            "/nav/operator_motion/ack",
            "/nav/operator_motion/status",
            "/nav/cmd_vel",
        ),
        capabilities=(
            "teleop",
            "operator_motion_typed_dds_interface",
            "native_operator_motion_authority",
            "command_staleness_gate",
            "velocity_limit",
            "final_cmd_vel_single_writer",
        ),
    ),
    _contract(
        "lingtu.product.teleop_avoid.v1",
        topics=(
            "/nav/command/request",
            "/nav/command/ack",
            "/nav/state",
            "/nav/operator_motion/control",
            "/nav/operator_motion/sample",
            "/nav/operator_motion/ack",
            "/nav/operator_motion/status",
            "/slam/odometry",
            "/slam/registered_cloud",
            "/slam/map_observation",
            "/slam/localization_health",
            "/nav/local_path",
            "/nav/way_point",
            "/nav/cmd_vel",
        ),
        capabilities=(
            "teleop",
            "operator_motion_typed_dds_interface",
            "native_operator_motion_authority",
            "localization_health_gate",
            "registered_cloud_collision_check",
            "path_follower_pre_command_output",
            "operator_assisted_local_planner_control",
            "final_cmd_vel_single_writer",
            "cmd_vel_arbitration",
        ),
    ),
    _contract(
        "lingtu.product.map.v1",
        topics=(
            "/lidar/raw_frame",
            "/imu/raw",
            "/slam/odometry",
            "/slam/registered_cloud",
            "/slam/map_observation",
            "/slam/map_cloud",
            "/slam/saved_map_cloud",
            "/slam/localization_health",
            "/maps/state",
            "/maps/scene",
            "/nav/command/request",
            "/nav/command/ack",
            "/nav/state",
            "/nav/operator_motion/control",
            "/nav/operator_motion/sample",
            "/nav/operator_motion/ack",
            "/nav/operator_motion/status",
            "/nav/cmd_vel",
        ),
        capabilities=(
            "native_slam_mapping",
            "map_artifact_save",
            "map_pcd_to_octomap_artifact",
            "teleop",
            "operator_motion_typed_dds_interface",
            "native_operator_motion_authority",
            "command_staleness_gate",
            "velocity_limit",
            "final_cmd_vel_single_writer",
        ),
    ),
    _contract(
        "lingtu.product.tracking.v1",
        topics=(
            "/tf",
            "/slam/odometry",
            "/slam/registered_cloud",
            "/slam/map_observation",
            "/slam/localization_health",
            "/maps/state",
            "/maps/scene",
            "/maps/activation/request",
            "/maps/activation/ack",
            "/nav/command/request",
            "/nav/command/ack",
            "/nav/goal/status",
            "/nav/state",
            "/nav/operator_motion/control",
            "/nav/operator_motion/sample",
            "/nav/operator_motion/ack",
            "/nav/operator_motion/status",
            "/nav/traversability",
            "/nav/global_path",
            "/nav/local_path",
            "/nav/way_point",
            "/nav/cmd_vel",
        ),
        capabilities=(
            "saved_map_relocalization",
            "explicit_goal_tracking",
            "local_planner_collision_and_traversability_scoring",
            "path_follower_pre_command_output",
            "final_cmd_vel_single_writer",
            "operator_motion_typed_dds_interface",
            "native_operator_motion_authority",
            "operator_teleop_takeover_with_safety_veto",
            "operator_assisted_local_planner_takeover",
        ),
    ),
    _contract(
        "lingtu.product.nav.v1",
        topics=(
            "/tf",
            "/slam/odometry",
            "/slam/registered_cloud",
            "/slam/map_observation",
            "/slam/map_cloud",
            "/slam/localization_health",
            "/maps/state",
            "/maps/scene",
            "/maps/activation/request",
            "/maps/activation/ack",
            "/nav/command/request",
            "/nav/command/ack",
            "/nav/goal/status",
            "/nav/state",
            "/nav/operator_motion/control",
            "/nav/operator_motion/sample",
            "/nav/operator_motion/ack",
            "/nav/operator_motion/status",
            "/nav/traversability",
            "/nav/global_path",
            "/nav/local_path",
            "/nav/way_point",
            "/nav/cmd_vel",
        ),
        capabilities=(
            "saved_map_relocalization",
            "octoplanner3d_global_planning",
            "local_planner_collision_and_traversability_scoring",
            "path_follower_pre_command_output",
            "final_cmd_vel_single_writer",
            "operator_motion_typed_dds_interface",
            "native_operator_motion_authority",
            "operator_teleop_takeover_with_safety_veto",
            "operator_assisted_local_planner_takeover",
        ),
    ),
    _contract(
        "lingtu.product.inspection.v1",
        topics=(
            "/tf",
            "/slam/odometry",
            "/slam/registered_cloud",
            "/slam/map_observation",
            "/slam/map_cloud",
            "/slam/localization_health",
            "/maps/state",
            "/maps/scene",
            "/maps/activation/request",
            "/maps/activation/ack",
            "/nav/command/request",
            "/nav/command/ack",
            "/nav/goal/status",
            "/nav/state",
            "/nav/operator_motion/control",
            "/nav/operator_motion/sample",
            "/nav/operator_motion/ack",
            "/nav/operator_motion/status",
            "/nav/inspection/task/request",
            "/nav/inspection/task/ack",
            "/nav/inspection/status",
            "/nav/inspection/task/event",
            "/nav/inspection/evidence/request",
            "/nav/inspection/evidence/result",
            "/nav/traversability",
            "/nav/global_path",
            "/nav/local_path",
            "/nav/way_point",
            "/nav/cmd_vel",
        ),
        capabilities=(
            "saved_map_relocalization",
            "operator_or_semantic_goal_source",
            "octoplanner3d_global_planning",
            "local_planner_collision_and_traversability_scoring",
            "final_cmd_vel_single_writer",
            "operator_motion_typed_dds_interface",
            "native_operator_motion_authority",
            "operator_teleop_takeover_with_safety_veto",
            "operator_assisted_local_planner_takeover",
            "inspection_evidence_capture_and_result_ack",
        ),
    ),
    _contract(
        "lingtu.product.explore.v1",
        topics=(
            "/tf",
            "/lidar/raw_frame",
            "/imu/raw",
            "/slam/odometry",
            "/slam/registered_cloud",
            "/slam/map_observation",
            "/slam/map_cloud",
            "/slam/localization_health",
            "/maps/state",
            "/maps/scene",
            "/nav/exploration/command",
            "/nav/exploration/ack",
            "/nav/exploration/run/event",
            "/nav/command/request",
            "/nav/command/ack",
            "/nav/goal/status",
            "/nav/state",
            "/nav/traversability",
            "/nav/exploration_snapshot",
            "/nav/exploration_execution_snapshot",
            "/nav/exploration_segment/request",
            "/nav/exploration_segment/ack",
            "/nav/exploration_segment/status",
            "/nav/global_path",
            "/nav/local_path",
            "/nav/cmd_vel",
        ),
        capabilities=(
            "native_slam_mapping",
            "frontier_or_tare_goal_source",
            "rolling_map_segment_execution",
            "local_planner_collision_and_traversability_scoring",
            "final_cmd_vel_single_writer",
        ),
    ),
    _contract(
        "lingtu.product.explore.map.v1",
        topics=(
            "/tf",
            "/lidar/raw_frame",
            "/imu/raw",
            "/slam/odometry",
            "/slam/registered_cloud",
            "/slam/map_observation",
            "/slam/map_cloud",
            "/slam/localization_health",
            "/maps/state",
            "/maps/scene",
            "/maps/activation/request",
            "/maps/activation/ack",
            "/nav/exploration/command",
            "/nav/exploration/ack",
            "/nav/exploration/run/event",
            "/nav/command/request",
            "/nav/command/ack",
            "/nav/state",
            "/nav/operator_motion/control",
            "/nav/operator_motion/sample",
            "/nav/operator_motion/ack",
            "/nav/operator_motion/status",
            "/nav/goal/status",
            "/nav/traversability",
            "/nav/exploration_snapshot",
            "/nav/exploration_execution_snapshot",
            "/nav/exploration_segment/request",
            "/nav/exploration_segment/ack",
            "/nav/exploration_segment/status",
            "/nav/global_path",
            "/nav/local_path",
            "/nav/way_point",
            "/nav/cmd_vel",
        ),
        capabilities=(
            "saved_map_relocalization",
            "tare_frontier_or_viewpoint_goal_source",
            "octoplanner3d_global_planning",
            "rolling_map_segment_execution",
            "local_planner_collision_and_traversability_scoring",
            "final_cmd_vel_single_writer",
            "operator_motion_typed_dds_interface",
            "native_operator_motion_authority",
            "operator_teleop_takeover_with_safety_veto",
            "operator_assisted_local_planner_takeover",
        ),
    ),
)

PRODUCT_RUNTIME_CONTRACTS: Mapping[str, ProductRuntimeContract] = MappingProxyType(
    {contract.contract_id: contract for contract in _CONTRACTS}
)


def resolve_product_runtime_contracts(
    value: Any,
    *,
    owner: str = "Product",
) -> ResolvedProductRuntimeContracts:
    """Resolve named IDs into their ordered topic and capability closure."""

    if not isinstance(value, list | tuple):
        raise ValueError(f"{owner} contracts must be a list")
    contract_ids = tuple(str(item).strip() for item in value)
    if (
        not contract_ids
        or any(not contract_id for contract_id in contract_ids)
        or len(set(contract_ids)) != len(contract_ids)
    ):
        raise ValueError(f"{owner} contracts contain invalid or duplicate IDs")
    unknown = tuple(contract_id for contract_id in contract_ids if contract_id not in PRODUCT_RUNTIME_CONTRACTS)
    if unknown:
        raise ValueError(f"{owner} references unknown contracts: {', '.join(unknown)}")

    topics: dict[str, None] = {}
    capabilities: dict[str, None] = {}
    for contract_id in contract_ids:
        contract = PRODUCT_RUNTIME_CONTRACTS[contract_id]
        topics.update((topic, None) for topic in contract.topics)
        capabilities.update((capability, None) for capability in contract.capabilities)
    return ResolvedProductRuntimeContracts(
        contract_ids=contract_ids,
        topics=tuple(topics),
        capabilities=tuple(capabilities),
    )


def resolve_product_spec_contracts(
    product: str,
    spec: Mapping[str, Any],
    *,
    product_variant: str | None = None,
) -> ResolvedProductRuntimeContracts:
    """Resolve the named runtime contracts declared by one Product."""

    from runtime.graph.loader import resolve_product_variant_spec

    spec = resolve_product_variant_spec(
        product,
        spec,
        product_variant=product_variant,
    )
    return resolve_product_runtime_contracts(
        spec.get("contracts"),
        owner=f"Product {product!r}",
    )


__all__ = [
    "PRODUCT_CONTROL_SESSION_HEADER",
    "PRODUCT_RUNTIME_CONTRACTS",
    "PRODUCT_SESSION_ID_ENV",
    "ProductRuntimeContract",
    "ResolvedProductRuntimeContracts",
    "resolve_product_runtime_contracts",
    "resolve_product_spec_contracts",
]
