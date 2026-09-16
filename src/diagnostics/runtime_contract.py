"""Diagnostic projections and explicit compatibility-adapter contracts.

Messages and wire values belong to message/idl; topics to message/topics;
frames and transforms to runtime/tf; Product assembly to runtime_graph.
This module describes those surfaces for diagnostics, not a second registry.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import asdict, dataclass
from typing import Any

from message.generated.schema import MESSAGE_FIELDS
from message.topics import TOPIC_SPECS, TOPICS
from runtime.adapters.topics import ADAPTER_RELAY_ALIASES, ADAPTER_TOPIC_ALIASES
from runtime.tf.frames import (
    FRAME_LINKS,
    FRAMES,
    REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS,
    runtime_frames_contract,
    runtime_topic_allowed_frame_contract,
    runtime_topic_default_frame_ids,
)
from runtime.tf.mounts import lidar_extrinsics

REAL_RUNTIME_CONTRACT = "real"
FIELD_DATA_SOURCE = "field"
REAL_RUNTIME_EVIDENCE_LABEL = "Real runtime evidence"

RUNTIME_CONTRACT_DATA_SOURCES = {
    REAL_RUNTIME_CONTRACT: FIELD_DATA_SOURCE,
}


def canonical_data_source_name(name: str | None) -> str | None:
    """Normalize a data-source/runtime-contract value."""

    if name is None:
        return None
    return str(name)


def runtime_contract_data_source(name: str | None) -> str | None:
    """Resolve an evidence contract to its concrete endpoint data source."""

    normalized = canonical_data_source_name(name)
    if normalized is None:
        return None
    return RUNTIME_CONTRACT_DATA_SOURCES.get(normalized, normalized)


@dataclass(frozen=True)
class RuntimeDataFlowStage:
    """Canonical runtime data-flow stage crossing endpoint and module boundaries."""

    name: str
    inputs: tuple[str, ...]
    outputs: tuple[str, ...]
    owner: str
    frame_role: str
    map_dependency: str
    producer: str = ""
    consumers: tuple[str, ...] = ()
    frequency: str = ""
    transport_policy: str = "direct"


@dataclass(frozen=True)
class ArtifactFormat:
    """Saved-map artifact contract shared by mapping and navigation."""

    name: str
    path: str
    artifact_type: str
    frame_role: str
    required_fields: tuple[str, ...] = ()
    required_metadata: tuple[str, ...] = ()
    note: str = ""


@dataclass(frozen=True)
class AlgorithmInterface:
    """Inputs and outputs owned by one LingTu algorithm surface."""

    name: str
    inputs: tuple[str, ...]
    outputs: tuple[str, ...]
    owner: str
    map_dependency: str


@dataclass(frozen=True)
class DataSourceContract:
    """How an endpoint must normalize its native data into LingTu."""

    name: str
    provider: str
    owns: tuple[str, ...]
    normalized_outputs: tuple[str, ...]
    command_sink: str
    source_outputs: tuple[str, ...] = ()
    algorithm_entry_outputs: tuple[str, ...] = ()
    algorithm_context_outputs: tuple[str, ...] = ()
    lidar_extrinsic_profile: str | None = None
    slam_source: str = "not_declared"
    localization_source: str = "not_declared"
    mapping_source: str = "not_declared"


@dataclass(frozen=True)
class ProductDataSourceBinding:
    """Which endpoint data source one Field Product is allowed to use."""

    product: str
    data_source: str
    mode: str
    note: str = ""


CORE_ALGORITHM_ENTRY_TOPICS = (
    TOPICS.odometry,
    TOPICS.registered_cloud,
    TOPICS.map_cloud,
)

CANONICAL_NAV_TOPICS = (
    *CORE_ALGORITHM_ENTRY_TOPICS,
    TOPICS.exploration_grid,
    TOPICS.exploration_snapshot,
    TOPICS.exploration_execution_snapshot,
    TOPICS.global_path,
    TOPICS.local_path,
    TOPICS.terrain_map,
    TOPICS.terrain_map_ext,
    TOPICS.traversability,
    TOPICS.cmd_vel,
)

CORE_REQUIRED_TOPICS = (
    *CANONICAL_NAV_TOPICS,
    TOPICS.nav_command_request,
)


RUNTIME_DATA_FLOW = (
    RuntimeDataFlowStage(
        name="endpoint_adapter",
        inputs=("source:data_source.source_outputs",),
        outputs=("source:data_source.normalized_outputs",),
        owner="endpoint_adapter",
        frame_role="native_to_canonical",
        map_dependency="declared_by_data_source",
        producer="driver_or_endpoint_adapter",
        consumers=("slam_or_source_adapter", "gateway_diagnostics"),
        frequency="sensor_native",
        transport_policy="direct_in_process_or_endpoint_transport",
    ),
    RuntimeDataFlowStage(
        name="slam_or_relayed_localization_map",
        inputs=(
            TOPICS.lidar_scan,
            TOPICS.imu,
            TOPICS.lidar_scan,
            TOPICS.imu,
            "source:data_source.algorithm_entry_outputs",
        ),
        outputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
        owner="slam_or_source_adapter",
        frame_role="map_odom_body",
        map_dependency="declared_by_data_source",
        producer="SlamModule_or_SlamAdapterModule",
        consumers=("map_layers", "navigation", "safety", "gateway"),
        frequency="odometry_50_200hz_cloud_5_20hz",
        transport_policy="direct_or_shm_for_clouds_dds_for_endpoint_bridge",
    ),
    RuntimeDataFlowStage(
        name="map_layers_and_exploration",
        inputs=(
            TOPICS.odometry,
            TOPICS.registered_cloud,
            TOPICS.map_cloud,
            TOPICS.exploration_grid,
            TOPICS.terrain_map_ext,
        ),
        outputs=(TOPICS.exploration_way_point,),
        owner="maps_or_tare_adapter",
        frame_role=FRAMES.map,
        map_dependency="live_map_or_occupancy_grid",
        producer="maps_modules_or_exploration_adapter",
        consumers=("global_planning", "gateway", "exploration"),
        frequency="map_layers_1_10hz_frontier_0.5_2hz",
        transport_policy="direct_default_shm_for_dense_maps_dds_for_external_exploration",
    ),
    RuntimeDataFlowStage(
        name="tare_exploration",
        inputs=(
            TOPICS.odometry,
            "/tf",
            TOPICS.exploration_snapshot,
            TOPICS.exploration_command,
            TOPICS.nav_goal_status,
            TOPICS.exploration_segment_ack,
            TOPICS.exploration_segment_status,
        ),
        outputs=(
            TOPICS.nav_command_request,
            TOPICS.exploration_ack,
            TOPICS.exploration_run_event,
            TOPICS.exploration_segment_request,
        ),
        owner="native_explore_endpoint",
        frame_role=FRAMES.map,
        map_dependency="identity_versioned_rolling_occupancy_snapshot",
        producer="native_explore_endpoint",
        consumers=("native_nav_endpoint", "gateway"),
        frequency="command_and_segment_event_driven",
        transport_policy="typed_dds_process_boundary",
    ),
    RuntimeDataFlowStage(
        name="rolling_map_segment_execution",
        inputs=(
            TOPICS.odometry,
            TOPICS.exploration_execution_snapshot,
            TOPICS.exploration_segment_request,
        ),
        outputs=(
            TOPICS.global_path,
            TOPICS.exploration_segment_ack,
            TOPICS.exploration_segment_status,
        ),
        owner="native_nav_endpoint",
        frame_role=FRAMES.map,
        map_dependency="atomic_identity_bound_rolling_execution_grid",
        producer="native_nav_endpoint",
        consumers=("native_explore_endpoint", "local_planning_and_following", "gateway"),
        frequency="segment_event_driven",
        transport_policy="typed_dds_process_boundary",
    ),
    RuntimeDataFlowStage(
        name="global_planning",
        inputs=(
            TOPICS.odometry,
            TOPICS.map_cloud,
            TOPICS.exploration_grid,
            TOPICS.exploration_way_point,
            TOPICS.nav_command_request,
            "artifact:octomap",
            "artifact:point_cloud",
        ),
        outputs=(TOPICS.global_path, TOPICS.nav_way_point),
        owner="lingtu_navigation_or_planner_backend",
        frame_role=FRAMES.map,
        map_dependency=("octoplanner3d_uses_headless_octomap_or_point_cloud"),
        producer="Navigation_global_planner",
        consumers=("LocalPlanner", "GatewayModule", "lingtu-nav-dds"),
        frequency="on_goal_or_replan",
        transport_policy="direct_in_process_module_chain; typed_dds_endpoint_for_process_boundary",
    ),
    RuntimeDataFlowStage(
        name="local_planning_and_following",
        inputs=(
            TOPICS.odometry,
            TOPICS.terrain_map,
            TOPICS.terrain_map_ext,
            TOPICS.traversability,
            TOPICS.global_path,
            TOPICS.nav_way_point,
        ),
        outputs=(TOPICS.local_path, TOPICS.local_planner_control_hint, TOPICS.cmd_vel),
        owner="lingtu_autonomy",
        frame_role="map_odom_body_local_terrain",
        map_dependency="live_terrain_maps_traversability_and_global_path",
        producer="LocalPlanner_and_PathFollower",
        consumers=("PathFollower", "SafetyRing", "VelocityMux", "GatewayModule"),
        frequency="local_plan_5_20hz_cmd_vel_20_50hz",
        transport_policy="direct_default_shm_for_external_cpp_local_planner_dds_for_low_rate_path_state",
    ),
    RuntimeDataFlowStage(
        name="dynamic_obstacle_gate",
        inputs=(TOPICS.added_obstacles, TOPICS.local_path, TOPICS.cmd_vel),
        outputs=(TOPICS.check_obstacle, TOPICS.planner_status),
        owner="local_planner_dynamic_obstacle_gate",
        frame_role="odom_body_registered_cloud",
        map_dependency="current_registered_cloud_and_added_obstacles",
        producer="SafetyRing_or_dynamic_obstacle_gate",
        consumers=("Navigation", "VelocityMux", "GatewayModule"),
        frequency="10_50hz",
        transport_policy="direct_for_safety_path",
    ),
    RuntimeDataFlowStage(
        name="command_boundary",
        inputs=(TOPICS.cmd_vel,),
        outputs=("sink:data_source.command_sink",),
        owner="command_arbiter_to_driver",
        frame_role="body_twist",
        map_dependency="none",
        producer="command_arbiter",
        consumers=("driver",),
        frequency="20_50hz",
        transport_policy="direct_or_dds_to_driver_at_hardware_boundary",
    ),
)

PRODUCT_SCOPED_RUNTIME_DATA_FLOW_STAGE_NAMES = frozenset(
    {
        "tare_exploration",
        "rolling_map_segment_execution",
    }
)


REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS = (
    TOPICS.lidar_scan,
    TOPICS.imu,
)

ARTIFACT_FORMATS = {
    "map_pcd": ArtifactFormat(
        name="map_pcd",
        path="map.pcd",
        artifact_type="pcd_xyz_or_xyzi",
        frame_role=FRAMES.map,
        required_fields=("x", "y", "z"),
        required_metadata=(
            "source_profile",
            "data_source",
            "slam_source",
            "frame_id",
            "point_count",
        ),
        note="Saved map point cloud used by relocalization and native map conversion.",
    ),
    "octomap": ArtifactFormat(
        name="octomap",
        path="octomap.ot",
        artifact_type="octomap_full_tree",
        frame_role=FRAMES.map,
        required_fields=("occupancy_tree",),
        required_metadata=("source_profile", "data_source", "frame_id"),
        note="OctoMap full-tree artifact consumed by the OctoPlanner3D headless backend.",
    ),
    "point_cloud": ArtifactFormat(
        name="point_cloud",
        path="map.pcd",
        artifact_type="pcd_xyz_or_xyzi",
        frame_role=FRAMES.map,
        required_fields=("x", "y", "z"),
        required_metadata=("source_profile", "data_source", "frame_id", "point_count"),
        note="Point cloud .pcd artifact convertible to OctoMap for OctoPlanner3D.",
    ),
    "occupancy_grid": ArtifactFormat(
        name="occupancy_grid",
        path="occupancy.npz",
        artifact_type="numpy_occupancy_grid",
        frame_role=FRAMES.map,
        required_fields=("grid", "resolution", "origin"),
        required_metadata=("source_profile", "data_source", "frame_id"),
        note="2D occupancy artifact for A*/frontier and map review.",
    ),
    "metadata": ArtifactFormat(
        name="metadata",
        path="metadata.json",
        artifact_type="json",
        frame_role=FRAMES.map,
        required_fields=(
            "source_profile",
            "data_source",
            "slam_source",
            "localization_source",
            "mapping_source",
            "frame_id",
            "created_at",
            "artifacts",
        ),
        required_metadata=(),
        note="Provenance file binding saved artifacts back to the runtime data source.",
    ),
}

ALGORITHM_INTERFACES = {
    "fastlio_mapping": AlgorithmInterface(
        name="fastlio_mapping",
        inputs=(TOPICS.lidar_scan, TOPICS.imu),
        outputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
        owner="slam",
        map_dependency="none_live_canonical_sensor_slam",
    ),
    "fastlio_raw_validation": AlgorithmInterface(
        name="fastlio_raw_validation",
        inputs=(TOPICS.lidar_scan, TOPICS.imu),
        outputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
        owner="slam_sim_validation",
        map_dependency="none_live_raw_sensor_slam",
    ),
    "exploration_strategy": AlgorithmInterface(
        name="exploration_strategy",
        inputs=(TOPICS.odometry, TOPICS.map_cloud, TOPICS.exploration_grid),
        outputs=(TOPICS.exploration_way_point,),
        owner="tare",
        map_dependency="live_map_or_occupancy_grid",
    ),
    "tare_exploration": AlgorithmInterface(
        name="tare_exploration",
        inputs=(
            TOPICS.odometry,
            "/tf",
            TOPICS.exploration_snapshot,
            TOPICS.exploration_command,
            TOPICS.nav_goal_status,
            TOPICS.exploration_segment_ack,
            TOPICS.exploration_segment_status,
        ),
        outputs=(
            TOPICS.nav_command_request,
            TOPICS.exploration_ack,
            TOPICS.exploration_run_event,
            TOPICS.exploration_segment_request,
        ),
        owner="native_explore_endpoint",
        map_dependency="identity_versioned_rolling_occupancy_snapshot",
    ),
    "rolling_map_segment_execution": AlgorithmInterface(
        name="rolling_map_segment_execution",
        inputs=(
            TOPICS.odometry,
            TOPICS.exploration_execution_snapshot,
            TOPICS.exploration_segment_request,
        ),
        outputs=(
            TOPICS.global_path,
            TOPICS.exploration_segment_ack,
            TOPICS.exploration_segment_status,
        ),
        owner="native_nav_endpoint",
        map_dependency="atomic_identity_bound_rolling_execution_grid",
    ),
    "global_planning": AlgorithmInterface(
        name="global_planning",
        inputs=(TOPICS.odometry, TOPICS.map_cloud, TOPICS.exploration_way_point, TOPICS.nav_command_request),
        outputs=(TOPICS.global_path, TOPICS.nav_way_point),
        owner="lingtu_navigation",
        map_dependency="planner_specific_octoplanner3d_octomap",
    ),
    "octoplanner3d_global_planning": AlgorithmInterface(
        name="octoplanner3d_global_planning",
        inputs=(
            TOPICS.odometry,
            "artifact:octomap",
            TOPICS.nav_command_request,
        ),
        outputs=(TOPICS.global_path, TOPICS.nav_way_point),
        owner="lingtu_octoplanner3d",
        map_dependency="saved_octomap_ot_or_bt",
    ),
    "local_planning_and_following": AlgorithmInterface(
        name="local_planning_and_following",
        inputs=(
            TOPICS.odometry,
            TOPICS.terrain_map,
            TOPICS.terrain_map_ext,
            TOPICS.traversability,
            TOPICS.global_path,
            TOPICS.nav_way_point,
        ),
        outputs=(TOPICS.local_path, TOPICS.local_planner_control_hint, TOPICS.cmd_vel),
        owner="lingtu_autonomy",
        map_dependency="live_terrain_maps_traversability_and_global_path",
    ),
}

RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES = {
    "slam_or_relayed_localization_map": (
        "fastlio_mapping",
        "fastlio_raw_validation",
    ),
    "map_layers_and_exploration": ("exploration_strategy",),
    "tare_exploration": ("tare_exploration",),
    "rolling_map_segment_execution": ("rolling_map_segment_execution",),
    "global_planning": (
        "global_planning",
        "octoplanner3d_global_planning",
    ),
    "local_planning_and_following": ("local_planning_and_following",),
}

DATA_SOURCE_CONTRACTS = {
    "in_process_stub": DataSourceContract(
        name="in_process_stub",
        provider="in_process",
        owns=("mock_odometry", "mock_map", "mock_commands"),
        normalized_outputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
        command_sink="module_graph_driver_cmd_vel",
        source_outputs=(),
        algorithm_entry_outputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
        algorithm_context_outputs=(),
        lidar_extrinsic_profile=None,
        slam_source="none",
        localization_source="mock_or_in_process_odometry",
        mapping_source="mock_or_in_process_map",
    ),
    FIELD_DATA_SOURCE: DataSourceContract(
        name=FIELD_DATA_SOURCE,
        provider="hardware",
        owns=("mid360_lidar", "imu", "robot_actuation"),
        normalized_outputs=(TOPICS.lidar_scan, TOPICS.imu),
        command_sink="driver",
        source_outputs=(TOPICS.lidar_scan, TOPICS.imu),
        algorithm_entry_outputs=(
            TOPICS.odometry,
            TOPICS.registered_cloud,
            TOPICS.map_cloud,
            TOPICS.localization_health,
            TOPICS.localization_quality,
        ),
        algorithm_context_outputs=(),
        # The selected Robot model supplies its own physical LiDAR mount.
        lidar_extrinsic_profile=None,
        slam_source="lingtu_fastlio_or_external_robot_slam",
        localization_source="slam_localizer",
        mapping_source="slam_map_cloud",
    ),
    "mujoco_module_graph": DataSourceContract(
        name="mujoco_module_graph",
        provider="mujoco",
        owns=(
            "physics",
            "rendered_lidar",
            "simulated_imu_state",
            "fixed_terrain_height_rays",
            "rgb_camera",
            "depth_camera",
            "simulation_actuation",
        ),
        normalized_outputs=(
            TOPICS.odometry,
            TOPICS.registered_cloud,
            TOPICS.map_cloud,
            TOPICS.height_rays,
            TOPICS.camera_color,
            TOPICS.camera_depth,
            TOPICS.camera_info,
        ),
        command_sink="mujoco_driver_module_cmd_vel",
        source_outputs=(
            TOPICS.odometry,
            TOPICS.registered_cloud,
            TOPICS.map_cloud,
            TOPICS.height_rays,
            TOPICS.camera_color,
            TOPICS.camera_depth,
            TOPICS.camera_info,
        ),
        algorithm_entry_outputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
        algorithm_context_outputs=(TOPICS.height_rays,),
        lidar_extrinsic_profile="mujoco_thunder_v3",
        slam_source="none",
        localization_source="mujoco_sim_odometry",
        mapping_source="mujoco_rendered_lidar_map_cloud",
    ),
    "mujoco_fastlio2_live": DataSourceContract(
        name="mujoco_fastlio2_live",
        provider="mujoco",
        owns=("physics", "mid360_pattern_lidar", "imu"),
        normalized_outputs=(TOPICS.lidar_scan, TOPICS.imu),
        command_sink="mujoco_velocity_adapter",
        source_outputs=(TOPICS.lidar_scan, TOPICS.imu),
        algorithm_entry_outputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
        algorithm_context_outputs=(),
        lidar_extrinsic_profile="mujoco_thunder_v3",
        slam_source="lingtu_fastlio2",
        localization_source="fastlio2_odometry",
        mapping_source="fastlio2_map_cloud",
    ),
}


def _dedupe_runtime_tokens(tokens: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(dict.fromkeys(token for token in tokens if token))


def _data_source_contract(data_source: str | DataSourceContract) -> DataSourceContract:
    if isinstance(data_source, DataSourceContract):
        return data_source
    data_source = runtime_contract_data_source(data_source) or ""
    try:
        return DATA_SOURCE_CONTRACTS[data_source]
    except KeyError as exc:
        available = ", ".join(sorted(DATA_SOURCE_CONTRACTS))
        raise ValueError(f"unknown data source {data_source!r}; available: {available}") from exc


def _resolved_runtime_data_flow(
    data_source: str | DataSourceContract,
    *,
    product_stage_names: frozenset[str],
) -> tuple[RuntimeDataFlowStage, ...]:
    source = _data_source_contract(data_source)
    stages: list[RuntimeDataFlowStage] = []
    minimal_command_only = (
        not source.normalized_outputs
        and not source.algorithm_entry_outputs
        and not source.algorithm_context_outputs
        and source.slam_source == "none"
        and source.mapping_source == "none"
    )
    minimal_stage_names = {"endpoint_adapter", "command_boundary"}

    for stage in RUNTIME_DATA_FLOW:
        if stage.name in PRODUCT_SCOPED_RUNTIME_DATA_FLOW_STAGE_NAMES and stage.name not in product_stage_names:
            continue
        if minimal_command_only and stage.name not in minimal_stage_names:
            continue

        inputs = stage.inputs
        outputs = stage.outputs

        if stage.name == "endpoint_adapter":
            inputs = source.source_outputs
            outputs = source.normalized_outputs
        elif stage.name == "slam_or_relayed_localization_map":
            inputs = source.normalized_outputs
            outputs = source.algorithm_entry_outputs
        elif stage.name == "map_layers_and_exploration":
            inputs = _dedupe_runtime_tokens(
                source.algorithm_entry_outputs
                + source.algorithm_context_outputs
                + (
                    TOPICS.exploration_grid,
                    TOPICS.terrain_map_ext,
                )
            )
        elif stage.name == "command_boundary":
            outputs = (source.command_sink,)

        stages.append(
            RuntimeDataFlowStage(
                name=stage.name,
                inputs=_dedupe_runtime_tokens(inputs),
                outputs=_dedupe_runtime_tokens(outputs),
                owner=stage.owner,
                frame_role=stage.frame_role,
                map_dependency=stage.map_dependency,
            )
        )

    return tuple(stages)


def resolved_runtime_data_flow(
    data_source: str | DataSourceContract,
) -> tuple[RuntimeDataFlowStage, ...]:
    """Return the concrete data-source flow, excluding Product-only stages.

    RUNTIME_DATA_FLOW is the shared template used by evidence validators. This
    resolver expands the source-owned boundary so operators can inspect actual
    topics and command sinks instead of template placeholders.
    """

    return _resolved_runtime_data_flow(
        data_source,
        product_stage_names=frozenset(),
    )


def product_runtime_data_flow_stage_names(
    required_topics: Iterable[str],
) -> tuple[str, ...]:
    """Derive Product-only stages from one Product's declared topic surface.

    A Product activates a scoped stage by declaring every output of that stage.
    Input delivery is verified from the observed runtime flow, not by a second
    static audit of this declaration.
    """

    declared_topics = frozenset(str(topic) for topic in required_topics)
    stage_names: list[str] = []
    for stage in RUNTIME_DATA_FLOW:
        if stage.name not in PRODUCT_SCOPED_RUNTIME_DATA_FLOW_STAGE_NAMES:
            continue
        output_topics = frozenset(token for token in stage.outputs if token.startswith("/"))
        if output_topics and output_topics <= declared_topics:
            stage_names.append(stage.name)
    return tuple(stage_names)


def resolved_product_runtime_data_flow(
    data_source: str | DataSourceContract,
    required_topics: Iterable[str],
) -> tuple[RuntimeDataFlowStage, ...]:
    """Return the data-source flow plus stages activated by one Product."""

    return _resolved_runtime_data_flow(
        data_source,
        product_stage_names=frozenset(product_runtime_data_flow_stage_names(required_topics)),
    )


def topic_formats(topic: str) -> tuple[str, ...]:
    """Project canonical DDS types, or an explicitly declared adapter format."""
    if topic in TOPIC_SPECS:
        return (TOPIC_SPECS[topic].message_type,)
    if topic in asdict(TOPICS).values():
        return ("python.object",)
    formats = tuple(
        dict.fromkeys(
            alias.msg_format for aliases in ADAPTER_TOPIC_ALIASES.values() for alias in aliases if alias.source == topic
        )
    )
    if not formats:
        raise ValueError(f"topic {topic!r} has no declared runtime format")
    return formats


def _message_formats() -> dict[str, dict[str, Any]]:
    """Describe IDL payloads without maintaining another field registry."""
    formats = {
        name: {"name": name, "frame_role": "message_specific", "required_fields": fields, "note": "DDS IDL fields"}
        for name, fields in MESSAGE_FIELDS.items()
    }
    for name in {"python.object"} | {
        alias.msg_format for aliases in ADAPTER_TOPIC_ALIASES.values() for alias in aliases
    }:
        formats.setdefault(
            name,
            {
                "name": name,
                "frame_role": "adapter_specific",
                "required_fields": (),
                "note": "Local or compatibility adapter payload; see its implementation.",
            },
        )
    return formats


def normalize_algorithm_interface_contract(
    interfaces: Mapping[str, Any] | None,
) -> dict[str, dict[str, Any]]:
    """Return JSON-ready algorithm interface contract data."""

    if not isinstance(interfaces, Mapping):
        return {}
    normalized: dict[str, dict[str, Any]] = {}
    for name, interface in interfaces.items():
        if isinstance(interface, AlgorithmInterface):
            source: Mapping[str, Any] = asdict(interface)
        elif isinstance(interface, Mapping):
            source = interface
        else:
            continue
        normalized[str(name)] = {
            "name": str(source.get("name") or ""),
            "inputs": _jsonable_string_list(source.get("inputs")),
            "outputs": _jsonable_string_list(source.get("outputs")),
            "owner": str(source.get("owner") or ""),
            "map_dependency": str(source.get("map_dependency") or ""),
        }
    return normalized


def runtime_algorithm_interface_contract() -> dict[str, dict[str, Any]]:
    """Return JSON-ready algorithm interface contract data."""

    return normalize_algorithm_interface_contract(ALGORITHM_INTERFACES)


def runtime_stage_algorithm_interface_contract() -> dict[str, list[str]]:
    """Return JSON-ready runtime data-flow stage to algorithm interface binding."""

    return {stage: list(interfaces) for stage, interfaces in RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES.items()}


def _jsonable_string_list(value: Any) -> list[str]:
    if isinstance(value, str):
        return [value]
    if isinstance(value, (list, tuple)):
        return [str(item) for item in value]
    return []


def runtime_data_flow_topics(runtime_contract: str) -> tuple[str, ...]:
    """Return unique canonical runtime stream tokens in one resolved data-flow."""

    topics: list[str] = []
    seen: set[str] = set()
    for stage in resolved_runtime_data_flow(runtime_contract):
        for token in (*stage.inputs, *stage.outputs):
            if not isinstance(token, str) or not token.startswith("/"):
                continue
            if token in seen:
                continue
            seen.add(token)
            topics.append(token)
    return tuple(topics)


def _product_data_sources() -> dict[str, ProductDataSourceBinding]:
    """Project field Product descriptions from the assembly declarations."""
    from lingtu.assembly.graph.loader import load_runtime_graph, resolve_product_variant_spec

    bindings = {}
    for name, declaration in load_runtime_graph().products.items():
        spec = resolve_product_variant_spec(name, declaration)
        bindings[name] = ProductDataSourceBinding(
            product=name,
            data_source=FIELD_DATA_SOURCE,
            mode=str(spec["session_mode"]),
            note=str(spec.get("summary", "")),
        )
    return bindings


def product_data_source(product: str) -> ProductDataSourceBinding:
    """Return the field data-source description of a declared Product."""
    try:
        return _product_data_sources()[product]
    except KeyError as exc:
        raise ValueError(f"unknown Product data-source binding {product!r}") from exc


def runtime_contract_manifest() -> dict[str, object]:
    """Export the full runtime contract as machine-checkable plain data."""

    return {
        "schema_version": "lingtu.runtime_interface.v1",
        "frames": runtime_frames_contract(),
        "topics": asdict(TOPICS),
        "core_required_topics": CORE_REQUIRED_TOPICS,
        "frame_links": {name: asdict(link) for name, link in FRAME_LINKS.items()},
        "runtime_data_flow": [asdict(stage) for stage in RUNTIME_DATA_FLOW],
        "resolved_runtime_data_flow": {
            name: [asdict(stage) for stage in resolved_runtime_data_flow(name)] for name in DATA_SOURCE_CONTRACTS
        },
        "lidar_extrinsics": {name: asdict(transform) for name, transform in lidar_extrinsics().items()},
        "message_formats": _message_formats(),
        "topic_formats": {
            topic: topic_formats(topic)
            for topic in (
                set(asdict(TOPICS).values())
                | {alias.source for aliases in ADAPTER_TOPIC_ALIASES.values() for alias in aliases}
            )
        },
        "topic_allowed_frame_ids": runtime_topic_allowed_frame_contract(None),
        "topic_default_frame_ids": runtime_topic_default_frame_ids(None),
        "real_runtime_topic_allowed_frame_ids": runtime_topic_allowed_frame_contract(REAL_RUNTIME_CONTRACT),
        "real_runtime_topic_default_frame_ids": runtime_topic_default_frame_ids(REAL_RUNTIME_CONTRACT),
        "real_runtime_required_topic_frame_ids": REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS,
        "real_runtime_required_endpoint_input_topics": (REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS),
        "runtime_data_flow_topics": {name: runtime_data_flow_topics(name) for name in DATA_SOURCE_CONTRACTS},
        "artifact_formats": {name: asdict(format_spec) for name, format_spec in ARTIFACT_FORMATS.items()},
        "algorithm_interfaces": runtime_algorithm_interface_contract(),
        "runtime_data_flow_stage_algorithm_interfaces": (runtime_stage_algorithm_interface_contract()),
        "data_sources": {name: asdict(source) for name, source in DATA_SOURCE_CONTRACTS.items()},
        "adapter_aliases": {
            name: [asdict(alias) for alias in aliases] for name, aliases in ADAPTER_TOPIC_ALIASES.items()
        },
        "adapter_relays": {
            name: [asdict(alias) for alias in aliases] for name, aliases in ADAPTER_RELAY_ALIASES.items()
        },
        "product_data_sources": {name: asdict(binding) for name, binding in _product_data_sources().items()},
    }
