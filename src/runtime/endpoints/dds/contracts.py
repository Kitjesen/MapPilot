"""Typed DDS endpoint contract for Thunder field runtime."""

from __future__ import annotations

from dataclasses import dataclass

from message.dds import TopicSpec, dds_topic_name, topic_spec
from runtime.runtime_interface import (
    THUNDER_FIELD_RUNTIME_CONTRACT,
    TOPICS,
    runtime_topic_allowed_frame_ids,
)
from runtime.tf import TF_STATIC_TOPIC, TF_TOPIC

THUNDER_FIELD_DDS_CONTRACT_NAME = "thunder_field_dds_v1"
DDS_PAYLOAD_FORMAT = "dds.idl.v1"


@dataclass(frozen=True)
class DDSEndpointBinding:
    """A canonical LingTu topic bound to one typed DDS topic."""

    topic: str
    channel: str
    direction: str
    schema: str
    type_name: str
    import_path: str
    idl_type: str
    cpp_type: str
    payload_format: str = DDS_PAYLOAD_FORMAT
    frame_ids: tuple[str, ...] = ()
    required: bool = True
    note: str = ""

    @property
    def typed(self) -> bool:
        return True


@dataclass(frozen=True)
class DDSEndpointContract:
    """Named typed DDS endpoint contract for a runtime endpoint."""

    name: str
    runtime_contract: str
    transport: str
    bindings: tuple[DDSEndpointBinding, ...]

    @property
    def topics(self) -> tuple[str, ...]:
        return tuple(binding.topic for binding in self.bindings)

    @property
    def required_topics(self) -> tuple[str, ...]:
        return tuple(binding.topic for binding in self.bindings if binding.required)

    def binding_for_topic(self, topic: str) -> DDSEndpointBinding:
        for binding in self.bindings:
            if binding.topic == topic:
                return binding
        raise KeyError(f"DDS contract '{self.name}' has no binding for topic '{topic}'")


def _topic_spec(topic: str) -> TopicSpec:
    spec = topic_spec(topic)
    if spec is None:
        raise KeyError(f"topic {topic!r} has no typed DDS topic spec")
    return spec


def _binding(
    topic: str,
    *,
    direction: str,
    schema: str,
    required: bool = True,
    note: str = "",
) -> DDSEndpointBinding:
    spec = _topic_spec(topic)
    return DDSEndpointBinding(
        topic=topic,
        channel=dds_topic_name(topic, typed=True),
        direction=direction,
        schema=schema,
        type_name=spec.type_name,
        import_path=spec.import_path,
        idl_type=spec.idl_type,
        cpp_type=spec.cpp_type,
        frame_ids=tuple(runtime_topic_allowed_frame_ids(THUNDER_FIELD_RUNTIME_CONTRACT).get(topic, ())),
        required=required,
        note=note,
    )


THUNDER_FIELD_DDS_CONTRACT = DDSEndpointContract(
    name=THUNDER_FIELD_DDS_CONTRACT_NAME,
    runtime_contract=THUNDER_FIELD_RUNTIME_CONTRACT,
    transport="dds",
    bindings=(
        _binding(
            TF_TOPIC,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.TFMessage",
            required=False,
            note="Dynamic frame transforms for the native runtime FrameTree.",
        ),
        _binding(
            TF_STATIC_TOPIC,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.TFMessage",
            required=False,
            note="Static frame transforms for sensor mounts and calibration.",
        ),
        _binding(
            TOPICS.lidar_scan,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.LivoxFrame",
            note="Native raw Livox MID-360 frame entering the SLAM/localization boundary.",
        ),
        _binding(
            TOPICS.imu,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.Imu",
            note="Time-aligned IMU stream for the field localization boundary.",
        ),
        _binding(
            TOPICS.odometry,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.Odometry",
        ),
        _binding(
            TOPICS.registered_cloud,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.PointCloud2",
        ),
        _binding(
            TOPICS.map_observation,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.MapObservation",
            note=(
                "One accepted incremental scan with the exact same-timestamp "
                "map-to-sensor pose, origin, reset epoch, and sequence."
            ),
        ),
        _binding(
            TOPICS.map_cloud,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.PointCloud2",
        ),
        _binding(
            TOPICS.saved_map_cloud,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.PointCloud2",
            required=False,
        ),
        _binding(
            TOPICS.maps_state,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.MapRuntimeState",
            note="Latched mapd process, publication, and capacity health state for HostBus readiness.",
        ),
        _binding(
            TOPICS.maps_scene,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.MapScene",
            note="Latest coherent mapd scene consumed by HostBus and Gateway.",
        ),
        _binding(
            TOPICS.localization_health,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.Text",
            required=False,
        ),
        _binding(
            TOPICS.localization_quality,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.Float32",
            required=False,
        ),
        _binding(
            TOPICS.nav_command_request,
            direction="lingtu_to_endpoint",
            schema="lingtu.dds.NavigationCommandRequest",
            note="Typed task/attempt identity plus goal, cancel, or operator velocity payload.",
        ),
        _binding(
            TOPICS.nav_command_ack,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.NavigationCommandAck",
            note="Business-level request-attempt ACK carrying the stable task_id.",
        ),
        _binding(
            TOPICS.operator_motion_control,
            direction="lingtu_to_endpoint",
            schema="lingtu.dds.OperatorMotionControl",
            note="Low-rate claim, release, or hold request for native operator motion authority.",
        ),
        _binding(
            TOPICS.operator_motion_sample,
            direction="lingtu_to_endpoint",
            schema="lingtu.dds.OperatorMotionSample",
            note="High-rate latest-only body-frame operator velocity intent; not final cmd_vel.",
        ),
        _binding(
            TOPICS.operator_motion_ack,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.OperatorMotionAck",
            note="Native operator motion business ACK with accepted and final output sequence evidence.",
        ),
        _binding(
            TOPICS.operator_motion_status,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.OperatorMotionStatus",
            note="Native authority/input-gate/final-output status for operator motion.",
        ),
        _binding(
            TOPICS.nav_goal_status,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.NavigationGoalStatus",
            note=(
                "Asynchronous native lifecycle status for a stable task_id and originating request_id; "
                "HostBus deduplicates its boot_id/sequence replay cursor."
            ),
        ),
        _binding(
            TOPICS.nav_state,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.NavigationState",
            note="Compact authoritative navd lifecycle state consumed by HostBus.",
        ),
        _binding(
            TOPICS.exploration_command,
            direction="lingtu_to_endpoint",
            schema="lingtu.dds.ExplorationCommandRequest",
            note=(
                "Typed lifecycle request or directed map-frame target set/clear "
                "command for the native exploration FSM; set carries has_directed_target, "
                "directed_target_x/y, and directed_target_ttl_s. It is a soft TARE "
                "candidate preference, not static-boundary path authorization."
            ),
        ),
        _binding(
            TOPICS.exploration_ack,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.ExplorationCommandAck",
            note=(
                "Business-level exploration acceptance or rejection for a request_id, "
                "including the accepted directed-target intent_revision."
            ),
        ),
        _binding(
            TOPICS.exploration_segment_request,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.ExplorationSegmentRequest",
            note=(
                "Identity-bound rolling-map segment request from the native "
                "exploration runtime to the native navigation runtime."
            ),
        ),
        _binding(
            TOPICS.exploration_segment_ack,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.ExplorationSegmentAck",
            note="Business acceptance or rejection for the matching exploration segment request.",
        ),
        _binding(
            TOPICS.exploration_segment_status,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.ExplorationSegmentStatus",
            note="Request-correlated lifecycle for the identity-bound exploration segment.",
        ),
        _binding(
            TOPICS.inspection_command,
            direction="lingtu_to_endpoint",
            schema="lingtu.dds.InspectionCommandRequest",
            note="Typed native route start, pause, resume, or cancel request.",
        ),
        _binding(
            TOPICS.inspection_ack,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.InspectionCommandAck",
            note="Business-level inspection command ACK.",
        ),
        _binding(
            TOPICS.inspection_task_request,
            direction="lingtu_to_endpoint",
            schema="lingtu.dds.InspectionTaskRequest",
            note="Task-addressed inspection request with caller-owned task_id.",
        ),
        _binding(
            TOPICS.inspection_task_ack,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.InspectionTaskAck",
            note="Business-level task ACK preserving task_id and request_id.",
        ),
        _binding(
            TOPICS.inspection_status,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.InspectionStatus",
            note="Current native inspection run and point progress.",
        ),
        _binding(
            TOPICS.inspection_task_event,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.InspectionTaskEvent",
            note=(
                "Ordered native inspection task facts. boot_id and event_sequence form "
                "the per-endpoint replay cursor; terminal facts follow native stop evidence."
            ),
        ),
        _binding(
            TOPICS.inspection_evidence_request,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.InspectionEvidenceRequest",
            note="Native inspection action request for the LingTu evidence worker.",
        ),
        _binding(
            TOPICS.inspection_evidence_result,
            direction="lingtu_to_endpoint",
            schema="lingtu.dds.InspectionEvidenceResult",
            note="Persisted evidence result returned to the native inspection executor.",
        ),
        _binding(
            TOPICS.goal_pose,
            direction="lingtu_to_endpoint",
            schema="geometry_msgs/msg/PoseStamped",
            required=False,
            note="Legacy compatibility reader; product clients use nav_command_request.",
        ),
        _binding(
            TOPICS.cancel,
            direction="lingtu_to_endpoint",
            schema="std_msgs/msg/String",
            required=False,
            note="Legacy compatibility reader; product clients use nav_command_request.",
        ),
        _binding(
            TOPICS.teleop_cmd_vel,
            direction="lingtu_to_endpoint",
            schema="geometry_msgs/msg/TwistStamped",
            required=False,
            note="Legacy compatibility reader; product clients use nav_command_request.",
        ),
        _binding(
            TOPICS.traversability,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.OccupancyGrid",
            required=False,
            note="Live terrain traversability risk grid for the native C++ local planner.",
        ),
        _binding(
            TOPICS.exploration_grid,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.OccupancyGrid",
            note="Observed free, occupied, and unknown cells for frontier exploration.",
        ),
        _binding(
            TOPICS.exploration_snapshot,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.ExplorationGrid",
            note="Versioned rolling occupancy with map identity and restart-safe generation.",
        ),
        _binding(
            TOPICS.exploration_execution_snapshot,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.ExplorationExecutionGrid",
            note="Atomic rolling occupancy and terrain-risk input for native navigation execution.",
        ),
        _binding(
            TOPICS.global_path,
            direction="lingtu_to_endpoint",
            schema="nav_msgs/msg/Path",
        ),
        _binding(
            TOPICS.local_path,
            direction="lingtu_to_endpoint",
            schema="nav_msgs/msg/Path",
        ),
        _binding(
            TOPICS.nav_way_point,
            direction="lingtu_to_endpoint",
            schema="geometry_msgs/msg/PoseStamped",
            required=False,
            note="Active navigation waypoint for endpoint visualization and supervision.",
        ),
        _binding(
            TOPICS.cmd_vel,
            direction="lingtu_to_endpoint",
            schema="lingtu.dds.FinalVelocityCommand",
            note=(
                "Native endpoint final body-frame command with boot, process, "
                "sequence, and source-clock freshness metadata."
            ),
        ),
    ),
)

_CONTRACTS: dict[str, DDSEndpointContract] = {
    THUNDER_FIELD_DDS_CONTRACT.name: THUNDER_FIELD_DDS_CONTRACT,
}


def endpoint_contract(name: str) -> DDSEndpointContract:
    """Return a named DDS endpoint contract."""

    try:
        return _CONTRACTS[name]
    except KeyError as exc:
        choices = ", ".join(endpoint_contract_names())
        raise KeyError(f"unknown DDS endpoint contract '{name}' (choices: {choices})") from exc


def endpoint_contract_names() -> tuple[str, ...]:
    """Return known DDS endpoint contract names."""

    return tuple(sorted(_CONTRACTS))


def binding_for_topic(contract_name: str, topic: str) -> DDSEndpointBinding:
    """Return the binding for a canonical topic in a contract."""

    return endpoint_contract(contract_name).binding_for_topic(topic)
