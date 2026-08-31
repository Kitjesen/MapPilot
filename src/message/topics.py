"""Host-readable metadata for the native DDS topic catalogue.

Python does not define or serialize DDS payloads.  The wire types come from
``idl/messages.idl`` and are generated for the native CycloneDDS clients.
This module only exposes the small amount of metadata needed by Product
resolution, diagnostics, and endpoint-contract validation.
"""

from __future__ import annotations

from dataclasses import dataclass

from runtime.runtime_interface import TOPICS

_DDS_TOPIC_OVERRIDES = {
    TOPICS.camera_color: "rt/camera/color",
    TOPICS.camera_depth: "rt/camera/depth",
    TOPICS.camera_info: "rt/camera/info",
}


@dataclass(frozen=True, slots=True)
class TopicSpec:
    """Logical topic and its native IDL payload type."""

    topic: str
    type_name: str

    @property
    def dds_topic(self) -> str:
        """Return the CycloneDDS wire topic."""
        return dds_topic_name(self.topic)

    @property
    def idl_type(self) -> str:
        """Return the fully qualified IDL type."""
        return f"lingtu.dds.{self.type_name}"

_TYPE_BY_TOPIC = {
    "/tf": "TFMessage",
    "/tf_static": "TFMessage",
    TOPICS.lidar_scan: "LivoxFrame",
    TOPICS.raw_lidar_packet: "LivoxFrame",
    TOPICS.imu: "Imu",
    TOPICS.raw_imu: "Imu",
    TOPICS.odom_prior: "Odometry",
    TOPICS.driver_odometry: "Odometry",
    TOPICS.odometry: "Odometry",
    TOPICS.state_estimation_at_scan: "Odometry",
    TOPICS.registered_cloud: "PointCloud2",
    TOPICS.map_observation: "MapObservation",
    TOPICS.map_cloud: "PointCloud2",
    TOPICS.cumulative_map_cloud: "PointCloud2",
    TOPICS.saved_map_cloud: "PointCloud2",
    TOPICS.maps_activation_request: "MapActivationRequest",
    TOPICS.maps_activation_ack: "MapActivationAck",
    TOPICS.maps_state: "MapRuntimeState",
    TOPICS.maps_live_cloud: "MapCloudLayer",
    TOPICS.maps_voxel_cloud: "MapCloudLayer",
    TOPICS.maps_local_collision: "MapCollisionLayer",
    TOPICS.maps_accumulated_cloud: "MapCloudLayer",
    TOPICS.maps_occupancy: "MapGrid",
    TOPICS.maps_elevation: "MapGrid",
    TOPICS.maps_esdf: "MapGrid",
    TOPICS.maps_scene: "MapScene",
    TOPICS.camera_color: "Image",
    TOPICS.camera_depth: "Image",
    TOPICS.camera_info: "CameraInfo",
    TOPICS.gnss_fix: "GnssFix",
    TOPICS.gnss_status: "GnssStatus",
    TOPICS.gnss_odom: "Odometry",
    TOPICS.slam_map_snapshot_request: "SlamMapSnapshotRequest",
    TOPICS.slam_map_snapshot_ack: "SlamMapSnapshotAck",
    TOPICS.slam_relocalization_request: "RelocalizationRequest",
    TOPICS.slam_relocalization_response: "RelocalizationResponse",
    TOPICS.localization_quality: "Float32",
    TOPICS.localization_health: "Text",
    TOPICS.global_path: "Path",
    TOPICS.local_path: "Path",
    TOPICS.nav_way_point: "PoseStamped",
    TOPICS.nav_command_request: "NavigationCommandRequest",
    TOPICS.nav_command_ack: "NavigationCommandAck",
    TOPICS.operator_motion_control: "OperatorMotionControl",
    TOPICS.operator_motion_sample: "OperatorMotionSample",
    TOPICS.operator_motion_ack: "OperatorMotionAck",
    TOPICS.operator_motion_status: "OperatorMotionStatus",
    TOPICS.nav_goal_status: "NavigationGoalStatus",
    TOPICS.nav_state: "NavigationState",
    TOPICS.exploration_command: "ExplorationCommandRequest",
    TOPICS.exploration_ack: "ExplorationCommandAck",
    TOPICS.exploration_run_event: "ExplorationRunEvent",
    TOPICS.exploration_segment_request: "ExplorationSegmentRequest",
    TOPICS.exploration_segment_ack: "ExplorationSegmentAck",
    TOPICS.exploration_segment_status: "ExplorationSegmentStatus",
    TOPICS.inspection_task_request: "InspectionTaskRequest",
    TOPICS.inspection_task_ack: "InspectionTaskAck",
    TOPICS.inspection_evidence_request: "InspectionEvidenceRequest",
    TOPICS.inspection_evidence_result: "InspectionEvidenceResult",
    TOPICS.inspection_status: "InspectionStatus",
    TOPICS.inspection_task_event: "InspectionTaskEvent",
    TOPICS.semantic_instruction: "Text",
    TOPICS.traversability: "OccupancyGrid",
    TOPICS.plan_request: "PlanRequest",
    TOPICS.plan_result: "PlanResult",
    TOPICS.local_traversability: "OccupancyGrid",
    TOPICS.terrain_map: "PointCloud2",
    TOPICS.terrain_map_ext: "PointCloud2",
    TOPICS.map_clearing: "Bool",
    TOPICS.cloud_clearing: "Bool",
    TOPICS.cmd_vel: "FinalVelocityCommand",
    TOPICS.exploration_grid: "OccupancyGrid",
    TOPICS.exploration_execution_snapshot: "ExplorationExecutionGrid",
    TOPICS.exploration_snapshot: "ExplorationGrid",
}

TOPIC_SPECS = {
    topic: TopicSpec(topic, type_name)
    for topic, type_name in _TYPE_BY_TOPIC.items()
}


def topic_spec(topic: str) -> TopicSpec | None:
    """Return native metadata for one logical topic."""
    return TOPIC_SPECS.get(str(topic))


def dds_topic_name(topic: str) -> str:
    """Map a logical topic to the native CycloneDDS topic name."""
    name = str(topic)
    if name in _DDS_TOPIC_OVERRIDES:
        return _DDS_TOPIC_OVERRIDES[name]
    return "rt" + name if name.startswith("/") else name
