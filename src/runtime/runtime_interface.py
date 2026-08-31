"""Canonical LingTu runtime interface contract.

This module is the Python source of truth for the runtime boundary shared by
real robot drivers and simulator adapters.  Different endpoints may own
different data sources, but they must normalize into the same frames, canonical
runtime stream tokens, message formats, and algorithm surfaces before entering
LingTu modules.  These stream tokens may be carried by ROS 2 at adapter
boundaries, but they are not the product acceptance interface.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from dataclasses import asdict, dataclass
from typing import Any

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
class RuntimeFrames:
    """Canonical frame names used at LingTu runtime boundaries."""

    map: str = "map"
    odom: str = "odom"
    body: str = "body"
    model_base: str = "base_link"
    lidar: str = "lidar_link"
    real_lidar: str = "livox_frame"
    imu: str = "imu_link"
    camera: str = "camera_link"
    gnss: str = "gnss_antenna"
    simulator_world: str = "world"
    axis_convention: str = "x_forward_y_left_z_up"
    body_aliases: tuple[str, ...] = ("base_link",)
    lidar_aliases: tuple[str, ...] = ("livox_frame",)

    @property
    def body_alias_note(self) -> str:
        """Describe the canonical body-frame alias relationship."""

        return f"{self.model_base} == {self.body}"

    @property
    def map_frame(self) -> str:
        """Return the canonical map frame."""

        return self.map

    @property
    def odom_frame(self) -> str:
        """Return the canonical odometry frame."""

        return self.odom

    @property
    def body_frame(self) -> str:
        """Return the canonical robot body frame."""

        return self.body

    @property
    def model_base_frame(self) -> str:
        """Return the simulator model-base frame alias."""

        return self.model_base

    @property
    def lidar_frame(self) -> str:
        """Return the normalized LiDAR frame."""

        return self.lidar

    @property
    def imu_frame(self) -> str:
        """Return the canonical IMU frame."""

        return self.imu

    @property
    def camera_frame(self) -> str:
        """Return the canonical camera frame."""

        return self.camera

    @property
    def gnss_frame(self) -> str:
        """Return the canonical GNSS antenna frame."""

        return self.gnss

    @property
    def world(self) -> str:
        """Return the simulator world frame."""

        return self.simulator_world

    @property
    def simulator_world_frame(self) -> str:
        """Return the simulator world frame."""

        return self.simulator_world


@dataclass(frozen=True)
class RuntimeTopics:
    """Canonical stream tokens for LingTu runtime modules."""

    raw_lidar_points: str = "/lidar/raw_frame"
    raw_lidar_packet: str = "/lidar/raw_packet"
    raw_imu: str = "/imu/raw"
    lidar_scan: str = "/lidar/raw_frame"
    imu: str = "/imu/raw"

    odom_prior: str = "/slam/odom_prior"
    odometry: str = "/slam/odometry"
    registered_cloud: str = "/slam/registered_cloud"
    map_observation: str = "/slam/map_observation"
    map_cloud: str = "/slam/map_cloud"
    cumulative_map_cloud: str = "/slam/cumulative_map_cloud"
    saved_map_cloud: str = "/slam/saved_map_cloud"
    maps_activation_request: str = "/maps/activation/request"
    maps_activation_ack: str = "/maps/activation/ack"
    maps_state: str = "/maps/state"
    maps_live_cloud: str = "/maps/live_cloud"
    maps_voxel_cloud: str = "/maps/voxel_cloud"
    maps_local_collision: str = "/maps/local_collision"
    maps_accumulated_cloud: str = "/maps/accumulated_cloud"
    maps_occupancy: str = "/maps/occupancy"
    maps_elevation: str = "/maps/elevation"
    maps_esdf: str = "/maps/esdf"
    maps_scene: str = "/maps/scene"
    slam_map_snapshot_request: str = "/slam/map_command"
    slam_map_snapshot_ack: str = "/slam/map_event"
    slam_relocalization_request: str = "/slam/relocalization/request"
    slam_relocalization_response: str = "/slam/relocalization/response"
    driver_odometry: str = "/driver/odometry"
    localization_quality: str = "/slam/localization_quality"
    localization_health: str = "/slam/localization_health"
    state_estimation_at_scan: str = "/slam/state_at_scan"
    exploration_way_point: str = "/exploration/way_point"
    exploration_grid: str = "/nav/exploration_grid"
    exploration_snapshot: str = "/nav/exploration_snapshot"
    exploration_execution_snapshot: str = "/nav/exploration_execution_snapshot"
    exploration_segment_request: str = "/nav/exploration_segment/request"
    exploration_segment_ack: str = "/nav/exploration_segment/ack"
    exploration_segment_status: str = "/nav/exploration_segment/status"
    global_path: str = "/nav/global_path"
    local_path: str = "/nav/local_path"
    far_reach_goal: str = "/nav/far_reach_goal"
    adapter_status: str = "/nav/adapter_status"
    mission_status: str = "/nav/mission_status"
    terrain_map: str = "/nav/terrain_map"
    terrain_map_ext: str = "/nav/terrain_map_ext"
    traversability: str = "/nav/traversability"
    local_traversability: str = "/nav/local_traversability"
    height_rays: str = "/nav/height_rays"
    nav_command_request: str = "/nav/command/request"
    nav_command_ack: str = "/nav/command/ack"
    plan_request: str = "/nav/plan/request"
    plan_result: str = "/nav/plan/result"
    operator_motion_control: str = "/nav/operator_motion/control"
    operator_motion_sample: str = "/nav/operator_motion/sample"
    operator_motion_ack: str = "/nav/operator_motion/ack"
    operator_motion_status: str = "/nav/operator_motion/status"
    nav_goal_status: str = "/nav/goal/status"
    exploration_command: str = "/nav/exploration/command"
    exploration_ack: str = "/nav/exploration/ack"
    exploration_run_event: str = "/nav/exploration/run/event"
    inspection_task_request: str = "/nav/inspection/task/request"
    inspection_task_ack: str = "/nav/inspection/task/ack"
    inspection_status: str = "/nav/inspection/status"
    inspection_task_event: str = "/nav/inspection/task/event"
    inspection_evidence_request: str = "/nav/inspection/evidence/request"
    inspection_evidence_result: str = "/nav/inspection/evidence/result"
    cmd_vel: str = "/nav/cmd_vel"
    nav_way_point: str = "/nav/way_point"
    map_clearing: str = "/nav/map_clearing"
    cloud_clearing: str = "/nav/cloud_clearing"
    added_obstacles: str = "/nav/added_obstacles"
    check_obstacle: str = "/nav/check_obstacle"
    planner_status: str = "/nav/planner_status"
    nav_state: str = "/nav/state"
    local_planner_clear_path: str = "/nav/local_planner/clear_path"
    local_planner_control_hint: str = "/nav/local_planner/control_hint"

    goal_point: str = "/nav/goal_point"
    navigation_boundary: str = "/nav/navigation_boundary"

    semantic_scene_graph: str = "/nav/semantic/scene_graph"
    semantic_instruction: str = "/nav/semantic/instruction"
    semantic_costmap: str = "/nav/costmap"

    reconstruction_semantic_cloud: str = "/nav/reconstruction/semantic_cloud"
    reconstruction_stats: str = "/nav/reconstruction/stats"
    reconstruction_save_ply_service: str = "/nav/reconstruction/save_ply"

    map_command: str = "/nav/map/command"
    map_response: str = "/nav/map/response"
    geofence_command: str = "/nav/geofence/command"
    geofence_response: str = "/nav/geofence/response"
    geofence_alert: str = "/nav/geofence/alert"
    history_query: str = "/nav/history/query"
    history_response: str = "/nav/history/response"

    visualization_detections: str = "/nav/detections"
    robot_marker: str = "/nav/robot_marker"
    building_cloud: str = "/nav/building_cloud"

    camera_color: str = "/camera/color/image_raw"
    camera_depth: str = "/camera/depth/image_raw"
    camera_info: str = "/camera/color/camera_info"
    gnss_fix: str = "/gnss/fix"
    gnss_status: str = "/gnss/status"
    gnss_odom: str = "/gnss/odom"

    @property
    def dog_odometry(self) -> str:
        """Legacy alias for the driver odometry stream.

        New code must use ``driver_odometry``. This property exists for one
        compatibility cycle so old imports do not need a flag day.
        """

        return self.driver_odometry


@dataclass(frozen=True)
class Transform3D:
    """Static child-frame mounting pose expressed in the parent frame.

    For a body->lidar mounting, x/y/z/rpy describe the LiDAR frame pose in
    body coordinates. Applying this transform to a LiDAR-local point returns
    the point in the body frame, matching ROS TF parent/body child/lidar use.
    """

    parent: str
    child: str
    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    @property
    def translation(self) -> tuple[float, float, float]:
        """Return translation components as an XYZ tuple."""

        return (self.x, self.y, self.z)

    @property
    def rotation_xyzw(self) -> tuple[float, float, float, float]:
        """Return the roll-pitch-yaw rotation as an XYZW quaternion."""

        return rpy_to_quaternion_xyzw(self.roll, self.pitch, self.yaw)


@dataclass(frozen=True)
class FrameLinkContract:
    """Runtime TF edge that endpoint adapters must provide or preserve."""

    parent: str
    child: str
    required: bool = True


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
class MessageFormat:
    """Runtime message contract for a topic or topic family."""

    name: str
    frame_role: str
    required_fields: tuple[str, ...] = ()
    note: str = ""


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
class AdapterTopicAlias:
    """Legacy/native endpoint topic mapped into the LingTu runtime contract."""

    source: str
    target: str
    msg_format: str
    scope: str = "adapter_only"
    note: str = ""


@dataclass(frozen=True)
class ProductDataSourceBinding:
    """Which endpoint data source one Field Product is allowed to use."""

    product: str
    data_source: str
    mode: str
    note: str = ""


FRAMES = RuntimeFrames()
TOPICS = RuntimeTopics()

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

FRAME_LINKS = {
    "map_to_odom": FrameLinkContract(
        parent=FRAMES.map,
        child=FRAMES.odom,
        required=True,
    ),
    "odom_to_body": FrameLinkContract(
        parent=FRAMES.odom,
        child=FRAMES.body,
        required=True,
    ),
    "body_to_lidar": FrameLinkContract(
        parent=FRAMES.body,
        child=FRAMES.lidar,
        required=True,
    ),
    "body_to_camera": FrameLinkContract(
        parent=FRAMES.body,
        child=FRAMES.camera,
        required=True,
    ),
    "body_to_gnss": FrameLinkContract(
        parent=FRAMES.body,
        child=FRAMES.gnss,
        required=False,
    ),
}

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
            TOPICS.raw_lidar_points,
            TOPICS.raw_imu,
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

LIDAR_EXTRINSICS = {
    "go2_mid360": Transform3D(
        parent=FRAMES.body,
        child=FRAMES.real_lidar,
        # Unitree official Go2 MID-360 bracket nominal, composed into the
        # LingTu body/base frame from the published internal-IMU mounting pose.
        x=0.16143,
        y=0.0,
        z=0.12262,
        roll=0.0,
        pitch=0.22689280275926285,
        yaw=0.0,
    ),
    "thunder_v4_mid360": Transform3D(
        parent=FRAMES.body,
        child=FRAMES.real_lidar,
        # DOSO Thunder V4 front-nose lidar1 mount.
        x=0.402876074867229,
        y=0.0,
        z=0.0582019450665819,
        roll=-math.pi,
        pitch=-math.pi / 4.0,
        yaw=0.0,
    ),
    "gazebo_proxy": Transform3D(
        parent=FRAMES.body,
        child=FRAMES.lidar,
        x=0.28,
        y=0.0,
        z=0.20,
    ),
    "mujoco_thunder_v3": Transform3D(
        parent=FRAMES.body,
        child=FRAMES.lidar,
        # Compiled thunderv4.xml lidar_site pose in the base_link frame.
        # Keep this synchronized with the compiled-site acceptance test rather
        # than copying a nested MJCF local position by inspection.
        x=-0.30638,
        y=0.0,
        z=0.19417,
    ),
    "portable_mid360_like": Transform3D(
        parent=FRAMES.body,
        child=FRAMES.lidar,
        x=0.0,
        y=0.0,
        z=0.20,
    ),
}

MESSAGE_FORMATS = {
    "tf_message": MessageFormat(
        name="tf_message",
        frame_role="per_transform",
        required_fields=("transforms",),
        note="Native frame-tree update; each transform carries its own parent and child frames.",
    ),
    "raw_livox_custom": MessageFormat(
        name="raw_livox_custom",
        frame_role="lidar_or_body_raw_fastlio_input",
        required_fields=("offset_time", "x", "y", "z", "reflectivity", "tag", "line"),
        note="Scan-level Livox frame for Fast-LIO2 lidar_type=1.",
    ),
    "raw_timed_pointcloud2": MessageFormat(
        name="raw_timed_pointcloud2",
        frame_role="lidar_or_body_raw_fastlio_input",
        required_fields=("x", "y", "z", "intensity", "time", "ring"),
        note="Timed XYZI cloud for Fast-LIO2 lidar_type=2.",
    ),
    "registered_cloud": MessageFormat(
        name="registered_cloud",
        frame_role=FRAMES.body,
        required_fields=("x", "y", "z"),
        note="Robot/body-frame current scan after source normalization.",
    ),
    "map_observation": MessageFormat(
        name="map_observation",
        frame_role=FRAMES.map,
        required_fields=(
            "header",
            "observation_sequence",
            "reset_epoch",
            "sensor_frame",
            "map_sensor",
            "sensor_origin",
            "scan",
            "pose_confidence",
            "localization_quality",
            "pose_state",
            "pose_reason",
        ),
        note=(
            "Accepted incremental scan with its exact same-timestamp map<-sensor "
            "transform. Consumers reject old epochs and non-increasing sequences."
        ),
    ),
    "map_cloud": MessageFormat(
        name="map_cloud",
        frame_role=f"{FRAMES.map}_or_{FRAMES.odom}",
        required_fields=("x", "y", "z"),
        note="World/local-map cloud; never body-relative.",
    ),
    "lingtu.dds.Image": MessageFormat(
        name="lingtu.dds.Image",
        frame_role=FRAMES.camera,
        required_fields=("header", "height", "width", "encoding", "step", "data"),
        note="Native DDS camera image payload; ROS Image is compatibility only.",
    ),
    "lingtu.dds.CameraInfo": MessageFormat(
        name="lingtu.dds.CameraInfo",
        frame_role=FRAMES.camera,
        required_fields=("header", "height", "width", "depth_scale", "k", "p"),
        note="Native DDS camera calibration payload; ROS CameraInfo is compatibility only.",
    ),
    "lingtu.dds.GnssFix": MessageFormat(
        name="lingtu.dds.GnssFix",
        frame_role=FRAMES.gnss,
        required_fields=(
            "header",
            "latitude",
            "longitude",
            "altitude",
            "fix_type",
            "position_covariance",
        ),
        note="Native DDS GNSS WGS84 fix; ROS NavSatFix is compatibility only.",
    ),
    "lingtu.dds.GnssStatus": MessageFormat(
        name="lingtu.dds.GnssStatus",
        frame_role=FRAMES.gnss,
        required_fields=("header", "device", "fix_type", "link_ok", "rtk"),
        note="Native DDS GNSS receiver health; diagnostics are compatibility only.",
    ),
    "lingtu.dds.Imu": MessageFormat(
        name="lingtu.dds.Imu",
        frame_role="imu",
        required_fields=(
            "header",
            "orientation",
            "angular_velocity",
            "linear_acceleration",
        ),
        note="Native DDS IMU payload; ROS Imu is compatibility only.",
    ),
    "odometry": MessageFormat(
        name="odometry",
        frame_role=f"{FRAMES.odom}->{FRAMES.body}",
        note="Continuous localization pose normalized before LingTu modules.",
    ),
    "state_estimation_at_scan": MessageFormat(
        name="state_estimation_at_scan",
        frame_role=f"{FRAMES.odom}->{FRAMES.body}",
        note="Scan-synchronized odometry from external simulation adapters.",
    ),
    "cmd_vel": MessageFormat(
        name="cmd_vel",
        frame_role=FRAMES.body,
        required_fields=(
            "header",
            "host_boot_id",
            "producer_boot_id",
            "output_seq",
            "source_wall_ns",
            "twist",
        ),
        note=(
            "Identity-bound final post-safety logical velocity envelope. "
            "Only the field driver may forward it to Brainstem; simulation "
            "adapters may explicitly project TwistStamped."
        ),
    ),
    "nav_command_request": MessageFormat(
        name="nav_command_request",
        frame_role="request_dependent",
        required_fields=("client_id", "task_id", "request_id", "kind"),
        note=(
            "Typed C++ field command envelope. task_id is stable across delivery "
            "attempts identified by request_id."
        ),
    ),
    "nav_command_ack": MessageFormat(
        name="nav_command_ack",
        frame_role=FRAMES.map,
        required_fields=("task_id", "request_id", "kind", "accepted", "reason"),
        note="Endpoint request-attempt acceptance; never a terminal task result.",
    ),
    "plan_request": MessageFormat(
        name="plan_request",
        frame_role=FRAMES.map,
        required_fields=("request_id", "goal"),
        note="Read-only native planning request; it does not start a navigation task.",
    ),
    "plan_result": MessageFormat(
        name="plan_result",
        frame_role=FRAMES.map,
        required_fields=(
            "request_id",
            "feasible",
            "start_valid",
            "reason",
            "elapsed_ms",
            "planner",
            "start",
            "goal",
            "path",
        ),
        note="Request-correlated native path preview result.",
    ),
    "operator_motion_control": MessageFormat(
        name="operator_motion_control",
        frame_role="metadata",
        required_fields=(
            "source_id",
            "source_epoch",
            "source_sequence",
            "request_id",
            "action",
            "lease_ttl_ms",
            "reason",
        ),
        note=(
            "Low-rate native operator motion authority control. Adapters may claim, "
            "release, or hold; the native endpoint remains the authority owner."
        ),
    ),
    "operator_motion_sample": MessageFormat(
        name="operator_motion_sample",
        frame_role=FRAMES.body,
        required_fields=(
            "source_id",
            "source_epoch",
            "source_sequence",
            "request_id",
            "deadman",
            "velocity",
            "freshness_budget_ms",
            "source_stamp_ns",
        ),
        note=(
            "High-rate latest-only body-frame operator velocity intent. It is not "
            "final velocity until native authority, obstacle, traversability, and "
            "final safety checks publish /nav/cmd_vel."
        ),
    ),
    "operator_motion_ack": MessageFormat(
        name="operator_motion_ack",
        frame_role="metadata",
        required_fields=(
            "source_id",
            "source_epoch",
            "source_sequence",
            "request_id",
            "action",
            "accepted",
            "reason",
            "accepted_sequence",
            "final_output_sequence",
        ),
        note=(
            "Native business ACK for operator motion Claim, Hold, or Release only. "
            "Sample is best-effort intent; status carries last_sample/admitted/final sequence evidence."
        ),
    ),
    "operator_motion_status": MessageFormat(
        name="operator_motion_status",
        frame_role=FRAMES.map,
        required_fields=(
            "active_source_id",
            "active_source_epoch",
            "has_active_authority",
            "holding",
            "has_active_sample",
            "last_sample_sequence",
            "admitted_sequence",
            "final_output_sequence",
            "authority_reason",
            "input_gate_reason",
            "teleop_output",
            "final_cmd_vel",
        ),
        note=(
            "Native operator motion DDS evidence stream. No in-repo Python DDS reader "
            "is declared today; Gateway readiness consumes the native JSON status mirror, "
            "while external diagnostics may subscribe directly. Header is map/status-frame; "
            "teleop_output and final_cmd_vel are body-frame twists."
        ),
    ),
    "nav_goal_status": MessageFormat(
        name="nav_goal_status",
        frame_role=FRAMES.map,
        required_fields=(
            "header",
            "boot_id",
            "event_sequence",
            "task_id",
            "request_id",
            "state",
            "goal_epoch",
            "reason",
        ),
        note=(
            "Native navigation goal lifecycle status with a map-frame Header. "
            "Host APIs expose event_sequence as sequence."
        ),
    ),
    "navigation_state": MessageFormat(
        name="navigation_state",
        frame_role=FRAMES.map,
        required_fields=(
            "header",
            "boot_id",
            "state_sequence",
            "control_mode",
            "lifecycle_state",
            "active_task_id",
            "active_request_id",
            "goal_epoch",
            "map_id",
            "map_content_epoch",
            "planning_state",
            "execution_state",
            "recovery_state",
            "progress",
            "authority",
            "hold_reason",
            "failure_code",
        ),
        note=(
            "Compact authoritative navd lifecycle state. Host APIs expose "
            "state_sequence as sequence; diagnostics are a separate contract."
        ),
    ),
    "exploration_command": MessageFormat(
        name="exploration_command",
        frame_role=FRAMES.map,
        required_fields=(
            "request_id",
            "exploration_run_id",
            "kind",
            "product_session_id",
            "has_directed_target",
            "directed_target_x",
            "directed_target_y",
            "directed_target_ttl_s",
            "reason",
        ),
        note=(
            "Typed lifecycle or directed-target request for one caller-owned exploration run. "
            "exploration_run_id is a canonical uppercase ULID and is distinct from request_id. "
            "Kinds 5 and 6 set and clear a directed map-frame target preference; the set "
            "payload carries has_directed_target, directed_target_x/y, and directed_target_ttl_s. "
            "It never authorizes a static-boundary path."
        ),
    ),
    "exploration_ack": MessageFormat(
        name="exploration_ack",
        frame_role=FRAMES.map,
        required_fields=(
            "request_id",
            "exploration_run_id",
            "kind",
            "accepted",
            "duplicate",
            "reason",
            "product_session_id",
            "intent_revision",
        ),
        note=(
            "Native exploration admission ACK with endpoint-owned duplicate replay truth; "
            "transport delivery alone is not acceptance or lifecycle truth. intent_revision "
            "identifies the accepted directed-target intent."
        ),
    ),
    "exploration_run_event": MessageFormat(
        name="exploration_run_event",
        frame_role=FRAMES.map,
        required_fields=(
            "timestamp_s",
            "frame_id",
            "boot_id",
            "event_sequence",
            "kind",
            "exploration_run_id",
            "start_request_id",
            "command_request_id",
            "product_session_id",
            "state",
            "route",
            "map_id",
            "map_content_epoch",
            "reason",
            "motion_stop_confirmed",
            "motion_stop_reason",
        ),
        note=(
            "Ordered native Explore run fact. Terminal states are published only "
            "after native motion-stop confirmation."
        ),
    ),
    "exploration_segment_request": MessageFormat(
        name="exploration_segment_request",
        frame_role=FRAMES.map,
        required_fields=(
            "request_id",
            "kind",
            "session_id",
            "reset_epoch",
            "minimum_generation",
            "target",
            "reason",
        ),
        note=(
            "Live exploration request for navd to select and validate an observed-free "
            "rolling-map prefix. It is separate from generic saved-map navigation goals."
        ),
    ),
    "exploration_segment_ack": MessageFormat(
        name="exploration_segment_ack",
        frame_role=FRAMES.map,
        required_fields=(
            "request_id",
            "kind",
            "accepted",
            "session_id",
            "reset_epoch",
            "generation",
            "live",
            "reason",
        ),
        note="navd business acceptance for an identity-bound rolling-map segment.",
    ),
    "exploration_segment_status": MessageFormat(
        name="exploration_segment_status",
        frame_role=FRAMES.map,
        required_fields=(
            "request_id",
            "state",
            "session_id",
            "reset_epoch",
            "generation",
            "live",
            "reason",
        ),
        note="navd lifecycle state for an identity-bound rolling-map segment.",
    ),
    "inspection_task_request": MessageFormat(
        name="inspection_task_request",
        frame_role=FRAMES.map,
        required_fields=("task_id", "request_id", "kind", "route_id", "route_revision"),
        note="Product inspection lifecycle command with distinct task and request identities.",
    ),
    "inspection_task_ack": MessageFormat(
        name="inspection_task_ack",
        frame_role=FRAMES.map,
        required_fields=("task_id", "request_id", "kind", "accepted", "reason", "run_id"),
        note="Native inspection task admission result preserving caller task identity.",
    ),
    "inspection_status": MessageFormat(
        name="inspection_status",
        frame_role=FRAMES.map,
        required_fields=("run_id", "route_id", "state", "point_index", "point_count"),
        note="Live status from the native C++ multi-point inspection executor.",
    ),
    "inspection_task_event": MessageFormat(
        name="inspection_task_event",
        frame_role=FRAMES.map,
        required_fields=(
            "boot_id",
            "event_sequence",
            "kind",
            "task_id",
            "request_id",
            "command_request_id",
            "state",
            "map_id",
            "map_content_epoch",
            "route_id",
            "route_revision",
            "point_index",
            "point_count",
            "loop_index",
            "retry_count",
            "point_id",
            "action",
            "action_request_id",
            "evidence_id",
            "reason",
        ),
        note=(
            "Ordered native inspection task fact. boot_id and event_sequence detect "
            "endpoint restart and delivery gaps."
        ),
    ),
    "inspection_evidence_request": MessageFormat(
        name="inspection_evidence_request",
        frame_role=FRAMES.map,
        required_fields=(
            "request_id",
            "run_id",
            "route_id",
            "revision",
            "map_id",
            "map_content_epoch",
            "point_index",
            "point_id",
            "action",
            "deadline_s",
        ),
        note="Native inspection action request consumed by the evidence worker.",
    ),
    "inspection_evidence_result": MessageFormat(
        name="inspection_evidence_result",
        frame_role=FRAMES.map,
        required_fields=(
            "request_id",
            "evidence_id",
            "persisted",
            "reason",
            "analysis_verdict",
        ),
        note="Durable evidence result returned to the native inspection executor.",
    ),
    "height_rays": MessageFormat(
        name="height_rays",
        frame_role=FRAMES.body,
        required_fields=("heights", "points_body", "points_world", "valid_mask"),
        note="Fixed-pattern terrain-ray observation in body frame; separate from LiDAR point clouds.",
    ),
    "traversability": MessageFormat(
        name="traversability",
        frame_role=FRAMES.map,
        note=(
            "Raw cell-level terrain and obstacle risk consumed by native navigation. Values "
            "are not robot-footprint inflated; LocalPlanner, recovery, and the final safety "
            "gate apply the configured rectangular footprint exactly once. Host Module "
            "payloads are dict projections; the field wire is typed lingtu.dds.OccupancyGrid "
            "with uint8 risk values in the inclusive range 0..100. The native traversability "
            "endpoint is the sole field writer; Gateway and Web are read-only consumers."
        ),
    ),
    "local_traversability": MessageFormat(
        name="local_traversability",
        frame_role=FRAMES.odom,
        note=(
            "Rolling odom-frame raw cell-level traversability grid consumed only by the "
            "native local planner. It is not robot-footprint inflated; native motion "
            "consumers apply their rectangular footprint exactly once. Latest-only "
            "control-risk input for native local planning; it is not map persistence and "
            "has no Gateway/Web projection."
        ),
    ),
    "maps_state": MessageFormat(
        name="maps_state",
        frame_role=FRAMES.map,
        required_fields=(
            "producer_boot_id",
            "running",
            "live",
            "reset_epoch",
            "observation_sequence",
            "generation",
            "dds_write_failures",
            "dds_unhealthy_writers",
            "required_publications_ready",
            "current_generation_published",
            "realtime_clouds_published_generation",
            "map_layers_published_generation",
            "scene_published_generation",
            "voxel_snapshot_omitted_cells",
            "voxel_capacity_rejections",
            "accumulated_capacity_rejections",
            "capacity_limited",
            "dds_scene_oversize_rejections",
        ),
        note=(
            "Native mapd process, input, engine, DDS-output health, bounded "
            "snapshot, and per-channel publication progress."
        ),
    ),
    "map_activation_request": MessageFormat(
        name="map_activation_request",
        frame_role="metadata",
        required_fields=(
            "request_id",
            "operation",
            "target",
            "previous",
            "caller",
            "reason",
        ),
        note="Exact native saved-map STAGE, RESTORE, or VERIFY request.",
    ),
    "map_activation_ack": MessageFormat(
        name="map_activation_ack",
        frame_role="metadata",
        required_fields=(
            "request_id",
            "operation",
            "accepted",
            "message",
            "changed",
            "target",
            "previous",
            "active",
            "producer_boot_id",
        ),
        note="Authoritative request-correlated result from native mapd.",
    ),
    "maps_cloud_layer": MessageFormat(
        name="maps_cloud_layer",
        frame_role=FRAMES.map,
        required_fields=(
            "header",
            "layer",
            "cloud",
            "reset_epoch",
            "observation_sequence",
            "generation",
            "live",
        ),
        note="Bounded native mapd point-cloud layer for live, voxel, or accumulated display.",
    ),
    "maps_grid": MessageFormat(
        name="maps_grid",
        frame_role=FRAMES.map,
        required_fields=(
            "header",
            "layer",
            "info",
            "data",
            "reset_epoch",
            "observation_sequence",
            "generation",
            "live",
        ),
        note="Native mapd occupancy, elevation, or ESDF grid layer.",
    ),
    "maps_scene": MessageFormat(
        name="maps_scene",
        frame_role=FRAMES.map,
        required_fields=(
            "header",
            "producer_boot_id",
            "reset_epoch",
            "observation_sequence",
            "generation",
            "live",
            "live_cloud",
            "voxel_cloud",
            "accumulated_cloud",
            "occupancy",
            "elevation",
            "esdf",
        ),
        note="Atomic bounded scene snapshot published by native mapd for Gateway and diagnostics.",
    ),
    "lingtu.dds.Bool": MessageFormat(
        name="lingtu.dds.Bool",
        frame_role="unframed_control_flag",
        required_fields=("data",),
        note=(
            "Native DDS boolean control flag. ROS/compat adapters mirror it as "
            "std_msgs/msg/Bool instead of making the canonical runtime topic a "
            "ROS topic."
        ),
    ),
    "lingtu.dds.SlamMapSnapshotRequest": MessageFormat(
        name="lingtu.dds.SlamMapSnapshotRequest",
        frame_role="map_save_control",
        required_fields=(
            "request_id",
            "map_id",
            "product_session_id",
            "output_path",
            "save_patches",
        ),
        note="Typed DDS request from mapd to the native SLAM map owner.",
    ),
    "lingtu.dds.SlamMapSnapshotAck": MessageFormat(
        name="lingtu.dds.SlamMapSnapshotAck",
        frame_role=FRAMES.map,
        required_fields=(
            "request_id",
            "map_id",
            "success",
            "message",
            "output_path",
            "runtime_instance_id",
            "product_session_id",
            "reset_epoch",
            "observation_sequence",
            "captured_at_ns",
            "frame_id",
            "point_count",
            "state",
            "healthy",
            "health_message",
        ),
        note="Typed DDS acknowledgement for one native SLAM map snapshot.",
    ),
    "lingtu.dds.RelocalizationRequest": MessageFormat(
        name="lingtu.dds.RelocalizationRequest",
        frame_role=f"{FRAMES.map}_or_{FRAMES.odom}",
        required_fields=("command", "map_path", "guess"),
        note=(
            "Typed DDS request for native SLAM relocalization. ROS compatibility "
            "mirrors it as JSON instead of treating the DDS type as a ROS message."
        ),
    ),
    "lingtu.dds.RelocalizationResponse": MessageFormat(
        name="lingtu.dds.RelocalizationResponse",
        frame_role=f"{FRAMES.map}_or_{FRAMES.odom}",
        required_fields=("request_id", "success", "state", "quality"),
        note=(
            "Typed DDS response for native SLAM relocalization. ROS compatibility "
            "mirrors it as JSON instead of treating the DDS type as a ROS message."
        ),
    ),
    "local_planner_control_hint": MessageFormat(
        name="local_planner_control_hint",
        frame_role="planner_status",
        note="Local planner advisory output for path follower speed/stop behavior.",
    ),
    "exploration_snapshot": MessageFormat(
        name="exploration_snapshot",
        frame_role=FRAMES.map,
        required_fields=(
            "header",
            "info",
            "data",
            "session_id",
            "map_id",
            "map_content_epoch",
            "reset_epoch",
            "generation",
            "live",
        ),
        note="Strict trinary rolling occupancy snapshot with restart-safe map identity.",
    ),
    "exploration_execution_snapshot": MessageFormat(
        name="exploration_execution_snapshot",
        frame_role=FRAMES.map,
        required_fields=(
            "header",
            "info",
            "occupancy",
            "terrain_cost",
            "session_id",
            "reset_epoch",
            "generation",
            "live",
            "terrain_risk_stamp",
            "terrain_risk_ready",
        ),
        note=(
            "Atomic rolling occupancy and terrain-risk execution input. Missing, "
            "stale, or inconsistent terrain risk blocks segment execution."
        ),
    ),
}

TOPIC_FORMATS = {
    "/tf": ("tf_message",),
    "/tf_static": ("tf_message",),
    TOPICS.raw_lidar_points: ("raw_livox_custom", "raw_timed_pointcloud2"),
    TOPICS.raw_lidar_packet: ("raw_livox_custom",),
    TOPICS.raw_imu: ("lingtu.dds.Imu",),
    TOPICS.odom_prior: ("odometry",),
    TOPICS.driver_odometry: ("odometry",),
    TOPICS.odometry: ("odometry",),
    TOPICS.state_estimation_at_scan: ("state_estimation_at_scan",),
    TOPICS.registered_cloud: ("registered_cloud",),
    TOPICS.map_observation: ("map_observation",),
    TOPICS.map_cloud: ("map_cloud",),
    TOPICS.camera_color: ("lingtu.dds.Image",),
    TOPICS.camera_depth: ("lingtu.dds.Image",),
    TOPICS.camera_info: ("lingtu.dds.CameraInfo",),
    TOPICS.gnss_fix: ("lingtu.dds.GnssFix",),
    TOPICS.gnss_status: ("lingtu.dds.GnssStatus",),
    TOPICS.gnss_odom: ("odometry",),
    TOPICS.cumulative_map_cloud: ("map_cloud",),
    TOPICS.saved_map_cloud: ("map_cloud",),
    TOPICS.maps_activation_request: ("map_activation_request",),
    TOPICS.maps_activation_ack: ("map_activation_ack",),
    TOPICS.maps_state: ("maps_state",),
    TOPICS.maps_live_cloud: ("maps_cloud_layer",),
    TOPICS.maps_voxel_cloud: ("maps_cloud_layer",),
    TOPICS.maps_accumulated_cloud: ("maps_cloud_layer",),
    TOPICS.maps_occupancy: ("maps_grid",),
    TOPICS.maps_elevation: ("maps_grid",),
    TOPICS.maps_esdf: ("maps_grid",),
    TOPICS.maps_scene: ("maps_scene",),
    TOPICS.slam_map_snapshot_request: ("lingtu.dds.SlamMapSnapshotRequest",),
    TOPICS.slam_map_snapshot_ack: ("lingtu.dds.SlamMapSnapshotAck",),
    TOPICS.slam_relocalization_request: ("lingtu.dds.RelocalizationRequest",),
    TOPICS.slam_relocalization_response: ("lingtu.dds.RelocalizationResponse",),
    TOPICS.exploration_grid: ("nav_msgs/msg/OccupancyGrid",),
    TOPICS.exploration_snapshot: ("exploration_snapshot",),
    TOPICS.exploration_execution_snapshot: ("exploration_execution_snapshot",),
    TOPICS.localization_quality: ("std_msgs/msg/Float32",),
    TOPICS.localization_health: ("std_msgs/msg/String",),
    TOPICS.global_path: ("nav_msgs/msg/Path",),
    TOPICS.local_path: ("nav_msgs/msg/Path",),
    TOPICS.terrain_map: ("map_cloud",),
    TOPICS.terrain_map_ext: ("map_cloud",),
    TOPICS.traversability: ("traversability", "nav_msgs/msg/OccupancyGrid"),
    TOPICS.local_traversability: ("local_traversability",),
    TOPICS.height_rays: ("height_rays",),
    TOPICS.nav_command_request: ("nav_command_request",),
    TOPICS.nav_command_ack: ("nav_command_ack",),
    TOPICS.plan_request: ("plan_request",),
    TOPICS.plan_result: ("plan_result",),
    TOPICS.operator_motion_control: ("operator_motion_control",),
    TOPICS.operator_motion_sample: ("operator_motion_sample",),
    TOPICS.operator_motion_ack: ("operator_motion_ack",),
    TOPICS.operator_motion_status: ("operator_motion_status",),
    TOPICS.nav_goal_status: ("nav_goal_status",),
    TOPICS.nav_state: ("navigation_state",),
    TOPICS.exploration_command: ("exploration_command",),
    TOPICS.exploration_ack: ("exploration_ack",),
    TOPICS.exploration_run_event: ("exploration_run_event",),
    TOPICS.exploration_segment_request: ("exploration_segment_request",),
    TOPICS.exploration_segment_ack: ("exploration_segment_ack",),
    TOPICS.exploration_segment_status: ("exploration_segment_status",),
    TOPICS.inspection_task_request: ("inspection_task_request",),
    TOPICS.inspection_task_ack: ("inspection_task_ack",),
    TOPICS.inspection_status: ("inspection_status",),
    TOPICS.inspection_task_event: ("inspection_task_event",),
    TOPICS.inspection_evidence_request: ("inspection_evidence_request",),
    TOPICS.inspection_evidence_result: ("inspection_evidence_result",),
    TOPICS.cmd_vel: ("cmd_vel",),
    TOPICS.map_clearing: ("lingtu.dds.Bool",),
    TOPICS.cloud_clearing: ("lingtu.dds.Bool",),
    TOPICS.added_obstacles: ("sensor_msgs/msg/PointCloud2",),
    TOPICS.check_obstacle: ("std_msgs/msg/Bool",),
    TOPICS.planner_status: ("std_msgs/msg/String",),
    TOPICS.local_planner_clear_path: ("std_msgs/msg/Bool",),
    TOPICS.local_planner_control_hint: ("local_planner_control_hint",),
    TOPICS.navigation_boundary: ("geometry_msgs/msg/PolygonStamped",),
    TOPICS.goal_point: ("geometry_msgs/msg/PointStamped",),
    TOPICS.semantic_instruction: ("std_msgs/msg/String",),
    TOPICS.exploration_way_point: ("geometry_msgs/msg/PointStamped",),
    TOPICS.nav_way_point: (
        "geometry_msgs/msg/PoseStamped",
        "geometry_msgs/msg/PointStamped",
    ),
    "/livox/lidar": ("raw_livox_custom",),
    "/livox/imu": ("sensor_msgs/msg/Imu",),
    "/imu/data": ("sensor_msgs/msg/Imu",),
    "/lidar/scan": ("raw_livox_custom",),
    "/model/thunder/odometry": ("odometry",),
    "/lingtu/gazebo/raw/lidar_points": ("sensor_msgs/msg/PointCloud2",),
    "/lingtu/gazebo/raw/lidar_scan": ("sensor_msgs/msg/LaserScan",),
    "/lingtu/gazebo/cmd_vel": ("geometry_msgs/msg/Twist",),
    "/state_estimation": ("odometry",),
    "/state_estimation_at_scan": ("state_estimation_at_scan",),
    "/registered_scan": ("map_cloud",),
    "/terrain_map": ("map_cloud",),
    "/terrain_map_ext": ("map_cloud",),
    "/way_point": ("geometry_msgs/msg/PointStamped",),
    "/cmd_vel": ("geometry_msgs/msg/Twist",),
    "/Odometry": ("odometry",),
    "/cloud_registered": ("registered_cloud",),
    "/cloud_map": ("map_cloud",),
    "/localization_quality": ("std_msgs/msg/Float32",),
    "/map_clearing": ("std_msgs/msg/Bool",),
    "/cloud_clearing": ("std_msgs/msg/Bool",),
    "/path": ("nav_msgs/msg/Path",),
    "/speed": ("std_msgs/msg/Float32",),
    "/stop": ("std_msgs/msg/Bool",),
    "/slow_down": ("std_msgs/msg/Float32",),
    "/navigation_boundary": ("geometry_msgs/msg/PolygonStamped",),
    "/added_obstacles": ("sensor_msgs/msg/PointCloud2",),
    "/check_obstacle": ("std_msgs/msg/Bool",),
    "/planner_status": ("std_msgs/msg/String",),
}


TOPIC_ALLOWED_FRAME_IDS = {
    TOPICS.lidar_scan: (FRAMES.lidar,),
    TOPICS.raw_lidar_packet: (FRAMES.lidar,),
    TOPICS.imu: (FRAMES.imu,),
    TOPICS.odom_prior: (FRAMES.odom,),
    TOPICS.driver_odometry: (FRAMES.odom,),
    TOPICS.odometry: (FRAMES.odom, FRAMES.map),
    TOPICS.state_estimation_at_scan: (FRAMES.odom,),
    TOPICS.registered_cloud: (FRAMES.body,),
    TOPICS.map_observation: (FRAMES.map,),
    TOPICS.map_cloud: (FRAMES.map, FRAMES.odom),
    TOPICS.cumulative_map_cloud: (FRAMES.map, FRAMES.odom),
    TOPICS.saved_map_cloud: (FRAMES.map, FRAMES.odom),
    TOPICS.maps_activation_request: (),
    TOPICS.maps_activation_ack: (),
    TOPICS.maps_state: (FRAMES.map,),
    TOPICS.maps_live_cloud: (FRAMES.map,),
    TOPICS.maps_voxel_cloud: (FRAMES.map, FRAMES.odom),
    TOPICS.maps_accumulated_cloud: (FRAMES.map,),
    TOPICS.maps_occupancy: (FRAMES.map, FRAMES.odom),
    TOPICS.maps_elevation: (FRAMES.map, FRAMES.odom),
    TOPICS.maps_esdf: (FRAMES.map, FRAMES.odom),
    TOPICS.maps_scene: (FRAMES.map, FRAMES.odom),
    TOPICS.gnss_fix: (FRAMES.gnss,),
    TOPICS.gnss_status: (FRAMES.gnss,),
    TOPICS.gnss_odom: (FRAMES.map, FRAMES.odom),
    TOPICS.exploration_grid: (FRAMES.map, FRAMES.odom),
    TOPICS.exploration_snapshot: (FRAMES.map,),
    TOPICS.exploration_execution_snapshot: (FRAMES.map,),
    TOPICS.terrain_map: (FRAMES.map, FRAMES.odom),
    TOPICS.terrain_map_ext: (FRAMES.map, FRAMES.odom),
    TOPICS.traversability: (FRAMES.map,),
    TOPICS.local_traversability: (FRAMES.odom,),
    TOPICS.height_rays: (FRAMES.body,),
    TOPICS.nav_command_request: (FRAMES.map, FRAMES.body),
    TOPICS.nav_command_ack: (FRAMES.map,),
    TOPICS.plan_request: (FRAMES.map,),
    TOPICS.plan_result: (FRAMES.map,),
    TOPICS.operator_motion_control: (),
    TOPICS.operator_motion_sample: (FRAMES.body,),
    TOPICS.operator_motion_ack: (),
    TOPICS.operator_motion_status: (FRAMES.map,),
    TOPICS.nav_goal_status: (FRAMES.map,),
    TOPICS.nav_state: (FRAMES.map,),
    TOPICS.exploration_command: (FRAMES.map,),
    TOPICS.exploration_ack: (FRAMES.map,),
    TOPICS.exploration_run_event: (FRAMES.map,),
    TOPICS.exploration_segment_request: (FRAMES.map,),
    TOPICS.exploration_segment_ack: (FRAMES.map,),
    TOPICS.exploration_segment_status: (FRAMES.map,),
    TOPICS.inspection_task_request: (FRAMES.map,),
    TOPICS.inspection_task_ack: (FRAMES.map,),
    TOPICS.inspection_status: (FRAMES.map,),
    TOPICS.inspection_task_event: (FRAMES.map,),
    TOPICS.inspection_evidence_request: (FRAMES.map,),
    TOPICS.inspection_evidence_result: (FRAMES.map,),
    TOPICS.global_path: (FRAMES.map, FRAMES.odom),
    TOPICS.local_path: (FRAMES.map, FRAMES.odom, FRAMES.body),
    TOPICS.exploration_way_point: (FRAMES.map, FRAMES.odom),
    TOPICS.nav_way_point: (FRAMES.map, FRAMES.odom),
    TOPICS.cmd_vel: (FRAMES.body,),
}

REAL_RUNTIME_TOPIC_ALLOWED_FRAME_IDS = {
    **TOPIC_ALLOWED_FRAME_IDS,
    TOPICS.map_cloud: (FRAMES.map,),
    TOPICS.maps_state: (FRAMES.map,),
    TOPICS.maps_live_cloud: (FRAMES.map,),
    TOPICS.maps_voxel_cloud: (FRAMES.map,),
    TOPICS.maps_accumulated_cloud: (FRAMES.map,),
    TOPICS.maps_occupancy: (FRAMES.map,),
    TOPICS.maps_elevation: (FRAMES.map,),
    TOPICS.maps_esdf: (FRAMES.map,),
    TOPICS.maps_scene: (FRAMES.map,),
    TOPICS.global_path: (FRAMES.map,),
}

REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS = (
    TOPICS.lidar_scan,
    TOPICS.imu,
    TOPICS.odometry,
    TOPICS.registered_cloud,
    TOPICS.map_cloud,
    TOPICS.global_path,
    TOPICS.local_path,
    TOPICS.cmd_vel,
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
        inputs=(TOPICS.raw_lidar_points, TOPICS.raw_imu),
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
        normalized_outputs=(TOPICS.raw_lidar_points, TOPICS.raw_imu),
        command_sink="mujoco_velocity_adapter",
        source_outputs=(TOPICS.raw_lidar_points, TOPICS.raw_imu),
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
        if (
            stage.name in PRODUCT_SCOPED_RUNTIME_DATA_FLOW_STAGE_NAMES
            and stage.name not in product_stage_names
        ):
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
        output_topics = frozenset(
            token for token in stage.outputs if token.startswith("/")
        )
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
        product_stage_names=frozenset(
            product_runtime_data_flow_stage_names(required_topics)
        ),
    )


ADAPTER_TOPIC_ALIASES = {
    "livox_driver": (
        AdapterTopicAlias(
            source="/livox/lidar",
            target=TOPICS.lidar_scan,
            msg_format="raw_livox_custom",
            note="Livox driver output normalized for LingTu SLAM input.",
        ),
        AdapterTopicAlias(
            source="/livox/imu",
            target=TOPICS.imu,
            msg_format="sensor_msgs/msg/Imu",
            note="Livox IMU output normalized for LingTu SLAM input.",
        ),
    ),
    "fastlio2": (
        AdapterTopicAlias(
            source="/cloud_registered",
            target=TOPICS.registered_cloud,
            msg_format="registered_cloud",
            note="Fast-LIO2 current scan in body frame.",
        ),
        AdapterTopicAlias(
            source="/cloud_map",
            target=TOPICS.map_cloud,
            msg_format="map_cloud",
            note="Fast-LIO2 local/world map cloud.",
        ),
        AdapterTopicAlias(
            source="/Odometry",
            target=TOPICS.odometry,
            msg_format="odometry",
            note="Fast-LIO2 odometry normalized to odom->body.",
        ),
        AdapterTopicAlias(
            source="/imu/data",
            target=TOPICS.imu,
            msg_format="sensor_msgs/msg/Imu",
            note="Canonical IMU input for Fast-LIO2 launch/service paths.",
        ),
        AdapterTopicAlias(
            source="/lidar/scan",
            target=TOPICS.lidar_scan,
            msg_format="raw_livox_custom",
            note="Canonical LiDAR input for Fast-LIO2 launch/service paths.",
        ),
    ),
    "tare": (
        AdapterTopicAlias(
            source="/registered_scan",
            target=TOPICS.map_cloud,
            msg_format="map_cloud",
            note="TARE/CMU registered scan is a world/map cloud, not body scan.",
        ),
        AdapterTopicAlias(
            source="/terrain_map",
            target=TOPICS.terrain_map,
            msg_format="map_cloud",
        ),
        AdapterTopicAlias(
            source="/terrain_map_ext",
            target=TOPICS.terrain_map_ext,
            msg_format="map_cloud",
        ),
        AdapterTopicAlias(
            source="/state_estimation",
            target=TOPICS.odometry,
            msg_format="odometry",
        ),
        AdapterTopicAlias(
            source="/state_estimation_at_scan",
            target=TOPICS.odometry,
            msg_format="odometry",
        ),
        AdapterTopicAlias(
            source="/way_point",
            target=TOPICS.exploration_way_point,
            msg_format="geometry_msgs/msg/PointStamped",
        ),
    ),
    "terrain_analysis": (
        AdapterTopicAlias(source="/Odometry", target=TOPICS.odometry, msg_format="odometry"),
        AdapterTopicAlias(source="/cloud_map", target=TOPICS.map_cloud, msg_format="map_cloud"),
        AdapterTopicAlias(source="/map_clearing", target=TOPICS.map_clearing, msg_format="std_msgs/msg/Bool"),
        AdapterTopicAlias(source="/terrain_map", target=TOPICS.terrain_map, msg_format="map_cloud"),
    ),
    "terrain_analysis_ext": (
        AdapterTopicAlias(source="/Odometry", target=TOPICS.odometry, msg_format="odometry"),
        AdapterTopicAlias(source="/cloud_map", target=TOPICS.map_cloud, msg_format="map_cloud"),
        AdapterTopicAlias(source="/cloud_clearing", target=TOPICS.cloud_clearing, msg_format="std_msgs/msg/Bool"),
        AdapterTopicAlias(source="/terrain_map", target=TOPICS.terrain_map, msg_format="map_cloud"),
        AdapterTopicAlias(source="/terrain_map_ext", target=TOPICS.terrain_map_ext, msg_format="map_cloud"),
    ),
    "local_planner": (
        AdapterTopicAlias(source="/Odometry", target=TOPICS.odometry, msg_format="odometry"),
        AdapterTopicAlias(source="/cloud_map", target=TOPICS.map_cloud, msg_format="map_cloud"),
        AdapterTopicAlias(source="/terrain_map", target=TOPICS.terrain_map, msg_format="map_cloud"),
        AdapterTopicAlias(source="/terrain_map_ext", target=TOPICS.terrain_map_ext, msg_format="map_cloud"),
        AdapterTopicAlias(
            source="/way_point",
            target=TOPICS.nav_way_point,
            msg_format="geometry_msgs/msg/PointStamped",
        ),
        AdapterTopicAlias(source="/path", target=TOPICS.local_path, msg_format="nav_msgs/msg/Path"),
        AdapterTopicAlias(
            source="/navigation_boundary",
            target=TOPICS.navigation_boundary,
            msg_format="geometry_msgs/msg/PolygonStamped",
        ),
        AdapterTopicAlias(
            source="/added_obstacles",
            target=TOPICS.added_obstacles,
            msg_format="sensor_msgs/msg/PointCloud2",
        ),
        AdapterTopicAlias(source="/check_obstacle", target=TOPICS.check_obstacle, msg_format="std_msgs/msg/Bool"),
    ),
    "path_follower": (
        AdapterTopicAlias(source="/Odometry", target=TOPICS.odometry, msg_format="odometry"),
        AdapterTopicAlias(source="/path", target=TOPICS.local_path, msg_format="nav_msgs/msg/Path"),
        AdapterTopicAlias(source="/cmd_vel", target=TOPICS.cmd_vel, msg_format="cmd_vel"),
        AdapterTopicAlias(source="/planner_status", target=TOPICS.planner_status, msg_format="std_msgs/msg/String"),
    ),
}

ADAPTER_RELAY_ALIASES = {}

PRODUCT_DATA_SOURCE_BINDINGS = {
    "teleop": ProductDataSourceBinding(
        product="teleop",
        data_source=FIELD_DATA_SOURCE,
        mode="real_robot_remote_control",
        note="Operator command only; no autonomous navigation decision chain.",
    ),
    "teleop_avoid": ProductDataSourceBinding(
        product="teleop_avoid",
        data_source=FIELD_DATA_SOURCE,
        mode="real_robot_remote_control_with_obstacle_avoidance",
        note="Operator command with live SLAM, obstacle, and traversability gates; no saved map or global planning.",
    ),
    "map": ProductDataSourceBinding(
        product="map",
        data_source=FIELD_DATA_SOURCE,
        mode="real_robot_mapping",
    ),
    "tracking": ProductDataSourceBinding(
        product="tracking",
        data_source=FIELD_DATA_SOURCE,
        mode="real_robot_tracking",
        note="Follow a selected RGB-D person through VisualServo map goals and native navigation.",
    ),
    "nav": ProductDataSourceBinding(
        product="nav",
        data_source=FIELD_DATA_SOURCE,
        mode="real_robot_saved_map_navigation",
    ),
    "inspection": ProductDataSourceBinding(
        product="inspection",
        data_source=FIELD_DATA_SOURCE,
        mode="real_robot_inspection",
        note="Typed multi-point inspection route execution over saved-map navigation.",
    ),
    "explore": ProductDataSourceBinding(
        product="explore",
        data_source=FIELD_DATA_SOURCE,
        mode="real_robot_exploration",
    ),
}


def adapter_aliases(surface: str) -> tuple[AdapterTopicAlias, ...]:
    """Return adapter-only legacy/native aliases for one endpoint surface."""

    try:
        return ADAPTER_TOPIC_ALIASES[surface]
    except KeyError as exc:
        available = ", ".join(sorted(ADAPTER_TOPIC_ALIASES))
        raise ValueError(f"unknown adapter alias surface {surface!r}; available: {available}") from exc


def adapter_remappings(surface: str) -> dict[str, str]:
    """Return source->target remappings for a native launch/service surface."""

    return {alias.source: alias.target for alias in adapter_aliases(surface)}


def adapter_source_for_target(surface: str, target: str) -> str:
    """Return the native source topic or service remapped to one target."""

    for alias in adapter_aliases(surface):
        if alias.target == target:
            return alias.source
    available = ", ".join(alias.target for alias in adapter_aliases(surface))
    raise ValueError(f"surface {surface!r} has no adapter alias targeting {target!r}; available targets: {available}")


def adapter_relay_aliases(surface: str) -> tuple[AdapterTopicAlias, ...]:
    """Return bidirectional relay aliases for simulation bridge surfaces."""

    try:
        return ADAPTER_RELAY_ALIASES[surface]
    except KeyError as exc:
        available = ", ".join(sorted(ADAPTER_RELAY_ALIASES))
        raise ValueError(f"unknown adapter relay surface {surface!r}; available: {available}") from exc


def topic_formats(topic: str) -> tuple[str, ...]:
    """Return the declared message format names or ROS types for a topic."""

    try:
        return TOPIC_FORMATS[topic]
    except KeyError as exc:
        raise ValueError(f"topic {topic!r} has no declared runtime format") from exc


def topic_allowed_frame_ids(topic: str) -> tuple[str, ...]:
    """Return the general allowed frame_ids for a runtime topic."""

    try:
        return TOPIC_ALLOWED_FRAME_IDS[topic]
    except KeyError as exc:
        raise ValueError(f"topic {topic!r} has no declared frame_id contract") from exc


def runtime_topic_default_frame_id(runtime_contract: str | None, topic: str) -> str:
    """Return the first declared frame_id for a topic in one runtime contract."""

    frames = runtime_topic_allowed_frame_ids(runtime_contract).get(topic)
    if not frames:
        raise ValueError(f"topic {topic!r} has no declared runtime frame_id contract")
    return frames[0]


def runtime_topic_default_frame_ids(runtime_contract: str | None) -> dict[str, str]:
    """Return the default frame_id for every framed topic in one runtime contract."""

    return {topic: frames[0] for topic, frames in runtime_topic_allowed_frame_ids(runtime_contract).items() if frames}


def runtime_frames_contract() -> dict[str, Any]:
    """Return canonical runtime frames as JSON-ready contract data."""

    return normalize_runtime_frames_contract(asdict(FRAMES))


def normalize_runtime_frames_contract(
    frames: Mapping[str, Any] | None,
) -> dict[str, Any]:
    """Return JSON-ready runtime frame contract data."""

    if not isinstance(frames, Mapping):
        return {}
    return {str(key): list(value) if isinstance(value, tuple) else value for key, value in frames.items()}


def runtime_topic_default_frame_contract(
    runtime_contract: str | None,
) -> dict[str, str]:
    """Return JSON-ready default frame_id contract for one runtime."""

    return dict(runtime_topic_default_frame_ids(runtime_contract))


def runtime_topic_allowed_frame_contract(
    runtime_contract: str | None,
) -> dict[str, list[str]]:
    """Return JSON-ready allowed frame_id contract for one runtime."""

    return {topic: list(frames) for topic, frames in runtime_topic_allowed_frame_ids(runtime_contract).items()}


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


def map_frame_id() -> str:
    """Return the canonical fixed map frame used at runtime boundaries."""

    return FRAMES.map


def odom_frame_id() -> str:
    """Return the canonical odometry frame used at runtime boundaries."""

    return FRAMES.odom


def body_frame_id() -> str:
    """Return the canonical body frame used at runtime boundaries."""

    return FRAMES.body


def lidar_frame_id() -> str:
    """Return the canonical normalized LiDAR frame used at runtime boundaries."""

    return FRAMES.lidar


def real_lidar_frame_id() -> str:
    """Return the physical Livox frame before runtime normalization."""

    return FRAMES.real_lidar


def camera_frame_id() -> str:
    """Return the canonical camera frame used at runtime boundaries."""

    return FRAMES.camera


def gnss_frame_id() -> str:
    """Return the canonical GNSS antenna frame used at runtime boundaries."""

    return FRAMES.gnss


def topic_default_frame_id(topic: str) -> str:
    """Return the default frame_id for a topic in the general runtime contract."""

    return runtime_topic_default_frame_id(None, topic)


def simulator_world_frame_id() -> str:
    """Return the simulator fixed-world frame used at runtime boundaries."""

    return FRAMES.simulator_world


def runtime_topic_allowed_frame_ids(runtime_contract: str | None) -> dict[str, tuple[str, ...]]:
    """Return the topic frame_id contract for one resolved runtime contract."""

    runtime_contract = canonical_data_source_name(runtime_contract)
    if runtime_contract == REAL_RUNTIME_CONTRACT:
        return dict(REAL_RUNTIME_TOPIC_ALLOWED_FRAME_IDS)
    return dict(TOPIC_ALLOWED_FRAME_IDS)


def normalize_frame_id(frame_id: str | None) -> str | None:
    """Return a canonical frame_id string without a leading slash."""

    if frame_id is None:
        return None
    normalized = str(frame_id).strip().lstrip("/")
    return normalized or None


def dedupe_frame_ids(frame_ids: tuple[str | None, ...]) -> tuple[str, ...]:
    """Return normalized frame_ids in first-seen order."""

    values: list[str] = []
    seen: set[str] = set()
    for frame_id in frame_ids:
        normalized = normalize_frame_id(frame_id)
        if normalized is None or normalized in seen:
            continue
        seen.add(normalized)
        values.append(normalized)
    return tuple(values)


def frame_id_aliases(frame_id: str | None) -> tuple[str, ...]:
    """Return normalized frame_id plus accepted runtime aliases."""

    normalized = normalize_frame_id(frame_id)
    if normalized is None:
        return ()
    aliases: list[str | None] = [normalized]
    if normalized == FRAMES.body:
        aliases.extend(FRAMES.body_aliases)
    if normalized == FRAMES.lidar:
        aliases.extend(FRAMES.lidar_aliases)
    return dedupe_frame_ids(tuple(aliases))


def expand_frame_id_aliases(frame_ids: tuple[str | None, ...]) -> tuple[str, ...]:
    """Expand canonical frame_ids to their accepted runtime aliases."""

    expanded: list[str | None] = []
    for frame_id in frame_ids:
        expanded.extend(frame_id_aliases(frame_id))
    return dedupe_frame_ids(tuple(expanded))


def runtime_topic_expected_frame_ids(
    runtime_contract: str | None,
    topic: str,
    *additional_frame_ids: str | None,
) -> tuple[str, ...]:
    """Return normalized frame_ids accepted for one topic in one runtime.

    Callers may prepend local context such as the active planning frame. This
    keeps topic-specific contract frames and local planner context in one
    canonical order for diagnostics.
    """

    contract_frames = runtime_topic_allowed_frame_ids(runtime_contract).get(topic, ())
    return dedupe_frame_ids((*additional_frame_ids, *contract_frames))


def runtime_fixed_path_frame_ids(
    *additional_frame_ids: str | None,
) -> tuple[str, ...]:
    """Return path frame_ids that are already expressed in a fixed reference."""

    return dedupe_frame_ids(
        (
            FRAMES.map,
            FRAMES.odom,
            FRAMES.simulator_world,
            *additional_frame_ids,
        )
    )


def runtime_required_topic_frame_ids(runtime_contract: str | None) -> tuple[str, ...]:
    """Return topics whose frame_id evidence is mandatory for one runtime."""

    runtime_contract = canonical_data_source_name(runtime_contract)
    if runtime_contract == REAL_RUNTIME_CONTRACT:
        return REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS
    return ()


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


def product_data_source(product: str) -> ProductDataSourceBinding:
    """Return the declared data-source binding for one Field Product."""

    try:
        return PRODUCT_DATA_SOURCE_BINDINGS[product]
    except KeyError as exc:
        available = ", ".join(sorted(PRODUCT_DATA_SOURCE_BINDINGS))
        raise ValueError(
            f"unknown Product data-source binding {product!r}; available: {available}"
        ) from exc


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
            name: [asdict(stage) for stage in resolved_runtime_data_flow(name)]
            for name in DATA_SOURCE_CONTRACTS
        },
        "lidar_extrinsics": {name: asdict(transform) for name, transform in LIDAR_EXTRINSICS.items()},
        "message_formats": {name: asdict(format_spec) for name, format_spec in MESSAGE_FORMATS.items()},
        "topic_formats": {topic: formats for topic, formats in TOPIC_FORMATS.items()},
        "topic_allowed_frame_ids": runtime_topic_allowed_frame_contract(None),
        "topic_default_frame_ids": runtime_topic_default_frame_ids(None),
        "real_runtime_topic_allowed_frame_ids": runtime_topic_allowed_frame_contract(REAL_RUNTIME_CONTRACT),
        "real_runtime_topic_default_frame_ids": runtime_topic_default_frame_ids(REAL_RUNTIME_CONTRACT),
        "real_runtime_required_topic_frame_ids": REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS,
        "real_runtime_required_endpoint_input_topics": (REAL_RUNTIME_REQUIRED_ENDPOINT_INPUT_TOPICS),
        "runtime_data_flow_topics": {
            name: runtime_data_flow_topics(name) for name in DATA_SOURCE_CONTRACTS
        },
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
        "product_data_sources": {name: asdict(binding) for name, binding in PRODUCT_DATA_SOURCE_BINDINGS.items()},
    }


def rpy_to_quaternion_xyzw(
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    """Convert roll, pitch, yaw radians to a ROS xyzw quaternion."""

    cr = math.cos(float(roll) * 0.5)
    sr = math.sin(float(roll) * 0.5)
    cp = math.cos(float(pitch) * 0.5)
    sp = math.sin(float(pitch) * 0.5)
    cy = math.cos(float(yaw) * 0.5)
    sy = math.sin(float(yaw) * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rotate_xyz_by_quaternion(
    point: tuple[float, float, float],
    quat_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """Rotate one XYZ point by a normalized or unnormalized xyzw quaternion."""

    x, y, z = point
    qx, qy, qz, qw = quat_xyzw
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return point
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def transform_xyz(
    point: tuple[float, float, float],
    transform: Transform3D,
) -> tuple[float, float, float]:
    """Transform a child/local-frame point into the transform parent frame."""

    rx, ry, rz = rotate_xyz_by_quaternion(point, transform.rotation_xyzw)
    return (rx + transform.x, ry + transform.y, rz + transform.z)


def lidar_extrinsic(profile: str) -> Transform3D:
    """Return a named body->LiDAR extrinsic or raise a clear error."""

    try:
        return LIDAR_EXTRINSICS[profile]
    except KeyError as exc:
        available = ", ".join(sorted(LIDAR_EXTRINSICS))
        raise ValueError(f"unknown LiDAR extrinsic profile {profile!r}; available: {available}") from exc
