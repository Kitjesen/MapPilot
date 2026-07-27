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
from dataclasses import asdict, dataclass
from typing import Any, Mapping

THUNDER_FIELD_RUNTIME_CONTRACT = "thunder_field"
LEGACY_REAL_RUNTIME_CONTRACT = "real_s100p"
REAL_RUNTIME_CONTRACT = THUNDER_FIELD_RUNTIME_CONTRACT
THUNDER_LITE_RUNTIME_CONTRACT = "thunder_lite_local"
THUNDER_FIELD_EVIDENCE_LABEL = "Thunder field evidence"

DATA_SOURCE_ALIASES = {
    LEGACY_REAL_RUNTIME_CONTRACT: THUNDER_FIELD_RUNTIME_CONTRACT,
}


def canonical_data_source_name(name: str | None) -> str | None:
    """Return the canonical data-source/runtime-contract name."""

    if name is None:
        return None
    value = str(name)
    return DATA_SOURCE_ALIASES.get(value, value)


@dataclass(frozen=True)
class RuntimeFrames:
    """Canonical frame names used at LingTu runtime boundaries."""

    map: str = "map"
    odom: str = "odom"
    body: str = "body"
    model_base: str = "base_link"
    lidar: str = "lidar_link"
    real_lidar: str = "livox_frame"
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
    map_cloud: str = "/slam/map_cloud"
    cumulative_map_cloud: str = "/slam/cumulative_map_cloud"
    saved_map_cloud: str = "/slam/saved_map_cloud"
    maps_voxel_cloud: str = "/maps/voxel_cloud"
    maps_occupancy: str = "/maps/occupancy"
    maps_elevation: str = "/maps/elevation"
    maps_esdf: str = "/maps/esdf"
    maps_traversability: str = "/maps/traversability"
    maps_scene: str = "/maps/scene"
    slam_map_command: str = "/slam/map_command"
    slam_map_event: str = "/slam/map_event"
    slam_relocalization_request: str = "/slam/relocalization/request"
    slam_relocalization_response: str = "/slam/relocalization/response"
    save_map_service: str = "/slam/save_map"
    driver_odometry: str = "/driver/odometry"
    localization_quality: str = "/slam/localization_quality"
    localization_health: str = "/slam/localization_health"
    state_estimation_at_scan: str = "/slam/state_at_scan"
    relocalize_service: str = "/slam/relocalize"
    global_relocalize_service: str = "/slam/global_relocalize"
    global_relocalize_status_service: str = "/slam/global_relocalize_status"
    relocalize_check_service: str = "/slam/relocalize_check"

    exploration_start: str = "/exploration/start"
    exploration_stop: str = "/exploration/stop"
    exploration_way_point: str = "/exploration/way_point"
    exploration_global_path_full: str = "/exploration/global_path_full"
    exploration_global_path: str = "/exploration/global_path"
    exploration_local_path: str = "/exploration/local_path"
    exploration_old_global_path: str = "/exploration/old_global_path"
    exploration_nearest_global_subspace_path: str = "/exploration/to_nearest_global_subspace_path"
    exploration_path: str = "/exploration/path"
    exploration_cmu_local_planner_path: str = "/exploration/cmu_local_planner_path"
    exploration_runtime_breakdown: str = "/exploration/runtime_breakdown"
    exploration_runtime: str = "/exploration/runtime"
    exploration_finish: str = "/exploration/finish"
    exploration_grid: str = "/nav/exploration_grid"
    exploration_snapshot: str = "/nav/exploration_snapshot"
    traversable_frontiers: str = "/nav/traversable_frontiers"
    frontier_candidate: str = "/nav/frontier_candidate"
    exploration_status: str = "/exploration/status"

    global_path: str = "/nav/global_path"
    local_path: str = "/nav/local_path"
    far_reach_goal: str = "/nav/far_reach_goal"
    adapter_status: str = "/nav/adapter_status"
    mission_status: str = "/nav/mission_status"
    terrain_map: str = "/nav/terrain_map"
    terrain_map_ext: str = "/nav/terrain_map_ext"
    traversability: str = "/nav/traversability"
    height_rays: str = "/nav/height_rays"
    teleop_cmd_vel: str = "/nav/teleop_cmd_vel"
    nav_command_request: str = "/nav/command/request"
    nav_command_ack: str = "/nav/command/ack"
    nav_goal_status: str = "/nav/goal/status"
    nav_state: str = "/nav/state"
    exploration_command: str = "/nav/exploration/command"
    exploration_ack: str = "/nav/exploration/ack"
    inspection_command: str = "/nav/inspection/command"
    inspection_ack: str = "/nav/inspection/ack"
    inspection_status: str = "/nav/inspection/status"
    inspection_evidence_request: str = "/nav/inspection/evidence/request"
    inspection_evidence_result: str = "/nav/inspection/evidence/result"
    cmd_vel: str = "/nav/cmd_vel"
    stop: str = "/nav/stop"
    slow_down: str = "/nav/slow_down"
    nav_way_point: str = "/nav/way_point"
    speed: str = "/nav/speed"
    map_clearing: str = "/nav/map_clearing"
    cloud_clearing: str = "/nav/cloud_clearing"
    added_obstacles: str = "/nav/added_obstacles"
    check_obstacle: str = "/nav/check_obstacle"
    planner_status: str = "/nav/planner_status"
    local_planner_clear_path: str = "/nav/local_planner/clear_path"
    local_planner_control_hint: str = "/nav/local_planner/control_hint"

    goal_pose: str = "/nav/goal_pose"
    goal_point: str = "/nav/goal_point"
    patrol_goals: str = "/nav/patrol_goals"
    cancel: str = "/nav/cancel"
    navigation_boundary: str = "/nav/navigation_boundary"

    semantic_detections_3d: str = "/nav/semantic/detections_3d"
    semantic_scene_graph: str = "/nav/semantic/scene_graph"
    semantic_query_service: str = "/nav/semantic/query"
    semantic_instruction: str = "/nav/semantic/instruction"
    semantic_cancel: str = "/nav/semantic/cancel"
    semantic_status: str = "/nav/semantic/status"
    semantic_costmap: str = "/nav/costmap"
    scg_plan_request: str = "/nav/scg/plan_request"
    scg_plan_result: str = "/nav/scg/plan_result"
    voice_response: str = "/nav/voice/response"

    reconstruction_semantic_cloud: str = "/nav/reconstruction/semantic_cloud"
    reconstruction_stats: str = "/nav/reconstruction/stats"
    reconstruction_save_ply_service: str = "/nav/reconstruction/save_ply"

    safety_state: str = "/nav/safety_state"
    execution_eval: str = "/nav/execution_eval"
    dialogue_state: str = "/nav/dialogue_state"

    map_command: str = "/nav/map/command"
    map_response: str = "/nav/map/response"
    poi_command: str = "/nav/poi/command"
    poi_response: str = "/nav/poi/response"
    patrol_command: str = "/nav/patrol/command"
    patrol_response: str = "/nav/patrol/response"
    geofence_command: str = "/nav/geofence/command"
    geofence_response: str = "/nav/geofence/response"
    schedule_command: str = "/nav/schedule/command"
    schedule_response: str = "/nav/schedule/response"
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
    ros_type: str
    frame_role: str
    required_fields: tuple[str, ...] = ()
    note: str = ""


@dataclass(frozen=True)
class DDSWireRevisionContract:
    """Deployment rule for an incompatible revision on retained DDS topics."""

    schema_revision: str
    topics: tuple[str, ...]
    deployment_policy: str
    mixed_versions_supported: bool
    note: str


@dataclass(frozen=True)
class ArtifactFormat:
    """Saved-map artifact contract shared by mapping, relocalization, and PCT."""

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
class ProfileDataSourceBinding:
    """Which endpoint data source a CLI/runtime profile is allowed to use."""

    profile: str
    data_source: str
    mode: str
    note: str = ""


FRAMES = RuntimeFrames()
TOPICS = RuntimeTopics()

NAVIGATION_TASK_IDENTITY_WIRE_CONTRACT = DDSWireRevisionContract(
    schema_revision="lingtu.dds.navigation_task_identity.v2",
    topics=(
        TOPICS.nav_command_request,
        TOPICS.nav_command_ack,
        TOPICS.nav_goal_status,
        TOPICS.nav_state,
    ),
    deployment_policy="cold_switch",
    mixed_versions_supported=False,
    note=(
        "These retained topics use @final IDL structs whose field layout changed. "
        "Stop every publisher and subscriber, deploy one generated schema revision, "
        "then restart them together; rolling or mixed-version operation is unsupported."
    ),
)

CORE_ALGORITHM_ENTRY_TOPICS = (
    TOPICS.odometry,
    TOPICS.registered_cloud,
    TOPICS.map_cloud,
)

CANONICAL_NAV_TOPICS = (
    *CORE_ALGORITHM_ENTRY_TOPICS,
    TOPICS.exploration_grid,
    TOPICS.exploration_snapshot,
    TOPICS.global_path,
    TOPICS.local_path,
    TOPICS.terrain_map,
    TOPICS.terrain_map_ext,
    TOPICS.traversability,
    TOPICS.cmd_vel,
)

CORE_REQUIRED_TOPICS = (
    *CANONICAL_NAV_TOPICS,
    TOPICS.goal_pose,
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
            "module:TraversableFrontierModule.fused_cost",
            "module:TraversableFrontierModule.slope_grid",
            "module:TraversableFrontierModule.esdf_field",
            "module:TraversableFrontierModule.elevation_map",
        ),
        outputs=(
            TOPICS.exploration_way_point,
            TOPICS.goal_pose,
            TOPICS.traversable_frontiers,
            TOPICS.frontier_candidate,
        ),
        owner="maps_frontier_or_tare_adapter",
        frame_role=FRAMES.map,
        map_dependency="live_map_or_occupancy_grid",
        producer="maps_modules_or_exploration_adapter",
        consumers=("global_planning", "gateway", "exploration"),
        frequency="map_layers_1_10hz_frontier_0.5_2hz",
        transport_policy="direct_default_shm_for_dense_maps_dds_for_external_exploration",
    ),
    RuntimeDataFlowStage(
        name="global_planning",
        inputs=(
            TOPICS.odometry,
            TOPICS.map_cloud,
            TOPICS.exploration_grid,
            TOPICS.exploration_way_point,
            TOPICS.goal_pose,
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

LIDAR_EXTRINSICS = {
    "real_mid360": Transform3D(
        parent=FRAMES.body,
        child=FRAMES.real_lidar,
        x=0.402876074867229,
        y=0.0,
        z=0.0582019450665819,
        roll=-3.141592653589793,
        pitch=-0.7853981633974483,
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
        ros_type="lingtu.dds.TFMessage",
        frame_role="per_transform",
        required_fields=("transforms",),
        note="Native frame-tree update; each transform carries its own parent and child frames.",
    ),
    "raw_livox_custom": MessageFormat(
        name="raw_livox_custom",
        ros_type="livox_ros_driver2/msg/CustomMsg",
        frame_role="lidar_or_body_raw_fastlio_input",
        required_fields=("offset_time", "x", "y", "z", "reflectivity", "tag", "line"),
        note="Scan-level Livox frame for Fast-LIO2 lidar_type=1.",
    ),
    "raw_timed_pointcloud2": MessageFormat(
        name="raw_timed_pointcloud2",
        ros_type="sensor_msgs/msg/PointCloud2",
        frame_role="lidar_or_body_raw_fastlio_input",
        required_fields=("x", "y", "z", "intensity", "time", "ring"),
        note="Timed XYZI cloud for Fast-LIO2 lidar_type=2.",
    ),
    "registered_cloud": MessageFormat(
        name="registered_cloud",
        ros_type="sensor_msgs/msg/PointCloud2",
        frame_role=FRAMES.body,
        required_fields=("x", "y", "z"),
        note="Robot/body-frame current scan after source normalization.",
    ),
    "map_cloud": MessageFormat(
        name="map_cloud",
        ros_type="sensor_msgs/msg/PointCloud2",
        frame_role=f"{FRAMES.map}_or_{FRAMES.odom}",
        required_fields=("x", "y", "z"),
        note="World/local-map cloud; never body-relative.",
    ),
    "lingtu.dds.Image": MessageFormat(
        name="lingtu.dds.Image",
        ros_type="sensor_msgs/msg/Image",
        frame_role=FRAMES.camera,
        required_fields=("header", "height", "width", "encoding", "step", "data"),
        note="Native DDS camera image payload; ROS Image is compatibility only.",
    ),
    "lingtu.dds.CameraInfo": MessageFormat(
        name="lingtu.dds.CameraInfo",
        ros_type="sensor_msgs/msg/CameraInfo",
        frame_role=FRAMES.camera,
        required_fields=("header", "height", "width", "depth_scale", "k", "p"),
        note="Native DDS camera calibration payload; ROS CameraInfo is compatibility only.",
    ),
    "lingtu.dds.GnssFix": MessageFormat(
        name="lingtu.dds.GnssFix",
        ros_type="sensor_msgs/msg/NavSatFix",
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
        ros_type="diagnostic_msgs/msg/DiagnosticStatus",
        frame_role=FRAMES.gnss,
        required_fields=("header", "device", "fix_type", "link_ok", "rtk"),
        note="Native DDS GNSS receiver health; diagnostics are compatibility only.",
    ),
    "lingtu.dds.Imu": MessageFormat(
        name="lingtu.dds.Imu",
        ros_type="sensor_msgs/msg/Imu",
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
        ros_type="nav_msgs/msg/Odometry",
        frame_role=f"{FRAMES.odom}->{FRAMES.body}",
        note="Continuous localization pose normalized before LingTu modules.",
    ),
    "state_estimation_at_scan": MessageFormat(
        name="state_estimation_at_scan",
        ros_type="nav_msgs/msg/Odometry",
        frame_role=f"{FRAMES.odom}->{FRAMES.body}",
        note="Scan-synchronized odometry from external simulation adapters.",
    ),
    "cmd_vel": MessageFormat(
        name="cmd_vel",
        ros_type="geometry_msgs/msg/TwistStamped",
        frame_role=FRAMES.body,
        note="Muxed command; endpoint adapters decide whether to relay it.",
    ),
    "teleop_cmd_vel": MessageFormat(
        name="teleop_cmd_vel",
        ros_type="geometry_msgs/msg/TwistStamped",
        frame_role=FRAMES.body,
        note=(
            "Operator velocity request from Web, joystick, SDK, or MCP. "
            "It is not a motor command until the native command arbiter "
            "checks freshness, localization, obstacles, traversability, and "
            "publishes the final command on /nav/cmd_vel."
        ),
    ),
    "nav_command_request": MessageFormat(
        name="nav_command_request",
        ros_type="lingtu.dds.NavigationCommandRequest",
        frame_role="request_dependent",
        required_fields=("client_id", "task_id", "request_id", "kind"),
        note=(
            "Typed C++ field command envelope. GOAL and strict CANCEL carry a stable "
            "task_id across attempts identified by request_id; legacy cancel-current "
            "and non-task control commands may leave task_id empty."
        ),
    ),
    "nav_command_ack": MessageFormat(
        name="nav_command_ack",
        ros_type="lingtu.dds.NavigationCommandAck",
        frame_role=FRAMES.map,
        required_fields=("task_id", "request_id", "kind", "accepted", "reason"),
        note=(
            "Endpoint request-attempt acceptance; never a terminal task result. "
            "task_id mirrors the request and may be empty for legacy non-task commands."
        ),
    ),
    "nav_goal_status": MessageFormat(
        name="nav_goal_status",
        ros_type="lingtu.dds.NavigationGoalStatus",
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
            "Native navigation task lifecycle status with the originating request "
            "attempt identity."
        ),
    ),
    "navigation_state": MessageFormat(
        name="navigation_state",
        ros_type="lingtu.dds.NavigationState",
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
            "map_version",
            "map_hash",
            "planning_state",
            "execution_state",
            "recovery_state",
            "progress",
            "authority",
            "hold_reason",
            "failure_code",
        ),
        note="Compact authoritative native navigation lifecycle state.",
    ),
    "exploration_command": MessageFormat(
        name="exploration_command",
        ros_type="lingtu.dds.ExplorationCommandRequest",
        frame_role=FRAMES.map,
        required_fields=("request_id", "kind", "session_id", "reason"),
        note="Typed start, pause, resume, or stop request for the native exploration FSM.",
    ),
    "exploration_ack": MessageFormat(
        name="exploration_ack",
        ros_type="lingtu.dds.ExplorationCommandAck",
        frame_role=FRAMES.map,
        required_fields=("request_id", "kind", "accepted", "reason", "session_id"),
        note="Native exploration business ACK; transport delivery alone is not acceptance.",
    ),
    "inspection_command": MessageFormat(
        name="inspection_command",
        ros_type="lingtu.dds.InspectionCommandRequest",
        frame_role=FRAMES.map,
        required_fields=("request_id", "kind", "route_id", "route_revision"),
        note="Typed start/pause/resume/cancel command referencing a native C++ route.",
    ),
    "inspection_ack": MessageFormat(
        name="inspection_ack",
        ros_type="lingtu.dds.InspectionCommandAck",
        frame_role=FRAMES.map,
        required_fields=("request_id", "kind", "accepted", "reason", "run_id"),
        note="Native inspection command acceptance or rejection with a map-frame Header.",
    ),
    "inspection_status": MessageFormat(
        name="inspection_status",
        ros_type="lingtu.dds.InspectionStatus",
        frame_role=FRAMES.map,
        required_fields=("run_id", "route_id", "state", "point_index", "point_count"),
        note="Live status from the native C++ multi-point inspection executor.",
    ),
    "inspection_evidence_request": MessageFormat(
        name="inspection_evidence_request",
        ros_type="lingtu.dds.InspectionEvidenceRequest",
        frame_role=FRAMES.map,
        required_fields=(
            "request_id",
            "run_id",
            "route_id",
            "revision",
            "map_id",
            "map_version",
            "point_index",
            "point_id",
            "action",
            "deadline_s",
        ),
        note="Native inspection action request consumed by the evidence worker.",
    ),
    "inspection_evidence_result": MessageFormat(
        name="inspection_evidence_result",
        ros_type="lingtu.dds.InspectionEvidenceResult",
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
        ros_type="application/json",
        frame_role=FRAMES.body,
        required_fields=("heights", "points_body", "points_world", "valid_mask"),
        note="Fixed-pattern terrain-ray observation in body frame; separate from LiDAR point clouds.",
    ),
    "traversability": MessageFormat(
        name="traversability",
        ros_type="application/json",
        frame_role=f"{FRAMES.map}_or_{FRAMES.odom}",
        note=(
            "Terrain risk summary/grid consumed by navigation and local planning. "
            "Module graph payloads are dicts; native DDS carries the grid form as "
            "lingtu.dds.OccupancyGrid on the same runtime topic."
        ),
    ),
    "maps_elevation": MessageFormat(
        name="maps_elevation",
        ros_type="application/json",
        frame_role=f"{FRAMES.map}_or_{FRAMES.odom}",
        required_fields=("grid", "resolution", "origin"),
        note="Map-domain elevation product; Python Module payload is a dict.",
    ),
    "maps_esdf": MessageFormat(
        name="maps_esdf",
        ros_type="application/json",
        frame_role=f"{FRAMES.map}_or_{FRAMES.odom}",
        required_fields=("distance", "resolution", "origin"),
        note="Map-domain ESDF product; Python Module payload is a dict.",
    ),
    "maps_scene": MessageFormat(
        name="maps_scene",
        ros_type="application/json",
        frame_role=f"{FRAMES.map}_or_{FRAMES.odom}",
        required_fields=("schema_version", "layers"),
        note="Layered map visualization/product snapshot for Gateway/Rerun-style consumers.",
    ),
    "lingtu.dds.Bool": MessageFormat(
        name="lingtu.dds.Bool",
        ros_type="std_msgs/msg/Bool",
        frame_role="unframed_control_flag",
        required_fields=("data",),
        note=(
            "Native DDS boolean control flag. ROS/compat adapters mirror it as "
            "std_msgs/msg/Bool instead of making the canonical runtime topic a "
            "ROS topic."
        ),
    ),
    "lingtu.dds.RelocalizationRequest": MessageFormat(
        name="lingtu.dds.RelocalizationRequest",
        ros_type="application/json",
        frame_role=f"{FRAMES.map}_or_{FRAMES.odom}",
        required_fields=("command", "map_path", "guess"),
        note=(
            "Typed DDS request for native SLAM relocalization. ROS compatibility "
            "mirrors it as JSON instead of treating the DDS type as a ROS message."
        ),
    ),
    "lingtu.dds.RelocalizationResponse": MessageFormat(
        name="lingtu.dds.RelocalizationResponse",
        ros_type="application/json",
        frame_role=f"{FRAMES.map}_or_{FRAMES.odom}",
        required_fields=("request_id", "success", "state", "quality"),
        note=(
            "Typed DDS response for native SLAM relocalization. ROS compatibility "
            "mirrors it as JSON instead of treating the DDS type as a ROS message."
        ),
    ),
    "local_planner_control_hint": MessageFormat(
        name="local_planner_control_hint",
        ros_type="application/json",
        frame_role="planner_status",
        note="Local planner advisory output for path follower speed/stop behavior.",
    ),
    "exploration_snapshot": MessageFormat(
        name="exploration_snapshot",
        ros_type="lingtu.dds.ExplorationGrid",
        frame_role=FRAMES.map,
        required_fields=(
            "header",
            "info",
            "data",
            "session_id",
            "map_id",
            "map_version",
            "artifact_hash",
            "reset_epoch",
            "generation",
            "live",
        ),
        note="Strict trinary rolling occupancy snapshot with restart-safe map identity.",
    ),
    "traversable_frontier_candidates": MessageFormat(
        name="traversable_frontier_candidates",
        ros_type="application/json",
        frame_role=FRAMES.map,
        required_fields=(
            "id",
            "centroid_3d",
            "reachable_score",
            "support_type",
            "esdf_clearance_m",
            "semantic_value",
            "evidence_layers",
            "reasons",
        ),
        note="Read-only traversability-scored frontier preview; never a command.",
    ),
}

TOPIC_FORMATS = {
    "/tf": ("tf_message",),
    "/tf_static": ("tf_message",),
    TOPICS.raw_lidar_points: ("raw_livox_custom", "raw_timed_pointcloud2"),
    TOPICS.raw_imu: ("lingtu.dds.Imu",),
    TOPICS.odom_prior: ("odometry",),
    TOPICS.driver_odometry: ("odometry",),
    TOPICS.odometry: ("odometry",),
    TOPICS.state_estimation_at_scan: ("state_estimation_at_scan",),
    TOPICS.registered_cloud: ("registered_cloud",),
    TOPICS.map_cloud: ("map_cloud",),
    TOPICS.camera_color: ("lingtu.dds.Image",),
    TOPICS.camera_depth: ("lingtu.dds.Image",),
    TOPICS.camera_info: ("lingtu.dds.CameraInfo",),
    TOPICS.gnss_fix: ("lingtu.dds.GnssFix",),
    TOPICS.gnss_status: ("lingtu.dds.GnssStatus",),
    TOPICS.gnss_odom: ("odometry",),
    TOPICS.cumulative_map_cloud: ("map_cloud",),
    TOPICS.saved_map_cloud: ("map_cloud",),
    TOPICS.maps_voxel_cloud: ("map_cloud",),
    TOPICS.maps_occupancy: ("nav_msgs/msg/OccupancyGrid",),
    TOPICS.maps_elevation: ("maps_elevation",),
    TOPICS.maps_esdf: ("maps_esdf",),
    TOPICS.maps_traversability: ("traversability",),
    TOPICS.maps_scene: ("maps_scene",),
    TOPICS.slam_map_command: ("std_msgs/msg/String",),
    TOPICS.slam_map_event: ("std_msgs/msg/String",),
    TOPICS.slam_relocalization_request: ("lingtu.dds.RelocalizationRequest",),
    TOPICS.slam_relocalization_response: ("lingtu.dds.RelocalizationResponse",),
    TOPICS.exploration_grid: ("nav_msgs/msg/OccupancyGrid",),
    TOPICS.exploration_snapshot: ("exploration_snapshot",),
    TOPICS.traversable_frontiers: ("traversable_frontier_candidates",),
    TOPICS.frontier_candidate: ("traversable_frontier_candidates",),
    TOPICS.save_map_service: ("service",),
    TOPICS.localization_quality: ("std_msgs/msg/Float32",),
    TOPICS.localization_health: ("std_msgs/msg/String",),
    TOPICS.relocalize_service: ("service",),
    TOPICS.global_relocalize_service: ("service",),
    TOPICS.global_relocalize_status_service: ("service",),
    TOPICS.relocalize_check_service: ("service",),
    TOPICS.global_path: ("nav_msgs/msg/Path",),
    TOPICS.local_path: ("nav_msgs/msg/Path",),
    TOPICS.terrain_map: ("map_cloud",),
    TOPICS.terrain_map_ext: ("map_cloud",),
    TOPICS.traversability: ("traversability", "nav_msgs/msg/OccupancyGrid"),
    TOPICS.height_rays: ("height_rays",),
    TOPICS.teleop_cmd_vel: ("teleop_cmd_vel",),
    TOPICS.nav_command_request: ("nav_command_request",),
    TOPICS.nav_command_ack: ("nav_command_ack",),
    TOPICS.nav_goal_status: ("nav_goal_status",),
    TOPICS.nav_state: ("navigation_state",),
    TOPICS.exploration_command: ("exploration_command",),
    TOPICS.exploration_ack: ("exploration_ack",),
    TOPICS.inspection_command: ("inspection_command",),
    TOPICS.inspection_ack: ("inspection_ack",),
    TOPICS.inspection_status: ("inspection_status",),
    TOPICS.inspection_evidence_request: ("inspection_evidence_request",),
    TOPICS.inspection_evidence_result: ("inspection_evidence_result",),
    TOPICS.cmd_vel: ("cmd_vel",),
    TOPICS.stop: ("std_msgs/msg/Bool",),
    TOPICS.slow_down: ("std_msgs/msg/Float32",),
    TOPICS.speed: ("std_msgs/msg/Float32",),
    TOPICS.map_clearing: ("lingtu.dds.Bool",),
    TOPICS.cloud_clearing: ("lingtu.dds.Bool",),
    TOPICS.added_obstacles: ("sensor_msgs/msg/PointCloud2",),
    TOPICS.check_obstacle: ("std_msgs/msg/Bool",),
    TOPICS.planner_status: ("std_msgs/msg/String",),
    TOPICS.local_planner_clear_path: ("std_msgs/msg/Bool",),
    TOPICS.local_planner_control_hint: ("local_planner_control_hint",),
    TOPICS.navigation_boundary: ("geometry_msgs/msg/PolygonStamped",),
    TOPICS.cancel: ("std_msgs/msg/String",),
    TOPICS.goal_pose: ("geometry_msgs/msg/PoseStamped",),
    TOPICS.goal_point: ("geometry_msgs/msg/PointStamped",),
    TOPICS.semantic_instruction: ("std_msgs/msg/String",),
    TOPICS.exploration_way_point: ("geometry_msgs/msg/PointStamped",),
    TOPICS.exploration_start: ("std_msgs/msg/Bool",),
    TOPICS.exploration_stop: ("std_msgs/msg/Bool",),
    TOPICS.exploration_global_path_full: ("nav_msgs/msg/Path",),
    TOPICS.exploration_global_path: ("nav_msgs/msg/Path",),
    TOPICS.exploration_local_path: ("nav_msgs/msg/Path",),
    TOPICS.exploration_old_global_path: ("nav_msgs/msg/Path",),
    TOPICS.exploration_nearest_global_subspace_path: ("nav_msgs/msg/Path",),
    TOPICS.exploration_path: ("nav_msgs/msg/Path",),
    TOPICS.exploration_cmu_local_planner_path: ("nav_msgs/msg/Path",),
    TOPICS.exploration_runtime_breakdown: ("std_msgs/msg/Float32",),
    TOPICS.exploration_runtime: ("std_msgs/msg/Float32",),
    TOPICS.exploration_finish: ("std_msgs/msg/Bool",),
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


def _ros_types_for_formats(formats: tuple[str, ...]) -> tuple[str, ...]:
    """Resolve LingTu runtime format names into concrete ROS payload types."""

    resolved: list[str] = []
    for format_name in formats:
        spec = MESSAGE_FORMATS.get(format_name)
        ros_type = spec.ros_type if spec is not None else format_name
        if ros_type not in resolved:
            resolved.append(ros_type)
    return tuple(resolved)


TOPIC_ROS_TYPES = {topic: _ros_types_for_formats(formats) for topic, formats in TOPIC_FORMATS.items()}

TOPIC_ALLOWED_FRAME_IDS = {
    TOPICS.lidar_scan: (FRAMES.lidar,),
    TOPICS.imu: (FRAMES.lidar,),
    TOPICS.odom_prior: (FRAMES.odom,),
    TOPICS.driver_odometry: (FRAMES.odom,),
    TOPICS.odometry: (FRAMES.odom, FRAMES.map),
    TOPICS.state_estimation_at_scan: (FRAMES.odom,),
    TOPICS.registered_cloud: (FRAMES.body,),
    TOPICS.map_cloud: (FRAMES.map, FRAMES.odom),
    TOPICS.cumulative_map_cloud: (FRAMES.map, FRAMES.odom),
    TOPICS.saved_map_cloud: (FRAMES.map, FRAMES.odom),
    TOPICS.maps_voxel_cloud: (FRAMES.map, FRAMES.odom),
    TOPICS.maps_occupancy: (FRAMES.map, FRAMES.odom),
    TOPICS.maps_elevation: (FRAMES.map, FRAMES.odom),
    TOPICS.maps_esdf: (FRAMES.map, FRAMES.odom),
    TOPICS.maps_traversability: (FRAMES.map, FRAMES.odom),
    TOPICS.maps_scene: (FRAMES.map, FRAMES.odom),
    TOPICS.gnss_fix: (FRAMES.gnss,),
    TOPICS.gnss_status: (FRAMES.gnss,),
    TOPICS.gnss_odom: (FRAMES.map, FRAMES.odom),
    TOPICS.exploration_grid: (FRAMES.map, FRAMES.odom),
    TOPICS.exploration_snapshot: (FRAMES.map,),
    TOPICS.traversable_frontiers: (FRAMES.map, FRAMES.odom),
    TOPICS.frontier_candidate: (FRAMES.map, FRAMES.odom),
    TOPICS.terrain_map: (FRAMES.map, FRAMES.odom),
    TOPICS.terrain_map_ext: (FRAMES.map, FRAMES.odom),
    TOPICS.traversability: (FRAMES.map, FRAMES.odom),
    TOPICS.height_rays: (FRAMES.body,),
    TOPICS.teleop_cmd_vel: (FRAMES.body,),
    TOPICS.nav_command_request: (FRAMES.map, FRAMES.body),
    TOPICS.nav_command_ack: (FRAMES.map,),
    TOPICS.nav_goal_status: (FRAMES.map,),
    TOPICS.nav_state: (FRAMES.map,),
    TOPICS.exploration_command: (FRAMES.map,),
    TOPICS.exploration_ack: (FRAMES.map,),
    TOPICS.inspection_command: (FRAMES.map,),
    TOPICS.inspection_ack: (FRAMES.map,),
    TOPICS.inspection_status: (FRAMES.map,),
    TOPICS.inspection_evidence_request: (FRAMES.map,),
    TOPICS.inspection_evidence_result: (FRAMES.map,),
    TOPICS.global_path: (FRAMES.map, FRAMES.odom),
    TOPICS.local_path: (FRAMES.map, FRAMES.odom, FRAMES.body),
    TOPICS.exploration_way_point: (FRAMES.map, FRAMES.odom),
    TOPICS.goal_pose: (FRAMES.map, FRAMES.odom),
    TOPICS.nav_way_point: (FRAMES.map, FRAMES.odom),
    TOPICS.cmd_vel: (FRAMES.body,),
}

REAL_RUNTIME_TOPIC_ALLOWED_FRAME_IDS = {
    **TOPIC_ALLOWED_FRAME_IDS,
    TOPICS.map_cloud: (FRAMES.map,),
    TOPICS.maps_voxel_cloud: (FRAMES.map,),
    TOPICS.maps_occupancy: (FRAMES.map,),
    TOPICS.maps_elevation: (FRAMES.map,),
    TOPICS.maps_esdf: (FRAMES.map,),
    TOPICS.maps_traversability: (FRAMES.map,),
    TOPICS.maps_scene: (FRAMES.map,),
    TOPICS.traversable_frontiers: (FRAMES.map,),
    TOPICS.frontier_candidate: (FRAMES.map,),
    TOPICS.global_path: (FRAMES.map,),
}

THUNDER_LITE_TOPIC_ALLOWED_FRAME_IDS = {
    TOPICS.cmd_vel: (FRAMES.body,),
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
            "sha256",
        ),
        note="Saved map point cloud. PCT/relocalization may consume it only with same-source metadata.",
    ),
    "octomap": ArtifactFormat(
        name="octomap",
        path="octomap.ot",
        artifact_type="octomap_full_tree",
        frame_role=FRAMES.map,
        required_fields=("occupancy_tree",),
        required_metadata=("source_map_sha256", "source_profile", "data_source", "frame_id"),
        note="OctoMap full-tree artifact consumed by the OctoPlanner3D headless backend.",
    ),
    "point_cloud": ArtifactFormat(
        name="point_cloud",
        path="map.pcd",
        artifact_type="pcd_xyz_or_xyzi",
        frame_role=FRAMES.map,
        required_fields=("x", "y", "z"),
        required_metadata=("source_profile", "data_source", "frame_id", "point_count", "sha256"),
        note="Point cloud .pcd artifact convertible to OctoMap for OctoPlanner3D.",
    ),
    "occupancy_grid": ArtifactFormat(
        name="occupancy_grid",
        path="occupancy.npz",
        artifact_type="numpy_occupancy_grid",
        frame_role=FRAMES.map,
        required_fields=("grid", "resolution", "origin"),
        required_metadata=("source_map_sha256", "source_profile", "data_source", "frame_id"),
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
        owner="tare_or_frontier",
        map_dependency="live_map_or_occupancy_grid",
    ),
    "wavefront_frontier_exploration": AlgorithmInterface(
        name="wavefront_frontier_exploration",
        inputs=(TOPICS.odometry, TOPICS.exploration_grid),
        outputs=(TOPICS.goal_pose,),
        owner="lingtu_frontier",
        map_dependency="live_occupancy_grid_unknown_free_boundary",
    ),
    "traversable_frontier_preview": AlgorithmInterface(
        name="traversable_frontier_preview",
        inputs=(
            TOPICS.odometry,
            TOPICS.exploration_grid,
            "module:TraversableFrontierModule.fused_cost",
            "module:TraversableFrontierModule.slope_grid",
            "module:TraversableFrontierModule.esdf_field",
            "module:TraversableFrontierModule.elevation_map",
        ),
        outputs=(TOPICS.traversable_frontiers, TOPICS.frontier_candidate),
        owner="lingtu_traversable_frontier",
        map_dependency=("live_occupancy_grid_and_traversability_layers_read_only_preview"),
    ),
    "tare_exploration": AlgorithmInterface(
        name="tare_exploration",
        inputs=(
            TOPICS.odometry,
            "/tf",
            TOPICS.exploration_snapshot,
            TOPICS.exploration_command,
        ),
        outputs=(TOPICS.nav_command_request, TOPICS.exploration_ack),
        owner="native_explore_endpoint",
        map_dependency="identity_versioned_rolling_occupancy_snapshot",
    ),
    "global_planning": AlgorithmInterface(
        name="global_planning",
        inputs=(TOPICS.odometry, TOPICS.map_cloud, TOPICS.exploration_way_point, TOPICS.goal_pose),
        outputs=(TOPICS.global_path, TOPICS.nav_way_point),
        owner="lingtu_navigation",
        map_dependency="planner_specific_octoplanner3d_octomap",
    ),
    "octoplanner3d_global_planning": AlgorithmInterface(
        name="octoplanner3d_global_planning",
        inputs=(
            TOPICS.odometry,
            "artifact:octomap",
            TOPICS.goal_pose,
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
    "map_layers_and_exploration": (
        "exploration_strategy",
        "wavefront_frontier_exploration",
        "traversable_frontier_preview",
        "tare_exploration",
    ),
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
    REAL_RUNTIME_CONTRACT: DataSourceContract(
        name=REAL_RUNTIME_CONTRACT,
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
        lidar_extrinsic_profile="real_mid360",
        slam_source="lingtu_fastlio_or_external_robot_slam",
        localization_source="slam_localizer",
        mapping_source="slam_map_cloud",
    ),
    THUNDER_LITE_RUNTIME_CONTRACT: DataSourceContract(
        name=THUNDER_LITE_RUNTIME_CONTRACT,
        provider="hardware",
        owns=("robot_actuation", "local_module_graph"),
        normalized_outputs=(),
        command_sink="driver",
        source_outputs=(),
        algorithm_entry_outputs=(),
        algorithm_context_outputs=(),
        lidar_extrinsic_profile=None,
        slam_source="none",
        localization_source="none",
        mapping_source="none",
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
    "gazebo_industrial": DataSourceContract(
        name="gazebo_industrial",
        provider="gazebo",
        owns=("world_geometry", "physics", "rendered_lidar", "simulation_actuation"),
        normalized_outputs=(
            TOPICS.odometry,
            TOPICS.state_estimation_at_scan,
            TOPICS.registered_cloud,
            TOPICS.map_cloud,
            TOPICS.exploration_grid,
        ),
        command_sink="/lingtu/gazebo/cmd_vel",
        source_outputs=(
            "/model/thunder/odometry",
            "/lingtu/gazebo/raw/lidar_points",
            "/lingtu/gazebo/raw/lidar_scan",
        ),
        algorithm_entry_outputs=(
            TOPICS.odometry,
            TOPICS.registered_cloud,
            TOPICS.map_cloud,
        ),
        algorithm_context_outputs=(
            TOPICS.state_estimation_at_scan,
            TOPICS.exploration_grid,
        ),
        lidar_extrinsic_profile="gazebo_proxy",
        slam_source="none",
        localization_source="gazebo_sim_odometry",
        mapping_source="gazebo_lidar_derived_map",
    ),
    "cmu_unity_external": DataSourceContract(
        name="cmu_unity_external",
        provider="cmu_unity",
        owns=("world_geometry", "physics", "external_registered_scan", "external_tare"),
        normalized_outputs=(
            TOPICS.odometry,
            TOPICS.state_estimation_at_scan,
            TOPICS.registered_cloud,
            TOPICS.map_cloud,
            TOPICS.terrain_map_ext,
            TOPICS.exploration_way_point,
        ),
        command_sink="/cmd_vel",
        source_outputs=(
            "/state_estimation",
            "/state_estimation_at_scan",
            "/registered_scan",
            "/terrain_map",
            "/terrain_map_ext",
            "/way_point",
        ),
        algorithm_entry_outputs=(
            TOPICS.odometry,
            TOPICS.registered_cloud,
            TOPICS.map_cloud,
        ),
        algorithm_context_outputs=(
            TOPICS.state_estimation_at_scan,
            TOPICS.terrain_map_ext,
            TOPICS.exploration_way_point,
        ),
        lidar_extrinsic_profile=None,
        slam_source="external_cmu_registered_scan",
        localization_source="cmu_state_estimation",
        mapping_source="cmu_registered_scan_and_terrain_map_ext",
    ),
}


def _dedupe_runtime_tokens(tokens: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(dict.fromkeys(token for token in tokens if token))


def _data_source_contract(data_source: str | DataSourceContract) -> DataSourceContract:
    if isinstance(data_source, DataSourceContract):
        return data_source
    data_source = canonical_data_source_name(data_source) or ""
    try:
        return DATA_SOURCE_CONTRACTS[data_source]
    except KeyError as exc:
        available = ", ".join(sorted(DATA_SOURCE_CONTRACTS))
        raise ValueError(f"unknown data source {data_source!r}; available: {available}") from exc


def resolved_runtime_data_flow(
    data_source: str | DataSourceContract,
) -> tuple[RuntimeDataFlowStage, ...]:
    """Return concrete runtime data flow for one endpoint data source.

    RUNTIME_DATA_FLOW is the shared template used by evidence validators. This
    resolver expands the source-owned boundary so operators can inspect actual
    topics and command sinks instead of template placeholders.
    """

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
                    "module:TraversableFrontierModule.fused_cost",
                    "module:TraversableFrontierModule.slope_grid",
                    "module:TraversableFrontierModule.esdf_field",
                    "module:TraversableFrontierModule.elevation_map",
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
        AdapterTopicAlias(
            source="save_map",
            target=TOPICS.save_map_service,
            msg_format="service",
            note="Fast-LIO2 map-save service alias.",
        ),
    ),
    "pgo": (
        AdapterTopicAlias(
            source="/cloud_registered",
            target=TOPICS.registered_cloud,
            msg_format="registered_cloud",
        ),
        AdapterTopicAlias(
            source="/Odometry",
            target=TOPICS.odometry,
            msg_format="odometry",
        ),
    ),
    "localizer": (
        AdapterTopicAlias(
            source="/cloud_registered",
            target=TOPICS.registered_cloud,
            msg_format="registered_cloud",
        ),
        AdapterTopicAlias(
            source="/Odometry",
            target=TOPICS.odometry,
            msg_format="odometry",
        ),
        AdapterTopicAlias(
            source="map_cloud",
            target=TOPICS.saved_map_cloud,
            msg_format="map_cloud",
            note="Static saved map from localizer; never the live map cloud.",
        ),
        AdapterTopicAlias(
            source="/localization_quality",
            target=TOPICS.localization_quality,
            msg_format="std_msgs/msg/Float32",
            note="Localizer ICP quality score normalized for LingTu diagnostics.",
        ),
        AdapterTopicAlias(
            source="relocalize",
            target=TOPICS.relocalize_service,
            msg_format="service",
            note="Seeded localizer relocalization service.",
        ),
        AdapterTopicAlias(
            source="relocalize_check",
            target=TOPICS.relocalize_check_service,
            msg_format="service",
            note="Localizer relocalization status service.",
        ),
        AdapterTopicAlias(
            source="global_relocalize",
            target=TOPICS.global_relocalize_service,
            msg_format="service",
            note="Global localizer relocalization service.",
        ),
        AdapterTopicAlias(
            source="global_relocalize_status",
            target=TOPICS.global_relocalize_status_service,
            msg_format="service",
            note="Global localizer backend and last-result status service.",
        ),
    ),
    "pointlio": (
        AdapterTopicAlias(
            source="cloud_registered_body",
            target=TOPICS.registered_cloud,
            msg_format="registered_cloud",
            note="Point-LIO body-frame current scan.",
        ),
        AdapterTopicAlias(
            source="cloud_registered",
            target=TOPICS.map_cloud,
            msg_format="map_cloud",
            note="Point-LIO world-frame registered map cloud.",
        ),
        AdapterTopicAlias(
            source="aft_mapped_to_init",
            target=TOPICS.odometry,
            msg_format="odometry",
        ),
        AdapterTopicAlias(
            source="livox/imu",
            target=TOPICS.imu,
            msg_format="sensor_msgs/msg/Imu",
        ),
        AdapterTopicAlias(
            source="livox/lidar",
            target=TOPICS.lidar_scan,
            msg_format="raw_livox_custom",
        ),
        AdapterTopicAlias(
            source="save_map",
            target=TOPICS.save_map_service,
            msg_format="service",
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
        AdapterTopicAlias(source="/speed", target=TOPICS.speed, msg_format="std_msgs/msg/Float32"),
        AdapterTopicAlias(source="/path", target=TOPICS.local_path, msg_format="nav_msgs/msg/Path"),
        AdapterTopicAlias(source="/stop", target=TOPICS.stop, msg_format="std_msgs/msg/Bool"),
        AdapterTopicAlias(source="/slow_down", target=TOPICS.slow_down, msg_format="std_msgs/msg/Float32"),
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

ADAPTER_RELAY_ALIASES = {
    "cmu_unity": (
        AdapterTopicAlias(
            source="/state_estimation",
            target=TOPICS.odometry,
            msg_format="odometry",
        ),
        AdapterTopicAlias(
            source="/state_estimation_at_scan",
            target=TOPICS.state_estimation_at_scan,
            msg_format="state_estimation_at_scan",
        ),
        AdapterTopicAlias(
            source="/registered_scan",
            target=TOPICS.map_cloud,
            msg_format="map_cloud",
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
            source=TOPICS.cmd_vel,
            target="/cmd_vel",
            msg_format="geometry_msgs/msg/TwistStamped",
            note="Simulation-only command relay; never a hardware output or hardware ROS domain.",
        ),
    ),
}

PROFILE_DATA_SOURCE_BINDINGS = {
    "stub": ProfileDataSourceBinding(
        profile="stub",
        data_source="in_process_stub",
        mode="framework_test",
    ),
    "dev": ProfileDataSourceBinding(
        profile="dev",
        data_source="in_process_stub",
        mode="semantic_pipeline_test",
    ),
    "sim_nav": ProfileDataSourceBinding(
        profile="sim_nav",
        data_source="in_process_stub",
        mode="pure_python_navigation_sim",
    ),
    "sim": ProfileDataSourceBinding(
        profile="sim",
        data_source="mujoco_module_graph",
        mode="module_graph_simulation",
        note="In-process MuJoCo profile; not the Fast-LIO raw sensor gate.",
    ),
    "portable_mujoco": ProfileDataSourceBinding(
        profile="portable_mujoco",
        data_source="mujoco_module_graph",
        mode="portable_no_ros_mujoco_planning_sensor_gate",
        note="Portable in-process MuJoCo kinematic planning/sensor profile; no ROS2 or Fast-LIO live adapter.",
    ),
    "sim_mujoco_live": ProfileDataSourceBinding(
        profile="sim_mujoco_live",
        data_source="mujoco_fastlio2_live",
        mode="mujoco_raw_mid360_fastlio_live",
        note="First-class raw-sensor MuJoCo path: MID-360 pattern LiDAR + IMU -> Fast-LIO -> /slam/*.",
    ),
    "sim_mujoco_octo_live": ProfileDataSourceBinding(
        profile="sim_mujoco_octo_live",
        data_source="mujoco_fastlio2_live",
        mode="mujoco_raw_mid360_fastlio_octoplanner3d_closed_loop",
        note=(
            "OctoPlanner3D MuJoCo path: MID-360 + Fast-LIO source, "
            "OctoPlanner3D global plan, local planner/path follower, "
            "and simulated cmd_vel closure."
        ),
    ),
    "sim_gazebo": ProfileDataSourceBinding(
        profile="sim_gazebo",
        data_source="gazebo_industrial",
        mode="gazebo_ros_native_simulation",
    ),
    "sim_industrial": ProfileDataSourceBinding(
        profile="sim_industrial",
        data_source="gazebo_industrial",
        mode="gazebo_industrial_delivery_demo",
    ),
    "sim_cmu_tare": ProfileDataSourceBinding(
        profile="sim_cmu_tare",
        data_source="cmu_unity_external",
        mode="external_cmu_unity_tare_adapter",
    ),
    "lite": ProfileDataSourceBinding(
        profile="lite",
        data_source="thunder_lite_local",
        mode="minimal_thunder_no_ros",
        note="Driver/safety/navigation shell only; no SLAM, map, semantic, ROS bridge, or external services.",
    ),
    "teleop": ProfileDataSourceBinding(
        profile="teleop",
        data_source=REAL_RUNTIME_CONTRACT,
        mode="real_robot_remote_control",
        note="Operator command only; no autonomous navigation decision chain.",
    ),
    "teleop_avoid": ProfileDataSourceBinding(
        profile="teleop_avoid",
        data_source=REAL_RUNTIME_CONTRACT,
        mode="real_robot_remote_control_with_obstacle_avoidance",
        note="Operator command with live SLAM, obstacle, and traversability gates; no saved map or global planning.",
    ),
    "map": ProfileDataSourceBinding(
        profile="map",
        data_source=REAL_RUNTIME_CONTRACT,
        mode="real_robot_mapping",
    ),
    "tracking": ProfileDataSourceBinding(
        profile="tracking",
        data_source=REAL_RUNTIME_CONTRACT,
        mode="real_robot_tracking",
        note="Follow explicit map-frame goals through native planning and path following.",
    ),
    "nav": ProfileDataSourceBinding(
        profile="nav",
        data_source=REAL_RUNTIME_CONTRACT,
        mode="real_robot_saved_map_navigation",
    ),
    "inspection": ProfileDataSourceBinding(
        profile="inspection",
        data_source=REAL_RUNTIME_CONTRACT,
        mode="real_robot_inspection",
        note="Typed multi-point inspection route execution over saved-map navigation.",
    ),
    "explore": ProfileDataSourceBinding(
        profile="explore",
        data_source=REAL_RUNTIME_CONTRACT,
        mode="real_robot_live_exploration",
    ),
    "tare_explore": ProfileDataSourceBinding(
        profile="tare_explore",
        data_source=REAL_RUNTIME_CONTRACT,
        mode="real_robot_tare_exploration",
    ),
    "super_lio": ProfileDataSourceBinding(
        profile="super_lio",
        data_source=REAL_RUNTIME_CONTRACT,
        mode="real_robot_super_lio_mapping",
    ),
    "super_lio_relocation": ProfileDataSourceBinding(
        profile="super_lio_relocation",
        data_source=REAL_RUNTIME_CONTRACT,
        mode="real_robot_super_lio_relocalization",
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


def topic_ros_types(topic: str) -> tuple[str, ...]:
    """Return concrete ROS message/service payload types accepted on a topic."""

    try:
        return TOPIC_ROS_TYPES[topic]
    except KeyError as exc:
        raise ValueError(f"topic {topic!r} has no declared ROS payload type") from exc


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
    if runtime_contract == THUNDER_LITE_RUNTIME_CONTRACT:
        return dict(THUNDER_LITE_TOPIC_ALLOWED_FRAME_IDS)
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


def profile_data_source(profile: str) -> ProfileDataSourceBinding:
    """Return the declared endpoint data-source binding for one CLI profile."""

    try:
        return PROFILE_DATA_SOURCE_BINDINGS[profile]
    except KeyError as exc:
        available = ", ".join(sorted(PROFILE_DATA_SOURCE_BINDINGS))
        raise ValueError(f"unknown profile data-source binding {profile!r}; available: {available}") from exc


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
        "lidar_extrinsics": {name: asdict(transform) for name, transform in LIDAR_EXTRINSICS.items()},
        "message_formats": {name: asdict(format_spec) for name, format_spec in MESSAGE_FORMATS.items()},
        "topic_formats": {topic: formats for topic, formats in TOPIC_FORMATS.items()},
        "topic_ros_types": {topic: ros_types for topic, ros_types in TOPIC_ROS_TYPES.items()},
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
        "profile_data_sources": {name: asdict(binding) for name, binding in PROFILE_DATA_SOURCE_BINDINGS.items()},
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
