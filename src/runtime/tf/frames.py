"""Canonical frame names, frame acceptance rules, and transform math."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass
from typing import Any

from message.topics import TOPICS


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


FRAMES = RuntimeFrames()


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

    if runtime_contract == "real":
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

    if runtime_contract == "real":
        return REAL_RUNTIME_REQUIRED_TOPIC_FRAME_IDS
    return ()


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
