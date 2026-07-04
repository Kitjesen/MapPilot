"""Standalone typed DDS endpoint service for Thunder field adapters."""

from __future__ import annotations

import json
import logging
import time
from collections import Counter
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

from runtime.adapters.dds.localization_adapter import (
    _from_dds_lidar_scan,
    _from_dds_odometry,
    _from_dds_pointcloud2,
)
from runtime.adapters.dds.codec import from_dds_pose_stamped, from_dds_string
from runtime.adapters.dds.contracts import (
    THUNDER_FIELD_DDS_CONTRACT_NAME,
    DDSEndpointBinding,
    DDSEndpointContract,
    endpoint_contract,
)
from runtime.msgs.geometry import PoseStamped, Twist
from runtime.msgs.nav import OccupancyGrid, Odometry, Path
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu, PointCloud2
from runtime.runtime_interface import TOPICS, body_frame_id, topic_default_frame_id
from runtime.tf import TF_STATIC_TOPIC, TF_TOPIC, tf_message_from_any
from runtime.transport.abc import TopicConfig

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class DDSEndpointEvent:
    topic: str
    channel: str
    schema: str
    message: Any
    ts: float


class DDSEndpointService:
    """Publish and consume the Thunder endpoint contract over typed DDS."""

    def __init__(
        self,
        endpoint_contract_name: str = THUNDER_FIELD_DDS_CONTRACT_NAME,
        *,
        transport: Any | None = None,
        transport_factory: Callable[[], Any] | None = None,
        on_lingtu_message: Callable[[DDSEndpointEvent], None] | None = None,
        subscribe_lingtu_outputs: bool = True,
        qos_depth: int = 10,
        reliable: bool = True,
        transport_strategy: str | None = None,
    ) -> None:
        self._contract = endpoint_contract(str(endpoint_contract_name or THUNDER_FIELD_DDS_CONTRACT_NAME))
        self._transport = transport
        self._transport_factory = transport_factory
        self._transport_strategy = str(transport_strategy or "dds")
        self._owns_transport = transport is None
        self._on_lingtu_message = on_lingtu_message
        self._subscribe_lingtu_outputs = bool(subscribe_lingtu_outputs)
        self._qos_depth = int(qos_depth)
        self._reliable = bool(reliable)
        self._publishers: dict[str, Any] = {}
        self._subscriptions: list[Any] = []
        self._publish_counts: Counter[str] = Counter()
        self._receive_counts: Counter[str] = Counter()
        self._decode_errors: Counter[str] = Counter()
        self._last_publish_ts = 0.0
        self._last_receive_ts = 0.0
        self._started = False

    @property
    def contract(self) -> DDSEndpointContract:
        return self._contract

    def start(self) -> None:
        if self._started:
            return
        self._transport = self._transport or self._create_default_transport()
        if self._subscribe_lingtu_outputs:
            for binding in self._contract.bindings:
                if binding.direction == "lingtu_to_endpoint":
                    self._subscriptions.append(self._subscribe_binding(binding))
        self._started = True

    def stop(self) -> None:
        for sub in self._subscriptions:
            close = getattr(sub, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("DDS endpoint subscription close failed", exc_info=True)
        self._subscriptions.clear()
        for pub in self._publishers.values():
            close = getattr(pub, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("DDS endpoint publisher close failed", exc_info=True)
        self._publishers.clear()
        if self._owns_transport and self._transport is not None:
            close = getattr(self._transport, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("DDS endpoint transport close failed", exc_info=True)
        self._transport = None
        self._started = False

    def publish_to_lingtu(self, topic: str, msg: Any) -> None:
        binding = self._binding(topic)
        if binding.direction != "endpoint_to_lingtu":
            raise ValueError(f"{topic} is not an endpoint-to-LingTu DDS binding")
        payload = (
            msg
            if self._transport_strategy == "local"
            else _to_dds_message(topic, msg)
        )
        self._publisher(topic).publish(payload)
        self._publish_counts[topic] += 1
        self._last_publish_ts = time.time()

    def publish_sensor_snapshot(
        self,
        *,
        lidar_scan: Any | None = None,
        imu: Any | None = None,
    ) -> int:
        published = 0
        if lidar_scan is not None:
            self.publish_to_lingtu(TOPICS.lidar_scan, lidar_scan)
            published += 1
        if imu is not None:
            self.publish_to_lingtu(TOPICS.imu, imu)
            published += 1
        return published

    def publish_localization_snapshot(
        self,
        *,
        odometry: Any | None = None,
        registered_cloud: Any | None = None,
        map_cloud: Any | None = None,
        localization_health: Mapping[str, Any] | None = None,
        localization_quality: float | None = None,
    ) -> int:
        published = 0
        if odometry is not None:
            self.publish_to_lingtu(TOPICS.odometry, odometry)
            published += 1
        if registered_cloud is not None:
            self.publish_to_lingtu(TOPICS.registered_cloud, registered_cloud)
            published += 1
        if map_cloud is not None:
            self.publish_to_lingtu(TOPICS.map_cloud, map_cloud)
            published += 1
        if localization_health is not None:
            self.publish_to_lingtu(TOPICS.localization_health, dict(localization_health))
            published += 1
        if localization_quality is not None:
            self.publish_to_lingtu(TOPICS.localization_quality, float(localization_quality))
            published += 1
        return published

    def health(self) -> dict[str, Any]:
        return {
            "service": "dds_endpoint_service",
            "endpoint_contract": self._contract.name,
            "runtime_contract": self._contract.runtime_contract,
            "transport": self._transport_strategy,
            "typed_messages": {
                binding.topic: binding.idl_type
                for binding in self._contract.bindings
            },
            "started": self._started,
            "endpoint_to_lingtu_channels": {
                binding.topic: binding.topic
                for binding in self._contract.bindings
                if binding.direction == "endpoint_to_lingtu"
            },
            "lingtu_to_endpoint_channels": {
                binding.topic: binding.topic
                for binding in self._contract.bindings
                if binding.direction == "lingtu_to_endpoint"
            },
            "publish_counts": dict(self._publish_counts),
            "receive_counts": dict(self._receive_counts),
            "decode_errors": dict(self._decode_errors),
            "last_publish_ts": self._last_publish_ts,
            "last_receive_ts": self._last_receive_ts,
        }

    def _create_default_transport(self) -> Any:
        if self._transport_factory is not None:
            return self._transport_factory()
        from runtime.transport.factory import create_transport

        return create_transport("dds")

    def _ensure_transport(self) -> Any:
        self._transport = self._transport or self._create_default_transport()
        return self._transport

    def _binding(self, topic: str) -> DDSEndpointBinding:
        return self._contract.binding_for_topic(topic)

    def _publisher(self, topic: str) -> Any:
        publisher = self._publishers.get(topic)
        if publisher is not None:
            return publisher
        transport = self._ensure_transport()
        publisher = transport.create_publisher(
            TopicConfig(name=topic, qos_depth=self._qos_depth, reliable=self._reliable)
        )
        self._publishers[topic] = publisher
        return publisher

    def _subscribe_binding(self, binding: DDSEndpointBinding) -> Any:
        transport = self._ensure_transport()
        return transport.create_subscriber(
            TopicConfig(name=binding.topic, qos_depth=self._qos_depth, reliable=self._reliable),
            lambda msg: self._on_binding_message(binding, msg),
        )

    def _on_binding_message(self, binding: DDSEndpointBinding, msg: Any) -> None:
        try:
            decoded = (
                msg
                if self._transport_strategy == "local"
                else _from_dds_message(binding.topic, msg)
            )
        except Exception:
            self._decode_errors[binding.topic] += 1
            logger.exception("Failed to decode DDS endpoint payload on %s", binding.topic)
            return
        self._receive_counts[binding.topic] += 1
        self._last_receive_ts = time.time()
        if self._on_lingtu_message is not None:
            self._on_lingtu_message(
                DDSEndpointEvent(
                    topic=binding.topic,
                    channel=binding.topic,
                    schema=binding.schema,
                    message=decoded,
                    ts=self._last_receive_ts,
                )
            )


def _dds() -> Any:
    from message import dds_types as dds_mod

    return dds_mod


def _to_dds_time(ts: float | int | None) -> Any:
    value = float(ts or time.time())
    sec = int(value)
    nanosec = int(max(0.0, value - sec) * 1_000_000_000)
    return _dds().DDS_Time(sec=sec, nanosec=nanosec)


def _to_dds_header(frame_id: str, ts: float | int | None) -> Any:
    return _dds().DDS_Header(stamp=_to_dds_time(ts), frame_id=str(frame_id or ""))


def _to_dds_vector(value: Any) -> Any:
    return _dds().DDS_Vector3(x=float(value.x), y=float(value.y), z=float(value.z))


def _to_dds_quaternion(value: Any) -> Any:
    return _dds().DDS_Quaternion(
        x=float(value.x),
        y=float(value.y),
        z=float(value.z),
        w=float(value.w),
    )


def _to_dds_point(value: Any) -> Any:
    return _dds().DDS_Point(x=float(value.x), y=float(value.y), z=float(value.z))


def _to_dds_pose(value: Any) -> Any:
    return _dds().DDS_Pose(
        position=_to_dds_point(value.position),
        orientation=_to_dds_quaternion(value.orientation),
    )


def _to_dds_odometry(odom: Odometry) -> Any:
    dds_mod = _dds()
    return dds_mod.DDS_Odometry(
        header=_to_dds_header(odom.frame_id, odom.ts),
        child_frame_id=str(odom.child_frame_id or body_frame_id()),
        pose=dds_mod.DDS_PoseWithCovariance(
            pose=_to_dds_pose(odom.pose),
            covariance=[0.0] * 36,
        ),
        twist=dds_mod.DDS_TwistWithCovariance(
            twist=dds_mod.DDS_Twist(
                linear=_to_dds_vector(odom.twist.linear),
                angular=_to_dds_vector(odom.twist.angular),
            ),
            covariance=[0.0] * 36,
        ),
    )


def _to_dds_pointcloud2(cloud: PointCloud2) -> Any:
    dds_mod = _dds()
    return dds_mod.DDS_PointCloud2(
        header=_to_dds_header(cloud.frame_id, cloud.ts),
        height=int(cloud.height),
        width=int(cloud.width),
        fields=[
            dds_mod.DDS_PointField(
                name=field.name,
                offset=int(field.offset),
                datatype=int(field.datatype),
                count=int(field.count),
            )
            for field in cloud.fields
        ],
        is_bigendian=bool(cloud.is_bigendian),
        point_step=int(cloud.point_step),
        row_step=int(cloud.row_step),
        data=list(cloud.data),
        is_dense=bool(cloud.is_dense),
    )


def _to_dds_occupancy_grid(grid: OccupancyGrid | Mapping[str, Any]) -> Any:
    dds_mod = _dds()
    if isinstance(grid, OccupancyGrid):
        payload = grid.to_dict()
    elif isinstance(grid, Mapping):
        payload = dict(grid)
    else:
        raise TypeError(
            f"traversability expects OccupancyGrid or dict, got {type(grid).__name__}"
        )

    values = np.asarray(payload.get("grid"), dtype=np.int16)
    if values.ndim != 2:
        values = np.zeros((0, 0), dtype=np.int16)
    height, width = values.shape
    origin = payload.get("origin")
    if isinstance(origin, Mapping):
        origin_xy = (
            float(origin.get("x", 0.0)),
            float(origin.get("y", 0.0)),
        )
    elif origin is not None:
        origin_xy = (float(origin[0]), float(origin[1]))
    else:
        origin_xy = (
            float(payload.get("origin_x") or 0.0),
            float(payload.get("origin_y") or 0.0),
        )
    ts = float(payload.get("ts") or time.time())
    return dds_mod.DDS_OccupancyGrid(
        header=_to_dds_header(str(payload.get("frame_id") or "map"), ts),
        info=dds_mod.DDS_MapMetaData(
            map_load_time=_to_dds_time(ts),
            resolution=float(payload.get("resolution") or 0.0),
            width=int(width),
            height=int(height),
            origin=dds_mod.DDS_Pose(
                position=dds_mod.DDS_Point(x=origin_xy[0], y=origin_xy[1], z=0.0),
                orientation=dds_mod.DDS_Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=np.clip(values, -1, 100).astype(np.int8).reshape(-1).tolist(),
    )


def _to_dds_imu(imu: Imu) -> Any:
    from message import dds_types

    return dds_types.Imu(
        header=_to_dds_header(imu.frame_id, imu.ts),
        orientation=_to_dds_quaternion(imu.orientation),
        orientation_covariance=list(imu.orientation_covariance),
        angular_velocity=_to_dds_vector(imu.angular_velocity),
        angular_velocity_covariance=list(imu.angular_velocity_covariance),
        linear_acceleration=_to_dds_vector(imu.linear_acceleration),
        linear_acceleration_covariance=list(imu.linear_acceleration_covariance),
    )


def _to_dds_livox_custom_msg(scan: Any) -> Any:
    from message import dds_types

    if hasattr(scan, "timebase") and hasattr(scan, "points"):
        return scan
    if hasattr(scan, "timestamp_ns") and hasattr(scan, "points"):
        return dds_types.livox_frame_to_msg(scan)
    if not isinstance(scan, PointCloud2):
        raise TypeError(f"lidar_scan expects LivoxCustomMsg or PointCloud2, got {type(scan).__name__}")

    points = np.asarray(scan.points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError(f"lidar_scan points must be (N,3) or (N,4), got {points.shape}")
    stamp_ns = int(float(scan.ts or time.time()) * 1_000_000_000)
    livox_points = []
    for row in points:
        intensity = float(row[3]) if row.shape[0] > 3 else 0.0
        livox_points.append(
            dds_types.LivoxPoint(
                offset_time=0,
                x=float(row[0]),
                y=float(row[1]),
                z=float(row[2]),
                reflectivity=max(0, min(255, int(round(intensity)))),
                tag=0,
                line=0,
            )
        )
    return dds_types.LivoxFrame(
        header=_to_dds_header(scan.frame_id, scan.ts),
        timebase=stamp_ns,
        point_num=len(livox_points),
        lidar_id=0,
        rsvd=[0, 0, 0],
        points=livox_points,
    )


def _to_dds_pose_stamped(pose: PoseStamped) -> Any:
    dds_mod = _dds()
    return dds_mod.DDS_PoseStamped(
        header=_to_dds_header(pose.frame_id, pose.ts),
        pose=_to_dds_pose(pose.pose),
    )


def _to_dds_tf_message(msg: Any) -> Any:
    dds_mod = _dds()
    tf_msg = tf_message_from_any(msg)
    return dds_mod.DDS_TFMessage(
        transforms=[
            dds_mod.DDS_TransformStamped(
                header=_to_dds_header(transform.frame_id, transform.ts),
                child_frame_id=str(transform.child_frame_id),
                transform=dds_mod.DDS_Transform(
                    translation=_to_dds_vector(transform.translation),
                    rotation=_to_dds_quaternion(transform.rotation),
                ),
            )
            for transform in tf_msg.transforms
        ]
    )


def _to_dds_string(value: Any) -> Any:
    return _dds().DDS_String(data=str(value or ""))


def _to_dds_text(value: Any) -> Any:
    return _dds().Text(data=str(value or ""))


def _to_dds_float32(value: Any) -> Any:
    return _dds().DDS_Float32(data=float(value))


def _to_dds_message(topic: str, msg: Any) -> Any:
    if topic in {TF_TOPIC, TF_STATIC_TOPIC}:
        return _to_dds_tf_message(msg)
    if topic == TOPICS.lidar_scan:
        return _to_dds_livox_custom_msg(msg)
    if topic in {TOPICS.registered_cloud, TOPICS.map_cloud, TOPICS.saved_map_cloud}:
        return _to_dds_pointcloud2(msg)
    if topic == TOPICS.traversability:
        return _to_dds_occupancy_grid(msg)
    if topic == TOPICS.odometry:
        return _to_dds_odometry(msg)
    if topic == TOPICS.imu:
        return _to_dds_imu(msg)
    if topic == TOPICS.localization_quality:
        return _to_dds_float32(msg)
    if topic == TOPICS.localization_health:
        return _to_dds_text(json.dumps(dict(msg), ensure_ascii=True, sort_keys=True))
    if topic == TOPICS.goal_pose:
        return _to_dds_pose_stamped(msg)
    if topic in {TOPICS.cancel, TOPICS.semantic_instruction}:
        return _to_dds_string(msg)
    raise ValueError(f"DDS endpoint publisher for {topic} is not implemented")


def _from_dds_twist_stamped(msg: Any) -> Twist:
    twist = getattr(msg, "twist", msg)
    return Twist(
        linear=_to_vector3(twist.linear),
        angular=_to_vector3(twist.angular),
    )


def _from_dds_occupancy_grid(msg: Any) -> dict[str, Any]:
    width = int(getattr(msg.info, "width", 0))
    height = int(getattr(msg.info, "height", 0))
    values = np.asarray(list(getattr(msg, "data", []) or []), dtype=np.int16)
    if width > 0 and height > 0 and values.size >= width * height:
        grid = values[: width * height].reshape((height, width)).astype(np.int16)
    else:
        grid = np.zeros((0, 0), dtype=np.int16)
    origin = getattr(msg.info, "origin", None)
    position = getattr(origin, "position", None)
    return {
        "grid": grid.tolist(),
        "resolution": float(getattr(msg.info, "resolution", 0.0)),
        "origin": [
            float(getattr(position, "x", 0.0)),
            float(getattr(position, "y", 0.0)),
        ],
        "frame_id": str(getattr(msg.header, "frame_id", "") or "map"),
        "ts": _stamp_to_seconds(getattr(msg.header, "stamp", None)),
        "width": width,
        "height": height,
    }


def _stamp_to_seconds(stamp: Any) -> float:
    return float(getattr(stamp, "sec", 0.0)) + float(
        getattr(stamp, "nanosec", 0.0)
    ) * 1e-9


def _to_vector3(value: Any) -> Any:
    from runtime.msgs.geometry import Vector3

    return Vector3(
        float(getattr(value, "x", 0.0)),
        float(getattr(value, "y", 0.0)),
        float(getattr(value, "z", 0.0)),
    )


def _from_dds_message(topic: str, msg: Any) -> Any:
    if topic in {TF_TOPIC, TF_STATIC_TOPIC}:
        return tf_message_from_any(msg)
    if topic == TOPICS.lidar_scan:
        return _from_dds_lidar_scan(msg)
    if topic in {TOPICS.global_path, TOPICS.local_path}:
        return msg
    if topic == TOPICS.nav_way_point:
        return from_dds_pose_stamped(msg, topic_default_frame_id(TOPICS.nav_way_point))
    if topic == TOPICS.cmd_vel:
        return _from_dds_twist_stamped(msg)
    if topic == TOPICS.goal_pose:
        return from_dds_pose_stamped(msg, topic_default_frame_id(TOPICS.goal_pose))
    if topic in {TOPICS.cancel, TOPICS.semantic_instruction}:
        return from_dds_string(msg)
    if topic == TOPICS.odometry:
        return _from_dds_odometry(msg)
    if topic in {TOPICS.registered_cloud, TOPICS.map_cloud, TOPICS.saved_map_cloud}:
        return _from_dds_pointcloud2(msg)
    if topic == TOPICS.traversability:
        return _from_dds_occupancy_grid(msg)
    return msg
