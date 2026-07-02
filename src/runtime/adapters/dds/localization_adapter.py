"""Typed DDS localization adapter for Thunder field runtime."""

from __future__ import annotations

import json
import logging
import time
from collections import Counter
from collections.abc import Callable, Mapping
from typing import Any

from runtime.backend_status import BackendStatus
from runtime.module import Module
from runtime.msgs.geometry import Pose, Quaternion, Transform, Twist, Vector3
from runtime.msgs.gnss import GnssOdom
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu, PointCloud2
from runtime.registry import register
from runtime.runtime_interface import TOPICS, body_frame_id, topic_default_frame_id
from runtime.stream import In, Out
from runtime.tf import FrameTree, TF_STATIC_TOPIC, TF_TOPIC, iter_tf_transforms
from runtime.transport.abc import TopicConfig

logger = logging.getLogger(__name__)


@register(
    "localization_adapter",
    "dds_endpoint",
    description="Typed DDS endpoint adapter for Thunder localization and map streams",
)
class DDSLocalizationAdapterModule(Module, layer=1):
    """Bridge typed DDS localization topics into LingTu Module ports."""

    lidar_scan: Out[PointCloud2]
    imu: Out[Imu]
    registered_cloud: Out[PointCloud2]
    map_cloud: Out[PointCloud2]
    saved_map: Out[PointCloud2]
    odometry: Out[Odometry]
    localization_quality: Out[float]
    alive: Out[bool]
    localization_status: Out[dict]
    gnss_fusion_health: Out[dict]
    map_odom_tf: Out[dict]
    map_frame_jump_event: Out[dict]
    scene_mode: Out[str]

    visual_odom: In[Odometry]
    gnss_odom: In[GnssOdom]

    def __init__(
        self,
        backend_profile: str = "bridge",
        transport: Any | None = None,
        transport_factory: Callable[[], Any] | None = None,
        domain_id: int = 0,
        qos_depth: int = 10,
        reliable: bool = True,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._backend_profile = str(backend_profile or "bridge")
        self._transport = transport
        self._transport_factory = transport_factory
        self._owns_transport = transport is None
        self._domain_id = int(domain_id)
        self._qos_depth = int(qos_depth)
        self._reliable = bool(reliable)
        self._subscriptions: list[Any] = []
        self._message_counts: Counter[str] = Counter()
        self._decode_errors: Counter[str] = Counter()
        self._last_message_ts = 0.0
        self._last_visual_odom: Odometry | None = None
        self._last_gnss_odom: GnssOdom | None = None
        self._frame_tree = kw.get("frame_tree") or FrameTree.from_robot_config()
        self._backend_status = BackendStatus.configured_as("dds_endpoint")

    def setup(self) -> None:
        self.visual_odom.subscribe(lambda msg: setattr(self, "_last_visual_odom", msg))
        self.gnss_odom.subscribe(lambda msg: setattr(self, "_last_gnss_odom", msg))
        if self._transport is None:
            self._transport = self._create_default_transport()
        for topic, callback in (
            (TF_TOPIC, self._on_tf),
            (TF_STATIC_TOPIC, self._on_static_tf),
            (TOPICS.lidar_scan, self._on_lidar_scan),
            (TOPICS.imu, self._on_imu),
            (TOPICS.odometry, self._on_odometry),
            (TOPICS.registered_cloud, self._on_registered_cloud),
            (TOPICS.map_cloud, self._on_map_cloud),
            (TOPICS.saved_map_cloud, self._on_saved_map),
            (TOPICS.localization_quality, self._on_localization_quality),
            (TOPICS.localization_health, self._on_localization_health),
        ):
            self._subscribe(topic, callback)

    def stop(self) -> None:
        for sub in self._subscriptions:
            close = getattr(sub, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("DDS localization subscription close failed", exc_info=True)
        self._subscriptions.clear()
        if self._owns_transport and self._transport is not None:
            close = getattr(self._transport, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("DDS localization transport close failed", exc_info=True)
        self._transport = None
        super().stop()

    def health(self) -> dict[str, Any]:
        return {
            **self._backend_status.as_health_fields(),
            "transport": "dds",
            "subscribed_topics": [topic for topic, _ in self._topic_callbacks()],
            "message_counts": dict(self._message_counts),
            "decode_errors": dict(self._decode_errors),
            "last_message_ts": self._last_message_ts,
        }

    def _create_default_transport(self) -> Any:
        if self._transport_factory is not None:
            return self._transport_factory()
        from runtime.transport.dds import DDSTransport

        return DDSTransport(domain_id=self._domain_id)

    def _topic_callbacks(self) -> tuple[tuple[str, Callable[[Any], None]], ...]:
        return (
            (TF_TOPIC, self._on_tf),
            (TF_STATIC_TOPIC, self._on_static_tf),
            (TOPICS.lidar_scan, self._on_lidar_scan),
            (TOPICS.imu, self._on_imu),
            (TOPICS.odometry, self._on_odometry),
            (TOPICS.registered_cloud, self._on_registered_cloud),
            (TOPICS.map_cloud, self._on_map_cloud),
            (TOPICS.saved_map_cloud, self._on_saved_map),
            (TOPICS.localization_quality, self._on_localization_quality),
            (TOPICS.localization_health, self._on_localization_health),
        )

    def _subscribe(self, topic: str, callback: Callable[[Any], None]) -> None:
        if self._transport is None:
            raise RuntimeError("DDS localization adapter transport is not initialized")
        sub = self._transport.create_subscriber(
            TopicConfig(name=topic, qos_depth=self._qos_depth, reliable=self._reliable),
            callback,
        )
        self._subscriptions.append(sub)

    def _record_message(self, topic: str) -> None:
        self._message_counts[topic] += 1
        self._last_message_ts = time.time()

    def _record_decode_error(self, topic: str) -> None:
        self._decode_errors[topic] += 1
        logger.exception("Failed to decode typed DDS localization payload for %s", topic)

    def _on_lidar_scan(self, msg: Any) -> None:
        try:
            cloud = _from_dds_lidar_scan(msg)
            if cloud is not None:
                self.lidar_scan.publish(cloud)
        except Exception:
            self._record_decode_error(TOPICS.lidar_scan)
            return
        self._record_message(TOPICS.lidar_scan)

    def _on_imu(self, msg: Any) -> None:
        try:
            self.imu.publish(_from_dds_imu(msg))
        except Exception:
            self._record_decode_error(TOPICS.imu)
            return
        self._record_message(TOPICS.imu)

    def _on_odometry(self, msg: Any) -> None:
        try:
            odom = _from_dds_odometry(msg)
            self._frame_tree.update_odometry(odom)
            self.odometry.publish(odom)
            self._publish_tracking_status(TOPICS.odometry, odom.ts)
        except Exception:
            self._record_decode_error(TOPICS.odometry)
            return
        self._record_message(TOPICS.odometry)

    def _on_registered_cloud(self, msg: Any) -> None:
        try:
            self.registered_cloud.publish(_from_dds_pointcloud2(msg))
        except Exception:
            self._record_decode_error(TOPICS.registered_cloud)
            return
        self._record_message(TOPICS.registered_cloud)

    def _on_map_cloud(self, msg: Any) -> None:
        try:
            cloud = _from_dds_pointcloud2(msg)
            self.map_cloud.publish(cloud)
            self._publish_tracking_status(TOPICS.map_cloud, cloud.ts)
        except Exception:
            self._record_decode_error(TOPICS.map_cloud)
            return
        self._record_message(TOPICS.map_cloud)

    def _on_saved_map(self, msg: Any) -> None:
        try:
            self.saved_map.publish(_from_dds_pointcloud2(msg))
        except Exception:
            self._record_decode_error(TOPICS.saved_map_cloud)
            return
        self._record_message(TOPICS.saved_map_cloud)

    def _on_localization_quality(self, msg: Any) -> None:
        try:
            self.localization_quality.publish(float(getattr(msg, "data", msg)))
        except Exception:
            self._record_decode_error(TOPICS.localization_quality)
            return
        self._record_message(TOPICS.localization_quality)

    def _on_localization_health(self, msg: Any) -> None:
        try:
            payload = _coerce_health_payload(msg)
            self._publish_localization_health(payload)
        except Exception:
            self._record_decode_error(TOPICS.localization_health)
            return
        self._record_message(TOPICS.localization_health)

    def _on_tf(self, msg: Any) -> None:
        try:
            for transform in iter_tf_transforms(msg):
                self._frame_tree.set_transform(transform, authority="dds_tf")
                self._publish_map_odom_if_match(transform)
        except Exception:
            self._record_decode_error(TF_TOPIC)
            return
        self._record_message(TF_TOPIC)

    def _on_static_tf(self, msg: Any) -> None:
        try:
            for transform in iter_tf_transforms(msg):
                self._frame_tree.set_static_transform(transform, authority="dds_tf_static")
                self._publish_map_odom_if_match(transform)
        except Exception:
            self._record_decode_error(TF_STATIC_TOPIC)
            return
        self._record_message(TF_STATIC_TOPIC)

    def _publish_tracking_status(self, source_topic: str, ts: float) -> None:
        status = {
            "state": "TRACKING",
            "confidence": 1.0,
            "backend": "dds_endpoint",
            "backend_profile": self._backend_profile,
            "health_source": "dds_endpoint",
            "source_topic": source_topic,
            "ts": ts or time.time(),
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": False,
            "map_save_supported": False,
            "map_save_source": "dds_endpoint",
        }
        self.alive.publish(True)
        self.localization_status.publish(status)

    def _publish_localization_health(self, msg: Mapping[str, Any] | str) -> None:
        status = dict(msg) if isinstance(msg, Mapping) else {"state": str(msg)}
        state = str(status.get("state") or "UNKNOWN").upper()
        status.setdefault("state", state)
        status.setdefault("confidence", 1.0 if state in {"TRACKING", "LOCKED", "OK"} else 0.0)
        status.setdefault("backend", "dds_endpoint")
        status.setdefault("backend_profile", self._backend_profile)
        status.setdefault("health_source", "dds_endpoint")
        status.setdefault("ts", time.time())
        status.setdefault("relocalization_supported", False)
        status.setdefault("saved_map_relocalization_supported", False)
        status.setdefault("restart_recovery_supported", False)
        status.setdefault("map_save_supported", False)
        status.setdefault("map_save_source", "dds_endpoint")

        self.alive.publish(state not in {"LOST", "DIVERGED", "UNINIT"})
        self.localization_status.publish(status)

        quality = status.get("quality", status.get("confidence"))
        if quality is not None:
            self.localization_quality.publish(float(quality))
        if isinstance(status.get("gnss_fusion_health"), dict):
            self.gnss_fusion_health.publish(dict(status["gnss_fusion_health"]))
        if isinstance(status.get("map_odom_tf"), dict):
            tf = dict(status["map_odom_tf"])
            self._update_map_odom_tf(tf)
            self.map_odom_tf.publish(tf)
        if isinstance(status.get("map_frame_jump_event"), dict):
            self.map_frame_jump_event.publish(dict(status["map_frame_jump_event"]))
        if status.get("scene_mode") is not None:
            self.scene_mode.publish(str(status["scene_mode"]))

    def _update_map_odom_tf(self, tf: Mapping[str, Any]) -> None:
        if not tf or not tf.get("valid", False):
            return
        self._frame_tree.set_transform(
            Transform(
                translation=Vector3(float(tf["tx"]), float(tf["ty"]), float(tf["tz"])),
                rotation=Quaternion(
                    float(tf["qx"]),
                    float(tf["qy"]),
                    float(tf["qz"]),
                    float(tf["qw"]),
                ),
                frame_id=topic_default_frame_id(TOPICS.map_cloud),
                child_frame_id=topic_default_frame_id(TOPICS.odometry),
                ts=float(tf.get("ts") or time.time()),
            ),
            authority="dds_localization_health",
        )

    def _publish_map_odom_if_match(self, transform: Transform) -> None:
        if (
            transform.frame_id != topic_default_frame_id(TOPICS.map_cloud)
            or transform.child_frame_id != topic_default_frame_id(TOPICS.odometry)
        ):
            return
        self.map_odom_tf.publish(
            {
                "valid": True,
                "frame_id": transform.frame_id,
                "child_frame_id": transform.child_frame_id,
                "tx": transform.translation.x,
                "ty": transform.translation.y,
                "tz": transform.translation.z,
                "qx": transform.rotation.x,
                "qy": transform.rotation.y,
                "qz": transform.rotation.z,
                "qw": transform.rotation.w,
                "ts": transform.ts,
            }
        )


def _stamp_seconds(header: Any) -> float:
    stamp = getattr(header, "stamp", None)
    return float(getattr(stamp, "sec", 0.0)) + float(getattr(stamp, "nanosec", 0.0)) * 1e-9


def _from_dds_odometry(msg: Any) -> Odometry:
    pose = msg.pose.pose
    twist = msg.twist.twist
    position = pose.position
    orientation = pose.orientation
    return Odometry(
        pose=Pose(
            position=Vector3(position.x, position.y, position.z),
            orientation=Quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
        ),
        twist=Twist(
            linear=Vector3(twist.linear.x, twist.linear.y, twist.linear.z),
            angular=Vector3(twist.angular.x, twist.angular.y, twist.angular.z),
        ),
        ts=_stamp_seconds(msg.header),
        frame_id=str(getattr(msg.header, "frame_id", "") or topic_default_frame_id(TOPICS.odometry)),
        child_frame_id=str(getattr(msg, "child_frame_id", "") or body_frame_id()),
    )


def _from_dds_pointcloud2(msg: Any) -> PointCloud2:
    n = int(getattr(msg, "width", 0)) * int(getattr(msg, "height", 0))
    step = int(getattr(msg, "point_step", 0))
    if n <= 0:
        return PointCloud2(points=np.zeros((0, 3), dtype=np.float32), frame_id=_frame_id(msg))
    if step < 12:
        raise ValueError(f"PointCloud2 point_step too small: {step}")
    raw = np.frombuffer(bytes(getattr(msg, "data", b"")), dtype=np.uint8).reshape(n, step)
    fields = {str(getattr(field, "name", "")): int(getattr(field, "offset", 0)) for field in getattr(msg, "fields", [])}
    x_off = fields.get("x", 0)
    y_off = fields.get("y", 4)
    z_off = fields.get("z", 8)
    cols = [
        raw[:, x_off : x_off + 4].copy().view(np.float32).reshape(n),
        raw[:, y_off : y_off + 4].copy().view(np.float32).reshape(n),
        raw[:, z_off : z_off + 4].copy().view(np.float32).reshape(n),
    ]
    intensity_off = fields.get("intensity")
    if intensity_off is not None and intensity_off + 4 <= step:
        cols.append(raw[:, intensity_off : intensity_off + 4].copy().view(np.float32).reshape(n))
    points = np.stack(cols, axis=1).astype(np.float32, copy=False)
    return PointCloud2(
        points=points,
        ts=_stamp_seconds(msg.header),
        frame_id=_frame_id(msg),
        height=int(getattr(msg, "height", 1) or 1),
        width=int(getattr(msg, "width", n) or n),
        is_bigendian=bool(getattr(msg, "is_bigendian", False)),
        is_dense=bool(getattr(msg, "is_dense", True)),
    )


def _from_dds_lidar_scan(msg: Any) -> PointCloud2 | None:
    if hasattr(msg, "points") and hasattr(msg, "timebase"):
        from message.dds_types.livox import livox_msg_to_numpy

        points = livox_msg_to_numpy(msg)
        if points is None:
            return None
        return PointCloud2(
            points=points,
            ts=float(getattr(msg, "timebase", 0) or 0) * 1e-9,
            frame_id=_frame_id(msg),
        )
    return _from_dds_pointcloud2(msg)


def _from_dds_imu(msg: Any) -> Imu:
    orientation = msg.orientation
    angular = msg.angular_velocity
    linear = msg.linear_acceleration
    return Imu(
        orientation=Quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
        orientation_covariance=list(getattr(msg, "orientation_covariance", [0.0] * 9)),
        angular_velocity=Vector3(angular.x, angular.y, angular.z),
        angular_velocity_covariance=list(getattr(msg, "angular_velocity_covariance", [0.0] * 9)),
        linear_acceleration=Vector3(linear.x, linear.y, linear.z),
        linear_acceleration_covariance=list(getattr(msg, "linear_acceleration_covariance", [0.0] * 9)),
        ts=_stamp_seconds(msg.header),
        frame_id=_frame_id(msg, default="imu_link"),
    )


def _coerce_health_payload(msg: Any) -> Mapping[str, Any] | str:
    data = getattr(msg, "data", msg)
    if isinstance(data, Mapping):
        return data
    text = str(data or "")
    if text.startswith("{"):
        return json.loads(text)
    return text


def _frame_id(msg: Any, *, default: str = "") -> str:
    header = getattr(msg, "header", None)
    return str(getattr(header, "frame_id", "") or default)
