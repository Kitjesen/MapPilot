"""Optional cyclonedds-python transport for Python Modules and diagnostics.

This generic reader accepts LingTu-owned IDL types from ``message.dds_types``.
The file also retains ROS wire-compatible types for explicit diagnostic and
compatibility tools. It does not import rclpy. Field SLAM, terrain, and
navigation endpoints use native C++ typed DDS instead.

NOTE: Do NOT add `from __future__ import annotations` because cyclonedds IdlStruct
needs real type objects at class definition time, not string annotations.

Usage::

    from message.dds_types import DDS_Odometry
    from runtime.adapters.dds.reader import DDSReader

    reader = DDSReader()
    reader.subscribe("/slam/odometry", DDS_Odometry, print)
    reader.spin_background()
"""

import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

logger = logging.getLogger(__name__)

# Standard ROS-wire-compatible message types as cyclonedds IDL structs.
# typename must match the DDS publisher type name exactly for subscription to work.
import os as _os

from runtime.transport.dds_metrics import record_publish, record_receive

# Kill switch: LINGTU_DISABLE_DDS=1 disables this legacy Python DDS helper.
# Use it when cyclonedds-python cannot initialize a diagnostic/compat topic.
_DDS_KILL = _os.environ.get("LINGTU_DISABLE_DDS", "").strip() in ("1", "true", "yes")

try:
    if _DDS_KILL:
        raise ImportError("LINGTU_DISABLE_DDS set; legacy Python DDS reader disabled")
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.idl import IdlStruct, types
    from cyclonedds.qos import Policy, Qos
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from cyclonedds.util import duration

    _HAS_CYCLONEDDS = True

    # Standard scalar messages.

    @dataclass
    class DDS_Time(IdlStruct):
        sec: types.int32
        nanosec: types.uint32

    @dataclass
    class DDS_Header(IdlStruct):
        stamp: DDS_Time
        frame_id: str

    @dataclass
    class DDS_Float32(IdlStruct, typename="std_msgs::msg::dds_::Float32_"):
        data: types.float32

    @dataclass
    class DDS_String(IdlStruct, typename="std_msgs::msg::dds_::String_"):
        data: str

    # Geometry messages.

    @dataclass
    class DDS_Point(IdlStruct):
        x: types.float64
        y: types.float64
        z: types.float64

    @dataclass
    class DDS_Quaternion(IdlStruct):
        x: types.float64
        y: types.float64
        z: types.float64
        w: types.float64

    @dataclass
    class DDS_Pose(IdlStruct):
        position: DDS_Point
        orientation: DDS_Quaternion

    @dataclass
    class DDS_PoseWithCovariance(IdlStruct):
        pose: DDS_Pose
        covariance: types.array[types.float64, 36]

    @dataclass
    class DDS_Vector3(IdlStruct):
        x: types.float64
        y: types.float64
        z: types.float64

    @dataclass
    class DDS_Twist(IdlStruct):
        linear: DDS_Vector3
        angular: DDS_Vector3

    @dataclass
    class DDS_TwistWithCovariance(IdlStruct):
        twist: DDS_Twist
        covariance: types.array[types.float64, 36]

    @dataclass
    class DDS_TwistStamped(IdlStruct, typename="geometry_msgs::msg::dds_::TwistStamped_"):
        header: DDS_Header
        twist: DDS_Twist

    @dataclass
    class DDS_PoseStamped(IdlStruct, typename="geometry_msgs::msg::dds_::PoseStamped_"):
        header: DDS_Header
        pose: DDS_Pose

    # Navigation messages.

    @dataclass
    class DDS_Odometry(IdlStruct, typename="nav_msgs::msg::dds_::Odometry_"):
        header: DDS_Header
        child_frame_id: str
        pose: DDS_PoseWithCovariance
        twist: DDS_TwistWithCovariance

    @dataclass
    class DDS_MapMetaData(IdlStruct):
        map_load_time: DDS_Time
        resolution: types.float32
        width: types.uint32
        height: types.uint32
        origin: DDS_Pose

    @dataclass
    class DDS_OccupancyGrid(IdlStruct, typename="nav_msgs::msg::dds_::OccupancyGrid_"):
        header: DDS_Header
        info: DDS_MapMetaData
        data: types.sequence[types.int8]

    @dataclass
    class DDS_Path(IdlStruct, typename="nav_msgs::msg::dds_::Path_"):
        header: DDS_Header
        poses: types.sequence[DDS_PoseStamped]

    # Sensor messages.

    @dataclass
    class DDS_PointField(IdlStruct):
        name: str
        offset: types.uint32
        datatype: types.uint8
        count: types.uint32

    @dataclass
    class DDS_PointCloud2(IdlStruct, typename="sensor_msgs::msg::dds_::PointCloud2_"):
        header: DDS_Header
        height: types.uint32
        width: types.uint32
        fields: types.sequence[DDS_PointField]
        is_bigendian: bool
        point_step: types.uint32
        row_step: types.uint32
        data: types.sequence[types.uint8]
        is_dense: bool

    @dataclass
    class DDS_Image(IdlStruct, typename="sensor_msgs::msg::dds_::Image_"):
        header: DDS_Header
        height: types.uint32
        width: types.uint32
        encoding: str
        is_bigendian: types.uint8
        step: types.uint32
        data: types.sequence[types.uint8]

    @dataclass
    class DDS_RegionOfInterest(IdlStruct, typename="sensor_msgs::msg::dds_::RegionOfInterest_"):
        x_offset: types.uint32
        y_offset: types.uint32
        height: types.uint32
        width: types.uint32
        do_rectify: bool

    @dataclass
    class DDS_CameraInfo(IdlStruct, typename="sensor_msgs::msg::dds_::CameraInfo_"):
        header: DDS_Header
        height: types.uint32
        width: types.uint32
        distortion_model: str
        d: types.sequence[types.float64]
        k: types.array[types.float64, 9]
        r: types.array[types.float64, 9]
        p: types.array[types.float64, 12]
        binning_x: types.uint32
        binning_y: types.uint32
        roi: DDS_RegionOfInterest

    # Transform and visualization messages.

    @dataclass
    class DDS_TransformStamped_Translation(IdlStruct):
        x: types.float64
        y: types.float64
        z: types.float64

    @dataclass
    class DDS_Transform(IdlStruct):
        translation: DDS_TransformStamped_Translation
        rotation: DDS_Quaternion

    @dataclass
    class DDS_TransformStamped(IdlStruct):
        header: DDS_Header
        child_frame_id: str
        transform: DDS_Transform

    @dataclass
    class DDS_TFMessage(IdlStruct, typename="tf2_msgs::msg::dds_::TFMessage_"):
        transforms: types.sequence[DDS_TransformStamped]

    # ── visualization_msgs (marker subset for rerun detections overlay) ──

    @dataclass
    class DDS_ColorRGBA(IdlStruct):
        r: types.float32
        g: types.float32
        b: types.float32
        a: types.float32

    @dataclass
    class DDS_Marker(IdlStruct):
        id: types.int32
        type: types.int32
        action: types.int32
        pose: DDS_Pose
        color: DDS_ColorRGBA
        text: str
        ns: str

    @dataclass
    class DDS_MarkerArray(IdlStruct, typename="lingtu::dds::MarkerArray"):
        markers: types.sequence[DDS_Marker]

except ImportError:
    _HAS_CYCLONEDDS = False


class DDSReader:
    """Polling DDS reader for optional Python consumers and diagnostics."""

    def __init__(self, domain_id: int | None = None):
        from runtime.transport.qos import resolve_domain_id

        self._domain_id = resolve_domain_id(domain_id)
        self._subs: list[dict] = []
        self._running = False
        self._thread: threading.Thread | None = None
        self._dp = None

    def subscribe(
        self,
        topic_name: str,
        dds_type,
        callback: Callable,
        *,
        dds_topic: str | None = None,
    ):
        """Register a typed subscription."""
        self._subs.append(
            {
                "topic_name": topic_name,
                "dds_topic": dds_topic or "rt" + topic_name,
                "dds_type": dds_type,
                "callback": callback,
                "reader": None,
            }
        )

    def start(self) -> bool:
        if not _HAS_CYCLONEDDS:
            logger.warning("DDSReader: cyclonedds not available")
            return False
        try:
            from runtime.transport.qos import qos_for_topic

            self._dp = DomainParticipant(domain_id=self._domain_id)
            # Default QoS preserves prior behavior (RELIABLE, 1s blocking) for
            # topics without a profile in config/qos_profiles.yaml.
            default_qos = Qos(Policy.Reliability.Reliable(duration(seconds=1)))
            for sub in self._subs:
                try:
                    topic = Topic(self._dp, sub["dds_topic"], sub["dds_type"])
                    qos = qos_for_topic(sub["topic_name"]) or default_qos
                    sub["reader"] = DataReader(self._dp, topic, qos=qos)
                    logger.info(
                        "DDSReader: %s -> %s",
                        sub["topic_name"],
                        sub["dds_topic"],
                    )
                except Exception as e:
                    logger.warning("DDSReader: failed %s: %s", sub["topic_name"], e)
            active = any(sub.get("reader") is not None for sub in self._subs)
            self._running = active
            if not active:
                logger.warning("DDSReader: no DDS subscriptions started")
            return active
        except Exception as e:
            logger.warning("DDSReader: start failed: %s", e)
            return False

    def spin_background(self) -> bool:
        if not self._running:
            if not self.start():
                return False
        if self._thread and self._thread.is_alive():
            return True
        self._thread = threading.Thread(target=self._spin_loop, daemon=True, name="dds_reader")
        self._thread.start()
        return True

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None

    def _spin_loop(self) -> None:
        """Poll readers and dispatch callbacks."""
        while self._running:
            got_any = False
            for sub in self._subs:
                reader = sub.get("reader")
                if reader is None:
                    continue
                try:
                    samples = reader.take(N=32)
                    for sample in samples:
                        if sample is not None:
                            got_any = True
                            record_receive(sub["topic_name"], sample)
                            sub["callback"](sample)
                except Exception as e:
                    logger.debug("DDSReader poll %s: %s", sub["topic_name"], e)
                # Yield GIL between readers so uvicorn can process HTTP
                time.sleep(0)
            # 10 ms active / 20 ms idle keeps latency bounded while freeing the
            # GIL for uvicorn event loop (~50 polls/s is plenty for nav)
            time.sleep(0.010 if got_any else 0.020)


# DDSWriter: publish legacy/diagnostic DDS messages.


class DDSWriter:
    """Publish messages with cyclonedds-python for legacy/diagnostic paths."""

    def __init__(self, domain_id: int | None = None):
        from runtime.transport.qos import resolve_domain_id

        self._domain_id = resolve_domain_id(domain_id)
        self._dp = None
        self._writers: dict = {}  # topic name -> DataWriter

    def start(self) -> bool:
        if not _HAS_CYCLONEDDS:
            return False
        try:
            from cyclonedds.pub import DataWriter as DW

            self._dp = DomainParticipant(domain_id=self._domain_id)
            self._DW = DW
            return True
        except Exception as e:
            logger.warning("DDSWriter: start failed: %s", e)
            return False

    def register(self, topic_name: str, dds_type) -> bool:
        """Register a topic for writing."""
        if not self._dp:
            return False
        try:
            from runtime.transport.qos import qos_for_topic

            topic = Topic(self._dp, "rt" + topic_name, dds_type)
            qos = qos_for_topic(topic_name)
            if qos is not None:
                self._writers[topic_name] = self._DW(self._dp, topic, qos=qos)
            else:
                self._writers[topic_name] = self._DW(self._dp, topic)
            logger.info("DDSWriter: %s", topic_name)
            return True
        except Exception as e:
            logger.warning("DDSWriter: failed %s: %s", topic_name, e)
            return False

    def write(self, topic_name: str, msg) -> bool:
        """Publish a message."""
        writer = self._writers.get(topic_name)
        if not writer:
            return False
        try:
            writer.write(msg)
            record_publish(topic_name)
            return True
        except Exception as e:
            logger.debug("DDSWriter write %s: %s", topic_name, e)
            return False

    def stop(self) -> None:
        self._writers.clear()
        self._dp = None
