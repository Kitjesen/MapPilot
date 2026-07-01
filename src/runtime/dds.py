"""Lightweight DDS layer 鈥?subscribes to ROS2 topics without rclpy.

Uses cyclonedds with IDL types that match ROS2 DDS type names exactly.
Zero ROS2 dependency: `pip install cyclonedds` is all you need.

NOTE: Do NOT add `from __future__ import annotations` 鈥?cyclonedds IdlStruct
needs real type objects at class definition time, not string annotations.

Usage::

    from runtime.dds import ROS2TopicReader

    reader = ROS2TopicReader()
    reader.on_odometry("/slam/odometry", lambda o: print(o.pose.pose.position.x))
    reader.on_pointcloud2("/slam/map_cloud", lambda pc: print(pc.width))
    reader.spin_background()
"""

import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

logger = logging.getLogger(__name__)

# 鈹€鈹€ ROS2 message types as cyclonedds IDL structs 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
# typename must match ROS2 DDS type name exactly for subscription to work.

import os as _os  # noqa: E402

# Kill switch: LINGTU_DISABLE_DDS=1 forces the rclpy fallback everywhere.
# Use this when cyclonedds segfaults during Topic() init 鈥?typically a
# QoS/GUID mismatch between a fastrtps publisher (e.g. orbbec_camera
# launched by rclpy) and a cyclonedds subscriber on the same topic.
# Workaround: LINGTU_DISABLE_DDS=1 ./start_lingtu.sh
_DDS_KILL = _os.environ.get("LINGTU_DISABLE_DDS", "").strip() in ("1", "true", "yes")

try:
    if _DDS_KILL:
        raise ImportError("LINGTU_DISABLE_DDS set 鈥?forcing rclpy fallback")
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.idl import IdlStruct, types
    from cyclonedds.qos import Policy, Qos
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    from cyclonedds.util import duration

    _HAS_CYCLONEDDS = True

    # 鈹€鈹€ std_msgs 鈹€鈹€

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

    # 鈹€鈹€ geometry_msgs 鈹€鈹€

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

    # 鈹€鈹€ nav_msgs 鈹€鈹€

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

    # 鈹€鈹€ sensor_msgs 鈹€鈹€

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

    # 鈹€鈹€ tf2_msgs 鈹€鈹€

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

except ImportError:
    _HAS_CYCLONEDDS = False


# 鈹€鈹€ DDSReader 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

# Map of ROS2 message type shortnames 鈫?(DDS IDL class, DDS topic prefix)
_MSG_TYPES: dict[str, Any] = {}
if _HAS_CYCLONEDDS:
    _MSG_TYPES = {
        "odometry": DDS_Odometry,
        "pointcloud2": DDS_PointCloud2,
        "image": DDS_Image,
        "camera_info": DDS_CameraInfo,
        "occupancygrid": DDS_OccupancyGrid,
        "path": DDS_Path,
        "tfmessage": DDS_TFMessage,
        "string": DDS_String,
        "twist_stamped": DDS_TwistStamped,
    }


class DDSReader:
    """Lightweight DDS reader for ROS2 topics 鈥?no rclpy needed.

    Uses Listener-based push delivery (zero-latency) with a thin dispatch thread.
    Falls back to polling if Listener is not available in this cyclonedds version.
    """

    def __init__(self, domain_id: int = 0):
        self._domain_id = domain_id
        self._subs: list[dict] = []
        self._running = False
        self._thread: threading.Thread | None = None
        self._dp = None

    def subscribe(self, ros2_topic: str, dds_type, callback: Callable):
        """Register a typed subscription."""
        self._subs.append({
            "ros2_topic": ros2_topic,
            "dds_topic": "rt" + ros2_topic,
            "dds_type": dds_type,
            "callback": callback,
            "reader": None,
        })

    def start(self) -> bool:
        if not _HAS_CYCLONEDDS:
            logger.warning("DDSReader: cyclonedds not available")
            return False
        try:
            self._dp = DomainParticipant(domain_id=self._domain_id)
            qos = Qos(Policy.Reliability.Reliable(duration(seconds=1)))
            for sub in self._subs:
                try:
                    topic = Topic(self._dp, sub["dds_topic"], sub["dds_type"])
                    sub["reader"] = DataReader(self._dp, topic, qos=qos)
                    logger.info("DDSReader: %s 鈫?%s", sub["ros2_topic"], sub["dds_topic"])
                except Exception as e:
                    logger.warning("DDSReader: failed %s: %s", sub["ros2_topic"], e)
            self._running = True
            return True
        except Exception as e:
            logger.warning("DDSReader: start failed: %s", e)
            return False

    def spin_background(self) -> None:
        if not self._running:
            if not self.start():
                return
        self._thread = threading.Thread(target=self._spin_loop, daemon=True, name="dds_reader")
        self._thread.start()

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
                            sub["callback"](sample)
                except Exception as e:
                    logger.debug("DDSReader poll %s: %s", sub["ros2_topic"], e)
                # Yield GIL between readers so uvicorn can process HTTP
                time.sleep(0)
            # 10ms active / 20ms idle 鈥?still <100ms latency, but frees
            # GIL for uvicorn event loop (~50 polls/s is plenty for nav)
            time.sleep(0.010 if got_any else 0.020)


# 鈹€鈹€ DDSWriter 鈥?publish to ROS2 topics 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

class DDSWriter:
    """Publish messages to ROS2 DDS topics without rclpy."""

    def __init__(self, domain_id: int = 0):
        self._domain_id = domain_id
        self._dp = None
        self._writers: dict = {}  # ros2_topic 鈫?DataWriter

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

    def register(self, ros2_topic: str, dds_type) -> bool:
        """Register a topic for writing."""
        if not self._dp:
            return False
        try:
            topic = Topic(self._dp, "rt" + ros2_topic, dds_type)
            self._writers[ros2_topic] = self._DW(self._dp, topic)
            logger.info("DDSWriter: %s", ros2_topic)
            return True
        except Exception as e:
            logger.warning("DDSWriter: failed %s: %s", ros2_topic, e)
            return False

    def write(self, ros2_topic: str, msg) -> bool:
        """Publish a message."""
        writer = self._writers.get(ros2_topic)
        if not writer:
            return False
        try:
            writer.write(msg)
            return True
        except Exception as e:
            logger.debug("DDSWriter write %s: %s", ros2_topic, e)
            return False

    def stop(self) -> None:
        self._writers.clear()
        self._dp = None


class ROS2TopicReader(DDSReader):
    """Convenience reader with typed subscribe helpers."""

    def on_odometry(self, topic: str, callback: Callable):
        if _HAS_CYCLONEDDS:
            self.subscribe(topic, DDS_Odometry, callback)

    def on_pointcloud2(self, topic: str, callback: Callable):
        if _HAS_CYCLONEDDS:
            self.subscribe(topic, DDS_PointCloud2, callback)

    def on_image(self, topic: str, callback: Callable):
        if _HAS_CYCLONEDDS:
            self.subscribe(topic, DDS_Image, callback)

    def on_camera_info(self, topic: str, callback: Callable):
        if _HAS_CYCLONEDDS:
            self.subscribe(topic, DDS_CameraInfo, callback)

    def on_occupancy_grid(self, topic: str, callback: Callable):
        if _HAS_CYCLONEDDS:
            self.subscribe(topic, DDS_OccupancyGrid, callback)

    def on_path(self, topic: str, callback: Callable):
        if _HAS_CYCLONEDDS:
            self.subscribe(topic, DDS_Path, callback)

    def on_tf(self, topic: str, callback: Callable):
        if _HAS_CYCLONEDDS:
            self.subscribe(topic, DDS_TFMessage, callback)

    def on_float32(self, topic: str, callback: Callable):
        if _HAS_CYCLONEDDS:
            self.subscribe(topic, DDS_Float32, callback)

    def on_string(self, topic: str, callback: Callable):
        if _HAS_CYCLONEDDS:
            self.subscribe(topic, DDS_String, callback)

    def on_twist_stamped(self, topic: str, callback: Callable):
        if _HAS_CYCLONEDDS:
            self.subscribe(topic, DDS_TwistStamped, callback)
