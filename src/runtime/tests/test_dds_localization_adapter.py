from __future__ import annotations

import json
from dataclasses import dataclass

from runtime.adapters.dds.localization_adapter import DDSLocalizationAdapterModule
from runtime.blueprints.stacks.slam import slam
from runtime.msgs.geometry import Transform, Vector3
from runtime.msgs.numpy_compat import np
from runtime.runtime_interface import TOPICS
from runtime.tf import FrameTree, TFMessage, TF_TOPIC


@dataclass
class DDS_Time:
    sec: int = 0
    nanosec: int = 0


@dataclass
class DDS_Header:
    stamp: DDS_Time
    frame_id: str


@dataclass
class DDS_Point:
    x: float
    y: float
    z: float


@dataclass
class DDS_Quaternion:
    x: float
    y: float
    z: float
    w: float


@dataclass
class DDS_Pose:
    position: DDS_Point
    orientation: DDS_Quaternion


@dataclass
class DDS_PoseWithCovariance:
    pose: DDS_Pose
    covariance: list[float]


@dataclass
class DDS_Vector3:
    x: float
    y: float
    z: float


@dataclass
class DDS_Twist:
    linear: DDS_Vector3
    angular: DDS_Vector3


@dataclass
class DDS_TwistWithCovariance:
    twist: DDS_Twist
    covariance: list[float]


@dataclass
class DDS_Odometry:
    header: DDS_Header
    child_frame_id: str
    pose: DDS_PoseWithCovariance
    twist: DDS_TwistWithCovariance


@dataclass
class DDS_PointField:
    name: str
    offset: int
    datatype: int
    count: int = 1


@dataclass
class DDS_PointCloud2:
    header: DDS_Header
    height: int
    width: int
    fields: list[DDS_PointField]
    is_bigendian: bool
    point_step: int
    row_step: int
    data: bytes
    is_dense: bool


@dataclass
class DDS_Float32:
    data: float


@dataclass
class DDS_String:
    data: str


class _FakeSubscriber:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.closed = False

    def close(self) -> None:
        self.closed = True


class _FakeDDSTransport:
    def __init__(self) -> None:
        self.callbacks = {}

    def create_subscriber(self, config, callback):
        self.callbacks[config.name] = callback
        return _FakeSubscriber(config.name)

    def emit(self, topic: str, msg) -> None:
        self.callbacks[topic](msg)


def test_dds_localization_adapter_publishes_odometry_and_status() -> None:
    transport = _FakeDDSTransport()
    adapter = DDSLocalizationAdapterModule(transport=transport)
    odometry_seen = []
    status_seen = []
    adapter.odometry.subscribe(odometry_seen.append)
    adapter.localization_status.subscribe(status_seen.append)
    adapter.setup()

    transport.emit(
        TOPICS.odometry,
        DDS_Odometry(
            header=DDS_Header(DDS_Time(42, 0), "odom"),
            child_frame_id="body",
            pose=DDS_PoseWithCovariance(
                DDS_Pose(
                    DDS_Point(1.0, 2.0, 0.0),
                    DDS_Quaternion(0.0, 0.0, 0.0, 1.0),
                ),
                [0.0] * 36,
            ),
            twist=DDS_TwistWithCovariance(
                DDS_Twist(DDS_Vector3(0.3, 0.0, 0.0), DDS_Vector3(0.0, 0.0, 0.1)),
                [0.0] * 36,
            ),
        ),
    )

    assert odometry_seen[-1].x == 1.0
    assert odometry_seen[-1].ts == 42.0
    assert status_seen[-1]["backend"] == "dds_endpoint"
    assert adapter.health()["message_counts"][TOPICS.odometry] == 1


def test_dds_localization_adapter_publishes_map_cloud() -> None:
    transport = _FakeDDSTransport()
    adapter = DDSLocalizationAdapterModule(transport=transport)
    clouds = []
    adapter.map_cloud.subscribe(clouds.append)
    adapter.setup()
    points = np.asarray([[1.0, 2.0, 3.0, 0.5], [4.0, 5.0, 6.0, 0.7]], dtype=np.float32)

    transport.emit(
        TOPICS.map_cloud,
        DDS_PointCloud2(
            header=DDS_Header(DDS_Time(5, 0), "map"),
            height=1,
            width=2,
            fields=[
                DDS_PointField("x", 0, 7),
                DDS_PointField("y", 4, 7),
                DDS_PointField("z", 8, 7),
                DDS_PointField("intensity", 12, 7),
            ],
            is_bigendian=False,
            point_step=16,
            row_step=32,
            data=points.tobytes(),
            is_dense=True,
        ),
    )

    assert clouds[-1].frame_id == "map"
    assert np.allclose(clouds[-1].points, points)


def test_dds_localization_adapter_updates_frame_tree_from_tf() -> None:
    transport = _FakeDDSTransport()
    tree = FrameTree()
    adapter = DDSLocalizationAdapterModule(transport=transport, frame_tree=tree)
    tf_seen = []
    adapter.map_odom_tf.subscribe(tf_seen.append)
    adapter.setup()

    transport.emit(
        TF_TOPIC,
        TFMessage(
            (
                Transform(
                    translation=Vector3(1.0, 2.0, 0.0),
                    frame_id="map",
                    child_frame_id="odom",
                    ts=9.0,
                ),
            )
        ),
    )

    assert tree.lookup("map", "odom", ts=9.0).translation.x == 1.0
    assert tf_seen[-1]["child_frame_id"] == "odom"
    assert tf_seen[-1]["tx"] == 1.0
    assert adapter.health()["message_counts"][TF_TOPIC] == 1


def test_dds_localization_adapter_updates_frame_tree_from_health_tf() -> None:
    transport = _FakeDDSTransport()
    tree = FrameTree()
    adapter = DDSLocalizationAdapterModule(transport=transport, frame_tree=tree)
    adapter.setup()

    transport.emit(
        TOPICS.localization_health,
        DDS_String(
            json.dumps(
                {
                    "state": "TRACKING",
                    "map_odom_tf": {
                        "valid": True,
                        "tx": 3.0,
                        "ty": 0.0,
                        "tz": 0.0,
                        "qx": 0.0,
                        "qy": 0.0,
                        "qz": 0.0,
                        "qw": 1.0,
                        "ts": 11.0,
                    },
                }
            )
        ),
    )

    assert tree.lookup("map", "odom", ts=11.0).translation.x == 3.0


def test_slam_stack_can_select_dds_localization_adapter() -> None:
    bp = slam("bridge", enable_visual_backup=False, localization_adapter="dds_endpoint")

    assert bp._entries[0].name == "SlamAdapterModule"
    assert bp._entries[0].module_cls is DDSLocalizationAdapterModule
