from __future__ import annotations

from dataclasses import dataclass

from runtime.adapters.dds.endpoint_service import DDSEndpointService
from runtime.msgs.geometry import Pose, Twist, Vector3
from runtime.msgs.nav import Odometry
from runtime.runtime_interface import TOPICS


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
class DDS_TwistStamped:
    header: DDS_Header
    twist: DDS_Twist


@dataclass
class DDS_Odometry:
    header: DDS_Header
    child_frame_id: str
    pose: DDS_PoseWithCovariance
    twist: DDS_TwistWithCovariance


class _FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class _FakeTransport:
    def __init__(self) -> None:
        self.publishers = {}
        self.callbacks = {}

    def create_publisher(self, config):
        publisher = _FakePublisher()
        self.publishers[config.name] = publisher
        return publisher

    def create_subscriber(self, config, callback):
        self.callbacks[config.name] = callback
        return object()

    def emit(self, topic: str, msg) -> None:
        self.callbacks[topic](msg)


def _install_fake_dds_types(monkeypatch) -> None:
    import message.dds_types as dds_mod

    for cls in (
        DDS_Time,
        DDS_Header,
        DDS_Point,
        DDS_Quaternion,
        DDS_Pose,
        DDS_PoseWithCovariance,
        DDS_Vector3,
        DDS_Twist,
        DDS_TwistWithCovariance,
        DDS_TwistStamped,
        DDS_Odometry,
    ):
        monkeypatch.setattr(dds_mod, cls.__name__, cls, raising=False)


def test_dds_endpoint_service_publishes_localization_snapshot(monkeypatch) -> None:
    _install_fake_dds_types(monkeypatch)
    transport = _FakeTransport()
    service = DDSEndpointService(transport=transport, subscribe_lingtu_outputs=False)
    service.start()

    service.publish_localization_snapshot(
        odometry=Odometry(pose=Pose(1.0, 2.0, 0.0), ts=12.0)
    )

    msg = transport.publishers[TOPICS.odometry].messages[-1]
    assert isinstance(msg, DDS_Odometry)
    assert msg.header.frame_id == "odom"
    assert msg.pose.pose.position.x == 1.0
    assert service.health()["publish_counts"][TOPICS.odometry] == 1


def test_dds_endpoint_service_consumes_cmd_vel() -> None:
    events = []
    transport = _FakeTransport()
    service = DDSEndpointService(transport=transport, on_lingtu_message=events.append)
    service.start()

    transport.emit(
        TOPICS.cmd_vel,
        DDS_TwistStamped(
            header=DDS_Header(DDS_Time(1, 0), "body"),
            twist=DDS_Twist(
                linear=DDS_Vector3(0.4, 0.0, 0.0),
                angular=DDS_Vector3(0.0, 0.0, 0.2),
            ),
        ),
    )

    assert events[-1].topic == TOPICS.cmd_vel
    assert isinstance(events[-1].message, Twist)
    assert events[-1].message.linear.x == 0.4
    assert events[-1].message.angular.z == 0.2
    assert service.health()["receive_counts"][TOPICS.cmd_vel] == 1
