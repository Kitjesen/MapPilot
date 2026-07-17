from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from message.dds import from_dds_message, to_dds_message, topic_spec
from message.livox_frame import POINT_DTYPE, LivoxPointFrame
from runtime.msgs.geometry import Transform, Vector3
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat
from runtime.runtime_interface import TOPICS
from runtime.tf import TF_TOPIC, TFMessage
from runtime.transport.adapter import TransportAdapter


@dataclass
class FakeDDSMessage:
    value: str


@dataclass
class FakeTime:
    sec: int
    nanosec: int


@dataclass
class FakeHeader:
    stamp: FakeTime
    frame_id: str


@dataclass
class FakeLivoxPoint:
    offset_time: int
    x: float
    y: float
    z: float
    reflectivity: int
    tag: int
    line: int


@dataclass
class FakeLivoxCustomMsg:
    header: FakeHeader
    timebase: int
    point_num: int
    lidar_id: int
    rsvd: list[int]
    points: list[FakeLivoxPoint]


class _FakePublisher:
    def __init__(self, config) -> None:
        self.config = config
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)

    def close(self) -> None:
        pass


class _FakeSubscriber:
    def start(self) -> None:
        pass

    def close(self) -> None:
        pass


class _FakeBackend:
    name = "dds"

    def __init__(self) -> None:
        self.publishers = {}
        self.callbacks = {}

    def create_publisher(self, config):
        publisher = _FakePublisher(config)
        self.publishers[config.name] = publisher
        return publisher

    def create_subscriber(self, config, callback):
        self.callbacks[config.name] = callback
        return _FakeSubscriber()

    def close(self) -> None:
        pass


def test_transport_adapter_uses_typed_dds_conversion_for_registered_topics() -> None:
    backend = _FakeBackend()
    adapter = TransportAdapter(
        backend,
        backend_msg_type=bytes,
        topic_msg_type=lambda topic: FakeDDSMessage if topic == "/typed" else None,
        topic_encoder=lambda topic, msg: FakeDDSMessage(str(msg)),
        topic_decoder=lambda topic, msg: f"decoded:{msg.value}",
    )
    received = []

    adapter.subscribe("/typed", received.append)
    adapter.publish("/typed", "scan")
    backend.callbacks["/typed"](FakeDDSMessage("imu"))

    publisher = backend.publishers["/typed"]
    assert publisher.config.msg_type is FakeDDSMessage
    assert publisher.messages == [FakeDDSMessage("scan")]
    assert received == ["decoded:imu"]


def test_livox_raw_frame_round_trips_through_registered_dds_payload(monkeypatch) -> None:
    points = np.zeros(1, dtype=POINT_DTYPE)
    points["x"] = [1.0]
    points["y"] = [2.0]
    points["z"] = [3.0]
    points["intensity"] = [42.0]
    points["offset_time_ns"] = [123]
    points["line"] = [2]
    points["tag"] = [7]
    frame = LivoxPointFrame(points=points, timestamp_ns=1_000_000_000, sequence=5)

    dds_msg = to_dds_message(TOPICS.raw_lidar_points, frame)
    roundtrip = from_dds_message(TOPICS.raw_lidar_points, dds_msg)

    assert dds_msg.timebase == 1_000_000_000
    assert dds_msg.point_num == 1
    assert dds_msg.points[0].offset_time == 123
    assert dds_msg.points[0].line == 2
    assert dds_msg.points[0].tag == 7
    assert roundtrip.point_count == 1
    assert float(roundtrip.points["x"][0]) == 1.0
    assert int(roundtrip.points["offset_time_ns"][0]) == 123


def test_camera_info_round_trips_through_registered_dds_payload() -> None:
    info = CameraIntrinsics(
        fx=10.0,
        fy=11.0,
        cx=1.0,
        cy=2.0,
        width=640,
        height=480,
        depth_scale=0.001,
        dist_k1=0.1,
        dist_k2=0.2,
        dist_p1=0.3,
        dist_p2=0.4,
        dist_k3=0.5,
        ts=123.5,
        frame_id="camera_link",
    )

    dds_msg = to_dds_message(TOPICS.camera_info, info)
    roundtrip = from_dds_message(TOPICS.camera_info, dds_msg)

    assert topic_spec(TOPICS.camera_info).cpp_type == "lingtu::dds::CameraInfo"
    assert dds_msg.header.frame_id == "camera_link"
    assert dds_msg.depth_scale == 0.001
    assert dds_msg.d == [0.1, 0.2, 0.3, 0.4, 0.5]
    assert roundtrip.fx == 10.0
    assert roundtrip.fy == 11.0
    assert roundtrip.depth_scale == 0.001
    assert roundtrip.frame_id == "camera_link"
    assert roundtrip.ts == 123.5
    assert roundtrip.D == [0.1, 0.2, 0.3, 0.4, 0.5]


def test_camera_images_round_trip_through_registered_dds_payloads() -> None:
    color = Image(
        data=np.asarray([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8),
        format=ImageFormat.RGB,
        ts=123.5,
        frame_id="camera_link",
    )
    depth = Image(
        data=np.asarray([[1000, 2000]], dtype=np.uint16),
        format=ImageFormat.DEPTH_U16,
        ts=124.5,
        frame_id="camera_link",
    )

    color_dds = to_dds_message(TOPICS.camera_color, color)
    depth_dds = to_dds_message(TOPICS.camera_depth, depth)
    color_roundtrip = from_dds_message(TOPICS.camera_color, color_dds)
    depth_roundtrip = from_dds_message(TOPICS.camera_depth, depth_dds)

    assert topic_spec(TOPICS.camera_color).cpp_type == "lingtu::dds::Image"
    assert topic_spec(TOPICS.camera_depth).cpp_type == "lingtu::dds::Image"
    assert color_dds.encoding == "rgb8"
    assert color_dds.step == 6
    assert color_dds.data == [1, 2, 3, 4, 5, 6]
    assert depth_dds.encoding == "16UC1"
    assert depth_dds.step == 4
    assert depth_dds.data == [232, 3, 208, 7]
    assert color_roundtrip.format is ImageFormat.RGB
    assert color_roundtrip.data.shape == (1, 2, 3)
    assert color_roundtrip.data[0, 1].tolist() == [4, 5, 6]
    assert color_roundtrip.ts == 123.5
    assert depth_roundtrip.format is ImageFormat.DEPTH_U16
    assert depth_roundtrip.data.shape == (1, 2)
    assert int(depth_roundtrip.data[0, 0]) == 1000
    assert depth_roundtrip.ts == 124.5


def test_dds_endpoint_service_accepts_livox_point_frame() -> None:
    from message.dds_codec import to_dds_livox_custom_msg

    points = np.zeros(1, dtype=POINT_DTYPE)
    points["x"] = [1.0]
    points["y"] = [2.0]
    points["z"] = [3.0]
    points["intensity"] = [42.0]
    points["offset_time_ns"] = [123]
    points["line"] = [2]
    points["tag"] = [7]
    frame = LivoxPointFrame(points=points, timestamp_ns=1_000_000_000, sequence=5)
    frame.frame_id = "lidar_link"

    msg = to_dds_livox_custom_msg(frame)

    assert msg.header.frame_id == "lidar_link"
    assert msg.timebase == 1_000_000_000
    assert msg.point_num == 1
    assert msg.points[0].offset_time == 123
    assert msg.points[0].line == 2
    assert msg.points[0].tag == 7


def test_livox_dds_contract_does_not_import_driver_layer() -> None:
    import message.dds_types.livox as livox_contract

    source = Path(livox_contract.__file__).read_text(encoding="utf-8")

    assert "drivers.real" not in source


def test_tf_message_round_trips_through_registered_dds_payload() -> None:
    msg = TFMessage(
        (
            Transform(
                translation=Vector3(1.0, 2.0, 3.0),
                frame_id="map",
                child_frame_id="odom",
                ts=7.5,
            ),
        )
    )

    dds_msg = to_dds_message(TF_TOPIC, msg)
    roundtrip = from_dds_message(TF_TOPIC, dds_msg)

    assert topic_spec(TF_TOPIC).cpp_type == "lingtu::dds::TFMessage"
    assert dds_msg.transforms[0].header.frame_id == "map"
    assert roundtrip.transforms[0].child_frame_id == "odom"
    assert roundtrip.transforms[0].translation.x == 1.0
    assert roundtrip.transforms[0].ts == 7.5
