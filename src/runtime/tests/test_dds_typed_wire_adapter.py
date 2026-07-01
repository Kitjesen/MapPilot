from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from message.livox_frame import POINT_DTYPE, LivoxPointFrame
from message.dds import from_dds_message, to_dds_message
from runtime.msgs.numpy_compat import np
from runtime.runtime_interface import TOPICS
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


def test_livox_dds_contract_does_not_import_driver_layer() -> None:
    import message.dds_types.livox as livox_contract

    source = Path(livox_contract.__file__).read_text(encoding="utf-8")

    assert "drivers.real" not in source
