from __future__ import annotations

from dataclasses import dataclass

from message.dds import dds_type_for_topic
from runtime.adapters.dds.map_output import DDSMapOutModule
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
class DDS_MapMetaData:
    map_load_time: DDS_Time
    resolution: float
    width: int
    height: int
    origin: DDS_Pose


@dataclass
class DDS_OccupancyGrid:
    header: DDS_Header
    info: DDS_MapMetaData
    data: list[int]


class _FakePublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)

    def close(self) -> None:
        pass


class _FakeDDSTransport:
    def __init__(self) -> None:
        self.publishers: dict[str, _FakePublisher] = {}
        self.closed = False

    def create_publisher(self, config):
        publisher = _FakePublisher(config.name)
        self.publishers[config.name] = publisher
        return publisher

    def close(self) -> None:
        self.closed = True


def _install_fake_dds_types(monkeypatch) -> None:
    import message.dds_types as dds_mod

    for cls in (
        DDS_Time,
        DDS_Header,
        DDS_Point,
        DDS_Quaternion,
        DDS_Pose,
        DDS_MapMetaData,
        DDS_OccupancyGrid,
    ):
        monkeypatch.setattr(dds_mod, cls.__name__, cls, raising=False)


def test_dds_topic_registry_covers_exploration_grid() -> None:
    assert dds_type_for_topic(TOPICS.exploration_grid).__name__ == "OccupancyGrid"


def test_dds_map_out_publishes_typed_occupancy_grid(monkeypatch) -> None:
    _install_fake_dds_types(monkeypatch)
    transport = _FakeDDSTransport()
    map_out = DDSMapOutModule(transport=transport, default_frame_id="map")
    map_out.setup()

    map_out.exploration_grid._deliver(
        {
            "grid": [[0, 101], [-2, 42]],
            "resolution": 0.25,
            "origin": (-1.0, -2.0),
            "frame_id": "map",
            "ts": 12.5,
        }
    )

    msg = transport.publishers[TOPICS.exploration_grid].messages[-1]
    assert isinstance(msg, DDS_OccupancyGrid)
    assert msg.header.frame_id == "map"
    assert msg.header.stamp.sec == 12
    assert msg.info.width == 2
    assert msg.info.height == 2
    assert msg.info.resolution == 0.25
    assert msg.info.origin.position.x == -1.0
    assert msg.info.origin.position.y == -2.0
    assert msg.data == [0, 100, -1, 42]
    assert map_out.health()["publish_counts"] == {TOPICS.exploration_grid: 1}
