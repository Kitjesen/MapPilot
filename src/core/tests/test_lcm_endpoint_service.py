from __future__ import annotations

import pytest

from compat.lcm.contracts import THUNDER_FIELD_LCM_CONTRACT_NAME, binding_for_topic
from compat.lcm.endpoint_codec import dumps_endpoint_message, loads_endpoint_message
from compat.lcm.endpoint_service import LCMEndpointEvent, LCMEndpointService
from core.msgs.geometry import Pose, Twist, Vector3
from core.msgs.nav import Odometry, Path
from core.msgs.numpy_compat import np
from core.msgs.sensor import Imu, PointCloud2
from core.runtime_interface import TOPICS


class _FakeLCMTransport:
    def __init__(self) -> None:
        self.published: list[tuple[str, bytes]] = []
        self.callbacks = {}
        self.closed = False

    def publish(self, channel: str, payload: bytes) -> None:
        self.published.append((channel, payload))

    def subscribe(self, channel, callback):
        self.callbacks[channel] = callback
        return _FakeSubscription(channel)

    def emit(self, channel, payload) -> None:
        self.callbacks[channel](payload)

    def close(self) -> None:
        self.closed = True


class _FakeSubscription:
    def __init__(self, channel: str) -> None:
        self.channel = channel
        self.closed = False

    def close(self) -> None:
        self.closed = True


def _decoded_by_topic(transport: _FakeLCMTransport, topic: str):
    binding = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT_NAME, topic)
    for channel, payload in reversed(transport.published):
        if channel == binding.channel:
            return loads_endpoint_message(binding, payload)
    raise AssertionError(f"missing payload for {topic}")


def test_lcm_endpoint_service_publishes_sensor_and_localization_snapshots() -> None:
    transport = _FakeLCMTransport()
    service = LCMEndpointService(transport=transport, subscribe_lingtu_outputs=False)
    service.start()

    lidar = PointCloud2(points=np.asarray([[1.0, 2.0, 3.0]], dtype=np.float32), frame_id="lidar_link")
    imu = Imu(frame_id="lidar_link")
    odom = Odometry(pose=Pose(1.0, 2.0, 0.0), frame_id="odom")
    registered = PointCloud2(points=np.asarray([[0.0, 0.0, 0.0]], dtype=np.float32), frame_id="body")
    map_cloud = PointCloud2(points=np.asarray([[4.0, 5.0, 6.0]], dtype=np.float32), frame_id="map")

    assert service.publish_sensor_snapshot(lidar_scan=lidar, imu=imu) == 2
    assert service.publish_localization_snapshot(
        odometry=odom,
        registered_cloud=registered,
        map_cloud=map_cloud,
        localization_health={"state": "TRACKING", "quality": 0.9},
        localization_quality=0.9,
    ) == 5

    assert _decoded_by_topic(transport, TOPICS.lidar_scan).frame_id == "lidar_link"
    assert _decoded_by_topic(transport, TOPICS.imu).frame_id == "lidar_link"
    assert _decoded_by_topic(transport, TOPICS.odometry).x == 1.0
    assert _decoded_by_topic(transport, TOPICS.registered_cloud).frame_id == "body"
    assert _decoded_by_topic(transport, TOPICS.map_cloud).frame_id == "map"
    assert _decoded_by_topic(transport, TOPICS.localization_health)["state"] == "TRACKING"
    assert _decoded_by_topic(transport, TOPICS.localization_quality) == 0.9
    assert service.health()["publish_counts"][TOPICS.map_cloud] == 1


def test_lcm_endpoint_service_consumes_lingtu_navigation_outputs() -> None:
    transport = _FakeLCMTransport()
    events: list[LCMEndpointEvent] = []
    service = LCMEndpointService(transport=transport, on_lingtu_message=events.append)
    service.start()

    twist_binding = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT_NAME, TOPICS.cmd_vel)
    path_binding = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT_NAME, TOPICS.local_path)
    transport.emit(
        twist_binding.channel,
        dumps_endpoint_message(
            twist_binding,
            Twist(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.1)),
        ),
    )
    transport.emit(path_binding.channel, dumps_endpoint_message(path_binding, Path(frame_id="map")))

    assert [event.topic for event in events] == [TOPICS.cmd_vel, TOPICS.local_path]
    assert isinstance(events[0].message, Twist)
    assert events[0].message.linear.x == 0.2
    assert isinstance(events[1].message, Path)
    assert service.health()["receive_counts"] == {
        TOPICS.cmd_vel: 1,
        TOPICS.local_path: 1,
    }

    service.stop()
    assert transport.closed is False


def test_lcm_endpoint_service_rejects_wrong_publish_direction() -> None:
    service = LCMEndpointService(transport=_FakeLCMTransport(), subscribe_lingtu_outputs=False)

    with pytest.raises(ValueError, match="endpoint-to-LingTu"):
        service.publish_to_lingtu(TOPICS.cmd_vel, Twist())


def test_lcm_endpoint_service_health_describes_contract_channels() -> None:
    service = LCMEndpointService(transport=_FakeLCMTransport())

    health = service.health()

    assert health["endpoint_contract"] == THUNDER_FIELD_LCM_CONTRACT_NAME
    assert health["endpoint_to_lingtu_channels"][TOPICS.odometry] == binding_for_topic(
        THUNDER_FIELD_LCM_CONTRACT_NAME,
        TOPICS.odometry,
    ).channel
    assert health["lingtu_to_endpoint_channels"][TOPICS.cmd_vel] == binding_for_topic(
        THUNDER_FIELD_LCM_CONTRACT_NAME,
        TOPICS.cmd_vel,
    ).channel
