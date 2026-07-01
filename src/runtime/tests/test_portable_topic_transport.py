from __future__ import annotations

from runtime.msgs.geometry import Twist, Vector3
from runtime.msgs.nav import Odometry, Path as NavPath
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import PointCloud2
from runtime.portable.contracts import PortableCommandFrame, PortablePlanningFrame, PortableSensorFrame
from runtime.portable.topic_transport import (
    PortableTopicTransport,
    create_portable_publisher,
    create_portable_subscriber,
    create_portable_topic_transport,
)
from runtime.portable.topics import COMMAND_TOPICS, PLANNING_TOPICS, SENSOR_TOPICS
from runtime.runtime_interface import TOPICS
from runtime.transport.abc import TransportStrategy


def test_portable_topic_transport_uses_existing_local_transport_backend() -> None:
    transport = create_portable_topic_transport(TransportStrategy.LOCAL)
    received: list[Odometry] = []
    try:
        transport.subscribe(SENSOR_TOPICS["odometry"], received.append)
        odom = Odometry()

        transport.publish(SENSOR_TOPICS["odometry"], odom)

        assert received == [odom]
    finally:
        transport.close()


def test_portable_topic_transport_publishes_sensor_command_and_planning_frames() -> None:
    transport = PortableTopicTransport("local")
    received: dict[str, object] = {}
    try:
        transport.subscribe(SENSOR_TOPICS["map_cloud"], lambda msg: received.setdefault("cloud", msg))
        transport.subscribe(COMMAND_TOPICS["cmd_vel"], lambda msg: received.setdefault("cmd", msg))
        transport.subscribe(PLANNING_TOPICS["global_path"], lambda msg: received.setdefault("path", msg))
        cloud = PointCloud2(points=np.asarray([[1.0, 2.0, 3.0, 4.0]], dtype=np.float32))
        cmd = Twist(linear=Vector3(0.1, 0.0, 0.0))
        path = NavPath()

        transport.publish_sensor_frame(PortableSensorFrame(map_cloud=cloud))
        transport.publish_command_frame(PortableCommandFrame(cmd_vel=cmd))
        transport.publish_planning_frame(PortablePlanningFrame(global_path=path))

        assert received["cloud"] is cloud
        assert received["cmd"] is cmd
        assert received["path"] is path
    finally:
        transport.close()


def test_portable_publisher_and_subscriber_helpers_share_backend_strategy() -> None:
    received: list[Twist] = []
    subscriber_transport = create_portable_subscriber(COMMAND_TOPICS["cmd_vel"], received.append, strategy="local")
    publish = create_portable_publisher(COMMAND_TOPICS["cmd_vel"], strategy="local")
    publisher_transport = getattr(publish, "transport")
    try:
        cmd = Twist(linear=Vector3(0.2, 0.0, 0.0))

        publish(cmd)

        assert received == [cmd]
    finally:
        publisher_transport.close()
        subscriber_transport.close()
