from __future__ import annotations

import ast
from pathlib import Path

from runtime.msgs.geometry import Twist, Vector3
from runtime.msgs.nav import Odometry, Path as NavPath
from runtime.msgs.sensor import PointCloud2
from runtime.msgs.numpy_compat import np
from runtime.portable.contracts import PortableCommandFrame, PortablePlanningFrame, PortableSensorFrame
from runtime.portable.topics import (
    COMMAND_TOPICS,
    PLANNING_TOPICS,
    SENSOR_TOPICS,
    LocalTopicHub,
    portable_topic_contracts,
)
from runtime.runtime_interface import TOPICS


FORBIDDEN_IMPORT_ROOTS = {
    "mujoco",
    "rclpy",
    "sensor_msgs",
    "nav_msgs",
    "geometry_msgs",
    "std_msgs",
    "tf2_ros",
    "launch_ros",
    "pcl",
    "pcl_conversions",
    "pcl_ros",
    "fastapi",
    "torch",
    "brainstem_api",
}


def test_local_topic_hub_publish_subscribe_and_last() -> None:
    hub = LocalTopicHub()
    received: list[Odometry] = []
    unsubscribe = hub.subscribe(SENSOR_TOPICS["odometry"], received.append)
    odom = Odometry()

    count = hub.publish(SENSOR_TOPICS["odometry"], odom)

    assert count == 1
    assert received == [odom]
    assert hub.last(TOPICS.odometry) is odom
    unsubscribe()
    assert hub.publish(SENSOR_TOPICS["odometry"], Odometry()) == 0


def test_local_topic_hub_publishes_frames_to_canonical_topics() -> None:
    hub = LocalTopicHub()
    points = PointCloud2(points=np.asarray([[1.0, 2.0, 3.0, 4.0]], dtype=np.float32))
    cmd = Twist(linear=Vector3(0.2, 0.0, 0.0))
    global_path = NavPath()

    hub.publish_sensor_frame(PortableSensorFrame(odometry=Odometry(), map_cloud=points))
    hub.publish_command_frame(PortableCommandFrame(cmd_vel=cmd, stop_signal=0))
    hub.publish_planning_frame(PortablePlanningFrame(global_path=global_path))

    assert hub.last(TOPICS.odometry) is not None
    assert hub.last(TOPICS.map_cloud) is points
    assert hub.last(TOPICS.cmd_vel) is cmd
    assert hub.last(TOPICS.stop) == 0
    assert hub.last(TOPICS.global_path) is global_path


def test_portable_topic_contracts_are_json_ready() -> None:
    contracts = portable_topic_contracts()

    assert contracts["odometry"]["topic"] == TOPICS.odometry
    assert contracts["cmd_vel"]["direction"] == "sink"
    assert contracts["global_path"]["payload_type"] == "Path"


def test_portable_topics_do_not_import_adapter_or_heavy_modules() -> None:
    path = Path("src/runtime/portable/topics.py")
    tree = ast.parse(path.read_text(encoding="utf-8"))
    imports: set[str] = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            imports.update(alias.name.split(".", 1)[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module.split(".", 1)[0])

    assert imports.isdisjoint(FORBIDDEN_IMPORT_ROOTS)
