from __future__ import annotations

import sys
import threading
import time
import types

import numpy as np

from compat.ros2.sim_driver import ROS2SimDriverModule
from core.msgs.geometry import Twist, Vector3


class FakeTwistStamped:
    def __init__(self) -> None:
        self.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=0, nanosec=0),
            frame_id="",
        )
        self.twist = types.SimpleNamespace(
            linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
        )


class FakePublisher:
    def __init__(self, msg_type, topic, qos) -> None:
        self.msg_type = msg_type
        self.topic = topic
        self.qos = qos
        self.published = []

    def publish(self, msg) -> None:
        self.published.append(msg)


class FakeExecutor:
    def __init__(self) -> None:
        self.added = []
        self.removed = []

    def add_node(self, node) -> None:
        self.added.append(node)

    def remove_node(self, node) -> None:
        self.removed.append(node)


class FakeNode:
    instances = []
    fail_subscription = False

    def __init__(self, name: str) -> None:
        self.name = name
        self.subscriptions = []
        self.publishers = []
        self.destroyed = False
        type(self).instances.append(self)

    def create_subscription(self, msg_type, topic, callback, qos, **kwargs):
        if self.fail_subscription:
            raise RuntimeError("subscription failed")
        self.subscriptions.append((msg_type, topic, callback, qos, kwargs))
        return object()

    def create_publisher(self, msg_type, topic, qos):
        publisher = FakePublisher(msg_type, topic, qos)
        self.publishers.append(publisher)
        return publisher

    def destroy_node(self) -> None:
        self.destroyed = True


class QoSProfile:
    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs


class ReliabilityPolicy:
    RELIABLE = object()
    BEST_EFFORT = object()


class MutuallyExclusiveCallbackGroup:
    pass


def _module(name: str, **attrs):
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    return module


def _install_fake_ros(monkeypatch, executor: FakeExecutor) -> None:
    FakeNode.instances = []
    FakeNode.fail_subscription = False

    geometry_msg = _module(
        "geometry_msgs.msg",
        TwistStamped=FakeTwistStamped,
        PoseStamped=type("PoseStamped", (), {}),
    )
    nav_msg = _module("nav_msgs.msg", Odometry=type("Odometry", (), {}))
    sensor_msg = _module(
        "sensor_msgs.msg",
        CameraInfo=type("CameraInfo", (), {}),
        PointCloud2=type("PointCloud2", (), {}),
        Image=type("Image", (), {}),
    )
    builtin_msg = _module(
        "builtin_interfaces.msg",
        Time=type("Time", (), {}),
    )

    monkeypatch.setitem(
        sys.modules, "builtin_interfaces", _module("builtin_interfaces", msg=builtin_msg)
    )
    monkeypatch.setitem(sys.modules, "builtin_interfaces.msg", builtin_msg)
    monkeypatch.setitem(sys.modules, "geometry_msgs", _module("geometry_msgs", msg=geometry_msg))
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geometry_msg)
    monkeypatch.setitem(sys.modules, "nav_msgs", _module("nav_msgs", msg=nav_msg))
    monkeypatch.setitem(sys.modules, "nav_msgs.msg", nav_msg)
    monkeypatch.setitem(sys.modules, "sensor_msgs", _module("sensor_msgs", msg=sensor_msg))
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor_msg)
    monkeypatch.setitem(sys.modules, "rclpy", _module("rclpy", ok=lambda: True))
    monkeypatch.setitem(
        sys.modules,
        "rclpy.callback_groups",
        _module(
            "rclpy.callback_groups",
            MutuallyExclusiveCallbackGroup=MutuallyExclusiveCallbackGroup,
        ),
    )
    monkeypatch.setitem(sys.modules, "rclpy.node", _module("rclpy.node", Node=FakeNode))
    monkeypatch.setitem(
        sys.modules,
        "rclpy.qos",
        _module("rclpy.qos", QoSProfile=QoSProfile, ReliabilityPolicy=ReliabilityPolicy),
    )

    import compat.ros2.context as ros2_context

    monkeypatch.setattr(ros2_context, "ensure_rclpy", lambda: None)
    monkeypatch.setattr(ros2_context, "get_shared_executor", lambda: executor)


def _pointcloud_msg(frame_id: str = "body"):
    points = np.array([[1.0, 2.0, 3.0, 0.0]], dtype=np.float32)
    return types.SimpleNamespace(
        width=1,
        height=1,
        point_step=16,
        data=points.tobytes(),
        header=types.SimpleNamespace(frame_id=frame_id),
    )


def test_ros2_sim_driver_uses_shared_executor(monkeypatch):
    executor = FakeExecutor()
    _install_fake_ros(monkeypatch, executor)

    module = ROS2SimDriverModule()
    module.setup()

    node = FakeNode.instances[0]
    assert executor.added == [node]
    assert module._executor is executor
    assert module._node is node
    assert module._pub_cmd_vel is not None
    assert not hasattr(module, "_spin_thread")
    qos_by_topic = {topic: qos for _, topic, _, qos, _ in node.subscriptions}
    assert qos_by_topic["/nav/odometry"].kwargs["reliability"] is ReliabilityPolicy.RELIABLE
    assert qos_by_topic["/nav/registered_cloud"].kwargs["reliability"] is ReliabilityPolicy.BEST_EFFORT
    assert qos_by_topic["/nav/registered_cloud"].kwargs["depth"] == 1
    assert qos_by_topic["/nav/map_cloud"].kwargs["reliability"] is ReliabilityPolicy.BEST_EFFORT
    assert qos_by_topic["/nav/map_cloud"].kwargs["depth"] == 1
    assert qos_by_topic["/nav/goal_pose"].kwargs["reliability"] is ReliabilityPolicy.RELIABLE
    assert module._pub_cmd_vel.qos.kwargs["reliability"] is ReliabilityPolicy.RELIABLE

    module.start()
    assert module.health()["ros2"]["running"] is True

    module.stop()
    assert executor.removed == [node]
    assert node.destroyed is True
    assert module._node is None
    assert module._executor is None
    assert module._pub_cmd_vel is None


def test_ros2_sim_driver_cleans_up_failed_shared_executor_setup(monkeypatch):
    executor = FakeExecutor()
    _install_fake_ros(monkeypatch, executor)
    FakeNode.fail_subscription = True

    module = ROS2SimDriverModule()
    alive = []
    module.alive.subscribe(alive.append)

    module.setup()

    node = FakeNode.instances[0]
    assert executor.added == [node]
    assert executor.removed == [node]
    assert node.destroyed is True
    assert module._node is None
    assert module._executor is None
    assert module._pub_cmd_vel is None

    module.start()
    assert alive[-1] is False
    assert module.health()["ros2"]["running"] is False


def test_ros2_sim_driver_cloud_callback_does_not_block_shared_executor(monkeypatch):
    executor = FakeExecutor()
    _install_fake_ros(monkeypatch, executor)

    module = ROS2SimDriverModule()
    module.setup()
    module.start()

    callback_entered = threading.Event()
    release_callback = threading.Event()
    callback_done = threading.Event()

    def slow_consumer(_cloud):
        callback_entered.set()
        assert release_callback.wait(timeout=1.0)
        callback_done.set()

    module.map_cloud._add_callback(slow_consumer)

    t0 = time.perf_counter()
    module._on_ros2_map_cloud(_pointcloud_msg(frame_id="odom"))
    elapsed_ms = (time.perf_counter() - t0) * 1000.0

    assert elapsed_ms < 50.0
    assert callback_entered.wait(timeout=1.0)

    module._on_ros2_map_cloud(_pointcloud_msg(frame_id="odom"))
    assert module._cloud_worker_drops == 1

    release_callback.set()
    assert callback_done.wait(timeout=1.0)
    if module._cloud_worker_thread is not None:
        module._cloud_worker_thread.join(timeout=1.0)
    assert module.health()["ros2"]["cloud_worker_drops"] == 1
    module.stop()


def test_ros2_sim_driver_separates_registered_cloud_from_map_cloud(monkeypatch):
    executor = FakeExecutor()
    _install_fake_ros(monkeypatch, executor)

    module = ROS2SimDriverModule()
    module.setup()
    module.start()

    lidar_clouds = []
    map_clouds = []
    module.lidar_cloud.subscribe(lidar_clouds.append)
    module.map_cloud.subscribe(map_clouds.append)

    module._on_ros2_registered_cloud(_pointcloud_msg())
    if module._cloud_worker_thread is not None:
        module._cloud_worker_thread.join(timeout=1.0)
    assert len(lidar_clouds) == 1
    assert map_clouds == []

    module._on_ros2_map_cloud(_pointcloud_msg(frame_id="odom"))
    if module._cloud_worker_thread is not None:
        module._cloud_worker_thread.join(timeout=1.0)
    assert len(map_clouds) == 1

    module.stop()


def test_ros2_sim_driver_stop_clear_reenables_cmd_vel(monkeypatch):
    executor = FakeExecutor()
    _install_fake_ros(monkeypatch, executor)

    module = ROS2SimDriverModule()
    module.setup()

    publisher = module._pub_cmd_vel
    assert publisher is not None

    module._on_cmd_vel(Twist(linear=Vector3(x=0.4), angular=Vector3(z=0.2)))
    assert publisher.published[-1].twist.linear.x == 0.4
    assert publisher.published[-1].twist.angular.z == 0.2

    module._on_stop(1)
    assert module._stopped is True
    assert publisher.published[-1].twist.linear.x == 0.0
    assert publisher.published[-1].twist.angular.z == 0.0

    publish_count = len(publisher.published)
    module._on_cmd_vel(Twist(linear=Vector3(x=0.7), angular=Vector3(z=0.3)))
    assert len(publisher.published) == publish_count

    module._on_stop(0)
    assert module._stopped is False
    module._on_cmd_vel(Twist(linear=Vector3(x=0.3), angular=Vector3(z=-0.1)))
    assert publisher.published[-1].twist.linear.x == 0.3
    assert publisher.published[-1].twist.angular.z == -0.1

    module.stop()


def test_ros2_sim_driver_non_latching_stop_allows_next_cmd_vel(monkeypatch):
    executor = FakeExecutor()
    _install_fake_ros(monkeypatch, executor)

    module = ROS2SimDriverModule(latch_stop_signal=False)
    module.setup()

    publisher = module._pub_cmd_vel
    assert publisher is not None

    module._on_cmd_vel(Twist(linear=Vector3(x=0.4), angular=Vector3(z=0.2)))
    assert publisher.published[-1].twist.linear.x == 0.4
    assert publisher.published[-1].twist.angular.z == 0.2

    module._on_stop(2)
    assert module._stopped is False
    assert publisher.published[-1].twist.linear.x == 0.0
    assert publisher.published[-1].twist.angular.z == 0.0

    module._on_cmd_vel(Twist(linear=Vector3(x=0.7), angular=Vector3(z=0.3)))
    assert publisher.published[-1].twist.linear.x == 0.7
    assert publisher.published[-1].twist.angular.z == 0.3

    module.stop()
