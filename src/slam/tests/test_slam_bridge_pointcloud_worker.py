from __future__ import annotations

import sys
import threading
import time
import types
from types import SimpleNamespace

import numpy as np
import pytest

pytestmark = [pytest.mark.ros2]

from core.runtime_interface import TOPICS, topic_default_frame_id


def _pointcloud_msg():
    points = np.array([[1.0, 2.0, 3.0, 0.0]], dtype=np.float32)
    return SimpleNamespace(
        width=1,
        height=1,
        point_step=16,
        data=points.tobytes(),
    )


def _dds_pointcloud_msg():
    points = np.array([[1.0, 2.0, 3.0, 0.0]], dtype=np.float32)
    return SimpleNamespace(
        width=1,
        height=1,
        point_step=16,
        data=points.tobytes(),
    )


def _odom_msg():
    def vec(x=0.0, y=0.0, z=0.0): return SimpleNamespace(x=x, y=y, z=z)
    quat = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    pose = SimpleNamespace(position=vec(1.0, 2.0, 0.0), orientation=quat)
    twist = SimpleNamespace(linear=vec(), angular=vec())
    pose_with_cov = SimpleNamespace(pose=pose, covariance=[0.0] * 36)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=123, nanosec=0)),
        pose=pose_with_cov,
        twist=SimpleNamespace(twist=twist),
    )


def _module(name: str, **attrs):
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    return module


@pytest.mark.ros2
def test_rclpy_pointcloud_callback_does_not_block_odom_executor():
    from slam.slam_bridge_module import SlamBridgeModule

    bridge = SlamBridgeModule()
    callback_entered = threading.Event()
    release_callback = threading.Event()
    callback_done = threading.Event()

    def slow_consumer(_cloud):
        callback_entered.set()
        assert release_callback.wait(timeout=1.0)
        callback_done.set()

    bridge.map_cloud._add_callback(slow_consumer)

    t0 = time.perf_counter()
    bridge._on_rclpy_cloud(_pointcloud_msg())
    elapsed_ms = (time.perf_counter() - t0) * 1000.0

    assert elapsed_ms < 50.0
    assert callback_entered.wait(timeout=1.0)

    bridge._on_rclpy_cloud(_pointcloud_msg())
    assert bridge._pointcloud_worker_drops == 1

    release_callback.set()
    assert callback_done.wait(timeout=1.0)
    if bridge._pointcloud_worker_thread is not None:
        bridge._pointcloud_worker_thread.join(timeout=1.0)
    assert bridge._last_cloud_time > 0.0


@pytest.mark.ros2
def test_rclpy_saved_map_has_independent_worker_from_live_cloud():
    from slam.slam_bridge_module import SlamBridgeModule

    bridge = SlamBridgeModule()
    cloud_callback_entered = threading.Event()
    release_cloud_callback = threading.Event()
    saved_map_received = threading.Event()

    def slow_cloud_consumer(_cloud):
        cloud_callback_entered.set()
        assert release_cloud_callback.wait(timeout=1.0)

    bridge.map_cloud._add_callback(slow_cloud_consumer)
    bridge.saved_map._add_callback(lambda _cloud: saved_map_received.set())

    bridge._on_rclpy_cloud(_pointcloud_msg())
    assert cloud_callback_entered.wait(timeout=1.0)

    bridge._on_rclpy_saved_map(_pointcloud_msg())
    assert saved_map_received.wait(timeout=1.0)
    assert bridge._saved_map_worker_drops == 0

    release_cloud_callback.set()
    if bridge._pointcloud_worker_thread is not None:
        bridge._pointcloud_worker_thread.join(timeout=1.0)
    if bridge._saved_map_worker_thread is not None:
        bridge._saved_map_worker_thread.join(timeout=1.0)


def test_dds_pointcloud_callback_does_not_block_reader_loop():
    from slam.slam_bridge_module import SlamBridgeModule

    bridge = SlamBridgeModule()
    bridge.setup()
    callback_entered = threading.Event()
    release_callback = threading.Event()
    callback_done = threading.Event()

    def slow_consumer(_cloud):
        callback_entered.set()
        assert release_callback.wait(timeout=1.0)
        callback_done.set()

    bridge.map_cloud._add_callback(slow_consumer)

    t0 = time.perf_counter()
    bridge._on_dds_cloud(_dds_pointcloud_msg())
    elapsed_ms = (time.perf_counter() - t0) * 1000.0

    assert elapsed_ms < 50.0
    assert callback_entered.wait(timeout=1.0)

    bridge._on_dds_cloud(_dds_pointcloud_msg())
    assert bridge._pointcloud_worker_drops == 1

    release_callback.set()
    assert callback_done.wait(timeout=1.0)
    if bridge._pointcloud_worker_thread is not None:
        bridge._pointcloud_worker_thread.join(timeout=1.0)
    assert bridge._last_cloud_time > 0.0


def _assert_odom_callback_does_not_block_receiver(callback_name: str):
    from slam.slam_bridge_module import SlamBridgeModule

    bridge = SlamBridgeModule()
    callback = getattr(bridge, callback_name)
    callback_entered = threading.Event()
    release_callback = threading.Event()
    callback_done = threading.Event()

    def slow_consumer(_odom):
        callback_entered.set()
        assert release_callback.wait(timeout=1.0)
        callback_done.set()

    bridge.odometry._add_callback(slow_consumer)

    t0 = time.perf_counter()
    callback(_odom_msg())
    elapsed_ms = (time.perf_counter() - t0) * 1000.0

    assert elapsed_ms < 50.0
    assert callback_entered.wait(timeout=1.0)

    assert bridge._last_odom_time > 0.0
    assert bridge._last_odom_mono > 0.0
    assert len(bridge._odom_recv_ts) == 1
    assert bridge._odom_worker_thread is not None
    assert bridge._odom_worker_thread.is_alive()

    callback(_odom_msg())
    assert bridge._odom_worker_drops == 1
    assert len(bridge._odom_recv_ts) == 2

    release_callback.set()
    assert callback_done.wait(timeout=1.0)
    if bridge._odom_worker_thread is not None:
        bridge._odom_worker_thread.join(timeout=1.0)


def _assert_odom_callback_normalizes_frame_ids(callback_name: str):
    from slam.slam_bridge_module import SlamBridgeModule

    bridge = SlamBridgeModule()
    callback = getattr(bridge, callback_name)
    received = []
    received_event = threading.Event()
    msg = _odom_msg()
    msg.header.frame_id = "/map"
    msg.child_frame_id = "/body"

    def capture(odom):
        received.append(odom)
        received_event.set()

    bridge.odometry._add_callback(capture)
    callback(msg)

    assert received_event.wait(timeout=1.0)
    if bridge._odom_worker_thread is not None:
        bridge._odom_worker_thread.join(timeout=1.0)
    assert received[0].frame_id == topic_default_frame_id(TOPICS.map_cloud)
    assert received[0].child_frame_id == topic_default_frame_id(TOPICS.cmd_vel)


def test_dds_odom_callback_does_not_block_reader_loop():
    _assert_odom_callback_does_not_block_receiver("_on_dds_odom")


@pytest.mark.ros2
def test_rclpy_odom_callback_does_not_block_executor():
    _assert_odom_callback_does_not_block_receiver("_on_rclpy_odom")


def test_dds_odom_normalizes_header_frame_ids():
    _assert_odom_callback_normalizes_frame_ids("_on_dds_odom")


@pytest.mark.ros2
def test_rclpy_odom_normalizes_header_frame_ids():
    _assert_odom_callback_normalizes_frame_ids("_on_rclpy_odom")


def test_slam_bridge_freshness_age_uses_monotonic_clock():
    from slam.slam_bridge_module import SlamBridgeModule

    bridge = SlamBridgeModule()
    bridge._mark_odom_received(wall_now=1000.0, mono_now=50.0)
    bridge._mark_cloud_received(wall_now=1000.0, mono_now=50.0)

    assert abs(bridge._odom_age_s(mono_now=50.4) - 0.4) < 1e-9
    assert abs(bridge._cloud_age_s(mono_now=50.9) - 0.9) < 1e-9


@pytest.mark.ros2
def test_rclpy_subscriptions_use_separate_callback_groups(monkeypatch):
    from slam.slam_bridge_module import SlamBridgeModule

    class FakeExecutor:
        def __init__(self) -> None:
            self.nodes = []

        def add_node(self, node) -> None:
            self.nodes.append(node)

    class FakeGroup:
        pass

    class FakeNode:
        def __init__(self, name: str) -> None:
            self.name = name
            self.subscriptions = []

        def create_subscription(self, msg_type, topic, callback, qos, **kwargs):
            self.subscriptions.append((topic, callback, qos, kwargs.get("callback_group")))
            return object()

    class QoSProfile:
        def __init__(self, **kwargs) -> None:
            self.kwargs = kwargs

    class ReliabilityPolicy:
        RELIABLE = object()
        BEST_EFFORT = object()

    nav_msgs = _module("nav_msgs.msg", Odometry=type("Odometry", (), {}))
    sensor_msgs = _module("sensor_msgs.msg", PointCloud2=type("PointCloud2", (), {}))
    std_msgs = _module(
        "std_msgs.msg",
        Float32=type("Float32", (), {}),
        Float32MultiArray=type("Float32MultiArray", (), {}),
        String=type("String", (), {}),
    )
    tf2_msgs = _module("tf2_msgs.msg", TFMessage=type("TFMessage", (), {}))
    monkeypatch.setitem(sys.modules, "nav_msgs", _module("nav_msgs", msg=nav_msgs))
    monkeypatch.setitem(sys.modules, "nav_msgs.msg", nav_msgs)
    monkeypatch.setitem(sys.modules, "sensor_msgs", _module("sensor_msgs", msg=sensor_msgs))
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs", _module("std_msgs", msg=std_msgs))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs)
    monkeypatch.setitem(sys.modules, "tf2_msgs", _module("tf2_msgs", msg=tf2_msgs))
    monkeypatch.setitem(sys.modules, "tf2_msgs.msg", tf2_msgs)
    monkeypatch.setitem(sys.modules, "rclpy.node", _module("rclpy.node", Node=FakeNode))
    monkeypatch.setitem(
        sys.modules,
        "rclpy.qos",
        _module("rclpy.qos", QoSProfile=QoSProfile, ReliabilityPolicy=ReliabilityPolicy),
    )
    monkeypatch.setitem(
        sys.modules,
        "rclpy.callback_groups",
        _module("rclpy.callback_groups", MutuallyExclusiveCallbackGroup=FakeGroup),
    )

    import core.ros2_context as ros2_context

    executor = FakeExecutor()
    monkeypatch.setattr(ros2_context, "ensure_rclpy", lambda: None)
    monkeypatch.setattr(ros2_context, "get_shared_executor", lambda: executor)

    bridge = SlamBridgeModule()

    assert bridge._try_rclpy() is True
    node = bridge._rclpy_node
    groups = {topic: group for topic, _callback, _qos, group in node.subscriptions}
    qos_by_topic = {topic: qos for topic, _callback, qos, _group in node.subscriptions}

    assert groups[bridge._odom_topic] is not None
    assert groups[bridge._cloud_topic] is not None
    assert groups[bridge._odom_topic] is not groups[bridge._cloud_topic]
    assert groups["/tf"] is not groups[bridge._odom_topic]
    assert qos_by_topic[bridge._odom_topic].kwargs["reliability"] is ReliabilityPolicy.BEST_EFFORT
    assert qos_by_topic[bridge._odom_topic].kwargs["depth"] == 5
    assert qos_by_topic[bridge._cloud_topic].kwargs["reliability"] is ReliabilityPolicy.BEST_EFFORT
    assert qos_by_topic[bridge._cloud_topic].kwargs["depth"] == 1
    assert qos_by_topic[bridge._saved_map_topic].kwargs["reliability"] is ReliabilityPolicy.RELIABLE
