from __future__ import annotations

import sys
import time
import types


class FakeMultiThreadedExecutor:
    instances = []

    def __init__(self, num_threads: int) -> None:
        self.num_threads = num_threads
        self.shutdown_called = False
        type(self).instances.append(self)

    def spin_once(self, timeout_sec: float) -> None:
        time.sleep(min(timeout_sec, 0.001))

    def shutdown(self) -> None:
        self.shutdown_called = True


class FakeSingleThreadedExecutor:
    instances = []

    def __init__(self) -> None:
        self.shutdown_called = False
        type(self).instances.append(self)

    def spin_once(self, timeout_sec: float) -> None:
        time.sleep(min(timeout_sec, 0.001))

    def shutdown(self) -> None:
        self.shutdown_called = True


class FakeExecutorWithNodes:
    def __init__(self) -> None:
        self.added = []
        self.removed = []

    def add_node(self, node) -> None:
        self.added.append(node)

    def remove_node(self, node) -> None:
        self.removed.append(node)


class FakeDestroyableExecutor:
    def __init__(self, ctx) -> None:
        self.ctx = ctx
        self.calls = 0

    def spin_once(self, timeout_sec: float) -> None:
        self.calls += 1
        self.ctx._running = False
        raise RuntimeError("cannot use Destroyable because destruction was requested")


class FakeNode:
    instances = []

    def __init__(self, name: str) -> None:
        self.name = name
        self.destroyed = False
        self.subscriptions = []
        type(self).instances.append(self)

    def create_subscription(self, msg_type, topic, callback, qos, **kwargs):
        self.subscriptions.append((msg_type, topic, callback, qos, kwargs))
        return object()

    def destroy_node(self) -> None:
        self.destroyed = True


class QoSProfile:
    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs


class ReliabilityPolicy:
    BEST_EFFORT = object()


def _module(name: str, **attrs):
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    return module


def _reset_ros2_context(ctx) -> None:
    ctx._running = False
    if ctx._spin_thread is not None:
        ctx._spin_thread.join(timeout=1.0)
    ctx._spin_thread = None
    if ctx._executor is not None:
        try:
            ctx._executor.shutdown()
        except Exception:
            pass
    ctx._executor = None


def _install_fake_rclpy(monkeypatch, executors_module) -> None:
    monkeypatch.setitem(
        sys.modules,
        "rclpy",
        _module("rclpy", ok=lambda: True, init=lambda: None),
    )
    monkeypatch.setitem(sys.modules, "rclpy.executors", executors_module)


def test_shared_executor_prefers_configured_multithreaded_executor(monkeypatch):
    import compat.ros2.context as ctx

    _reset_ros2_context(ctx)
    FakeMultiThreadedExecutor.instances = []
    _install_fake_rclpy(
        monkeypatch,
        _module("rclpy.executors", MultiThreadedExecutor=FakeMultiThreadedExecutor),
    )
    monkeypatch.setenv("LINGTU_ROS2_EXECUTOR_THREADS", "3")

    executor = ctx.get_shared_executor()

    assert isinstance(executor, FakeMultiThreadedExecutor)
    assert executor.num_threads == 3
    assert ctx._spin_thread is not None
    assert ctx._spin_thread.is_alive()

    ctx.shutdown_shared_executor()
    assert executor.shutdown_called is True
    assert ctx._executor is None


def test_shared_executor_falls_back_when_multithreaded_unavailable(monkeypatch):
    import compat.ros2.context as ctx

    _reset_ros2_context(ctx)
    FakeSingleThreadedExecutor.instances = []
    _install_fake_rclpy(
        monkeypatch,
        _module("rclpy.executors", SingleThreadedExecutor=FakeSingleThreadedExecutor),
    )

    executor = ctx.get_shared_executor()

    assert isinstance(executor, FakeSingleThreadedExecutor)

    ctx.shutdown_shared_executor()
    assert executor.shutdown_called is True


def test_shared_executor_ignores_expected_destroyed_entity_race(monkeypatch, caplog):
    import logging

    import compat.ros2.context as ctx

    _reset_ros2_context(ctx)
    _install_fake_rclpy(
        monkeypatch,
        _module("rclpy.executors", MultiThreadedExecutor=FakeMultiThreadedExecutor),
    )
    ctx._executor = FakeDestroyableExecutor(ctx)
    ctx._running = True

    with caplog.at_level(logging.WARNING):
        ctx._spin_loop()

    assert ctx._executor.calls == 1
    assert "ROS2 spin_once error" not in caplog.text
    _reset_ros2_context(ctx)


def test_camera_bridge_reconnect_removes_old_node_from_shared_executor(monkeypatch):
    executor = FakeExecutorWithNodes()
    FakeNode.instances = []
    sensor_msg = _module(
        "sensor_msgs.msg",
        CameraInfo=type("CameraInfo", (), {}),
        Image=type("Image", (), {}),
    )
    monkeypatch.setitem(sys.modules, "sensor_msgs", _module("sensor_msgs", msg=sensor_msg))
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", sensor_msg)
    monkeypatch.setitem(sys.modules, "rclpy.node", _module("rclpy.node", Node=FakeNode))
    monkeypatch.setitem(
        sys.modules,
        "rclpy.qos",
        _module("rclpy.qos", QoSProfile=QoSProfile, ReliabilityPolicy=ReliabilityPolicy),
    )

    import compat.ros2.context as ros2_context

    monkeypatch.setattr(ros2_context, "ensure_rclpy", lambda: None)
    monkeypatch.setattr(ros2_context, "get_shared_executor", lambda: executor)

    from compat.ros2.camera_bridge import CameraBridgeModule

    module = CameraBridgeModule()
    assert module._create_ros2_node() is True
    first = FakeNode.instances[0]

    assert module._create_ros2_node() is True
    second = FakeNode.instances[1]

    assert executor.added == [first, second]
    assert executor.removed == [first]
    assert first.destroyed is True

    module.stop()
    assert executor.removed == [first, second]
    assert second.destroyed is True
    assert module._node is None
    assert module._executor is None


def test_camera_bridge_service_recovery_is_opt_in(monkeypatch):
    from compat.ros2.camera_bridge import CameraBridgeModule

    monkeypatch.delenv("LINGTU_CAMERA_ALLOW_SERVICE_RECOVERY", raising=False)
    module = CameraBridgeModule(max_reconnects=3)

    assert module._allow_service_recovery is False
    assert module._suppress_service_recovery(level=2) is True
    assert module._service_recovery_suppressed is True
    assert module._reconnect_count == module._max_reconnects + 1

    enabled = CameraBridgeModule(max_reconnects=3, allow_service_recovery=True)
    assert enabled._allow_service_recovery is True
    assert enabled._suppress_service_recovery(level=2) is False


def test_camera_bridge_recovery_uses_robot_camera_service(monkeypatch):
    import subprocess

    import compat.ros2.camera_bridge as camera_bridge
    from compat.ros2.camera_bridge import CameraBridgeModule

    calls = []

    def fake_run(args, **kwargs):
        calls.append(tuple(args))

        class Result:
            returncode = 0
            stdout = ""
            stderr = ""

        return Result()

    monkeypatch.delenv("LINGTU_CAMERA_SERVICE", raising=False)
    monkeypatch.delenv("LINGTU_CAMERA_LEGACY_SERVICES", raising=False)
    monkeypatch.setattr(subprocess, "run", fake_run)
    monkeypatch.setattr(camera_bridge.time, "sleep", lambda _seconds: None)

    CameraBridgeModule._restart_camera_service()

    assert ("sudo", "systemctl", "stop", "camera.service") in calls
    assert ("sudo", "systemctl", "stop", "orbbec-camera.service") in calls
    assert ("sudo", "systemctl", "stop", "robot-camera.service") in calls
    assert ("sudo", "systemctl", "start", "robot-camera.service") in calls
    assert ("sudo", "systemctl", "start", "camera.service") not in calls
    assert any(call[:4] == ("sudo", "pkill", "-TERM", "-f") for call in calls)
