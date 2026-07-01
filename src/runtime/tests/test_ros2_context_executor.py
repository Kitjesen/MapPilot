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
    import runtime.adapters.ros2.context as ctx

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
    import runtime.adapters.ros2.context as ctx

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

    import runtime.adapters.ros2.context as ctx

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

