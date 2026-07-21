from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import pytest

from gateway.services import teleop
from gateway.services.command_boundary import CommandBoundaryError
from runtime.msgs.geometry import Twist, Vector3


class _Port:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class _Client:
    def __init__(self) -> None:
        self.commands = []

    def send_teleop(self, vx, vy, wz, *, request_id=None) -> bool:
        self.commands.append((vx, vy, wz, request_id))
        return True


class _BlockingClient(_Client):
    def __init__(self) -> None:
        super().__init__()
        self.entered = threading.Event()
        self.release = threading.Event()

    def send_teleop(self, vx, vy, wz, *, request_id=None) -> bool:
        self.commands.append((vx, vy, wz, request_id))
        self.entered.set()
        self.release.wait(timeout=2.0)
        return True


class _FailingClient(_Client):
    def send_teleop(self, vx, vy, wz, *, request_id=None) -> bool:
        raise CommandBoundaryError("endpoint rejected teleop")


def _wait_until(predicate, timeout: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return bool(predicate())


def test_teleop_uses_assembled_navigation_command_capability():
    client = _Client()
    gateway = SimpleNamespace(cmd_vel=_Port())
    teleop.init_teleop_state(
        gateway,
        max_speed=0.5,
        max_yaw=1.0,
        release_timeout=0.5,
        bridge_addr_raw="",
        dds_enabled=True,
    )
    teleop.bind_navigation_commands(gateway, client)
    command = Twist(
        linear=Vector3(x=0.2, y=-0.1),
        angular=Vector3(z=0.4),
    )

    delivered = teleop.publish_remote_velocity_request(
        gateway,
        command,
        publish_local_compat=False,
    )

    assert delivered is True
    assert client.commands == [(0.2, -0.1, 0.4, None)]
    assert gateway.cmd_vel.messages == []


def test_field_teleop_fails_closed_when_native_capability_is_unavailable():
    gateway = SimpleNamespace(cmd_vel=_Port())
    teleop.init_teleop_state(
        gateway,
        max_speed=0.5,
        max_yaw=1.0,
        release_timeout=0.5,
        bridge_addr_raw="",
        dds_enabled=True,
    )
    command = Twist(linear=Vector3(x=0.2), angular=Vector3(z=0.1))

    with pytest.raises(CommandBoundaryError, match="capability is unavailable"):
        teleop.publish_remote_velocity_request(gateway, command)

    teleop.on_joy(gateway, 0.4, 0.0, 0.1)
    assert gateway.cmd_vel.messages == []


def test_endpoint_only_is_the_primary_native_command_policy():
    assert (
        teleop.resolve_native_command_boundary(
            command_output_mode="endpoint_only",
            legacy_dds_env=None,
        )
        is True
    )
    assert (
        teleop.resolve_native_command_boundary(
            command_output_mode="local_driver",
            legacy_dds_env=None,
        )
        is False
    )


def test_native_command_policy_rejects_conflicting_legacy_override():
    with pytest.raises(ValueError, match="conflicts"):
        teleop.resolve_native_command_boundary(
            command_output_mode="endpoint_only",
            legacy_dds_env="0",
        )
    with pytest.raises(ValueError, match="conflicts"):
        teleop.resolve_native_command_boundary(
            command_output_mode="local_driver",
            legacy_dds_env="1",
        )


def test_native_joystick_is_nonblocking_and_keeps_only_latest_pending():
    client = _BlockingClient()
    gateway = SimpleNamespace(cmd_vel=_Port())
    teleop.init_teleop_state(
        gateway,
        max_speed=1.0,
        max_yaw=1.0,
        release_timeout=0.5,
        bridge_addr_raw="",
        dds_enabled=True,
    )
    teleop.bind_navigation_commands(gateway, client)
    try:
        started = time.monotonic()
        teleop.on_joy(gateway, 0.1, 0.0, 0.0)
        assert time.monotonic() - started < 0.1
        assert client.entered.wait(timeout=1.0)

        teleop.on_joy(gateway, 0.2, 0.0, 0.0)
        teleop.on_joy(gateway, 0.7, 0.0, 0.0)
        client.release.set()

        assert _wait_until(lambda: len(client.commands) == 2)
        assert [command[0] for command in client.commands] == [0.1, 0.7]
    finally:
        teleop.shutdown_teleop(gateway)


def test_native_release_orders_zero_after_inflight_and_drops_pending():
    client = _BlockingClient()
    gateway = SimpleNamespace(cmd_vel=_Port())
    teleop.init_teleop_state(
        gateway,
        max_speed=1.0,
        max_yaw=1.0,
        release_timeout=0.5,
        bridge_addr_raw="",
        dds_enabled=True,
    )
    teleop.bind_navigation_commands(gateway, client)
    release_thread = None
    try:
        teleop.on_joy(gateway, 0.3, 0.0, 0.0)
        assert client.entered.wait(timeout=1.0)
        teleop.on_joy(gateway, 0.8, 0.0, 0.0)

        release_thread = threading.Thread(target=teleop.release, args=(gateway,))
        release_thread.start()
        time.sleep(0.02)
        client.release.set()
        release_thread.join(timeout=2.0)

        assert not release_thread.is_alive()
        assert _wait_until(lambda: len(client.commands) == 2)
        assert [command[0] for command in client.commands] == [0.3, 0.0]
    finally:
        client.release.set()
        if release_thread is not None:
            release_thread.join(timeout=1.0)
        teleop.shutdown_teleop(gateway)


def test_native_publish_failure_disables_followup_joystick_commands():
    client = _FailingClient()
    gateway = SimpleNamespace(cmd_vel=_Port())
    teleop.init_teleop_state(
        gateway,
        max_speed=1.0,
        max_yaw=1.0,
        release_timeout=0.5,
        bridge_addr_raw="",
        dds_enabled=True,
    )
    teleop.bind_navigation_commands(gateway, client)
    try:
        assert teleop.on_joy(gateway, 0.2, 0.0, 0.0) is True
        assert _wait_until(lambda: gateway._teleop_native_publisher.last_error is not None)
        assert teleop.on_joy(gateway, 0.3, 0.0, 0.0) is False
    finally:
        teleop.shutdown_teleop(gateway)
