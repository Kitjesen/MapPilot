from __future__ import annotations

import builtins
import threading
import time
from types import SimpleNamespace

import pytest

from gateway.services import teleop
from gateway.services.command_boundary import CommandBoundaryError
from runtime.msgs.geometry import Twist, Vector3
from runtime.msgs.nav import OperatorMotionAction, OperatorMotionReceipt


def _receipt(
    action: OperatorMotionAction,
    source_id: str,
    source_epoch: int,
    sequence: int,
    request_id: str | None,
    *,
    accepted: bool = True,
    final_output_sequence: int = 0,
    reason: str = "accepted",
) -> OperatorMotionReceipt:
    return OperatorMotionReceipt(
        accepted=accepted,
        action=int(action),
        request_id=str(request_id or f"{source_id}:{action.name.lower()}:{sequence}"),
        source_id=source_id,
        source_epoch=source_epoch,
        source_sequence=sequence,
        accepted_sequence=sequence if accepted else 0,
        final_output_sequence=final_output_sequence if accepted else 0,
        endpoint_timestamp_s=time.time() or 1.0,
        reason=reason,
    )


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

    def claim(self, source_id, source_epoch, sequence, *, lease_ttl_ms, request_id=None) -> OperatorMotionReceipt:
        return _receipt(OperatorMotionAction.CLAIM, source_id, source_epoch, sequence, request_id)

    def sample(
        self,
        source_id,
        source_epoch,
        sequence,
        vx,
        vy,
        wz,
        *,
        deadman=True,
        freshness_budget_ms=350,
        request_id=None,
    ) -> bool:
        return self.send_teleop(vx, vy, wz, request_id=request_id)

    def hold(
        self, source_id, source_epoch, sequence, *, reason="operator_hold", request_id=None
    ) -> OperatorMotionReceipt:
        self.send_teleop(0.0, 0.0, 0.0, request_id=request_id)
        return _receipt(
            OperatorMotionAction.HOLD,
            source_id,
            source_epoch,
            sequence,
            request_id,
            final_output_sequence=sequence,
            reason=reason,
        )

    def release(
        self,
        source_id,
        source_epoch,
        sequence,
        *,
        reason="operator_release",
        request_id=None,
    ) -> OperatorMotionReceipt:
        return _receipt(
            OperatorMotionAction.RELEASE,
            source_id,
            source_epoch,
            sequence,
            request_id,
            final_output_sequence=sequence,
            reason=reason,
        )


class _OperatorClient:
    def __init__(self) -> None:
        self.commands = []

    def claim(self, source_id, source_epoch, sequence, *, lease_ttl_ms, request_id=None) -> OperatorMotionReceipt:
        self.commands.append(("claim", source_id, source_epoch, sequence, lease_ttl_ms, request_id))
        return _receipt(OperatorMotionAction.CLAIM, source_id, source_epoch, sequence, request_id)

    def sample(
        self,
        source_id,
        source_epoch,
        sequence,
        vx,
        vy,
        wz,
        *,
        deadman=True,
        freshness_budget_ms=350,
        request_id=None,
    ) -> bool:
        self.commands.append(
            (
                "sample",
                source_id,
                source_epoch,
                sequence,
                vx,
                vy,
                wz,
                deadman,
                freshness_budget_ms,
                request_id,
            )
        )
        return True

    def hold(
        self, source_id, source_epoch, sequence, *, reason="operator_hold", request_id=None
    ) -> OperatorMotionReceipt:
        self.commands.append(("hold", source_id, source_epoch, sequence, reason, request_id))
        return _receipt(
            OperatorMotionAction.HOLD,
            source_id,
            source_epoch,
            sequence,
            request_id,
            final_output_sequence=sequence,
            reason=reason,
        )

    def release(
        self,
        source_id,
        source_epoch,
        sequence,
        *,
        reason="operator_release",
        request_id=None,
    ) -> OperatorMotionReceipt:
        self.commands.append(("release", source_id, source_epoch, sequence, reason, request_id))
        return _receipt(
            OperatorMotionAction.RELEASE,
            source_id,
            source_epoch,
            sequence,
            request_id,
            final_output_sequence=sequence,
            reason=reason,
        )


class _IncompleteOperatorClient:
    def claim(self, source_id, source_epoch, sequence, *, lease_ttl_ms, request_id=None) -> OperatorMotionReceipt:
        return _receipt(OperatorMotionAction.CLAIM, source_id, source_epoch, sequence, request_id)

    def sample(
        self,
        source_id,
        source_epoch,
        sequence,
        vx,
        vy,
        wz,
        *,
        deadman=True,
        freshness_budget_ms=350,
        request_id=None,
    ) -> bool:
        return True

    def hold(self, source_id, source_epoch, sequence, *, reason="operator_hold", request_id=None) -> bool:
        return True


class _RejectingClaimOperatorClient(_OperatorClient):
    def claim(self, source_id, source_epoch, sequence, *, lease_ttl_ms, request_id=None) -> OperatorMotionReceipt:
        super().claim(
            source_id,
            source_epoch,
            sequence,
            lease_ttl_ms=lease_ttl_ms,
            request_id=request_id,
        )
        return _receipt(
            OperatorMotionAction.CLAIM,
            source_id,
            source_epoch,
            sequence,
            request_id,
            accepted=False,
            reason="claim_rejected",
        )


class _MissingClaimOperatorClient:
    def sample(self, *_args, **_kwargs) -> bool:
        return True

    def hold(self, *_args, **_kwargs) -> bool:
        return True

    def release(self, *_args, **_kwargs) -> bool:
        return True


class _BlockingClient(_Client):
    def __init__(self) -> None:
        super().__init__()
        self.entered = threading.Event()
        self.unblock_write = threading.Event()

    def send_teleop(self, vx, vy, wz, *, request_id=None) -> bool:
        self.commands.append((vx, vy, wz, request_id))
        self.entered.set()
        self.unblock_write.wait(timeout=2.0)
        return True


class _FailingClient(_Client):
    def send_teleop(self, vx, vy, wz, *, request_id=None) -> bool:
        raise CommandBoundaryError("endpoint rejected teleop")


class _FailOnceClient(_Client):
    def __init__(self) -> None:
        super().__init__()
        self.attempts = 0

    def send_teleop(self, vx, vy, wz, *, request_id=None) -> bool:
        self.attempts += 1
        self.commands.append((vx, vy, wz, request_id))
        if self.attempts == 1:
            raise CommandBoundaryError("first sample rejected")
        return True


class _ConfigurableAckOperatorClient(_OperatorClient):
    def __init__(self, ack) -> None:
        super().__init__()
        self.ack = ack

    def claim(self, *args, **kwargs):
        super().claim(*args, **kwargs)
        return self.ack

    def sample(self, *args, **kwargs):
        super().sample(*args, **kwargs)
        return self.ack

    def hold(self, *args, **kwargs):
        super().hold(*args, **kwargs)
        return self.ack

    def release(self, *args, **kwargs):
        super().release(*args, **kwargs)
        return self.ack


def _wait_until(predicate, timeout: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return bool(predicate())


def test_teleop_uses_typed_operator_motion_capability():
    client = _OperatorClient()
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

    try:
        assert teleop.claim(
            gateway,
            source_id="rest:operator-a",
            source_epoch=42,
            sequence=1,
            lease_ttl_ms=1000,
            request_id="claim-1",
        )
        delivered = teleop.publish_remote_velocity_request(
            gateway,
            command,
            publish_local_compat=False,
            source_id="rest:operator-a",
            source_epoch=42,
            sequence=2,
        )

        assert delivered is True
        assert _wait_until(lambda: len(client.commands) == 2)
    finally:
        gateway._teleop_native_publisher.close()

    assert client.commands == [
        ("claim", "rest:operator-a", 42, 1, 1000, "claim-1"),
        ("sample", "rest:operator-a", 42, 2, 0.2, -0.1, 0.4, True, 350, None),
    ]
    assert gateway.cmd_vel.messages == []


def test_field_remote_velocity_rejects_stateless_unclaimed_source():
    client = _OperatorClient()
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
    command = Twist(linear=Vector3(x=0.2), angular=Vector3(z=0.1))

    try:
        with pytest.raises(CommandBoundaryError, match="explicit claimed source session"):
            teleop.publish_remote_velocity_request(gateway, command)
    finally:
        gateway._teleop_native_publisher.close()

    assert client.commands == []
    assert gateway.cmd_vel.messages == []


def test_native_teleop_publisher_sequences_claim_sample_hold_and_release():
    client = _OperatorClient()
    publisher = teleop.LatestNativeTeleopPublisher(client)
    try:
        publisher.claim(
            source_id="ws:operator-a",
            source_epoch=42,
            sequence=1,
            lease_ttl_ms=1000,
            request_id="claim-1",
        )
        assert publisher.submit(
            0.2,
            -0.1,
            0.5,
            source_id="ws:operator-a",
            source_epoch=42,
            sequence=2,
            request_id="sample-1",
        )
        assert _wait_until(lambda: len(client.commands) == 2)
        publisher.quiesce_and_send_zero(
            source_id="ws:operator-a",
            source_epoch=42,
            sequence=3,
            reason="manual_hold",
            request_id="hold-1",
        )
        publisher.release_source(
            source_id="ws:operator-a",
            source_epoch=42,
            sequence=4,
            reason="disconnect",
            request_id="release-1",
        )
    finally:
        publisher.close()

    assert client.commands == [
        ("claim", "ws:operator-a", 42, 1, 1000, "claim-1"),
        ("sample", "ws:operator-a", 42, 2, 0.2, -0.1, 0.5, True, 350, "sample-1"),
        ("hold", "ws:operator-a", 42, 3, "manual_hold", "hold-1"),
        ("release", "ws:operator-a", 42, 4, "disconnect", "release-1"),
    ]


def test_field_binding_rejects_incomplete_operator_motion_interface(monkeypatch):
    monkeypatch.delenv("LINGTU_NAV_CLIENT_LIB", raising=False)
    gateway = SimpleNamespace(cmd_vel=_Port())
    teleop.init_teleop_state(
        gateway,
        max_speed=0.5,
        max_yaw=1.0,
        release_timeout=0.5,
        bridge_addr_raw="",
        dds_enabled=True,
    )

    teleop.bind_navigation_commands(gateway, _IncompleteOperatorClient())

    assert gateway._teleop_native_publisher is None


def test_field_binding_uses_only_assembled_commands_capability(monkeypatch):
    commands = _OperatorClient()
    gateway = SimpleNamespace(cmd_vel=_Port())
    teleop.init_teleop_state(
        gateway,
        max_speed=0.5,
        max_yaw=1.0,
        release_timeout=0.5,
        bridge_addr_raw="",
        dds_enabled=True,
    )
    real_import = builtins.__import__
    nav_imports = []

    def reject_nav_import(name, *args, **kwargs):
        if name == "nav" or name.startswith("nav."):
            nav_imports.append(name)
            raise AssertionError(f"Gateway bypassed nav.commands via {name}")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", reject_nav_import)

    teleop.bind_navigation_commands(gateway, commands)
    try:
        assert gateway._teleop_native_publisher is not None
        assert gateway._teleop_native_publisher._client is commands
        assert nav_imports == []
    finally:
        gateway._teleop_native_publisher.close()


def test_operator_motion_release_source_fails_closed_when_release_is_missing():
    publisher = teleop.LatestNativeTeleopPublisher(_IncompleteOperatorClient())
    try:
        with pytest.raises(CommandBoundaryError, match="release capability"):
            publisher.release_source(
                source_id="ws:operator-a",
                source_epoch=42,
                sequence=1,
                request_id="disconnect-1",
            )
    finally:
        publisher.close()


def test_operator_motion_claim_fails_closed_when_claim_is_missing():
    publisher = teleop.LatestNativeTeleopPublisher(_MissingClaimOperatorClient())
    try:
        with pytest.raises(CommandBoundaryError, match="claim capability"):
            publisher.claim(
                source_id="ws:operator-a",
                source_epoch=42,
                sequence=1,
                lease_ttl_ms=1000,
                request_id="claim-1",
            )
    finally:
        publisher.close()


def test_operator_motion_claim_fails_closed_on_explicit_rejection():
    client = _RejectingClaimOperatorClient()
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
    try:
        receipt = teleop.claim(
            gateway,
            source_id="ws:operator-a",
            source_epoch=42,
            sequence=1,
            lease_ttl_ms=1000,
            request_id="claim-rejected",
        )
        assert isinstance(receipt, OperatorMotionReceipt)
        assert receipt.source_accepted is False
        assert receipt.reason == "claim_rejected"
    finally:
        gateway._teleop_native_publisher.close()

    assert client.commands == [("claim", "ws:operator-a", 42, 1, 1000, "claim-rejected")]


@pytest.mark.parametrize("ack", [None, "true", {"accepted": True}, 1])
@pytest.mark.parametrize("action", ["claim", "hold", "release"])
def test_typed_operator_motion_control_rejects_non_literal_true_ack(ack, action):
    publisher = teleop.LatestNativeTeleopPublisher(_ConfigurableAckOperatorClient(ack))
    try:
        with pytest.raises(CommandBoundaryError, match="receipt"):
            if action == "claim":
                publisher.claim(
                    source_id="ws:operator-a",
                    source_epoch=42,
                    sequence=1,
                    lease_ttl_ms=1000,
                    request_id="claim-1",
                )
            elif action == "hold":
                publisher.quiesce_and_send_zero(
                    source_id="ws:operator-a",
                    source_epoch=42,
                    sequence=2,
                    request_id="hold-1",
                )
            else:
                publisher.release_source(
                    source_id="ws:operator-a",
                    source_epoch=42,
                    sequence=3,
                    request_id="release-1",
                )
    finally:
        publisher.close()


@pytest.mark.parametrize("ack", [None, "true", {"accepted": True}, 1])
def test_typed_operator_motion_sample_rejects_non_literal_true_ack(ack):
    publisher = teleop.LatestNativeTeleopPublisher(_ConfigurableAckOperatorClient(ack))
    try:
        assert publisher.submit(
            0.2,
            0.0,
            0.1,
            source_id="ws:operator-a",
            source_epoch=42,
            sequence=2,
            request_id="sample-1",
        )
        assert _wait_until(lambda: publisher.last_error is not None and "acknowledgement" in publisher.last_error)
    finally:
        publisher.close()


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
        )
        is True
    )
    assert (
        teleop.resolve_native_command_boundary(
            command_output_mode="local_driver",
        )
        is False
    )


def test_native_command_policy_rejects_unknown_mode():
    with pytest.raises(ValueError, match="unsupported command_output_mode"):
        teleop.resolve_native_command_boundary(
            command_output_mode="hybrid",
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
        client.unblock_write.set()

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
        client.unblock_write.set()
        release_thread.join(timeout=2.0)

        assert not release_thread.is_alive()
        assert _wait_until(lambda: len(client.commands) == 2)
        assert [command[0] for command in client.commands] == [0.3, 0.0]
    finally:
        client.unblock_write.set()
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


def test_async_sample_failure_is_pushed_as_gateway_event():
    events = []
    gateway = SimpleNamespace(cmd_vel=_Port(), push_event=events.append)
    teleop.init_teleop_state(
        gateway,
        max_speed=1.0,
        max_yaw=1.0,
        release_timeout=0.5,
        bridge_addr_raw="",
        dds_enabled=True,
    )
    teleop.bind_navigation_commands(gateway, _FailingClient())
    command = Twist(linear=Vector3(x=0.2), angular=Vector3(z=0.1))

    try:
        assert teleop.publish_remote_velocity_request(
            gateway,
            command,
            source_id="ws:operator-a",
            source_epoch=42,
            sequence=7,
            request_id="sample-fail-1",
        )
        assert _wait_until(lambda: len(events) == 1)
    finally:
        teleop.shutdown_teleop(gateway)

    assert events == [
        {
            "type": "operator_motion_sample_failed",
            "data": {
                "source_id": "ws:operator-a",
                "source_epoch": 42,
                "source_sequence": 7,
                "request_id": "sample-fail-1",
                "error": "endpoint rejected teleop",
                "stage": "dds_submission_failed",
                "final_cmd_vel_confirmed": False,
                "motor_confirmed": False,
            },
        }
    ]


def test_failure_callback_exception_does_not_kill_publisher_worker():
    client = _FailOnceClient()
    callback_called = threading.Event()

    def raising_callback(_failure):
        callback_called.set()
        raise RuntimeError("observer unavailable")

    publisher = teleop.LatestNativeTeleopPublisher(
        client,
        failure_callback=raising_callback,
    )
    try:
        assert publisher.submit(
            0.1,
            0.0,
            0.0,
            source_id="ws:operator-a",
            source_epoch=42,
            sequence=1,
            request_id="first",
        )
        assert callback_called.wait(timeout=1.0)
        assert _wait_until(lambda: publisher.last_error == "first sample rejected")

        publisher.resume()
        assert publisher.submit(
            0.2,
            0.0,
            0.0,
            source_id="ws:operator-a",
            source_epoch=42,
            sequence=2,
            request_id="second",
        )
        assert _wait_until(lambda: len(client.commands) == 2)
        assert _wait_until(lambda: publisher.last_error is None)
    finally:
        publisher.close()


def test_operator_motion_hold_without_final_output_keeps_zero_barrier():
    class HoldWithoutFinalClient(_OperatorClient):
        def hold(self, source_id, source_epoch, sequence, *, reason="operator_hold", request_id=None):
            self.commands.append(("hold", source_id, source_epoch, sequence, reason, request_id))
            return _receipt(
                OperatorMotionAction.HOLD,
                source_id,
                source_epoch,
                sequence,
                request_id,
                final_output_sequence=0,
                reason="zero_not_published",
            )

    publisher = teleop.LatestNativeTeleopPublisher(HoldWithoutFinalClient())
    try:
        receipt = publisher.quiesce_and_send_zero(
            source_id="ws:operator-a",
            source_epoch=42,
            sequence=3,
            request_id="hold-no-final",
        )
        assert isinstance(receipt, OperatorMotionReceipt)
        assert receipt.source_accepted is True
        assert receipt.final_output_published is False
        assert publisher.last_error == "zero_not_published"
        assert (
            publisher.submit(
                0.1,
                0.0,
                0.0,
                source_id="ws:operator-a",
                source_epoch=42,
                sequence=4,
            )
            is False
        )
    finally:
        publisher.close()


@pytest.mark.parametrize(
    ("field", "value", "match"),
    [
        ("request_id", "other-request", "request_id"),
        ("source_id", "other-source", "source_id"),
        ("source_epoch", 43, "source_epoch"),
        ("source_sequence", 4, "source_sequence"),
        ("action", OperatorMotionAction.RELEASE, "action"),
    ],
)
def test_operator_motion_receipt_identity_mismatch_is_rejected(field, value, match):
    class MismatchedReceiptClient(_OperatorClient):
        def claim(self, source_id, source_epoch, sequence, *, lease_ttl_ms, request_id=None):
            values = {
                "accepted": True,
                "action": int(OperatorMotionAction.CLAIM),
                "request_id": request_id,
                "source_id": source_id,
                "source_epoch": source_epoch,
                "source_sequence": sequence,
                "accepted_sequence": sequence,
                "final_output_sequence": 0,
                "endpoint_timestamp_s": time.time() or 1.0,
                "reason": "accepted",
            }
            values[field] = int(value) if isinstance(value, OperatorMotionAction) else value
            if field == "source_sequence":
                values["accepted_sequence"] = value
            return OperatorMotionReceipt(**values)

    publisher = teleop.LatestNativeTeleopPublisher(MismatchedReceiptClient())
    try:
        with pytest.raises(CommandBoundaryError, match=match):
            publisher.claim(
                source_id="ws:operator-a",
                source_epoch=42,
                sequence=1,
                lease_ttl_ms=1000,
                request_id="claim-1",
            )
    finally:
        publisher.close()
