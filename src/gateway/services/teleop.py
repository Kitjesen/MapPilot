"""Teleop gateway helpers.

This module owns the gateway-side transport details for operator velocity
requests. GatewayModule keeps the public method names while the socket/DDS
mechanics live here.
"""

from __future__ import annotations

import logging
import socket
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from gateway.services.command_boundary import (
    CommandBoundaryError,
)
from runtime.msgs.geometry import Twist, Vector3
from runtime.msgs.nav import OperatorMotionAction, OperatorMotionReceipt
from runtime.runtime_interface import TOPICS

logger = logging.getLogger(__name__)


def _require_literal_true_ack(result: Any, *, operation: str) -> bool:
    if result is True:
        return True
    if result is False:
        raise CommandBoundaryError(f"native endpoint rejected {operation}")
    raise CommandBoundaryError(
        f"native endpoint returned an invalid acknowledgement for {operation}"
    )


def _require_operator_motion_receipt(
    result: Any,
    *,
    operation: str,
    action: OperatorMotionAction,
    request_id: str | None,
    source_id: str,
    source_epoch: int,
    sequence: int,
) -> OperatorMotionReceipt:
    if not isinstance(result, OperatorMotionReceipt):
        raise CommandBoundaryError(
            f"native endpoint returned an invalid receipt for {operation}"
        )
    expected_request_id = str(request_id or "")
    if not expected_request_id:
        raise CommandBoundaryError(
            f"gateway did not provide a request_id for {operation}"
        )
    if result.request_id != expected_request_id:
        raise CommandBoundaryError(
            f"native endpoint returned a mismatched request_id for {operation}"
        )
    if result.source_id != str(source_id):
        raise CommandBoundaryError(
            f"native endpoint returned a mismatched source_id for {operation}"
        )
    if int(result.source_epoch) != int(source_epoch):
        raise CommandBoundaryError(
            f"native endpoint returned a mismatched source_epoch for {operation}"
        )
    if int(result.source_sequence) != int(sequence):
        raise CommandBoundaryError(
            f"native endpoint returned a mismatched source_sequence for {operation}"
        )
    if int(result.action) != int(action):
        raise CommandBoundaryError(
            f"native endpoint returned a mismatched action for {operation}"
        )
    return result


@dataclass(frozen=True)
class _OperatorMotionSample:
    vx: float
    vy: float
    wz: float
    source_id: str
    source_epoch: int
    sequence: int
    request_id: str | None
    freshness_budget_ms: int


class LatestNativeTeleopPublisher:
    """Non-blocking latest-value publisher for high-rate joystick commands.

    Ordinary joystick samples are replaceable: while one typed DDS submission
    call is in flight, only the newest pending sample is retained. Samples do
    not carry an endpoint application ACK.
    Safety-critical release/stop paths use :meth:`quiesce_and_send_zero`, which
    prevents an older sample from being published after the zero command.
    """

    def __init__(
        self,
        client: Any,
        *,
        failure_callback: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self._client = client
        self._failure_callback = failure_callback
        self._condition = threading.Condition()
        self._pending: _OperatorMotionSample | None = None
        self._inflight = False
        self._accepting = True
        self._closed = False
        self._last_error: str | None = None
        self._thread = threading.Thread(
            target=self._run,
            name="lingtu-native-teleop",
            daemon=True,
        )
        self._thread.start()

    @property
    def last_error(self) -> str | None:
        with self._condition:
            return self._last_error

    def submit(
        self,
        vx: float,
        vy: float,
        wz: float,
        *,
        request_id: str | None = None,
        source_id: str = "gateway:teleop",
        source_epoch: int = 1,
        sequence: int = 1,
        freshness_budget_ms: int = 350,
    ) -> bool:
        with self._condition:
            if self._closed or not self._accepting:
                return False
            self._pending = _OperatorMotionSample(
                float(vx),
                float(vy),
                float(wz),
                str(source_id),
                int(source_epoch),
                int(sequence),
                request_id,
                int(freshness_budget_ms),
            )
            self._condition.notify()
            return True

    def claim(
        self,
        *,
        source_id: str,
        source_epoch: int,
        sequence: int,
        lease_ttl_ms: int,
        request_id: str | None = None,
    ) -> OperatorMotionReceipt | bool:
        request_id = str(request_id or f"{source_id}:claim:{int(sequence)}")
        client_claim = getattr(self._client, "claim", None)
        if client_claim is not None:
            receipt = client_claim(
                source_id,
                source_epoch,
                sequence,
                lease_ttl_ms=lease_ttl_ms,
                request_id=request_id,
            )
            return _require_operator_motion_receipt(
                receipt,
                operation="operator motion claim",
                action=OperatorMotionAction.CLAIM,
                request_id=request_id,
                source_id=source_id,
                source_epoch=source_epoch,
                sequence=sequence,
            )
        raise CommandBoundaryError("native operator motion claim capability is unavailable")

    def release_source(
        self,
        *,
        source_id: str,
        source_epoch: int,
        sequence: int,
        reason: str = "operator_release",
        request_id: str | None = None,
    ) -> OperatorMotionReceipt | bool:
        request_id = str(request_id or f"{source_id}:release:{int(sequence)}")
        client_release = getattr(self._client, "release", None)
        if client_release is not None:
            receipt = client_release(
                source_id,
                source_epoch,
                sequence,
                reason=reason,
                request_id=request_id,
            )
            return _require_operator_motion_receipt(
                receipt,
                operation="operator motion release",
                action=OperatorMotionAction.RELEASE,
                request_id=request_id,
                source_id=source_id,
                source_epoch=source_epoch,
                sequence=sequence,
            )
        raise CommandBoundaryError("native operator motion release capability is unavailable")

    def quiesce_and_send_zero(
        self,
        *,
        request_id: str | None = None,
        source_id: str = "gateway:teleop",
        source_epoch: int = 1,
        sequence: int = 1,
        reason: str = "operator_hold",
        timeout_s: float = 2.0,
    ) -> OperatorMotionReceipt | bool:
        """Drop pending samples, finish the in-flight write, then await the hold ACK."""

        request_id = str(request_id or f"{source_id}:hold:{int(sequence)}")
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        with self._condition:
            if self._closed:
                raise CommandBoundaryError("native teleop publisher is closed")
            self._accepting = False
            self._pending = None
            while self._inflight:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise CommandBoundaryError("native teleop publisher did not quiesce before release")
                self._condition.wait(remaining)
        try:
            client_hold = getattr(self._client, "hold", None)
            if client_hold is not None:
                receipt = client_hold(
                    source_id,
                    source_epoch,
                    sequence,
                    reason=reason,
                    request_id=request_id,
                )
                result = _require_operator_motion_receipt(
                    receipt,
                    operation="operator motion hold",
                    action=OperatorMotionAction.HOLD,
                    request_id=request_id,
                    source_id=source_id,
                    source_epoch=source_epoch,
                    sequence=sequence,
                )
            else:
                raise CommandBoundaryError("native operator motion hold capability is unavailable")
            if isinstance(result, OperatorMotionReceipt) and not result.final_output_published:
                error = result.reason or "operator motion hold did not publish final output"
            else:
                error = None
        except Exception as exc:
            error = str(exc)
            raise
        finally:
            with self._condition:
                self._last_error = error
                self._accepting = not self._closed and error is None
                self._condition.notify_all()
        return result

    def quiesce(self, *, timeout_s: float = 2.0) -> None:
        """Prevent queued/in-flight joystick samples from crossing a stop."""

        deadline = time.monotonic() + max(0.1, float(timeout_s))
        with self._condition:
            if self._closed:
                return
            self._accepting = False
            self._pending = None
            while self._inflight:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise CommandBoundaryError("native teleop publisher did not quiesce before stop")
                self._condition.wait(remaining)

    def resume(self) -> None:
        with self._condition:
            if not self._closed:
                self._last_error = None
                self._accepting = True
                self._condition.notify_all()

    def close(self, *, timeout_s: float = 2.0) -> None:
        with self._condition:
            self._closed = True
            self._accepting = False
            self._pending = None
            self._condition.notify_all()
        self._thread.join(timeout=max(0.0, float(timeout_s)))

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._closed:
                    self._condition.wait()
                if self._closed:
                    return
                command = self._pending
                self._pending = None
                self._inflight = True
            assert command is not None
            failure: dict[str, Any] | None = None
            try:
                client_sample = getattr(self._client, "sample", None)
                if client_sample is not None:
                    accepted = client_sample(
                        command.source_id,
                        command.source_epoch,
                        command.sequence,
                        command.vx,
                        command.vy,
                        command.wz,
                        deadman=True,
                        freshness_budget_ms=command.freshness_budget_ms,
                        request_id=command.request_id,
                    )
                    _require_literal_true_ack(
                        accepted,
                        operation="operator motion sample",
                    )
                else:
                    raise CommandBoundaryError("native operator motion sample capability is unavailable")
                error = None
            except Exception as exc:  # endpoint freshness still fails closed
                error = str(exc)
                failure = {
                    "source_id": command.source_id,
                    "source_epoch": command.source_epoch,
                    "source_sequence": command.sequence,
                    "request_id": command.request_id,
                    "error": error,
                    "stage": "dds_submission_failed",
                    "final_cmd_vel_confirmed": False,
                    "motor_confirmed": False,
                }
                logger.error("GatewayModule: native teleop publish failed: %s", exc)
            finally:
                with self._condition:
                    self._last_error = error
                    if error is not None:
                        self._accepting = False
                        self._pending = None
                    self._inflight = False
                    self._condition.notify_all()
            if failure is not None:
                self._notify_failure(failure)

    def _notify_failure(self, failure: dict[str, Any]) -> None:
        callback = self._failure_callback
        if callback is None:
            return
        try:
            callback(dict(failure))
        except Exception as exc:
            logger.error("GatewayModule: native teleop failure callback failed: %s", exc)


def resolve_native_command_boundary(
    *,
    command_output_mode: str,
) -> bool:
    """Resolve the Product command boundary from its compiled policy."""

    mode = (command_output_mode or "").strip().lower()
    if mode == "endpoint_only":
        return True
    if mode in {"", "local_driver"}:
        return False
    raise ValueError(f"unsupported command_output_mode: {command_output_mode!r}")


def parse_bridge_addr(raw: str) -> tuple[str, int] | None:
    raw = (raw or "").strip()
    if not raw:
        return None
    if ":" not in raw:
        raise ValueError("LINGTU_TELEOP_BRIDGE_ADDR must be HOST:PORT")
    host, port = raw.rsplit(":", 1)
    return host.strip(), int(port)


def init_teleop_state(
    gw: Any,
    *,
    max_speed: float,
    max_yaw: float,
    release_timeout: float,
    bridge_addr_raw: str,
    dds_enabled: bool,
) -> None:
    gw._teleop_module = None
    gw._camera_module = None
    gw._teleop_clients = 0
    gw._teleop_clients_lock = threading.Lock()
    gw._latest_jpeg = None
    gw._latest_jpeg_seq = 0
    gw._jpeg_lock = threading.Lock()
    gw._teleop_max_speed = max_speed
    gw._teleop_max_yaw = max_yaw
    gw._teleop_release_timeout = release_timeout
    gw._teleop_bridge_addr = parse_bridge_addr(bridge_addr_raw)
    gw._teleop_bridge_sock = (
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if gw._teleop_bridge_addr is not None else None
    )
    gw._teleop_dds_enabled = dds_enabled
    gw._teleop_native_publisher = None


def bind_navigation_commands(gw: Any, commands: Any | None) -> None:
    """Bind the Blueprint-assembled command capability after module discovery."""

    gw._nav_commands = commands
    publisher = getattr(gw, "_teleop_native_publisher", None)
    if publisher is not None:
        publisher.close()
    gw._teleop_native_publisher = None
    if not bool(getattr(gw, "_teleop_dds_enabled", False)):
        return
    command_client = None
    if commands is not None:
        has_typed_operator_motion = all(
            callable(getattr(commands, name, None))
            for name in ("claim", "sample", "hold", "release")
        )
        if has_typed_operator_motion:
            command_client = commands
    if command_client is None:
        logger.error(
            "GatewayModule: %s disabled because nav.commands lacks typed operator motion",
            TOPICS.teleop_cmd_vel,
        )
        return
    push_event = getattr(gw, "push_event", None)

    def report_sample_failure(failure: dict[str, Any]) -> None:
        if callable(push_event):
            push_event(
                {
                    "type": "operator_motion_sample_failed",
                    "data": failure,
                }
            )

    gw._teleop_native_publisher = LatestNativeTeleopPublisher(
        command_client,
        failure_callback=report_sample_failure,
    )


def configure_teleop(
    gw: Any,
    *,
    max_speed: float,
    max_yaw: float,
    release_timeout: float,
) -> None:
    gw._teleop_max_speed = max_speed
    gw._teleop_max_yaw = max_yaw
    gw._teleop_release_timeout = release_timeout


def write_bridge(gw: Any, twist: Twist) -> bool:
    sock = gw._teleop_bridge_sock
    addr = gw._teleop_bridge_addr
    if sock is None or addr is None:
        return False
    try:
        payload = (f"{float(twist.linear.x):.9g} {float(twist.linear.y):.9g} {float(twist.angular.z):.9g}\n").encode(
            "ascii"
        )
        sock.sendto(payload, addr)
        return True
    except Exception as exc:
        logger.debug("GatewayModule: teleop bridge write failed: %s", exc)
        return False


def publish_remote_velocity_request(
    gw: Any,
    twist: Twist,
    *,
    publish_local_compat: bool = True,
    request_id: str | None = None,
    source_id: str | None = None,
    source_epoch: int | None = None,
    sequence: int | None = None,
) -> bool:
    """Publish an operator velocity request.

    Field endpoints send typed operator-motion samples to the native command
    arbiter. It checks authority/freshness/localization/live obstacles/
    traversability and is the only writer of final ``/nav/cmd_vel``. The
    in-process publish is kept only for explicit dev/sim/compat profiles.
    """

    if bool(getattr(gw, "_teleop_dds_enabled", False)):
        publisher = getattr(gw, "_teleop_native_publisher", None)
        if publisher is None:
            raise CommandBoundaryError("native operator motion capability is unavailable")
        normalized_source_id = str(source_id or "").strip()
        normalized_source_epoch = int(source_epoch or 0)
        normalized_sequence = int(sequence or 0)
        if (
            not normalized_source_id
            or normalized_source_epoch <= 0
            or normalized_sequence <= 0
        ):
            raise CommandBoundaryError(
                "native operator motion requires an explicit claimed source session; "
                "use /ws/teleop or the typed operator-motion client"
            )
        if not publisher.submit(
            twist.linear.x,
            twist.linear.y,
            twist.angular.z,
            request_id=request_id,
            source_id=normalized_source_id,
            source_epoch=normalized_source_epoch,
            sequence=normalized_sequence,
        ):
            raise CommandBoundaryError("native operator motion publisher rejected the request")
        return True

    wrote_dds = write_bridge(gw, twist)
    if publish_local_compat:
        gw.cmd_vel.publish(twist)
    return wrote_dds


def twist_from_joy(gw: Any, lx: float, ly: float, az: float) -> Twist:
    vx = max(-1.0, min(1.0, float(lx))) * getattr(gw, "_teleop_max_speed", 0.5)
    vy = max(-1.0, min(1.0, float(ly))) * getattr(gw, "_teleop_max_speed", 0.5)
    wz = max(-1.0, min(1.0, float(az))) * getattr(gw, "_teleop_max_yaw", 1.0)
    return Twist(
        linear=Vector3(x=vx, y=vy, z=0.0),
        angular=Vector3(x=0.0, y=0.0, z=wz),
    )


def on_joy(gw: Any, lx: float, ly: float, az: float) -> bool:
    return on_joy_with_request_id(gw, lx, ly, az, request_id=None)


def on_joy_with_request_id(
    gw: Any,
    lx: float,
    ly: float,
    az: float,
    *,
    request_id: str | None = None,
    source_id: str = "gateway:teleop",
    source_epoch: int = 1,
    sequence: int = 1,
) -> bool:
    request = twist_from_joy(gw, lx, ly, az)
    if bool(getattr(gw, "_teleop_dds_enabled", False)):
        publisher = getattr(gw, "_teleop_native_publisher", None)
        if publisher is None:
            logger.error("GatewayModule: dropping field teleop request: native teleop publisher is unavailable")
            return False
        if not publisher.submit(
            request.linear.x,
            request.linear.y,
            request.angular.z,
            request_id=request_id,
            source_id=source_id,
            source_epoch=source_epoch,
            sequence=sequence,
        ):
            logger.debug("GatewayModule: teleop request rejected by stop/release barrier")
            return False
        return True
    tm = gw._teleop_module
    if tm is not None and hasattr(tm, "joy_input"):
        tm.joy_input._deliver({"lx": lx, "ly": ly, "az": az})
    else:
        gw.cmd_vel.publish(request)
    return True


def claim(
    gw: Any,
    *,
    source_id: str,
    source_epoch: int,
    sequence: int,
    lease_ttl_ms: int,
    request_id: str | None = None,
) -> OperatorMotionReceipt | bool | None:
    if not bool(getattr(gw, "_teleop_dds_enabled", False)):
        return True
    publisher = getattr(gw, "_teleop_native_publisher", None)
    if publisher is None:
        logger.error("GatewayModule: field teleop claim failed: publisher unavailable")
        return None
    try:
        receipt = publisher.claim(
            source_id=source_id,
            source_epoch=source_epoch,
            sequence=sequence,
            lease_ttl_ms=lease_ttl_ms,
            request_id=request_id,
        )
    except Exception as exc:
        logger.error("GatewayModule: field teleop claim failed: %s", exc)
        return None
    if not isinstance(receipt, OperatorMotionReceipt):
        logger.error("GatewayModule: field teleop claim failed: invalid receipt")
        return None
    if not receipt.source_accepted:
        logger.error("GatewayModule: field teleop claim rejected: %s", receipt.reason)
    return receipt


def release(
    gw: Any,
    *,
    source_id: str = "gateway:teleop",
    source_epoch: int = 1,
    sequence: int = 1,
    release_sequence: int | None = None,
    request_id: str | None = None,
    reason: str = "operator_hold",
) -> OperatorMotionReceipt | bool | None:
    if bool(getattr(gw, "_teleop_dds_enabled", False)):
        publisher = getattr(gw, "_teleop_native_publisher", None)
        if publisher is None:
            logger.error("GatewayModule: field teleop release failed: publisher unavailable")
            return None
        try:
            if reason == "disconnect" and (
                release_sequence is None or int(release_sequence) <= int(sequence)
            ):
                raise CommandBoundaryError(
                    "disconnect release requires a sequence newer than the hold"
                )
            hold_receipt = publisher.quiesce_and_send_zero(
                request_id=request_id,
                source_id=source_id,
                source_epoch=source_epoch,
                sequence=sequence,
                reason=reason,
                timeout_s=max(2.0, float(gw._teleop_release_timeout) + 1.0),
            )
            if not isinstance(hold_receipt, OperatorMotionReceipt):
                raise CommandBoundaryError("native endpoint returned no hold receipt")
            if not hold_receipt.final_output_published:
                return hold_receipt
            if reason == "disconnect":
                release_receipt = publisher.release_source(
                    source_id=source_id,
                    source_epoch=source_epoch,
                    sequence=int(release_sequence),
                    reason="disconnect",
                    request_id=f"{request_id}:release" if request_id else None,
                )
                if not isinstance(release_receipt, OperatorMotionReceipt):
                    raise CommandBoundaryError("native endpoint returned no release receipt")
                if not release_receipt.final_output_published:
                    return release_receipt
                return release_receipt
        except CommandBoundaryError as exc:
            logger.error("GatewayModule: field teleop release failed: %s", exc)
            return None
        except Exception as exc:
            logger.error("GatewayModule: field teleop release zero failed: %s", exc)
            return None
        return hold_receipt
    tm = gw._teleop_module
    if tm is not None:
        tm.force_release()
    else:
        gw.cmd_vel.publish(Twist())
    return True


def quiesce_native_teleop(
    gw: Any,
    *,
    source_id: str = "gateway:teleop",
    source_epoch: int = 1,
    sequence: int = 1,
    request_id: str | None = None,
    reason: str = "operator_hold",
    timeout_s: float = 2.0,
) -> OperatorMotionReceipt | bool:
    """Drain replaceable joystick work and publish a zero barrier."""

    if not bool(getattr(gw, "_teleop_dds_enabled", False)):
        return False
    publisher = getattr(gw, "_teleop_native_publisher", None)
    if publisher is None:
        raise CommandBoundaryError("native teleop publisher is unavailable")
    return publisher.quiesce_and_send_zero(
        request_id=request_id,
        source_id=source_id,
        source_epoch=source_epoch,
        sequence=sequence,
        reason=reason,
        timeout_s=max(2.0, float(timeout_s)),
    )


def resume_native_teleop(gw: Any) -> None:
    publisher = getattr(gw, "_teleop_native_publisher", None)
    if publisher is not None:
        publisher.resume()


def shutdown_teleop(gw: Any) -> None:
    publisher = getattr(gw, "_teleop_native_publisher", None)
    if publisher is not None:
        publisher.close()
    gw._teleop_native_publisher = None
