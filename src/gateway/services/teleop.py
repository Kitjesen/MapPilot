"""Teleop gateway helpers.

This module owns the gateway-side transport details for operator velocity
requests. GatewayModule keeps the public method names while the socket/DDS
mechanics live here.
"""

from __future__ import annotations

import logging
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
    manual_mode: bool
    freshness_budget_ms: int


class LatestNativeTeleopPublisher:
    """Non-blocking latest-value publisher for high-rate velocity commands.

    Ordinary velocity samples are replaceable: while one typed DDS submission
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
        manual_mode: bool = False,
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
                bool(manual_mode),
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
        """Prevent queued/in-flight velocity samples from crossing a stop."""

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
                        manual_mode=command.manual_mode,
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


NATIVE_TELEOP_LEASE_TTL_MS = 1000
NATIVE_TELEOP_RECLAIM_AFTER_S = NATIVE_TELEOP_LEASE_TTL_MS * 0.5e-3
_CLAIM_RETRY_REASONS = {
    "authority_lease_expired",
    "authority_change_requires_zero_barrier",
    "stale_epoch",
    "zero_barrier_pending",
}


@dataclass(frozen=True)
class TeleopSessionResult:
    """Outcome kept inside Gateway's Web teleop adapter."""

    accepted: bool
    reason: str
    final_output_confirmed: bool = False


class NativeTeleopSession:
    """Connection-scoped Web teleop session.

    The browser only connects, moves, holds, and disconnects. Native lease,
    epoch, sequence, zero-barrier, and reclaim details remain inside Gateway.
    """

    def __init__(self, gw: Any, source_id: str) -> None:
        self._gw = gw
        self._source_id = str(source_id)
        self._source_epoch = max(1, time.monotonic_ns())
        self._sequence = 0
        self._opened = False
        self._owns_native_authority = False
        self._claim_before_move = True
        self._last_native_activity_s = 0.0

    def open(self) -> TeleopSessionResult:
        """Reserve the one connection-scoped Web controller slot."""

        lock = self._gw._web_teleop_owner_lock
        with lock:
            owner = self._gw._web_teleop_owner
            if owner not in (None, self._source_id):
                return TeleopSessionResult(False, "connection_in_use")
            self._gw._web_teleop_owner = self._source_id
        self._opened = True
        return TeleopSessionResult(True, "connected")

    def move(
        self,
        vx_mps: float,
        vy_mps: float,
        yaw_rps: float,
        *,
        request_id: str,
        manual_mode: bool = False,
    ) -> TeleopSessionResult:
        """Claim native authority when needed and queue the latest velocity."""

        if not self._opened:
            return TeleopSessionResult(False, "session_closed")
        now = time.monotonic()
        if (
            self._claim_before_move
            or not self._owns_native_authority
            or now - self._last_native_activity_s >= NATIVE_TELEOP_RECLAIM_AFTER_S
        ):
            claimed = self._ensure_native_authority()
            if not claimed.accepted:
                return claimed
        sequence = self._next_sequence()
        accepted = bool(
            self._gw._teleop_on_velocity(
                vx_mps,
                vy_mps,
                yaw_rps,
                request_id=request_id,
                source_id=self._source_id,
                source_epoch=self._source_epoch,
                sequence=sequence,
                manual_mode=manual_mode,
            )
        )
        if not accepted:
            self._claim_before_move = True
            return TeleopSessionResult(False, "publisher_unavailable")
        self._last_native_activity_s = time.monotonic()
        return TeleopSessionResult(True, "queued")

    def hold(self, *, request_id: str, reason: str = "web_operator_hold") -> TeleopSessionResult:
        """Publish an ordered zero barrier for this session when it has control."""

        if not self._opened:
            return TeleopSessionResult(False, "session_closed")
        if not self._owns_native_authority:
            self._claim_before_move = True
            return TeleopSessionResult(True, "already_idle")
        receipt = self._gw._teleop_release(
            source_id=self._source_id,
            source_epoch=self._source_epoch,
            sequence=self._next_sequence(),
            request_id=request_id,
            reason=reason,
        )
        self._claim_before_move = True
        self._last_native_activity_s = time.monotonic()
        if isinstance(receipt, OperatorMotionReceipt) and receipt.final_output_published:
            return TeleopSessionResult(True, "held", final_output_confirmed=True)
        internal_reason = receipt.reason if isinstance(receipt, OperatorMotionReceipt) else "hold_failed"
        return TeleopSessionResult(False, internal_reason)

    def disconnect(self, *, request_id: str) -> TeleopSessionResult:
        """Hold, release native authority, and free the Web controller slot."""

        try:
            if not self._owns_native_authority:
                return TeleopSessionResult(True, "already_idle")
            hold_sequence = self._next_sequence()
            receipt = self._gw._teleop_release(
                source_id=self._source_id,
                source_epoch=self._source_epoch,
                sequence=hold_sequence,
                release_sequence=self._next_sequence(),
                request_id=request_id,
                reason="disconnect",
            )
            if isinstance(receipt, OperatorMotionReceipt) and receipt.final_output_published:
                return TeleopSessionResult(True, "disconnected", final_output_confirmed=True)
            internal_reason = (
                receipt.reason if isinstance(receipt, OperatorMotionReceipt) else "disconnect_zero_failed"
            )
            return TeleopSessionResult(False, internal_reason)
        finally:
            self._owns_native_authority = False
            self._claim_before_move = True
            self._opened = False
            with self._gw._web_teleop_owner_lock:
                if self._gw._web_teleop_owner == self._source_id:
                    self._gw._web_teleop_owner = None

    def _ensure_native_authority(self) -> TeleopSessionResult:
        publisher = getattr(self._gw, "_teleop_native_publisher", None)
        publisher_quiesce = getattr(publisher, "quiesce", None)
        publisher_resume = getattr(publisher, "resume", None)
        if callable(publisher_quiesce):
            try:
                publisher_quiesce()
            except Exception as exc:
                logger.error("GatewayModule: native teleop claim barrier failed: %s", exc)
                return TeleopSessionResult(False, "publisher_unavailable")
        last_reason = "claim_failed"
        for _ in range(3):
            sequence = self._next_sequence()
            receipt = self._gw._teleop_claim(
                source_id=self._source_id,
                source_epoch=self._source_epoch,
                sequence=sequence,
                lease_ttl_ms=NATIVE_TELEOP_LEASE_TTL_MS,
                request_id=f"{self._source_id}:claim:{sequence}",
            )
            if isinstance(receipt, OperatorMotionReceipt) and receipt.source_accepted:
                if callable(publisher_resume):
                    publisher_resume()
                self._owns_native_authority = True
                self._claim_before_move = False
                self._last_native_activity_s = time.monotonic()
                return TeleopSessionResult(True, "claimed")
            last_reason = receipt.reason if isinstance(receipt, OperatorMotionReceipt) else "claim_failed"
            if last_reason not in _CLAIM_RETRY_REASONS:
                break
            if last_reason in {"authority_lease_expired", "stale_epoch"}:
                self._source_epoch = max(self._source_epoch + 1, time.monotonic_ns())
        self._owns_native_authority = False
        self._claim_before_move = True
        if callable(publisher_resume):
            publisher_resume()
        return TeleopSessionResult(False, last_reason)

    def _next_sequence(self) -> int:
        self._sequence += 1
        return self._sequence


def init_teleop_state(
    gw: Any,
    *,
    max_speed: float,
    max_yaw: float,
    release_timeout: float,
) -> None:
    gw._camera_module = None
    gw._teleop_clients = 0
    gw._teleop_clients_lock = threading.Lock()
    gw._web_teleop_owner = None
    gw._web_teleop_owner_lock = threading.Lock()
    gw._latest_jpeg = None
    gw._latest_jpeg_seq = 0
    gw._jpeg_lock = threading.Lock()
    gw._teleop_max_speed = max_speed
    gw._teleop_max_yaw = max_yaw
    gw._teleop_release_timeout = release_timeout
    gw._teleop_native_publisher = None


def bind_navigation_commands(gw: Any, commands: Any | None) -> None:
    """Bind the Blueprint-assembled command capability after module discovery."""

    gw._nav_commands = commands
    publisher = getattr(gw, "_teleop_native_publisher", None)
    if publisher is not None:
        publisher.close()
    gw._teleop_native_publisher = None
    command_client = None
    if commands is not None:
        has_typed_operator_motion = all(
            callable(getattr(commands, name, None))
            for name in ("claim", "sample", "hold", "release")
        )
        if has_typed_operator_motion:
            command_client = commands
    if command_client is None:
        logger.error("GatewayModule teleop disabled because nav.commands lacks typed operator motion")
        return
    push_event = getattr(gw, "push_event", None)

    def report_sample_failure(failure: dict[str, Any]) -> None:
        if callable(push_event):
            push_event(
                {
                    "type": "teleop_command_failed",
                    "data": {
                        "request_id": failure.get("request_id"),
                        "error": "control_unavailable",
                        "stage": "dds_submission_failed",
                        "final_cmd_vel_confirmed": False,
                        "motor_confirmed": False,
                    },
                }
            )

    gw._teleop_native_publisher = LatestNativeTeleopPublisher(
        command_client,
        failure_callback=report_sample_failure,
    )


def publish_remote_velocity_request(
    gw: Any,
    twist: Twist,
    *,
    request_id: str | None = None,
    source_id: str | None = None,
    source_epoch: int | None = None,
    sequence: int | None = None,
    manual_mode: bool = False,
) -> bool:
    """Publish an operator velocity request.

    Send a typed operator-motion sample to the native command arbiter.
    """
    publisher = getattr(gw, "_teleop_native_publisher", None)
    if publisher is None:
        raise CommandBoundaryError("native operator motion capability is unavailable")
    normalized_source_id = str(source_id or "").strip()
    normalized_source_epoch = int(source_epoch or 0)
    normalized_sequence = int(sequence or 0)
    if not normalized_source_id or normalized_source_epoch <= 0 or normalized_sequence <= 0:
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
        manual_mode=manual_mode,
    ):
        raise CommandBoundaryError("native operator motion publisher rejected the request")
    return True


def twist_from_velocity(
    gw: Any,
    vx_mps: float,
    vy_mps: float,
    yaw_rps: float,
) -> Twist:
    max_speed = abs(float(getattr(gw, "_teleop_max_speed", 0.5)))
    max_yaw = abs(float(getattr(gw, "_teleop_max_yaw", 1.0)))
    vx = max(-max_speed, min(max_speed, float(vx_mps)))
    vy = max(-max_speed, min(max_speed, float(vy_mps)))
    wz = max(-max_yaw, min(max_yaw, float(yaw_rps)))
    return Twist(
        linear=Vector3(x=vx, y=vy, z=0.0),
        angular=Vector3(x=0.0, y=0.0, z=wz),
    )


def on_velocity(gw: Any, vx_mps: float, vy_mps: float, yaw_rps: float) -> bool:
    return on_velocity_with_request_id(
        gw,
        vx_mps,
        vy_mps,
        yaw_rps,
        request_id=None,
    )


def on_velocity_with_request_id(
    gw: Any,
    vx_mps: float,
    vy_mps: float,
    yaw_rps: float,
    *,
    request_id: str | None = None,
    source_id: str = "gateway:teleop",
    source_epoch: int = 1,
    sequence: int = 1,
    manual_mode: bool = False,
) -> bool:
    request = twist_from_velocity(gw, vx_mps, vy_mps, yaw_rps)
    publisher = getattr(gw, "_teleop_native_publisher", None)
    if publisher is None:
        logger.error("GatewayModule: dropping teleop request: native teleop publisher is unavailable")
        return False
    if not publisher.submit(
        request.linear.x,
        request.linear.y,
        request.angular.z,
        request_id=request_id,
        source_id=source_id,
        source_epoch=source_epoch,
        sequence=sequence,
        manual_mode=manual_mode,
    ):
        logger.debug("GatewayModule: teleop request rejected by stop/release barrier")
        return False
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
    """Drain replaceable velocity work and publish a zero barrier."""

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
    owner_lock = getattr(gw, "_web_teleop_owner_lock", None)
    if owner_lock is not None:
        with owner_lock:
            gw._web_teleop_owner = None
