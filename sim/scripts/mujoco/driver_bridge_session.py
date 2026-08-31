"""Strict controller-side session for the native MuJoCo driver protocol.

This module owns only protocol state.  Transport and process lifetime remain
owned by its caller.
"""

from __future__ import annotations

import math
import re
import secrets
import time
from dataclasses import dataclass
from typing import Callable

from lingtu.switch_contracts import is_product_session_id

_MAX_UINT64 = (1 << 64) - 1
_BOOT_ID_RE = re.compile(r"[0-9a-f]{32}")
_FLOAT_RE = re.compile(r"-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?")
_KINDS = frozenset(
    {
        "activation_zero",
        "nav",
        "deactivate_zero",
        "writer_fault_zero",
        "safety_zero",
    }
)

__all__ = [
    "DriverBridgeCommand",
    "DriverBridgeSession",
    "DriverBridgeSessionError",
    "DriverBridgeStoppedEvidence",
]


class DriverBridgeSessionError(RuntimeError):
    """The peer or transport violated the fail-closed driver session."""


@dataclass(frozen=True)
class DriverBridgeCommand:
    bridge_boot_id: str
    controller_boot_id: str
    bridge_command_seq: int
    kind: str
    producer_boot_id: str
    output_sequence: int
    walk_x: float
    walk_y: float
    walk_z: float


@dataclass(frozen=True)
class DriverBridgeStoppedEvidence:
    bridge_boot_id: str
    controller_boot_id: str
    bridge_command_seq: int
    applied_step_seq: int
    kind: str
    producer_boot_id: str
    output_sequence: int
    walk_x: float
    walk_y: float
    walk_z: float
    terminal_ack: bool


def _valid_token(value: str) -> bool:
    return (
        bool(value)
        and len(value) <= 128
        and value.isascii()
        and value[0].isalnum()
        and all(character.isalnum() or character in "._:@-" for character in value)
    )


def _positive_uint64(value: str, field: str) -> int:
    parsed = _uint64(value, field)
    if parsed == 0:
        raise DriverBridgeSessionError(f"{field} must be positive")
    return parsed


def _uint64(value: str, field: str) -> int:
    if not value or not value.isascii() or not value.isdecimal():
        raise DriverBridgeSessionError(f"{field} must be decimal uint64")
    parsed = int(value)
    if parsed > _MAX_UINT64:
        raise DriverBridgeSessionError(f"{field} overflows uint64")
    return parsed


def _finite_float(value: str, field: str) -> float:
    if _FLOAT_RE.fullmatch(value) is None:
        raise DriverBridgeSessionError(f"{field} has invalid decimal grammar")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise DriverBridgeSessionError(f"{field} must be finite")
    return parsed


def _exact_positive_zero(value: float) -> bool:
    return value == 0.0 and math.copysign(1.0, value) > 0.0


class DriverBridgeSession:
    """Drive one authenticated driver-v2 stream through typed operations."""

    def __init__(
        self,
        *,
        send_line: Callable[[str], None],
        recv_line: Callable[[], str],
        close: Callable[[], None],
        expected_product_session_id: str,
        operation_timeout_s: float = 2.0,
        max_line_bytes: int = 512,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not callable(send_line) or not callable(recv_line) or not callable(close):
            raise TypeError("driver bridge transport callbacks must be callable")
        if not is_product_session_id(expected_product_session_id):
            raise ValueError("expected_product_session_id is invalid")
        if (
            isinstance(operation_timeout_s, bool)
            or not isinstance(operation_timeout_s, (int, float))
            or not math.isfinite(float(operation_timeout_s))
            or float(operation_timeout_s) <= 0.0
        ):
            raise ValueError("operation_timeout_s must be finite and positive")
        if (
            isinstance(max_line_bytes, bool)
            or not isinstance(max_line_bytes, int)
            or not 1 <= max_line_bytes <= 512
        ):
            raise ValueError("max_line_bytes must be an integer in [1, 512]")
        if not callable(clock):
            raise TypeError("clock must be callable")

        self._send_line = send_line
        self._recv_line = recv_line
        self._close_transport = close
        self._expected_product_session_id = expected_product_session_id
        self._operation_timeout_s = float(operation_timeout_s)
        self._max_line_bytes = max_line_bytes
        self._clock = clock
        self._controller_boot_id = secrets.token_hex(16)
        self._bridge_boot_id = ""
        self._state = "new"
        self._failed = False
        self._closed = False
        self._control_seq = 0
        self._last_command_seq = 0
        self._last_step_seq = 0
        self._inflight: DriverBridgeCommand | None = None
        self._queued_command: DriverBridgeCommand | None = None
        self._last_applied: tuple[DriverBridgeCommand, int] | None = None
        self._ready_expected = False
        self._heartbeat_after_applied = False

    @property
    def controller_boot_id(self) -> str:
        return self._controller_boot_id

    @property
    def bridge_boot_id(self) -> str:
        if not self._bridge_boot_id:
            raise DriverBridgeSessionError("driver bridge HELLO has not been received")
        return self._bridge_boot_id

    def activate(self) -> DriverBridgeCommand:
        """Bind HELLO identity, send ACTIVATE, and return its physical zero."""

        self._require_state("new")
        deadline = self._deadline()
        try:
            fields = self._receive_fields(deadline)
            if len(fields) != 2 or fields[0] != "LT_DRIVER_HELLO_V2":
                raise DriverBridgeSessionError("expected exact LT_DRIVER_HELLO_V2")
            self._require_boot_id(fields[1], "bridge_boot_id")
            self._bridge_boot_id = fields[1]
            self._control_seq = 1
            self._send(
                "LT_DRIVER_ACTIVATE_V2"
                f"\t{self._bridge_boot_id}\t{self._controller_boot_id}\t1",
                deadline,
            )
            command = self._receive_command(deadline, expected_kind="activation_zero")
            self._inflight = command
            self._state = "activating"
            return command
        except Exception:
            self._failed = True
            raise

    def complete_step(self, command: DriverBridgeCommand, *, step_seq: int) -> None:
        """Report one command only after the caller physically applied its walk."""

        self._require_state(
            "activating", "applying", "deactivating_pending", "deactivating"
        )
        if command is not self._inflight:
            self._failed = True
            raise DriverBridgeSessionError("APPLIED command is not the in-flight evidence")
        drains_pending_deactivation = self._state == "deactivating_pending"
        try:
            parsed_step_seq = self._validated_step_seq(step_seq)
            producer = command.producer_boot_id or "-"
            deadline = self._deadline()
            self._send(
                "LT_DRIVER_APPLIED_V2"
                f"\t{command.bridge_boot_id}\t{command.controller_boot_id}"
                f"\t{command.bridge_command_seq}\t{command.kind}\t{producer}"
                f"\t{command.output_sequence}\t{self._format_float(command.walk_x)}"
                f"\t{self._format_float(command.walk_y)}"
                f"\t{self._format_float(command.walk_z)}\t{parsed_step_seq}",
                deadline,
            )
        except Exception:
            self._failed = True
            raise
        self._inflight = None
        self._last_step_seq = parsed_step_seq
        self._last_applied = (command, parsed_step_seq)
        self._heartbeat_after_applied = False
        if command.kind == "deactivate_zero":
            self._ready_expected = False
            self._state = "awaiting_stopped"
        elif drains_pending_deactivation:
            self._ready_expected = False
            self._state = "deactivation_waiting_zero"
        else:
            self._ready_expected = True
            self._state = "awaiting_ready"

    def heartbeat(self, *, step_seq: int) -> None:
        """Send one monotonic controller heartbeat without consuming peer output."""

        self._require_state("active", "awaiting_ready")
        try:
            parsed_step_seq = self._validated_step_seq(step_seq)
            self._control_seq += 1
            deadline = self._deadline()
            self._send(
                "LT_DRIVER_HEARTBEAT_V2"
                f"\t{self._bridge_boot_id}\t{self._controller_boot_id}"
                f"\t{self._control_seq}\t{parsed_step_seq}",
                deadline,
            )
        except Exception:
            self._failed = True
            raise
        self._last_step_seq = parsed_step_seq
        if self._ready_expected:
            self._heartbeat_after_applied = True

    def confirm_ready(self) -> None:
        """Consume the exact READY ACK for the last physically applied command."""

        self._require_state("awaiting_ready")
        if not self._heartbeat_after_applied or self._last_applied is None:
            self._failed = True
            raise DriverBridgeSessionError("READY requires a later physical heartbeat")
        deadline = self._deadline()
        try:
            while True:
                fields = self._receive_fields(deadline)
                if fields and fields[0] == "LT_DRIVER_FAULT_V2":
                    _raise_native_fault(fields)
                if fields and fields[0] == "LT_DRIVER_COMMAND_V2":
                    if self._queued_command is not None:
                        raise DriverBridgeSessionError(
                            "multiple driver commands arrived before READY"
                        )
                    queued = self._parse_command_fields(fields)
                    if queued.kind not in {
                        "nav",
                        "writer_fault_zero",
                        "safety_zero",
                    }:
                        raise DriverBridgeSessionError(
                            "unexpected command before READY"
                        )
                    self._queued_command = queued
                    if queued.kind in {"writer_fault_zero", "safety_zero"}:
                        # A native safety transition replaces the prior READY
                        # with a physical zero that must be applied first.
                        self._ready_expected = False
                        self._heartbeat_after_applied = False
                        self._state = "command_available"
                        return
                    continue
                if len(fields) != 6 or fields[0] != "LT_DRIVER_READY_V2":
                    raise DriverBridgeSessionError("expected exact LT_DRIVER_READY_V2")
                self._require_identity(fields[1], fields[2])
                accepted_sequence = _positive_uint64(
                    fields[3], "accepted_sequence"
                )
                producer = "" if fields[4] == "-" else fields[4]
                output_sequence = _uint64(fields[5], "accepted_output_sequence")
                applied, _ = self._last_applied
                if (
                    accepted_sequence != applied.bridge_command_seq
                    or producer != applied.producer_boot_id
                    or output_sequence != applied.output_sequence
                ):
                    raise DriverBridgeSessionError(
                        "READY does not ACK the applied command"
                    )
                break
        except Exception:
            self._failed = True
            raise
        self._ready_expected = False
        self._heartbeat_after_applied = False
        self._state = "command_available" if self._queued_command else "active"

    def receive_command(self) -> DriverBridgeCommand:
        """Receive the next native motion command without owning its application."""

        self._require_state("active", "command_available")
        try:
            if self._queued_command is not None:
                command = self._queued_command
                self._queued_command = None
            else:
                command = self._receive_command(self._deadline())
            if command.kind not in {"nav", "writer_fault_zero", "safety_zero"}:
                raise DriverBridgeSessionError("unexpected command in active session")
        except Exception:
            self._failed = True
            raise
        self._inflight = command
        self._state = "applying"
        return command

    def begin_deactivate(self) -> DriverBridgeCommand:
        """Request or continue shutdown and return its next physical command."""

        self._require_state(
            "active", "command_available", "deactivation_waiting_zero"
        )
        continuing = self._state == "deactivation_waiting_zero"
        deadline = self._deadline()
        try:
            if continuing:
                command = self._receive_command(
                    deadline, expected_kind="deactivate_zero"
                )
            else:
                self._control_seq += 1
                self._send(
                    "LT_DRIVER_DEACTIVATE_V2"
                    f"\t{self._bridge_boot_id}\t{self._controller_boot_id}"
                    f"\t{self._control_seq}",
                    deadline,
                )
                if self._queued_command is not None:
                    command = self._queued_command
                    self._queued_command = None
                else:
                    command = self._receive_command(deadline)
                if command.kind not in {"nav", "deactivate_zero"}:
                    raise DriverBridgeSessionError(
                        "unexpected command while deactivating"
                    )
        except Exception:
            self._failed = True
            raise
        self._inflight = command
        self._state = (
            "deactivating_pending" if command.kind == "nav" else "deactivating"
        )
        return command

    def wait_stopped(self) -> DriverBridgeStoppedEvidence:
        """Return terminal evidence only for the exact applied deactivation zero."""

        self._require_state("awaiting_stopped")
        if self._last_applied is None:
            self._failed = True
            raise DriverBridgeSessionError("STOPPED requires applied deactivation evidence")
        command, applied_step_seq = self._last_applied
        deadline = self._deadline()
        try:
            fields = self._receive_fields(deadline)
            if fields and fields[0] == "LT_DRIVER_FAULT_V2":
                _raise_native_fault(fields)
            if len(fields) != 6 or fields[0] != "LT_DRIVER_STOPPED_V2":
                raise DriverBridgeSessionError("expected exact LT_DRIVER_STOPPED_V2")
            self._require_identity(fields[1], fields[2])
            stopped_command_seq = _positive_uint64(
                fields[3], "stopped_bridge_command_seq"
            )
            stopped_step_seq = _positive_uint64(fields[4], "stopped_applied_step_seq")
            if (
                stopped_command_seq != command.bridge_command_seq
                or stopped_step_seq != applied_step_seq
                or fields[5] != "deactivate_zero"
                or command.kind != "deactivate_zero"
            ):
                raise DriverBridgeSessionError("STOPPED does not match deactivation APPLIED")
        except Exception:
            self._failed = True
            raise
        self._state = "stopped"
        return DriverBridgeStoppedEvidence(
            bridge_boot_id=self._bridge_boot_id,
            controller_boot_id=self._controller_boot_id,
            bridge_command_seq=command.bridge_command_seq,
            applied_step_seq=applied_step_seq,
            kind=command.kind,
            producer_boot_id=command.producer_boot_id,
            output_sequence=command.output_sequence,
            walk_x=command.walk_x,
            walk_y=command.walk_y,
            walk_z=command.walk_z,
            terminal_ack=True,
        )

    def close(self) -> None:
        """Close only the injected transport, at most once."""

        if self._closed:
            return
        self._closed = True
        try:
            self._close_transport()
        except Exception as exc:
            self._failed = True
            raise DriverBridgeSessionError("driver bridge close failed") from exc

    def _receive_command(
        self, deadline: float, *, expected_kind: str | None = None
    ) -> DriverBridgeCommand:
        fields = self._receive_fields(deadline)
        if fields and fields[0] == "LT_DRIVER_FAULT_V2":
            _raise_native_fault(fields)
        return self._parse_command_fields(fields, expected_kind=expected_kind)

    def _parse_command_fields(
        self, fields: list[str], *, expected_kind: str | None = None
    ) -> DriverBridgeCommand:
        if len(fields) != 10 or fields[0] != "LT_DRIVER_COMMAND_V2":
            raise DriverBridgeSessionError("expected exact LT_DRIVER_COMMAND_V2")
        self._require_identity(fields[1], fields[2])
        sequence = _positive_uint64(fields[3], "bridge_command_seq")
        if sequence != self._last_command_seq + 1:
            raise DriverBridgeSessionError(
                "bridge_command_seq must advance by exactly one"
            )
        kind = fields[4]
        if kind not in _KINDS or (expected_kind is not None and kind != expected_kind):
            raise DriverBridgeSessionError("driver bridge command kind is invalid")
        producer = "" if fields[5] == "-" else fields[5]
        output_sequence = _uint64(fields[6], "output_sequence")
        walk_x = _finite_float(fields[7], "walk_x")
        walk_y = _finite_float(fields[8], "walk_y")
        walk_z = _finite_float(fields[9], "walk_z")
        if any(
            value == 0.0 and not _exact_positive_zero(value)
            for value in (walk_x, walk_y, walk_z)
        ):
            raise DriverBridgeSessionError("driver walk must not contain negative zero")
        if any(abs(value) > 1.0 for value in (walk_x, walk_y, walk_z)):
            raise DriverBridgeSessionError("normalized driver walk exceeds [-1, 1]")
        if kind == "nav":
            if not self._valid_derived_producer(producer) or output_sequence == 0:
                raise DriverBridgeSessionError("nav producer identity is invalid")
        elif (
            producer
            or output_sequence != 0
            or not all(
                _exact_positive_zero(value) for value in (walk_x, walk_y, walk_z)
            )
        ):
            raise DriverBridgeSessionError("internal driver command is not exact zero")
        command = DriverBridgeCommand(
            bridge_boot_id=self._bridge_boot_id,
            controller_boot_id=self._controller_boot_id,
            bridge_command_seq=sequence,
            kind=kind,
            producer_boot_id=producer,
            output_sequence=output_sequence,
            walk_x=walk_x,
            walk_y=walk_y,
            walk_z=walk_z,
        )
        self._last_command_seq = sequence
        return command

    def _valid_derived_producer(self, value: str) -> bool:
        prefix = f"{self._expected_product_session_id}:"
        if not _valid_token(value) or not value.startswith(prefix):
            return False
        suffix = value[len(prefix) :].split(":")
        if len(suffix) != 2:
            return False
        try:
            process_id = _positive_uint64(suffix[0], "producer_process_id")
            start_boottime_ns = _positive_uint64(
                suffix[1], "producer_start_boottime_ns"
            )
        except DriverBridgeSessionError:
            return False
        return suffix == [str(process_id), str(start_boottime_ns)]

    def _validated_step_seq(self, step_seq: int) -> int:
        if (
            isinstance(step_seq, bool)
            or not isinstance(step_seq, int)
            or not 0 < step_seq <= _MAX_UINT64
            or step_seq <= self._last_step_seq
        ):
            raise DriverBridgeSessionError("controller step_seq must advance")
        return step_seq

    @staticmethod
    def _format_float(value: float) -> str:
        if not math.isfinite(value):
            raise DriverBridgeSessionError("driver bridge walk must be finite")
        if _exact_positive_zero(value):
            return "0"
        return format(value, ".17g")

    def _receive_fields(self, deadline: float) -> list[str]:
        try:
            line = self._recv_line()
        except Exception as exc:
            raise DriverBridgeSessionError(
                f"driver bridge receive failed in {self._state}: {exc}"
            ) from exc
        self._check_deadline(deadline)
        if not isinstance(line, str):
            raise DriverBridgeSessionError("driver bridge receive returned non-text data")
        if not line or "\n" in line or "\r" in line:
            raise DriverBridgeSessionError("driver bridge record must be one non-empty line")
        try:
            encoded = line.encode("ascii", errors="strict")
        except UnicodeEncodeError as exc:
            raise DriverBridgeSessionError("driver bridge record must be ASCII") from exc
        if len(encoded) > self._max_line_bytes:
            raise DriverBridgeSessionError("driver bridge record exceeds the byte limit")
        return line.split("\t")

    def _send(self, line: str, deadline: float) -> None:
        self._check_deadline(deadline)
        if not isinstance(line, str) or not line or "\n" in line or "\r" in line:
            raise DriverBridgeSessionError("driver bridge send requires one line")
        try:
            encoded = line.encode("ascii", errors="strict")
        except UnicodeEncodeError as exc:
            raise DriverBridgeSessionError("driver bridge send must be ASCII") from exc
        if len(encoded) > self._max_line_bytes:
            raise DriverBridgeSessionError("driver bridge send exceeds the byte limit")
        try:
            self._send_line(line)
        except Exception as exc:
            raise DriverBridgeSessionError("driver bridge send failed") from exc
        self._check_deadline(deadline)

    def _deadline(self) -> float:
        now = float(self._clock())
        if not math.isfinite(now):
            raise DriverBridgeSessionError("driver bridge clock is invalid")
        return now + self._operation_timeout_s

    def _check_deadline(self, deadline: float) -> None:
        now = float(self._clock())
        if not math.isfinite(now) or now >= deadline:
            raise DriverBridgeSessionError("driver bridge operation deadline expired")

    def _require_state(self, *allowed: str) -> None:
        if self._closed:
            raise DriverBridgeSessionError("driver bridge session is closed")
        if self._failed:
            raise DriverBridgeSessionError("driver bridge session is fault-closed")
        if self._state not in allowed:
            raise DriverBridgeSessionError("driver bridge operation is out of order")

    @staticmethod
    def _require_boot_id(value: str, field: str) -> None:
        if _BOOT_ID_RE.fullmatch(value) is None:
            raise DriverBridgeSessionError(f"{field} must be lowercase 32 hex")

    def _require_identity(self, bridge_boot_id: str, controller_boot_id: str) -> None:
        if (
            bridge_boot_id != self._bridge_boot_id
            or controller_boot_id != self._controller_boot_id
        ):
            raise DriverBridgeSessionError("driver bridge identity mismatch")
def _raise_native_fault(fields: list[str]) -> None:
    reason = fields[3] if len(fields) == 4 and fields[3] else "unspecified"
    raise DriverBridgeSessionError(f"native driver bridge reported FAULT: {reason}")
