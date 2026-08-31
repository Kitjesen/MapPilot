"""Strict loopback wire contract for RobotSimUE operator intent.

This module owns validation and bounded delivery only.  It deliberately has no
reference to ``RuntimeCoordinator``: a UDP receiver thread may deposit input,
but only the interactive simulation owner thread may consume it and submit a
controller command.
"""

from __future__ import annotations

import hashlib
import ipaddress
import json
import math
import re
import socket
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Mapping

CONTROL_INTENT_SCHEMA = "lingtu.sim.ue-control-intent.v1"
RUNTIME_REQUEST_SCHEMA = "lingtu.sim.ue-runtime-request.v1"
CONTROL_ACK_SCHEMA = "lingtu.sim.ue-control-ack.v1"
MAX_CONTROL_DATAGRAM_BYTES = 4096

_SAFE_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")
_SAFE_CONTROL_RE = re.compile(r"[a-z0-9][a-z0-9_.-]{0,63}\Z")
_MAX_INT64 = (1 << 63) - 1
_MOTION_FIELDS = frozenset(
    {
        "schema",
        "run_id",
        "session_id",
        "boot_id",
        "model_generation",
        "reset_generation",
        "source_id",
        "source_epoch",
        "source_sequence",
        "event_id",
        "input_mode",
        "ui_mode",
        "camera_mode",
        "input_device",
        "viewport_focused",
        "deadman",
        "axes",
        "active_controls",
        "source_monotonic_ns",
    }
)
_AXES_FIELDS = frozenset({"forward", "left", "yaw_left"})
_REQUEST_FIELDS = frozenset(
    {
        "schema",
        "run_id",
        "session_id",
        "boot_id",
        "model_generation",
        "reset_generation",
        "source_id",
        "source_epoch",
        "source_sequence",
        "event_id",
        "request",
        "ui_mode",
        "camera_mode",
        "source_monotonic_ns",
    }
)
_RUNTIME_REQUESTS = {
    "control_claim",
    "control_release",
    "pause",
    "resume",
    "record_start",
    "record_stop_commit",
    "safe_stop",
    "exit",
    "ui_state_update",
}
_ACK_FIELDS = frozenset(
    {
        "schema",
        "run_id",
        "session_id",
        "boot_id",
        "model_generation",
        "reset_generation",
        "server_status_sequence",
        "source_id",
        "source_epoch",
        "source_sequence",
        "event_id",
        "intent_datagram_sha256",
        "status",
        "reason",
    }
)
_ACK_STATUSES = {
    "pending",
    "accepted",
    "rejected",
    "released",
    "timeout_zero",
    "confirmed",
}


class ControlIntentValidationError(ValueError):
    """Raised when an operator-intent datagram cannot be admitted safely."""


@dataclass(frozen=True, slots=True)
class OperatorIntentIdentity:
    """Allocation-owned identity expected on every RobotSimUE input datagram."""

    run_id: str
    session_id: str
    boot_id: str
    model_generation: int
    reset_generation: int
    source_id: str

    def __post_init__(self) -> None:
        _safe_id(self.run_id, "run_id")
        _safe_id(self.session_id, "session_id")
        _safe_id(self.boot_id, "boot_id")
        _bounded_nonnegative_int(self.model_generation, "model_generation")
        _bounded_nonnegative_int(self.reset_generation, "reset_generation")
        _safe_id(self.source_id, "source_id")


@dataclass(frozen=True, slots=True)
class OperatorMotionAxes:
    """Normalized body-motion axes supplied by RobotSimUE."""

    forward: float
    left: float
    yaw_left: float


@dataclass(frozen=True, slots=True)
class OperatorMotionIntent:
    """One validated UE-origin normalized motion sample."""

    identity: OperatorIntentIdentity
    source_epoch: int
    source_sequence: int
    event_id: str
    input_mode: str
    input_device: str
    viewport_focused: bool
    deadman: bool
    axes: OperatorMotionAxes
    active_controls: tuple[str, ...]
    source_monotonic_ns: int
    arrival_monotonic_ns: int
    datagram_sha256: str
    ui_mode: str = "unavailable"
    camera_mode: str = "unavailable"


@dataclass(frozen=True, slots=True)
class OperatorRuntimeRequest:
    """One validated UE-origin lifecycle or recording request."""

    identity: OperatorIntentIdentity
    source_epoch: int
    source_sequence: int
    event_id: str
    request: str
    source_monotonic_ns: int
    arrival_monotonic_ns: int
    datagram_sha256: str
    ui_mode: str = "unavailable"
    camera_mode: str = "unavailable"


class LatestOperatorIntentInbox:
    """Thread-safe single-slot motion inbox with latest-wins semantics."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: OperatorMotionIntent | None = None

    def put(self, intent: OperatorMotionIntent) -> None:
        """Atomically replace any unread motion intent with ``intent``."""

        if not isinstance(intent, OperatorMotionIntent):
            raise TypeError("motion inbox accepts OperatorMotionIntent only")
        with self._lock:
            self._latest = intent

    def peek_latest(self) -> OperatorMotionIntent | None:
        """Return the current immutable intent without consuming it."""

        with self._lock:
            return self._latest

    def take_latest(self) -> OperatorMotionIntent | None:
        """Atomically return and clear the current latest intent."""

        with self._lock:
            latest = self._latest
            self._latest = None
            return latest

    def clear(self) -> None:
        """Discard the current value during pause, reset, stop, or teardown."""

        with self._lock:
            self._latest = None

    def __len__(self) -> int:
        with self._lock:
            return int(self._latest is not None)


class BoundedRuntimeRequestInbox:
    """Thread-safe non-dropping FIFO for lifecycle and recording requests."""

    def __init__(self, *, capacity: int = 32) -> None:
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity <= 0:
            raise ValueError("request inbox capacity must be a positive integer")
        self._capacity = capacity
        self._lock = threading.Lock()
        self._requests: deque[OperatorRuntimeRequest] = deque()
        self._seen_event_ids: set[str] = set()

    @property
    def capacity(self) -> int:
        """Return the configured hard queue bound."""

        return self._capacity

    def put(self, request: OperatorRuntimeRequest) -> None:
        """Append one request or fail explicitly without dropping existing work."""

        if not isinstance(request, OperatorRuntimeRequest):
            raise TypeError("request inbox accepts OperatorRuntimeRequest only")
        with self._lock:
            if request.event_id in self._seen_event_ids:
                raise ControlIntentValidationError(
                    f"runtime request has duplicate event_id {request.event_id!r}"
                )
            if len(self._requests) >= self._capacity:
                raise ControlIntentValidationError("runtime request queue is full")
            self._requests.append(request)
            self._seen_event_ids.add(request.event_id)

    def pop_request(self) -> OperatorRuntimeRequest | None:
        """Return and remove the oldest queued request, if any."""

        with self._lock:
            return self._requests.popleft() if self._requests else None

    def clear(self) -> None:
        """Discard requests and deduplication history for a lifecycle reset."""

        with self._lock:
            self._requests.clear()
            self._seen_event_ids.clear()

    def __len__(self) -> int:
        with self._lock:
            return len(self._requests)


class UdpLoopbackOperatorIntentReceiver:
    """Loopback-only UDP adapter that validates and deposits operator intent.

    The adapter intentionally exposes no coordinator or controller dependency.
    Its optional background thread performs only socket receive, validation,
    ordering, and inbox deposit.
    """

    def __init__(
        self,
        port: int,
        *,
        expected_identity: OperatorIntentIdentity,
        motion_inbox: LatestOperatorIntentInbox,
        request_inbox: BoundedRuntimeRequestInbox,
        host: str = "127.0.0.1",
        socket_factory: Callable[..., socket.socket] = socket.socket,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        receive_timeout_s: float = 0.05,
    ) -> None:
        if host != "127.0.0.1":
            raise ValueError("operator intent receiver must bind to 127.0.0.1")
        _port(port, "control intent port")
        if (
            isinstance(receive_timeout_s, bool)
            or not isinstance(receive_timeout_s, (int, float))
            or not math.isfinite(receive_timeout_s)
            or receive_timeout_s <= 0.0
        ):
            raise ValueError("receive_timeout_s must be a positive finite number")
        self._expected_identity = expected_identity
        self._motion_inbox = motion_inbox
        self._request_inbox = request_inbox
        self._monotonic_ns = monotonic_ns
        self._state_lock = threading.Lock()
        self._last_source_epoch: int | None = None
        self._last_source_sequence: int | None = None
        self._errors_lock = threading.Lock()
        self._errors: deque[ControlIntentValidationError] = deque(maxlen=32)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._closed = False
        transport = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        transport.bind((host, port))
        transport.settimeout(float(receive_timeout_s))
        self._socket = transport

    def receive_once(self) -> OperatorMotionIntent | OperatorRuntimeRequest | None:
        """Receive, validate, order, and deposit at most one datagram."""

        try:
            payload, source = self._socket.recvfrom(MAX_CONTROL_DATAGRAM_BYTES + 1)
        except TimeoutError:
            return None
        if not _is_ipv4_loopback_source(source):
            raise ControlIntentValidationError(
                "operator intent source must be IPv4 loopback"
            )
        arrival_monotonic_ns = self._monotonic_ns()
        with self._state_lock:
            intent = decode_operator_intent_datagram(
                payload,
                expected_identity=self._expected_identity,
                arrival_monotonic_ns=arrival_monotonic_ns,
            )
            self._accept_source_order(intent.source_epoch, intent.source_sequence)
            if isinstance(intent, OperatorMotionIntent):
                self._motion_inbox.put(intent)
            else:
                self._request_inbox.put(intent)
            return intent

    def start(self) -> None:
        """Start the validation/deposit loop in one owned daemon thread."""

        with self._state_lock:
            if self._closed:
                raise RuntimeError("operator intent receiver is closed")
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("operator intent receiver is already running")
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._receive_loop,
                name="lingtu-ue-control-intent",
                daemon=True,
            )
            self._thread.start()

    def reset(self, expected_identity: OperatorIntentIdentity) -> None:
        """Install a new generation identity and clear all unread/order state."""

        if not isinstance(expected_identity, OperatorIntentIdentity):
            raise TypeError("expected_identity must be OperatorIntentIdentity")
        with self._state_lock:
            self._expected_identity = expected_identity
            self._last_source_epoch = None
            self._last_source_sequence = None
            self._motion_inbox.clear()
            self._request_inbox.clear()

    def drain_validation_errors(self) -> tuple[ControlIntentValidationError, ...]:
        """Return and clear bounded background validation failures."""

        with self._errors_lock:
            errors = tuple(self._errors)
            self._errors.clear()
            return errors

    def close(self) -> None:
        """Stop the optional receive thread and release the UDP socket."""

        with self._state_lock:
            if self._closed:
                return
            self._closed = True
            self._stop_event.set()
            thread = self._thread
        self._socket.close()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)

    def __enter__(self) -> UdpLoopbackOperatorIntentReceiver:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _accept_source_order(self, epoch: int, sequence: int) -> None:
        previous_epoch = self._last_source_epoch
        previous_sequence = self._last_source_sequence
        if previous_epoch is None:
            self._last_source_epoch = epoch
            self._last_source_sequence = sequence
            return
        if epoch < previous_epoch:
            raise ControlIntentValidationError("source_epoch moved backward")
        if epoch == previous_epoch:
            if previous_sequence is None or sequence <= previous_sequence:
                raise ControlIntentValidationError(
                    "source_sequence must increase within one source_epoch"
                )
        self._last_source_epoch = epoch
        self._last_source_sequence = sequence

    def _receive_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.receive_once()
            except ControlIntentValidationError as exc:
                with self._errors_lock:
                    self._errors.append(exc)
            except OSError as exc:
                if not self._stop_event.is_set():
                    with self._errors_lock:
                        self._errors.append(
                            ControlIntentValidationError(
                                f"operator intent UDP receive failed: {exc}"
                            )
                        )
                return


def encode_control_ack(document: Mapping[str, Any]) -> bytes:
    """Encode one exact v1 ACK without claiming the complete status schema."""

    if not isinstance(document, Mapping):
        raise ControlIntentValidationError("control ACK must be an object")
    _exact_fields(document, _ACK_FIELDS, "control ACK")
    if document["schema"] != CONTROL_ACK_SCHEMA:
        raise ControlIntentValidationError(
            f"control ACK schema must be {CONTROL_ACK_SCHEMA}"
        )
    identity = OperatorIntentIdentity(
        run_id=document["run_id"],
        session_id=document["session_id"],
        boot_id=document["boot_id"],
        model_generation=document["model_generation"],
        reset_generation=document["reset_generation"],
        source_id=document["source_id"],
    )
    _positive_int(document["server_status_sequence"], "server_status_sequence")
    source_epoch = _positive_int(document["source_epoch"], "source_epoch")
    source_sequence = _positive_int(document["source_sequence"], "source_sequence")
    _event_id(document["event_id"], identity.boot_id, source_epoch, source_sequence)
    _sha256(document["intent_datagram_sha256"], "intent_datagram_sha256")
    status = _enum(document["status"], _ACK_STATUSES, "status")
    reason = _reason(document["reason"])
    if status not in {"accepted", "confirmed"} and not reason:
        raise ControlIntentValidationError(
            f"reason must be non-empty when status is {status}"
        )
    try:
        payload = json.dumps(
            dict(document),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ControlIntentValidationError(
            f"control ACK is not JSON-serializable: {exc}"
        ) from exc
    if len(payload) > MAX_CONTROL_DATAGRAM_BYTES:
        raise ControlIntentValidationError("control ACK exceeds 4096 bytes")
    return payload


class UdpLoopbackControlAckPublisher:
    """Non-blocking loopback publisher for exact ACK/status envelopes.

    The historical class name remains the compatibility surface used by the
    playable runner.  ``publish`` selects one exact encoder by ``schema``;
    the ACK v1 field set and validation are intentionally unchanged.
    """

    def __init__(
        self,
        port: int,
        *,
        host: str = "127.0.0.1",
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        if host != "127.0.0.1":
            raise ValueError("control ACK destination must be 127.0.0.1")
        _port(port, "control ACK port")
        self._destination = (host, port)
        transport = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        transport.setblocking(False)
        self._socket = transport

    def publish(self, document: Mapping[str, Any]) -> int:
        """Send one exact ACK/full status or report a non-blocking drop."""

        schema = document.get("schema") if isinstance(document, Mapping) else None
        if schema == CONTROL_ACK_SCHEMA:
            payload = encode_control_ack(document)
        else:
            # Local import avoids a module cycle: full status reuses the strict
            # allocation identity and validation error declared in this module.
            from .control_status import CONTROL_STATUS_SCHEMA, encode_control_status

            if schema != CONTROL_STATUS_SCHEMA:
                raise ControlIntentValidationError(
                    "control response schema is unsupported"
                )
            payload = encode_control_status(document)
        try:
            sent = self._socket.sendto(payload, self._destination)
        except BlockingIOError:
            return 0
        if sent != len(payload):
            raise ControlIntentValidationError(
                f"control ACK datagram was truncated: {sent} of {len(payload)} bytes"
            )
        return sent

    def close(self) -> None:
        """Release the UDP socket."""

        self._socket.close()

    def __enter__(self) -> UdpLoopbackControlAckPublisher:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


# The complete status publisher shares the same loopback socket and strict
# schema-switch implementation.  Keep the ACK name above for runner/API
# compatibility while giving new code the product-accurate name.
UdpLoopbackControlStatusPublisher = UdpLoopbackControlAckPublisher


def decode_operator_intent_datagram(
    payload: bytes,
    *,
    expected_identity: OperatorIntentIdentity,
    arrival_monotonic_ns: int,
) -> OperatorMotionIntent | OperatorRuntimeRequest:
    """Decode one exact, allocation-bound UE input datagram.

    Stateful source ordering belongs to the receiver because it spans both
    motion and runtime-request schemas.
    """

    if not isinstance(payload, bytes):
        raise ControlIntentValidationError("control datagram must be bytes")
    datagram_sha256 = hashlib.sha256(payload).hexdigest()
    if len(payload) > MAX_CONTROL_DATAGRAM_BYTES:
        raise ControlIntentValidationError(
            "control datagram exceeds 4096 bytes"
        )
    _bounded_nonnegative_int(arrival_monotonic_ns, "arrival_monotonic_ns")
    document = _decode_strict_json(payload)
    schema = document.get("schema")
    if schema == CONTROL_INTENT_SCHEMA:
        return _parse_motion_document(
            document,
            expected_identity=expected_identity,
            arrival_monotonic_ns=arrival_monotonic_ns,
            datagram_sha256=datagram_sha256,
        )
    if schema == RUNTIME_REQUEST_SCHEMA:
        return _parse_request_document(
            document,
            expected_identity=expected_identity,
            arrival_monotonic_ns=arrival_monotonic_ns,
            datagram_sha256=datagram_sha256,
        )
    raise ControlIntentValidationError("control datagram schema is unsupported")


def _parse_motion_document(
    document: Mapping[str, Any],
    *,
    expected_identity: OperatorIntentIdentity,
    arrival_monotonic_ns: int,
    datagram_sha256: str,
) -> OperatorMotionIntent:
    _exact_fields(document, _MOTION_FIELDS, "motion intent")
    identity = _parse_and_match_identity(document, expected_identity)
    source_epoch = _positive_int(document["source_epoch"], "source_epoch")
    source_sequence = _positive_int(document["source_sequence"], "source_sequence")
    event_id = _event_id(
        document["event_id"], identity.boot_id, source_epoch, source_sequence
    )
    input_mode = _enum(document["input_mode"], {"drive"}, "input_mode")
    ui_mode = _enum(document["ui_mode"], {"drive"}, "ui_mode")
    camera_mode = _enum(
        document["camera_mode"],
        {"unavailable", "follow", "inspection", "free"},
        "camera_mode",
    )
    input_device = _enum(
        document["input_device"], {"keyboard", "gamepad"}, "input_device"
    )
    viewport_focused = _boolean(document["viewport_focused"], "viewport_focused")
    deadman = _boolean(document["deadman"], "deadman")
    axes_value = document["axes"]
    if not isinstance(axes_value, Mapping):
        raise ControlIntentValidationError("axes must be an object")
    _exact_fields(axes_value, _AXES_FIELDS, "axes")
    axes = OperatorMotionAxes(
        forward=_normalized_axis(axes_value["forward"], "axes.forward"),
        left=_normalized_axis(axes_value["left"], "axes.left"),
        yaw_left=_normalized_axis(axes_value["yaw_left"], "axes.yaw_left"),
    )
    active_controls = _active_controls(document["active_controls"])
    source_monotonic_ns = _bounded_nonnegative_int(
        document["source_monotonic_ns"], "source_monotonic_ns"
    )
    return OperatorMotionIntent(
        identity=identity,
        source_epoch=source_epoch,
        source_sequence=source_sequence,
        event_id=event_id,
        input_mode=input_mode,
        input_device=input_device,
        viewport_focused=viewport_focused,
        deadman=deadman,
        axes=axes,
        active_controls=active_controls,
        source_monotonic_ns=source_monotonic_ns,
        arrival_monotonic_ns=arrival_monotonic_ns,
        datagram_sha256=datagram_sha256,
        ui_mode=ui_mode,
        camera_mode=camera_mode,
    )


def _parse_request_document(
    document: Mapping[str, Any],
    *,
    expected_identity: OperatorIntentIdentity,
    arrival_monotonic_ns: int,
    datagram_sha256: str,
) -> OperatorRuntimeRequest:
    _exact_fields(document, _REQUEST_FIELDS, "runtime request")
    ui_mode = _enum(
        document["ui_mode"],
        {"drive", "build", "tactical", "menu"},
        "ui_mode",
    )
    camera_mode = _enum(
        document["camera_mode"],
        {"unavailable", "follow", "inspection", "free"},
        "camera_mode",
    )
    identity = _parse_and_match_identity(document, expected_identity)
    source_epoch = _positive_int(document["source_epoch"], "source_epoch")
    source_sequence = _positive_int(document["source_sequence"], "source_sequence")
    event_id = _event_id(
        document["event_id"], identity.boot_id, source_epoch, source_sequence
    )
    request = _enum(document["request"], _RUNTIME_REQUESTS, "request")
    source_monotonic_ns = _bounded_nonnegative_int(
        document["source_monotonic_ns"], "source_monotonic_ns"
    )
    return OperatorRuntimeRequest(
        identity=identity,
        source_epoch=source_epoch,
        source_sequence=source_sequence,
        event_id=event_id,
        request=request,
        source_monotonic_ns=source_monotonic_ns,
        arrival_monotonic_ns=arrival_monotonic_ns,
        datagram_sha256=datagram_sha256,
        ui_mode=ui_mode,
        camera_mode=camera_mode,
    )


def _decode_strict_json(payload: bytes) -> Mapping[str, Any]:
    try:
        text = payload.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        raise ControlIntentValidationError("control datagram is not valid UTF-8") from exc

    def object_from_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ControlIntentValidationError(f"duplicate JSON key {key!r}")
            result[key] = value
        return result

    def reject_constant(value: str) -> None:
        raise ControlIntentValidationError(
            f"non-finite JSON value is not allowed: {value}"
        )

    try:
        value = json.loads(
            text,
            object_pairs_hook=object_from_pairs,
            parse_constant=reject_constant,
        )
    except ControlIntentValidationError:
        raise
    except json.JSONDecodeError as exc:
        raise ControlIntentValidationError(
            f"control datagram JSON is invalid: {exc.msg}"
        ) from exc
    if not isinstance(value, Mapping):
        raise ControlIntentValidationError("control datagram must be a JSON object")
    return value


def _parse_and_match_identity(
    document: Mapping[str, Any], expected: OperatorIntentIdentity
) -> OperatorIntentIdentity:
    actual = OperatorIntentIdentity(
        run_id=document["run_id"],
        session_id=document["session_id"],
        boot_id=document["boot_id"],
        model_generation=document["model_generation"],
        reset_generation=document["reset_generation"],
        source_id=document["source_id"],
    )
    for field_name in (
        "run_id",
        "session_id",
        "boot_id",
        "model_generation",
        "reset_generation",
        "source_id",
    ):
        if getattr(actual, field_name) != getattr(expected, field_name):
            raise ControlIntentValidationError(f"{field_name} does not match allocation")
    return actual


def _exact_fields(
    value: Mapping[str, Any], expected: frozenset[str], context: str
) -> None:
    unknown = sorted(set(value) - expected)
    if unknown:
        raise ControlIntentValidationError(
            f"{context} has unknown field(s): {', '.join(unknown)}"
        )
    missing = sorted(expected - set(value))
    if missing:
        raise ControlIntentValidationError(
            f"{context} is missing required field(s): {', '.join(missing)}"
        )


def _safe_id(value: Any, field_name: str) -> str:
    if not isinstance(value, str) or _SAFE_ID_RE.fullmatch(value) is None:
        raise ControlIntentValidationError(f"{field_name} is not a safe identifier")
    return value


def _sha256(value: Any, field_name: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise ControlIntentValidationError(
            f"{field_name} must be a lowercase SHA-256 digest"
        )
    return value


def _bounded_nonnegative_int(value: Any, field_name: str) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 <= value <= _MAX_INT64
    ):
        raise ControlIntentValidationError(
            f"{field_name} must be a non-negative signed 64-bit integer"
        )
    return value


def _positive_int(value: Any, field_name: str) -> int:
    result = _bounded_nonnegative_int(value, field_name)
    if result == 0:
        raise ControlIntentValidationError(f"{field_name} must be positive")
    return result


def _event_id(value: Any, boot_id: str, epoch: int, sequence: int) -> str:
    expected = f"{boot_id}:{epoch}:{sequence}"
    if value != expected:
        raise ControlIntentValidationError(
            "event_id must equal <boot_id>:<source_epoch>:<source_sequence>"
        )
    return expected


def _enum(value: Any, allowed: set[str], field_name: str) -> str:
    if not isinstance(value, str) or value not in allowed:
        raise ControlIntentValidationError(
            f"{field_name} must be one of: {', '.join(sorted(allowed))}"
        )
    return value


def _boolean(value: Any, field_name: str) -> bool:
    if not isinstance(value, bool):
        raise ControlIntentValidationError(f"{field_name} must be a boolean")
    return value


def _normalized_axis(value: Any, field_name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or not -1.0 <= float(value) <= 1.0
    ):
        raise ControlIntentValidationError(
            f"{field_name} must be finite and within [-1, 1]"
        )
    return float(value)


def _active_controls(value: Any) -> tuple[str, ...]:
    if not isinstance(value, list) or len(value) > 32:
        raise ControlIntentValidationError(
            "active_controls must be a list of at most 32 controls"
        )
    controls: list[str] = []
    for item in value:
        if not isinstance(item, str) or _SAFE_CONTROL_RE.fullmatch(item) is None:
            raise ControlIntentValidationError(
                "active_controls contains an unsafe control identifier"
            )
        if item in controls:
            raise ControlIntentValidationError(
                f"active_controls contains duplicate control {item!r}"
            )
        controls.append(item)
    return tuple(controls)


def _reason(value: Any) -> str:
    if not isinstance(value, str):
        raise ControlIntentValidationError("reason must be a string")
    if len(value.encode("utf-8")) > 512:
        raise ControlIntentValidationError("reason exceeds 512 UTF-8 bytes")
    if value and value != value.strip():
        raise ControlIntentValidationError("reason must be trimmed")
    if any(ord(character) < 32 or ord(character) == 127 for character in value):
        raise ControlIntentValidationError("reason contains a control character")
    return value


def _port(value: Any, field_name: str) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 1 <= value <= 65535
    ):
        raise ValueError(f"{field_name} must be an integer from 1 to 65535")
    return value


def _is_ipv4_loopback_source(source: object) -> bool:
    if (
        not isinstance(source, tuple)
        or len(source) < 2
        or not isinstance(source[0], str)
    ):
        return False
    try:
        address = ipaddress.ip_address(source[0])
    except ValueError:
        return False
    return address.version == 4 and address.is_loopback
