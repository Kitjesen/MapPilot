"""Authenticated loopback clients for Product-owned MuJoCo native processes.

This module consumes a published readiness artifact and connects to an already
running local process.  It deliberately owns no process lifecycle or runtime
graph resolution.
"""

from __future__ import annotations

import json
import math
import os
import re
import select
import socket
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from lingtu.switch_contracts import is_product_session_id

READINESS_SCHEMA = "lingtu.sim.local_endpoint.v1"
DRIVER_ROLE = "driver_bridge"
DRIVER_PROTOCOL = "driver-v2"
LIDAR_ROLE = "lidar_publisher"
IMU_ROLE = "imu_publisher"
CAMERA_ROLE = "camera_publisher"
SENSOR_PROTOCOL = "ltu1-v1"
_READINESS_FIELDS = frozenset(
    {
        "schema",
        "ready",
        "role",
        "protocol",
        "product_session_id",
        "host",
        "port",
        "auth_file",
    }
)
_MAX_READINESS_BYTES = 4096
_MAX_HANDSHAKE_BYTES = 4096
_MAX_PROTECTED_AUTH_BYTES = 4096
_AUTH_KEY_BYTES = 32
_MAX_DRIVER_LINE_BYTES = 512
_SAFE_AUTH_BASENAME = re.compile(
    r"[A-Za-z0-9](?:[A-Za-z0-9_.-]{0,126}[A-Za-z0-9])?\Z"
)


class NativeRuntimeEndpointError(RuntimeError):
    """The local native endpoint contract could not be trusted or used."""


class NativeRuntimeEndpointTimeout(NativeRuntimeEndpointError, TimeoutError):
    """An endpoint operation did not finish before its absolute deadline."""


def _read_bounded(path: Path, limit: int, *, label: str) -> bytes:
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise NativeRuntimeEndpointError(f"{label} is unavailable") from exc
    if len(raw) > limit:
        raise NativeRuntimeEndpointError(f"{label} exceeds the byte limit")
    return raw


def _unprotect_windows_auth(raw: bytes) -> bytes:
    import ctypes
    from ctypes import wintypes

    class DataBlob(ctypes.Structure):
        _fields_ = [
            ("cbData", wintypes.DWORD),
            ("pbData", ctypes.POINTER(ctypes.c_ubyte)),
        ]

    source = (ctypes.c_ubyte * len(raw)).from_buffer_copy(raw)
    source_blob = DataBlob(len(raw), ctypes.cast(source, ctypes.POINTER(ctypes.c_ubyte)))
    plain_blob = DataBlob()
    crypt32 = ctypes.WinDLL("Crypt32.dll", use_last_error=True)
    kernel32 = ctypes.WinDLL("Kernel32.dll", use_last_error=True)
    crypt32.CryptUnprotectData.argtypes = [
        ctypes.POINTER(DataBlob),
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        wintypes.DWORD,
        ctypes.POINTER(DataBlob),
    ]
    crypt32.CryptUnprotectData.restype = wintypes.BOOL
    kernel32.LocalFree.argtypes = [ctypes.c_void_p]
    kernel32.LocalFree.restype = ctypes.c_void_p
    if not crypt32.CryptUnprotectData(
        ctypes.byref(source_blob),
        None,
        None,
        None,
        None,
        1,  # CRYPTPROTECT_UI_FORBIDDEN
        ctypes.byref(plain_blob),
    ):
        raise NativeRuntimeEndpointError("native endpoint auth file cannot be unprotected")
    try:
        return ctypes.string_at(plain_blob.pbData, plain_blob.cbData)
    finally:
        kernel32.LocalFree(ctypes.cast(plain_blob.pbData, ctypes.c_void_p))


def _load_auth_key(path: Path) -> bytes:
    limit = _MAX_PROTECTED_AUTH_BYTES if os.name == "nt" else _AUTH_KEY_BYTES
    stored = _read_bounded(path, limit, label="native endpoint auth file")
    authkey = _unprotect_windows_auth(stored) if os.name == "nt" else stored
    if len(authkey) != _AUTH_KEY_BYTES:
        raise NativeRuntimeEndpointError("native endpoint auth file is invalid")
    return authkey


@dataclass(frozen=True)
class LocalEndpoint:
    """Validated public connection metadata and its private auth-file path."""

    role: str
    protocol: str
    product_session_id: str
    host: str
    port: int
    auth_file: Path


def _reject_json_constant(value: str) -> None:
    raise NativeRuntimeEndpointError(f"readiness JSON constant is forbidden: {value}")


def _strict_json_object(raw: bytes) -> dict[str, Any]:
    try:
        text = raw.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        raise NativeRuntimeEndpointError("readiness file must be valid UTF-8") from exc

    def pairs_hook(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise NativeRuntimeEndpointError(
                    f"readiness JSON contains duplicate key: {key}"
                )
            result[key] = value
        return result

    try:
        value = json.loads(
            text,
            object_pairs_hook=pairs_hook,
            parse_constant=_reject_json_constant,
        )
    except NativeRuntimeEndpointError:
        raise
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise NativeRuntimeEndpointError("readiness file must contain valid JSON") from exc
    if not isinstance(value, dict):
        raise NativeRuntimeEndpointError("readiness JSON must be an object")
    return value


def _auth_path(readiness_path: Path, value: Any) -> Path:
    if not isinstance(value, str) or _SAFE_AUTH_BASENAME.fullmatch(value) is None:
        raise NativeRuntimeEndpointError("auth_file must be a safe basename")
    auth_path = readiness_path.with_name(value)
    if auth_path.parent != readiness_path.parent:
        raise NativeRuntimeEndpointError("auth_file must be beside the readiness file")
    return auth_path


def load_local_endpoint(
    path: str | os.PathLike[str],
    *,
    role: str,
    protocol: str,
    product_session_id: str,
) -> LocalEndpoint:
    """Load and authenticate the exact local readiness declaration."""

    if not isinstance(role, str) or not role or role.strip() != role:
        raise NativeRuntimeEndpointError("expected role must be a non-empty string")
    if not isinstance(protocol, str) or not protocol or protocol.strip() != protocol:
        raise NativeRuntimeEndpointError("expected protocol must be a non-empty string")
    if not is_product_session_id(product_session_id):
        raise NativeRuntimeEndpointError("expected product_session_id is invalid")

    readiness_path = Path(path)
    raw = _read_bounded(readiness_path, _MAX_READINESS_BYTES, label="readiness file")
    payload = _strict_json_object(raw)
    if frozenset(payload) != _READINESS_FIELDS:
        raise NativeRuntimeEndpointError("readiness JSON fields do not match the schema")
    if payload["schema"] != READINESS_SCHEMA or payload["ready"] is not True:
        raise NativeRuntimeEndpointError("native endpoint is not ready")
    if payload["role"] != role or payload["protocol"] != protocol:
        raise NativeRuntimeEndpointError("readiness role or protocol does not match")
    if payload["product_session_id"] != product_session_id:
        raise NativeRuntimeEndpointError("readiness product_session_id does not match")
    if payload["host"] != "127.0.0.1":
        raise NativeRuntimeEndpointError("native endpoint host must be 127.0.0.1")
    port = payload["port"]
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise NativeRuntimeEndpointError("native endpoint port is invalid")
    return LocalEndpoint(
        role=role,
        protocol=protocol,
        product_session_id=product_session_id,
        host="127.0.0.1",
        port=port,
        auth_file=_auth_path(readiness_path, payload["auth_file"]),
    )


def _timeout_deadline(timeout_s: float) -> tuple[float, float]:
    if (
        isinstance(timeout_s, bool)
        or not isinstance(timeout_s, (int, float))
        or not math.isfinite(float(timeout_s))
        or timeout_s <= 0
    ):
        raise NativeRuntimeEndpointError("timeout_s must be a finite positive number")
    normalized = float(timeout_s)
    return time.monotonic() + normalized, normalized


def _remaining(deadline: float) -> float:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise NativeRuntimeEndpointTimeout("native endpoint operation timed out")
    return remaining


def _input_poll_deadline(timeout_s: float) -> tuple[float, float]:
    if (
        isinstance(timeout_s, bool)
        or not isinstance(timeout_s, (int, float))
        or not math.isfinite(float(timeout_s))
        or timeout_s < 0
    ):
        raise NativeRuntimeEndpointError(
            "timeout_s must be a finite non-negative number"
        )
    normalized = float(timeout_s)
    return time.monotonic() + normalized, normalized


def _send_all(connection: socket.socket, payload: bytes, deadline: float) -> None:
    view = memoryview(payload)
    while view:
        connection.settimeout(_remaining(deadline))
        try:
            sent = connection.send(view)
        except TimeoutError as exc:
            raise NativeRuntimeEndpointTimeout(
                "native endpoint operation timed out"
            ) from exc
        except OSError as exc:
            raise NativeRuntimeEndpointError("native endpoint connection failed") from exc
        if sent <= 0:
            raise NativeRuntimeEndpointError("native endpoint connection closed")
        view = view[sent:]


def _recv_exact(connection: socket.socket, size: int, deadline: float) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining:
        connection.settimeout(_remaining(deadline))
        try:
            chunk = connection.recv(remaining)
        except TimeoutError as exc:
            raise NativeRuntimeEndpointTimeout(
                "native endpoint operation timed out"
            ) from exc
        except OSError as exc:
            raise NativeRuntimeEndpointError("native endpoint connection failed") from exc
        if not chunk:
            raise NativeRuntimeEndpointError("native endpoint connection closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _canonical_json(payload: dict[str, Any]) -> bytes:
    return json.dumps(
        payload,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("ascii")


def _send_frame(connection: socket.socket, payload: bytes, deadline: float) -> None:
    if not payload or len(payload) > _MAX_HANDSHAKE_BYTES:
        raise NativeRuntimeEndpointError("native endpoint frame size is invalid")
    _send_all(connection, struct.pack(">I", len(payload)) + payload, deadline)


def _recv_frame(connection: socket.socket, deadline: float) -> bytes:
    (size,) = struct.unpack(">I", _recv_exact(connection, 4, deadline))
    if not 0 < size <= _MAX_HANDSHAKE_BYTES:
        raise NativeRuntimeEndpointError("native endpoint frame size is invalid")
    return _recv_exact(connection, size, deadline)


def _connect_authenticated(
    readiness_path: str | os.PathLike[str],
    *,
    role: str,
    protocol: str,
    product_session_id: str,
    timeout_s: float,
) -> tuple[socket.socket, float]:
    deadline, operation_timeout_s = _timeout_deadline(timeout_s)
    endpoint = load_local_endpoint(
        readiness_path,
        role=role,
        protocol=protocol,
        product_session_id=product_session_id,
    )
    authkey = _load_auth_key(endpoint.auth_file)
    nonce = authkey.hex()
    connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        connection.settimeout(_remaining(deadline))
        connection.connect((endpoint.host, endpoint.port))
        request = {
            "schema": "lingtu.sim.local_endpoint.connect.v1",
            "role": role,
            "protocol": protocol,
            "product_session_id": product_session_id,
            "nonce": nonce,
        }
        _send_frame(connection, _canonical_json(request), deadline)
        expected_ack = {
            "schema": "lingtu.sim.local_endpoint.ack.v1",
            "ok": True,
            "role": role,
            "protocol": protocol,
            "product_session_id": product_session_id,
        }
        if _recv_frame(connection, deadline) != _canonical_json(expected_ack):
            raise NativeRuntimeEndpointError("native endpoint authentication failed")
        return connection, operation_timeout_s
    except NativeRuntimeEndpointError:
        connection.close()
        raise
    except TimeoutError as exc:
        connection.close()
        raise NativeRuntimeEndpointTimeout(
            "native endpoint operation timed out"
        ) from exc
    except OSError as exc:
        connection.close()
        raise NativeRuntimeEndpointError("native endpoint connection failed") from exc


class SensorPublisherClient:
    """Authenticated byte stream for native LTU1 sensor records."""

    def __init__(self, connection: socket.socket, operation_timeout_s: float) -> None:
        self._connection = connection
        self._operation_timeout_s = operation_timeout_s
        self._closed = False

    @classmethod
    def connect(
        cls,
        readiness_path: str | os.PathLike[str],
        *,
        role: str,
        product_session_id: str,
        timeout_s: float,
    ) -> SensorPublisherClient:
        """Authenticate to an existing Product-owned sensor endpoint."""

        connection, operation_timeout_s = _connect_authenticated(
            readiness_path,
            role=role,
            protocol=SENSOR_PROTOCOL,
            product_session_id=product_session_id,
            timeout_s=timeout_s,
        )
        return cls(connection, operation_timeout_s)

    def write(self, payload: bytes) -> int:
        """Write one or more already-framed native sensor records."""

        if self._closed:
            raise NativeRuntimeEndpointError("native endpoint client is closed")
        if not isinstance(payload, bytes):
            raise TypeError("sensor payload must be bytes")
        if payload:
            deadline = time.monotonic() + self._operation_timeout_s
            _send_all(self._connection, payload, deadline)
        return len(payload)

    def flush(self) -> None:
        """Match the binary writer interface; socket writes are unbuffered."""

        return None

    def close(self) -> None:
        """Close the local stream idempotently."""

        if not self._closed:
            self._closed = True
            self._connection.close()


class DriverBridgeClient:
    """Authenticated, bounded ASCII line stream for the native driver bridge."""

    def __init__(self, connection: socket.socket, operation_timeout_s: float) -> None:
        self._connection = connection
        self._operation_timeout_s = operation_timeout_s
        self._buffer = bytearray()
        self._closed = False

    @classmethod
    def connect(
        cls,
        readiness_path: str | os.PathLike[str],
        *,
        product_session_id: str,
        timeout_s: float,
    ) -> DriverBridgeClient:
        """Authenticate to an existing Product-owned driver endpoint."""

        connection, operation_timeout_s = _connect_authenticated(
            readiness_path,
            role=DRIVER_ROLE,
            protocol=DRIVER_PROTOCOL,
            product_session_id=product_session_id,
            timeout_s=timeout_s,
        )
        return cls(connection, operation_timeout_s)

    def send_line(self, line: str) -> None:
        """Send one bounded ASCII line without allowing embedded delimiters."""

        if self._closed:
            raise NativeRuntimeEndpointError("native endpoint client is closed")
        if not isinstance(line, str):
            raise TypeError("driver bridge line must be a string")
        if "\n" in line or "\r" in line:
            raise NativeRuntimeEndpointError("driver bridge line must be a single line")
        try:
            encoded = line.encode("ascii", errors="strict")
        except UnicodeEncodeError as exc:
            raise NativeRuntimeEndpointError("driver bridge line must be ASCII") from exc
        if len(encoded) > _MAX_DRIVER_LINE_BYTES:
            raise NativeRuntimeEndpointError("driver bridge line exceeds 512 bytes")
        deadline = time.monotonic() + self._operation_timeout_s
        _send_all(self._connection, encoded + b"\n", deadline)

    def recv_line(self) -> str:
        """Receive one bounded ASCII line before one absolute deadline."""

        if self._closed:
            raise NativeRuntimeEndpointError("native endpoint client is closed")
        deadline = time.monotonic() + self._operation_timeout_s
        while True:
            newline = self._buffer.find(b"\n")
            if newline >= 0:
                if newline > _MAX_DRIVER_LINE_BYTES:
                    raise NativeRuntimeEndpointError(
                        "driver bridge line exceeds 512 bytes"
                    )
                encoded = bytes(self._buffer[:newline])
                del self._buffer[: newline + 1]
                try:
                    return encoded.decode("ascii", errors="strict")
                except UnicodeDecodeError as exc:
                    raise NativeRuntimeEndpointError(
                        "driver bridge line must be ASCII"
                    ) from exc
            if len(self._buffer) > _MAX_DRIVER_LINE_BYTES:
                raise NativeRuntimeEndpointError("driver bridge line exceeds 512 bytes")
            self._connection.settimeout(_remaining(deadline))
            try:
                chunk = self._connection.recv(
                    min(1024, _MAX_DRIVER_LINE_BYTES + 1 - len(self._buffer))
                )
            except TimeoutError as exc:
                raise NativeRuntimeEndpointTimeout(
                    "native endpoint operation timed out"
                ) from exc
            except OSError as exc:
                raise NativeRuntimeEndpointError(
                    "native endpoint connection failed"
                ) from exc
            if not chunk:
                raise NativeRuntimeEndpointError("native endpoint connection closed")
            self._buffer.extend(chunk)

    def input_available(self, timeout_s: float = 0.0) -> bool:
        """Return whether a complete line is buffered or arrives by the deadline."""

        if self._closed:
            raise NativeRuntimeEndpointError("native endpoint client is closed")
        deadline, _ = _input_poll_deadline(timeout_s)
        while True:
            newline = self._buffer.find(b"\n")
            if newline >= 0:
                if newline > _MAX_DRIVER_LINE_BYTES:
                    raise NativeRuntimeEndpointError(
                        "driver bridge line exceeds 512 bytes"
                    )
                return True
            if len(self._buffer) > _MAX_DRIVER_LINE_BYTES:
                raise NativeRuntimeEndpointError("driver bridge line exceeds 512 bytes")
            wait_s = max(0.0, deadline - time.monotonic())
            try:
                readable, _, _ = select.select([self._connection], [], [], wait_s)
            except OSError as exc:
                raise NativeRuntimeEndpointError(
                    "native endpoint connection failed"
                ) from exc
            if not readable:
                return False
            previous_timeout = self._connection.gettimeout()
            self._connection.settimeout(0.0)
            try:
                chunk = self._connection.recv(
                    min(1024, _MAX_DRIVER_LINE_BYTES + 1 - len(self._buffer))
                )
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    return False
                continue
            except OSError as exc:
                raise NativeRuntimeEndpointError(
                    "native endpoint connection failed"
                ) from exc
            finally:
                self._connection.settimeout(previous_timeout)
            if not chunk:
                raise NativeRuntimeEndpointError("native endpoint connection closed")
            self._buffer.extend(chunk)

    def close(self) -> None:
        """Close the local stream idempotently."""

        if not self._closed:
            self._closed = True
            self._connection.close()
