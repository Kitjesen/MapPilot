"""Small local client for the native ``mapd`` query socket.

DDS remains the real-time map data plane.  This client is only for bounded,
same-host request/reply operations and for receiving an already validated map
artifact file descriptor from ``mapd``.
"""

from __future__ import annotations

import array
import json
import os
import socket
import stat
import struct
from collections.abc import Callable, Iterator, Mapping
from dataclasses import dataclass, field
from typing import Any

DEFAULT_SOCKET_PATH = "/run/lingtu-mapd/mapd.sock"
SOCKET_PATH_ENV = "LINGTU_MAPD_QUERY_SOCKET"

_MAGIC_REQUEST = b"LTMP"
_MAGIC_RESPONSE = b"LTMR"
_VERSION = 1
_HAS_FD = 1
_MAX_MAP_ID_BYTES = 128
_MAX_CAPABILITY_BYTES = 64
_MAX_RESPONSE_BYTES = 1024 * 1024
_REQUEST_HEADER = struct.Struct("!4sBBHIHH")
_RESPONSE_HEADER = struct.Struct("!4sBBHI")
_SCM_RIGHTS = getattr(socket, "SCM_RIGHTS", 1)

_PING = 1
_OPEN_ARTIFACT = 5


class MapClientError(RuntimeError):
    """A stable failure returned by, or while contacting, native ``mapd``."""

    def __init__(
        self,
        reason_code: str,
        message: str,
        *,
        response: Mapping[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.reason_code = reason_code
        self.response = dict(response or {})


@dataclass
class ArtifactHandle:
    """One verified regular-file descriptor returned by ``mapd``."""

    metadata: dict[str, Any]
    size_bytes: int
    sha256: str = ""
    media_type: str = "application/octet-stream"
    filename: str = "map.pcd"
    _descriptor: int = field(default=-1, repr=False)

    def __enter__(self) -> ArtifactHandle:
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close()

    def __del__(self) -> None:
        self.close()

    def close(self) -> None:
        """Close the artifact unless ownership has moved to an iterator."""

        descriptor = self._descriptor
        self._descriptor = -1
        if descriptor >= 0:
            try:
                os.close(descriptor)
            except OSError:
                pass

    def iter_bytes(self, *, chunk_size: int = 1024 * 1024) -> Iterator[bytes]:
        """Transfer descriptor ownership to a bounded streaming iterator."""

        if chunk_size <= 0:
            raise ValueError("chunk_size must be positive")
        descriptor = self._descriptor
        if descriptor < 0:
            raise MapClientError("artifact_closed", "artifact file descriptor is closed")
        self._descriptor = -1

        def chunks() -> Iterator[bytes]:
            with os.fdopen(descriptor, "rb", closefd=True) as handle:
                while True:
                    block = handle.read(chunk_size)
                    if not block:
                        return
                    yield block

        return chunks()


class MapClient:
    """Stateless client for ``mapd``'s private Unix-domain query endpoint."""

    def __init__(
        self,
        socket_path: str | os.PathLike[str] | None = None,
        *,
        timeout_s: float = 1.0,
        connector: Callable[[], Any] | None = None,
    ) -> None:
        configured = socket_path or os.environ.get(SOCKET_PATH_ENV) or DEFAULT_SOCKET_PATH
        self.socket_path = os.fspath(configured)
        self.timeout_s = max(0.001, min(60.0, float(timeout_s)))
        self._connector = connector or self._new_socket

    @staticmethod
    def _new_socket():
        return socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

    def ping(self) -> dict[str, Any]:
        """Check that the native query endpoint is responding."""

        response, _ = self._request(_PING)
        return response

    def open_artifact(self, map_id: str, capability: str) -> ArtifactHandle:
        """Receive one validated, read-only map artifact descriptor."""

        response, descriptors = self._request(
            _OPEN_ARTIFACT,
            map_id,
            capability,
            expect_fd=True,
        )
        if len(descriptors) != 1:
            _close_descriptors(descriptors)
            raise MapClientError(
                "artifact_fd_missing",
                "mapd did not return exactly one artifact file descriptor",
                response=response,
            )
        descriptor = descriptors[0]
        try:
            os.set_inheritable(descriptor, False)
            file_stat = os.fstat(descriptor)
            if not stat.S_ISREG(file_stat.st_mode):
                raise MapClientError(
                    "artifact_not_regular",
                    "mapd artifact file descriptor is not a regular file",
                    response=response,
                )
        except Exception:
            try:
                os.close(descriptor)
            except OSError:
                pass
            raise

        artifact = response.get("artifact")
        artifact_metadata = dict(artifact) if isinstance(artifact, Mapping) else {}
        sha256 = str(artifact_metadata.get("sha256") or response.get("sha256") or "").strip()
        media_type = str(
            artifact_metadata.get("media_type")
            or artifact_metadata.get("mime")
            or "application/octet-stream"
        ).strip()
        filename = str(artifact_metadata.get("filename") or "map.pcd").strip() or "map.pcd"
        return ArtifactHandle(
            metadata=response,
            size_bytes=int(file_stat.st_size),
            sha256=sha256,
            media_type=media_type,
            filename=os.path.basename(filename),
            _descriptor=descriptor,
        )

    def _request(
        self,
        opcode: int,
        map_id: str = "",
        capability: str = "",
        *,
        expect_fd: bool = False,
    ) -> tuple[dict[str, Any], list[int]]:
        map_bytes = _encode_argument("map_id", map_id, _MAX_MAP_ID_BYTES)
        capability_bytes = _encode_argument(
            "capability",
            capability,
            _MAX_CAPABILITY_BYTES,
        )
        deadline_ms = max(1, min(60_000, int(self.timeout_s * 1000.0)))
        frame = _REQUEST_HEADER.pack(
            _MAGIC_REQUEST,
            _VERSION,
            opcode,
            0,
            deadline_ms,
            len(map_bytes),
            len(capability_bytes),
        ) + map_bytes + capability_bytes

        try:
            transport = self._connector()
            with transport:
                transport.settimeout(self.timeout_s)
                transport.connect(self.socket_path)
                transport.sendall(frame)
                if expect_fd:
                    raw_header, descriptors = _receive_header_with_descriptors(transport)
                else:
                    raw_header = _receive_exact(transport, _RESPONSE_HEADER.size)
                    descriptors = []
                response = self._decode_response(transport, raw_header, descriptors, expect_fd)
                return response, descriptors
        except MapClientError:
            raise
        except (OSError, TimeoutError) as exc:
            raise MapClientError(
                "mapd_unavailable",
                f"mapd query endpoint unavailable: {exc}",
            ) from exc

    @staticmethod
    def _decode_response(
        transport: Any,
        raw_header: bytes,
        descriptors: list[int],
        expect_fd: bool,
    ) -> dict[str, Any]:
        try:
            magic, version, status_code, flags, body_size = _RESPONSE_HEADER.unpack(raw_header)
            if magic != _MAGIC_RESPONSE or version != _VERSION:
                raise MapClientError("invalid_response", "invalid mapd response header")
            if flags & ~_HAS_FD:
                raise MapClientError("invalid_response", "unsupported mapd response flags")
            if body_size > _MAX_RESPONSE_BYTES:
                raise MapClientError("response_too_large", "mapd response exceeds the size limit")
            body = _receive_exact(transport, body_size)
            try:
                decoded = json.loads(body.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise MapClientError("invalid_response", "mapd returned invalid JSON") from exc
            if not isinstance(decoded, dict):
                raise MapClientError("invalid_response", "mapd response must be a JSON object")

            has_fd = bool(flags & _HAS_FD)
            if has_fd != bool(descriptors):
                raise MapClientError(
                    "invalid_response",
                    "mapd artifact file descriptor flag does not match the response",
                    response=decoded,
                )
            if has_fd and not expect_fd:
                raise MapClientError(
                    "unexpected_artifact_fd",
                    "mapd returned an unexpected artifact file descriptor",
                    response=decoded,
                )
            if status_code != 0 or decoded.get("success") is False:
                raise MapClientError(
                    str(decoded.get("reason_code") or "mapd_query_failed"),
                    str(decoded.get("message") or "mapd query failed"),
                    response=decoded,
                )
            return decoded
        except Exception:
            _close_descriptors(descriptors)
            descriptors.clear()
            raise


def _encode_argument(name: str, value: str, limit: int) -> bytes:
    if not isinstance(value, str):
        raise TypeError(f"{name} must be a string")
    encoded = value.encode("utf-8")
    if len(encoded) > limit:
        raise ValueError(f"{name} exceeds {limit} UTF-8 bytes")
    if any(byte == 0 or byte < 0x20 or byte == 0x7F for byte in encoded):
        raise ValueError(f"{name} contains control characters")
    return encoded


def _receive_exact(transport: Any, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        block = transport.recv(size - len(chunks))
        if not block:
            raise MapClientError("connection_closed", "mapd closed the connection early")
        chunks.extend(block)
    return bytes(chunks)


def _receive_header_with_descriptors(transport: Any) -> tuple[bytes, list[int]]:
    recvmsg = getattr(transport, "recvmsg", None)
    if not callable(recvmsg):
        raise MapClientError(
            "fd_passing_unsupported",
            "this platform cannot receive a map artifact file descriptor",
        )
    descriptor_bytes = array.array("i").itemsize
    ancillary_size = (
        socket.CMSG_SPACE(descriptor_bytes * 2)
        if hasattr(socket, "CMSG_SPACE")
        else descriptor_bytes * 8
    )
    header, ancillary, message_flags, _address = recvmsg(
        _RESPONSE_HEADER.size,
        ancillary_size,
    )
    descriptors: list[int] = []
    try:
        for level, kind, payload in ancillary:
            if level != socket.SOL_SOCKET or kind != _SCM_RIGHTS:
                continue
            usable_size = len(payload) - (len(payload) % descriptor_bytes)
            values = array.array("i")
            values.frombytes(payload[:usable_size])
            descriptors.extend(int(value) for value in values)
        if message_flags & getattr(socket, "MSG_CTRUNC", 0):
            raise MapClientError(
                "invalid_response",
                "mapd descriptor response was truncated",
            )
        if len(header) < _RESPONSE_HEADER.size:
            header += _receive_exact(transport, _RESPONSE_HEADER.size - len(header))
        if len(descriptors) > 1:
            raise MapClientError(
                "invalid_response",
                "mapd returned more than one artifact file descriptor",
            )
        return header, descriptors
    except Exception:
        _close_descriptors(descriptors)
        raise


def _close_descriptors(descriptors: list[int]) -> None:
    for descriptor in descriptors:
        try:
            os.close(descriptor)
        except OSError:
            pass


__all__ = [
    "DEFAULT_SOCKET_PATH",
    "SOCKET_PATH_ENV",
    "ArtifactHandle",
    "MapClient",
    "MapClientError",
]
