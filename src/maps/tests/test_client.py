from __future__ import annotations

import array
import json
import os
import socket
import struct
from pathlib import Path

import pytest

from maps.client import MapClient, MapClientError

_RESPONSE_HEADER = struct.Struct("!4sBBHI")
_REQUEST_HEADER = struct.Struct("!4sBBHIHH")
_SCM_RIGHTS = getattr(socket, "SCM_RIGHTS", 1)


class _ScriptedSocket:
    def __init__(
        self,
        payload: bytes,
        *,
        descriptors: tuple[int, ...] = (),
        message_flags: int = 0,
    ) -> None:
        self._payload = payload
        self._descriptors = descriptors
        self._message_flags = message_flags
        self._header_read = False
        self.sent = bytearray()
        self.connected_to = None
        self.timeout = None
        self.closed = False

    def settimeout(self, timeout: float) -> None:
        self.timeout = timeout

    def connect(self, path: str) -> None:
        self.connected_to = path

    def sendall(self, payload: bytes) -> None:
        self.sent.extend(payload)

    def recv(self, size: int) -> bytes:
        chunk = self._payload[:size]
        self._payload = self._payload[size:]
        return chunk

    def recvmsg(self, size: int, _ancillary_size: int):
        assert not self._header_read
        self._header_read = True
        header = self._payload[:size]
        self._payload = self._payload[size:]
        ancillary = []
        if self._descriptors:
            values = array.array("i", self._descriptors).tobytes()
            ancillary.append((socket.SOL_SOCKET, _SCM_RIGHTS, values))
        return header, ancillary, self._message_flags, None

    def close(self) -> None:
        self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close()


def _response(payload: dict, *, ok: bool = True, has_fd: bool = False) -> bytes:
    body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    return _RESPONSE_HEADER.pack(b"LTMR", 1, 0 if ok else 1, 1 if has_fd else 0, len(body)) + body


def test_ping_uses_bounded_versioned_wire_request() -> None:
    transport = _ScriptedSocket(_response({"success": True, "action": "ping"}))
    client = MapClient(
        socket_path="/run/lingtu/test-mapd.sock",
        timeout_s=0.25,
        connector=lambda: transport,
    )

    assert client.ping() == {"success": True, "action": "ping"}
    assert transport.connected_to == "/run/lingtu/test-mapd.sock"
    assert transport.timeout == 0.25
    assert transport.closed is True

    magic, version, opcode, flags, deadline_ms, map_len, capability_len = _REQUEST_HEADER.unpack(
        transport.sent[: _REQUEST_HEADER.size]
    )
    assert (magic, version, opcode, flags) == (b"LTMP", 1, 1, 0)
    assert 1 <= deadline_ms <= 250
    assert (map_len, capability_len) == (0, 0)
    assert len(transport.sent) == _REQUEST_HEADER.size


def test_open_artifact_returns_verified_regular_file_handle(tmp_path: Path) -> None:
    artifact_path = tmp_path / "map.pcd"
    artifact_path.write_bytes(b"pcd-body")
    descriptor = os.open(artifact_path, os.O_RDONLY)
    transport = _ScriptedSocket(
        _response(
            {
                "success": True,
                "map_id": "yard",
                "capability": "source_pointcloud",
                "artifact": {"sha256": "a" * 64},
            },
            has_fd=True,
        ),
        descriptors=(descriptor,),
    )
    client = MapClient(connector=lambda: transport)
    map_id = "yard-" + "a" * 80
    capability = "source_pointcloud"

    with client.open_artifact(map_id, capability) as artifact:
        assert artifact.size_bytes == len(b"pcd-body")
        assert artifact.sha256 == "a" * 64
        assert b"".join(artifact.iter_bytes(chunk_size=3)) == b"pcd-body"

    _, _, opcode, _, _, map_len, capability_len = _REQUEST_HEADER.unpack(
        transport.sent[: _REQUEST_HEADER.size]
    )
    assert opcode == 5
    assert map_len == len(map_id)
    assert capability_len == len(capability)
    assert bytes(transport.sent[_REQUEST_HEADER.size :]) == (map_id + capability).encode()
    assert transport.closed is True
    with pytest.raises(OSError):
        os.fstat(descriptor)


def test_error_reply_raises_stable_client_error() -> None:
    transport = _ScriptedSocket(
        _response(
            {
                "success": False,
                "reason_code": "map_not_found",
                "message": "map not found",
            },
            ok=False,
        )
    )
    client = MapClient(connector=lambda: transport)

    with pytest.raises(MapClientError) as raised:
        client.ping()

    assert raised.value.reason_code == "map_not_found"
    assert str(raised.value) == "map not found"


def test_reply_without_required_artifact_fd_is_rejected() -> None:
    transport = _ScriptedSocket(_response({"success": True}, has_fd=False))
    client = MapClient(connector=lambda: transport)

    with pytest.raises(MapClientError, match="file descriptor"):
        client.open_artifact("yard", "source_pointcloud")


def test_truncated_descriptor_reply_closes_delivered_descriptor(tmp_path: Path) -> None:
    artifact_path = tmp_path / "map.pcd"
    artifact_path.write_bytes(b"pcd-body")
    descriptor = os.open(artifact_path, os.O_RDONLY)
    transport = _ScriptedSocket(
        _response({"success": True}, has_fd=True),
        descriptors=(descriptor,),
        message_flags=getattr(socket, "MSG_CTRUNC", 8),
    )
    client = MapClient(connector=lambda: transport)

    with pytest.raises(MapClientError, match="truncated"):
        client.open_artifact("yard", "source_pointcloud")

    with pytest.raises(OSError):
        os.fstat(descriptor)
