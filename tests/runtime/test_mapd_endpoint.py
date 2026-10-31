from __future__ import annotations

import array
import json
import os
import socket
import struct
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pytest

from runtime.endpoints.mapd import ArtifactHandle, MapClient, MapClientError

_RESPONSE_HEADER = struct.Struct("!4sBBHII")
_REQUEST_HEADER = struct.Struct("!4sBBHIHI")
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
    return _RESPONSE_HEADER.pack(b"LTMR", 2, 0 if ok else 1, 1 if has_fd else 0, len(body), 0) + body


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
    magic, version, opcode, flags, deadline_ms, request_id_len, json_len = _REQUEST_HEADER.unpack(
        transport.sent[: _REQUEST_HEADER.size]
    )
    assert (magic, version, opcode, flags) == (b"LTMP", 2, 1, 0)
    assert 1 <= deadline_ms <= 250
    assert request_id_len > 0
    assert json.loads(bytes(transport.sent[-json_len:])) == {}


def test_default_socket_follows_the_product_session(monkeypatch, tmp_path) -> None:
    monkeypatch.delenv("LINGTU_MAPD_QUERY_SOCKET", raising=False)
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))

    client = MapClient(connector=lambda: None)

    assert client.socket_path == str(tmp_path / "mapd.sock")

@pytest.mark.skipif(os.name != "nt", reason="requires Windows Winsock AF_UNIX")
def test_windows_client_uses_native_unix_socket_for_map_record(monkeypatch) -> None:
    import ctypes
    import select

    from runtime.endpoints.mapd import _receive_exact

    # Exercise the address bridge used by Windows Python builds without AF_UNIX.
    monkeypatch.delattr(socket, "AF_UNIX", raising=False)
    winsock = ctypes.WinDLL("ws2_32.dll")
    winsock.bind.argtypes = [ctypes.c_size_t, ctypes.c_void_p, ctypes.c_int]
    winsock.bind.restype = ctypes.c_int
    winsock.accept.argtypes = [ctypes.c_size_t, ctypes.c_void_p, ctypes.c_void_p]
    winsock.accept.restype = ctypes.c_size_t
    with tempfile.TemporaryDirectory(prefix="lt-mapd-") as directory:
        socket_path = os.path.join(directory, "mapd.sock")
        native_address = ctypes.create_string_buffer(struct.pack("H108s", 1, os.fsencode(socket_path)))
        with socket.socket(1, socket.SOCK_STREAM) as listener:
            listener.settimeout(2.0)
            assert winsock.bind(listener.fileno(), native_address, 110) == 0
            listener.listen(1)

            def reply() -> dict:
                assert select.select([listener], [], [], 2.0)[0]
                descriptor = winsock.accept(listener.fileno(), None, None)
                assert descriptor != ctypes.c_size_t(-1).value
                with socket.socket(1, socket.SOCK_STREAM, fileno=descriptor) as connection:
                    connection.settimeout(2.0)
                    header = _receive_exact(connection, _REQUEST_HEADER.size)
                    magic, version, opcode, flags, _, request_size, json_size = _REQUEST_HEADER.unpack(header)
                    assert (magic, version, opcode, flags) == (b"LTMP", 2, 2, 0)
                    _receive_exact(connection, request_size)
                    body = json.loads(_receive_exact(connection, json_size))
                    connection.sendall(_response({"success": True, "record": {"content_epoch": 7}}))
                    return body

            with ThreadPoolExecutor(max_workers=1) as executor:
                pending = executor.submit(reply)
                try:
                    result = MapClient(socket_path=socket_path).service("get_record", map_id="yard")
                finally:
                    body = pending.result(timeout=2.0)
                assert result == {"success": True, "record": {"content_epoch": 7}}
                assert body == {"action": "get_record", "map_id": "yard"}
        os.unlink(socket_path)


def test_saved_map_preview_accepts_the_web_80000_point_budget() -> None:
    payload = {"success": True, "map_id": "yard", "points": [[12.34567, -9.876543, 0.456789]] * 80000}
    transport = _ScriptedSocket(_response(payload))
    result = MapClient(connector=lambda: transport).service("get_map_points", map_id="yard", max_points=80000)
    assert len(result["points"]) == 80000


def test_response_above_preview_budget_is_rejected_before_reading_body() -> None:
    transport = _ScriptedSocket(_RESPONSE_HEADER.pack(b"LTMR", 2, 0, 0, 4 * 1024 * 1024 + 1, 0))
    with pytest.raises(MapClientError) as error:
        MapClient(connector=lambda: transport).ping()
    assert error.value.reason_code == "response_too_large"
    assert transport.closed


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
                "artifact": {"filename": "map.pcd"},
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
        assert not hasattr(artifact, "sha256")
        assert b"".join(artifact.iter_bytes(chunk_size=3)) == b"pcd-body"

    _, _, opcode, _, _, request_id_len, json_len = _REQUEST_HEADER.unpack(transport.sent[: _REQUEST_HEADER.size])
    assert opcode == 5
    body_offset = _REQUEST_HEADER.size + request_id_len
    assert json.loads(bytes(transport.sent[body_offset : body_offset + json_len])) == {
        "map_id": map_id,
        "capability": capability,
    }
    assert transport.closed is True
    with pytest.raises(OSError):
        os.fstat(descriptor)


@pytest.mark.parametrize("start_reading", [False, True])
def test_abandoned_artifact_iterator_closes_descriptor(tmp_path: Path, start_reading: bool) -> None:
    artifact_path = tmp_path / "map.pcd"
    artifact_path.write_bytes(b"pcd-body")
    descriptor = os.open(artifact_path, os.O_RDONLY)
    artifact = ArtifactHandle(metadata={}, size_bytes=8, _descriptor=descriptor)
    chunks = artifact.iter_bytes(chunk_size=3)
    del artifact
    try:
        if start_reading:
            assert next(chunks) == b"pcd"
        chunks.close()
        with pytest.raises(OSError):
            os.fstat(descriptor)
    finally:
        # Release the descriptor even when reproducing a leak in the old code.
        try:
            os.close(descriptor)
        except OSError:
            pass


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


def test_service_sends_flat_json_and_returns_domain_failure() -> None:
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

    response = client.service("get_record", map_id="yard", max_points=10)

    assert response["success"] is False
    _, version, opcode, _, _, request_id_len, json_len = _REQUEST_HEADER.unpack(transport.sent[: _REQUEST_HEADER.size])
    assert (version, opcode) == (2, 2)
    body_offset = _REQUEST_HEADER.size + request_id_len
    assert json.loads(bytes(transport.sent[body_offset : body_offset + json_len])) == {
        "action": "get_record",
        "map_id": "yard",
        "max_points": 10,
    }


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
