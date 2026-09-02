from __future__ import annotations

import ctypes
import json
import os
import shutil
import socket
import struct
import time
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import TimeoutError as FutureTimeoutError
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Callable, Iterator, cast

import pytest

from sim.scripts.mujoco.native_runtime_endpoint import (
    DRIVER_PROTOCOL,
    DRIVER_ROLE,
    LIDAR_ROLE,
    SENSOR_PROTOCOL,
    DriverBridgeClient,
    NativeRuntimeEndpointError,
    NativeRuntimeEndpointTimeout,
    SensorPublisherClient,
    load_local_endpoint,
)

_PRODUCT_SESSION_ID = "a" * 32
_SECRET = bytes.fromhex("b" * 64)
_NONCE = _SECRET.hex()
_CASE_TMP_ROOT = Path("pytest_native_runtime_endpoint_cases").resolve()


def _stored_secret(secret: bytes) -> bytes:
    if os.name != "nt":
        return secret
    from ctypes import wintypes

    class DataBlob(ctypes.Structure):
        _fields_ = [
            ("cbData", wintypes.DWORD),
            ("pbData", ctypes.POINTER(ctypes.c_ubyte)),
        ]

    source = (ctypes.c_ubyte * len(secret)).from_buffer_copy(secret)
    source_blob = DataBlob(len(secret), ctypes.cast(source, ctypes.POINTER(ctypes.c_ubyte)))
    protected_blob = DataBlob()
    crypt32 = ctypes.WinDLL("Crypt32.dll", use_last_error=True)
    kernel32 = ctypes.WinDLL("Kernel32.dll", use_last_error=True)
    crypt32.CryptProtectData.argtypes = [
        ctypes.POINTER(DataBlob),
        ctypes.c_wchar_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_void_p,
        wintypes.DWORD,
        ctypes.POINTER(DataBlob),
    ]
    crypt32.CryptProtectData.restype = wintypes.BOOL
    kernel32.LocalFree.argtypes = [ctypes.c_void_p]
    kernel32.LocalFree.restype = ctypes.c_void_p
    assert crypt32.CryptProtectData(
        ctypes.byref(source_blob),
        "LingTu local runtime secret",
        None,
        None,
        None,
        1,
        ctypes.byref(protected_blob),
    )
    try:
        return ctypes.string_at(protected_blob.pbData, protected_blob.cbData)
    finally:
        kernel32.LocalFree(ctypes.cast(protected_blob.pbData, ctypes.c_void_p))


def _write_readiness(
    path: Path,
    *,
    port: int = 41001,
    secret: bytes = _SECRET,
    role: str = LIDAR_ROLE,
    protocol: str = SENSOR_PROTOCOL,
    auth_file: str = "native.auth",
) -> None:
    auth_path = path.with_name(auth_file)
    auth_path.write_bytes(_stored_secret(secret))
    auth_path.chmod(0o600)
    path.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.local_endpoint.v1",
                "ready": True,
                "role": role,
                "protocol": protocol,
                "product_session_id": _PRODUCT_SESSION_ID,
                "host": "127.0.0.1",
                "port": port,
                "auth_file": auth_file,
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )


@pytest.fixture
def case_tmp_path(request: pytest.FixtureRequest) -> Iterator[Path]:
    path = (_CASE_TMP_ROOT / request.node.name).resolve()
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True)
    try:
        yield path
    finally:
        shutil.rmtree(path, ignore_errors=True)


def test_world_readable_readiness_contains_no_plaintext_secret(
    case_tmp_path: Path,
) -> None:
    readiness = (case_tmp_path / "sensor.ready.json").resolve()
    _write_readiness(readiness)
    readiness.chmod(0o644)

    endpoint = load_local_endpoint(
        readiness,
        role=LIDAR_ROLE,
        protocol=SENSOR_PROTOCOL,
        product_session_id=_PRODUCT_SESSION_ID,
    )

    assert endpoint.host == "127.0.0.1"
    assert endpoint.port == 41001
    assert endpoint.auth_file == readiness.with_name("native.auth")
    readiness_bytes = readiness.read_bytes()
    assert b"nonce" not in readiness_bytes
    assert _NONCE.encode("ascii") not in readiness_bytes
    stored = endpoint.auth_file.read_bytes()
    if os.name == "nt":
        assert stored != _SECRET
    else:
        assert stored == _SECRET


def _recv_exact(connection: socket.socket, size: int) -> bytes:
    chunks: list[bytes] = []
    remaining = size
    while remaining:
        chunk = connection.recv(remaining)
        if not chunk:
            raise AssertionError("client closed before the expected bytes arrived")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _recv_json_frame(connection: socket.socket) -> dict[str, object]:
    (size,) = struct.unpack(">I", _recv_exact(connection, 4))
    value = json.loads(_recv_exact(connection, size).decode("ascii"))
    if not isinstance(value, dict):
        raise AssertionError("expected JSON frame object")
    return cast(dict[str, object], value)


def _send_json_frame(connection: socket.socket, payload: dict[str, object]) -> None:
    encoded = json.dumps(
        payload,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("ascii")
    connection.sendall(struct.pack(">I", len(encoded)) + encoded)


@contextmanager
def _serve_once(handler: Callable[[socket.socket], None]) -> Iterator[int]:
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    listener.settimeout(2.0)
    port = listener.getsockname()[1]

    def run() -> None:
        connection, _ = listener.accept()
        with connection:
            connection.settimeout(2.0)
            handler(connection)

    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(run)
        try:
            yield port
        finally:
            try:
                future.result(timeout=2.0)
            except FutureTimeoutError:
                listener.close()
                try:
                    future.result(timeout=2.0)
                except OSError:
                    pass
            finally:
                listener.close()


@contextmanager
def _connected_driver_client() -> Iterator[tuple[DriverBridgeClient, socket.socket]]:
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    listener.settimeout(2.0)
    port = listener.getsockname()[1]

    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(listener.accept)
        client_socket = socket.create_connection(("127.0.0.1", port), timeout=2.0)
        peer_socket, _ = future.result(timeout=2.0)
        listener.close()
        client_socket.settimeout(1.0)
        peer_socket.settimeout(1.0)
        client = DriverBridgeClient(client_socket, 1.0)
        try:
            yield client, peer_socket
        finally:
            client.close()
            peer_socket.close()


def test_sensor_client_authenticates_then_forwards_binary_ltu1(
    case_tmp_path: Path,
) -> None:
    role = "lidar_publisher"
    payload = b"LTU1\x01\x00\x00\x00" + bytes(range(64))

    def handle(connection: socket.socket) -> None:
        assert _recv_json_frame(connection) == {
            "schema": "lingtu.sim.local_endpoint.connect.v1",
            "role": role,
            "protocol": SENSOR_PROTOCOL,
            "product_session_id": _PRODUCT_SESSION_ID,
            "nonce": _NONCE,
        }
        _send_json_frame(
            connection,
            {
                "schema": "lingtu.sim.local_endpoint.ack.v1",
                "ok": True,
                "role": role,
                "protocol": SENSOR_PROTOCOL,
                "product_session_id": _PRODUCT_SESSION_ID,
            },
        )
        assert _recv_exact(connection, len(payload)) == payload

    with _serve_once(handle) as port:
        readiness = (case_tmp_path / "lidar.ready.json").resolve()
        _write_readiness(readiness, port=port, role=role)
        readiness.chmod(0o644)
        client = SensorPublisherClient.connect(
            readiness,
            role=role,
            product_session_id=_PRODUCT_SESSION_ID,
            timeout_s=1.0,
        )
        try:
            assert client.write(payload) == len(payload)
            client.flush()
        finally:
            client.close()


def test_driver_client_authenticates_and_exchanges_ascii_lines(
    case_tmp_path: Path,
) -> None:
    def handle(connection: socket.socket) -> None:
        assert _recv_json_frame(connection) == {
            "schema": "lingtu.sim.local_endpoint.connect.v1",
            "role": DRIVER_ROLE,
            "protocol": DRIVER_PROTOCOL,
            "product_session_id": _PRODUCT_SESSION_ID,
            "nonce": _NONCE,
        }
        _send_json_frame(
            connection,
            {
                "schema": "lingtu.sim.local_endpoint.ack.v1",
                "ok": True,
                "role": DRIVER_ROLE,
                "protocol": DRIVER_PROTOCOL,
                "product_session_id": _PRODUCT_SESSION_ID,
            },
        )
        assert _recv_exact(connection, len(b"COMMAND 17\n")) == b"COMMAND 17\n"
        connection.sendall(b"APPLIED 17\n")

    with _serve_once(handle) as port:
        readiness = (case_tmp_path / "driver.ready.json").resolve()
        _write_readiness(
            readiness,
            port=port,
            role=DRIVER_ROLE,
            protocol=DRIVER_PROTOCOL,
        )
        readiness.chmod(0o644)
        client = DriverBridgeClient.connect(
            readiness,
            product_session_id=_PRODUCT_SESSION_ID,
            timeout_s=1.0,
        )
        try:
            client.send_line("COMMAND 17")
            assert client.recv_line() == "APPLIED 17"
        finally:
            client.close()


def test_driver_client_rejects_caller_selected_identity(case_tmp_path: Path) -> None:
    connect = cast(Any, DriverBridgeClient.connect)

    with pytest.raises(TypeError, match="unexpected keyword argument"):
        connect(
            case_tmp_path / "driver.ready.json",
            role=DRIVER_ROLE,
            protocol=DRIVER_PROTOCOL,
            product_session_id=_PRODUCT_SESSION_ID,
            timeout_s=1.0,
        )


def test_driver_client_rejects_legacy_line_v1_protocol(case_tmp_path: Path) -> None:
    readiness = (case_tmp_path / "legacy-driver.ready.json").resolve()
    _write_readiness(
        readiness,
        role=DRIVER_ROLE,
        protocol="line-v1",
    )
    readiness.chmod(0o644)

    with pytest.raises(NativeRuntimeEndpointError, match="role or protocol"):
        DriverBridgeClient.connect(
            readiness,
            product_session_id=_PRODUCT_SESSION_ID,
            timeout_s=1.0,
        )


def test_driver_client_uses_native_512_byte_line_limit() -> None:
    client_socket, peer_socket = socket.socketpair()
    client = DriverBridgeClient(client_socket, 1.0)
    try:
        client.send_line("x" * 512)
        assert _recv_exact(peer_socket, 513) == b"x" * 512 + b"\n"
        with pytest.raises(NativeRuntimeEndpointError, match="exceeds 512 bytes"):
            client.send_line("x" * 513)
    finally:
        client.close()
        peer_socket.close()


def test_driver_input_available_returns_true_for_buffered_data() -> None:
    with _connected_driver_client() as (client, peer_socket):
        peer_socket.sendall(b"FIRST\nSECOND\n")

        assert client.recv_line() == "FIRST"
        assert client.input_available() is True
        assert client.recv_line() == "SECOND"


def test_driver_input_available_returns_false_for_partial_buffered_line() -> None:
    with _connected_driver_client() as (client, peer_socket):
        peer_socket.sendall(b"FIRST\nPART")

        assert client.recv_line() == "FIRST"
        assert client.input_available(timeout_s=0.0) is False
        peer_socket.sendall(b"IAL\n")
        assert client.input_available(timeout_s=0.5) is True
        assert client.recv_line() == "PARTIAL"


def test_driver_input_available_buffers_partial_then_complete_line() -> None:
    with _connected_driver_client() as (client, peer_socket):
        peer_socket.sendall(b"PART")

        assert client.input_available(timeout_s=0.5) is False
        peer_socket.sendall(b"IAL\n")
        assert client.input_available(timeout_s=0.5) is True
        assert client.recv_line() == "PARTIAL"


def test_driver_input_available_fails_closed_after_partial_peer_close() -> None:
    with _connected_driver_client() as (client, peer_socket):
        peer_socket.sendall(b"PART")

        assert client.input_available(timeout_s=0.5) is False
        peer_socket.close()
        with pytest.raises(NativeRuntimeEndpointError, match="connection closed"):
            client.input_available(timeout_s=0.5)


def test_driver_input_available_fails_closed_when_line_exceeds_limit() -> None:
    with _connected_driver_client() as (client, peer_socket):
        peer_socket.sendall(b"x" * 513)

        with pytest.raises(NativeRuntimeEndpointError, match="exceeds 512 bytes"):
            client.input_available(timeout_s=0.5)


def test_driver_input_available_returns_false_without_data() -> None:
    with _connected_driver_client() as (client, _peer_socket):
        assert client.input_available(timeout_s=0.0) is False


def test_driver_input_available_returns_true_when_socket_becomes_readable() -> None:
    with _connected_driver_client() as (client, peer_socket):
        peer_socket.sendall(b"READY\n")

        assert client.input_available(timeout_s=0.5) is True
        assert client.recv_line() == "READY"


@pytest.mark.parametrize("timeout_s", [True, False, -0.1, float("nan"), float("inf"), "0"])
def test_driver_input_available_rejects_invalid_timeout(timeout_s: object) -> None:
    with _connected_driver_client() as (client, _peer_socket):
        with pytest.raises(NativeRuntimeEndpointError, match="finite non-negative"):
            client.input_available(timeout_s=timeout_s)  # type: ignore[arg-type]


def test_driver_input_available_fails_closed_when_client_is_closed() -> None:
    with _connected_driver_client() as (client, peer_socket):
        client.close()
        peer_socket.close()

        with pytest.raises(NativeRuntimeEndpointError, match="client is closed"):
            client.input_available()


def test_driver_input_available_fails_closed_when_peer_closes() -> None:
    with _connected_driver_client() as (client, peer_socket):
        peer_socket.close()

        with pytest.raises(NativeRuntimeEndpointError, match="connection closed"):
            client.input_available(timeout_s=0.5)


def test_client_fails_closed_when_auth_secret_is_not_accepted(
    case_tmp_path: Path,
) -> None:
    wrong_secret = bytes.fromhex("c" * 64)
    wrong_nonce = wrong_secret.hex()

    def reject_wrong_auth(connection: socket.socket) -> None:
        request = _recv_json_frame(connection)
        assert request["nonce"] == wrong_nonce
        assert request["nonce"] != _NONCE

    with _serve_once(reject_wrong_auth) as port:
        readiness = (case_tmp_path / "wrong-auth.ready.json").resolve()
        _write_readiness(readiness, port=port, secret=wrong_secret)
        readiness.chmod(0o644)
        with pytest.raises(NativeRuntimeEndpointError, match="connection closed"):
            SensorPublisherClient.connect(
                readiness,
                role=LIDAR_ROLE,
                product_session_id=_PRODUCT_SESSION_ID,
                timeout_s=1.0,
            )


def test_handshake_uses_one_absolute_deadline_across_dripped_bytes(
    case_tmp_path: Path,
) -> None:
    def drip_ack(connection: socket.socket) -> None:
        _recv_json_frame(connection)
        ack = {
            "schema": "lingtu.sim.local_endpoint.ack.v1",
            "ok": True,
            "role": LIDAR_ROLE,
            "protocol": SENSOR_PROTOCOL,
            "product_session_id": _PRODUCT_SESSION_ID,
        }
        encoded = json.dumps(
            ack,
            allow_nan=False,
            ensure_ascii=True,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("ascii")
        framed = struct.pack(">I", len(encoded)) + encoded
        try:
            for byte in framed:
                connection.sendall(bytes((byte,)))
                time.sleep(0.04)
        except OSError:
            pass

    with _serve_once(drip_ack) as port:
        readiness = (case_tmp_path / "slow.ready.json").resolve()
        _write_readiness(readiness, port=port)
        readiness.chmod(0o644)
        started = time.monotonic()
        with pytest.raises(NativeRuntimeEndpointTimeout):
            SensorPublisherClient.connect(
                readiness,
                role=LIDAR_ROLE,
                product_session_id=_PRODUCT_SESSION_ID,
                timeout_s=0.15,
            )
        assert time.monotonic() - started < 0.35


def test_sensor_client_rejects_legacy_ltu1_protocol(case_tmp_path: Path) -> None:
    readiness = (case_tmp_path / "legacy.ready.json").resolve()
    _write_readiness(readiness, protocol="ltu1")
    readiness.chmod(0o644)

    with pytest.raises(NativeRuntimeEndpointError, match="role or protocol"):
        SensorPublisherClient.connect(
            readiness,
            role=LIDAR_ROLE,
            product_session_id=_PRODUCT_SESSION_ID,
            timeout_s=1.0,
        )
