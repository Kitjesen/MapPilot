"""Exercise SDK credentials before each gRPC cardinality and the TLS boundary."""

import shutil
import socket
import subprocess
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from types import SimpleNamespace

import grpc
import pytest

from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import system_pb2_grpc as system_rpc
from gateway.rpc.security import APIKeyInterceptor
from gateway.rpc.server import start


@pytest.fixture(autouse=True)
def security_environment(monkeypatch):
    for name in (
        "LINGTU_GRPC_API_KEY", "LINGTU_GRPC_REQUIRE_API_KEY",
        "LINGTU_GRPC_TLS_CERT_FILE", "LINGTU_GRPC_TLS_KEY_FILE",
    ):
        monkeypatch.delenv(name, raising=False)


def address():
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        return f"127.0.0.1:{listener.getsockname()[1]}"


@contextmanager
def listener():
    target = address()
    server = start(SimpleNamespace(), target)
    try:
        yield target
    finally:
        server.stop(0).wait(3)


@contextmanager
def echo_server():
    called = []

    def unary(request, context):
        called.append("unary")
        return request

    def server_stream(request, context):
        called.append("server_stream")
        yield request

    def client_stream(requests, context):
        called.append("client_stream")
        return b"".join(requests)

    def bidi(requests, context):
        called.append("bidi")
        yield from requests

    server = grpc.server(ThreadPoolExecutor(max_workers=4), interceptors=(APIKeyInterceptor("test-sdk-key"),))
    server.add_generic_rpc_handlers((grpc.method_handlers_generic_handler("security.Test", {
        "Unary": grpc.unary_unary_rpc_method_handler(unary),
        "ServerStream": grpc.unary_stream_rpc_method_handler(server_stream),
        "ClientStream": grpc.stream_unary_rpc_method_handler(client_stream),
        "Bidi": grpc.stream_stream_rpc_method_handler(bidi),
    }),))
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    try:
        with grpc.insecure_channel(f"127.0.0.1:{port}") as channel:
            yield channel, called
    finally:
        server.stop(0).wait(3)


def invoke(channel, kind, metadata):
    method, request, streaming = {
        "unary": (channel.unary_unary, b"sample", False),
        "server_stream": (channel.unary_stream, b"sample", True),
        "client_stream": (channel.stream_unary, iter([b"sample"]), False),
        "bidi": (channel.stream_stream, iter([b"sample"]), True),
    }[kind]
    name = {"unary": "Unary", "server_stream": "ServerStream", "client_stream": "ClientStream", "bidi": "Bidi"}[kind]
    response = method(f"/security.Test/{name}")(request, metadata=metadata, timeout=3)
    return list(response) if streaming else response


@pytest.mark.parametrize("kind", ["unary", "server_stream", "client_stream", "bidi"])
@pytest.mark.parametrize("metadata", [(), (("x-api-key", "wrong-key"),), (("x-api-key", "test-sdk-key"), ("x-api-key", "test-sdk-key"))])
def test_invalid_key_never_enters_handler(kind, metadata):
    with echo_server() as (channel, called):
        with pytest.raises(grpc.RpcError) as error:
            invoke(channel, kind, metadata)
        assert error.value.code() == grpc.StatusCode.UNAUTHENTICATED
        assert "test-sdk-key" not in error.value.details()
        assert "wrong-key" not in error.value.details()
        assert called == []


@pytest.mark.parametrize("kind", ["unary", "server_stream", "client_stream", "bidi"])
def test_valid_key_preserves_cardinality_and_payload(kind):
    with echo_server() as (channel, called):
        response = invoke(channel, kind, (("x-api-key", "test-sdk-key"),))
        assert response == ([b"sample"] if kind in {"server_stream", "bidi"} else b"sample")
        assert called == [kind]


def test_listener_enforces_configured_key(monkeypatch):
    monkeypatch.setenv("LINGTU_GRPC_API_KEY", "configured-sdk-key")
    with listener() as target, grpc.insecure_channel(target) as channel:
        ping = system_rpc.SystemStub(channel).Ping
        with pytest.raises(grpc.RpcError) as error:
            ping(common_pb.Empty(), timeout=3)
        assert error.value.code() == grpc.StatusCode.UNAUTHENTICATED
        assert ping(common_pb.Empty(), metadata=(("x-api-key", "configured-sdk-key"),), timeout=3).status == "ok"


def test_default_listener_keeps_development_access():
    with listener() as target, grpc.insecure_channel(target) as channel:
        assert system_rpc.SystemStub(channel).Ping(common_pb.Empty(), timeout=3).status == "ok"


@pytest.mark.parametrize("settings", [
    {"LINGTU_GRPC_REQUIRE_API_KEY": "1"},
    {"LINGTU_GRPC_API_KEY": "\nsecret-value"},
    {"LINGTU_GRPC_TLS_CERT_FILE": "missing-certificate.pem"},
    {"LINGTU_GRPC_TLS_KEY_FILE": "missing-private-key.pem"},
    {"LINGTU_GRPC_TLS_CERT_FILE": "missing.pem", "LINGTU_GRPC_TLS_KEY_FILE": "missing.key"},
])
def test_invalid_security_configuration_fails_before_listening(monkeypatch, settings):
    for name, value in settings.items():
        monkeypatch.setenv(name, value)
    with pytest.raises(ValueError) as error:
        start(SimpleNamespace(), address())
    assert "secret-value" not in str(error.value)


@pytest.mark.parametrize("contents", [b"", b"invalid-pem"])
def test_invalid_tls_files_fail_without_insecure_fallback(monkeypatch, tmp_path, contents):
    for name, filename in (("CERT", "server.pem"), ("KEY", "server.key")):
        path = tmp_path / filename
        path.write_bytes(contents)
        monkeypatch.setenv(f"LINGTU_GRPC_TLS_{name}_FILE", str(path))
    with pytest.raises((ValueError, RuntimeError)):
        start(SimpleNamespace(), address())


@pytest.fixture
def certificates(tmp_path):
    openssl = shutil.which("openssl")
    if openssl is None:
        pytest.skip("TLS fixture generation requires the installed openssl tool")
    config = tmp_path / "openssl.cnf"
    config.write_text("[req]\ndistinguished_name=dn\n[dn]\n", encoding="ascii")
    extensions = tmp_path / "server.ext"
    extensions.write_text(
        "basicConstraints=critical,CA:FALSE\nkeyUsage=critical,digitalSignature,keyEncipherment\n"
        "extendedKeyUsage=serverAuth\nsubjectAltName=DNS:localhost,IP:127.0.0.1\n",
        encoding="ascii",
    )
    commands = [
        ["req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "1", "-config", str(config),
         "-subj", "/CN=DOSO SDK Test CA", "-addext", "basicConstraints=critical,CA:TRUE",
         "-keyout", str(tmp_path / "ca.key"), "-out", str(tmp_path / "ca.pem")],
        ["req", "-new", "-newkey", "rsa:2048", "-nodes", "-config", str(config), "-subj", "/CN=localhost",
         "-keyout", str(tmp_path / "server.key"), "-out", str(tmp_path / "server.csr")],
        ["x509", "-req", "-in", str(tmp_path / "server.csr"), "-CA", str(tmp_path / "ca.pem"),
         "-CAkey", str(tmp_path / "ca.key"), "-CAcreateserial", "-days", "1", "-extfile", str(extensions),
         "-out", str(tmp_path / "server.pem")],
    ]
    for arguments in commands:
        subprocess.run([openssl, *arguments], check=True, capture_output=True)
    return tmp_path


def test_tls_private_ca_and_api_key(monkeypatch, certificates):
    monkeypatch.setenv("LINGTU_GRPC_TLS_CERT_FILE", str(certificates / "server.pem"))
    monkeypatch.setenv("LINGTU_GRPC_TLS_KEY_FILE", str(certificates / "server.key"))
    monkeypatch.setenv("LINGTU_GRPC_API_KEY", "secure-sdk-key")
    credentials = grpc.ssl_channel_credentials((certificates / "ca.pem").read_bytes())
    with listener() as target, grpc.secure_channel(target, credentials) as channel:
        ping = system_rpc.SystemStub(channel).Ping
        with pytest.raises(grpc.RpcError) as error:
            ping(common_pb.Empty(), timeout=3)
        assert error.value.code() == grpc.StatusCode.UNAUTHENTICATED
        assert ping(common_pb.Empty(), metadata=(("x-api-key", "secure-sdk-key"),), timeout=3).status == "ok"


@pytest.mark.parametrize("wrong_host", [False, True])
def test_tls_rejects_untrusted_or_wrong_host(monkeypatch, certificates, wrong_host):
    monkeypatch.setenv("LINGTU_GRPC_TLS_CERT_FILE", str(certificates / "server.pem"))
    monkeypatch.setenv("LINGTU_GRPC_TLS_KEY_FILE", str(certificates / "server.key"))
    credentials = grpc.ssl_channel_credentials((certificates / "ca.pem").read_bytes() if wrong_host else None)
    options = (("grpc.ssl_target_name_override", "wrong.example"),) if wrong_host else ()
    with listener() as target, grpc.secure_channel(target, credentials, options=options) as channel:
        with pytest.raises(grpc.RpcError) as error:
            system_rpc.SystemStub(channel).Ping(common_pb.Empty(), timeout=2)
    assert error.value.code() in {grpc.StatusCode.UNAVAILABLE, grpc.StatusCode.DEADLINE_EXCEEDED}
