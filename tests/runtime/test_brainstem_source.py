"""Brainstem profile reads use the shared endpoint without acquiring control."""

from concurrent import futures
from contextlib import contextmanager

import grpc
import pytest
from google.protobuf.empty_pb2 import Empty

from runtime.adapters.brainstem.generated.brainstem_api.cms_pb2 import ProfileInfo
from runtime.adapters.brainstem.source import Observations


@contextmanager
def profile_source(reply, *, status=None):
    calls = []

    def query(request, context):
        calls.append("GetProfile")
        if status is not None:
            context.abort(status, "profile source unavailable")
        return reply

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))
    server.add_generic_rpc_handlers((grpc.method_handlers_generic_handler(
        "brainstem.api.v1.RobotControl",
        {"GetProfile": grpc.unary_unary_rpc_method_handler(
            query,
            request_deserializer=Empty.FromString,
            response_serializer=ProfileInfo.SerializeToString,
        )},
    ),))
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    try:
        yield Observations({"LINGTU_DRIVER_TARGET": f"127.0.0.1:{port}"}), calls
    finally:
        server.stop(0).wait(3)


def test_profile_preserves_source_fields_without_control_calls():
    expected = ProfileInfo(
        current="flat",
        available=["flat", "stairs"],
        descriptions=["Flat terrain", "Stair traversal"],
        current_description="Flat terrain",
    )
    with profile_source(expected) as (source, calls):
        with source.read("profile", timeout=3) as pending:
            actual = pending.result()
    assert isinstance(actual, ProfileInfo)
    assert actual == expected
    assert calls == ["GetProfile"]


@pytest.mark.parametrize("status", [grpc.StatusCode.UNIMPLEMENTED, grpc.StatusCode.UNAVAILABLE])
def test_profile_preserves_upstream_read_error(status):
    with profile_source(ProfileInfo(), status=status) as (source, calls):
        with source.read("profile", timeout=3) as pending:
            with pytest.raises(grpc.RpcError) as error:
                pending.result()
    assert error.value.code() == status
    assert calls == ["GetProfile"]


@pytest.mark.parametrize("host", ["10.2.3.4", "172.16.0.1", "172.31.255.254", "192.168.66.64",
                                  "localhost", "127.0.0.1", "[::1]"])
def test_explicit_lan_development_uses_plaintext(monkeypatch, host):
    calls = []
    monkeypatch.setattr(grpc, "insecure_channel", lambda target, options: calls.append((target, options)) or "channel")
    target = f"{host}:13145"
    source = Observations({"LINGTU_DRIVER_TARGET": target, "LINGTU_OBSERVATIONS_TRANSPORT": "lan-dev"})
    assert source._channel() == "channel"
    assert calls == [(target, [(grpc.experimental.ChannelOptions.SingleThreadedUnaryStream, 1)])]


@pytest.mark.parametrize("host", ["8.8.8.8", "169.254.1.2", "100.64.0.1", "172.32.0.1",
                                  "robot.local", "[fd00::1]"])
def test_lan_development_requires_rfc1918_literal(host):
    with pytest.raises(ValueError, match="literal RFC1918"):
        Observations({"LINGTU_DRIVER_TARGET": f"{host}:13145",
                      "LINGTU_OBSERVATIONS_TRANSPORT": "lan-dev"})._channel()


@pytest.mark.parametrize("configuration", [
    {},
    {"LINGTU_DRIVER_TRANSPORT": "lan-dev"},
    {"LINGTU_OBSERVATIONS_TRANSPORT": "mtls"},
])
def test_remote_default_requires_mtls_independently_of_driver(configuration):
    with pytest.raises(ValueError, match="require mTLS"):
        Observations({"LINGTU_DRIVER_TARGET": "192.168.66.64:13145", **configuration})._channel()


@pytest.mark.parametrize("configuration,reason", [
    ({"LINGTU_OBSERVATIONS_TRANSPORT": "invalid"}, "transport must be"),
    ({"LINGTU_OBSERVATIONS_TRANSPORT": ""}, "transport must be"),
    ({"LINGTU_OBSERVATIONS_TRANSPORT": "lan-dev", "LINGTU_DRIVER_TLS_CA_FILE": "ca.pem"}, "cannot use TLS"),
    ({"LINGTU_OBSERVATIONS_TRANSPORT": "lan-dev", "LINGTU_DRIVER_TLS_SERVER_NAME": "robot"}, "cannot use TLS"),
    ({"LINGTU_OBSERVATIONS_TRANSPORT": "lan-dev", "LINGTU_DRIVER_TLS_CA_FILE": "ca.pem",
      "LINGTU_DRIVER_TLS_CERT_FILE": "cert.pem", "LINGTU_DRIVER_TLS_KEY_FILE": "key.pem"}, "cannot use TLS"),
])
def test_invalid_transport_and_conflicting_tls_fail_without_downgrade(configuration, reason):
    with pytest.raises(ValueError, match=reason):
        Observations({"LINGTU_DRIVER_TARGET": "127.0.0.1:13145", **configuration})._channel()
