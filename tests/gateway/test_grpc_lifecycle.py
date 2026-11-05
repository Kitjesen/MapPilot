"""The listener follows the Gateway's existing start/stop owner."""

import socket

import grpc

from gateway.gateway_module import GatewayModule
from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import system_pb2_grpc as system_rpc
from gateway.services.lifecycle import start_background_threads, stop_background_threads


def test_listener_stops_with_gateway():
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    gateway = GatewayModule(host="127.0.0.1", grpc_port=port)
    gateway._drift_watchdog_enabled = False
    gateway._run_server = lambda stopped: stopped.wait(3)
    start_background_threads(gateway)
    try:
        with grpc.insecure_channel(f"127.0.0.1:{port}") as channel:
            assert system_rpc.SystemStub(channel).Ping(common_pb.Empty(), timeout=3).status == "ok"
    finally:
        stop_background_threads(gateway)
    assert gateway._grpc_server is None
    assert gateway._server_thread is None


def test_deferred_gateway_does_not_listen():
    gateway = GatewayModule()
    gateway._defer_server = True
    gateway._drift_watchdog_enabled = False
    start_background_threads(gateway)
    assert gateway._grpc_server is None
    stop_background_threads(gateway)
