"""Motor diagnostics through the source and Gateway gRPC boundaries."""

import socket
import threading
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from types import SimpleNamespace

import grpc
import pytest
from google.protobuf.empty_pb2 import Empty

from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import observations_pb2_grpc as observations_rpc
from gateway.rpc.server import start
from runtime.adapters.brainstem.generated.brainstem_api import cms_pb2 as source_pb
from runtime.adapters.brainstem.source import Observations


@contextmanager
def diagnostics(handlers):
    upstream = grpc.server(ThreadPoolExecutor(max_workers=2))
    upstream.add_generic_rpc_handlers((grpc.method_handlers_generic_handler(
        "brainstem.api.v1.RobotControl",
        {name: grpc.unary_unary_rpc_method_handler(
            handler, request_deserializer=Empty.FromString, response_serializer=lambda event: event.SerializeToString(),
        ) for name, handler in handlers.items()},
    ),))
    port = upstream.add_insecure_port("127.0.0.1:0")
    upstream.start()
    source = Observations({"LINGTU_DRIVER_TARGET": f"127.0.0.1:{port}"})
    try:
        with gateway(source) as stub:
            yield stub
    finally:
        upstream.stop(0).wait(3)


@contextmanager
def gateway(source):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    server = start(SimpleNamespace(_observations=source), f"127.0.0.1:{port}")
    with grpc.insecure_channel(f"127.0.0.1:{port}") as channel:
        try:
            yield observations_rpc.ObservationsStub(channel)
        finally:
            server.stop(0).wait(3)


def test_motor_diagnostics_preserve_reported_values():
    motors = source_pb.MotorStatusResponse(motors=[source_pb.MotorState(
        id=index, online=index != 2, status_code=7 if index != 2 else 0,
        temperature=42.5 if index != 2 else 0, voltage=999, errors=[3, 5] if index == 3 else [],
    ) for index in range(16)])
    volts = [48.1] * 16
    volts[2] = 0
    with diagnostics({
        "GetMotorStatus": lambda request, context: motors,
        "GetVoltage": lambda request, context: source_pb.Voltage(values=volts),
    }) as stub:
        state = stub.Motors(common_pb.Empty(), timeout=3)
        voltage = stub.MotorVoltages(common_pb.Empty(), timeout=3)
    assert [motor.id for motor in state.motors] == list(range(16))
    assert (state.motors[0].online, state.motors[0].status_code, state.motors[0].temperature_c) == (True, 7, 42.5)
    assert not state.motors[2].online
    assert state.motors[2].temperature_c == 0
    assert list(state.motors[3].errors) == [3, 5]
    assert "voltage" not in state.motors[0].DESCRIPTOR.fields_by_name
    assert list(voltage.volts) == volts


@pytest.mark.parametrize("upstream_method,method,response", [
    ("GetMotorStatus", "Motors", source_pb.MotorStatusResponse()),
    ("GetMotorStatus", "Motors", source_pb.MotorStatusResponse(
        motors=[source_pb.MotorState(id=0) for _ in range(16)],
    )),
    ("GetVoltage", "MotorVoltages", source_pb.Voltage(values=[48])),
])
def test_incomplete_or_duplicate_motor_data_is_unavailable(upstream_method, method, response):
    with diagnostics({upstream_method: lambda request, context: response}) as stub:
        with pytest.raises(grpc.RpcError) as caught:
            getattr(stub, method)(common_pb.Empty(), timeout=3)
    assert caught.value.code() == grpc.StatusCode.UNAVAILABLE


@pytest.mark.parametrize("upstream_method,method", [("GetMotorStatus", "Motors"), ("GetVoltage", "MotorVoltages")])
def test_motor_source_errors_are_preserved(upstream_method, method):
    def denied(request, context):
        context.set_trailing_metadata((("source-error", "denied"),))
        context.abort(grpc.StatusCode.PERMISSION_DENIED, "diagnostics denied")

    with diagnostics({upstream_method: denied}) as stub:
        with pytest.raises(grpc.RpcError) as caught:
            getattr(stub, method)(common_pb.Empty(), timeout=3)
    assert caught.value.code() == grpc.StatusCode.PERMISSION_DENIED
    assert caught.value.details() == "diagnostics denied"
    assert ("source-error", "denied") in caught.value.trailing_metadata()


@pytest.mark.parametrize("method", ["Motors", "MotorVoltages"])
@pytest.mark.parametrize("source", [None, Observations({"LINGTU_DRIVER_TARGET": "192.168.66.12:13145"})])
def test_motor_source_missing_configuration_is_unavailable(method, source):
    with gateway(source) as stub, pytest.raises(grpc.RpcError) as caught:
        getattr(stub, method)(common_pb.Empty(), timeout=3)
    assert caught.value.code() == grpc.StatusCode.UNAVAILABLE


@pytest.mark.parametrize("cancel", [False, True])
def test_motor_deadline_and_cancel_release_source(cancel):
    started = threading.Event()
    cancelled = threading.Event()

    def waiting(request, context):
        context.add_callback(cancelled.set)
        started.set()
        cancelled.wait(3)
        return source_pb.Voltage(values=[48] * 16)

    with diagnostics({"GetVoltage": waiting}) as stub:
        call = stub.MotorVoltages.future(common_pb.Empty(), timeout=3 if cancel else 0.2)
        assert started.wait(2)
        if cancel:
            assert call.cancel()
            with pytest.raises(grpc.FutureCancelledError):
                call.result()
        else:
            with pytest.raises(grpc.RpcError) as caught:
                call.result()
            assert caught.value.code() == grpc.StatusCode.DEADLINE_EXCEEDED
        assert cancelled.wait(2)
