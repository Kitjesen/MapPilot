"""Posture RPCs preserve native admission, ownership and request identity."""

import socket
from concurrent.futures import Future
from contextlib import contextmanager

import grpc
import pytest

from gateway.gateway_module import GatewayModule
from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import error_pb2 as error_pb
from gateway.rpc.generated import motion_pb2 as motion_pb
from gateway.rpc.generated import motion_pb2_grpc as motion_rpc
from gateway.rpc.server import start
from runtime.adapters.brainstem.generated.brainstem_api.cms_pb2 import ProfileInfo
from runtime.msgs import NavigationCommandKind


class Controls:
    def __init__(self):
        self.calls = []
        self.accepted = True
        self.failure = None
        self.reason = None
        self.names = []
        self.omit_acceptance = False

    def stand(self, reason, *, request_id):
        return self.reply("stand", 11, reason, request_id)

    def sit(self, reason, *, request_id):
        return self.reply("sit", 12, reason, request_id)

    def enable_motors(self, reason, *, request_id):
        return self.reply("enable_motors", int(NavigationCommandKind.ENABLE_MOTORS), reason, request_id)

    def disable_motors(self, reason, *, request_id):
        return self.reply("disable_motors", int(NavigationCommandKind.DISABLE_MOTORS), reason, request_id)

    def switch_profile(self, reason, *, request_id, name):
        self.names.append(name)
        return self.reply("switch_profile", int(NavigationCommandKind.SWITCH_PROFILE), reason, request_id)

    def reply(self, action, kind, reason, request_id):
        self.calls.append((action, reason, request_id))
        if self.failure:
            raise self.failure
        result = {
            "accepted": self.accepted, "kind": kind, "task_id": "",
            "request_id": request_id, "endpoint_timestamp_s": 10.0,
            "reason": self.reason or ("posture_accepted" if self.accepted else "operator_active"),
        }
        if self.omit_acceptance:
            result.pop("accepted")
        return result


@contextmanager
def serve(robot):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        address = f"127.0.0.1:{listener.getsockname()[1]}"
    server = start(robot, address)
    channel = grpc.insecure_channel(address)
    try:
        grpc.channel_ready_future(channel).result(timeout=3)
        yield motion_rpc.MotionStub(channel)
    finally:
        channel.close()
        server.stop(0).wait(3)


def gateway():
    robot = GatewayModule()
    robot.setup()
    controls = Controls()
    robot.on_system_modules({"nav.commands": controls})
    return robot, controls


def command():
    return common_pb.Command(client_id="sdk", request_id="posture-1")


@pytest.mark.parametrize(("method", "action"), [("StandUp", "stand"), ("SitDown", "sit")])
def test_posture_uses_native_receipt_once(method, action):
    robot, controls = gateway()
    with serve(robot) as stub:
        operation = getattr(stub, method)
        first = operation(command(), timeout=3)
        repeated = operation(command(), timeout=3)
    assert first.accepted and repeated.accepted
    assert first.request_id == "posture-1"
    assert first.stage == "posture_requested"
    assert first.details["execution_confirmed"] is False
    assert first.details["native_ack"]["reason"] == "posture_accepted"
    assert controls.calls == [(action, f"operator_{action}", "posture-1")]


@pytest.mark.parametrize("blocker", ["owner", "estop", "native", "missing"])
def test_posture_preserves_existing_motion_gates(blocker):
    robot, controls = gateway()
    if blocker == "owner":
        assert robot._lease.acquire("other", 30)
    elif blocker == "estop":
        robot._navigation_state = {"authority": "estop"}
    elif blocker == "native":
        controls.accepted = False
    else:
        robot._nav_commands = None
        robot._all_modules = {}
    with serve(robot) as stub:
        with pytest.raises(grpc.RpcError) as error:
            stub.StandUp(command(), timeout=3)
    expected = grpc.StatusCode.PERMISSION_DENIED if blocker == "owner" else grpc.StatusCode.FAILED_PRECONDITION
    assert error.value.code() == expected
    assert len(controls.calls) == (1 if blocker == "native" else 0)


@pytest.mark.parametrize("reason", [
    None, "posture_ack_timeout", "posture_cancel_requested", "posture_action_ack_unknown", "posture_publish_failed",
])
def test_posture_unknown_ack_is_not_reported_as_rejection(reason):
    robot, controls = gateway()
    if reason is None:
        controls.failure = RuntimeError("native driver ACK timed out")
    else:
        controls.accepted = False
        controls.reason = reason
    with serve(robot) as stub:
        with pytest.raises(grpc.RpcError) as error:
            stub.SitDown(command(), timeout=3)
    assert error.value.code() == grpc.StatusCode.UNAVAILABLE
    metadata = dict(error.value.trailing_metadata())
    detail = error_pb.ErrorDetail.FromString(metadata["doso-error-bin"])
    assert detail.reason_code == "posture_ack_unknown"
    assert len(controls.calls) == 1


def test_posture_requires_request_identity():
    robot, controls = gateway()
    with serve(robot) as stub:
        with pytest.raises(grpc.RpcError) as error:
            stub.StandUp(common_pb.Command(client_id="sdk"), timeout=3)
    assert error.value.code() == grpc.StatusCode.INVALID_ARGUMENT
    assert not controls.calls


BODY_METHODS = [
    ("EnableMotors", "enable_motors", "motor_power_acknowledged"),
    ("DisableMotors", "disable_motors", "motor_power_acknowledged"),
    ("SwitchProfile", "switch_profile", "profile_switch_acknowledged"),
]


def body_request(method, *, name="stairs", identity=None):
    identity = command() if identity is None else identity
    if method == "SwitchProfile":
        return motion_pb.SwitchProfileRequest(command=identity, name=name)
    return identity


def error_detail(error):
    metadata = dict(error.trailing_metadata())
    return error_pb.ErrorDetail.FromString(metadata["doso-error-bin"])


@pytest.mark.parametrize(("method", "action", "stage"), BODY_METHODS)
def test_body_write_replays_exact_request_once(method, action, stage):
    robot, controls = gateway()
    controls.reason = "source_command_acknowledged"
    with serve(robot) as stub:
        operation = getattr(stub, method)
        first = operation(body_request(method), timeout=3)
        repeated = operation(body_request(method), timeout=3)
    assert first.accepted and repeated.accepted
    assert first.request_id == "posture-1"
    assert first.stage == stage
    assert first.details["execution_confirmed"] is False
    assert first.details["native_ack"]["reason"] == "source_command_acknowledged"
    assert repeated.details["command"]["replay"] is True
    assert controls.calls == [(action, f"operator_{action}", "posture-1")]
    assert controls.names == (["stairs"] if method == "SwitchProfile" else [])


def test_profile_name_participates_in_journal_identity():
    robot, controls = gateway()
    with serve(robot) as stub:
        first = stub.SwitchProfile(body_request("SwitchProfile", name="stairs"), timeout=3)
        assert first.accepted
        with pytest.raises(grpc.RpcError) as error:
            stub.SwitchProfile(body_request("SwitchProfile", name="flat"), timeout=3)
    assert error.value.code() == grpc.StatusCode.FAILED_PRECONDITION
    assert error_detail(error.value).reason_code == "idempotency_conflict"
    assert controls.names == ["stairs"]
    assert len(controls.calls) == 1


class ProfileSource:
    def __init__(self):
        self.calls = []
        self.value = ProfileInfo(
            current="flat", available=["flat", "stairs"],
            descriptions=["Flat ground", "Stairs"], current_description="Flat ground",
        )

    @contextmanager
    def read(self, sensor, timeout):
        self.calls.append(sensor)
        pending = Future()
        value = ProfileInfo()
        value.CopyFrom(self.value)
        pending.set_result(value)
        yield pending


def test_profile_query_reads_source_fields_again_without_control_commands():
    robot, controls = gateway()
    source = ProfileSource()
    robot._observations = source
    with serve(robot) as stub:
        first = stub.Profile(common_pb.Empty(), timeout=3)
        assert first.current == "flat"
        assert list(first.available) == ["flat", "stairs"]
        assert list(first.descriptions) == ["Flat ground", "Stairs"]
        assert first.current_description == "Flat ground"
        source.value.current = "stairs"
        source.value.current_description = "Stairs"
        repeated = stub.Profile(common_pb.Empty(), timeout=3)
    assert repeated.current == "stairs"
    assert repeated.current_description == "Stairs"
    assert source.calls == ["profile", "profile"]
    assert controls.calls == []


@pytest.mark.parametrize(("method", "action", "stage"), BODY_METHODS)
@pytest.mark.parametrize("missing", ["client_id", "request_id"])
def test_body_write_requires_command_identity(method, action, stage, missing):
    robot, controls = gateway()
    identity = command()
    setattr(identity, missing, " ")
    with serve(robot) as stub:
        with pytest.raises(grpc.RpcError) as error:
            getattr(stub, method)(body_request(method, identity=identity), timeout=3)
    assert error.value.code() == grpc.StatusCode.INVALID_ARGUMENT
    assert controls.calls == []


@pytest.mark.parametrize("name", ["", "   "])
def test_profile_switch_requires_nonempty_name(name):
    robot, controls = gateway()
    with serve(robot) as stub:
        with pytest.raises(grpc.RpcError) as error:
            stub.SwitchProfile(body_request("SwitchProfile", name=name), timeout=3)
    assert error.value.code() == grpc.StatusCode.INVALID_ARGUMENT
    assert controls.calls == []


@pytest.mark.parametrize(("method", "expected"), [
    ("EnableMotors", grpc.StatusCode.FAILED_PRECONDITION),
    ("DisableMotors", None),
])
def test_motor_power_preserves_stop_direction_at_safety_stop(method, expected):
    robot, controls = gateway()
    robot._navigation_state = {"authority": "estop"}
    with serve(robot) as stub:
        operation = getattr(stub, method)
        if expected is None:
            assert operation(command(), timeout=3).accepted
        else:
            with pytest.raises(grpc.RpcError) as error:
                operation(command(), timeout=3)
            assert error.value.code() == expected
    assert len(controls.calls) == (1 if expected is None else 0)


@pytest.mark.parametrize("method", ["EnableMotors", "DisableMotors"])
def test_motor_power_preserves_other_control_owner(method):
    robot, controls = gateway()
    assert robot._lease.acquire("other", 30)
    with serve(robot) as stub:
        with pytest.raises(grpc.RpcError) as error:
            getattr(stub, method)(command(), timeout=3)
    assert error.value.code() == grpc.StatusCode.PERMISSION_DENIED
    assert controls.calls == []


@pytest.mark.parametrize(("method", "action", "stage"), BODY_METHODS)
@pytest.mark.parametrize("failure", ["exception", "timeout_receipt", "missing_acceptance"])
def test_body_write_keeps_unknown_ack_unavailable(method, action, stage, failure):
    robot, controls = gateway()
    if failure == "exception":
        controls.failure = TimeoutError("native driver ACK timed out")
    elif failure == "timeout_receipt":
        controls.accepted = False
        controls.reason = "posture_ack_timeout"
    else:
        controls.omit_acceptance = True
    with serve(robot) as stub:
        with pytest.raises(grpc.RpcError) as error:
            getattr(stub, method)(body_request(method), timeout=3)
    assert error.value.code() == grpc.StatusCode.UNAVAILABLE
    detail = error_detail(error.value)
    assert detail.reason_code == f"{action}_ack_unknown"
    assert len(controls.calls) == 1


@pytest.mark.parametrize(("method", "action", "stage"), BODY_METHODS)
def test_body_write_preserves_native_rejection_reason(method, action, stage):
    robot, controls = gateway()
    controls.accepted = False
    controls.reason = "source_permission_denied"
    with serve(robot) as stub:
        with pytest.raises(grpc.RpcError) as error:
            getattr(stub, method)(body_request(method), timeout=3)
    assert error.value.code() == grpc.StatusCode.FAILED_PRECONDITION
    assert error_detail(error.value).details["detail"]["reason"] == "source_permission_denied"
    assert len(controls.calls) == 1
