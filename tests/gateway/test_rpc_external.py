"""Exercise external controller ownership and joint windows through local gRPC."""

import queue
import threading
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager

import grpc
import pytest

from gateway.rpc import external
from gateway.rpc.generated import common_pb2 as common
from gateway.rpc.generated import error_pb2 as error_pb
from gateway.rpc.generated import motion_pb2 as pb
from gateway.rpc.generated import motion_pb2_grpc as rpc
from gateway.rpc.security import APIKeyInterceptor
from gateway.services.commands import ControlLease


class Commands:
    def __init__(self):
        self.calls = []
        self.frames = []
        self.heartbeats = []
        self.exited = threading.Event()
        self.disabled = threading.Event()
        self.heartbeat_seen = threading.Event()
        self.rejected = None
        self.failure = None
        self.omit_acceptance = False
        self.snapshot = {
            "selected": False, "active": False, "motors_enabled": False,
            "session_id": "", "last_sequence": 0, "reason": "builtin",
            "fresh": True, "joint_count": 16, "input_window": 0,
        }

    def external_status(self):
        return self.snapshot

    def ack(self, action, request_id, session_id=None):
        self.calls.append((action, request_id, session_id))
        if self.failure:
            raise self.failure
        result = {
            "accepted": action != self.rejected,
            "kind": {"enter": 16, "enable": 18, "disable": 14, "close": 17}[action],
            "request_id": request_id, "task_id": "", "endpoint_timestamp_s": 12.0,
            "reason": "native_owner_busy" if action == self.rejected else f"external_{action}_acknowledged",
        }
        if self.omit_acceptance:
            result.pop("accepted")
        return result

    def external_enter(self, reason, *, request_id):
        result = self.ack("enter", request_id)
        if result.get("accepted"):
            self.snapshot.update(selected=True, session_id=request_id, input_window=123456)
        return result

    def external_enable(self, session_id, reason, *, request_id):
        result = self.ack("enable", request_id, session_id)
        if result.get("accepted"):
            self.snapshot.update(active=True, motors_enabled=True)
        return result

    def external_disable(self, session_id, reason, *, request_id):
        result = self.ack("disable", request_id, session_id)
        if result.get("accepted"):
            self.snapshot.update(active=False, motors_enabled=False)
        self.disabled.set()
        return result

    def external_exit(self, session_id, reason, *, request_id):
        result = self.ack("close", request_id, session_id)
        if result.get("accepted"):
            self.snapshot.update(selected=False, active=False, motors_enabled=False, session_id="", input_window=0)
        self.exited.set()
        return result

    def external_frame(self, session_id, sequence, joints, input_window):
        self.frames.append((session_id, sequence, joints, input_window))
        return True

    def external_heartbeat(self, session_id):
        self.heartbeats.append(session_id)
        self.heartbeat_seen.set()
        return True


class Gateway:
    def __init__(self):
        self._nav_commands = Commands()
        self._state_lock = threading.Lock()
        self._navigation_state = {}
        self._lease = ControlLease()


class Requests:
    def __init__(self):
        self.pending = queue.Queue()
        self.sequence = 0

    def __iter__(self):
        while (request := self.pending.get()) is not None:
            yield request

    def send(self, action, request_id, *, sequence=None, client="sdk", window=0):
        if sequence is None:
            sequence = self.sequence + 1
        self.sequence = max(sequence, self.sequence)
        self.pending.put(pb.ExternalControlRequest(
            command=common.Command(client_id=client, request_id=request_id),
            sequence=sequence, input_window=window, **action,
        ))

    def close(self):
        self.pending.put(None)


@contextmanager
def serve(gateway, *, key=None, slots=2):
    available = threading.BoundedSemaphore(slots)

    class Service(rpc.MotionServicer):
        def Controller(self, request, context):
            return external.state(gateway)

        def ExternalControl(self, requests, context):
            return external.control(gateway, requests, context, available)

    server = grpc.server(ThreadPoolExecutor(max_workers=4), interceptors=(APIKeyInterceptor(key),) if key else ())
    rpc.add_MotionServicer_to_server(Service(), server)
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    channel = grpc.insecure_channel(f"127.0.0.1:{port}")
    streams = []

    def connect(metadata=()):
        requests = Requests()
        call = rpc.MotionStub(channel).ExternalControl(iter(requests), timeout=4, metadata=metadata)
        streams.append((requests, call))
        return requests, call

    try:
        yield connect, rpc.MotionStub(channel)
    finally:
        for requests, call in streams:
            requests.close()
            call.cancel()
        channel.close()
        server.stop(0).wait(3)


def response(call):
    while True:
        reply = next(call)
        if reply.request_id:
            return reply


def enter(requests, call):
    requests.send({"enter": common.Empty()}, "external-session")
    reply = response(call)
    assert reply.accepted
    return reply


def joints(count=16):
    return pb.JointTargets(joints=[pb.JointCommand(q=0.1, dq=0.2, kp=30, kd=1, tau=0.3) for _ in range(count)])


def detail(error):
    return error_pb.ErrorDetail.FromString(dict(error.trailing_metadata())["doso-error-bin"])


def test_enter_enable_frames_disable_close_preserve_native_session():
    gateway = Gateway()
    native = gateway._nav_commands
    with serve(gateway) as (connect, _):
        requests, call = connect()
        opened = enter(requests, call)
        assert opened.input_window == 123456
        assert opened.state.mode == pb.CONTROLLER_MODE_EXTERNAL
        requests.send({"enable": common.Empty()}, "enable")
        assert response(call).state.motor_output_enabled
        requests.send({"frame": joints()}, "frame", window=opened.input_window)
        queued = response(call)
        assert queued.accepted and queued.ingress_accepted
        assert queued.reason == "queued"
        assert queued.state.last_sequence == 0
        session, sequence, targets, window = native.frames[0]
        assert (session, sequence, window) == ("external-session", 3, 123456)
        assert targets[0] == {"q": 0.1, "dq": 0.2, "kp": 30, "kd": 1, "tau": 0.3}
        native.snapshot["last_sequence"] = sequence
        requests.send({"disable": common.Empty()}, "disable")
        disabled = response(call)
        assert disabled.accepted and not disabled.state.motor_output_enabled
        assert disabled.state.last_sequence == 3
        requests.send({"close": common.Empty()}, "close")
        assert response(call).accepted
        with pytest.raises(StopIteration):
            next(call)
    assert [item[0] for item in native.calls] == ["enter", "enable", "disable", "close"]


def test_controller_query_preserves_unknown_and_stale_source():
    gateway = Gateway()
    native = gateway._nav_commands
    with serve(gateway) as (_, stub):
        known = stub.Controller(common.Empty(), timeout=2)
        assert known.fresh and known.joint_count == 16
        assert known.mode == pb.CONTROLLER_MODE_BUILTIN
        native.snapshot.update(selected=True, active=True, motors_enabled=True, fresh=False)
        stale = stub.Controller(common.Empty(), timeout=2)
        assert not stale.fresh and not stale.active and not stale.motor_output_enabled
        assert stale.mode == pb.CONTROLLER_MODE_UNKNOWN
        native.snapshot = None
        unavailable = stub.Controller(common.Empty(), timeout=2)
        assert not unavailable.fresh and unavailable.joint_count == 0
    assert native.calls == []


def test_original_window_and_zero_sequence_heartbeat_are_preserved():
    gateway = Gateway()
    native = gateway._nav_commands
    with serve(gateway) as (connect, _):
        requests, call = connect()
        opened = enter(requests, call)
        native.snapshot["input_window"] = 999999
        requests.send({"frame": joints()}, "first-frame", window=opened.input_window)
        assert response(call).ingress_accepted
        requests.send({"heartbeat": common.Empty()}, "external-session:heartbeat", sequence=0)
        assert native.heartbeat_seen.wait(2)
        requests.send({"frame": joints()}, "second-frame", window=999999)
        assert response(call).ingress_accepted
        requests.send({"close": common.Empty()}, "close")
        assert response(call).accepted
    assert [frame[1] for frame in native.frames] == [2, 3]
    assert [frame[3] for frame in native.frames] == [123456, 999999]
    assert native.heartbeats == ["external-session"]


def test_heartbeat_reply_is_only_a_state_update():
    gateway = Gateway()
    with serve(gateway) as (connect, _):
        requests, call = connect()
        enter(requests, call)
        requests.send({"heartbeat": common.Empty()}, "heartbeat", sequence=0)
        while True:
            reply = next(call)
            assert reply.sequence == 0 and reply.request_id == ""
            if reply.reason == "queued":
                break
        requests.send({"close": common.Empty()}, "close")
        assert response(call).accepted


@pytest.mark.parametrize("ending", ["half_close", "cancel", "idle"])
def test_stream_termination_disables_and_preserves_external_isolation(ending, monkeypatch):
    gateway = Gateway()
    if ending == "idle":
        monkeypatch.setattr(external, "IDLE_TIMEOUT_S", 0.08)
    with serve(gateway) as (connect, _):
        requests, call = connect()
        enter(requests, call)
        if ending == "half_close":
            requests.close()
            list(call)
        elif ending == "cancel":
            call.cancel()
        else:
            with pytest.raises(grpc.RpcError) as error:
                response(call)
            assert error.value.code() == grpc.StatusCode.DEADLINE_EXCEEDED
        assert gateway._nav_commands.disabled.wait(2)
    assert gateway._nav_commands.calls[-1][0] == "disable"
    assert gateway._nav_commands.calls[-1][2] == "external-session"
    assert gateway._nav_commands.snapshot["selected"] is True
    assert gateway._nav_commands.snapshot["active"] is False


@pytest.mark.parametrize("fault", ["wrong_handshake", "client", "sequence", "nonfinite", "negative_gain", "count", "window"])
def test_invalid_external_inputs_never_reach_joint_output(fault):
    gateway = Gateway()
    with serve(gateway) as (connect, _):
        requests, call = connect()
        if fault == "wrong_handshake":
            requests.send({"enable": common.Empty()}, "enable")
        else:
            enter(requests, call)
            if fault == "client":
                requests.send({"enable": common.Empty()}, "enable", client="other")
            elif fault == "sequence":
                requests.send({"heartbeat": common.Empty()}, "heartbeat", sequence=2)
            else:
                targets = joints(15 if fault == "count" else 16)
                if fault == "nonfinite":
                    targets.joints[0].tau = float("nan")
                if fault == "negative_gain":
                    targets.joints[0].kp = -1
                requests.send({"frame": targets}, "frame", window=0 if fault == "window" else 123456)
        with pytest.raises(grpc.RpcError) as error:
            response(call)
        assert error.value.code() == grpc.StatusCode.INVALID_ARGUMENT
    assert gateway._nav_commands.frames == []


def test_replayed_frame_sequence_is_rejected_without_native_submission():
    gateway = Gateway()
    with serve(gateway) as (connect, _):
        requests, call = connect()
        enter(requests, call)
        requests.send({"frame": joints()}, "frame", sequence=2, window=123456)
        assert response(call).ingress_accepted
        requests.send({"frame": joints()}, "duplicate", sequence=2, window=123456)
        rejected = response(call)
        assert not rejected.accepted and rejected.reason == "stale_sequence"
        requests.send({"close": common.Empty()}, "close")
        assert response(call).accepted
    assert len(gateway._nav_commands.frames) == 1


@pytest.mark.parametrize("blocker", ["lease", "safety"])
def test_enter_preserves_existing_control_guards(blocker):
    gateway = Gateway()
    if blocker == "lease":
        assert gateway._lease.acquire("other", 3)
    else:
        gateway._navigation_state = {"authority": "estop"}
    with serve(gateway) as (connect, _):
        requests, call = connect()
        requests.send({"enter": common.Empty()}, "external-session")
        with pytest.raises(grpc.RpcError) as error:
            response(call)
    expected = grpc.StatusCode.PERMISSION_DENIED if blocker == "lease" else grpc.StatusCode.FAILED_PRECONDITION
    assert error.value.code() == expected
    assert gateway._nav_commands.calls == []


@pytest.mark.parametrize("unknown", ["exception", "missing_acceptance"])
def test_unknown_enter_ack_still_cleans_up_possible_session(unknown):
    gateway = Gateway()
    native = gateway._nav_commands
    original = native.external_enter

    def uncertain(reason, *, request_id):
        if unknown == "exception":
            native.calls.append(("enter", request_id, None))
            raise TimeoutError("native ACK timed out")
        native.omit_acceptance = True
        try:
            return original(reason, request_id=request_id)
        finally:
            native.omit_acceptance = False

    native.external_enter = uncertain
    with serve(gateway) as (connect, _):
        requests, call = connect()
        requests.send({"enter": common.Empty()}, "external-session")
        with pytest.raises(grpc.RpcError) as error:
            response(call)
        assert error.value.code() == grpc.StatusCode.UNAVAILABLE
        assert detail(error.value).reason_code == "external_enter_ack_unknown"
        assert native.disabled.wait(2)
    assert native.calls[-1][2] == "external-session"


def test_explicit_native_rejection_preserves_reason():
    gateway = Gateway()
    gateway._nav_commands.rejected = "enter"
    with serve(gateway) as (connect, _):
        requests, call = connect()
        requests.send({"enter": common.Empty()}, "external-session")
        rejected = response(call)
        assert not rejected.accepted and rejected.reason == "native_owner_busy"
    assert len(gateway._nav_commands.calls) == 1


def test_api_key_protects_external_stream():
    gateway = Gateway()
    with serve(gateway, key="test-key") as (connect, _):
        requests, call = connect()
        requests.send({"enter": common.Empty()}, "external-session")
        with pytest.raises(grpc.RpcError) as error:
            response(call)
    assert error.value.code() == grpc.StatusCode.UNAUTHENTICATED
    assert gateway._nav_commands.calls == []



def test_external_stream_shares_available_worker_slots():
    gateway = Gateway()
    with serve(gateway, slots=1) as (connect, _):
        first, first_call = connect()
        enter(first, first_call)
        second, second_call = connect()
        second.send({"enter": common.Empty()}, "second-session")
        with pytest.raises(grpc.RpcError) as error:
            response(second_call)
        assert error.value.code() == grpc.StatusCode.RESOURCE_EXHAUSTED
        first.send({"close": common.Empty()}, "close")
        assert response(first_call).accepted
    assert [item[0] for item in gateway._nav_commands.calls] == ["enter", "close"]


@pytest.mark.parametrize("broken", ["request_id", "kind", "timeout_reason"])
def test_uncorrelated_native_ack_is_unknown(broken):
    gateway = Gateway()
    native = gateway._nav_commands
    original = native.external_enable

    def uncorrelated(session_id, reason, *, request_id):
        result = original(session_id, reason, request_id=request_id)
        if broken == "timeout_reason":
            result["accepted"] = False
            result["reason"] = "posture_ack_timeout"
        else:
            result[broken] = "other-request" if broken == "request_id" else 16
        return result

    native.external_enable = uncorrelated
    with serve(gateway) as (connect, _):
        requests, call = connect()
        enter(requests, call)
        requests.send({"enable": common.Empty()}, "enable")
        with pytest.raises(grpc.RpcError) as error:
            response(call)
        assert error.value.code() == grpc.StatusCode.UNAVAILABLE
        assert detail(error.value).reason_code == "external_enable_ack_unknown"
        assert native.disabled.wait(2)


@pytest.mark.parametrize("fault", ["unknown_count", "stale_status", "queue_exception"])
def test_unavailable_output_is_not_reported_as_applied(fault):
    gateway = Gateway()
    native = gateway._nav_commands
    with serve(gateway) as (connect, _):
        requests, call = connect()
        enter(requests, call)
        if fault == "unknown_count":
            native.snapshot.pop("joint_count")
        elif fault == "stale_status":
            native.snapshot["fresh"] = False
        else:
            def unavailable(*args):
                raise RuntimeError("native frame publish failed")
            native.external_frame = unavailable
        requests.send({"frame": joints()}, "frame", window=123456)
        with pytest.raises(grpc.RpcError) as error:
            response(call)
        assert error.value.code() == grpc.StatusCode.UNAVAILABLE
        assert native.disabled.wait(2)
    assert native.frames == []
