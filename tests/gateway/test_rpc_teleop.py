"""Exercise SDK teleop over local gRPC with the real native session adapter."""

import queue
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from types import SimpleNamespace

import grpc
import pytest

from gateway.rpc import teleop
from gateway.rpc.generated import common_pb2 as common
from gateway.rpc.generated import motion_pb2 as pb
from gateway.rpc.generated import motion_pb2_grpc as rpc
from gateway.rpc.security import APIKeyInterceptor
from runtime.msgs.nav import OperatorMotionAction, OperatorMotionReceipt


class Gateway:
    def __init__(self):
        self._state_lock = threading.Lock()
        self._web_teleop_owner_lock = threading.Lock()
        self._web_teleop_owner = None
        self._navigation_state = {"authority": "autonomy"}
        self._teleop_native_publisher = None
        self.claims = []
        self.samples = []
        self.releases = []
        self.released = threading.Event()
        self.reject_release = False

    def _teleop_claim(self, **values):
        self.claims.append(values)
        return self._receipt(OperatorMotionAction.CLAIM, values)

    def _teleop_on_velocity(self, vx, vy, yaw, **values):
        self.samples.append((vx, vy, yaw, values))
        return True

    def _teleop_release(self, **values):
        self.releases.append(values)
        action = OperatorMotionAction.RELEASE if "release_sequence" in values else OperatorMotionAction.HOLD
        self.released.set()
        return self._receipt(action, values, accepted=not self.reject_release)

    @staticmethod
    def _receipt(action, values, *, accepted=True):
        sequence = values.get("release_sequence", values["sequence"])
        return OperatorMotionReceipt(
            accepted=accepted,
            action=action,
            request_id=values["request_id"],
            source_id=values["source_id"],
            source_epoch=values["source_epoch"],
            source_sequence=sequence,
            accepted_sequence=sequence if accepted else 0,
            final_output_sequence=sequence if accepted and action != OperatorMotionAction.CLAIM else 0,
            endpoint_timestamp_s=time.time(),
            reason="accepted" if accepted else "stop_unconfirmed",
        )


class Requests:
    def __init__(self):
        self.pending = queue.Queue()
        self.sequence = 0

    def __iter__(self):
        while (value := self.pending.get()) is not None:
            yield value

    def send(self, action, request_id, *, client="sdk-test", window=0, sequence=None):
        if sequence is None:
            sequence = self.sequence + 1
        self.sequence = max(sequence, self.sequence)
        self.pending.put(
            pb.TeleopRequest(
                command=common.Command(request_id=request_id, client_id=client),
                input_window=window,
                sequence=sequence,
                **action,
            )
        )
        return sequence

    def close(self):
        self.pending.put(None)


@contextmanager
def serve(gateway, *, api_key=None):
    slots = threading.BoundedSemaphore(2)

    class Service(rpc.MotionServicer):
        def Teleop(self, requests, context):
            return teleop.teleop(gateway, requests, context, slots)

    interceptors = (APIKeyInterceptor(api_key),) if api_key else ()
    server = grpc.server(ThreadPoolExecutor(max_workers=4), interceptors=interceptors)
    rpc.add_MotionServicer_to_server(Service(), server)
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    channel = grpc.insecure_channel(f"127.0.0.1:{port}")
    pipes = []

    def connect():
        pipe = Requests()
        call = rpc.MotionStub(channel).Teleop(iter(pipe), timeout=5)
        pipes.append((pipe, call))
        return pipe, call

    try:
        yield connect
    finally:
        for pipe, call in pipes:
            pipe.close()
            call.cancel()
        channel.close()
        server.stop(0).wait(3)


def move(pipe, call, request_id="move-1"):
    pipe.send({"input": common.Empty()}, "input-1")
    window = response(call).input_window
    pipe.send({"velocity": pb.TeleopVelocity(vx_m_s=0.2, deadman=True)}, request_id, window=window)
    return window, response(call)


def response(call):
    while True:
        reply = next(call)
        if reply.request_id:
            return reply


def test_velocity_hold_and_close_preserve_native_session():
    gateway = Gateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        _, reply = move(pipe, call)
        assert reply.accepted and reply.ingress_accepted and not reply.final_output_confirmed
        assert reply.request_id == "move-1"
        assert gateway.samples[0][:3] == (0.2, 0.0, 0.0)
        assert gateway.samples[0][3]["deadline_monotonic_s"] > time.monotonic()
        pipe.send({"hold": common.Empty()}, "hold-1")
        reply = response(call)
        assert reply.accepted and reply.final_output_confirmed and not reply.ingress_accepted
        pipe.send({"close": common.Empty()}, "close-1")
        reply = response(call)
        assert reply.accepted and reply.final_output_confirmed
        assert gateway._web_teleop_owner is None
        assert len(gateway.releases) == 2
        with pytest.raises(StopIteration):
            response(call)
    assert len(gateway.claims) == 1


def test_windows_allow_multiple_samples_and_reject_replayed_sequence():
    gateway = Gateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        pipe.send({"input": common.Empty()}, "input")
        response(call)
        pipe.send({"velocity": pb.TeleopVelocity(vx_m_s=0.4, deadman=True)}, "missing")
        assert response(call).reason == "input_expired"
        assert gateway.claims == [] and gateway.samples == []
        window, accepted = move(pipe, call)
        assert accepted.accepted
        pipe.send({"velocity": pb.TeleopVelocity(vx_m_s=0.3, deadman=True)}, "same-window", window=window)
        accepted = response(call)
        assert accepted.accepted and accepted.sequence == pipe.sequence
        pipe.send(
            {"velocity": pb.TeleopVelocity(vx_m_s=0.9, deadman=True)},
            "replayed",
            window=window,
            sequence=accepted.sequence,
        )
        rejected = response(call)
        assert not rejected.accepted and rejected.reason == "stale_sequence"
        assert len(gateway.samples) == 2


def test_expired_window_holds_without_claiming(monkeypatch):
    monkeypatch.setattr(teleop, "INPUT_WINDOW_S", 0.01)
    gateway = Gateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        pipe.send({"input": common.Empty()}, "input")
        window = response(call).input_window
        time.sleep(0.03)
        pipe.send({"velocity": pb.TeleopVelocity(vx_m_s=0.5, deadman=True)}, "expired", window=window)
        assert response(call).reason == "input_expired"
        assert gateway.samples == [] and gateway.claims == []


@pytest.mark.parametrize("cancel", [True, False])
def test_disconnect_and_idle_release_native_authority(monkeypatch, cancel):
    monkeypatch.setattr(teleop, "IDLE_TIMEOUT_S", 0.05)
    gateway = Gateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        assert move(pipe, call)[1].accepted
        if cancel:
            call.cancel()
        else:
            with pytest.raises(grpc.RpcError) as error:
                response(call)
            assert error.value.code() == grpc.StatusCode.DEADLINE_EXCEEDED
        assert gateway.released.wait(2)
        deadline = time.monotonic() + 2
        while gateway._web_teleop_owner is not None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert gateway._web_teleop_owner is None
        assert "release_sequence" in gateway.releases[-1]


def test_web_and_sdk_share_single_controller_slot():
    gateway = Gateway()
    gateway._web_teleop_owner = "web:existing"
    with serve(gateway) as connect:
        pipe, call = connect()
        pipe.send({"input": common.Empty()}, "input")
        with pytest.raises(grpc.RpcError) as error:
            response(call)
        assert error.value.code() == grpc.StatusCode.RESOURCE_EXHAUSTED
    assert gateway._web_teleop_owner == "web:existing"


def test_rejected_release_is_reported_without_double_release():
    gateway = Gateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        assert move(pipe, call)[1].accepted
        gateway.reject_release = True
        pipe.send({"close": common.Empty()}, "close")
        reply = response(call)
        assert not reply.accepted and not reply.final_output_confirmed
        assert reply.reason == "stop_unconfirmed"
        assert gateway._web_teleop_owner is None
        assert len(gateway.releases) == 1


def test_invalid_velocity_closes_session():
    gateway = Gateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        pipe.send({"input": common.Empty()}, "input")
        window = response(call).input_window
        pipe.send({"velocity": pb.TeleopVelocity(vx_m_s=float("nan"), deadman=True)}, "bad", window=window)
        with pytest.raises(grpc.RpcError) as error:
            response(call)
        assert error.value.code() == grpc.StatusCode.INVALID_ARGUMENT
    assert gateway.samples == []


def test_changed_identity_closes_session_and_releases_authority():
    gateway = Gateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        assert move(pipe, call)[1].accepted
        pipe.send({"input": common.Empty()}, "changed", client="other-client")
        with pytest.raises(grpc.RpcError) as error:
            response(call)
        assert error.value.code() == grpc.StatusCode.INVALID_ARGUMENT
        assert gateway.released.wait(2)
    assert len(gateway.samples) == 1


def test_deadman_release_holds_without_valid_input_window():
    gateway = Gateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        assert move(pipe, call)[1].accepted
        pipe.send({"velocity": pb.TeleopVelocity(vx_m_s=0.9, deadman=False)}, "deadman")
        held = response(call)
        assert held.accepted and held.final_output_confirmed
        assert not held.ingress_accepted
        assert len(gateway.samples) == 1


def test_input_half_close_releases_authority():
    gateway = Gateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        assert move(pipe, call)[1].accepted
        pipe.close()
        with pytest.raises(StopIteration):
            response(call)
        assert gateway.released.wait(2)
    assert gateway._web_teleop_owner is None


def test_claim_delay_cannot_extend_input_window(monkeypatch):
    monkeypatch.setattr(teleop, "INPUT_WINDOW_S", 0.02)
    gateway = Gateway()
    claim = gateway._teleop_claim

    def slow_claim(**values):
        time.sleep(0.04)
        return claim(**values)

    gateway._teleop_claim = slow_claim
    with serve(gateway) as connect:
        pipe, call = connect()
        reply = move(pipe, call)[1]
        assert not reply.accepted and reply.reason == "input_expired"
    assert gateway.samples == []


def test_safety_stop_rejects_velocity():
    gateway = Gateway()
    gateway._navigation_state = {"authority": "estop"}
    with serve(gateway) as connect:
        pipe, call = connect()
        rejected = move(pipe, call)[1]
        assert not rejected.accepted and rejected.reason == "safety_stop"
    assert gateway.samples == [] and gateway.claims == []


def test_unauthenticated_stream_cannot_reserve_or_claim():
    gateway = Gateway()
    with serve(gateway, api_key="required-key") as connect:
        pipe, call = connect()
        pipe.send({"input": common.Empty()}, "unauthenticated")
        with pytest.raises(grpc.RpcError) as error:
            response(call)
        assert error.value.code() == grpc.StatusCode.UNAUTHENTICATED
    assert gateway._web_teleop_owner is None
    assert gateway.claims == [] and gateway.samples == [] and gateway.releases == []


def test_disconnect_exception_frees_owner_without_retrying_release():
    gateway = Gateway()
    release_attempts = []

    def failed_release(**values):
        release_attempts.append(values)
        raise RuntimeError("native release unavailable")

    with serve(gateway) as connect:
        pipe, call = connect()
        assert move(pipe, call)[1].accepted
        gateway._teleop_release = failed_release
        pipe.send({"close": common.Empty()}, "close-failed")
        with pytest.raises(grpc.RpcError) as error:
            response(call)
        assert error.value.code() == grpc.StatusCode.UNKNOWN
        assert gateway._web_teleop_owner is None
        assert len(release_attempts) == 1
        other_pipe, other_call = connect()
        other_pipe.send({"input": common.Empty()}, "reconnected")
        assert response(other_call).accepted
    assert len(gateway.claims) == 1
    assert len(release_attempts) == 1


def test_window_deadline_never_renews_with_samples_or_new_windows(monkeypatch):
    monkeypatch.setattr(teleop, "INPUT_WINDOW_S", 0.12)
    monkeypatch.setattr(teleop, "WINDOW_INTERVAL_S", 0.02)
    gateway = Gateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        window, accepted = move(pipe, call)
        assert accepted.accepted
        deadline = gateway.samples[0][3]["deadline_monotonic_s"]
        time.sleep(0.04)
        pipe.send({"velocity": pb.TeleopVelocity(vx_m_s=0.3, deadman=True)}, "same-window", window=window)
        assert response(call).accepted
        assert gateway.samples[1][3]["deadline_monotonic_s"] == deadline
        time.sleep(max(0.0, deadline - time.monotonic()) + 0.02)
        pipe.send({"velocity": pb.TeleopVelocity(vx_m_s=0.9, deadman=True)}, "old-window", window=window)
        rejected = response(call)
        assert not rejected.accepted and rejected.reason == "input_expired"
        assert rejected.input_window > window
        assert len(gateway.samples) == 2


def test_periodic_windows_do_not_extend_client_idle_timeout(monkeypatch):
    monkeypatch.setattr(teleop, "IDLE_TIMEOUT_S", 0.18)
    monkeypatch.setattr(teleop, "WINDOW_INTERVAL_S", 0.03)
    gateway = Gateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        sequence = pipe.send({"input": common.Empty()}, "input")
        initial = response(call)
        assert initial.sequence == sequence
        updates = []
        with pytest.raises(grpc.RpcError) as error:
            while True:
                update = next(call)
                assert update.sequence == 0 and update.request_id == ""
                assert update.accepted and update.reason == "input_ready"
                updates.append(update.input_window)
        assert error.value.code() == grpc.StatusCode.DEADLINE_EXCEEDED
        assert len(updates) >= 2
        assert updates == sorted(set(updates))
    assert gateway.claims == [] and gateway.samples == []


def test_hold_invalidates_previous_windows_and_returns_fresh_window():
    gateway = Gateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        window, accepted = move(pipe, call)
        assert accepted.accepted
        pipe.send({"hold": common.Empty()}, "hold")
        held = response(call)
        assert held.accepted and held.final_output_confirmed
        assert held.input_window > window
        pipe.send({"velocity": pb.TeleopVelocity(vx_m_s=0.9, deadman=True)}, "queued-before-hold", window=window)
        expired = response(call)
        assert not expired.accepted and expired.reason == "input_expired"
        assert expired.input_window > held.input_window
        pipe.send({"velocity": pb.TeleopVelocity(vx_m_s=0.2, deadman=True)}, "resumed", window=expired.input_window)
        assert response(call).accepted
    assert len(gateway.samples) == 2


def test_only_eight_recent_windows_are_retained():
    gateway = Gateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        pipe.send({"input": common.Empty()}, "initial")
        initial = response(call).input_window
        for index in range(8):
            pipe.send({"input": common.Empty()}, f"input-{index}")
            assert response(call).accepted
        pipe.send({"velocity": pb.TeleopVelocity(vx_m_s=0.9, deadman=True)}, "evicted", window=initial)
        assert response(call).reason == "input_expired"
    assert gateway.claims == [] and gateway.samples == []


@pytest.mark.parametrize("sequence", [0, 1])
def test_zero_or_decreasing_sequence_cannot_send_velocity(sequence):
    gateway = Gateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        window, accepted = move(pipe, call)
        assert accepted.accepted
        pipe.send(
            {"velocity": pb.TeleopVelocity(vx_m_s=0.9, deadman=True)}, "old-sequence", window=window, sequence=sequence
        )
        rejected = response(call)
        assert rejected.sequence == sequence and rejected.reason == "stale_sequence"
    assert len(gateway.samples) == 1


def test_initial_handshake_requires_positive_sequence():
    gateway = Gateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        pipe.send({"input": common.Empty()}, "input", sequence=0)
        with pytest.raises(grpc.RpcError) as error:
            response(call)
        assert error.value.code() == grpc.StatusCode.INVALID_ARGUMENT
    assert gateway.claims == [] and gateway.samples == []


@pytest.mark.parametrize("action", ["hold", "close", "deadman"])
def test_pending_input_replaces_velocity_and_preserves_controls(action):
    stopped = threading.Event()
    pending = teleop._Pending(stopped)

    def velocity(sequence):
        return pb.TeleopRequest(sequence=sequence, velocity=pb.TeleopVelocity(deadman=True))

    pending.put(velocity(1))
    pending.put(velocity(2))
    assert pending.get(0).sequence == 2
    control = pb.TeleopRequest(sequence=3)
    if action == "deadman":
        control.velocity.CopyFrom(pb.TeleopVelocity(deadman=False))
    else:
        getattr(control, action).CopyFrom(common.Empty())
    pending.put(control)
    started = threading.Event()
    finished = threading.Event()

    def publish():
        started.set()
        pending.put(velocity(4))
        finished.set()

    writer = threading.Thread(target=publish)
    writer.start()
    try:
        assert started.wait(1)
        assert not finished.wait(0.02)
        assert pending.get(0).sequence == 3
        assert finished.wait(1)
        assert pending.get(0).sequence == 4
    finally:
        stopped.set()
        writer.join(1)
    assert not writer.is_alive()


class AuthorityGateway(Gateway):
    def __init__(self):
        super().__init__()
        self.authority = {
            "active_source_id": "", "active_source_epoch": 0, "has_active_authority": False,
            "holding": False, "authority_reason": "available", "received_age_ms": 1.0, "fresh": True,
        }
        self.status_reads = []
        self._nav_commands = SimpleNamespace(operator_motion_status=self._operator_status)

    def _operator_status(self, *, max_age_ms):
        self.status_reads.append(max_age_ms)
        return dict(self.authority)

    def _teleop_claim(self, **values):
        result = super()._teleop_claim(**values)
        self.authority.update(
            active_source_id=values["source_id"], active_source_epoch=values["source_epoch"],
            has_active_authority=True, authority_reason="claimed",
        )
        return result

    def _teleop_release(self, **values):
        result = super()._teleop_release(**values)
        if result.accepted:
            self.authority["holding"] = "release_sequence" not in values
            if "release_sequence" in values:
                self.authority.update(active_source_id="", active_source_epoch=0, has_active_authority=False)
        return result


def test_explicit_claim_returns_this_session_control_without_velocity():
    gateway = AuthorityGateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        pipe.send({"input": common.Empty()}, "input")
        initial = response(call)
        assert initial.control.available and initial.control.held_by_client is False
        assert gateway.claims == []
        pipe.send({"claim": common.Empty()}, "claim")
        claimed = response(call)
        assert claimed.accepted and claimed.reason == "claimed"
        assert not claimed.ingress_accepted and not claimed.final_output_confirmed
        assert claimed.control.held_by_client and not claimed.control.available
        native_claim = gateway.claims[0]
        assert claimed.control.owner == native_claim["source_id"]
        assert claimed.control.owner_epoch == native_claim["source_epoch"]
        assert gateway.samples == []
        pipe.send({"close": common.Empty()}, "close")
        closed = response(call)
        assert closed.accepted and closed.control.available
        assert closed.control.held_by_client is False
        assert "release_sequence" in gateway.releases[-1]
        assert gateway._web_teleop_owner is None
    assert len(gateway.claims) == 1 and gateway.samples == []


@pytest.mark.parametrize("native_ack", [None, False])
def test_explicit_claim_missing_ack_preserves_unknown_result(native_ack):
    gateway = AuthorityGateway()

    def claim(**values):
        gateway.claims.append(values)
        return native_ack

    gateway._teleop_claim = claim
    with serve(gateway) as connect:
        pipe, call = connect()
        pipe.send({"input": common.Empty()}, "input")
        response(call)
        pipe.send({"claim": common.Empty()}, "claim")
        with pytest.raises(grpc.RpcError) as failure:
            response(call)
        assert failure.value.code() == grpc.StatusCode.UNAVAILABLE
        assert "doso-error-bin" in dict(failure.value.trailing_metadata())
        assert len(gateway.claims) == 1 and gateway.samples == []


def test_periodic_control_feedback_does_not_claim_or_send_velocity():
    gateway = AuthorityGateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        pipe.send({"input": common.Empty()}, "input")
        assert response(call).control.available
        for _ in range(3):
            update = next(call)
            assert not update.request_id and update.sequence == 0
            assert update.control.fresh and update.control.available
            assert update.control.held_by_client is False
        assert gateway.status_reads and set(gateway.status_reads) == {250}
        assert gateway.claims == [] and gateway.samples == []


@pytest.mark.parametrize("loss", ["source", "epoch", "stale"])
def test_session_feedback_observes_lost_or_unknown_native_authority(loss):
    gateway = AuthorityGateway()
    with serve(gateway) as connect:
        pipe, call = connect()
        pipe.send({"input": common.Empty()}, "input")
        response(call)
        pipe.send({"claim": common.Empty()}, "claim")
        assert response(call).control.held_by_client
        if loss == "source":
            gateway.authority["active_source_id"] = "web:other"
        elif loss == "epoch":
            gateway.authority["active_source_epoch"] += 1
        else:
            gateway.authority.update(fresh=False, received_age_ms=500.0)
        for _ in range(20):
            state = next(call).control
            if not state.held_by_client:
                break
        assert not state.held_by_client
        if loss == "stale":
            assert not state.HasField("held_by_client") and not state.HasField("available")
        else:
            assert state.HasField("held_by_client") and state.available is False
        assert gateway.samples == [] and len(gateway.claims) == 1
