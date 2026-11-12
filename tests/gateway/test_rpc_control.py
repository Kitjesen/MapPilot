"""Control queries stay read-only and motion resume preserves native acknowledgement."""

import socket
import threading
import time
from contextlib import contextmanager
from types import SimpleNamespace

import grpc
import pytest

from gateway.gateway_module import GatewayModule
from gateway.rpc import control, services
from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import error_pb2 as error_pb
from gateway.rpc.generated import motion_pb2_grpc as rpc
from gateway.rpc.server import start
from gateway.services import resume as resume_service
from runtime.msgs import NavigationCommandKind


class Commands:
    def __init__(self):
        self.snapshot = {
            "active_source_id": "", "active_source_epoch": 0, "has_active_authority": False,
            "holding": False, "authority_reason": "available", "received_age_ms": 3.0, "fresh": True,
        }
        self.reads = []
        self.resumes = []
        self.outcome = "accepted"

    def operator_motion_status(self, *, max_age_ms):
        self.reads.append(max_age_ms)
        return self.snapshot

    def resume_autonomy_with_receipt(self, *, reason, request_id):
        self.resumes.append((reason, request_id))
        if self.outcome == "timeout":
            raise TimeoutError("native ACK timed out")
        if self.outcome == "missing":
            return None
        return {
            "accepted": self.outcome != "rejected",
            "kind": int(NavigationCommandKind.RESUME_AUTONOMY), "task_id": "",
            "request_id": "wrong-request" if self.outcome == "wrong_id" else request_id,
            "reason": "safety_stop" if self.outcome == "rejected" else "teleop_resume_ready_reassert_command",
            "endpoint_timestamp_s": time.time(),
        }


@pytest.fixture
def gateway(monkeypatch):
    robot = GatewayModule()
    robot.setup()
    robot._nav_commands = Commands()
    monkeypatch.setattr(resume_service, "native_motion_resume_context", lambda: {
        "status_fresh": False, "observed_control_mode": None, "resume_was_required": None,
        "goal_reissue_required": None, "fresh_operator_command_required": None,
    })
    return robot


@contextmanager
def serve(gateway):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        address = f"127.0.0.1:{listener.getsockname()[1]}"
    server = start(gateway, address)
    with grpc.insecure_channel(address) as channel:
        try:
            grpc.channel_ready_future(channel).result(timeout=3)
            yield rpc.MotionStub(channel)
        finally:
            server.stop(0).wait(3)


@pytest.mark.parametrize("occupied", [False, True])
def test_control_query_and_watch_are_read_only(gateway, occupied):
    gateway._nav_commands.snapshot.update(
        active_source_id="web:operator" if occupied else "",
        active_source_epoch=7 if occupied else 0, has_active_authority=occupied,
    )
    with serve(gateway) as client:
        state = client.ControlState(common_pb.Empty(), timeout=3)
        assert state.available is not occupied
        assert state.fresh and state.received_age_ms == 3.0
        assert not state.HasField("held_by_client")
        stream = client.WatchControlState(common_pb.Empty(), timeout=3)
        assert next(stream).available is not occupied
        assert next(stream).available is not occupied
        stream.cancel()
    assert gateway._nav_commands.resumes == []
    assert gateway._nav_commands.reads and set(gateway._nav_commands.reads) == {250}
    assert gateway._web_teleop_owner is None


@pytest.mark.parametrize(("owner", "epoch", "held"), [("sdk:session", 7, True), ("sdk:session", 8, False), ("web:other", 7, False)])
def test_session_ownership_requires_matching_source_and_epoch(gateway, owner, epoch, held):
    gateway._nav_commands.snapshot.update(active_source_id=owner, active_source_epoch=epoch, has_active_authority=True)
    result = control.state(gateway, ("sdk:session", 7))
    assert result.held_by_client is held
    assert result.HasField("held_by_client") and result.available is False


def test_stale_control_snapshot_keeps_ownership_unknown(gateway):
    gateway._nav_commands.snapshot.update(
        active_source_id="sdk:session", active_source_epoch=7, has_active_authority=True,
        holding=True, received_age_ms=600.0, fresh=False,
    )
    result = control.state(gateway, ("sdk:session", 7))
    assert result.fresh is False
    assert result.owner == "sdk:session" and result.owner_epoch == 7
    assert not result.HasField("available")
    assert not result.HasField("held_by_client")
    assert not result.HasField("holding")


@pytest.mark.parametrize("missing", [True, False])
def test_unavailable_control_source_keeps_optional_fields_absent(gateway, missing):
    if missing:
        gateway._nav_commands = SimpleNamespace()
    else:
        gateway._nav_commands.snapshot = None
    result = control.state(gateway)
    assert result.reason == "operator_status_unavailable"
    assert not result.HasField("available") and not result.HasField("held_by_client")
    assert result.fresh is False


def test_control_watch_uses_and_releases_shared_slot(gateway):
    slots = threading.BoundedSemaphore(1)
    service = services.Motion(gateway, slots)
    context = SimpleNamespace(add_callback=lambda _: True, is_active=lambda: True)
    stream = service.WatchControlState(common_pb.Empty(), context)
    assert next(stream).fresh
    assert not slots.acquire(blocking=False)
    stream.close()
    assert slots.acquire(blocking=False)


def command(request_id="resume-1", client_id="sdk"):
    return common_pb.Command(request_id=request_id, client_id=client_id)


def error_reason(error):
    metadata = dict(error.trailing_metadata())
    return error_pb.ErrorDetail.FromString(metadata["doso-error-bin"]).reason_code


def test_resume_preserves_native_ack_and_journal_replay(gateway):
    with serve(gateway) as client:
        first = client.Resume(command(), timeout=3)
        repeated = client.Resume(command(), timeout=3)
    assert first.accepted and repeated.accepted
    assert first.request_id == repeated.request_id == "resume-1"
    assert first.details["native_reason"] == "teleop_resume_ready_reassert_command"
    assert first.details["previous_motion_restored"] is False
    assert repeated.details["command"]["replay"] is True
    assert gateway._nav_commands.resumes == [("operator_resume", "resume-1")]


def test_resume_rejects_other_control_lease_without_dispatch(gateway):
    assert gateway._lease.acquire("other-client", 30)
    with serve(gateway) as client:
        with pytest.raises(grpc.RpcError) as error:
            client.Resume(command(), timeout=3)
    assert error.value.code() == grpc.StatusCode.PERMISSION_DENIED
    assert error_reason(error.value) == "control_lease"
    assert gateway._nav_commands.resumes == []


def test_resume_preserves_explicit_native_rejection(gateway):
    gateway._nav_commands.outcome = "rejected"
    with serve(gateway) as client:
        with pytest.raises(grpc.RpcError) as error:
            client.Resume(command(), timeout=3)
    assert error.value.code() == grpc.StatusCode.FAILED_PRECONDITION
    assert error_reason(error.value) == "native_command_rejected"
    assert len(gateway._nav_commands.resumes) == 1


@pytest.mark.parametrize("outcome", ["timeout", "missing", "wrong_id"])
def test_resume_unknown_ack_is_unavailable(gateway, outcome):
    gateway._nav_commands.outcome = outcome
    with serve(gateway) as client:
        with pytest.raises(grpc.RpcError) as error:
            client.Resume(command(), timeout=3)
    assert error.value.code() == grpc.StatusCode.UNAVAILABLE
    assert error_reason(error.value) == "motion_resume_ack_unknown"
    assert len(gateway._nav_commands.resumes) == 1
