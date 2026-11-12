"""SDK task controls retain Web admission and motion-guard semantics."""

import socket
from contextlib import contextmanager

import grpc
import pytest

from gateway.gateway_module import GatewayModule
from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import navigation_pb2 as navigation_pb
from gateway.rpc.generated import navigation_pb2_grpc as navigation_rpc
from gateway.rpc.server import start


class Controls:
    def __init__(self):
        self.calls = []

    def submit_pause(self, reason, *, task_id, request_id):
        self.calls.append(("pause", task_id, request_id, reason))
        return {"accepted": True, "state": "pause_requested", "task_id": task_id, "request_id": request_id}

    def submit_resume(self, reason, *, task_id, request_id):
        self.calls.append(("resume", task_id, request_id, reason))
        return {"accepted": True, "state": "resume_requested", "task_id": task_id, "request_id": request_id}


@contextmanager
def serve(gateway):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        address = f"127.0.0.1:{listener.getsockname()[1]}"
    server = start(gateway, address)
    channel = grpc.insecure_channel(address)
    try:
        grpc.channel_ready_future(channel).result(timeout=3)
        yield navigation_rpc.NavigationStub(channel)
    finally:
        channel.close()
        server.stop(0).wait(3)


def request(action, *, task_id="task-1", request_id=None):
    return navigation_pb.TaskControl(
        command=common_pb.Command(client_id="sdk", request_id=request_id or f"{action}-1"),
        task_id=task_id, reason=f"operator_{action}",
    )


def gateway():
    robot = GatewayModule()
    robot.setup()
    controls = Controls()
    robot.on_system_modules({"nav.goals": controls})
    return robot, controls


@pytest.mark.parametrize("action", ["pause", "resume"])
def test_task_control_preserves_receipt_and_deduplicates(action):
    robot, controls = gateway()
    assert robot._lease.acquire("sdk", 30)
    with serve(robot) as stub:
        operation = getattr(stub, action.title())
        first = operation(request(action), timeout=3)
        repeated = operation(request(action), timeout=3)
    assert first.accepted and repeated.accepted
    assert first.request_id == f"{action}-1"
    assert first.task_id == "task-1"
    assert first.details["execution_confirmed"] is False
    assert len(controls.calls) == 1
    assert controls.calls[0] == (action, "task-1", f"{action}-1", f"operator_{action}")


def test_pause_available_and_resume_guarded_by_control_owner():
    robot, controls = gateway()
    assert robot._lease.acquire("another-client", 30)
    with serve(robot) as stub:
        assert stub.Pause(request("pause"), timeout=3).accepted
        with pytest.raises(grpc.RpcError) as error:
            stub.Resume(request("resume"), timeout=3)
        assert error.value.code() == grpc.StatusCode.PERMISSION_DENIED
    assert [call[0] for call in controls.calls] == ["pause"]


@pytest.mark.parametrize("action", ["pause", "resume"])
def test_task_control_rejects_conflicting_identity(action):
    robot, controls = gateway()
    with serve(robot) as stub:
        with pytest.raises(grpc.RpcError) as error:
            getattr(stub, action.title())(request(action, request_id="task-1"), timeout=3)
    assert error.value.code() == grpc.StatusCode.FAILED_PRECONDITION
    assert not controls.calls
