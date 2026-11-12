"""Navigation SDK options use the existing guarded owners over real local gRPC."""

import time
from types import SimpleNamespace
from unittest.mock import Mock

import grpc
import pytest

from gateway.gateway_module import GatewayModule
from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import navigation_pb2 as navigation_pb
from gateway.rpc.generated import navigation_pb2_grpc as navigation_rpc
from runtime.msgs import NavigationCommandKind, NavigationCommandReceipt
from tests.gateway.test_grpc_sdk import command, serve


def test_goal_options_reach_goal_owner():
    calls = []

    def submit_goal(goal, *, task_id, request_id, action, **limits):
        calls.append((goal, limits))
        return {"accepted": True, "task_id": task_id, "request_id": request_id,
                "native_ack": {"accepted": True}}

    gateway = GatewayModule()
    gateway._goals = SimpleNamespace(submit_goal=submit_goal)
    with serve(gateway) as channel:
        reply = navigation_rpc.NavigationStub(channel).GoWithOptions(navigation_pb.Goal(
            command=command("goal-limits"), pose=common_pb.Pose(x=1, y=2, z=3),
            max_speed_mps=0.8, acceptance_radius_m=0.25,
        ), timeout=3)
    assert reply.accepted and reply.task_id
    assert calls[0][1] == {"max_speed_mps": 0.8, "acceptance_radius_m": 0.25}
    assert calls[0][0].pose.position.z == 3


def test_goal_options_entry_requires_an_option_before_delivery():
    owner = Mock()
    gateway = GatewayModule()
    gateway._goals = SimpleNamespace(submit_goal=owner)
    with serve(gateway) as channel:
        with pytest.raises(grpc.RpcError) as rejected:
            navigation_rpc.NavigationStub(channel).GoWithOptions(navigation_pb.Goal(
                command=command("missing-options"), pose=common_pb.Pose(x=1),
            ), timeout=3)
    assert rejected.value.code() == grpc.StatusCode.INVALID_ARGUMENT
    owner.assert_not_called()


@pytest.mark.parametrize("field,value", [
    ("max_speed_mps", 0), ("max_speed_mps", -1), ("max_speed_mps", 5.1),
    ("max_speed_mps", float("nan")), ("max_speed_mps", float("inf")),
    ("acceptance_radius_m", 0), ("acceptance_radius_m", 20.1),
])
def test_invalid_goal_options_do_not_reach_owner(field, value):
    owner = Mock()
    gateway = GatewayModule()
    gateway._goals = SimpleNamespace(submit_goal=owner)
    with serve(gateway) as channel:
        with pytest.raises(grpc.RpcError) as rejected:
            navigation_rpc.NavigationStub(channel).GoWithOptions(navigation_pb.Goal(
                command=command("invalid-goal"), pose=common_pb.Pose(x=1), **{field: value},
            ), timeout=3)
    assert rejected.value.code() == grpc.StatusCode.INVALID_ARGUMENT
    owner.assert_not_called()


def test_preview_preserves_native_xyz_and_does_not_submit_motion():
    preview = Mock(return_value={
        "feasible": True, "start_valid": True, "start": {"x": 0, "y": 0, "z": 1},
        "goal": {"x": 3, "y": 4, "z": 2},
        "path": [{"x": 0, "y": 0, "z": 1}, {"x": 3, "y": 4, "z": 2}],
        "frame_id": "map", "elapsed_ms": 2.5, "planner": "native_global", "timestamp_s": 10,
    })
    motion = Mock(side_effect=AssertionError("preview submitted motion"))
    gateway = SimpleNamespace(_nav_commands=SimpleNamespace(preview_plan=preview), _run_control_command=motion)
    with serve(gateway) as channel:
        reply = navigation_rpc.NavigationStub(channel).PreviewPlan(navigation_pb.PlanQuery(
            pose=common_pb.Pose(x=3, y=4, z=2), acceptance_radius_m=0.25,
        ), timeout=3)
    assert reply.ok and reply.feasible
    assert reply.path.frame_id == "map" and reply.path.points[-1].z == 2
    assert reply.start.z == 1 and reply.goal.z == 2
    assert reply.plan_ms == 2.5 and reply.distance_m == pytest.approx(26 ** 0.5)
    preview.assert_called_once_with(3.0, 4.0, 2.0, acceptance_radius_m=0.25)
    motion.assert_not_called()


def test_preview_unavailable_is_not_reported_as_success():
    with serve(SimpleNamespace()) as channel:
        reply = navigation_rpc.NavigationStub(channel).PreviewPlan(navigation_pb.PlanQuery(
            pose=common_pb.Pose(x=1),
        ), timeout=3)
    assert not reply.ok and not reply.feasible
    assert reply.reasons == ["native navigation plan preview is unavailable"]
    assert not reply.HasField("distance_m")


@pytest.mark.parametrize("speed", [0, -1, float("nan"), float("inf")])
def test_invalid_task_speed_does_not_reach_owner(speed):
    owner = Mock()
    gateway = GatewayModule()
    gateway._nav_commands = SimpleNamespace(set_task_speed_limit=owner)
    with serve(gateway) as channel:
        with pytest.raises(grpc.RpcError) as rejected:
            navigation_rpc.NavigationStub(channel).SetSpeedLimit(navigation_pb.TaskSpeedLimit(
                command=command("invalid-speed"), task_id="task-1", max_speed_mps=speed,
            ), timeout=3)
    assert rejected.value.code() == grpc.StatusCode.INVALID_ARGUMENT
    owner.assert_not_called()


@pytest.mark.parametrize("wrong_identity", [False, True])
def test_task_speed_uses_native_ack_identity_and_lease(wrong_identity):
    calls = []

    def set_speed(*, task_id, request_id, max_speed_mps):
        calls.append((task_id, request_id, max_speed_mps))
        return NavigationCommandReceipt(
            True, int(NavigationCommandKind.TASK_SPEED_LIMIT),
            "wrong-task" if wrong_identity else task_id,
            request_id, time.time(), "task_speed_limit_updated",
        )

    gateway = GatewayModule()
    gateway._nav_commands = SimpleNamespace(set_task_speed_limit=set_speed)
    with serve(gateway) as channel:
        stub = navigation_rpc.NavigationStub(channel)
        request = navigation_pb.TaskSpeedLimit(command=command("speed-1"), task_id="task-1", max_speed_mps=0.7)
        if wrong_identity:
            with pytest.raises(grpc.RpcError) as rejected:
                stub.SetSpeedLimit(request, timeout=3)
            assert rejected.value.code() == grpc.StatusCode.FAILED_PRECONDITION
        else:
            reply = stub.SetSpeedLimit(request, timeout=3)
            assert reply.accepted and reply.request_id == "speed-1" and reply.task_id == "task-1"
        assert gateway._lease.acquire("other-operator", 30)
        request.command.CopyFrom(command("speed-2"))
        with pytest.raises(grpc.RpcError) as denied:
            stub.SetSpeedLimit(request, timeout=3)
        assert denied.value.code() == grpc.StatusCode.PERMISSION_DENIED
    assert calls == [("task-1", "speed-1", 0.7)]
