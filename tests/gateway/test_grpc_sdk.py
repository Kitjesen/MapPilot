"""Exercise the SDK transport against real Gateway operations, without hardware."""

import json
import socket
import threading
import time
from contextlib import contextmanager
from types import SimpleNamespace
from unittest.mock import Mock

import grpc
import pytest
from fastapi import HTTPException

from gateway.gateway_module import GatewayModule
from gateway.maps import routes as map_routes
from gateway.routes import operations
from gateway.rpc import codec, services
from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import error_pb2 as error_pb
from gateway.rpc.generated import localization_pb2 as localization_pb
from gateway.rpc.generated import localization_pb2_grpc as localization_rpc
from gateway.rpc.generated import maps_pb2 as maps_pb
from gateway.rpc.generated import maps_pb2_grpc as maps_rpc
from gateway.rpc.generated import motion_pb2 as motion_pb
from gateway.rpc.generated import motion_pb2_grpc as motion_rpc
from gateway.rpc.generated import navigation_pb2 as navigation_pb
from gateway.rpc.generated import navigation_pb2_grpc as navigation_rpc
from gateway.rpc.generated import system_pb2 as system_pb
from gateway.rpc.generated import system_pb2_grpc as system_rpc
from gateway.rpc.server import start
from runtime import config as runtime_config


@pytest.fixture(autouse=True)
def clear_device_id(monkeypatch):
    monkeypatch.delenv("DOSO_DEVICE_ID", raising=False)


@contextmanager
def serve(gateway):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    address = f"127.0.0.1:{port}"
    server = start(gateway, address)
    channel = grpc.insecure_channel(address)
    try:
        grpc.channel_ready_future(channel).result(timeout=3)
        yield channel
    finally:
        channel.close()
        server.stop(0).wait(timeout=3)


def command(request_id):
    return common_pb.Command(request_id=request_id, client_id="sdk-test")


def error_detail(error):
    metadata = dict(error.trailing_metadata())
    return error_pb.ErrorDetail.FromString(metadata["doso-error-bin"])


def test_ping():
    with serve(SimpleNamespace()) as channel:
        assert system_rpc.SystemStub(channel).Ping(common_pb.Empty(), timeout=3).status == "ok"


def test_handshake_configuration_identity(monkeypatch, tmp_path):
    robot_id = "8f7e1c62-0a4d-4b89-96e2-6c47b03db1a5"
    monkeypatch.setenv("DOSO_DEVICE_ID", robot_id)
    monkeypatch.setenv("LINGTU_ROBOT", "unitree/go2")
    config = Mock(return_value=SimpleNamespace(raw={
        "robot": {"id": "configured-7", "firmware_version": "old-config-version"},
        "camera": {"serial_number": "camera-serial"},
    }))
    monkeypatch.setattr(runtime_config, "get_config", config)
    metadata = tmp_path / "metadata.json"
    metadata.write_text(json.dumps({"version": "v2.3.0-test.1"}), encoding="utf-8")
    monkeypatch.setattr(services, "_RELEASE_METADATA", metadata)
    control = Mock(side_effect=AssertionError("handshake invoked control"))
    gateway = SimpleNamespace(
        _compiled_run_plan=SimpleNamespace(robot="doso/thunder_v4"),
        _compiled_env="sim",
        _run_control_command=control,
    )
    with serve(gateway) as channel:
        reply = system_rpc.SystemStub(channel).Handshake(
            system_pb.Handshake(protocol_version=2, client_id="sdk-test", sdk_version="test-sdk"), timeout=3,
        )
    assert reply.protocol_version == 2
    assert reply.HasField("info")
    info = reply.info
    assert info.robot_id == robot_id
    assert info.robot_version == "doso/thunder_v4"
    assert info.firmware_version == "v2.3.0-test.1"
    config.assert_not_called()
    control.assert_not_called()


@pytest.mark.parametrize("protocol, client_id, code, reason", [
    (0, "sdk-test", grpc.StatusCode.FAILED_PRECONDITION, "protocol_mismatch"),
    (1, "sdk-test", grpc.StatusCode.FAILED_PRECONDITION, "protocol_mismatch"),
    (2, "  ", grpc.StatusCode.INVALID_ARGUMENT, "invalid_client_id"),
])
def test_handshake_invalid_request(monkeypatch, protocol, client_id, code, reason):
    monkeypatch.setenv("DOSO_DEVICE_ID", "invalid-server-configuration")
    with serve(SimpleNamespace()) as channel:
        with pytest.raises(grpc.RpcError) as error:
            system_rpc.SystemStub(channel).Handshake(
                system_pb.Handshake(protocol_version=protocol, client_id=client_id, sdk_version="test-sdk"), timeout=3,
            )
    assert error.value.code() == code
    detail = error_detail(error.value)
    assert detail.reason_code == reason
    if reason == "protocol_mismatch":
        assert detail.details.fields["supported_protocol"].number_value == 2
        assert detail.details.fields["requested_protocol"].number_value == protocol


@pytest.mark.parametrize("robot_id", [None, ""])
def test_handshake_missing_identity(monkeypatch, tmp_path, robot_id):
    if robot_id is not None:
        monkeypatch.setenv("DOSO_DEVICE_ID", robot_id)
    monkeypatch.delenv("LINGTU_ROBOT", raising=False)

    monkeypatch.setattr(services, "_RELEASE_METADATA", tmp_path / "metadata.json")
    with serve(SimpleNamespace()) as channel:
        reply = system_rpc.SystemStub(channel).Handshake(
            system_pb.Handshake(protocol_version=2, client_id="sdk-test"), timeout=3,
        )
    assert reply.HasField("info")
    for field in ("robot_id", "robot_version", "firmware_version"):
        assert not reply.info.HasField(field)


@pytest.mark.parametrize("metadata", [{}, {"version": 3}])
def test_handshake_without_release_version(monkeypatch, tmp_path, metadata):
    path = tmp_path / "metadata.json"
    path.write_text(json.dumps(metadata), encoding="utf-8")
    monkeypatch.setattr(services, "_RELEASE_METADATA", path)
    with serve(SimpleNamespace()) as channel:
        reply = system_rpc.SystemStub(channel).Handshake(
            system_pb.Handshake(protocol_version=2, client_id="sdk-test"), timeout=3,
        )
    assert not reply.info.HasField("firmware_version")


@pytest.mark.parametrize("robot_id", [
    "thunder_v4_001",
    "6ba7b810-9dad-11d1-80b4-00c04fd430c8",
    "8F7E1C62-0A4D-4B89-96E2-6C47B03DB1A5",
    "8f7e1c620a4d4b8996e26c47b03db1a5",
])
def test_handshake_invalid_robot_id(monkeypatch, robot_id):
    monkeypatch.setenv("DOSO_DEVICE_ID", robot_id)
    with serve(SimpleNamespace()) as channel:
        with pytest.raises(grpc.RpcError) as error:
            system_rpc.SystemStub(channel).Handshake(
                system_pb.Handshake(protocol_version=2, client_id="sdk-test"), timeout=3,
            )
    assert error.value.code() == grpc.StatusCode.FAILED_PRECONDITION
    assert error_detail(error.value).reason_code == "invalid_robot_id"


def test_maps_save_list_operation(monkeypatch):
    calls = []

    def mapd(gateway, request):
        calls.append(request)
        if request["action"] == "list_maps":
            return {
                "success": True,
                "active": "lab",
                "maps": [{"name": "lab", "can_activate": True, "has_pcd": True, "has_octomap": True}],
            }
        if request["action"] == "save_map":
            return {
                "success": False,
                "accepted": True,
                "status": "running",
                "request_id": request["request_id"],
                "job_id": "save_job_1",
            }
        assert request == {"action": "get_save_map_status", "job_id": "save_job_1"}
        return {"success": True, "status": {"job_id": "save_job_1", "state": "SUCCEEDED"}}

    monkeypatch.setattr(map_routes, "_mapd_http_request", mapd)
    gateway = SimpleNamespace(_get_slam_profile=lambda: "native_dds")
    with serve(gateway) as channel:
        maps = maps_rpc.MapsStub(channel)
        listed = maps.List(common_pb.Empty(), timeout=3)
        assert listed.active == "lab"
        assert [(item.name, item.can_activate, item.has_octomap) for item in listed.maps] == [("lab", True, True)]
        saved = maps.Save(maps_pb.SaveMap(command=command("save-1"), name="lab"), timeout=3)
        assert saved.ok and saved.accepted
        assert saved.operation_id == "save_job_1"
        assert saved.request_id == "save-1"
        completed = maps.Operation(maps_pb.MapQuery(operation_id=saved.operation_id), timeout=3)
        assert completed.operation_id == saved.operation_id
        assert completed.state == maps_pb.MAP_SUCCEEDED
    assert calls[1] == {"action": "save_map", "map_id": "lab", "slam_profile": "native_dds", "request_id": "save-1"}


@pytest.mark.parametrize("method, query", [
    ("List", common_pb.Empty()),
    ("Operation", maps_pb.MapQuery(operation_id="save_job_1")),
])
def test_mapd_unavailable(monkeypatch, method, query):
    def unavailable(gateway, command):
        raise RuntimeError("mapd unavailable")

    monkeypatch.setattr(map_routes, "mapd_request", unavailable)
    with serve(SimpleNamespace()) as channel:
        with pytest.raises(grpc.RpcError) as error:
            getattr(maps_rpc.MapsStub(channel), method)(query, timeout=3)
        assert error.value.code() == grpc.StatusCode.UNAVAILABLE
        assert error.value.details() == "mapd unavailable"


def test_relocalization_and_invalid_pose(monkeypatch):
    calls = []
    result = SimpleNamespace(success=True, timed_out=False, message="", quality=0.7, details={})

    def seeded(map_id, x, y, yaw, *, z, timeout_s):
        calls.append((map_id, x, y, z, yaw, timeout_s))
        return result

    def global_search(*, timeout_s):
        calls.append(("global", timeout_s))
        return result

    def mapd(gateway, request):
        assert request == {"action": "get_record", "map_id": "lab"}
        return {"success": True}

    monkeypatch.setattr(operations, "mapd_request", mapd)
    gateway = SimpleNamespace(localization=SimpleNamespace(
        available=True, relocalize_saved_map=seeded, trigger_global_relocalize=global_search,
    ))
    with serve(gateway) as channel:
        localization = localization_rpc.LocalizationStub(channel)
        reply = localization.Relocalize(localization_pb.Relocalize(
            command=command("reloc-1"), map_name="lab", initial_pose=common_pb.Pose(x=1, y=2, z=3, yaw=0.4),
        ), timeout=3)
        assert reply.ok and reply.request_id == "reloc-1"
        assert localization.Relocalize(
            localization_pb.Relocalize(command=command("reloc-2"), map_name="lab"), timeout=3,
        ).ok
        with pytest.raises(grpc.RpcError) as invalid:
            localization.Relocalize(localization_pb.Relocalize(
                command=command("reloc-3"), map_name="lab", initial_pose=common_pb.Pose(x=float("nan")),
            ), timeout=3)
        assert invalid.value.code() == grpc.StatusCode.INVALID_ARGUMENT
    assert calls == [("lab", 1, 2, 3, 0.4, 30), ("global", 45)]


def test_pose_unavailable():
    with serve(GatewayModule()) as channel:
        with pytest.raises(grpc.RpcError) as unavailable:
            localization_rpc.LocalizationStub(channel).Pose(common_pb.Empty(), timeout=3)
        assert unavailable.value.code() == grpc.StatusCode.UNAVAILABLE


def test_navigation_goal_cancel_and_watch():
    gateway = GatewayModule()
    calls = []

    def submit_goal(goal, *, task_id, request_id, action, **limits):
        assert action == "goal"
        calls.append(("goal", goal, task_id, request_id))
        return {"accepted": True, "task_id": task_id, "request_id": request_id, "native_ack": {"accepted": True}}

    def submit_cancel(reason, *, task_id, request_id):
        calls.append(("cancel", reason, task_id, request_id))
        return {"accepted": True, "task_id": task_id, "request_id": request_id}

    gateway._goals = SimpleNamespace(submit_goal=submit_goal, submit_cancel=submit_cancel)
    with serve(gateway) as channel:
        navigation = navigation_rpc.NavigationStub(channel)
        reply = navigation.Go(
            navigation_pb.Goal(command=command("goal-1"), pose=common_pb.Pose(x=1, y=2, z=3, yaw=0.4)), timeout=3,
        )
        assert reply.ok and reply.accepted and reply.task_id
        goal = calls[0][1]
        assert (goal.pose.position.x, goal.pose.position.y, goal.pose.position.z) == (1, 2, 3)
        assert calls[0][2:] == (reply.task_id, "goal-1")
        status = {"task_id": reply.task_id, "request_id": "goal-1", "phase": "PLANNING"}
        with gateway._state_lock:
            gateway._navigation_goal_status_by_task[reply.task_id] = status
        watching = navigation.Watch(navigation_pb.TaskQuery(task_id=reply.task_id), timeout=3)
        assert next(watching).state == navigation_pb.TASK_PLANNING
        with gateway._state_lock:
            gateway._navigation_goal_status_by_task[reply.task_id] = {**status, "phase": "REACHED"}
        remaining = list(watching)
        assert remaining[-1].found and remaining[-1].state == navigation_pb.TASK_SUCCESS
        assert remaining[-1].details["source"] == "live_gateway_cache"
        cancelled = navigation.Cancel(navigation_pb.CancelTask(
            command=command("cancel-1"), task_id=reply.task_id, reason="operator",
        ), timeout=3)
        assert cancelled.ok and cancelled.accepted and cancelled.task_id == reply.task_id
        assert calls[1] == ("cancel", "operator", reply.task_id, "cancel-1")
        assert gateway._lease.acquire("another-operator", 10)
        with pytest.raises(grpc.RpcError) as denied:
            navigation.Go(navigation_pb.Goal(command=command("goal-2"), pose=common_pb.Pose(x=4)), timeout=3)
        assert denied.value.code() == grpc.StatusCode.PERMISSION_DENIED
        detail = error_detail(denied.value)
        assert detail.reason_code == "control_lease"
        assert detail.details["command"]["request_id"] == "goal-2"
        assert detail.details["command"]["accepted"] is False
        assert detail.details["detail"]["lease"]["holder"] == "another-operator"
        assert len(calls) == 2


def test_unconfirmed_native_goal_preserves_identity():
    gateway = GatewayModule()

    def submit_goal(goal, *, task_id, request_id, **kwargs):
        return {"accepted": False, "admission_unconfirmed": True, "task_id": task_id, "request_id": request_id}

    gateway._goals = SimpleNamespace(submit_goal=submit_goal)
    with serve(gateway) as channel:
        with pytest.raises(grpc.RpcError) as error:
            navigation_rpc.NavigationStub(channel).Go(
                navigation_pb.Goal(command=command("unknown-1"), pose=common_pb.Pose(x=1)), timeout=3,
            )
        assert error.value.code() == grpc.StatusCode.UNAVAILABLE
        detail = error_detail(error.value)
        assert detail.reason_code == "native_command_unconfirmed"
        assert detail.details["detail"]["request_id"] == "unknown-1"
        assert detail.details["detail"]["task_id"].startswith("nav-task-")


def test_http_exception_preserves_owner_detail(monkeypatch):
    original = {"reason_code": "map_service_unavailable", "message": "Map owner unavailable", "operation_id": "save-1"}

    def unavailable(gateway):
        raise HTTPException(status_code=503, detail=original)

    monkeypatch.setattr(services, "list_maps", unavailable)
    with serve(SimpleNamespace()) as channel:
        with pytest.raises(grpc.RpcError) as error:
            maps_rpc.MapsStub(channel).List(common_pb.Empty(), timeout=3)
        assert error.value.code() == grpc.StatusCode.UNAVAILABLE
        assert error.value.details() == "Map owner unavailable"
        detail = error_detail(error.value)
        assert detail.reason_code == original["reason_code"]
        assert dict(detail.details["detail"]) == original


@pytest.mark.parametrize("payload, ok, accepted", [
    ({"accepted": True, "success": False}, True, True),
    ({"accepted": False, "success": True}, False, False),
    ({"success": True}, True, None),
    ({"ok": False, "success": True}, False, None),
])
def test_admission_is_independent_of_execution(monkeypatch, payload, ok, accepted):
    value = {**payload, "request_id": "admission-1", "task_id": "task-1", "reason_code": "owner_reason"}
    monkeypatch.setattr(services, "go", lambda gateway, body: value)
    with serve(SimpleNamespace()) as channel:
        reply = navigation_rpc.NavigationStub(channel).Go(
            navigation_pb.Goal(command=command("admission-1"), pose=common_pb.Pose(x=1)), timeout=3,
        )
        assert reply.ok is ok
        assert reply.HasField("accepted") is (accepted is not None)
        if accepted is not None:
            assert reply.accepted is accepted
        assert reply.reason_code == "owner_reason"
        assert "reason_code" in reply.details


def test_conflicting_admission_is_reported(monkeypatch):
    value = {"accepted": True, "command": {"request_id": "conflict-1", "accepted": False}}
    monkeypatch.setattr(services, "go", lambda gateway, body: value)
    with serve(SimpleNamespace()) as channel:
        with pytest.raises(grpc.RpcError) as error:
            navigation_rpc.NavigationStub(channel).Go(
                navigation_pb.Goal(command=command("conflict-1"), pose=common_pb.Pose(x=1)), timeout=3,
            )
        assert error.value.code() == grpc.StatusCode.INTERNAL
        assert error.value.details() == "contradictory command admission"
        detail = error_detail(error.value)
        assert detail.details["accepted"] is True
        assert detail.details["command"]["accepted"] is False


def test_terminal_reason_codes_preserve_owner_values():
    task = codec.task({
        "task_id": "task-1", "found": True,
        "status": {"state_name": "FAILED", "reason_code": "planner_failure", "failure_reason": "No route"},
    })
    assert (task.state, task.reason_code, task.reason) == (navigation_pb.TASK_FAILED, "planner_failure", "No route")
    operation = codec.map_operation({
        "operation": {"operation_id": "save-1", "state": "FAILED", "reason_code": "disk_full", "message": "No space"},
    })
    assert (operation.state, operation.reason_code, operation.reason) == (maps_pb.MAP_FAILED, "disk_full", "No space")
    product = codec.product_operation({
        "request_id": "switch-1", "state": "interrupted", "message": "Switch interrupted",
        "result": {"reason_code": "owner_interrupted"},
    })
    assert product.state == maps_pb.PRODUCT_INTERRUPTED
    assert product.reason_code == "owner_interrupted"
    assert product.request_id == "switch-1"


@pytest.mark.parametrize("motion_watch_count", [0, 4, 8])
def test_watch_limit_preserves_commands_and_releases_slot(monkeypatch, motion_watch_count):
    gateway = GatewayModule()
    calls = []
    monkeypatch.setattr(services, "build_navigation_status", lambda gateway: {"state": "idle"})

    def estop(*, reason, request_id):
        calls.append("stop")
        return True

    gateway._nav_commands = SimpleNamespace(estop=estop)
    gateway._goals = SimpleNamespace(submit_cancel=lambda *args, **kwargs: calls.append("cancel"))
    gateway._navigation_goal_status_by_task["task-1"] = {"task_id": "task-1", "phase": "PLANNING"}
    watches = []
    with serve(gateway) as channel:
        navigation = navigation_rpc.NavigationStub(channel)
        motion = motion_rpc.MotionStub(channel)
        query = navigation_pb.TaskQuery(task_id="task-1")
        try:
            for index in range(8):
                if index >= motion_watch_count:
                    watching = navigation.Watch(query, timeout=10)
                elif index % 2 == 0:
                    watching = motion.WatchState(common_pb.Empty(), timeout=10)
                else:
                    watching = motion.WatchStateAt(motion_pb.WatchStateRequest(frequency_hz=2), timeout=10)
                watches.append(watching)
                state = next(watching)
                if index < motion_watch_count:
                    assert state.fields["state"] == "idle"
                else:
                    assert state.state == navigation_pb.TASK_PLANNING
            with pytest.raises(grpc.RpcError) as full:
                next(navigation.Watch(query, timeout=3))
            assert full.value.code() == grpc.StatusCode.RESOURCE_EXHAUSTED
            with pytest.raises(grpc.RpcError) as full:
                next(motion.WatchState(common_pb.Empty(), timeout=3))
            assert full.value.code() == grpc.StatusCode.RESOURCE_EXHAUSTED
            with pytest.raises(grpc.RpcError) as full:
                next(motion.WatchStateAt(motion_pb.WatchStateRequest(frequency_hz=1), timeout=3))
            assert full.value.code() == grpc.StatusCode.RESOURCE_EXHAUSTED
            assert system_rpc.SystemStub(channel).Ping(common_pb.Empty(), timeout=3).status == "ok"
            stopped = motion.Stop(command("stop-busy"), timeout=3)
            assert stopped.ok and stopped.accepted

            watches.pop().cancel()
            deadline = time.monotonic() + 3
            while True:
                replacement = navigation.Watch(query, timeout=3)
                try:
                    assert next(replacement).state == navigation_pb.TASK_PLANNING
                except grpc.RpcError as error:
                    assert error.code() == grpc.StatusCode.RESOURCE_EXHAUSTED
                    assert time.monotonic() < deadline, "cancelled watch did not release its slot"
                    time.sleep(0.01)
                else:
                    watches.append(replacement)
                    break
        finally:
            for watching in watches:
                watching.cancel()
    assert calls == ["stop"]


@pytest.mark.parametrize("frequency", [None, 2])
def test_motion_watch_reuses_status_and_delivers_updates(monkeypatch, frequency):
    state = {"state": "idle", "control": {"estop_latched": False}, "source": "owner_fixture"}
    monkeypatch.setattr(services, "build_navigation_status", lambda gateway: dict(state))
    control = Mock(side_effect=AssertionError("observation submitted a command"))
    with serve(SimpleNamespace(_run_control_command=control)) as channel:
        motion = motion_rpc.MotionStub(channel)
        watching = (
            motion.WatchState(common_pb.Empty(), timeout=3) if frequency is None else
            motion.WatchStateAt(motion_pb.WatchStateRequest(frequency_hz=frequency), timeout=3)
        )
        try:
            assert next(watching) == motion.Status(common_pb.Empty(), timeout=3)
            state["state"] = "estop"
            state["control"] = {"estop_latched": True}
            updated = next(watching)
            assert updated.fields["state"] == "estop"
            assert updated.fields["control"]["estop_latched"] is True
            assert updated.fields["source"] == "owner_fixture"
        finally:
            watching.cancel()
    control.assert_not_called()


@pytest.mark.parametrize("frequency", [None, 1, 2, 3, 4])
def test_motion_watch_uses_selected_interval(monkeypatch, frequency):
    waits = []

    def wait(interval):
        waits.append(interval)
        return True

    stopped = SimpleNamespace(set=lambda: None, wait=wait)
    slots = threading.BoundedSemaphore(1)
    motion = services.Motion(SimpleNamespace(), slots)
    monkeypatch.setattr(services, "threading", SimpleNamespace(Event=lambda: stopped))
    monkeypatch.setattr(services, "build_navigation_status", lambda gateway: {"state": "idle"})
    context = SimpleNamespace(add_callback=lambda callback: True, is_active=lambda: True)
    stream = (
        motion.WatchState(common_pb.Empty(), context) if frequency is None else
        motion.WatchStateAt(motion_pb.WatchStateRequest(frequency_hz=frequency), context)
    )
    assert len(list(stream)) == 1
    assert waits == [1.0 / (frequency or 4)]
    assert slots.acquire(blocking=False)


@pytest.mark.parametrize("frequency", [0, 5, 2**32 - 1])
def test_motion_watch_rejects_unsupported_frequency(monkeypatch, frequency):
    status = Mock(side_effect=AssertionError("invalid frequency read status"))
    monkeypatch.setattr(services, "build_navigation_status", status)
    with serve(SimpleNamespace()) as channel:
        with pytest.raises(grpc.RpcError) as error:
            next(motion_rpc.MotionStub(channel).WatchStateAt(
                motion_pb.WatchStateRequest(frequency_hz=frequency), timeout=3,
            ))
    assert error.value.code() == grpc.StatusCode.INVALID_ARGUMENT
    status.assert_not_called()


def test_motion_watch_preserves_status_failure(monkeypatch):
    def unavailable(gateway):
        raise HTTPException(status_code=503, detail={"reason_code": "motion_status_unavailable"})

    monkeypatch.setattr(services, "build_navigation_status", unavailable)
    with serve(SimpleNamespace()) as channel:
        with pytest.raises(grpc.RpcError) as error:
            next(motion_rpc.MotionStub(channel).WatchState(common_pb.Empty(), timeout=3))
    assert error.value.code() == grpc.StatusCode.UNAVAILABLE
    assert error_detail(error.value).reason_code == "motion_status_unavailable"


@pytest.mark.parametrize("accepted", [True, False])
def test_software_stop_ack(accepted):
    gateway = GatewayModule()
    calls = []

    def estop(*, reason, request_id):
        calls.append((reason, request_id))
        return accepted

    gateway._nav_commands = SimpleNamespace(estop=estop)
    with serve(gateway) as channel:
        motion = motion_rpc.MotionStub(channel)
        if accepted:
            reply = motion.Stop(command("stop-1"), timeout=3)
            assert reply.ok and reply.accepted
            assert reply.stage == "native_acknowledged"
        else:
            with pytest.raises(grpc.RpcError) as rejected:
                motion.Stop(command("stop-1"), timeout=3)
            assert rejected.value.code() == grpc.StatusCode.FAILED_PRECONDITION
    assert calls == [("rest_emergency_stop", "stop-1")]
