"""Inspection gRPC uses the existing product transaction with local owner doubles."""

import threading

import grpc
import pytest
from google.protobuf.json_format import ParseDict
from google.protobuf.struct_pb2 import Struct

from gateway.rpc.generated import tasks_pb2_grpc as tasks_rpc
from gateway.rpc.generated.messages import inspection_pb2 as inspection_pb
from gateway.services.inspection_task_lifecycle import ensure_inspection_task_timeline
from tests.gateway.test_gateway_inspection_routes import _client, _route, _task_event
from tests.gateway.test_grpc_sdk import command, error_detail, serve


def binding(session="inspection-component-session", env="sim"):
    return inspection_pb.InspectionBinding(product_session_id=session, env=env)


def put(stub, route=None):
    return stub.PutInspectionRoute(inspection_pb.PutInspectionRoute(
        command=command("route-put"), route=ParseDict(route or _route(), Struct()),
    ), timeout=3)


def start_request(**changes):
    return inspection_pb.StartInspection(**{
        "command": command("inspection-start"), "map_id": "field-map", "route_id": "route-a",
        "revision": 1, "binding": binding(), **changes,
    })


def test_route_crud_uses_existing_store_without_starting_motion(monkeypatch):
    client, native = _client(monkeypatch)
    gateway = client.app.state.gateway
    with client, serve(gateway) as channel:
        stub = tasks_rpc.TasksStub(channel)
        stored = put(stub)
        assert (stored.map_id, stored.route_id, stored.revision, stored.request_id) == (
            "field-map", "route-a", 1, "route-put",
        )
        query = inspection_pb.InspectionRouteQuery(map_id="field-map", route_id="route-a")
        routes = stub.InspectionRoutes(query, timeout=3)
        assert len(routes.routes) == 1 and routes.routes[0].route_id == "route-a"
        assert stub.InspectionRoute(query, timeout=3).details.fields["map_content_epoch"].number_value == 3
        deleted = stub.DeleteInspectionRoute(inspection_pb.DeleteInspectionRoute(
            command=command("route-delete"), map_id="field-map", route_id="route-a",
        ), timeout=3)
        assert deleted.ok and deleted.accepted and deleted.request_id == "route-delete"
        assert not stub.InspectionRoutes(query, timeout=3).routes
    assert native.calls == [] and gateway._recording.starts == []


def test_start_is_admission_only_then_controls_use_same_task_and_session(monkeypatch):
    client, native = _client(monkeypatch)
    gateway = client.app.state.gateway
    with client, serve(gateway) as channel:
        stub = tasks_rpc.TasksStub(channel)
        put(stub)
        admitted = stub.StartInspection(start_request(), timeout=3)
        assert admitted.ok and admitted.accepted and admitted.task_id
        assert admitted.request_id == "inspection-start"
        query = inspection_pb.InspectionTaskQuery(task_id=admitted.task_id)
        state = stub.InspectionTask(query, timeout=3)
        assert state.found and not state.execution_confirmed and not state.terminal
        assert not state.current_state and not state.state_available
        for name in ("PauseInspection", "ResumeInspection", "CancelInspection"):
            receipt = getattr(stub, name)(inspection_pb.InspectionControl(
                command=command(name), task_id=admitted.task_id, reason="operator", binding=binding(),
            ), timeout=3)
            assert receipt.ok and receipt.task_id == admitted.task_id and receipt.request_id == name
    assert [call[0] for call in native.calls] == ["task_start", "task_pause", "task_resume", "task_cancel"]
    assert len(gateway._recording.starts) == 1


@pytest.mark.parametrize("change,error", [
    ({"map_id": "other-map"}, "inspection_active_map_mismatch"),
    ({"revision": 2}, "inspection_route_revision_mismatch"),
    ({"binding": binding(session="other-session")}, "inspection_product_session_mismatch"),
    ({"binding": binding(env="real")}, "inspection_expected_env_mismatch"),
])
def test_start_rejects_existing_map_revision_and_runtime_conflicts(monkeypatch, change, error):
    client, native = _client(monkeypatch)
    gateway = client.app.state.gateway
    with client, serve(gateway) as channel:
        stub = tasks_rpc.TasksStub(channel)
        put(stub)
        with pytest.raises(grpc.RpcError) as rejected:
            stub.StartInspection(start_request(**change), timeout=3)
        assert rejected.value.code() == grpc.StatusCode.FAILED_PRECONDITION
        assert error_detail(rejected.value).details.fields["error"].string_value == error
    assert native.calls == [] and gateway._recording.starts == []


def test_native_rejection_keeps_recording_rollback(monkeypatch):
    client, native = _client(monkeypatch)
    gateway = client.app.state.gateway
    native.accepted = False
    with client, serve(gateway) as channel:
        stub = tasks_rpc.TasksStub(channel)
        put(stub)
        with pytest.raises(grpc.RpcError) as rejected:
            stub.StartInspection(start_request(), timeout=3)
        assert rejected.value.code() == grpc.StatusCode.FAILED_PRECONDITION
        details = error_detail(rejected.value).details
        assert details.fields["error"].string_value == "inspection_task_rejected"
    assert len(gateway._recording.starts) == 1 and len(gateway._recording.stops) == 1
    assert gateway._recording.stops[0]["expected_session_id"].startswith("recording-")


def test_invalid_binding_and_route_epoch_do_not_start_owner(monkeypatch):
    client, native = _client(monkeypatch)
    gateway = client.app.state.gateway
    with client, serve(gateway) as channel:
        stub = tasks_rpc.TasksStub(channel)
        with pytest.raises(grpc.RpcError) as rejected:
            stub.StartInspection(start_request(binding=inspection_pb.InspectionBinding()), timeout=3)
        assert rejected.value.code() == grpc.StatusCode.INVALID_ARGUMENT
        route = {**_route(), "map_content_epoch": 4}
        with pytest.raises(grpc.RpcError) as rejected:
            put(stub, route)
        assert rejected.value.code() == grpc.StatusCode.FAILED_PRECONDITION
    assert native.calls == [] and gateway._recording.starts == []


def test_watch_ends_only_on_native_terminal_fact_and_cancellation_does_not_cancel_task(monkeypatch):
    client, native = _client(monkeypatch)
    gateway = client.app.state.gateway
    timeline = ensure_inspection_task_timeline(gateway)
    with client, serve(gateway) as channel:
        stub = tasks_rpc.TasksStub(channel)
        put(stub)
        admitted = stub.StartInspection(start_request(), timeout=3)
        query = inspection_pb.InspectionTaskQuery(task_id=admitted.task_id)
        stream = stub.WatchInspection(query, timeout=3)
        for _ in range(3):
            state = next(stream)
            assert state.task_id == admitted.task_id and not state.terminal
        stream.cancel()
        assert [call[0] for call in native.calls] == ["task_start"]
        event = _task_event(task_id=admitted.task_id, sequence=1, state=7)
        event["route_revision"] = 1
        assert timeline.observe(event)
        updates = list(stub.WatchInspection(query, timeout=3))
        assert len(updates) == 1 and updates[0].terminal
        assert updates[0].current_state == "SUCCESS" and updates[0].execution_confirmed
        details = updates[0].details.fields
        assert "boot_id" not in details["delivery"].struct_value.fields
        for public_event in [details["latest_event"], *details["timeline"].list_value.values]:
            assert "boot_id" not in public_event.struct_value.fields
            assert "event_id" not in public_event.struct_value.fields
        assert timeline.query(admitted.task_id)["latest_event"]["boot_id"] == event["boot_id"]
        report = stub.InspectionReport(query, timeout=3)
        assert report.fields.fields["task_id"].string_value == admitted.task_id


def test_resume_obeys_safety_and_control_obeys_bound_product_session(monkeypatch):
    client, native = _client(monkeypatch)
    gateway = client.app.state.gateway
    with client, serve(gateway) as channel:
        stub = tasks_rpc.TasksStub(channel)
        put(stub)
        admitted = stub.StartInspection(start_request(), timeout=3)
        request = inspection_pb.InspectionControl(
            command=command("resume"), task_id=admitted.task_id, reason="operator", binding=binding(session="other-session"),
        )
        with pytest.raises(grpc.RpcError) as rejected:
            stub.ResumeInspection(request, timeout=3)
        assert rejected.value.code() == grpc.StatusCode.FAILED_PRECONDITION
        gateway._state_lock = threading.Lock()
        gateway._navigation_state = {"authority": "estop"}
        request.binding.CopyFrom(binding())
        with pytest.raises(grpc.RpcError) as rejected:
            stub.ResumeInspection(request, timeout=3)
        assert rejected.value.code() == grpc.StatusCode.FAILED_PRECONDITION
    assert [call[0] for call in native.calls] == ["task_start"]


def test_task_list_filters_retained_native_facts_and_includes_terminal_history(monkeypatch):
    client, native = _client(monkeypatch)
    gateway = client.app.state.gateway
    timeline = ensure_inspection_task_timeline(gateway)
    for sequence, task_id, state, changes in [
        (1, "active-a", 3, {}),
        (2, "done-a", 7, {}),
        (3, "active-b", 3, {"route_id": "route-b"}),
        (4, "other-map", 3, {"map_id": "other-map"}),
    ]:
        event = _task_event(task_id=task_id, sequence=sequence, state=state)
        event.update(changes)
        assert timeline.observe(event)
    with client, serve(gateway) as channel:
        stub = tasks_rpc.TasksStub(channel)
        query = inspection_pb.InspectionTaskListQuery(map_id="field-map", route_id="route-a")
        active = stub.InspectionTasks(query, timeout=3)
        assert active.count == 1 and [task.task_id for task in active.tasks] == ["active-a"]
        assert active.retention == "process_local_gateway_projection" and active.observed_at_unix_s > 0
        query.include_terminal = True
        history = stub.InspectionTasks(query, timeout=3)
        assert history.count == 2 and [task.task_id for task in history.tasks] == ["done-a", "active-a"]
        assert history.tasks[0].terminal and history.tasks[0].current_state == "SUCCESS"
        assert not history.tasks[1].terminal and history.tasks[1].execution_confirmed
        query.limit = 0
        assert [task.task_id for task in stub.InspectionTasks(query, timeout=3).tasks] == ["done-a"]
        all_tasks = stub.InspectionTasks(inspection_pb.InspectionTaskListQuery(include_terminal=True), timeout=3)
        assert all_tasks.count == 4
        query.map_id = " "
        with pytest.raises(grpc.RpcError) as rejected:
            stub.InspectionTasks(query, timeout=3)
        assert rejected.value.code() == grpc.StatusCode.INVALID_ARGUMENT
    assert native.calls == [] and gateway._recording.starts == []
