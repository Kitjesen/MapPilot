"""Inspection RPCs share route transactions, journal and native task ownership."""

from types import SimpleNamespace

import grpc
from google.protobuf.json_format import MessageToDict

from gateway.routes import inspection as operations
from gateway.schemas import InspectionRouteRequest, InspectionTaskControlRequest, InspectionTaskStartRequest
from gateway.services.control_commands import ControlCommandService

from .codec import command, fields, invoke, model, receipt
from .generated import common_pb2 as common_pb
from .generated.messages import inspection_pb2 as inspection_pb


def _read(context, operation, *args, **kwargs):
    result = invoke(context, operation, *args, **kwargs)
    return result.model_dump(mode="json") if hasattr(result, "model_dump") else result


def _text(context, value, name):
    if not value.strip():
        context.abort(grpc.StatusCode.INVALID_ARGUMENT, f"{name} is required")
    return value


def _route(value, request_id=""):
    return inspection_pb.InspectionRoute(
        map_id=value.get("map_id") or "", route_id=value.get("id") or "",
        revision=value.get("revision") or 0, details=fields(value), request_id=request_id,
    )


def routes(gateway, request, context):
    map_id = _text(context, request.map_id, "map_id")
    result = _read(context, operations.list_routes, gateway, map_id)
    return inspection_pb.InspectionRoutes(map_id=result["map_id"], routes=[_route(route) for route in result["routes"]])


def route(gateway, request, context):
    map_id = _text(context, request.map_id, "map_id")
    route_id = _text(context, request.route_id, "route_id")
    return _route(_read(context, operations.get_route, gateway, route_id, map_id)["route"])


def put_route(gateway, request, context):
    metadata = command(context, request.command)
    if not request.HasField("route"):
        context.abort(grpc.StatusCode.INVALID_ARGUMENT, "route is required")
    value = MessageToDict(request.route)
    epoch = value.get("map_content_epoch")
    if isinstance(epoch, float) and epoch.is_integer():
        value["map_content_epoch"] = int(epoch)
    body = model(context, InspectionRouteRequest, **value)
    return _route(_read(context, operations.put_route, gateway, body)["route"], metadata["request_id"])


def delete_route(gateway, request, context):
    metadata = command(context, request.command)
    map_id = _text(context, request.map_id, "map_id")
    route_id = _text(context, request.route_id, "route_id")
    result = _read(context, operations.delete_route, gateway, route_id, map_id)
    return receipt(context, result, metadata["request_id"])


def _binding(context, binding):
    return {
        "expected_product_session_id": _text(context, binding.product_session_id, "product_session_id"),
        "expected_env": binding.env or None,
    }


def _guard(gateway, context, action, metadata, task_id=None):
    caller = SimpleNamespace(**metadata, task_id=task_id)
    rejected = ControlCommandService(gateway).motion_safety_rejection(action, caller)
    if rejected is not None:
        invoke(context, lambda: rejected)


def start(gateway, request, context):
    metadata = command(context, request.command)
    body = model(context, InspectionTaskStartRequest,
                 request_id=metadata["request_id"], route_id=request.route_id,
                 map_id=request.map_id or None, revision=request.revision,
                 **_binding(context, request.binding))
    _guard(gateway, context, "start_inspection_task", metadata)
    return receipt(context, _read(context, operations.start_task, gateway, body), metadata["request_id"])


def control(gateway, request, context, action):
    metadata = command(context, request.command)
    task_id = _text(context, request.task_id, "task_id")
    body = model(context, InspectionTaskControlRequest, reason=request.reason or f"operator_{action}",
                 request_id=metadata["request_id"], **_binding(context, request.binding))
    if action == "resume":
        _guard(gateway, context, "resume_inspection_task", metadata, task_id)
    else:
        rejected = ControlCommandService(gateway).control_lease_rejection(f"{action}_inspection_task", SimpleNamespace(**metadata, task_id=task_id))
        if rejected is not None:
            invoke(context, lambda: rejected)
    result = _read(context, operations.control_task, gateway, task_id, body, action=action)
    return receipt(context, result, metadata["request_id"])


def task(gateway, request, context):
    task_id = _text(context, request.task_id, "task_id")
    result = _read(context, operations.get_task, gateway, task_id)
    return _task(result)


def _task(result):
    details = dict(result)
    details["delivery"] = {key: value for key, value in result["delivery"].items() if key != "boot_id"}
    details["latest_event"] = _event(result["latest_event"]) if result["latest_event"] is not None else None
    details["timeline"] = [_event(event) for event in result["timeline"]]
    return inspection_pb.InspectionTask(
        task_id=result.get("task_id") or "", found=result.get("found") is True,
        current_state=result.get("current_state") or "", state_available=result.get("state_available") is True,
        phase=result.get("phase") or "", terminal=result.get("terminal") is True,
        execution_confirmed=result.get("execution_confirmed") is True,
        reason=result.get("reason") or "", details=fields(details),
    )


def _event(event):
    # Native boot/cursor identities remain private to the runtime owner.
    return {key: value for key, value in event.items() if key not in {"boot_id", "event_id"}}


def tasks(gateway, request, context):
    filters = {
        name: _text(context, getattr(request, name), name) if request.HasField(name) else None
        for name in ("map_id", "route_id")
    }
    result = _read(context, operations.list_tasks, gateway, **filters,
                   include_terminal=request.include_terminal, limit=request.limit if request.HasField("limit") else 20)
    return inspection_pb.InspectionTasks(
        tasks=[_task(task) for task in result["tasks"]], retention=result["retention"],
        count=result["count"], observed_at_unix_s=result["ts"],
    )


def report(gateway, request, context):
    task_id = _text(context, request.task_id, "task_id")
    return common_pb.Snapshot(fields=fields(_read(context, operations.task_report, gateway, task_id)))
