"""Bounded workbench tools using the same owners as the HTTP map interface."""

import asyncio
import json
import uuid
from types import SimpleNamespace

from fastapi import HTTPException
from fastapi.responses import JSONResponse

from gateway.maps.locations import location_entries, upsert_location
from gateway.maps.routes import (
    list_maps,
    preview_map_edits_request,
    public_map_name_error,
    save_edited_map_as_request,
    set_no_go_request,
)
from gateway.maps.transport import mapd_request
from gateway.navigation.actions import cancel as cancel_navigation
from gateway.navigation.actions import control_task, go as start_navigation
from gateway.navigation.tasks import query_navigation_task_status
from gateway.routes.inspection import get_task, list_routes, start_task
from gateway.schemas import (
    CancelRequest,
    GoalRequest,
    InspectionTaskStartRequest,
    LocationUpsertRequest,
    NavigationTaskPauseRequest,
    NavigationTaskResumeRequest,
    NavigationTaskSpeedLimitRequest,
    NoGoRequest,
    PlanPreviewRequest,
)
from gateway.services import robot_tools
from gateway.services.control_commands import ControlCommandService
from gateway.services.telemetry_normalizers import build_locations_response
from runtime.utils.python_analysis import run as run_python


def workbench_tools(gw, *, client_id: str = "web"):
    tools = []
    observations = {}

    def bind_agent_task(*, task_id: str = "", request_id: str = "", run_id: str = "",
                        conversation_id: str = "", turn_id: str = "", kind: str) -> None:
        if not run_id:
            return
        binding = {
            "run_id": run_id,
            "conversation_id": conversation_id,
            "turn_id": turn_id,
            "kind": kind,
        }
        lock = getattr(gw, "_state_lock", None)
        if lock is None:
            if task_id:
                getattr(gw, "_agent_task_bindings", {})[task_id] = dict(binding)
            if request_id:
                getattr(gw, "_agent_request_bindings", {})[request_id] = dict(binding)
            return
        with lock:
            if task_id:
                gw._agent_task_bindings[task_id] = dict(binding)
                while len(gw._agent_task_bindings) > 256:
                    gw._agent_task_bindings.pop(next(iter(gw._agent_task_bindings)), None)
            if request_id:
                gw._agent_request_bindings[request_id] = dict(binding)
                while len(gw._agent_request_bindings) > 256:
                    gw._agent_request_bindings.pop(next(iter(gw._agent_request_bindings)), None)

    def unbind_agent_task(*, task_id: str = "", request_id: str = "") -> None:
        lock = getattr(gw, "_state_lock", None)
        if lock is None:
            bindings = getattr(gw, "_agent_task_bindings", {})
            requests = getattr(gw, "_agent_request_bindings", {})
            bindings.pop(task_id, None)
            requests.pop(request_id, None)
            return
        with lock:
            gw._agent_task_bindings.pop(task_id, None)
            gw._agent_request_bindings.pop(request_id, None)

    def tool(name, description, properties, required=(), *, writes=False):
        definitions = {}
        for value in properties.values():
            definitions.update(value.pop("$defs", {}))
        tools.append({"type": "function", "writes": writes, "function": {
            "name": name, "description": description, "parameters": {
                "type": "object", "properties": properties, "required": list(required),
                "additionalProperties": False,
                **({"$defs": definitions} if definitions else {}),
            },
        }})

    map_id = {"type": "string", "description": "Exact saved map ID from context or list_saved_maps."}
    edits = {
        "map_id": map_id,
        "expected_source_content_epoch": {"type": "integer", "minimum": 1},
        "operations": {"type": "array", "items": {"type": "object"}, "description": (
            "Ordered edits: delete_inside/keep_inside {bounds:{min_x,min_y,min_z,max_x,max_y,max_z}}, "
            "add_plane {bounds,axis:x|y|z,offset,spacing}, voxel {voxel_size}, "
            "statistical {neighbors,std_ratio}, radius {radius,min_points}. Distances in metres. "
            "Use observed selection or explicit user coordinates; never invent bounds."
        )},
    }
    tool("list_saved_maps", "List saved maps and actual processing/readiness status.", {})
    tool("inspect_saved_map", "Read map metadata and existing navigation regions; no motion.", {"map_id": map_id}, ["map_id"])
    tool("view_saved_map", "Show an interactive 3D saved-map card in the conversation. Use when the operator asks to see, open or rotate a map. Obtain the exact map ID from context or list_saved_maps. Read-only: does not activate, edit or navigate. Geometry is loaded by the viewer, not returned to the model.", {"map_id": map_id}, ["map_id"])
    tool("list_inspection_routes", "List stored complete inspection routes on the current active map. Read-only; use the returned route ID before starting a route.", {})
    tool("find_inspection_points", "Find named inspection points by name, region name or tag within a map.",
         {"map_id": map_id, "query": {"type": "string"}}, ["map_id"])
    tool("start_inspection_route", "Start one stored complete inspection route. The route must already exist; this submits the native task and starts its evidence recording. The result confirms submission only, not arrival or inspection completion.",
         {"route_id": {"type": "string", "minLength": 1},
          "revision": {"type": "integer", "minimum": 0, "default": 0}}, ["route_id"], writes=True)
    tool("get_inspection_status", "Read the native lifecycle of one inspection task. Use the exact task_id returned by start_inspection_route until terminal evidence is reported.",
         {"task_id": {"type": "string", "minLength": 1}}, ["task_id"])
    tool("preview_navigation", "Preview a native navigation goal without publishing it. Use exact map-frame coordinates and report the returned feasibility reasons.",
         {"x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number", "default": 0.0},
          "acceptance_radius_m": {"type": "number", "exclusiveMinimum": 0}}, ["x", "y"])
    tool("start_navigation", "Start native navigation to an exact map-frame coordinate. The result is admission only. Retain task_id and query get_navigation_status until native feedback reports success, failure or cancellation.",
         {"x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number", "default": 0.0},
          "yaw": {"type": ["number", "null"], "default": None},
          "acceptance_radius_m": {"type": ["number", "null"], "exclusiveMinimum": 0},
          "max_speed_mps": {"type": ["number", "null"], "exclusiveMinimum": 0}}, ["x", "y"], writes=True)
    navigation_task_id = {"type": "string", "minLength": 1, "description": "Exact task_id returned by start_navigation or native navigation status."}
    tool("get_navigation_status", "Read the native lifecycle of one navigation task. A missing or stale status is not success or idle.",
         {"task_id": navigation_task_id}, ["task_id"])
    tool("cancel_navigation", "Request cancellation of exactly one native navigation task. Query get_navigation_status until stop confirmation is reported.",
         {"task_id": navigation_task_id, "reason": {"type": "string", "default": "agent_cancel"}}, ["task_id"], writes=True)
    tool("pause_navigation", "Request a stop-confirmed pause of exactly one native navigation task.",
         {"task_id": navigation_task_id, "reason": {"type": "string", "default": "agent_pause"}}, ["task_id"], writes=True)
    tool("resume_navigation", "Resume exactly one paused native navigation task. This does not create a replacement goal.",
         {"task_id": navigation_task_id, "reason": {"type": "string", "default": "agent_resume"}}, ["task_id"], writes=True)
    tool("set_navigation_speed", "Change the speed limit of exactly one active native navigation task while retaining its route.",
         {"task_id": navigation_task_id, "max_speed_mps": {"type": "number", "exclusiveMinimum": 0}},
         ["task_id", "max_speed_mps"], writes=True)
    tool("preview_map_edits", "Compute a point-cloud edit preview. Does not save or activate a map.", edits, edits)
    tool("save_map_copy", "Save a reviewed edit as a NEW map; preserves the original. Report actual build state; never activate or move.",
         {**edits, "name": {"type": "string"}}, [*edits, "name"], writes=True)
    tool("set_navigation_region", "Save a named 3D no-go/glass region using exact bounds. Changes navigation constraints.",
         {"map_id": map_id, "request": NoGoRequest.model_json_schema()}, ["map_id", "request"], writes=True)
    tool("save_inspection_point", "Save a named inspection point and region/action settings. Coordinates are BODY coordinates, not ground height. Never guess body height. metadata.inspection uses dwell and action. Does not navigate.",
         {"request": LocationUpsertRequest.model_json_schema()}, ["request"], writes=True)

    tool("get_robot_capabilities", "Discover the selected robot's observation adapters, fields, units, joint layout and unavailable sensors. Configured is not live. Call before choosing sensors; never guess endpoints or hardware layout.", {})
    tool("read_robot_sensor", "Read one measured IMU, joint, motor or voltage sample through the selected adapter. Preserve source time/clock and missing fields; a joint sample may be partial.",
         {"sensor": {"type": "string", "enum": list(robot_tools.SENSORS)}}, ["sensor"])
    tool("capture_camera", "Capture the robot camera JPEG for the operator and vision-capable model. May be cached; not a synchronized collision measurement.", {})
    tool("view_robot_card", "Show an expandable live observation card: robot status, joints and faults, IMU and records, or navigation task. Does not start recording or motion.", {"view": {"type": "string", "enum": ["robot", "joints", "imu", "navigation"]}}, ["view"])
    tool("view_live_camera", "Offer a collapsible live robot-camera card in chat when the operator asks to watch real-time video. The operator clicks to connect; no recording, camera reconfiguration or robot motion. A displayed card does not prove the camera is online.", {})
    tool("inspect_lidar", "Read one actual LiDAR viewer scan, with frame, timestamp, bounds and sampled 3D preview. Not an authoritative collision test or contact detector.", {})
    tool("inspect_navigation", "Read native navigation diagnostics to explain planning, stopping and final-control reasons. Do not invent contact or motion permission.", {})
    tool("run_python", "Run trusted operator-approved Host Python analysis (NOT a security sandbox). The data variable contains successful observations captured THIS turn, including sampled LiDAR points. Read sensors first. Standard environment libraries, temporary working directory, 10 second limit. Do not access devices, bypass native navigation/driver, or spawn background work. Print analysis results; files are temporary.",
         {"code": {"type": "string", "minLength": 1, "maxLength": 20000}}, ["code"], writes=True)

    tool("open_imu_monitor", "Open the operator's live IMU side panel, independently of chat scroll. Does not start recording.", {})
    monitors = (*robot_tools.SENSORS, "camera", "lidar", "gnss", "thermal", "acoustic", "semantic", "catalog")
    tool("open_sensor_monitor", "Open a sensor card beside chat, or catalog for all cards and display settings. Does not start recording or change device settings; unavailable sources remain unavailable.",
         {"sensor": {"type": "string", "enum": list(monitors)}}, ["sensor"])
    tool("start_imu_recording", "Start a persistent Host IMU monitoring record using the same stream as the panel. Continues if the browser closes. Returns actual record ID.", {}, writes=True)
    identity = {"type": "string", "description": "Exact IMU record ID returned by start/list; never invent it."}
    tool("stop_imu_recording", "Stop and save exactly the referenced IMU record. Never stop a different active record.", {"id": identity}, ["id"], writes=True)
    tool("list_imu_records", "List saved or interrupted IMU monitoring records for review.", {})
    tool("read_imu_record", "Read a recorded IMU interval. Summary statistics use ALL received interval samples; chart samples are downsampled extrema, not suitable for exact means or derivatives. Data is available to run_python as data['imu_record'].",
         {"id": identity, "start": {"type": "number", "minimum": 0}, "end": {"type": "number", "minimum": 0}}, ["id"])

    async def invoke(name, args, *, run_id: str = "", conversation_id: str = "", turn_id: str = ""):
        try:
            if name == "view_robot_card":
                if args["view"] not in {"robot", "joints", "imu", "navigation"}:
                    raise ValueError("Unknown robot card")
                return {"success": True, "observation_view": args["view"], "read_only": True}
            if name == "view_live_camera":
                return {"success": True, "camera_view": {"source": "robot"}, "read_only": True}
            if name == "open_imu_monitor":
                return {"success": True, "monitor": "imu"}
            if name == "open_sensor_monitor":
                if args["sensor"] not in monitors:
                    raise ValueError("Unknown sensor monitor")
                return {"success": True, "monitor": args["sensor"]}
            if name == "list_inspection_routes":
                result = await list_routes(gw, None)
                if isinstance(result, JSONResponse):
                    return json.loads(result.body)
                return result
            if name == "start_inspection_route":
                request_id = f"agent-inspection-{uuid.uuid4().hex}"
                body = InspectionTaskStartRequest(
                    route_id=args["route_id"],
                    revision=args.get("revision", 0),
                    request_id=request_id,
                )
                safety_body = SimpleNamespace(client_id=client_id, request_id=request_id)
                rejection = ControlCommandService(gw).motion_safety_rejection(
                    "start_inspection_task", safety_body
                )
                if rejection is not None:
                    return json.loads(rejection.body)
                # Bind by request before native submission; the first native
                # lifecycle event may arrive before the HTTP receipt returns.
                bind_agent_task(request_id=request_id, run_id=run_id,
                                conversation_id=conversation_id, turn_id=turn_id, kind="inspection")
                result = await start_task(gw, body)
                if isinstance(result, JSONResponse):
                    payload = json.loads(result.body)
                    if not payload.get("ok", payload.get("accepted", False)):
                        unbind_agent_task(request_id=request_id)
                    return payload
                if hasattr(result, "model_dump"):
                    result = result.model_dump(mode="json")
                task_id = str(result.get("task_id") or "") if isinstance(result, dict) else ""
                bind_agent_task(task_id=task_id, request_id=request_id, run_id=run_id,
                                conversation_id=conversation_id, turn_id=turn_id, kind="inspection")
                return result
            if name == "get_inspection_status":
                result = await get_task(gw, args["task_id"])
                if isinstance(result, JSONResponse):
                    return json.loads(result.body)
                if hasattr(result, "model_dump"):
                    return result.model_dump(mode="json")
                return result
            if name == "preview_navigation":
                result = ControlCommandService(gw).preview_plan(PlanPreviewRequest(
                    x=args["x"], y=args["y"], z=args.get("z", 0.0),
                    acceptance_radius_m=args.get("acceptance_radius_m"),
                ))
                return result
            if name == "start_navigation":
                task_id = f"agent-nav-task-{uuid.uuid4().hex}"
                request_id = f"agent-nav-{uuid.uuid4().hex}"
                bind_agent_task(task_id=task_id, request_id=request_id, run_id=run_id,
                                conversation_id=conversation_id, turn_id=turn_id, kind="navigation")
                body = GoalRequest(
                    x=args["x"], y=args["y"], z=args.get("z", 0.0), yaw=args.get("yaw"),
                    acceptance_radius_m=args.get("acceptance_radius_m"),
                    max_speed_mps=args.get("max_speed_mps"), source="api", target_type="coordinate",
                    instruction="Agent native navigation", client_id=client_id,
                    task_id=task_id, request_id=request_id,
                )
                result = start_navigation(gw, body)
                payload = json.loads(result.body) if isinstance(result, JSONResponse) else result
                if not isinstance(payload, dict) or payload.get("accepted") is not True:
                    unbind_agent_task(task_id=task_id, request_id=request_id)
                return payload
            if name == "get_navigation_status":
                return query_navigation_task_status(gw, args["task_id"])
            if name == "cancel_navigation":
                result = cancel_navigation(gw, CancelRequest(
                    task_id=args["task_id"], reason=args.get("reason", "agent_cancel"), client_id=client_id,
                    request_id=f"agent-cancel-{uuid.uuid4().hex}",
                ), args["task_id"])
                return json.loads(result.body) if isinstance(result, JSONResponse) else result
            if name in {"pause_navigation", "resume_navigation", "set_navigation_speed"}:
                task_id = args["task_id"]
                if name == "pause_navigation":
                    body = NavigationTaskPauseRequest(task_id=task_id, reason=args.get("reason", "agent_pause"), client_id=client_id)
                    action = "pause"
                elif name == "resume_navigation":
                    body = NavigationTaskResumeRequest(task_id=task_id, reason=args.get("reason", "agent_resume"), client_id=client_id)
                    action = "resume"
                else:
                    body = NavigationTaskSpeedLimitRequest(task_id=task_id, max_speed_mps=args["max_speed_mps"], client_id=client_id)
                    action = "speed_limit"
                result = control_task(gw, body, route_task_id=task_id, action=action)
                return json.loads(result.body) if isinstance(result, JSONResponse) else result
            if name in {"start_imu_recording", "stop_imu_recording", "list_imu_records", "read_imu_record"}:
                monitor = gw._imu_monitor
                if name == "start_imu_recording":
                    return {"success": True, "monitor": "imu", "record": await asyncio.to_thread(monitor.start_record)}
                if name == "stop_imu_recording":
                    return {"success": True, "monitor": "imu", "record": await asyncio.to_thread(monitor.stop_record, args["id"])}
                if name == "list_imu_records":
                    return {"success": True, "records": await asyncio.to_thread(monitor.records)}
                start, end = args.get("start", 0), args.get("end", 1e12)
                if end < start:
                    raise ValueError("Record end precedes start")
                result = await asyncio.to_thread(monitor.read_record, args["id"], start, end)
                observations["imu_record"] = result
                return {"success": True, **result}
            if name in {"get_robot_capabilities", "read_robot_sensor", "capture_camera", "inspect_lidar", "inspect_navigation"}:
                result = await robot_tools.invoke(gw, name, args)
                if result.get("success"):
                    observations[args.get("sensor", result.get("sensor", name))] = result
                return result
            if name == "run_python":
                return await run_python(args["code"], observations)
            if "map_id" in args:
                error = public_map_name_error(args["map_id"])
                if error:
                    raise ValueError(error)
            if name == "list_saved_maps":
                result = await list_maps(gw)
                if isinstance(result, dict) and isinstance(result.get("maps"), list):
                    result = {
                        **result,
                        "maps": [
                            {
                                key: value
                                for key, value in map_record.items()
                                if key != "content_epoch"
                            }
                            for map_record in result["maps"]
                            if isinstance(map_record, dict)
                        ],
                    }
            elif name == "view_saved_map":
                response = await asyncio.to_thread(mapd_request, gw, {"action": "get_record", "map_id": args["map_id"]})
                if response.get("success") is not True:
                    return {"success": False, "error": response.get("reason_code") or "map_not_found"}
                record = response.get("record") or {}
                if not isinstance(record, dict):
                    raise ValueError("Invalid saved map record")
                return {"success": True, "map_view": {"map_id": args["map_id"]},
                        "state": record.get("state"), "read_only": True}
            elif name == "inspect_saved_map":
                record = await asyncio.to_thread(mapd_request, gw, {"action": "get_record", "map_id": args["map_id"]})
                regions = await asyncio.to_thread(mapd_request, gw, {"action": "get_no_go_regions", "map_id": args["map_id"]})
                result = {"map": record, "regions": regions}
            elif name == "find_inspection_points":
                query = args.get("query", "").casefold()
                entries = await asyncio.to_thread(location_entries, gw)
                result = build_locations_response([entry for entry in entries
                    if entry.get("metadata", {}).get("map_id") == args["map_id"]
                    and query in " ".join([entry.get("name", ""), str(entry.get("metadata", {}).get("region", "")),
                                           *entry.get("tags", [])]).casefold()])
            elif name == "set_navigation_region":
                result = await set_no_go_request(gw, args["map_id"], NoGoRequest.model_validate(args["request"]))
            elif name == "save_inspection_point":
                request = LocationUpsertRequest.model_validate(args["request"])
                result = await asyncio.to_thread(upsert_location, gw, request, path_name=None)
            elif name in {"preview_map_edits", "save_map_copy"}:
                handler = preview_map_edits_request if name == "preview_map_edits" else save_edited_map_as_request
                result = await handler(gw, args["map_id"], args)
            else:
                raise ValueError("unsupported_tool")
            if isinstance(result, JSONResponse):
                result = json.loads(result.body)
            if name == "preview_map_edits":
                # Geometry stays in the viewer; the language model only needs counts and the edit recipe.
                result = {key: value for key, value in result.items()
                          if key not in {"preview_points", "preview_removed"}}
                result["preview_request"] = args
            return result
        except (ValueError, OSError, HTTPException) as exc:
            return {"success": False, "error": str(exc)}

    return tools, invoke
