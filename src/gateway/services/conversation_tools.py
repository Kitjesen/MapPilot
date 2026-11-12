"""Bounded workbench tools using the same owners as the HTTP map interface."""

import asyncio
import json

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
from gateway.schemas import LocationUpsertRequest, NoGoRequest
from gateway.services import robot_tools
from gateway.services.telemetry_normalizers import build_locations_response
from runtime.utils.python_analysis import run as run_python


def workbench_tools(gw):
    tools = []
    observations = {}

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
    tool("find_inspection_points", "Find named inspection points by name, region name or tag within a map.",
         {"map_id": map_id, "query": {"type": "string"}}, ["map_id"])
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

    async def invoke(name, args):
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
            elif name == "view_saved_map":
                response = await asyncio.to_thread(mapd_request, gw, {"action": "get_record", "map_id": args["map_id"]})
                if response.get("success") is not True:
                    return {"success": False, "error": response.get("reason_code") or "map_not_found"}
                record = response.get("record") or {}
                if not isinstance(record, dict):
                    raise ValueError("Invalid saved map record")
                epoch = record.get("content_epoch")
                if not isinstance(epoch, int) or isinstance(epoch, bool) or epoch < 1:
                    raise ValueError("Map has no valid content version; refresh the saved map list")
                return {"success": True, "map_view": {"map_id": args["map_id"], "content_epoch": epoch},
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
