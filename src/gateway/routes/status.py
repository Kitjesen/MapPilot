"""Status, telemetry, and health routes for GatewayModule."""

from __future__ import annotations

import asyncio
import json
import os
import threading
import time
from collections.abc import Mapping, Sequence
from typing import Annotated, Any

from fastapi import Query
from fastapi.responses import JSONResponse, StreamingResponse

from gateway.schemas import (
    DevicesResponse,
    HealthResponse,
    LivenessResponse,
    LocalizationStatusResponse,
    LocationOperationResponse,
    LocationUpsertRequest,
    LocationsResponse,
    NavigationDdsSnapshotResponse,
    NavigationStatusResponse,
    PathResponse,
    ReadinessResponse,
    RuntimeDataflowResponse,
    RuntimeDataflowSubscribeRequest,
    RuntimeDataflowSubscribeResponse,
    RuntimeDataflowTopicDetailResponse,
    RuntimeSwitchPlanRequest,
    RuntimeSwitchPlanResponse,
    RuntimeSwitchRequest,
    RuntimeSwitchResponse,
    SSEEventEnvelope,
    SceneGraphResponse,
    StateResponse,
)
from gateway.services.media_status import build_camera_status
from gateway.services.readiness import build_readiness_snapshot
from gateway.services.runtime_dataflow import (
    build_runtime_dataflow_snapshot,
    build_runtime_dataflow_subscription,
    build_runtime_dataflow_topic_detail,
)
from gateway.services.runtime_switch_plan import build_runtime_switch_plan
from gateway.services.runtime_switch_execute import build_runtime_switch_response
from gateway.services.runtime_status import (
    build_localization_status,
    build_navigation_status,
)
from gateway.services.state_snapshot import build_state_snapshot
from gateway.services.traffic import (
    SSE_RETRY_MS,
    format_sse_message,
    normalize_sse_event,
)
from gateway.services.telemetry_normalizers import (
    build_locations_response,
    build_path_response,
    build_scene_graph_response,
)

DEFAULT_NAV_ENDPOINT_STATUS_FILE = "/dev/shm/lingtu/nav_endpoint_status.json"
DEFAULT_TRAVERSABILITY_STATUS_FILE = "/dev/shm/lingtu/traversability_status.json"


def _read_json_snapshot(path: str) -> dict[str, Any] | None:
    try:
        with open(path, encoding="utf-8") as fh:
            payload = json.load(fh)
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _native_nav_endpoint_status() -> dict[str, Any] | None:
    return _read_json_snapshot(
        os.environ.get("LINGTU_NAV_STATUS_FILE") or DEFAULT_NAV_ENDPOINT_STATUS_FILE
    )


def _native_traversability_status() -> dict[str, Any] | None:
    return _read_json_snapshot(
        os.environ.get("LINGTU_TRAVERSABILITY_STATUS_FILE")
        or DEFAULT_TRAVERSABILITY_STATUS_FILE
    )


def _native_path_points(payload: Mapping[str, Any] | None, key: str) -> list[dict[str, Any]]:
    if not isinstance(payload, Mapping):
        return []
    raw_path = payload.get(key)
    if not isinstance(raw_path, Sequence) or isinstance(raw_path, str):
        return []

    points: list[dict[str, Any]] = []
    for item in raw_path:
        if isinstance(item, Mapping):
            point = dict(item)
        elif isinstance(item, Sequence) and not isinstance(item, str) and len(item) >= 2:
            point = {
                "x": item[0],
                "y": item[1],
                "z": item[2] if len(item) >= 3 else 0.0,
            }
        else:
            continue
        point.setdefault("frame_id", "map")
        points.append(point)
    return points


def _native_float(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def _native_cmd_vel_payload(payload: Mapping[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(payload, Mapping):
        return None
    last_local = payload.get("last_local")
    if not isinstance(last_local, Mapping):
        return None
    cmd = last_local.get("cmd_vel")
    if not isinstance(cmd, Mapping):
        return None
    return {
        "frame_id": "base_link",
        "linear": {
            "x": _native_float(cmd.get("vx")),
            "y": _native_float(cmd.get("vy")),
            "z": 0.0,
        },
        "angular": {
            "x": 0.0,
            "y": 0.0,
            "z": _native_float(cmd.get("wz")),
        },
        "active_source": (
            "native_nav_endpoint"
            if payload.get("publish_cmd_vel") is True
            else "native_nav_endpoint_preview"
        ),
        "ts": payload.get("stamp_s"),
    }


def _probe_brainstem() -> dict[str, Any]:
    import brainstem_api as bapi
    import grpc

    ch = grpc.insecure_channel("127.0.0.1:13145")
    try:
        stub = bapi.RobotControlStub(ch)
        state = stub.GetCmsState(bapi.Empty(), timeout=1.0)
        fsm_map = {
            0: "ZERO",
            1: "GROUNDED",
            2: "STANDING",
            3: "WALKING",
            4: "TRANSITIONING",
        }
        info: dict[str, Any] = {
            "status": "connected",
            "host": "127.0.0.1:13145",
            "fsm": fsm_map.get(state.kind, str(state.kind)),
        }
        try:
            v = stub.GetVoltage(bapi.Empty(), timeout=1.0)
            if v.values:
                info["voltage_avg"] = round(sum(v.values) / len(v.values), 1)
        except (grpc.RpcError, AttributeError, KeyError):
            pass
        return info
    finally:
        ch.close()


_BRAINSTEM_TRANSIENT_FIELDS = {
    "cached",
    "cache_age_s",
    "stale",
    "refreshing",
}


def _cacheable_brainstem_info(info: dict[str, Any]) -> dict[str, Any]:
    return {
        key: value
        for key, value in dict(info).items()
        if key not in _BRAINSTEM_TRANSIENT_FIELDS
    }


def _probe_brainstem_safely() -> dict[str, Any]:
    try:
        return _probe_brainstem()
    except ImportError:
        return {
            "status": "unavailable",
            "reason": "brainstem_api not installed",
        }
    except Exception as e:
        return {
            "status": "unreachable",
            "host": "127.0.0.1:13145",
            "error": str(e)[:120],
        }


def _store_brainstem_health(gw, info: dict[str, Any]) -> None:
    lock = getattr(gw, "_brainstem_health_lock", None)
    if lock is None:
        return
    with lock:
        gw._brainstem_health_cache = _cacheable_brainstem_info(info)
        gw._brainstem_health_cache_ts = time.monotonic()


def _start_brainstem_refresh(gw) -> bool:
    lock = getattr(gw, "_brainstem_health_lock", None)
    if lock is None:
        return False
    with lock:
        if getattr(gw, "_brainstem_health_refreshing", False):
            return False
        gw._brainstem_health_refreshing = True

    def _refresh() -> None:
        try:
            _store_brainstem_health(gw, _probe_brainstem_safely())
        finally:
            with lock:
                gw._brainstem_health_refreshing = False

    try:
        thread = threading.Thread(
            target=_refresh,
            daemon=True,
            name="brainstem_health_refresh",
        )
        with lock:
            gw._brainstem_health_refresh_thread = thread
        thread.start()
        return True
    except Exception:
        with lock:
            gw._brainstem_health_refreshing = False
            gw._brainstem_health_refresh_thread = None
        return False


async def _brainstem_health(gw, *, force_live: bool = False) -> dict[str, Any]:
    now = time.monotonic()
    ttl = float(getattr(gw, "_brainstem_health_cache_ttl_s", 0.0) or 0.0)
    lock = getattr(gw, "_brainstem_health_lock", None)
    if ttl > 0.0 and lock is not None:
        cached = None
        age = 0.0
        refreshing = False
        with lock:
            cached = getattr(gw, "_brainstem_health_cache", None)
            cache_ts = float(getattr(gw, "_brainstem_health_cache_ts", 0.0) or 0.0)
            age = now - cache_ts
            refreshing = bool(getattr(gw, "_brainstem_health_refreshing", False))
        if cached is not None and age <= ttl and not force_live:
            info = dict(cached)
            info["cached"] = True
            info["cache_age_s"] = round(max(0.0, age), 3)
            return info
        if not force_live:
            scheduled = _start_brainstem_refresh(gw)
            if cached is not None:
                info = dict(cached)
                info["cached"] = True
                info["cache_age_s"] = round(max(0.0, age), 3)
                info["stale"] = True
                info["refreshing"] = bool(scheduled or refreshing)
                return info
            return {
                "status": "unknown",
                "host": "127.0.0.1:13145",
                "reason": "probe_pending",
                "cached": False,
                "stale": True,
                "refreshing": bool(scheduled or refreshing),
            }

    try:
        loop = asyncio.get_running_loop()
        info = await loop.run_in_executor(None, _probe_brainstem_safely)
    except Exception as e:
        info = {"status": "unreachable", "error": str(e)[:120]}

    info = dict(info)
    info["cached"] = False
    if ttl > 0.0 and lock is not None:
        _store_brainstem_health(gw, info)
    return info


def _health_module_needs_detail(name: str) -> bool:
    lowered = name.lower()
    return any(
        token in lowered
        for token in (
            "lidarmodule",
            "camera",
            "slambridge",
            "slammodule",
            "navigation",
        )
    )


def _location_entries(gw) -> list[Any]:
    tlm = gw._tagged_loc_module
    if tlm is None:
        return []
    try:
        return list(tlm.store.list_all())
    except Exception:
        try:
            return list(tlm.store._store.values())
        except Exception:
            return []


def _pose_value(value: Any, key: str) -> float | None:
    if isinstance(value, Mapping):
        raw = value.get(key)
    else:
        raw = getattr(value, key, None)
    if raw is None:
        pose = getattr(value, "pose", None)
        position = getattr(pose, "position", None)
        raw = getattr(position, key, None)
    try:
        num = float(raw)
    except (TypeError, ValueError):
        return None
    return num if num == num and num not in (float("inf"), float("-inf")) else None


def _positive_float(value: Any) -> float:
    try:
        num = float(value)
    except (TypeError, ValueError):
        return 0.0
    if num != num or num in (float("inf"), float("-inf")):
        return 0.0
    return num if num > 0.0 else 0.0


def _current_pose(gw) -> tuple[float, float, float, float | None] | None:
    with gw._state_lock:
        odom = gw._odom
    if odom is None:
        return None
    x = _pose_value(odom, "x")
    y = _pose_value(odom, "y")
    if x is None or y is None:
        return None
    z = _pose_value(odom, "z") or 0.0
    yaw = _pose_value(odom, "yaw")
    return x, y, z, yaw


def _locations_operation_payload(
    gw,
    *,
    ok: bool,
    status: str,
    action: str,
    location: dict[str, Any] | None = None,
    message: str | None = None,
    error: str | None = None,
    request_id: str | None = None,
    client_id: str = "unknown",
) -> dict[str, Any]:
    locations = build_locations_response(_location_entries(gw))
    payload = {
        "schema_version": 1,
        "ok": ok,
        "status": status,
        "action": action,
        "location": location,
        "locations": locations,
        "message": message,
        "error": error,
        "request_id": request_id,
        "client_id": client_id,
        "ts": time.time(),
    }
    if hasattr(gw, "push_event"):
        gw.push_event({"type": "location", "data": payload})
        gw.push_event({"type": "locations", "data": locations})
    return payload


def _upsert_location(
    gw,
    body: LocationUpsertRequest,
    *,
    path_name: str | None,
) -> dict[str, Any] | JSONResponse:
    if path_name is not None and body.name != path_name:
        payload = _locations_operation_payload(
            gw,
            ok=False,
            status="invalid",
            action="update",
            message="Request body name must match the URL location name.",
            error="location_name_mismatch",
            request_id=body.request_id,
            client_id=body.client_id,
        )
        return JSONResponse(payload, status_code=400)

    tlm = gw._tagged_loc_module
    if tlm is None:
        return _locations_operation_payload(
            gw,
            ok=False,
            status="unavailable",
            action="create" if path_name is None else "update",
            message="Tagged locations module is not available.",
            error="location_store_unavailable",
            request_id=body.request_id,
            client_id=body.client_id,
        )

    pose = None
    if body.use_current_pose:
        pose = _current_pose(gw)
        if pose is None:
            return _locations_operation_payload(
                gw,
                ok=False,
                status="invalid",
                action="create" if path_name is None else "update",
                message="Current robot pose is unavailable.",
                error="current_pose_unavailable",
                request_id=body.request_id,
                client_id=body.client_id,
            )

    x = body.x
    y = body.y
    z = body.z
    yaw = body.yaw
    if pose is not None:
        x, y, z, pose_yaw = pose
        yaw = yaw if yaw is not None else pose_yaw
    if x is None or y is None:
        return _locations_operation_payload(
            gw,
            ok=False,
            status="invalid",
            action="create" if path_name is None else "update",
            message="x and y are required unless use_current_pose is true.",
            error="coordinates_required",
            request_id=body.request_id,
            client_id=body.client_id,
        )

    existed = False
    try:
        existed = bool(tlm.store.query(body.name))
        tlm.store.tag(
            body.name,
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            tags=body.tags,
            source=body.source,
            metadata=body.metadata,
        )
        entry = tlm.store.query(body.name)
    except Exception as exc:
        return _locations_operation_payload(
            gw,
            ok=False,
            status="error",
            action="update" if path_name is not None or existed else "create",
            message="Failed to save tagged location.",
            error=str(exc),
            request_id=body.request_id,
            client_id=body.client_id,
        )

    if hasattr(tlm, "tag_status"):
        tlm.tag_status.publish(f"saved:{body.name}")

    response = build_locations_response([entry])
    location = response["locations"][0] if response["locations"] else None
    return _locations_operation_payload(
        gw,
        ok=True,
        status="saved",
        action="update" if existed else "create",
        location=location,
        message=f"Saved location {body.name!r}.",
        request_id=body.request_id,
        client_id=body.client_id,
    )


def register_status_routes(app, gw) -> None:
    @app.get(
        "/api/v1/events",
        summary="SSE event stream",
        response_class=StreamingResponse,
        responses={
            200: {
                "content": {
                    "text/event-stream": {"schema": SSEEventEnvelope.model_json_schema()}
                }
            }
        },
    )
    async def sse_events(
        topic: Annotated[
            str | None,
            Query(
                description=(
                    "Optional runtime dataflow topic or alias. When set, the "
                    "SSE stream emits only Gateway events backing that stream."
                )
            ),
        ] = None,
    ):
        q, snapshot_event_id = gw._sse_subscribe_with_event_id()
        topic_filter = topic.strip() if isinstance(topic, str) else ""
        selected_event_types: set[str] | None = None
        subscription_payload: dict[str, Any] | None = None
        if topic_filter:
            detail = build_runtime_dataflow_topic_detail(gw, topic_filter)
            inspection = (
                detail.get("inspection")
                if isinstance(detail.get("inspection"), Mapping)
                else {}
            )
            stream_interfaces = [
                dict(item)
                for item in (inspection.get("stream_interfaces") or [])
                if isinstance(item, Mapping)
            ]
            selected_event_types = {
                str(item.get("event_type"))
                for item in stream_interfaces
                if item.get("event_type")
            }
            subscription_payload = {
                "ok": bool(detail.get("ok")) and bool(selected_event_types),
                "selector": topic_filter,
                "topic": (
                    (detail.get("topic") or {}).get("topic")
                    if isinstance(detail.get("topic"), Mapping)
                    else None
                ),
                "event_types": sorted(selected_event_types),
                "stream_interfaces": stream_interfaces,
                "ros2_topic_required": False,
                "blockers": [] if selected_event_types else ["no_gateway_sse_stream"],
            }

        async def _stream():
            try:
                if subscription_payload is not None:
                    yield format_sse_message(
                        normalize_sse_event(
                            {
                                "type": "runtime_dataflow_subscription",
                                "data": subscription_payload,
                            },
                            event_id=snapshot_event_id,
                        ),
                        retry_ms=SSE_RETRY_MS,
                    )
                else:
                    snapshot = {
                        "type": "snapshot",
                        "data": build_state_snapshot(gw),
                    }
                    yield format_sse_message(
                        normalize_sse_event(snapshot, event_id=snapshot_event_id),
                        retry_ms=SSE_RETRY_MS,
                    )

                while True:
                    try:
                        event = await asyncio.wait_for(q.get(), timeout=1.0)
                    except asyncio.TimeoutError:
                        yield format_sse_message(
                            normalize_sse_event(
                                {"type": "ping"},
                                now=time.time(),
                            )
                        )
                        continue
                    if (
                        selected_event_types is not None
                        and event.get("type") not in selected_event_types
                    ):
                        continue
                    yield format_sse_message(event)
                    await asyncio.sleep(0)
            finally:
                gw._sse_unsubscribe(q)

        return StreamingResponse(
            _stream(),
            media_type="text/event-stream",
            headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
        )

    @app.get(
        "/api/v1/state",
        summary="Full robot state snapshot",
        response_model=StateResponse,
    )
    async def get_state():
        return build_state_snapshot(gw)

    @app.get(
        "/api/v1/scene_graph",
        summary="Current scene graph",
        response_model=SceneGraphResponse,
    )
    async def get_scene_graph():
        with gw._state_lock:
            sg = gw._sg_json
        return build_scene_graph_response(sg)

    @app.get(
        "/api/v1/locations",
        summary="List tagged navigation locations",
        response_model=LocationsResponse,
    )
    async def get_locations():
        return build_locations_response(_location_entries(gw))

    @app.post(
        "/api/v1/locations",
        summary="Create or update a tagged navigation location",
        response_model=LocationOperationResponse,
    )
    async def post_location(body: LocationUpsertRequest):
        return _upsert_location(gw, body, path_name=None)

    @app.put(
        "/api/v1/locations/{name}",
        summary="Update a tagged navigation location",
        response_model=LocationOperationResponse,
    )
    async def put_location(name: str, body: LocationUpsertRequest):
        return _upsert_location(gw, body, path_name=name)

    @app.delete(
        "/api/v1/locations/{name}",
        summary="Delete a tagged navigation location",
        response_model=LocationOperationResponse,
    )
    async def delete_location(name: str):
        tlm = gw._tagged_loc_module
        if tlm is None:
            return _locations_operation_payload(
                gw,
                ok=False,
                status="unavailable",
                action="delete",
                message="Tagged locations module is not available.",
                error="location_store_unavailable",
            )
        try:
            removed = bool(tlm.store.remove(name))
        except Exception as exc:
            return _locations_operation_payload(
                gw,
                ok=False,
                status="error",
                action="delete",
                message="Failed to delete tagged location.",
                error=str(exc),
            )
        if hasattr(tlm, "tag_status"):
            tlm.tag_status.publish(f"removed:{name}" if removed else f"not_found:{name}")
        return _locations_operation_payload(
            gw,
            ok=removed,
            status="deleted" if removed else "not_found",
            action="delete",
            message=f"Deleted location {name!r}." if removed else f"Location {name!r} not found.",
            error=None if removed else "location_not_found",
        )

    @app.get(
        "/api/v1/path",
        summary="Latest planned path",
        response_model=PathResponse,
    )
    async def get_path():
        with gw._state_lock:
            path = gw._last_path
            robot = gw._odom
        return build_path_response(path, robot)

    @app.get(
        "/api/v1/navigation/dds_snapshot",
        summary="Latest navigation data for the native DDS endpoint",
        response_model=NavigationDdsSnapshotResponse,
    )
    async def get_navigation_dds_snapshot():
        with gw._state_lock:
            global_path = list(gw._last_path)
            local_path = list(gw._last_local_path)
            robot = gw._odom
        nav_endpoint = _native_nav_endpoint_status()
        traversability_endpoint = _native_traversability_status()
        if not global_path:
            global_path = _native_path_points(nav_endpoint, "global_path")
        if not local_path:
            local_path = _native_path_points(nav_endpoint, "local_path")
        navigation = build_navigation_status(gw)
        cmd_vel = (
            navigation.get("control", {})
            .get("cmd_vel_mux", {})
            .get("last_driver_cmd_vel")
        )
        if isinstance(cmd_vel, Mapping):
            cmd_payload = {
                "frame_id": "base_link",
                "linear": dict(cmd_vel.get("linear") or {}),
                "angular": dict(cmd_vel.get("angular") or {}),
                "active_source": str(cmd_vel.get("active_source") or "none"),
                "ts": cmd_vel.get("ts"),
            }
        else:
            cmd_payload = _native_cmd_vel_payload(nav_endpoint)
        return {
            "schema_version": "lingtu.navigation.dds_snapshot.v1",
            "global_path": build_path_response(global_path, robot),
            "local_path": build_path_response(local_path, robot),
            "cmd_vel": cmd_payload,
            "nav_endpoint": nav_endpoint,
            "traversability_endpoint": traversability_endpoint,
            "navigation": navigation,
            "ts": time.time(),
            "source": "gateway_navigation_cache+native_status",
        }

    @app.get(
        "/api/v1/localization/status",
        summary="Localization status for app and web clients",
        response_model=LocalizationStatusResponse,
    )
    async def get_localization_status():
        return build_localization_status(gw)

    @app.get(
        "/api/v1/navigation/status",
        summary="Navigation mission and control status",
        response_model=NavigationStatusResponse,
    )
    async def get_navigation_status():
        return build_navigation_status(gw)

    @app.get(
        "/api/v1/runtime/dataflow",
        summary="Runtime dataflow and Module port observability",
        response_model=RuntimeDataflowResponse,
    )
    async def get_runtime_dataflow():
        return build_runtime_dataflow_snapshot(gw)

    @app.get(
        "/api/v1/runtime/dataflow/topic",
        summary="Inspect one runtime dataflow topic",
        response_model=RuntimeDataflowTopicDetailResponse,
    )
    async def get_runtime_dataflow_topic(
        topic: Annotated[str, Query(description="Canonical topic or short alias")],
    ):
        return build_runtime_dataflow_topic_detail(gw, topic)

    @app.post(
        "/api/v1/runtime/dataflow/subscribe",
        summary="Create a read-only runtime dataflow SSE subscription plan",
        response_model=RuntimeDataflowSubscribeResponse,
    )
    async def post_runtime_dataflow_subscribe(
        request: RuntimeDataflowSubscribeRequest,
    ):
        return build_runtime_dataflow_subscription(gw, request)

    @app.post(
        "/api/v1/runtime/switch-plan",
        summary="Dry-run runtime endpoint switch plan",
        response_model=RuntimeSwitchPlanResponse,
    )
    async def post_runtime_switch_plan(
        request: RuntimeSwitchPlanRequest,
    ):
        return build_runtime_switch_plan(request)

    @app.post(
        "/api/v1/runtime/switch",
        summary="Validate and optionally execute a product mode switch",
        response_model=RuntimeSwitchResponse,
    )
    async def post_runtime_switch(
        request: RuntimeSwitchRequest,
    ):
        return build_runtime_switch_response(gw, request)

    @app.get(
        "/api/v1/navigation",
        response_model=NavigationStatusResponse,
        include_in_schema=False,
    )
    async def get_navigation_status_legacy_alias():
        return build_navigation_status(gw)

    @app.get(
        "/api/v1/devices",
        summary="Hardware device registry status",
        response_model=DevicesResponse,
    )
    async def get_devices():
        modules = getattr(gw, "_all_modules", None) or {}
        mgr = modules.get("DeviceManager")
        if mgr is None:
            return {"devices": [], "manager": "not_loaded"}
        try:
            health = mgr.health()
            return {
                "manager": "ok",
                "spec_count": health.get("spec_count", 0),
                "opened_count": health.get("opened_count", 0),
                "devices": health.get("devices", []),
            }
        except Exception as e:
            return {"devices": [], "manager": "error", "error": str(e)}

    @app.get(
        "/api/v1/health",
        summary="System health overview",
        response_model=HealthResponse,
    )
    async def get_health(
        details: Annotated[bool, Query(
            description="Probe every module health detail; default app polling path only probes displayed sensors.",
        )] = False
    ):
        traffic = gw._traffic_stats_snapshot()
        commands = gw._command_stats_snapshot()
        n_sse = traffic["sse"]["clients"]
        with gw._map_cloud_lock:
            map_pts = len(gw._map_points) if gw._map_points is not None else 0

        sensors: dict[str, Any] = {}
        modules_ok = 0
        modules_fail = 0
        module_summary: dict[str, str] = {}

        modules = getattr(gw, "_all_modules", None) or {}
        if modules:
            for name, mod in modules.items():
                probe_module = details or _health_module_needs_detail(str(name))
                if not probe_module:
                    module_summary[name] = "ok"
                    modules_ok += 1
                    continue
                try:
                    h = mod.health() if hasattr(mod, "health") else {}
                    module_summary[name] = "ok"
                    modules_ok += 1

                    if "LidarModule" in name:
                        lidar_h = h.get("lidar", {})
                        sensors["lidar"] = {
                            "status": lidar_h.get("state", "unknown"),
                            "ip": lidar_h.get("ip", "?"),
                            "cloud_hz": round(
                                h.get("ports_out", {})
                                .get("scan", {})
                                .get("rate_hz", 0),
                                1,
                            ),
                        }
                    elif "CameraBridge" in name:
                        sensors["camera"] = build_camera_status(gw)
                    elif "SlamBridge" in name:
                        odom_out = h.get("ports_out", {}).get("odometry", {})
                        slam_rate = round(odom_out.get("rate_hz", 0), 1)
                        sensors["slam"] = {
                            "status": (
                                "active"
                                if slam_rate > 0.0
                                else "inactive"
                            ),
                            "hz": slam_rate,
                            "messages": odom_out.get("msg_count", 0),
                        }
                    elif "nav.mission" in name:
                        nav = h.get("navigation", h)
                        sensors["navigation"] = {
                            "state": nav.get(
                                "state",
                                h.get("mission_state", "idle"),
                            ),
                            "replan_count": nav.get(
                                "replan_count",
                                h.get("replan_count", 0),
                            ),
                        }
                except Exception:
                    module_summary[name] = "error"
                    modules_fail += 1

        localization_status = getattr(gw, "_localization_status", None)
        localization_status = localization_status if isinstance(localization_status, Mapping) else {}
        processed_scan_hz = _positive_float(localization_status.get("processed_scan_hz"))
        odom_hz = _positive_float(sensors.get("slam", {}).get("hz"))
        if odom_hz <= 0.0:
            odom_hz = _positive_float(gw._get_slam_hz_cached())
        slam_hz = processed_scan_hz or odom_hz
        has_odom = gw._odom is not None
        if localization_status or has_odom or odom_hz > 0.0:
            slam_sensor = sensors.setdefault("slam", {})
            state = str(
                localization_status.get("state")
                or slam_sensor.get("status")
                or ("active" if odom_hz > 0.0 else "inactive")
            ).lower()
            slam_sensor.update(
                {
                    "status": state,
                    "hz": round(slam_hz, 1),
                    "processed_scan_hz": round(processed_scan_hz, 1),
                    "odom_hz": round(odom_hz, 1),
                    "source": (
                        "processed_scan_hz"
                        if processed_scan_hz > 0.0
                        else (
                            "localization_status"
                            if localization_status
                            else "gateway_odom_window"
                        )
                    ),
                }
            )
            if not localization_status:
                slam_sensor.setdefault("reason", "localization_status_missing")
            for key in (
                "reason",
                "status_snapshot_stale",
                "status_snapshot_age_s",
                "lidar_input_hz",
                "imu_input_hz",
                "slam_tick_hz",
            ):
                if key in localization_status:
                    slam_sensor[key] = localization_status[key]

        brainstem_info = await _brainstem_health(gw, force_live=details)

        return {
            "status": "ok" if modules_fail == 0 else "degraded",
            "modules_ok": modules_ok,
            "modules_fail": modules_fail,
            "gateway": {
                "port": gw._port,
                "mode": gw._mode,
                "sse_clients": n_sse,
                "traffic": traffic,
                "commands": commands,
                "diagnostic_details": details,
            },
            "teleop": {
                "active": gw._teleop_active,
                "clients": gw._teleop_client_count(),
            },
            "sensors": sensors,
            "slam_hz": round(slam_hz, 1),
            "map_points": map_pts,
            "has_odom": has_odom,
            "modules": module_summary,
            "brainstem": brainstem_info,
        }

    @app.get(
        "/health",
        summary="Liveness probe",
        response_model=LivenessResponse,
    )
    async def liveness_health():
        return {"status": "ok", "ts": time.time()}

    @app.get(
        "/ready",
        summary="Readiness probe",
        response_model=ReadinessResponse,
        responses={503: {"model": ReadinessResponse}},
    )
    async def readiness_ready(
        details: Annotated[bool, Query(
            description="Include per-module health details; default probe payload is summary-only.",
        )] = False
    ):
        payload, status_code = build_readiness_snapshot(gw, include_details=details)
        return JSONResponse(payload, status_code=status_code)

    @app.get(
        "/api/v1/readiness",
        summary="Client readiness snapshot",
        response_model=ReadinessResponse,
    )
    async def api_readiness(
        details: Annotated[bool, Query(
            description="Include per-module health details for operator screens.",
        )] = False
    ):
        payload, _status_code = build_readiness_snapshot(gw, include_details=details)
        return payload
