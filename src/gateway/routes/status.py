"""Status, telemetry, and health routes for GatewayModule."""

from __future__ import annotations

import asyncio
import json
import math
import os
import threading
import time
from collections.abc import Mapping, Sequence
from typing import Annotated, Any

from fastapi import HTTPException, Query
from fastapi.responses import JSONResponse, StreamingResponse

from gateway.schemas import (
    HealthResponse,
    LivenessResponse,
    LocalizationStatusResponse,
    LocationOperationResponse,
    LocationsResponse,
    LocationUpsertRequest,
    NavigationDdsSnapshotResponse,
    NavigationGoalStatusQueryResponse,
    NavigationStatusResponse,
    NavigationTaskStatusQueryResponse,
    PathResponse,
    ReadinessResponse,
    RuntimeDataflowResponse,
    RuntimeDataflowSubscribeRequest,
    RuntimeDataflowSubscribeResponse,
    RuntimeDataflowTopicDetailResponse,
    SceneGraphResponse,
    SSEEventEnvelope,
    StateResponse,
)
from gateway.services.environment_map_feedback import EnvironmentMapFeedback
from gateway.services.mapd_transport import mapd_query
from gateway.services.media_status import build_camera_status
from gateway.services.navigation_lifecycle import (
    query_navigation_goal_status,
    query_navigation_task_status,
)
from gateway.services.readiness import build_readiness_snapshot
from gateway.services.runtime_dataflow import (
    build_runtime_dataflow_snapshot,
    build_runtime_dataflow_subscription,
    build_runtime_dataflow_topic_detail,
)
from gateway.services.runtime_status import (
    build_localization_status,
    build_navigation_status,
)
from gateway.services.sse import subscribe_with_event_id, unsubscribe
from gateway.services.state_snapshot import build_state_snapshot
from gateway.services.telemetry_normalizers import (
    build_locations_response,
    build_path_response,
    build_scene_graph_response,
)
from gateway.services.traffic import (
    SSE_RETRY_MS,
    format_sse_message,
    normalize_sse_event,
)
from gateway.services.traffic import (
    snapshot as traffic_snapshot,
)
from runtime.contracts import CAMERA_ROLE, LIDAR_ROLE

DEFAULT_NAV_ENDPOINT_STATUS_FILE = "/dev/shm/lingtu/nav_endpoint_status.json"
DEFAULT_TRAVERSABILITY_STATUS_FILE = "/dev/shm/lingtu/traversability_status.json"
_LOCATION_BINDING_METADATA_KEYS = frozenset(
    {"map_id", "map_content_epoch", "frame_id", "binding_status"}
)


def _read_json_snapshot(path: str) -> dict[str, Any] | None:
    try:
        with open(path, encoding="utf-8") as fh:
            payload = json.load(fh)
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _native_nav_endpoint_status() -> dict[str, Any] | None:
    return _read_json_snapshot(os.environ.get("LINGTU_NAV_STATUS_FILE") or DEFAULT_NAV_ENDPOINT_STATUS_FILE)


def _native_traversability_status() -> dict[str, Any] | None:
    return _read_json_snapshot(
        os.environ.get("LINGTU_TRAVERSABILITY_STATUS_FILE") or DEFAULT_TRAVERSABILITY_STATUS_FILE
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


def _native_int(value: Any) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def _operator_motion_twist_payload(twist: Any) -> dict[str, Any]:
    if not isinstance(twist, Mapping):
        return {
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        }
    return {
        "linear": {
            "x": _native_float(twist.get("vx")),
            "y": _native_float(twist.get("vy")),
            "z": 0.0,
        },
        "angular": {
            "x": 0.0,
            "y": 0.0,
            "z": _native_float(twist.get("wz")),
        },
    }


def _native_operator_motion_trace(payload: Mapping[str, Any] | None) -> dict[str, Any]:
    if not isinstance(payload, Mapping):
        return {}
    operator_motion = payload.get("operator_motion")
    if not isinstance(operator_motion, Mapping):
        return {}

    operator_status = operator_motion.get("status")
    operator_status = operator_status if isinstance(operator_status, Mapping) else {}
    last_ack = operator_motion.get("last_ack")
    last_ack = last_ack if isinstance(last_ack, Mapping) else {}

    teleop_output = operator_status.get("teleop_output")
    final_cmd_vel = operator_status.get("final_cmd_vel")
    input_gate_reason = operator_status.get("input_gate_reason")
    teleop_reason = operator_status.get("authority_reason")

    return {
        "operator_motion": {
            "schema_version": _native_int(operator_motion.get("schema_version")),
            "interface_enabled": operator_motion.get("interface_enabled") is True,
            "authority_owner": operator_motion.get("authority_owner"),
            "control_mode": operator_motion.get("control_mode"),
            "allow_teleop_takeover": operator_motion.get("allow_teleop_takeover") is True,
            "teleop_output": _operator_motion_twist_payload(teleop_output),
            "final_cmd_vel": _operator_motion_twist_payload(final_cmd_vel),
            "teleop": {
                "reason": str(teleop_reason or ""),
                "output": _operator_motion_twist_payload(teleop_output),
            },
            "input_gate": {
                "reason": str(input_gate_reason or ""),
            },
            "last_ack": {
                "observed": last_ack.get("observed") is True,
                "published": last_ack.get("published") is True,
                "source_id": str(last_ack.get("source_id") or ""),
                "request_id": str(last_ack.get("request_id") or ""),
                "source_sequence": _native_int(last_ack.get("source_sequence")),
                "final_output_sequence": _native_int(last_ack.get("final_output_sequence")),
                "accepted": last_ack.get("accepted") is True,
                "reason": str(last_ack.get("reason") or ""),
            },
            "status": {
                "observed": operator_status.get("observed") is True,
                "published": operator_status.get("published") is True,
                "has_active_sample": operator_status.get("has_active_sample") is True,
                "holding": operator_status.get("holding") is True,
                "has_active_authority": operator_status.get("has_active_authority") is True,
                "last_sample_sequence": _native_int(
                    operator_status.get("last_sample_sequence")
                ),
                "admitted_sequence": _native_int(operator_status.get("admitted_sequence")),
                "final_output_sequence": _native_int(
                    operator_status.get("final_output_sequence")
                ),
            },
        }
    }


def _native_cmd_vel_payload(payload: Mapping[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(payload, Mapping):
        return None

    control_mode = str(payload.get("control_mode") or "")
    active_cmd_source = str(payload.get("active_cmd_source") or "")
    teleop_active = (
        control_mode in {"teleop", "teleop_avoid"}
        or active_cmd_source == "teleop"
    )
    final_output = payload.get("final_output")
    final_output = final_output if isinstance(final_output, Mapping) else {}
    output_sequence = _native_int(final_output.get("output_sequence"))
    final_output_published = (
        payload.get("publish_cmd_vel") is True
        and final_output.get("published") is True
        and output_sequence > 0
    )

    cmd = payload.get("final_cmd_vel")
    if isinstance(cmd, Mapping):
        active_source = "native_teleop" if teleop_active else "native_nav_endpoint"
        if not final_output_published:
            active_source = f"{active_source}_preview"
        evidence_stage = (
            "final_output_published"
            if final_output_published
            else "final_policy_output_not_published"
        )
    else:
        teleop = payload.get("teleop")
        cmd = (
            teleop.get("output")
            if teleop_active and isinstance(teleop, Mapping)
            else None
        )
        if isinstance(cmd, Mapping):
            active_source = "native_teleop_policy_preview"
            evidence_stage = "teleop_policy_output"
        else:
            last_local = payload.get("last_local")
            cmd = last_local.get("cmd_vel") if isinstance(last_local, Mapping) else None
            active_source = "native_local_planner_preview"
            evidence_stage = "local_planner_output"
    if not isinstance(cmd, Mapping):
        return None

    cmd_vel_payload = {
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
        "active_source": active_source,
        "evidence_stage": evidence_stage,
        "final_output_confirmed": final_output_published,
        "driver_delivery_accepted": (
            final_output_published and final_output.get("driver_delivery_accepted") is True
        ),
        "output_sequence": output_sequence if final_output_published else 0,
        "ts": payload.get("stamp_s"),
    }

    operator_motion = _native_operator_motion_trace(payload)
    if operator_motion:
        cmd_vel_payload["operator_motion"] = operator_motion.get("operator_motion")
    return cmd_vel_payload


def _probe_brainstem() -> dict[str, Any]:
    """Project the authoritative native driver snapshot into health telemetry."""

    path = os.environ.get("LINGTU_DRIVER_STATUS_FILE", "").strip() or (
        "/dev/shm/lingtu/driver_status.json"
    )
    payload = _read_json_snapshot(path)
    if payload is None:
        return {
            "status": "unavailable",
            "source": "lingtu-driver-status",
            "reason": "driver_status_missing",
            "driver_status_file": path,
        }

    adapter = payload.get("adapter")
    adapter = dict(adapter) if isinstance(adapter, Mapping) else {}
    control = payload.get("control")
    control = dict(control) if isinstance(control, Mapping) else {}
    output_ack = payload.get("output_ack")
    output_ack = dict(output_ack) if isinstance(output_ack, Mapping) else {}
    try:
        stamp_s = float(payload.get("stamp_s"))
        max_age_s = float(os.environ.get("LINGTU_DRIVER_STATUS_MAX_AGE_S", "1.5") or "1.5")
        age_s = time.time() - stamp_s
    except (TypeError, ValueError):
        age_s = math.inf
        max_age_s = 1.5
    stale = not math.isfinite(age_s) or age_s < -0.05 or age_s > max_age_s
    connected = payload.get("connected") is True
    status = "stale" if stale else ("connected" if connected else "unreachable")

    info = dict(control)
    info.update(
        {
            "status": status,
            "source": "lingtu-driver-status",
            "driver_status_file": path,
            "host": str(adapter.get("target") or ""),
            "protocol": adapter.get("protocol"),
            "owner": adapter.get("control_owner"),
            "owner_id": adapter.get("control_owner_id"),
            "connected": connected,
            "ready": payload.get("ready") is True,
            "output_ack": output_ack,
            "last_reason": payload.get("last_reason"),
            "last_error": payload.get("last_error"),
            "status_age_s": round(age_s, 3) if math.isfinite(age_s) else None,
            "stale": stale,
        }
    )
    return info


_BRAINSTEM_TRANSIENT_FIELDS = {
    "cached",
    "cache_age_s",
    "stale",
    "refreshing",
}


def _cacheable_brainstem_info(info: dict[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in dict(info).items() if key not in _BRAINSTEM_TRANSIENT_FIELDS}


def _probe_brainstem_safely() -> dict[str, Any]:
    try:
        return _probe_brainstem()
    except Exception as e:
        return {
            "status": "unreachable",
            "source": "lingtu-driver-status",
            "reason": "driver_status_probe_failed",
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
                "source": "lingtu-driver-status",
                "reason": "driver_status_probe_pending",
                "driver_status_file": (
                    os.environ.get("LINGTU_DRIVER_STATUS_FILE", "").strip()
                    or "/dev/shm/lingtu/driver_status.json"
                ),
                "cached": False,
                "stale": True,
                "refreshing": bool(scheduled or refreshing),
            }

    try:
        loop = asyncio.get_running_loop()
        info = await loop.run_in_executor(None, _probe_brainstem_safely)
    except Exception as e:
        info = {
            "status": "unreachable",
            "source": "lingtu-driver-status",
            "reason": "driver_status_probe_failed",
            "error": str(e)[:120],
        }

    info = dict(info)
    info["cached"] = False
    if ttl > 0.0 and lock is not None:
        _store_brainstem_health(gw, info)
    return info


def _health_module_needs_detail(name: str) -> bool:
    lowered = name.lower()
    if lowered in {LIDAR_ROLE, CAMERA_ROLE}:
        return True
    return any(
        token in lowered
        for token in (
            "slambridge",
            "slamadapter",
            "slammodule",
            "navigation",
        )
    )


def _module_odometry_status(health: Mapping[str, Any]) -> dict[str, Any] | None:
    """Return live SLAM telemetry when a module publishes odometry."""
    ports_out = health.get("ports_out")
    if not isinstance(ports_out, Mapping):
        return None
    odometry = ports_out.get("odometry")
    if not isinstance(odometry, Mapping):
        return None
    rate_hz = round(_positive_float(odometry.get("rate_hz")), 1)
    return {
        "status": "active" if rate_hz > 0.0 else "inactive",
        "hz": rate_hz,
        "messages": odometry.get("msg_count", 0),
    }


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


def _map_content_epoch_value(value: Any) -> int | None:
    return value if not isinstance(value, bool) and isinstance(value, int) and value > 0 else None


def _active_map_from_service(gw: Any) -> str:
    response = mapd_query(gw, {"action": "get_active_map"})
    if not isinstance(response, Mapping) or response.get("success") is not True:
        raise RuntimeError("mapd did not return active-map state")
    return str(response.get("active") or "").strip()


def _location_map_binding(gw) -> dict[str, Any]:
    """Snapshot the active saved-map identity for a tagged location."""
    for attempt in range(2):
        try:
            map_id = _active_map_from_service(gw)
        except Exception:
            return {"frame_id": "map", "binding_status": "unavailable"}
        if not map_id:
            return {"frame_id": "map", "binding_status": "unbound"}

        binding: dict[str, Any] = {
            "map_id": map_id,
            "frame_id": "map",
            "binding_status": "content_epoch_unavailable",
        }
        try:
            response = mapd_query(gw, {"action": "get_record", "map_id": map_id})
        except Exception:
            response = None
        try:
            active_after_query = _active_map_from_service(gw)
        except Exception:
            return {"frame_id": "map", "binding_status": "unavailable"}
        if active_after_query != map_id:
            if attempt == 0:
                continue
            return {"frame_id": "map", "binding_status": "active_map_changed"}

        record = response.get("record") if isinstance(response, Mapping) else None
        if (
            not isinstance(response, Mapping)
            or response.get("success") is not True
            or not isinstance(record, Mapping)
        ):
            return binding
        content_epoch = _map_content_epoch_value(record.get("content_epoch"))
        if content_epoch is None:
            return binding
        binding["map_content_epoch"] = content_epoch
        binding["binding_status"] = "bound"
        return binding
    return {"frame_id": "map", "binding_status": "active_map_changed"}


def _location_metadata(
    gw,
    existing: Any,
    requested: Mapping[str, Any],
) -> dict[str, Any]:
    """Merge caller metadata while keeping map binding server-owned."""
    merged: dict[str, Any] = {}
    existing_raw = existing if isinstance(existing, Mapping) else {}
    existing_metadata = existing_raw.get("metadata")
    for source in (existing_metadata, requested):
        if not isinstance(source, Mapping):
            continue
        merged.update(
            {
                str(key): value
                for key, value in source.items()
                if str(key) not in _LOCATION_BINDING_METADATA_KEYS
            }
        )
    merged.update(_location_map_binding(gw))
    return merged


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


def _slam_rates(
    gw: Any,
    localization_status: Mapping[str, Any],
    observed_odom_hz: Any = 0.0,
) -> tuple[float, float, float]:
    processed_scan_hz = _positive_float(localization_status.get("processed_scan_hz"))
    odom_hz = _positive_float(observed_odom_hz)
    if odom_hz <= 0.0:
        odom_hz = _positive_float(gw._get_slam_hz_cached())
    return processed_scan_hz, odom_hz, processed_scan_hz or odom_hz


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
        existing = tlm.store.query(body.name)
        existed = bool(existing)
        tlm.store.tag(
            body.name,
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            tags=body.tags,
            source=body.source,
            metadata=_location_metadata(gw, existing, body.metadata),
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
        responses={200: {"content": {"text/event-stream": {"schema": SSEEventEnvelope.model_json_schema()}}}},
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
        include_elevation: Annotated[
            bool,
            Query(
                description=(
                    "Opt in to the large /maps/elevation grid payload. "
                    "The default stream sends elevation metadata only."
                )
            ),
        ] = False,
    ):
        topic_filter = topic.strip() if isinstance(topic, str) else ""
        selected_event_types: set[str] | None = None
        subscription_payload: dict[str, Any] | None = None
        if topic_filter:
            detail = build_runtime_dataflow_topic_detail(gw, topic_filter)
            inspection = detail.get("inspection") if isinstance(detail.get("inspection"), Mapping) else {}
            stream_interfaces = [
                dict(item) for item in (inspection.get("stream_interfaces") or []) if isinstance(item, Mapping)
            ]
            selected_event_types = {str(item.get("event_type")) for item in stream_interfaces if item.get("event_type")}
            subscription_payload = {
                "ok": bool(detail.get("ok")) and bool(selected_event_types),
                "selector": topic_filter,
                "topic": (
                    (detail.get("topic") or {}).get("topic") if isinstance(detail.get("topic"), Mapping) else None
                ),
                "event_types": sorted(selected_event_types),
                "stream_interfaces": stream_interfaces,
                "blockers": [] if selected_event_types else ["no_gateway_sse_stream"],
            }

        # /maps/elevation is an explicit capability request.  A general map
        # scene subscription remains metadata-only unless include_elevation=1
        # is supplied by the client.
        elevation_payload = bool(
            include_elevation or topic_filter.rstrip("/") == "/maps/elevation"
        )
        q, snapshot_event_id = subscribe_with_event_id(
            gw,
            event_types=selected_event_types,
            include_elevation_payload=elevation_payload,
        )

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
                    if selected_event_types is not None and event.get("type") not in selected_event_types:
                        continue
                    yield format_sse_message(event)
                    await asyncio.sleep(0)
            finally:
                unsubscribe(gw, q)

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
        "/api/v1/maps/environment/layers",
        summary="Operator-facing environment map layer state",
    )
    async def get_environment_map_layers():
        feedback = getattr(gw, "_environment_map_feedback", None)
        if not isinstance(feedback, EnvironmentMapFeedback):
            raise HTTPException(status_code=503, detail="environment_map_feedback_unavailable")
        return feedback.snapshot(
            traversability_status=_native_traversability_status(),
            nav_endpoint_status=_native_nav_endpoint_status(),
        )

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
        operator_motion = _native_operator_motion_trace(nav_endpoint)
        cmd_payload = _native_cmd_vel_payload(nav_endpoint)
        if cmd_payload is not None and operator_motion:
            cmd_payload["operator_motion"] = operator_motion.get("operator_motion")
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
        "/api/v1/navigation/goals/{request_id}",
        summary="Request-correlated native navigation lifecycle status",
        response_model=NavigationGoalStatusQueryResponse,
    )
    async def get_navigation_goal_status(request_id: str):
        return query_navigation_goal_status(gw, request_id)

    @app.get(
        "/api/v1/navigation/tasks/{task_id}",
        summary="Stable native navigation task lifecycle status",
        response_model=NavigationTaskStatusQueryResponse,
    )
    async def get_navigation_task_status(task_id: str):
        return query_navigation_task_status(gw, task_id)

    @app.get(
        "/api/v1/runtime/dataflow",
        summary="Read-only Product motion and Gateway observability",
        response_model=RuntimeDataflowResponse,
    )
    async def get_runtime_dataflow():
        return build_runtime_dataflow_snapshot(gw)

    @app.get(
        "/api/v1/runtime/dataflow/topic",
        summary="Inspect one Gateway-observable Product topic",
        response_model=RuntimeDataflowTopicDetailResponse,
    )
    async def get_runtime_dataflow_topic(
        topic: Annotated[str, Query(description="Canonical topic or short alias")],
    ):
        return build_runtime_dataflow_topic_detail(gw, topic)

    @app.post(
        "/api/v1/runtime/dataflow/subscribe",
        summary="Create a read-only Gateway SSE subscription plan",
        response_model=RuntimeDataflowSubscribeResponse,
    )
    async def post_runtime_dataflow_subscribe(
        request: RuntimeDataflowSubscribeRequest,
    ):
        return build_runtime_dataflow_subscription(gw, request)

    @app.get(
        "/api/v1/health",
        summary="System health overview",
        response_model=HealthResponse,
    )
    async def get_health(
        details: Annotated[
            bool,
            Query(
                description="Probe every module health detail; default app polling path only probes displayed sensors.",
            ),
        ] = False,
    ):
        traffic = traffic_snapshot(gw)
        commands = gw._command_stats_snapshot()
        cloud_debug = gw._cloud_viewer.debug_snapshot()
        n_sse = traffic["sse"]["clients"]
        map_pts = gw._cloud_viewer.cache_point_count()

        sensors: dict[str, Any] = {}
        modules_ok = 0
        modules_fail = 0
        module_summary: dict[str, str] = {}

        def _probe_modules() -> tuple[dict[str, str], dict[str, Any], int, int]:
            """Probe module health in a thread to avoid blocking the event loop."""
            _summary: dict[str, str] = {}
            _sensors: dict[str, Any] = {}
            _ok = 0
            _fail = 0
            modules = getattr(gw, "_all_modules", None) or {}
            for name, mod in modules.items():
                probe_module = details or _health_module_needs_detail(str(name))
                if not probe_module:
                    _summary[name] = "ok"
                    _ok += 1
                    continue
                try:
                    h = mod.health() if hasattr(mod, "health") else {}
                    _summary[name] = "ok"
                    _ok += 1

                    name_l = str(name).lower()
                    if name_l == LIDAR_ROLE:
                        lidar_h = h.get("lidar", {})
                        _sensors["lidar"] = {
                            "status": lidar_h.get("state", "unknown"),
                            "ip": lidar_h.get("ip", "?"),
                            "cloud_hz": round(
                                h.get("ports_out", {}).get("scan", {}).get("rate_hz", 0),
                                1,
                            ),
                        }
                    elif name_l == CAMERA_ROLE:
                        _sensors["camera"] = build_camera_status(gw)
                    elif "slam" in name_l:
                        slam_status = _module_odometry_status(h)
                        if slam_status is not None:
                            _sensors["slam"] = slam_status
                except Exception:
                    _summary[name] = "error"
                    _fail += 1
            return _summary, _sensors, _ok, _fail

        # Run module probing in a thread to avoid blocking SSE/WS heartbeats
        modules = getattr(gw, "_all_modules", None) or {}
        if modules:
            module_summary, sensors, modules_ok, modules_fail = (
                await asyncio.get_running_loop().run_in_executor(None, _probe_modules)
            )

        localization_status = getattr(gw, "_localization_status", None)
        localization_status = localization_status if isinstance(localization_status, Mapping) else {}
        processed_scan_hz, odom_hz, slam_hz = _slam_rates(
            gw,
            localization_status,
            sensors.get("slam", {}).get("hz"),
        )
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
                        else ("localization_status" if localization_status else "gateway_odom_window")
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
                "cloud": cloud_debug,
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
        "/api/v1/metrics",
        summary="Operator-facing runtime metrics snapshot",
    )
    async def get_metrics():
        traffic = traffic_snapshot(gw)
        commands = gw._command_stats_snapshot()
        localization_status = getattr(gw, "_localization_status", None)
        localization_status = localization_status if isinstance(localization_status, Mapping) else {}
        camera = build_camera_status(gw)
        map_points = gw._cloud_viewer.cache_point_count()
        nav_endpoint = _native_nav_endpoint_status() or {}
        control_loop_health = nav_endpoint.get("control_loop_health")
        if not isinstance(control_loop_health, Mapping):
            control_loop_health = {}

        processed_scan_hz, odom_hz, slam_hz = _slam_rates(gw, localization_status)
        lidar_input_hz = _positive_float(localization_status.get("lidar_input_hz"))
        imu_input_hz = _positive_float(localization_status.get("imu_input_hz"))
        slam_tick_hz = _positive_float(localization_status.get("slam_tick_hz"))
        return {
            "schema_version": 1,
            "ok": True,
            "ts": time.time(),
            "gateway": {
                "port": gw._port,
                "mode": gw._mode,
                "sse_clients": traffic.get("sse", {}).get("clients", 0),
                "teleop_clients": gw._teleop_client_count(),
            },
            "slam": {
                "hz": round(slam_hz, 3),
                "processed_scan_hz": round(processed_scan_hz, 3),
                "lidar_input_hz": round(lidar_input_hz, 3),
                "imu_input_hz": round(imu_input_hz, 3),
                "slam_tick_hz": round(slam_tick_hz, 3),
                "odom_hz": round(odom_hz, 3),
                "state": str(localization_status.get("state") or "").lower(),
                "backend": localization_status.get("backend"),
                "mode": localization_status.get("mode"),
                "map_tf": localization_status.get("map_tf"),
            },
            "navigation": {
                "tick_hz": _positive_float(nav_endpoint.get("tick_hz")),
                "control_loop_health": dict(control_loop_health),
            },
            "map": {
                "points": map_points,
                "active": getattr(gw, "_active_map", None),
            },
            "camera": {
                "available": camera.get("available", False),
                "status": camera.get("status", "unknown"),
                "reason": camera.get("reason"),
                "fps": camera.get("fps", 0.0),
                "frames": camera.get("frames", 0),
                "backend": camera.get("backend"),
            },
            "traffic": traffic,
            "commands": commands,
            "websocket": (
                gw._ws_registry.snapshot()
                if hasattr(gw, "_ws_registry") and gw._ws_registry is not None
                else {}
            ),
        }

    @app.get(
        "/health",
        summary="Liveness probe",
        response_model=LivenessResponse,
    )
    async def liveness_health():
        traffic = traffic_snapshot(gw)
        cloud = gw._cloud_viewer.debug_snapshot()
        localization_status = getattr(gw, "_localization_status", None)
        localization_status = localization_status if isinstance(localization_status, Mapping) else {}
        processed_scan_hz, odom_hz, slam_hz = _slam_rates(gw, localization_status)
        return {
            "status": "ok",
            "ts": time.time(),
            "details_url": "/api/v1/health?details=true",
            "gateway": {
                "mode": gw._mode,
                "sse_clients": traffic.get("sse", {}).get("clients", 0),
                "cloud_clients": traffic.get("cloud", {}).get("clients", 0),
                "cloud": cloud,
            },
            "sensors": {
                "slam": {
                    "status": str(localization_status.get("state") or "").lower() or "unknown",
                    "hz": round(slam_hz, 1),
                    "processed_scan_hz": round(processed_scan_hz, 1),
                    "odom_hz": round(odom_hz, 1),
                    "lidar_input_hz": localization_status.get("lidar_input_hz"),
                }
            },
        }

    @app.get(
        "/ready",
        summary="Readiness probe",
        response_model=ReadinessResponse,
        responses={503: {"model": ReadinessResponse}},
    )
    async def readiness_ready(
        details: Annotated[
            bool,
            Query(
                description="Include per-module health details; default probe payload is summary-only.",
            ),
        ] = False,
    ):
        payload, status_code = build_readiness_snapshot(gw, include_details=details)
        return JSONResponse(payload, status_code=status_code)

    @app.get(
        "/api/v1/readiness",
        summary="Client readiness snapshot",
        response_model=ReadinessResponse,
    )
    async def api_readiness(
        details: Annotated[
            bool,
            Query(
                description="Include per-module health details for operator screens.",
            ),
        ] = False,
    ):
        payload, _status_code = build_readiness_snapshot(gw, include_details=details)
        return payload
