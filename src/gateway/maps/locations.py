"""Tagged location API and server-owned active-map binding."""

from __future__ import annotations

import asyncio
import time
from collections.abc import Mapping
from typing import Any

from fastapi.responses import JSONResponse

from gateway.maps.transport import mapd_request
from gateway.schemas import LocationOperationResponse, LocationsResponse, LocationUpsertRequest
from gateway.services.runtime_status import POSE_FRESH_MAX_ODOM_AGE_MS, classify_pose_freshness
from gateway.services.telemetry_normalizers import build_locations_response
from runtime.tf.frames import map_frame_id

_LOCATION_BINDING_METADATA_KEYS = frozenset(
    {"map_id", "map_content_epoch", "frame_id", "binding_status"}
)


def location_entries(gw: Any) -> list[Any]:
    tlm = getattr(gw, "_tagged_loc_module", None)
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
    response = mapd_request(gw, {"action": "get_active_map"})
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
            response = mapd_request(gw, {"action": "get_record", "map_id": map_id})
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
    binding = _location_map_binding(gw)
    if (
        isinstance(existing_metadata, Mapping)
        and existing_metadata.get("binding_status") == "bound"
        and binding.get("binding_status") != "bound"
    ):
        raise ValueError("location_binding_unavailable")
    merged.update(binding)
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


def _current_pose(gw) -> tuple[float, float, float, float | None] | None:
    with gw._state_lock:
        odom = dict(gw._odom or {})
        received_at = gw._odom_timestamps[-1] if gw._odom_timestamps else None
        invalid = bool(gw._last_invalid_odometry)
        localization_status = dict(gw._localization_status or {})
    if (
        not odom
        or invalid
        or received_at is None
        or not 0.0 <= (time.time() - received_at) * 1000.0 <= POSE_FRESH_MAX_ODOM_AGE_MS
        or odom.get("frame_id") != map_frame_id()
        or classify_pose_freshness(localization_status)[0] is False
    ):
        return None
    x = _pose_value(odom, "x")
    y = _pose_value(odom, "y")
    z = _pose_value(odom, "z")
    if x is None or y is None or z is None:
        return None
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
    locations = build_locations_response(location_entries(gw))
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


def upsert_location(
    gw,
    body: LocationUpsertRequest,
    *,
    path_name: str | None,
) -> dict[str, Any] | JSONResponse:
    """Save a location with shared HTTP/MCP pose, binding, and persistence rules."""
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
                message="A fresh, valid map-frame robot pose is required.",
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


def register_location_routes(app, gw) -> None:
    @app.get(
        "/api/v1/locations",
        summary="List tagged navigation locations",
        response_model=LocationsResponse,
    )
    async def get_locations():
        return build_locations_response(await asyncio.to_thread(location_entries, gw))

    @app.post(
        "/api/v1/locations",
        summary="Create or update a tagged navigation location",
        response_model=LocationOperationResponse,
    )
    async def post_location(body: LocationUpsertRequest):
        return await asyncio.to_thread(upsert_location, gw, body, path_name=None)

    @app.put(
        "/api/v1/locations/{name}",
        summary="Update a tagged navigation location",
        response_model=LocationOperationResponse,
    )
    async def put_location(name: str, body: LocationUpsertRequest):
        return await asyncio.to_thread(upsert_location, gw, body, path_name=name)

    @app.delete(
        "/api/v1/locations/{name}",
        summary="Delete a tagged navigation location",
        response_model=LocationOperationResponse,
    )
    async def delete_location(name: str):
        return await asyncio.to_thread(_delete_location, name)

    def _delete_location(name: str):
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
