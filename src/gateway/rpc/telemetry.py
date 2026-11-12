"""Read map-pose and route caches without system-status or disk polling."""

import math
import threading
import time

import grpc

from gateway.services.runtime_status import (
    POSE_FRESH_MAX_ODOM_AGE_MS,
    build_localization_status,
    classify_pose_freshness,
)

from .codec import fields
from .generated import common_pb2 as common_pb
from .generated import localization_pb2 as localization_pb
from .generated import navigation_pb2 as navigation_pb
from .generated import observations_pb2 as observations_pb


def localization_state(gateway):
    status = build_localization_status(gateway)
    source = status.get("state") or ""
    quality = {
        "initializing": localization_pb.LOCALIZATION_QUALITY_INITIALIZING,
        "ready": localization_pb.LOCALIZATION_QUALITY_LOCALIZED,
        "lost": localization_pb.LOCALIZATION_QUALITY_LOST,
        "degraded": localization_pb.LOCALIZATION_QUALITY_DEGRADED,
        "relocalizing": localization_pb.LOCALIZATION_QUALITY_RELOCALIZING,
        "no_odometry": localization_pb.LOCALIZATION_QUALITY_NO_ODOMETRY,
        "tracking": localization_pb.LOCALIZATION_QUALITY_TRACKING,
    }.get(source, localization_pb.LOCALIZATION_QUALITY_UNKNOWN)
    result = localization_pb.LocalizationState(
        quality=quality, source_state=source,
        map_name=status.get("active_map") or "",
        reason="; ".join(status.get("reasons") or []),
    )
    for name in ("ready", "pose_fresh"):
        if isinstance(status.get(name), bool):
            setattr(result, name, status[name])
    return result


def map_pose(gateway, context, *, require_fresh=False):
    with gateway._state_lock:
        odometry = dict(gateway._odom or {})
        timestamps = getattr(gateway, "_odom_timestamps", None)
        received_at = timestamps[-1] if timestamps else None
        diagnostics = dict(getattr(gateway, "_localization_status", None) or {})
    if odometry.get("frame_id") != "map" or any(
        type(odometry.get(key)) not in (int, float) or not math.isfinite(odometry[key])
        for key in ("x", "y", "z", "yaw")
    ):
        context.abort(grpc.StatusCode.UNAVAILABLE, "map pose unavailable")
    fresh, _ = classify_pose_freshness(diagnostics)
    age_ms = None
    if received_at is not None:
        age = (time.time() - received_at) * 1000
        if math.isfinite(age) and age >= 0:
            age_ms = age
            fresh = fresh is not False and age <= POSE_FRESH_MAX_ODOM_AGE_MS
    if age_ms is None and fresh is not False:
        fresh = None
    if require_fresh and fresh is False:
        context.abort(grpc.StatusCode.UNAVAILABLE, "map pose is stale or localization is lost")
    return localization_pb.PoseSample(
        pose=common_pb.Pose(**{key: odometry[key] for key in ("x", "y", "z", "yaw")}),
        frame_id="map",
        details=fields(odometry),
        received_age_ms=age_ms,
        fresh=fresh,
    )


def navigation_path(gateway, context):
    with gateway._state_lock:
        path = list(gateway._last_path)
        frame_id = (getattr(gateway, "_last_path_meta", None) or {}).get("frame_id")
    points = []
    for point in path:
        if (point.get("frame_id") or frame_id) != "map" or any(
            type(point.get(key)) not in (int, float) or not math.isfinite(point[key])
            for key in ("x", "y", "z")
        ):
            context.abort(grpc.StatusCode.UNAVAILABLE, "map route unavailable")
        points.append(observations_pb.Vector3(**{key: point[key] for key in ("x", "y", "z")}))
    return navigation_pb.NavigationPath(
        frame_id="map" if points else frame_id or "",
        points=points,
    )


def watch(context, slots, read, *, interval=0.01):
    if not slots.acquire(blocking=False):
        context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED, "SDK watch limit reached")
    try:
        stopped = threading.Event()
        if not context.add_callback(stopped.set):
            return
        while context.is_active():
            started = time.monotonic()
            yield read()
            if stopped.wait(max(0.0, interval - (time.monotonic() - started))):
                return
    finally:
        slots.release()
