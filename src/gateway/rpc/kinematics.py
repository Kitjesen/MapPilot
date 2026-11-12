"""Read localization feedback from the authoritative Gateway odometry cache."""

import math
import time

import grpc

from gateway.services.runtime_status import POSE_FRESH_MAX_ODOM_AGE_MS, classify_pose_freshness

from .generated import motion_pb2 as pb
from .generated import observations_pb2 as observations_pb


def _unavailable(context):
    context.abort(grpc.StatusCode.UNAVAILABLE, "Localization motion feedback unavailable or stale")


def _finite(values):
    return all(type(value) in (int, float) and math.isfinite(value) for value in values)


def _snapshot(gateway, context):
    with gateway._state_lock:
        data = dict(gateway._odom or {})
        timestamps = getattr(gateway, "_odom_timestamps", None)
        received_at = timestamps[-1] if timestamps else None
        diagnostics = dict(getattr(gateway, "_localization_status", None) or {})
        invalid = getattr(gateway, "_last_invalid_odometry", None)
    values = [data.get(name) for name in ("vx", "vy", "vz", "wx", "wy", "wz")]
    if (
        invalid or data.get("twist_available") is not True
        or data.get("frame_id") != "map" or data.get("child_frame_id") != "body"
        or not _finite(values) or not _finite([received_at])
    ):
        _unavailable(context)
    age_ms = (time.time() - received_at) * 1000
    fresh, _ = classify_pose_freshness(diagnostics)
    if (
        not math.isfinite(age_ms) or age_ms < 0 or age_ms > POSE_FRESH_MAX_ODOM_AGE_MS
        or fresh is False or diagnostics.get("source_data_stale") is True
    ):
        _unavailable(context)
    linear = observations_pb.Vector3(x=values[0], y=values[1], z=values[2])
    angular = observations_pb.Vector3(x=values[3], y=values[4], z=values[5])
    return data, linear, angular, age_ms


def velocity(gateway, context):
    data, linear, angular, age_ms = _snapshot(gateway, context)
    return pb.VelocitySample(
        frame_id=data["child_frame_id"], linear_m_s=linear, angular_rad_s=angular,
        received_age_ms=age_ms, fresh=True, source="localization",
    )


def odometry(gateway, context):
    data, linear, angular, age_ms = _snapshot(gateway, context)
    position = [data.get(name) for name in ("x", "y", "z")]
    orientation = data.get("orientation")
    if (
        not _finite(position) or not isinstance(orientation, (list, tuple)) or len(orientation) != 4
        or not _finite(orientation) or abs(math.hypot(*orientation) - 1.0) > 1e-3
    ):
        _unavailable(context)
    return pb.OdometrySample(
        position_m=observations_pb.Vector3(x=position[0], y=position[1], z=position[2]),
        orientation=observations_pb.Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]),
        parent_frame=data["frame_id"], child_frame=data["child_frame_id"],
        linear_velocity_m_s=linear, angular_velocity_rad_s=angular,
        received_age_ms=age_ms, fresh=True, source="localization",
    )
