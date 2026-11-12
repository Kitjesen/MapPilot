"""Expose native plan previews without submitting motion commands."""

import grpc

from gateway.schemas import PlanPreviewRequest
from gateway.services.control_commands import ControlCommandService

from .codec import invoke, model, pose
from .generated import navigation_pb2 as navigation_pb
from .generated import observations_pb2 as observations_pb


def preview(gateway, request, context):
    if not request.HasField("pose"):
        context.abort(grpc.StatusCode.INVALID_ARGUMENT, "pose is required")
    coordinates = pose(request.pose)
    coordinates.pop("yaw")
    body = model(context, PlanPreviewRequest, **coordinates, frame_id="map",
                 acceptance_radius_m=request.acceptance_radius_m if request.HasField("acceptance_radius_m") else None)
    result = invoke(context, ControlCommandService(gateway).preview_plan, body)
    path = navigation_pb.NavigationPath(frame_id=result.get("frame_id") or "map")
    for point in result.get("path") or []:
        path.points.add(x=point["x"], y=point["y"], z=point["z"])
    reply = navigation_pb.PlanPreview(
        ok=result.get("ok") is True, feasible=result.get("feasible") is True,
        path=path, planner=result.get("planner") or "", reasons=result.get("reasons") or [],
    )
    for name in ("start", "goal"):
        if point := result.get(name):
            getattr(reply, name).CopyFrom(observations_pb.Vector3(**{key: point[key] for key in ("x", "y", "z")}))
    for name in ("distance_m", "plan_ms"):
        if result.get(name) is not None:
            setattr(reply, name, result[name])
    return reply
