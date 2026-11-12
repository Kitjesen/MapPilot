"""Task pause and resume use the same guarded operations as the Web routes."""

from gateway.navigation.actions import control_task
from gateway.schemas import NavigationTaskPauseRequest, NavigationTaskResumeRequest, NavigationTaskSpeedLimitRequest

from .codec import command, invoke, model, receipt


def control(gateway, request, context, action):
    metadata = command(context, request.command)
    schemas = {"pause": NavigationTaskPauseRequest, "resume": NavigationTaskResumeRequest,
               "speed_limit": NavigationTaskSpeedLimitRequest}
    values = {"max_speed_mps": request.max_speed_mps} if action == "speed_limit" else {"reason": request.reason}
    body = model(context, schemas[action], **metadata, task_id=request.task_id, **values)
    result = invoke(context, control_task, gateway, body, route_task_id=request.task_id, action=action)
    return receipt(context, result, metadata["request_id"])
