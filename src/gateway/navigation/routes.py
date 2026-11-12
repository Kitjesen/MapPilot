"""Navigation HTTP routes: goals, task control, and status."""

from __future__ import annotations

import asyncio
import time
from typing import Any

from gateway.navigation.actions import cancel, control_task, go
from gateway.navigation.commands import invoke_navigation_command
from gateway.navigation.goals import construct_goal_from_request
from gateway.schemas import (
    CancelRequest,
    ClickNavRequest,
    ControlCommandResponse,
    GatewayErrorResponse,
    GoalCandidateRequest,
    GoalCandidateResponse,
    GoalRequest,
    NavigationTaskPauseRequest,
    NavigationTaskResumeRequest,
    NavigationTaskSpeedLimitRequest,
    PlanPreviewRequest,
    PlanPreviewResponse,
    SelectPlannerRequest,
    StopRequest,
)
from gateway.services.control_commands import ControlCommandService
from gateway.services.resume import resume

CONTROL_COMMAND_ERROR_RESPONSES = {
    409: {"model": GatewayErrorResponse},
}

LEASE_ERROR_RESPONSES = {
    403: {"model": GatewayErrorResponse},
    409: {"model": GatewayErrorResponse},
}
GOAL_ERROR_RESPONSES = {
    **LEASE_ERROR_RESPONSES,
    503: {"model": GatewayErrorResponse},
}


from gateway.navigation.status import build_navigation_status
from gateway.navigation.tasks import query_navigation_goal_status, query_navigation_task_status
from gateway.schemas import (
    NavigationGoalStatusQueryResponse,
    NavigationStatusResponse,
    NavigationTaskStatusQueryResponse,
)


def register_navigation_routes(app, gw) -> None:
    """Register navigation commands and queries without changing their public URLs."""
    command_service = ControlCommandService(gw)

    @app.post(
        "/api/v1/navigation/planner",
        summary="Select the local planner after ending the current navigation task",
        response_model=ControlCommandResponse,
        responses=LEASE_ERROR_RESPONSES,
    )
    async def post_navigation_planner(body: SelectPlannerRequest):
        body = body.model_copy(update={"request_id": body.request_id or f"planner-{time.time_ns()}"})
        return await asyncio.to_thread(
            command_service.run_motion_guarded_command, "select_planner", body,
            lambda: invoke_navigation_command(
                gw, "select_planner", required=True, accept_receipt=True,
                name=body.name, request_id=body.request_id,
            ),
        )

    @app.post(
        "/api/v1/navigation/plan",
        summary="Preview navigation plan without publishing a goal",
        response_model=PlanPreviewResponse,
    )
    async def post_navigation_plan(body: PlanPreviewRequest):
        return await asyncio.to_thread(command_service.preview_plan, body)

    @app.post(
        "/api/v1/navigation/goal_candidate",
        summary="Construct and optionally preview a navigation goal without publishing it",
        response_model=GoalCandidateResponse,
    )
    async def post_navigation_goal_candidate(body: GoalCandidateRequest):
        ts = time.time()
        try:
            goal = construct_goal_from_request(
                body,
                gw=gw,
                default_source=body.source,
                default_target_type=body.target_type,
            )
        except ValueError as exc:
            return {
                "schema_version": 1,
                "ok": False,
                "status": "invalid",
                "target": None,
                "preview": None,
                "reasons": [str(exc)],
                "error": str(exc),
                "ts": ts,
            }

        preview = None
        reasons: list[str] = []
        status = "constructed"
        if body.preview:
            preview = await asyncio.to_thread(
                command_service.preview_plan,
                goal.preview_request(),
            )
            reasons = list(preview.get("reasons") or [])
            if preview.get("ok") is not True:
                return {
                    "schema_version": 1,
                    "ok": False,
                    "status": "preview_unavailable",
                    "target": goal.target_payload(ts=ts),
                    "preview": preview,
                    "reasons": reasons,
                    "error": preview.get("error") or "preview_unavailable",
                    "ts": ts,
                }
            status = "preview_feasible" if bool(preview.get("feasible", False)) else "preview_infeasible"

        return {
            "schema_version": 1,
            "ok": True,
            "status": status,
            "target": goal.target_payload(ts=ts),
            "preview": preview,
            "reasons": reasons,
            "error": None,
            "ts": ts,
        }

    @app.post(
        "/api/v1/goal",
        summary="Send navigation goal",
        response_model=ControlCommandResponse,
        responses=GOAL_ERROR_RESPONSES,
    )
    async def post_goal(body: GoalRequest):
        return await asyncio.to_thread(go, gw, body)

    @app.post(
        "/api/v1/navigate/click",
        summary="Navigate to map-viewer click point",
        response_model=ControlCommandResponse,
        responses=GOAL_ERROR_RESPONSES,
    )
    async def post_navigate_click(body: ClickNavRequest):
        return await asyncio.to_thread(go, gw, body, command="navigate_click")

    async def _submit_navigation_cancel(
        body: CancelRequest,
        *,
        route_task_id: str | None = None,
    ):
        return await asyncio.to_thread(cancel, gw, body, route_task_id)

    @app.post(
        "/api/v1/navigation/cancel",
        summary="Compatibility cancel for the current navigation mission",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_navigation_cancel(body: CancelRequest):
        return await _submit_navigation_cancel(body)

    @app.post(
        "/api/v1/navigation/tasks/{task_id}/cancel",
        summary="Request cancellation of one navigation task",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_navigation_task_cancel(task_id: str, body: CancelRequest):
        return await _submit_navigation_cancel(body, route_task_id=task_id)

    async def _submit_navigation_task_control(
        body: NavigationTaskPauseRequest | NavigationTaskResumeRequest | NavigationTaskSpeedLimitRequest,
        *,
        route_task_id: str,
        action: str,
    ) -> Any:
        return await asyncio.to_thread(
            control_task, gw, body, route_task_id=route_task_id, action=action,
        )

    @app.post(
        "/api/v1/navigation/tasks/{task_id}/pause",
        summary="Request a stop-confirmed pause of one navigation task",
        response_model=ControlCommandResponse,
        status_code=202,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_navigation_task_pause(
        task_id: str,
        body: NavigationTaskPauseRequest,
    ):
        return await _submit_navigation_task_control(
            body,
            route_task_id=task_id,
            action="pause",
        )

    @app.post(
        "/api/v1/navigation/tasks/{task_id}/speed-limit",
        summary="Update the active task speed limit while retaining its route",
        response_model=ControlCommandResponse,
        status_code=202,
        responses=LEASE_ERROR_RESPONSES,
    )
    async def post_navigation_task_speed_limit(
        task_id: str, body: NavigationTaskSpeedLimitRequest,
    ):
        return await _submit_navigation_task_control(
            body, route_task_id=task_id, action="speed_limit",
        )

    @app.post(
        "/api/v1/navigation/tasks/{task_id}/resume",
        summary="Request continuation of the same paused navigation task",
        response_model=ControlCommandResponse,
        status_code=202,
        responses=LEASE_ERROR_RESPONSES,
    )
    async def post_navigation_task_resume(
        task_id: str,
        body: NavigationTaskResumeRequest,
    ):
        return await _submit_navigation_task_control(
            body,
            route_task_id=task_id,
            action="resume",
        )

    @app.post(
        "/api/v1/navigation/resume",
        summary="Release a native motion hold without replaying previous motion",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_navigation_resume(body: StopRequest | None = None):
        return await asyncio.to_thread(resume, gw, body)

    @app.get(
        "/api/v1/navigation/status",
        summary="Public navigation status",
        response_model=NavigationStatusResponse,
    )
    async def get_navigation_status():
        return await asyncio.to_thread(build_navigation_status, gw)

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
