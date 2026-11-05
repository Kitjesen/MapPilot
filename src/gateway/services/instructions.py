"""Submit one instruction to the Host's existing task owner."""

from __future__ import annotations

import json
from typing import Any

from gateway.schemas import AgentTaskCancelRequest, CancelRequest, InstructionRequest
from gateway.services.control_commands import ControlCommandService


def send(gw: Any, body: InstructionRequest):
    def publish() -> dict[str, Any]:
        return submit_instruction(gw, body.text, body.run_id or "", body.control_only)

    if body.control_only or instruction_control(gw, body.text):
        return gw._run_control_command("instruction", body, publish)
    return ControlCommandService(gw).run_motion_guarded_command("instruction", body, publish)


def get_run(gw: Any, run_id: str) -> dict[str, Any]:
    agent = (getattr(gw, "_all_modules", {}) or {}).get("AgentPlannerModule")
    if agent is None:
        return {"found": False, "run_id": run_id, "reason": "agent_unavailable"}
    result = agent.get_agent_status()
    result = json.loads(result) if isinstance(result, str) else result
    if result.get("run_id") != run_id:
        return {"found": False, "run_id": run_id, "reason": "not_current"}
    return {**result, "found": True, "retention": "current_host_run"}


def cancel_run(gw: Any, run_id: str, body: AgentTaskCancelRequest):
    commands = ControlCommandService(gw)
    agent = (getattr(gw, "_all_modules", {}) or {}).get("AgentPlannerModule")
    if agent is None:
        return commands.rejected_response(
            "agent_cancel", body, error="agent_unavailable",
            message="The current Host does not provide Agent tasks.",
            detail=commands.command_error_detail(reason_code="agent_unavailable", source="gateway_modules"),
        )

    def cancel() -> dict[str, Any]:
        result = agent.cancel_agent_task(run_id)
        result = json.loads(result) if isinstance(result, str) else result
        accepted = result.get("cancel_requested") is True
        return {
            **result, "accepted": accepted, "run_id": run_id,
            "status": "cancelling" if accepted else "rejected",
            "stage": "submitted" if accepted else "rejected",
            "execution_confirmed": False,
        }

    journal_body = CancelRequest(task_id=run_id, request_id=body.request_id, client_id=body.client_id)
    return gw._run_control_command("agent_cancel", journal_body, cancel)


def instruction_control(host_adapter: Any, text: str) -> str | None:
    """Ask the task owner to classify controls; Gateway owns no language policy."""
    agent = (getattr(host_adapter, "_all_modules", {}) or {}).get("AgentPlannerModule")
    classify = getattr(agent, "classify_instruction_control", None)
    return classify(text) if callable(classify) else None


def submit_instruction(
    host_adapter: Any, text: str, run_id: str = "", control_only: bool = False,
) -> dict[str, Any]:
    """Prefer Agent admission; a Host without Agent keeps its instruction port."""
    modules = getattr(host_adapter, "_all_modules", {}) or {}
    agent = modules.get("AgentPlannerModule")
    operation = instruction_control(host_adapter, text)
    if control_only and operation is None:
        return {
            "ok": False, "accepted": False, "operation": "start", "reason": "motion_start_blocked",
            "status": "rejected", "stage": "rejected", "executor": "agent" if agent else "semantic",
            "run_id": "", "state": "unavailable", "instruction": text,
            "execution_confirmed": False, "motor_confirmed": False,
        }
    if agent is not None:
        receipt = (agent.handle_instruction_control(text, run_id)
                   if operation else agent.run_agent_task(text))
        receipt = json.loads(receipt) if isinstance(receipt, str) else receipt
        run_id = str(receipt.get("run_id") or "")
        accepted = receipt.get("accepted") is True if "accepted" in receipt else bool(run_id)
        return {
            **receipt,
            "ok": accepted,
            "accepted": accepted,
            "operation": operation or "start",
            "status": ("cancelling" if operation == "cancel" else "ok") if accepted else "rejected",
            "stage": ("handled" if operation else "submitted") if accepted else "rejected",
            "executor": "agent",
            "run_id": run_id,
            "state": receipt.get("state", "unavailable"),
            "instruction": receipt.get("instruction", text),
            "execution_confirmed": False,
            "motor_confirmed": False,
        }
    host_adapter.instruction.publish(text)
    return {
        "ok": True,
        "accepted": True,
        "status": "ok",
        "stage": "submitted",
        "executor": "semantic",
        "operation": "start",
        "instruction": text,
        "execution_confirmed": False,
        "motor_confirmed": False,
    }
