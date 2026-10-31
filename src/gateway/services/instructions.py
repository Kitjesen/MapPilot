"""Submit one instruction to the Host's existing task owner."""

from __future__ import annotations

import json
from typing import Any


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
