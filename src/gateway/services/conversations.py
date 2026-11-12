"""Conversation transport delegates history and task lifecycle to the Agent owner."""

from __future__ import annotations

import json

from fastapi import HTTPException
from fastapi.responses import JSONResponse

from gateway.services.control_commands import ControlCommandService
from gateway.services.conversation_tools import workbench_tools


def owner(gw):
    agent = (getattr(gw, "_all_modules", {}) or {}).get("AgentPlannerModule")
    if agent is None:
        raise HTTPException(503, detail={"reason": "agent_unavailable"})
    return agent


def call(gw, method, *args, **kwargs):
    try:
        return getattr(owner(gw), method)(*args, **kwargs)
    except ValueError as exc:
        reason = str(exc)
        status = (503 if reason == "agent_unavailable" else 404 if reason.endswith("not_found")
                  else 422 if reason == "share_empty"
                  else 409 if reason.endswith("conflict") or reason in {
                      "turn_active", "approval_closed", "conversation_active", "conversation_archived"
                  } else 400)
        raise HTTPException(status, detail={"reason": reason}) from exc


def submit(gw, conversation_id, body):
    tools, invoke = workbench_tools(gw)
    def guard(admit, tool_call_id):
        result = ControlCommandService(gw).run_motion_guarded_command("agent_tool", body, admit)
        return json.loads(result.body) if isinstance(result, JSONResponse) else result

    return call(gw, "submit_conversation_message", conversation_id, body.message_id, body.text,
                body.reply_to_message_id, reply_quote=body.reply_quote, motion_guard=guard, model=body.model,
                reasoning=body.reasoning, attachment_ids=body.attachment_ids, permission_mode=body.permission_mode, persona=body.persona, enqueue=body.enqueue,
                domain_tools=tools, invoke_domain_tool=invoke,
                map_context=body.map_context.model_dump(mode="json") if body.map_context else None)
