"""Offline Codex JSON-RPC peer, launched through the real stdio transport."""

import json
import os
import sys
from pathlib import Path

scenario = json.loads(os.environ["CODEX_FIXTURE_SCENARIO"])
log = Path(os.environ["CODEX_FIXTURE_LOG"])
tool_request_id = None
duplicate_sent = False
thread_number = 0
thread_id = ""
turn_id = ""


def emit(message):
    print(json.dumps(message), flush=True)


def tool_request(request_id):
    emit({"id": request_id, "method": "item/tool/call", "params": {
        "threadId": thread_id, "turnId": turn_id, "callId": "call_1",
        "tool": scenario.get("tool", "inspect"), "arguments": scenario.get("arguments", {"target": "map"}),
    }})


for line in sys.stdin:
    message = json.loads(line)
    with log.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(message) + "\n")
    method = message.get("method")
    request_id = message.get("id")
    if method == "initialize":
        emit({"id": request_id, "result": {}})
    elif method == "account/read":
        emit({"id": request_id, "result": {"account": {"type": "chatgpt"}}})
    elif method == "thread/start":
        thread_number += 1
        thread_id, turn_id = f"thread-{thread_number}", f"turn-{thread_number}"
        emit({"id": request_id, "result": {"thread": {"id": thread_id}}})
    elif method == "turn/start":
        emit({"id": request_id, "result": {"turn": {"id": turn_id, "status": "inProgress"}}})
        if scenario.get("late_previous_events") and thread_number > 1:
            emit({"method": "item/agentMessage/delta", "params": {
                "threadId": "thread-1", "turnId": "turn-1", "delta": "OLD CANCELLED REPLY"}})
            emit({"method": "item/started", "params": {
                "threadId": "thread-1", "turnId": "turn-1", "item": {"id": "old-item", "type": "commandExecution"}}})
        tool_request_id = request_id
        tool_request(tool_request_id)
        if scenario.get("duplicate_pending"):
            tool_request("pending-duplicate")
    elif method == "turn/interrupt":
        emit({"id": request_id, "result": {}})
        emit({"method": "turn/completed", "params": {
            "threadId": message["params"]["threadId"],
            "turn": {"id": message["params"]["turnId"], "status": "interrupted"}}})
    elif method is None and request_id in {tool_request_id, "duplicate", "pending-duplicate"}:
        if scenario.get("duplicate") and not duplicate_sent:
            duplicate_sent = True
            tool_request("duplicate")
        elif not scenario.get("wait_for_interrupt") and not (scenario.get("wait_for_interrupt_first") and thread_number == 1):
            emit({"method": "item/agentMessage/delta", "params": {
                "threadId": thread_id, "turnId": turn_id, "delta": scenario.get("reply", "地图已检查")}})
            emit({"method": "turn/completed", "params": {
                "threadId": thread_id, "turn": {"id": turn_id, "status": "completed"}}})
