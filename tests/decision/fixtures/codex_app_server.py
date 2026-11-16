"""Offline Codex JSON-RPC peer, launched through the real stdio transport."""

import json
import os
import sys
from pathlib import Path

scenario = json.loads(os.environ["CODEX_FIXTURE_SCENARIO"])
log = Path(os.environ["CODEX_FIXTURE_LOG"])
tool_request_id = None
duplicate_sent = False


def emit(message):
    print(json.dumps(message), flush=True)


def tool_request(request_id):
    emit({"id": request_id, "method": "item/tool/call", "params": {
        "threadId": "thread-1", "turnId": "turn-1", "callId": "call_1",
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
        emit({"id": request_id, "result": {"thread": {"id": "thread-1"}}})
    elif method == "turn/start":
        emit({"id": request_id, "result": {"turn": {"id": "turn-1", "status": "inProgress"}}})
        tool_request_id = request_id
        tool_request(tool_request_id)
        if scenario.get("duplicate_pending"):
            tool_request("pending-duplicate")
    elif method == "turn/interrupt":
        emit({"id": request_id, "result": {}})
        emit({"method": "turn/completed", "params": {
            "threadId": "thread-1", "turn": {"id": "turn-1", "status": "interrupted"}}})
    elif method is None and request_id in {tool_request_id, "duplicate", "pending-duplicate"}:
        if scenario.get("duplicate") and not duplicate_sent:
            duplicate_sent = True
            tool_request("duplicate")
        elif not scenario.get("wait_for_interrupt"):
            emit({"method": "item/agentMessage/delta", "params": {"delta": scenario.get("reply", "地图已检查")}})
            emit({"method": "turn/completed", "params": {
                "threadId": "thread-1", "turn": {"id": "turn-1", "status": "completed"}}})
