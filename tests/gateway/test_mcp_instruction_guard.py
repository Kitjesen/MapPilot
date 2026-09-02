from __future__ import annotations

import json
import threading
from typing import Any

from gateway.services.commands import CommandJournal, run_control_command


class _Port:
    def __init__(self) -> None:
        self.messages: list[Any] = []

    def publish(self, message: Any) -> None:
        self.messages.append(message)


class _Lease:
    def __init__(self, allowed: bool) -> None:
        self.allowed = allowed

    def check(self, _client_id: str) -> bool:
        return self.allowed

    def to_dict(self) -> dict[str, Any]:
        return {"owner": "other"} if not self.allowed else {"owner": "mcp"}


class _Gateway:
    def __init__(self) -> None:
        self.instruction = _Port()
        self._state_lock = threading.RLock()
        self._navigation_state: dict[str, Any] | None = None
        self._lease: _Lease | None = None
        self._command_journal = CommandJournal()
        self.acks: list[tuple[int | None, dict[str, Any]]] = []
        self.events: list[dict[str, Any]] = []

    def _run_control_command(
        self,
        command: str,
        body: Any,
        action: Any,
        *,
        success_status_code: int = 200,
    ) -> dict[str, Any] | Any:
        return run_control_command(
            self,
            command,
            body,
            action,
            success_status_code=success_status_code,
        )

    def _publish_command_ack(
        self,
        payload: dict[str, Any],
        *,
        status_code: int | None = None,
    ) -> None:
        self.acks.append((status_code, payload))

    def push_event(self, event: dict[str, Any]) -> None:
        self.events.append(event)


def test_mcp_send_instruction_uses_gateway_motion_guard_and_returns_submitted_only() -> None:
    from gateway.mcp_server import MCPServerModule

    gateway = _Gateway()
    mcp = MCPServerModule(host="127.0.0.1")
    mcp.on_system_modules({"MCPServerModule": mcp, "GatewayModule": gateway})

    payload = json.loads(mcp.send_instruction("inspect pump three"))

    assert gateway.instruction.messages == ["inspect pump three"]
    assert payload["ok"] is True
    assert payload["accepted"] is True
    assert payload["status"] == "submitted"
    assert payload["stage"] == "submitted"
    assert payload["execution_confirmed"] is False
    assert payload["motor_confirmed"] is False
    assert payload["command"]["client_id"] == "mcp"
    assert payload["command"]["request_id"].startswith("mcp-send_instruction-")


def test_mcp_send_instruction_rejects_malformed_gateway_ack(monkeypatch) -> None:
    from gateway.mcp_server import MCPServerModule
    from gateway.services.control_commands import ControlCommandService

    gateway = _Gateway()
    monkeypatch.setattr(
        ControlCommandService,
        "run_motion_guarded_command",
        lambda *args, **kwargs: {
            "ok": "true",
            "status": "submitted",
            "command": {"accepted": 1},
        },
    )
    mcp = MCPServerModule(host="127.0.0.1")
    mcp.on_system_modules({"MCPServerModule": mcp, "GatewayModule": gateway})

    payload = json.loads(mcp.send_instruction("inspect pump three"))

    assert payload["ok"] is False
    assert payload["accepted"] is False
    assert payload["error"] == "invalid_command_ack"
    assert payload["status"] == "rejected"


def test_mcp_navigate_to_object_rejects_safety_stop_without_publish() -> None:
    from gateway.mcp_server import MCPServerModule

    gateway = _Gateway()
    gateway._navigation_state = {"authority": "estop", "hold_reason": "operator_estop"}
    mcp = MCPServerModule(host="127.0.0.1")
    mcp.on_system_modules({"MCPServerModule": mcp, "GatewayModule": gateway})

    payload = json.loads(mcp.navigate_to_object("red toolbox"))

    assert gateway.instruction.messages == []
    assert payload["ok"] is False
    assert payload["error"] == "safety_stop"
    assert payload["command"]["accepted"] is False
    assert payload["execution_confirmed"] is False


def test_mcp_send_instruction_rejects_lease_conflict_without_publish() -> None:
    from gateway.mcp_server import MCPServerModule

    gateway = _Gateway()
    gateway._lease = _Lease(allowed=False)
    mcp = MCPServerModule(host="127.0.0.1")
    mcp.on_system_modules({"MCPServerModule": mcp, "GatewayModule": gateway})

    payload = json.loads(mcp.send_instruction("go to bay one"))

    assert gateway.instruction.messages == []
    assert payload["ok"] is False
    assert payload["error"] == "control_lease"
    assert payload["command"]["accepted"] is False
    assert payload["execution_confirmed"] is False


def test_mcp_field_instruction_fails_closed_without_gateway(monkeypatch) -> None:
    from gateway import mcp_server
    from gateway.mcp_server import MCPServerModule

    monkeypatch.setattr(mcp_server, "endpoint_only_enabled", lambda _owner=None: True)
    mcp = MCPServerModule(host="127.0.0.1")
    mcp.on_system_modules({"MCPServerModule": mcp})

    payload = json.loads(mcp.send_instruction("move forward"))

    assert payload["ok"] is False
    assert payload["accepted"] is False
    assert payload["error"] == "gateway_unavailable"
    assert payload["execution_confirmed"] is False
    assert payload["motor_confirmed"] is False


def test_mcp_local_compat_instruction_is_explicitly_submitted_only(monkeypatch) -> None:
    from gateway import mcp_server
    from gateway.mcp_server import MCPServerModule

    monkeypatch.setattr(mcp_server, "endpoint_only_enabled", lambda _owner=None: False)
    mcp = MCPServerModule(host="127.0.0.1")
    mcp.on_system_modules({"MCPServerModule": mcp})

    payload = json.loads(mcp.send_instruction("local sim instruction"))

    assert payload["ok"] is True
    assert payload["accepted"] is True
    assert payload["stage"] == "local_compat_submitted"
    assert payload["local_compat_submitted"] is True
    assert payload["execution_confirmed"] is False
    assert payload["motor_confirmed"] is False
