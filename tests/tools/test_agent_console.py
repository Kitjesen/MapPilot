from unittest.mock import Mock

from fastapi.testclient import TestClient
from tools.diagnostics.agent_console import start_console

from decision.llm import codex
from decision.tasks import conversation_inputs


def test_console_serves_real_accounts_and_conversations_without_robot_writes(tmp_path, monkeypatch):
    monkeypatch.setenv("LINGTU_CONVERSATION_DB_PATH", str(tmp_path / "chat.sqlite3"))
    monkeypatch.setattr(codex, "account", codex.CodexAccount(tmp_path / "codex.json"))
    monkeypatch.setattr(conversation_inputs, "_client", Mock(side_effect=ValueError("unconfigured")))
    host = start_console()
    try:
        with TestClient(host.get_module("GatewayModule")._app) as client:
            response = client.get("/api/v1/agent/connections/codex")
            assert response.status_code == 200 and response.json()["connected"] is False
            assert "no-store" in response.headers["cache-control"]
            options = client.get("/api/v1/agent/options")
            assert options.status_code == 200
            assert {model["provider"] for model in options.json()["models"]} == {
                "moonshot", "qwen", "openai", "claude", "codex"}
            created = client.post("/api/v1/agent/conversations", json={})
            assert created.status_code == 201
            conversation = client.get("/api/v1/agent/conversations/" + created.json()["conversation_id"])
            assert conversation.status_code == 200 and not conversation.json()["messages"]
            refused = client.post("/api/v1/mode", json={"mode": "nav"})
            assert refused.status_code == 409 and refused.json()["error"] == "local_agent_console"
    finally:
        host.stop()
