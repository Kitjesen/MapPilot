from decision.llm.codex_app_server import CodexAppServer


def test_missing_cli_is_an_unavailable_connection(tmp_path):
    server = CodexAppServer(command=str(tmp_path / "missing-codex"))
    status = server.status()
    assert status["connected"] is False and status["available"] is False
    assert status["reason"] == "codex_unavailable"
    assert status["auth_mode"] == "app_server" and status["inference_verified"] is False


def test_chat_accepts_string_turn_status(monkeypatch):
    server = CodexAppServer(command="unused")
    observed = []
    events = iter([
        {"id": 1, "result": {"turn": {"status": "inProgress"}}},
        {"method": "item/agentMessage/delta", "params": {"delta": "ok"}},
        {"method": "turn/completed", "params": {
            "threadId": "thread-1", "turn": {"status": "completed", "usage": {"inputTokens": 3}}}},
    ])
    monkeypatch.setattr(server, "_account", lambda: {"account": {"type": "chatgpt"}})
    monkeypatch.setattr(server, "_request", lambda method, params: {
        "thread": {"id": "thread-1"}})
    monkeypatch.setattr(server, "_write", lambda message: None)
    monkeypatch.setattr(server, "_take", lambda timeout=30.0, approval_handler=None, stop=None: next(events))

    assert server.chat([{"role": "user", "content": "hello"}], model="gpt-6.1-sol",
                       on_event=observed.append) == "ok"
    assert observed[-1]["method"] == "turn/usage/updated"


def test_full_access_requests_workspace_write(monkeypatch):
    server = CodexAppServer(command="unused")
    requests = []
    events = iter([
        {"id": 1, "result": {"turn": {"status": "completed"}}},
        {"method": "turn/completed", "params": {
            "threadId": "thread-1", "turn": {"status": "completed"}}},
    ])
    monkeypatch.setattr(server, "_account", lambda: {"account": {"type": "chatgpt"}})

    def request(method, params):
        requests.append((method, params))
        return {"thread": {"id": "thread-1"}}

    monkeypatch.setattr(server, "_request", request)
    monkeypatch.setattr(server, "_write", lambda message: requests.append(("write", message)))
    monkeypatch.setattr(server, "_take", lambda timeout=30.0, approval_handler=None, stop=None: next(events))

    assert server.chat([{"role": "user", "content": "edit"}], model="gpt-6.1-sol",
                       permission_mode="full_access") == ""
    thread = next(params for method, params in requests if method == "thread/start")
    turn = next(params["params"] for method, params in requests
                if method == "write" and params.get("method") == "turn/start")
    assert thread["sandbox"] == turn["sandbox"] == "workspace-write"


def test_approval_request_uses_lingtu_decision(monkeypatch):
    server = CodexAppServer(command="unused")
    replies = []
    monkeypatch.setattr(server, "_respond", lambda request_id, result=None, error=None:
                        replies.append((request_id, result, error)))

    server._handle_server_request(
        {"id": 7, "method": "item/fileChange/requestApproval", "params": {"path": "a.py"}},
        lambda request: "accept",
    )

    assert replies == [(7, {"decision": "accept"}, None)]
