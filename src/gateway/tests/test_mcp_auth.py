import asyncio
import json

import pytest

from gateway.auth import APIKeyMiddleware


async def _ok_app(scope, receive, send):
    await send({"type": "http.response.start", "status": 200, "headers": []})
    await send({"type": "http.response.body", "body": b"ok"})


async def _request(
    app,
    *,
    headers=None,
    path="/mcp",
    method="GET",
    client=("127.0.0.1", 5050),
):
    sent = []

    async def receive():
        return {"type": "http.request", "body": b"", "more_body": False}

    async def send(message):
        sent.append(message)

    await app(
        {
            "type": "http",
            "path": path,
            "method": method,
            "headers": headers or [],
            "query_string": b"",
            "client": client,
        },
        receive,
        send,
    )
    return sent


@pytest.mark.asyncio
async def test_api_key_middleware_rejects_missing_key_when_required(monkeypatch):
    monkeypatch.delenv("LINGTU_API_KEY", raising=False)
    monkeypatch.setattr("gateway.auth._get_configured_key", lambda: None)
    app = APIKeyMiddleware(_ok_app, api_key=None, require_key=True)

    sent = await _request(app)

    assert sent[0]["status"] == 401


@pytest.mark.asyncio
async def test_api_key_middleware_accepts_valid_key_when_required(monkeypatch):
    monkeypatch.setenv("LINGTU_API_KEY", "secret")
    app = APIKeyMiddleware(_ok_app, require_key=True)

    sent = await _request(app, headers=[(b"x-api-key", b"secret")])

    assert sent[0]["status"] == 200


@pytest.mark.asyncio
async def test_api_key_middleware_rejects_wrong_key_when_required(monkeypatch):
    monkeypatch.setenv("LINGTU_API_KEY", "secret")
    app = APIKeyMiddleware(_ok_app, require_key=True)

    sent = await _request(app, headers=[(b"x-api-key", b"wrong")])

    assert sent[0]["status"] == 403


@pytest.mark.asyncio
async def test_api_key_middleware_keeps_no_key_pass_through_by_default(monkeypatch):
    monkeypatch.delenv("LINGTU_API_KEY", raising=False)
    monkeypatch.setattr("gateway.auth._get_configured_key", lambda: None)
    app = APIKeyMiddleware(_ok_app, api_key=None)

    sent = await _request(app)

    assert sent[0]["status"] == 200


@pytest.mark.asyncio
async def test_required_gateway_keeps_loopback_health_available(monkeypatch):
    monkeypatch.delenv("LINGTU_API_KEY", raising=False)
    monkeypatch.setattr("gateway.auth._get_configured_key", lambda: None)
    app = APIKeyMiddleware(_ok_app, api_key=None, require_key=True)

    sent = await _request(
        app,
        path="/health",
        client=("127.0.0.1", 5050),
    )

    assert sent[0]["status"] == 200


@pytest.mark.asyncio
async def test_required_gateway_keeps_loopback_ready_available(monkeypatch):
    monkeypatch.delenv("LINGTU_API_KEY", raising=False)
    monkeypatch.setattr("gateway.auth._get_configured_key", lambda: None)
    app = APIKeyMiddleware(_ok_app, api_key=None, require_key=True)

    sent = await _request(
        app,
        path="/ready",
        client=("127.0.0.1", 5050),
    )

    assert sent[0]["status"] == 200


@pytest.mark.asyncio
async def test_required_gateway_protects_remote_health(monkeypatch):
    monkeypatch.delenv("LINGTU_API_KEY", raising=False)
    monkeypatch.setattr("gateway.auth._get_configured_key", lambda: None)
    app = APIKeyMiddleware(_ok_app, api_key=None, require_key=True)

    sent = await _request(
        app,
        path="/health",
        client=("192.168.114.50", 5050),
    )

    assert sent[0]["status"] == 401


@pytest.mark.asyncio
async def test_required_gateway_protects_remote_ready(monkeypatch):
    monkeypatch.delenv("LINGTU_API_KEY", raising=False)
    monkeypatch.setattr("gateway.auth._get_configured_key", lambda: None)
    app = APIKeyMiddleware(_ok_app, api_key=None, require_key=True)

    sent = await _request(
        app,
        path="/ready",
        client=("192.168.114.50", 5050),
    )

    assert sent[0]["status"] == 401


@pytest.mark.asyncio
async def test_required_gateway_keeps_loopback_api_protected(monkeypatch):
    monkeypatch.delenv("LINGTU_API_KEY", raising=False)
    monkeypatch.setattr("gateway.auth._get_configured_key", lambda: None)
    app = APIKeyMiddleware(_ok_app, api_key=None, require_key=True)

    sent = await _request(
        app,
        path="/api/v1/state",
        client=("127.0.0.1", 5050),
    )

    assert sent[0]["status"] == 401


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("method", "path"),
    (
        ("POST", "/api/v1/explore/start"),
        ("GET", "/api/v1/explore/status"),
    ),
)
async def test_loopback_explore_operator_accepts_current_product_session(
    monkeypatch,
    method,
    path,
):
    monkeypatch.delenv("LINGTU_API_KEY", raising=False)
    monkeypatch.setattr("gateway.auth._get_configured_key", lambda: None)
    app = APIKeyMiddleware(
        _ok_app,
        api_key=None,
        require_key=True,
        product_session_id="session-token-1234",
    )

    sent = await _request(
        app,
        method=method,
        path=path,
        headers=[(b"x-lingtu-product-session", b"session-token-1234")],
    )

    assert sent[0]["status"] == 200


@pytest.mark.asyncio
async def test_product_session_is_rejected_for_remote_explore_request(monkeypatch):
    monkeypatch.delenv("LINGTU_API_KEY", raising=False)
    monkeypatch.setattr("gateway.auth._get_configured_key", lambda: None)
    app = APIKeyMiddleware(
        _ok_app,
        api_key=None,
        require_key=True,
        product_session_id="session-token-1234",
    )

    sent = await _request(
        app,
        method="POST",
        path="/api/v1/explore/start",
        client=("192.168.66.42", 5050),
        headers=[(b"x-lingtu-product-session", b"session-token-1234")],
    )

    assert sent[0]["status"] == 403


@pytest.mark.asyncio
async def test_product_session_cannot_authorize_other_loopback_api(monkeypatch):
    monkeypatch.delenv("LINGTU_API_KEY", raising=False)
    monkeypatch.setattr("gateway.auth._get_configured_key", lambda: None)
    app = APIKeyMiddleware(
        _ok_app,
        api_key=None,
        require_key=True,
        product_session_id="session-token-1234",
    )

    sent = await _request(
        app,
        method="POST",
        path="/api/v1/goal",
        headers=[(b"x-lingtu-product-session", b"session-token-1234")],
    )

    assert sent[0]["status"] == 403


def test_real_env_gateway_requires_api_key(monkeypatch):
    from gateway.auth import gateway_api_key_required

    monkeypatch.setenv("LINGTU_ENV", "real")
    monkeypatch.delenv("LINGTU_GATEWAY_REQUIRE_API_KEY", raising=False)

    assert gateway_api_key_required() is True


def test_legacy_field_endpoint_does_not_select_real_env(monkeypatch):
    from gateway.auth import gateway_api_key_required

    monkeypatch.setenv("LINGTU_ENV", "sim")
    monkeypatch.setenv("LINGTU_ENDPOINT", "thunder-field")
    monkeypatch.delenv("LINGTU_GATEWAY_REQUIRE_API_KEY", raising=False)

    assert gateway_api_key_required() is False


def test_sim_env_gateway_keeps_auth_optional_by_default(monkeypatch):
    from gateway.auth import gateway_api_key_required

    monkeypatch.setenv("LINGTU_ENV", "sim")
    monkeypatch.delenv("LINGTU_GATEWAY_REQUIRE_API_KEY", raising=False)

    assert gateway_api_key_required() is False


def test_explicit_gateway_auth_requirement_is_honored(monkeypatch):
    from gateway.auth import gateway_api_key_required

    monkeypatch.setenv("LINGTU_ENV", "sim")
    monkeypatch.setenv("LINGTU_GATEWAY_REQUIRE_API_KEY", "1")

    assert gateway_api_key_required() is True


def _mcp_auth_kwargs(app):
    from gateway.auth import APIKeyMiddleware

    for middleware in app.user_middleware:
        if middleware.cls is APIKeyMiddleware:
            return middleware.kwargs
    raise AssertionError("APIKeyMiddleware was not installed")


def _capture_mcp_app(monkeypatch, module):
    captured = {}

    class CaptureServer:
        def __init__(self, config):
            captured["app"] = config.app
            self.should_exit = False
            self.force_exit = False

        def run(self):
            self.should_exit = True

    monkeypatch.setattr("uvicorn.Server", CaptureServer)
    assert module._run_server() is True
    return captured["app"]


def test_mcp_server_requires_key_by_default_on_non_localhost(monkeypatch):
    from gateway.mcp_server import MCPServerModule

    app = _capture_mcp_app(monkeypatch, MCPServerModule(host="0.0.0.0"))

    assert _mcp_auth_kwargs(app)["require_key"] is True


def test_mcp_server_keeps_localhost_dev_pass_through_by_default(monkeypatch):
    from gateway.mcp_server import MCPServerModule

    app = _capture_mcp_app(monkeypatch, MCPServerModule(host="127.0.0.1"))

    assert _mcp_auth_kwargs(app)["require_key"] is False


def _capture_mcp_endpoint(monkeypatch, module):
    app = _capture_mcp_app(monkeypatch, module)
    return next(route.endpoint for route in app.routes if getattr(route, "path", None) == "/mcp")


def test_unknown_mcp_tool_is_invalid_params_error(monkeypatch):
    from gateway.mcp_server import MCPServerModule

    endpoint = _capture_mcp_endpoint(monkeypatch, MCPServerModule(host="127.0.0.1"))
    response = asyncio.run(
        endpoint(
            {
                "jsonrpc": "2.0",
                "id": 7,
                "method": "tools/call",
                "params": {"name": "does_not_exist", "arguments": {}},
            }
        )
    )
    payload = json.loads(response.body)

    assert payload["error"]["code"] == -32602
    assert "result" not in payload


def test_mcp_tool_exception_returns_call_tool_error_result(monkeypatch):
    from gateway.mcp_server import MCPServerModule

    module = MCPServerModule(host="127.0.0.1")

    def explode():
        raise RuntimeError("actuator unavailable")

    module._tool_registry["explode"] = explode
    endpoint = _capture_mcp_endpoint(monkeypatch, module)
    response = asyncio.run(
        endpoint(
            {
                "jsonrpc": "2.0",
                "id": 8,
                "method": "tools/call",
                "params": {"name": "explode", "arguments": {}},
            }
        )
    )
    payload = json.loads(response.body)

    assert payload["result"]["isError"] is True
    error_text = payload["result"]["content"][0]["text"]
    assert "failed" in error_text.lower()
    assert "actuator unavailable" not in error_text


@pytest.mark.parametrize("arguments", [[], {"unexpected": True}])
def test_mcp_tool_rejects_invalid_arguments_before_execution(monkeypatch, arguments):
    from gateway.mcp_server import MCPServerModule

    module = MCPServerModule(host="127.0.0.1")
    calls = []

    def requires_target(target):
        calls.append(target)
        return target

    module._tool_registry["requires_target"] = requires_target
    endpoint = _capture_mcp_endpoint(monkeypatch, module)
    response = asyncio.run(
        endpoint(
            {
                "jsonrpc": "2.0",
                "id": 9,
                "method": "tools/call",
                "params": {"name": "requires_target", "arguments": arguments},
            }
        )
    )
    payload = json.loads(response.body)

    assert payload["error"]["code"] == -32602
    assert calls == []
