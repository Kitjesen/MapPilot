from __future__ import annotations

import asyncio

import pytest

from gateway.auth import APIKeyMiddleware


async def _ok_app(scope, receive, send):
    del scope, receive
    await send({"type": "http.response.start", "status": 200, "headers": []})
    await send({"type": "http.response.body", "body": b"ok"})


async def _request(headers):
    sent = []

    async def receive():
        return {"type": "http.request", "body": b"", "more_body": False}

    async def send(message):
        sent.append(message)

    app = APIKeyMiddleware(_ok_app, api_key="shared-secret", require_key=True)
    await app(
        {
            "type": "http",
            "method": "POST",
            "path": "/api/v1/safety/modes/estop",
            "headers": headers,
            "query_string": b"",
            "client": ("127.0.0.1", 5050),
        },
        receive,
        send,
    )
    return sent


@pytest.mark.asyncio
async def test_bearer_key_is_accepted_for_gateway_service_call():
    sent = await _request([(b"authorization", b"Bearer shared-secret")])

    assert sent[0]["status"] == 200


@pytest.mark.asyncio
async def test_bearer_scheme_is_case_insensitive():
    sent = await _request([(b"authorization", b"bearer shared-secret")])

    assert sent[0]["status"] == 200


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "authorization",
    [b"Basic shared-secret", b"Bearer", b"Bearer wrong-secret"],
)
async def test_invalid_authorization_does_not_bypass_gateway_auth(authorization):
    sent = await _request([(b"authorization", authorization)])

    assert sent[0]["status"] in {401, 403}


@pytest.mark.asyncio
async def test_explicit_x_api_key_keeps_precedence_over_bearer():
    sent = await _request(
        [
            (b"x-api-key", b"wrong-secret"),
            (b"authorization", b"Bearer shared-secret"),
        ]
    )

    assert sent[0]["status"] == 403


def test_auth_login_accepts_configured_key(monkeypatch):
    from fastapi import FastAPI

    from gateway.routes.auth import register_auth_routes
    from gateway.schemas import AuthLoginRequest

    monkeypatch.setattr("gateway.auth._get_configured_key", lambda: "shared-secret")
    app = FastAPI()
    register_auth_routes(app)
    endpoint = next(route.endpoint for route in app.routes if route.path == "/api/v1/auth/login")

    response = asyncio.run(endpoint(AuthLoginRequest(key="shared-secret")))

    assert response.status_code == 200
    assert "lingtu_api_key=shared-secret" in response.headers["set-cookie"]
