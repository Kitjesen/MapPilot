import pytest

from gateway.auth import APIKeyMiddleware


async def _ok_app(scope, receive, send):
    await send({"type": "http.response.start", "status": 200, "headers": []})
    await send({"type": "http.response.body", "body": b"ok"})


async def _request(
    app,
    *,
    method: str,
    path: str,
    key: str | None = None,
    cookie: str | None = None,
    query_string: bytes = b"",
):
    sent = []

    async def receive():
        return {"type": "http.request", "body": b"", "more_body": False}

    async def send(message):
        sent.append(message)

    headers = [(b"x-api-key", key.encode())] if key is not None else []
    if cookie:
        headers.append((b"cookie", cookie.encode()))

    await app(
        {
            "type": "http",
            "method": method,
            "path": path,
            "headers": headers,
            "query_string": query_string,
        },
        receive,
        send,
    )
    return sent


@pytest.mark.asyncio
async def test_rmf_scoped_key_allows_only_high_level_navigation_routes() -> None:
    app = APIKeyMiddleware(
        _ok_app,
        api_key="operator-secret",
        rmf_api_key="rmf-secret",
        require_key=True,
    )

    allowed = await _request(
        app,
        method="POST",
        path="/api/v1/goal",
        key="rmf-secret",
    )
    blocked = await _request(
        app,
        method="POST",
        path="/api/v1/cmd_vel",
        key="rmf-secret",
    )

    assert allowed[0]["status"] == 200
    assert blocked[0]["status"] == 403


@pytest.mark.asyncio
async def test_rmf_scoped_key_rejects_query_parameter_auth() -> None:
    app = APIKeyMiddleware(
        _ok_app,
        api_key="operator-secret",
        rmf_api_key="rmf-secret",
        require_key=True,
    )

    response = await _request(
        app,
        method="POST",
        path="/api/v1/goal",
        query_string=b"api_key=rmf-secret",
    )

    assert response[0]["status"] == 403


@pytest.mark.asyncio
async def test_rmf_scoped_key_rejects_query_auth_with_empty_header() -> None:
    app = APIKeyMiddleware(
        _ok_app,
        api_key="operator-secret",
        rmf_api_key="rmf-secret",
        require_key=True,
    )

    response = await _request(
        app,
        method="POST",
        path="/api/v1/goal",
        key="",
        query_string=b"api_key=rmf-secret",
    )

    assert response[0]["status"] == 403


@pytest.mark.asyncio
async def test_rmf_scoped_key_rejects_cookie_auth() -> None:
    app = APIKeyMiddleware(
        _ok_app,
        api_key="operator-secret",
        rmf_api_key="rmf-secret",
        require_key=True,
    )

    response = await _request(
        app,
        method="POST",
        path="/api/v1/goal",
        cookie="lingtu_api_key=rmf-secret",
    )

    assert response[0]["status"] == 403


@pytest.mark.asyncio
async def test_operator_key_keeps_full_gateway_access() -> None:
    app = APIKeyMiddleware(
        _ok_app,
        api_key="operator-secret",
        rmf_api_key="rmf-secret",
        require_key=True,
    )

    response = await _request(
        app,
        method="POST",
        path="/api/v1/cmd_vel",
        key="operator-secret",
    )

    assert response[0]["status"] == 200


def test_rmf_scoped_key_must_differ_from_operator_key() -> None:
    with pytest.raises(ValueError, match="must differ"):
        APIKeyMiddleware(
            _ok_app,
            api_key="shared-secret",
            rmf_api_key="shared-secret",
            require_key=True,
        )
