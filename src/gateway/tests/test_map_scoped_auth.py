# ruff: noqa: S101

from __future__ import annotations

import pytest

from gateway.auth import APIKeyMiddleware


async def _ok_app(scope, receive, send):
    del scope, receive
    await send({"type": "http.response.start", "status": 200, "headers": []})
    await send({"type": "http.response.body", "body": b"ok"})


async def _request(
    app,
    *,
    method: str,
    path: str,
    headers: list[tuple[bytes, bytes]] | None = None,
    query_string: bytes = b"",
):
    sent = []

    async def receive():
        return {"type": "http.request", "body": b"", "more_body": False}

    async def send(message):
        sent.append(message)

    await app(
        {
            "type": "http",
            "method": method,
            "path": path,
            "headers": headers or [],
            "query_string": query_string,
        },
        receive,
        send,
    )
    return sent


@pytest.mark.asyncio
async def test_map_key_can_list_maps_through_one_x_api_key_header() -> None:
    app = APIKeyMiddleware(
        _ok_app,
        api_key="operator-secret",
        map_api_key="customer-map-secret",
        require_key=True,
    )

    response = await _request(
        app,
        method="GET",
        path="/api/v1/slam/maps",
        headers=[(b"x-api-key", b"customer-map-secret")],
    )

    assert response[0]["status"] == 200


@pytest.mark.parametrize(
    "method,path",
    [
        ("POST", "/api/v1/map/save"),
        ("GET", "/api/v1/maps/operations"),
        ("GET", "/api/v1/maps/operations/save_123"),
        ("POST", "/api/v1/maps/operations/save_123/cancel"),
        ("POST", "/api/v1/maps/operations/save_123/retry"),
        ("GET", "/api/v1/maps/warehouse.v2/pcd"),
    ],
)
@pytest.mark.asyncio
async def test_map_key_can_use_only_the_customer_map_workflow(method: str, path: str) -> None:
    app = APIKeyMiddleware(
        _ok_app,
        api_key="operator-secret",
        map_api_key="customer-map-secret",
        require_key=True,
    )

    response = await _request(
        app,
        method=method,
        path=path,
        headers=[(b"x-api-key", b"customer-map-secret")],
    )

    assert response[0]["status"] == 200


def test_map_key_must_differ_from_operator_key() -> None:
    with pytest.raises(ValueError, match="must differ"):
        APIKeyMiddleware(
            _ok_app,
            api_key="customer-map-secret",
            map_api_key="customer-map-secret",
        )


@pytest.mark.parametrize(
    "method,path",
    [
        ("GET", "/ws/teleop"),
        ("GET", "/api/v1/health"),
        ("GET", "/api/v1/map/save"),
        ("GET", "/api/v1/maps"),
        ("GET", "/api/v1/maps/operations/bad.id"),
        ("GET", "/api/v1/maps/save-jobs/save_123"),
        ("GET", "/api/v1/maps/warehouse..backup/pcd"),
    ],
)
@pytest.mark.asyncio
async def test_map_key_is_forbidden_from_motion_runtime_and_all_other_routes(method: str, path: str) -> None:
    app = APIKeyMiddleware(
        _ok_app,
        api_key="operator-secret",
        map_api_key="customer-map-secret",
        require_key=True,
    )
    response = await _request(
        app,
        method=method,
        path=path,
        headers=[(b"x-api-key", b"customer-map-secret")],
    )

    assert response[0]["status"] == 403


@pytest.mark.parametrize(
    "headers,query_string",
    [
        ([(b"authorization", b"Bearer customer-map-secret")], b""),
        ([(b"cookie", b"lingtu_api_key=customer-map-secret")], b""),
        ([(b"x-api-key", b"customer-map-secret"), (b"x-api-key", b"customer-map-secret")], b""),
        ([(b"x-api-key", b"")], b"api_key=customer-map-secret"),
    ],
)
@pytest.mark.asyncio
async def test_map_key_rejects_everything_except_one_x_api_key_header(
    headers: list[tuple[bytes, bytes]], query_string: bytes
) -> None:
    app = APIKeyMiddleware(
        _ok_app,
        api_key="operator-secret",
        map_api_key="customer-map-secret",
        require_key=True,
    )
    response = await _request(
        app,
        method="GET",
        path="/api/v1/slam/maps",
        headers=headers,
        query_string=query_string,
    )

    assert response[0]["status"] == 403
