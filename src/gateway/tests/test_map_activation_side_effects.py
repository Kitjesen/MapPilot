from __future__ import annotations

import asyncio

import pytest

pytest.importorskip("fastapi")


def test_rejected_map_activation_does_not_clear_live_map_cache() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import MapNameRequest

    gateway = GatewayModule()
    gateway.setup()
    gateway._session_mode = "navigating"
    gateway._session_map = "warehouse"
    cache_resets: list[str] = []
    gateway.clear_map_cloud_cache = lambda *, reason: cache_resets.append(reason)
    endpoint = next(
        route.endpoint for route in gateway._app.routes if route.path == "/api/v1/map/activate"
    )

    response = asyncio.run(endpoint(MapNameRequest(name="office")))

    assert response.status_code == 409
    assert cache_resets == []
