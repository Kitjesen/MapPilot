from __future__ import annotations

import pytest
from fastapi.testclient import TestClient

from gateway.gateway_module import GatewayModule


@pytest.fixture
def gateway(monkeypatch):
    monkeypatch.setenv("LINGTU_ENV", "real")
    monkeypatch.setenv("LINGTU_API_KEY", "configured-operator-key")
    monkeypatch.setenv("LINGTU_MAP_API_KEY", "configured-map-key")
    monkeypatch.setenv("LINGTU_GATEWAY_REQUIRE_API_KEY", "1")
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "current-product-session")
    module = GatewayModule()
    module.setup()
    yield module
    module.stop()


@pytest.mark.parametrize("headers", [{}, {"X-API-Key": "old-key", "X-Lingtu-Product-Session": "old-session"}])
def test_gateway_http_has_no_login_gate_even_with_existing_credentials(gateway, headers):
    with TestClient(gateway._app, headers=headers) as client:
        for path in ("/api/v1/state", "/api/v1/localization/status", "/api/v1/navigation/dds_snapshot"):
            response = client.get(path)
            assert response.status_code == 200, response.text
        assert not client.cookies


@pytest.mark.parametrize("path, publish", [("/ws/scan", "publish_scan_frame"), ("/ws/cloud", "publish_cloud_frame")])
def test_gateway_websocket_stream_requires_no_key(gateway, path, publish):
    getattr(gateway._cloud_viewer, publish)(b"current-frame")
    with TestClient(gateway._app) as client, client.websocket_connect(path) as socket:
        assert socket.receive_bytes() == b"current-frame"


def test_removed_login_routes_are_not_advertised_or_replaced_with_fake_success(gateway):
    with TestClient(gateway._app) as client:
        capabilities = client.get("/api/v1/app/capabilities").json()
        bootstrap = client.get("/api/v1/app/bootstrap").json()
        assert "auth" not in capabilities
        assert "auth" not in capabilities["endpoints"]
        assert "auth_login" not in bootstrap["links"]
        assert "auth_check" not in bootstrap["links"]
        paths = gateway._app.openapi()["paths"]
        assert "/api/v1/auth/login" not in paths
        assert "/api/v1/auth/check" not in paths
        assert client.post("/api/v1/auth/login", json={"key": "anything"}).status_code in {404, 405}


def test_open_web_preserves_validation_and_native_motion_stop(gateway):
    published = []
    gateway.instruction.subscribe(published.append)
    gateway._navigation_state = {"authority": "estop", "hold_reason": "operator_estop"}
    with TestClient(gateway._app) as client:
        invalid = client.post("/api/v1/instruction", json={"text": ""})
        assert invalid.status_code == 422
        assert invalid.json()["error"] == "validation_error"
        stopped = client.post("/api/v1/instruction", json={"text": "move forward"})
        assert stopped.status_code == 409
        assert stopped.json()["error"] == "safety_stop"
        assert published == []
