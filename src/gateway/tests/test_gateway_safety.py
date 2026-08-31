from fastapi.testclient import TestClient

from gateway.gateway_module import GatewayModule


def test_estop_reads_navigation_state(monkeypatch):
    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._navigation_state = {"authority": "none", "ts": 123.5}
        gateway._mode = "estop"

    response = TestClient(gateway._app).get("/api/v1/safety/modes/estop")

    assert response.status_code == 200
    assert response.json()["active"] is True
    assert response.json()["timestamp"] == 123.5


def test_estop_uses_native_boundary_and_replays(monkeypatch):
    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    calls: list[tuple[str, str | None]] = []
    monkeypatch.setattr(
        "gateway.routes.safety.native_control.estop",
        lambda _gw, reason, *, request_id=None: calls.append((reason, request_id)) or True,
    )
    gateway = GatewayModule()
    gateway.setup()
    client = TestClient(gateway._app)
    headers = {"X-Request-Id": "estop-request", "X-Operator-Id": "operator"}

    first = client.post("/api/v1/safety/modes/estop", json={"enabled": True}, headers=headers)
    second = client.post("/api/v1/safety/modes/estop", json={"enabled": True}, headers=headers)

    assert first.status_code == second.status_code == 200
    assert first.json()["control_boundary"] == "native_estop"
    assert second.json()["replay"] is True
    assert calls == [("gateway_estop", "estop-request")]


def test_estop_endpoint_cannot_clear_stop(monkeypatch):
    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._mode = "estop"

    response = TestClient(gateway._app).post(
        "/api/v1/safety/modes/estop",
        json={"enabled": False},
        headers={"X-Request-Id": "clear-denied"},
    )

    assert response.status_code == 409
    assert response.json()["active"] is True
    assert gateway._mode == "estop"


def test_estop_openapi_contract_is_typed(monkeypatch):
    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    gateway = GatewayModule()
    gateway.setup()

    path = gateway._app.openapi()["paths"]["/api/v1/safety/modes/estop"]

    assert path["get"]["responses"]["200"]["content"]["application/json"]["schema"]["$ref"].endswith(
        "/SafetyEstopResponse"
    )
    assert path["post"]["requestBody"]["content"]["application/json"]["schema"]["$ref"].endswith(
        "/SafetyEstopRequest"
    )
