from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]
pytest.importorskip("fastapi")


def test_voice_turn_accepts_askme_payload_and_publishes_trimmed_unicode(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    gateway = GatewayModule()
    gateway.setup()
    published: list[str] = []
    gateway.instruction.subscribe(published.append)

    response = TestClient(gateway._app).post(
        "/api/v1/voice/turns",
        json={
            "text": "  前往三号泵站  ",
            "operator_id": "askme.voice",
            "session_id": "voice-session-1",
            "channel": "voice",
            "robot_id": "dog-01",
            "site_id": "plant-a",
            "submit": True,
            "metadata": {"locale": "zh-CN"},
        },
        headers={
            "X-Request-Id": "voice-request-1",
            "Idempotency-Key": "voice-turn-fallback-id",
            "X-Operator-Id": "askme.voice",
        },
    )

    assert response.status_code == 200
    payload = response.json()
    assert payload["handled"] is True
    assert payload["turn"]["action_type"] == "runtime"
    assert payload["turn"]["spoken_reply"].strip()
    assert payload["turn"]["text"] == "前往三号泵站"
    assert payload["turn"]["submitted"] is True
    assert payload["turn"]["status"] == "submitted"
    assert payload["turn"]["request_id"] == "voice-request-1"
    assert payload["turn"]["client_id"] == "askme.voice"
    assert payload["turn"]["session_id"] == "voice-session-1"
    assert payload["turn"]["metadata"] == {"locale": "zh-CN"}
    assert published == ["前往三号泵站"]


@pytest.mark.parametrize("malformed_ack", ["true", 1])
def test_voice_turn_does_not_coerce_truthy_non_boolean_command_ack(
    monkeypatch,
    malformed_ack,
):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule
    from gateway.services.control_commands import ControlCommandService

    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    monkeypatch.setattr(
        ControlCommandService,
        "run_motion_guarded_command",
        lambda *args, **kwargs: {
            "ok": malformed_ack,
            "submitted": malformed_ack,
            "status": "submitted",
            "command": {
                "name": "instruction",
                "accepted": malformed_ack,
                "replay": False,
                "request_id": "voice-malformed-ack",
                "client_id": "askme.voice",
                "ts": 1.0,
            },
        },
    )
    gateway = GatewayModule()
    gateway.setup()

    response = TestClient(gateway._app).post(
        "/api/v1/voice/turns",
        json={"text": "move forward", "operator_id": "askme.voice"},
        headers={"X-Request-Id": "voice-malformed-ack"},
    )

    assert response.status_code == 200
    turn = response.json()["turn"]
    assert turn["accepted"] is False
    assert turn["submitted"] is False


def test_voice_turn_rejects_mismatched_operator_identity(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    gateway = GatewayModule()
    gateway.setup()

    response = TestClient(gateway._app).post(
        "/api/v1/voice/turns",
        json={"text": "去六层某公司", "operator_id": "body-operator"},
        headers={"X-Operator-Id": "header-operator"},
    )

    assert response.status_code == 422
    assert gateway.instruction.msg_count == 0


def test_voice_turn_replays_idempotency_key_without_republishing(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    gateway = GatewayModule()
    gateway.setup()
    published: list[str] = []
    gateway.instruction.subscribe(published.append)
    client = TestClient(gateway._app)
    request = {
        "text": "inspect the loading bay",
        "operator_id": "askme.operator",
        "session_id": "voice-session-2",
        "channel": "voice",
        "robot_id": "dog-01",
        "site_id": "plant-a",
        "submit": True,
        "metadata": {},
    }
    headers = {"Idempotency-Key": "voice-turn-idempotent-1"}

    first = client.post("/api/v1/voice/turns", json=request, headers=headers)
    second = client.post("/api/v1/voice/turns", json=request, headers=headers)

    assert first.status_code == second.status_code == 200
    assert first.json()["turn"]["replay"] is False
    assert second.json()["turn"]["replay"] is True
    assert second.json()["turn"]["request_id"] == "voice-turn-idempotent-1"
    assert second.json()["turn"]["spoken_reply"].strip()
    assert published == ["inspect the loading bay"]


def test_voice_turn_rejects_changed_payload_for_the_same_idempotency_key(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    gateway = GatewayModule()
    gateway.setup()
    published: list[str] = []
    gateway.instruction.subscribe(published.append)
    client = TestClient(gateway._app)
    headers = {
        "Idempotency-Key": "voice-turn-conflict",
        "X-Operator-Id": "askme.operator",
    }

    first = client.post(
        "/api/v1/voice/turns",
        json={"text": "inspect bay one", "submit": True},
        headers=headers,
    )
    conflict = client.post(
        "/api/v1/voice/turns",
        json={"text": "inspect bay two", "submit": True},
        headers=headers,
    )

    assert first.status_code == 200
    assert conflict.status_code == 409
    assert conflict.json()["error"] == "idempotency_conflict"
    assert conflict.json()["command"]["accepted"] is False
    assert conflict.json()["command"]["client_id"] == "askme.operator"
    assert published == ["inspect bay one"]


def test_voice_turn_replay_precedes_changed_safety_state(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    gateway = GatewayModule()
    gateway.setup()
    published: list[str] = []
    gateway.instruction.subscribe(published.append)
    client = TestClient(gateway._app)
    request = {
        "text": "去六层某公司",
        "operator_id": "askme.voice",
        "submit": True,
    }
    headers = {"Idempotency-Key": "voice-turn-before-estop"}

    first = client.post("/api/v1/voice/turns", json=request, headers=headers)
    with gateway._state_lock:
        gateway._safety = {"level": 2, "ts": 123.0}
    replay = client.post("/api/v1/voice/turns", json=request, headers=headers)

    assert first.status_code == replay.status_code == 200
    assert replay.json()["turn"]["accepted"] is True
    assert replay.json()["turn"]["replay"] is True
    assert replay.json()["turn"]["status"] == "submitted"
    assert published == ["去六层某公司"]


def test_voice_turn_submit_false_is_a_handled_nonexecuting_preview(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    gateway = GatewayModule()
    gateway.setup()

    response = TestClient(gateway._app).post(
        "/api/v1/voice/turns",
        json={
            "text": "preview a patrol request",
            "submit": False,
            "request_id": "body-preview-request",
            "client_id": "preview-client",
        },
    )

    assert response.status_code == 200
    turn = response.json()["turn"]
    assert turn["spoken_reply"].strip()
    assert turn["status"] == "preview"
    assert turn["submitted"] is False
    assert turn["accepted"] is True
    assert turn["request_id"] == "body-preview-request"
    assert turn["client_id"] == "preview-client"
    assert gateway.instruction.msg_count == 0


def test_voice_turn_safety_rejection_is_handled_without_publish(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._safety = {"level": 2, "ts": 123.0}

    response = TestClient(gateway._app).post(
        "/api/v1/voice/turns",
        json={
            "text": "move forward",
            "operator_id": "askme.voice",
            "submit": True,
        },
        headers={"X-Request-Id": "voice-safety-blocked"},
    )

    assert response.status_code == 200
    payload = response.json()
    assert payload["handled"] is True
    assert payload["turn"]["status"] == "rejected"
    assert payload["turn"]["accepted"] is False
    assert payload["turn"]["submitted"] is False
    assert "safety stop" in payload["turn"]["spoken_reply"].lower()
    assert gateway.instruction.msg_count == 0


def test_voice_turn_lease_rejection_is_handled_without_publish(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    gateway = GatewayModule()
    gateway.setup()
    assert gateway._lease.acquire("operator-a", 30.0) is True

    response = TestClient(gateway._app).post(
        "/api/v1/voice/turns",
        json={
            "text": "start patrol",
            "operator_id": "operator-b",
            "submit": True,
        },
        headers={"X-Request-Id": "voice-lease-blocked"},
    )

    assert response.status_code == 200
    payload = response.json()
    assert payload["handled"] is True
    assert payload["turn"]["status"] == "rejected"
    assert payload["turn"]["accepted"] is False
    assert payload["turn"]["submitted"] is False
    assert "another operator" in payload["turn"]["spoken_reply"].lower()
    assert payload["turn"]["client_id"] == "operator-b"
    assert gateway.instruction.msg_count == 0


def test_estop_get_combines_safety_state_and_gateway_mode(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._safety = {"level": 0, "ts": 123.5}
        gateway._mode = "estop"

    response = TestClient(gateway._app).get("/api/v1/safety/modes/estop")

    assert response.status_code == 200
    assert response.json()["active"] is True
    assert response.json()["enabled"] is True
    assert response.json()["timestamp"] == 123.5


def test_estop_post_is_idempotent_and_uses_local_compatibility(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    gateway = GatewayModule()
    gateway.setup()
    client = TestClient(gateway._app)
    headers = {
        "X-Request-Id": "askme-estop-request",
        "X-Operator-Id": "askme.voice",
    }

    first = client.post(
        "/api/v1/safety/modes/estop",
        json={"enabled": True},
        headers=headers,
    )
    second = client.post(
        "/api/v1/safety/modes/estop",
        json={"enabled": True},
        headers=headers,
    )

    assert first.status_code == second.status_code == 200
    assert first.json()["active"] is True
    assert first.json()["control_boundary"] == "local_compat"
    assert second.json()["replay"] is True
    assert gateway.stop_cmd.msg_count == 1
    assert gateway.cmd_vel.msg_count == 1
    assert gateway._mode == "estop"


def test_estop_false_is_rejected_without_clearing_or_publishing(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._mode = "estop"

    response = TestClient(gateway._app).post(
        "/api/v1/safety/modes/estop",
        json={"enabled": False},
        headers={"X-Request-Id": "askme-clear-denied"},
    )

    assert response.status_code == 409
    assert response.json()["accepted"] is False
    assert response.json()["active"] is True
    assert "cannot clear" in response.json()["message"].lower()
    assert gateway._mode == "estop"
    assert gateway.stop_cmd.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0


def test_voice_and_estop_openapi_contracts_are_typed(monkeypatch):
    from gateway.gateway_module import GatewayModule

    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    gateway = GatewayModule()
    gateway.setup()
    openapi = gateway._app.openapi()

    voice_post = openapi["paths"]["/api/v1/voice/turns"]["post"]
    estop_path = openapi["paths"]["/api/v1/safety/modes/estop"]
    assert voice_post["requestBody"]["content"]["application/json"]["schema"]["$ref"].endswith("/VoiceTurnRequest")
    assert voice_post["responses"]["200"]["content"]["application/json"]["schema"]["$ref"].endswith(
        "/VoiceTurnResponse"
    )
    assert voice_post["responses"]["409"]["content"]["application/json"]["schema"]["$ref"].endswith(
        "/GatewayErrorResponse"
    )
    assert estop_path["get"]["responses"]["200"]["content"]["application/json"]["schema"]["$ref"].endswith(
        "/SafetyEstopResponse"
    )
    assert estop_path["post"]["requestBody"]["content"]["application/json"]["schema"]["$ref"].endswith(
        "/SafetyEstopRequest"
    )


def test_voice_and_estop_reject_oversized_identity_headers(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    gateway = GatewayModule()
    gateway.setup()
    client = TestClient(gateway._app)
    oversized = "x" * 129

    voice = client.post(
        "/api/v1/voice/turns",
        json={"text": "去六层某公司"},
        headers={"X-Request-Id": oversized},
    )
    estop = client.post(
        "/api/v1/safety/modes/estop",
        json={"enabled": True},
        headers={"X-Operator-Id": oversized},
    )

    assert voice.status_code == 422
    assert estop.status_code == 422
    assert gateway.instruction.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0
