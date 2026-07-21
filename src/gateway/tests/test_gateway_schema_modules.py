"""Compatibility contracts for the modular Gateway schema implementation."""

from __future__ import annotations

import pickle
from pathlib import Path

import pytest
from pydantic import BaseModel, ValidationError


def test_public_schema_interface_covers_every_domain_and_keeps_pickle_identity():
    import gateway.schemas as schemas

    representatives = {
        "GatewayErrorResponse",
        "RuntimeContractResponse",
        "GoalRequest",
        "SafetyEstopRequest",
        "VoiceTurnRequest",
        "MapLifecycleResponse",
        "SessionTransitionResponse",
        "SceneGraphResponse",
        "SlamStatusResponse",
        "RuntimeDataflowResponse",
        "RuntimeSwitchPlanResponse",
        "AuthLoginRequest",
        "InspectionStartRequest",
        "NavigationStatusResponse",
        "AppBootstrapResponse",
        "DriverSwapResponse",
    }

    assert len(schemas.__all__) == 170
    assert len(set(schemas.__all__)) == len(schemas.__all__)
    assert representatives <= set(schemas.__all__)

    goal = schemas.GoalRequest(x=1.0, y=2.0)
    payload = pickle.dumps(goal)

    assert schemas.GoalRequest.__module__ == "gateway.schemas"
    assert b"gateway.schemas" in payload
    assert b"gateway._schemas" not in payload
    assert pickle.loads(payload) == goal


def test_every_public_pydantic_model_is_complete_and_emits_json_schema():
    import gateway.schemas as schemas

    models = [
        getattr(schemas, name)
        for name in schemas.__all__
        if isinstance(getattr(schemas, name), type)
        and issubclass(getattr(schemas, name), BaseModel)
    ]

    assert len(models) == 163
    for model in models:
        assert model.__module__ == "gateway.schemas"
        assert model.__pydantic_complete__ is True
        generated = model.model_json_schema(
            ref_template="#/components/schemas/{model}",
        )
        assert generated["title"] == model.__name__


def test_cross_domain_json_schema_refs_keep_stable_component_names():
    from gateway.schemas import (
        AppBootstrapResponse,
        ControlCommandResponse,
        RuntimeContractResponse,
    )

    runtime_schema = RuntimeContractResponse.model_json_schema(
        ref_template="#/components/schemas/{model}",
    )
    control_schema = ControlCommandResponse.model_json_schema(
        ref_template="#/components/schemas/{model}",
    )
    app_schema = AppBootstrapResponse.model_json_schema(
        ref_template="#/components/schemas/{model}",
    )

    assert runtime_schema["properties"]["manifest"]["$ref"] == (
        "#/components/schemas/RuntimeContractManifest"
    )
    assert control_schema["properties"]["command"]["$ref"] == (
        "#/components/schemas/CommandReceipt"
    )
    assert control_schema["properties"]["target"]["anyOf"][0]["$ref"] == (
        "#/components/schemas/ConstructedGoalTarget"
    )
    assert app_schema["properties"]["navigation"]["$ref"] == (
        "#/components/schemas/NavigationStatusResponse"
    )


def test_response_extra_policy_and_strict_voice_control_requests_are_preserved():
    from gateway.schemas import HealthResponse, SafetyEstopRequest, VoiceTurnRequest

    health = HealthResponse.model_validate(
        {
            "status": "ok",
            "teleop": {"active": False, "clients": 0},
            "future_gateway_field": {"enabled": True},
        }
    )
    assert health.model_extra == {
        "future_gateway_field": {"enabled": True},
    }

    with pytest.raises(ValidationError):
        VoiceTurnRequest.model_validate(
            {"text": "带我去展厅", "unexpected": True},
        )
    with pytest.raises(ValidationError):
        SafetyEstopRequest.model_validate(
            {"enabled": True, "unexpected": True},
        )


def test_runtime_switch_alias_and_serialization_are_preserved():
    from gateway.schemas import RuntimeSwitchPlanResponse

    response = RuntimeSwitchPlanResponse.model_validate(
        {
            "ok": True,
            "ts": 1.0,
            "from": {"profile": "nav"},
            "current_validation": {"ok": True},
            "target_validation": {"ok": True},
        }
    )

    assert response.from_ == {"profile": "nav"}
    serialized = response.model_dump(by_alias=True)
    assert serialized["from"] == {"profile": "nav"}
    assert "from_" not in serialized


def test_internal_schema_modules_do_not_reverse_import_gateway_runtime_layers():
    schema_dir = Path(__file__).resolve().parents[1] / "_schemas"

    for path in schema_dir.glob("*.py"):
        source = path.read_text(encoding="utf-8")
        assert "gateway.routes" not in source
        assert "gateway.services" not in source
        assert "gateway.gateway_module" not in source
