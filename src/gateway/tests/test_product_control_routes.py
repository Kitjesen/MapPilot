from __future__ import annotations

import asyncio
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _payload(response_or_payload):
    if hasattr(response_or_payload, "body"):
        return json.loads(response_or_payload.body)
    return response_or_payload


def _field_run_plan():
    return SimpleNamespace(
        env="real",
        product="nav",
        fingerprint="run-plan-fingerprint",
        process_control="systemd",
        host_config={
            "command_output_mode": "endpoint_only",
            "hardware_control_boundary": "driver",
        },
        processes=(SimpleNamespace(name="slam", target="lingtu-slam-dds.service"),),
    )


def test_native_product_rejects_gateway_hot_switch() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule(run_plan=_field_run_plan())
    gateway.setup()

    response = asyncio.run(
        _endpoint(gateway, "/api/v1/slam/switch")({"profile": "localizer"})
    )

    assert response.status_code == 409
    message = _payload(response)["message"]
    assert "POST /api/v1/runtime/switch-plan" in message
    assert "ProductControl command outside the Host" in message


def test_native_product_restart_returns_operator_control_command() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule(run_plan=_field_run_plan())
    gateway.setup()

    response = asyncio.run(_endpoint(gateway, "/api/v1/slam/restart")())
    payload = _payload(response)

    assert response.status_code == 409
    assert payload["ok"] is False
    assert payload["details"] == {
        "reason_code": "operator_product_control_required",
        "operator_command": (
            "python -m lingtu.control restart --process slam --env real"
        ),
    }


def test_gateway_only_imports_product_control_for_read_only_switch_plan() -> None:
    gateway_root = Path(__file__).resolve().parents[1]
    violations: list[str] = []
    for path in gateway_root.rglob("*.py"):
        if "tests" in path.parts:
            continue
        source = path.read_text(encoding="utf-8")
        if "from lingtu.control import" in source or "import lingtu.control" in source:
            violations.append(path.relative_to(gateway_root).as_posix())

    assert violations == ["services/runtime_switch_plan.py"]
    preview = (gateway_root / violations[0]).read_text(encoding="utf-8")
    assert "preview_control.resolve(" in preview
    assert "product_variant=target_variant" in preview
    for forbidden in (
        "preview_control.switch(",
        "preview_control.stop(",
        "preview_control.restart(",
        "preview_control.apply(",
    ):
        assert forbidden not in preview


def test_native_product_restart_rejects_missing_exact_run_plan() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    response = asyncio.run(_endpoint(gateway, "/api/v1/slam/restart")())

    assert response.status_code == 409
    assert "exact RunPlan" in _payload(response)["message"]
