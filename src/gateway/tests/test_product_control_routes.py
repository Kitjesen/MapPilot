from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("fastapi")


def _field_run_plan():
    return SimpleNamespace(
        env="real",
        product="nav",
        product_session_id="1" * 32,
        run_plan_path="/run/lingtu/plans/plan-session.json",
        process_control="systemd",
        host_config={
            "command_output_mode": "endpoint_only",
            "hardware_control_boundary": "driver",
        },
        processes=(SimpleNamespace(name="slam", target="lt-slam.service"),),
    )


def test_gateway_exposes_no_product_switch_route() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule(run_plan=_field_run_plan())
    gateway.setup()
    paths = {route.path for route in gateway._app.routes}

    assert "/api/v1/slam/switch" not in paths
    assert "/api/v1/runtime/switch" not in paths


def test_gateway_does_not_expose_a_slam_restart_route() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule(run_plan=_field_run_plan())
    gateway.setup()

    assert "/api/v1/slam/restart" not in {
        route.path for route in gateway._app.routes
    }


def test_gateway_product_control_imports_are_read_only() -> None:
    gateway_root = Path(__file__).resolve().parents[1]
    violations: list[str] = []
    for path in gateway_root.rglob("*.py"):
        if "tests" in path.parts:
            continue
        source = path.read_text(encoding="utf-8")
        if "from lingtu.control import" in source or "import lingtu.control" in source:
            violations.append(path.relative_to(gateway_root).as_posix())

    assert violations == ["services/recording.py"]
    for forbidden in (
        ".stop(",
        ".restart(",
        ".apply(",
    ):
        for path in violations:
            source = (gateway_root / path).read_text(encoding="utf-8")
            assert forbidden not in source
