from __future__ import annotations

from types import SimpleNamespace

import pytest

pytest.importorskip("fastapi")

from fastapi import FastAPI

from gateway.routes.session import register_session_routes
from gateway.services.session_view import refresh_session_projection


def _gateway(*, mode: str, requires_map: bool = False):
    product = "nav" if mode == "navigating" else "teleop"
    plan = SimpleNamespace(
        product=product,
        lifecycle={
            "product": product,
            "session_mode": mode,
            "slam_mode": "localization" if requires_map else "none",
            "requires_map": requires_map,
        },
    )
    return SimpleNamespace(
        _compiled_run_plan=plan,
        _compiled_env="real",
        _compiled_product=product,
        _compiled_product_session_id="run-1234",
        _session_active_map_name=lambda: "warehouse",
    )


def test_session_projection_comes_from_run_plan_identity_and_mapd():
    gateway = _gateway(mode="navigating", requires_map=True)

    projection = refresh_session_projection(gateway)

    assert projection["mode"] == "navigating"
    assert projection["product"] == "nav"
    assert projection["env"] == "real"
    assert projection["active_map"] == "warehouse"
    assert projection["product_session_id"] == "run-1234"
    assert not hasattr(gateway, "_session_pending")


def test_non_session_product_projects_idle_without_claiming_active_map():
    gateway = _gateway(mode="none")

    projection = refresh_session_projection(gateway)

    assert projection["mode"] == "idle"
    assert projection["product"] == "teleop"
    assert projection["active_map"] is None


def test_session_routes_are_read_only():
    app = FastAPI()
    register_session_routes(app, SimpleNamespace(_session_snapshot=lambda: {}))

    methods = {
        method
        for route in app.routes
        if getattr(route, "path", None) == "/api/v1/session"
        for method in route.methods
    }
    paths = {getattr(route, "path", None) for route in app.routes}

    assert methods == {"GET"}
    assert "/api/v1/session/start" not in paths
    assert "/api/v1/session/end" not in paths
