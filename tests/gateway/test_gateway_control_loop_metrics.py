from __future__ import annotations

import asyncio
import json
import time

import pytest

pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def test_metrics_surface_native_control_loop_health(monkeypatch, tmp_path):
    from gateway.gateway_module import GatewayModule

    status_path = tmp_path / "nav_endpoint_status.json"
    status_path.write_text(
        json.dumps(
            {
                "stamp_s": time.time(),
                "tick_hz": 20.0,
                "control_loop_health": {
                    "ready": True,
                    "healthy": True,
                    "reason": "healthy",
                    "window_samples": 600,
                    "deadline_miss_ratio": 0.01,
                    "p95_utilization": 0.42,
                },
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(status_path))

    gateway = GatewayModule()
    gateway.setup()

    metrics = asyncio.run(_endpoint(gateway, "/api/v1/metrics")())

    assert metrics["navigation"]["tick_hz"] == 20.0
    assert metrics["navigation"]["control_loop_health"] == {
        "ready": True,
        "healthy": True,
        "reason": "healthy",
        "window_samples": 600,
        "deadline_miss_ratio": 0.01,
        "p95_utilization": 0.42,
    }

