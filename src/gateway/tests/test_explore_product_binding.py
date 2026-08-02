"""Field Explore task binding and recovery contracts."""

from __future__ import annotations

import asyncio
import json
import os
import threading
import time
from types import SimpleNamespace

import pytest
from fastapi import FastAPI

from gateway.routes.operations import register_operation_routes
from gateway.services import exploration, session_view
from gateway.services.session_view import (
    detect_current_mode,
    recover_external_explore_session,
)


def _plan(*, variant: str = "live", fingerprint: str = "run-plan-fp") -> SimpleNamespace:
    return SimpleNamespace(
        product="explore",
        product_variant=variant,
        env="real",
        process_control="systemd",
        fingerprint=fingerprint,
        lifecycle={
            "product": "explore",
            "product_variant": variant,
            "product_session": "exploration",
            "session_mode": "exploring",
            "slam_mode": "mapping" if variant == "live" else "localization",
            "requires_map": variant == "map",
        },
    )


def _status(
    *,
    session_id: str,
    variant: str = "live",
    active: bool = False,
    paused: bool = False,
) -> dict:
    map_backed = variant == "map"
    return {
        "schema_version": "lingtu.explore.status.v2",
        "endpoint": "lingtu_explore_dds",
        "boot_id": "explore-boot-a",
        "stamp_s": time.time(),
        "route": variant,
        "ready": True,
        "active": active,
        "paused": paused,
        "state": "paused" if paused else ("running" if active else "idle"),
        "session_id": session_id if active else "",
        "pending_goal": None,
        "pending_segment": None,
        "input": {"odometry_age_s": 0.1, "snapshot_age_s": 0.1},
        "map": {
            "frame_id": "map",
            "session_id": session_id,
            "map_id": "yard" if map_backed else "",
            "map_version": 3 if map_backed else 0,
            "artifact_hash": "a" * 64 if map_backed else "",
            "reset_epoch": 2,
            "generation": 9,
            "live": not map_backed,
        },
        "counters": {"odometry_messages": 3, "snapshot_messages": 4},
    }


def _write_current(
    tmp_path,
    *,
    variant: str = "live",
    session_id: str = "session-a",
    map_name: str | None = None,
) -> None:
    map_identity = None
    if variant == "map":
        map_identity = {
            "map_id": "yard",
            "version_id": "yard:v3",
            "frame_id": "map",
            "map_dir": "/maps/yard",
            "artifacts": [
                {
                    "artifact_type": "POINTCLOUD",
                    "uri": "file:///maps/yard/map.pcd",
                    "sha256": "a" * 64,
                    "size_bytes": 123,
                }
            ],
        }
    (tmp_path / "current.json").write_text(
        json.dumps(
            {
                "schema_version": "lingtu.current.v1",
                "product": "explore",
                "product_variant": variant,
                "env": "real",
                "run_plan_path": "/run/lingtu/run-plan.json",
                "fingerprint": "run-plan-fp",
                "product_session_id": session_id,
                **({"map_name": map_name} if map_name is not None else {}),
                "map_identity": map_identity,
                "committed_at": 10.0,
            }
        ),
        encoding="utf-8",
    )


def _gateway(plan: SimpleNamespace | None = None) -> SimpleNamespace:
    return SimpleNamespace(
        _compiled_run_plan=plan,
        _frontier_explorer=None,
        _tare_explorer=None,
        _session_mode="idle",
        _session_product=None,
        _session_product_session="idle",
        _session_map=None,
        _session_slam_profile="stopped",
        _session_since=None,
        _exploring=False,
    )


def _route_endpoint(app: FastAPI, path: str):
    for route in app.routes:
        if getattr(route, "path", None) == path:
            return route.endpoint
    raise AssertionError(f"route not found: {path}")


def test_field_binding_requires_committed_exact_live_identity(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-a")
    monkeypatch.setattr(exploration, "_native_status", lambda: _status(session_id="session-a"))
    gateway = _gateway(_plan())

    missing = exploration.external_explore_binding(gateway)
    assert missing["valid"] is False
    assert "product_activation_not_committed" in missing["blockers"]

    _write_current(tmp_path)
    valid = exploration.external_explore_binding(gateway)
    assert valid["valid"] is True
    assert valid["session_id"] == "session-a"
    assert valid["variant"] == "live"

    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-b")
    mismatch = exploration.external_explore_binding(gateway)
    assert mismatch["valid"] is False
    assert "product_session_mismatch" in mismatch["blockers"]


def test_field_binding_requires_exact_saved_map_identity(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-a")
    monkeypatch.setenv("LINGTU_MAP_ID", "yard")
    monkeypatch.setenv("LINGTU_MAP_VERSION", "yard:v3")
    monkeypatch.setenv("LINGTU_MAP_FRAME", "map")
    monkeypatch.setenv("LINGTU_MAP_POINTCLOUD_SHA256", "a" * 64)
    _write_current(tmp_path, variant="map", map_name="North Yard")
    status = _status(session_id="session-a", variant="map")
    monkeypatch.setattr(exploration, "_native_status", lambda: status)
    gateway = _gateway(_plan(variant="map"))

    assert exploration.external_explore_binding(gateway)["valid"] is True
    assert exploration.external_explore_binding(gateway)["map_name"] == "North Yard"
    status["map"]["artifact_hash"] = "b" * 64
    rejected = exploration.external_explore_binding(gateway)
    assert rejected["valid"] is False
    assert "exploration_map_identity_mismatch" in rejected["blockers"]


def test_field_binding_requires_native_endpoint_boot_identity(
    tmp_path, monkeypatch
) -> None:
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-a")
    _write_current(tmp_path)
    status = _status(session_id="session-a")
    status.pop("boot_id")
    monkeypatch.setattr(exploration, "_native_status", lambda: status)

    rejected = exploration.external_explore_binding(_gateway(_plan()))

    assert rejected["valid"] is False
    assert "native_exploration_boot_identity_missing" in rejected["blockers"]


def test_native_field_start_is_locked_revalidated_and_uses_server_session(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-a")
    _write_current(tmp_path)
    monkeypatch.setattr(exploration, "_native_status", lambda: _status(session_id="session-a"))

    calls: list[tuple[str, str]] = []

    class Commands:
        def start_exploration(self, *, session_id: str, reason: str):
            calls.append((session_id, reason))
            return True

        def stop_exploration(self, *, reason: str):
            return True

    gateway = _gateway(_plan())
    monkeypatch.setattr(exploration, "_native_commands", lambda _gw: Commands())

    assert exploration.begin_exploration(gateway)["accepted"] is True
    assert calls == [("session-a", "gateway_start")]


def test_native_field_start_never_calls_command_before_commit(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-a")
    monkeypatch.setattr(exploration, "_native_status", lambda: _status(session_id="session-a"))
    calls: list[str] = []

    class Commands:
        def start_exploration(self, *, session_id: str, reason: str):
            calls.append(session_id)
            return True

        def stop_exploration(self, *, reason: str):
            return True

    gateway = _gateway(_plan())
    monkeypatch.setattr(exploration, "_native_commands", lambda _gw: Commands())

    with pytest.raises(RuntimeError, match="product_activation_not_committed"):
        exploration.begin_exploration(gateway)
    assert calls == []


def test_native_field_start_rejects_cancellation_window(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-a")
    _write_current(tmp_path)
    status = _status(session_id="session-a")
    status["state"] = "cancelling"
    status["pending_goal"] = {"task_id": "old-task"}
    monkeypatch.setattr(exploration, "_native_status", lambda: status)
    calls: list[str] = []

    class Commands:
        def start_exploration(self, *, session_id: str, reason: str):
            calls.append(session_id)
            return True

        def stop_exploration(self, *, reason: str):
            return True

    gateway = _gateway(_plan())
    monkeypatch.setattr(exploration, "_native_commands", lambda _gw: Commands())

    with pytest.raises(RuntimeError, match="native_exploration_not_idle"):
        exploration.begin_exploration(gateway)
    assert calls == []


@pytest.mark.parametrize("missing_field", ["pending_goal", "pending_segment"])
def test_native_field_start_rejects_incomplete_idle_status(
    tmp_path, monkeypatch, missing_field: str
) -> None:
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-a")
    _write_current(tmp_path)
    status = _status(session_id="session-a")
    del status[missing_field]
    monkeypatch.setattr(exploration, "_native_status", lambda: status)
    calls: list[str] = []

    class Commands:
        def start_exploration(self, *, session_id: str, reason: str):
            calls.append(session_id)
            return True

        def stop_exploration(self, *, reason: str):
            return True

    gateway = _gateway(_plan())
    monkeypatch.setattr(exploration, "_native_commands", lambda _gw: Commands())

    with pytest.raises(RuntimeError, match="native_exploration_not_idle"):
        exploration.begin_exploration(gateway)
    assert calls == []


def test_native_field_start_rejects_product_control_mutation_race(tmp_path, monkeypatch) -> None:
    from lingtu.product_lock import ProductControlLock

    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-a")
    _write_current(tmp_path)
    monkeypatch.setattr(exploration, "_native_status", lambda: _status(session_id="session-a"))

    calls: list[str] = []

    class Commands:
        def start_exploration(self, *, session_id: str, reason: str):
            calls.append(session_id)
            return True

        def stop_exploration(self, *, reason: str):
            return True

    gateway = _gateway(_plan())
    monkeypatch.setattr(exploration, "_native_commands", lambda _gw: Commands())
    locked = threading.Event()
    release = threading.Event()

    def hold_product_control_lock() -> None:
        with ProductControlLock(environment=dict(os.environ), timeout_s=1.0):
            locked.set()
            release.wait(timeout=5.0)

    holder = threading.Thread(target=hold_product_control_lock)
    holder.start()
    assert locked.wait(timeout=2.0)
    try:
        with pytest.raises(RuntimeError, match="product_activation_pending"):
            exploration.begin_exploration(gateway)
    finally:
        release.set()
        holder.join(timeout=2.0)
    assert calls == []


@pytest.mark.parametrize("action", ["set", "clear"])
def test_field_directed_command_rejects_product_control_mutation_race(
    tmp_path, monkeypatch, action: str
) -> None:
    from lingtu.product_lock import ProductControlLock

    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-a")
    _write_current(tmp_path)
    monkeypatch.setattr(
        exploration,
        "_native_status",
        lambda: _status(session_id="session-a", active=True),
    )
    calls: list[tuple[object, ...]] = []

    class Commands:
        def set_directed_exploration_target(self, *args: object) -> bool:
            calls.append(("set", *args))
            return True

        def clear_directed_exploration_target(self, *args: object) -> bool:
            calls.append(("clear", *args))
            return True

    gateway = _gateway(_plan())
    monkeypatch.setattr(exploration, "navigation_commands", lambda _gw: Commands())
    locked = threading.Event()
    release = threading.Event()

    def hold_product_control_lock() -> None:
        with ProductControlLock(environment=dict(os.environ), timeout_s=1.0):
            locked.set()
            release.wait(timeout=5.0)

    holder = threading.Thread(target=hold_product_control_lock)
    holder.start()
    assert locked.wait(timeout=2.0)
    try:
        with pytest.raises(exploration.DirectedExplorationError) as caught:
            if action == "set":
                exploration.directed_exploration_target(
                    gateway,
                    x=1.0,
                    y=2.0,
                    ttl_s=30.0,
                    reason="test",
                    request_id="request-1",
                )
            else:
                exploration.clear_directed_exploration_target(
                    gateway,
                    reason="test",
                    request_id="request-1",
                )
    finally:
        release.set()
        holder.join(timeout=2.0)

    assert caught.value.code == "product_activation_pending"
    assert caught.value.status_code == 409
    assert calls == []


@pytest.mark.parametrize("action", ["set", "clear"])
def test_field_directed_command_uses_one_locked_verified_native_snapshot(
    tmp_path, monkeypatch, action: str
) -> None:
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-a")
    _write_current(tmp_path)
    statuses = iter(
        [
            _status(session_id="session-a", active=True),
            _status(session_id="stale-session", active=True),
        ]
    )
    monkeypatch.setattr(exploration, "_native_status", lambda: next(statuses))
    calls: list[tuple[object, ...]] = []

    class Commands:
        def set_directed_exploration_target(self, *args: object) -> bool:
            calls.append(("set", *args))
            return True

        def clear_directed_exploration_target(self, *args: object) -> bool:
            calls.append(("clear", *args))
            return True

    gateway = _gateway(_plan())
    monkeypatch.setattr(exploration, "navigation_commands", lambda _gw: Commands())

    if action == "set":
        result = exploration.directed_exploration_target(
            gateway,
            x=1.0,
            y=2.0,
            ttl_s=30.0,
            reason="test",
            request_id="request-1",
        )
        assert calls == [("set", 1.0, 2.0, 30.0, "session-a", "test", "request-1")]
    else:
        result = exploration.clear_directed_exploration_target(
            gateway,
            reason="test",
            request_id="request-1",
        )
        assert calls == [("clear", "session-a", "test", "request-1")]

    assert result["intent"]["session_id"] == "session-a"
    assert result["intent"]["frame_id"] == "map"


def test_field_directed_command_fails_closed_when_binding_state_is_unavailable(
    tmp_path, monkeypatch
) -> None:
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-a")
    _write_current(tmp_path)
    monkeypatch.setattr(exploration, "_native_status", lambda: None)
    gateway = _gateway(_plan())

    with pytest.raises(exploration.DirectedExplorationError) as caught:
        exploration.directed_exploration_target(
            gateway,
            x=1.0,
            y=2.0,
            ttl_s=30.0,
            reason="test",
            request_id="request-1",
        )

    assert caught.value.code == "product_activation_state_unavailable"
    assert caught.value.status_code == 503


def test_staged_field_explore_session_is_not_falsely_blocked_as_non_idle(
    tmp_path, monkeypatch
) -> None:
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-a")
    _write_current(tmp_path)
    status = _status(session_id="session-a")
    monkeypatch.setattr(exploration, "_native_status", lambda: status)
    gateway = _gateway(_plan())
    gateway._session_mode = "exploring"
    gateway._session_pending = False
    gateway._exploring = False
    gateway._mode = "normal"
    gateway._odom = object()

    blockers = session_view._exploration_blockers(
        gateway,
        explorer_available=True,
        safety_clear=True,
        localization_status={"state": "ready", "recovery_signal": "NONE"},
        pose_fresh=True,
        algorithm_healthy=True,
    )

    assert "session_not_idle" not in blockers
    assert blockers == []

    safety_blockers = session_view._exploration_blockers(
        gateway,
        explorer_available=True,
        safety_clear=False,
        localization_status={"state": "ready", "recovery_signal": "NONE"},
        pose_fresh=True,
        algorithm_healthy=True,
    )
    assert "safety_stop" in safety_blockers

    status.update(active=True, state="running", session_id="session-a")
    active_native_blockers = session_view._exploration_blockers(
        gateway,
        explorer_available=True,
        safety_clear=True,
        localization_status={"state": "ready", "recovery_signal": "NONE"},
        pose_fresh=True,
        algorithm_healthy=True,
    )
    assert "session_not_idle" in active_native_blockers

    status.update(
        active=False,
        state="cancelling",
        session_id="",
        pending_goal={"task_id": "old-task"},
    )
    cancelling_native_blockers = session_view._exploration_blockers(
        gateway,
        explorer_available=True,
        safety_clear=True,
        localization_status={"state": "ready", "recovery_signal": "NONE"},
        pose_fresh=True,
        algorithm_healthy=True,
    )
    assert "session_not_idle" in cancelling_native_blockers


def test_gateway_restart_recovers_only_verified_external_explore(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-a")
    monkeypatch.setenv("LINGTU_MAP_ID", "yard")
    monkeypatch.setenv("LINGTU_MAP_VERSION", "yard:v3")
    monkeypatch.setenv("LINGTU_MAP_FRAME", "map")
    monkeypatch.setenv("LINGTU_MAP_POINTCLOUD_SHA256", "a" * 64)
    _write_current(tmp_path, variant="map", map_name="North Yard")
    status = _status(session_id="session-a", variant="map", active=True)
    monkeypatch.setattr(exploration, "_native_status", lambda: status)
    gateway = _gateway(_plan(variant="map"))

    assert recover_external_explore_session(gateway) is True
    assert gateway._session_mode == "exploring"
    assert gateway._session_product == "explore"
    assert gateway._session_product_session == "exploration"
    assert gateway._session_map == "North Yard"
    assert gateway._exploring is True

    gateway = _gateway(_plan(variant="map"))
    status["map"]["session_id"] = "stale-session"
    assert recover_external_explore_session(gateway) is False
    assert gateway._session_mode == "idle"
    assert gateway._exploring is False


def test_gateway_restart_recovers_verified_paused_external_explore(
    tmp_path, monkeypatch
) -> None:
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-a")
    _write_current(tmp_path)
    monkeypatch.setattr(
        exploration,
        "_native_status",
        lambda: _status(session_id="session-a", active=True, paused=True),
    )
    gateway = _gateway(_plan())

    assert recover_external_explore_session(gateway) is True
    assert gateway._session_mode == "exploring"
    assert gateway._exploring is True


def test_map_backed_explore_keeps_active_map_in_user_session_state() -> None:
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._session_mode = "exploring"
    gateway._session_map = "yard"

    assert detect_current_mode(gateway) == ("exploring", "yard")
    assert gateway._session_snapshot()["active_map"] == "yard"

    gateway._session_map = None
    assert detect_current_mode(gateway) == ("exploring", None)
    assert gateway._session_snapshot()["active_map"] is None


def test_readiness_recovers_commit_that_landed_after_gateway_setup(tmp_path, monkeypatch) -> None:
    from gateway.services import runtime_status

    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "session-a")
    _write_current(tmp_path)
    monkeypatch.setattr(exploration, "_native_status", lambda: _status(session_id="session-a"))
    monkeypatch.setattr(
        runtime_status,
        "build_navigation_status",
        lambda _gw: {
            "state": "ready",
            "can_accept_goal": True,
            "readiness": {
                "can_execute_autonomy": True,
                "blockers": [],
                "advisories": [],
            },
        },
    )
    gateway = _gateway(_plan())
    gateway._session_pending = False
    gateway._explorer_available = lambda: True

    readiness = exploration.exploration_start_readiness(gateway)

    assert gateway._session_mode == "exploring"
    assert readiness["can_start"] is True
    assert readiness["blockers"] == []


def test_public_field_stop_hands_off_to_product_control() -> None:
    events: list[dict] = []
    gateway = SimpleNamespace(
        _compiled_run_plan=_plan(),
        _go2rtc_upstream="",
        _explorer_stop_available=lambda: True,
        _end_exploration=lambda: (_ for _ in ()).throw(
            AssertionError("Gateway must not claim a field Product is parked")
        ),
        _exploring=True,
        push_event=events.append,
    )
    app = FastAPI()
    register_operation_routes(app, gateway)

    response = asyncio.run(_route_endpoint(app, "/api/v1/explore/stop")())
    payload = json.loads(response.body)

    assert response.status_code == 409
    assert payload["error"] == "product_control_stop_required"
    assert payload["detail"]["operator_command"] == "scripts/lingtu explore stop"
    assert gateway._exploring is True
    assert events == []
