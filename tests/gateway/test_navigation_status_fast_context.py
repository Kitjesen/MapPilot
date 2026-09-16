from __future__ import annotations

import json
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from gateway.gateway_module import GatewayModule
from gateway.navigation.status import evaluate_navigation_gate, handle_navigation_state
from gateway.services.runtime_status import compiled_session_context


@pytest.fixture
def native_gateway(monkeypatch, tmp_path):
    gateway = GatewayModule()
    parameter_names = (
        "PATH_FOLLOWER_MAX_SPEED_MPS",
        "PATH_FOLLOWER_MIN_SPEED_MPS",
        "PATH_FOLLOWER_MAX_ACCEL_MPS2",
        "PATH_FOLLOWER_LOOKAHEAD_M",
        "PATH_FOLLOWER_GOAL_TOLERANCE_M",
        "WAYPOINT_REACHED_M",
        "GOAL_REACHED_M",
        "CORRIDOR_LOOKAHEAD_M",
        "TELEOP_PLANNER_HORIZON_M",
        "TELEOP_PLANNER_MAX_DEVIATION_DEG",
    )
    gateway._compiled_run_plan = SimpleNamespace(
        product="nav",
        lifecycle={"product": "nav", "session_mode": "navigating", "slam_mode": "localization", "requires_map": True},
        native_nav={"control_mode": "autonomy", "global_planner": "far"},
        required_capabilities=(),
        native_process_environment={
            (f"LINGTU_{name}" if name.startswith("TELEOP_") else f"LINGTU_NAV_{name}"): "1.0"
            for name in parameter_names
        },
        has_process=lambda role: role == "nav",
    )
    gateway._compiled_product = "nav"
    gateway._compiled_env = "real"
    gateway._compiled_product_session_id = "product-joint-display"
    gateway._session_mode = "idle"
    gateway._session_snapshot = Mock(side_effect=AssertionError("full session read on telemetry path"))
    gateway._map_client = SimpleNamespace(service=Mock(side_effect=AssertionError("map query on telemetry path")))
    gateway._odom = {"frame_id": "map", "x": 0.0, "y": 0.0}
    gateway._odom_timestamps.append(time.time())
    gateway._localization_status = {"state": "TRACKING", "pose_fresh": True}
    gateway._mode = "autonomous"
    events = []
    gateway.push_event = events.append
    status_path = tmp_path / "nav.status.json"
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(status_path))
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "2")
    snapshot = {
        "stamp_s": time.time(),
        "native_product": {"product": "nav"},
        "control_loop_health": {"ready": True, "reason": "healthy"},
        "input_gate": {"ready": True},
        "active_cmd_source": "autonomy",
        "control_authority": {"owner": "native_endpoint"},
        "control_mode": "autonomy",
        "global_planner": "far",
        "planner_map": "/maps/yard/octomap.ot",
        "far_input": {"required": True, "ready": True, "map_id": "yard", "content_epoch": 7},
        "publish_cmd_vel": True,
        "navigation_ready": True,
        "path_follower": dict.fromkeys(
            ("max_speed_mps", "min_speed_mps", "max_accel_mps2", "lookahead_m", "goal_tolerance_m"), 1.0
        ),
        "nav_loop": dict.fromkeys(("waypoint_reached_m", "goal_reached_m", "corridor_lookahead_m"), 1.0),
    }
    status_path.write_text(json.dumps(snapshot), encoding="utf-8")
    return gateway, snapshot, status_path, events


def test_repeated_native_navigation_callbacks_do_not_query_full_session_or_maps(native_gateway):
    gateway, _snapshot, _path, events = native_gateway
    for sequence in range(20):
        handle_navigation_state(gateway, {"ts": time.time(), "boot_id": "nav-boot", "sequence": sequence})
    assert len(events) == 20
    assert all(event["type"] == "navigation_status" for event in events)
    gate = evaluate_navigation_gate(gateway)
    assert gate["session_mode"] == "navigating"
    assert gate["can_accept_goal"] is True
    gateway._session_snapshot.assert_not_called()
    gateway._map_client.service.assert_not_called()


@pytest.mark.parametrize(
    ("change", "blocker"),
    [
        (lambda state: state.update(stamp_s=time.time() - 10), "native_endpoint_status_missing_or_stale"),
        (lambda state: state["control_authority"].update(estop_latched=True), "estop_latched"),
        (lambda state: state["control_authority"].update(control_loop_hold=True), "native_control_loop_unhealthy"),
        (
            lambda state: state["input_gate"].update(ready=False, reason="map_identity_mismatch"),
            "native_input_gate_not_ready",
        ),
        (lambda state: state["far_input"].update(content_epoch=0), "native_far_input_identity_invalid"),
        (lambda state: state["native_product"].update(product="map"), "native_product_mismatch"),
    ],
)
def test_lightweight_context_still_reads_current_native_safety_and_map_evidence(native_gateway, change, blocker):
    gateway, snapshot, path, _events = native_gateway
    assert evaluate_navigation_gate(gateway)["can_accept_goal"] is True
    change(snapshot)
    path.write_text(json.dumps(snapshot), encoding="utf-8")
    gate = evaluate_navigation_gate(gateway)
    assert gate["can_accept_goal"] is not True
    assert blocker in gate["blockers"] or blocker == gate["reason"]
    gateway._session_snapshot.assert_not_called()


def test_lightweight_context_preserves_localization_freshness_evidence(native_gateway):
    gateway, _snapshot, _path, _events = native_gateway
    assert evaluate_navigation_gate(gateway)["odometry_fresh"] is True
    gateway._odom_timestamps.clear()
    gateway._odom_timestamps.append(time.time() - 10)
    gateway._localization_status.update(state="LOST", pose_fresh=False)
    gate = evaluate_navigation_gate(gateway)
    assert gate["odometry_fresh"] is False
    assert gate["navigation_state_fresh"] is False


def test_compiled_session_context_rejects_mismatched_product(native_gateway):
    gateway, _snapshot, _path, _events = native_gateway
    gateway._compiled_run_plan.lifecycle["product"] = "map"
    with pytest.raises(RuntimeError, match="lifecycle Product mismatch"):
        compiled_session_context(gateway)
