import time
from types import SimpleNamespace

import pytest

from gateway.rpc import system_state
from gateway.rpc.generated import system_pb2 as pb


@pytest.mark.parametrize("stopped", [False, True])
def test_known_status_preserves_false_and_unknown(monkeypatch, stopped):
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "1")
    monkeypatch.setattr(system_state, "build_readiness_snapshot", lambda _: (
        {"ready": False, "reasons": ["localization:lost", "driver:unavailable"]}, 503,
    ))
    monkeypatch.setattr(system_state, "read_navigation_status", lambda: {
        "stamp_s": time.time(), "control_authority": {"estop_latched": stopped},
    })
    result = system_state.robot_status(SimpleNamespace(_mode="manual"))
    assert result.mode == "manual"
    assert result.HasField("ready") and result.ready is False
    assert result.HasField("software_emergency_stop") and result.software_emergency_stop is stopped
    assert result.reason == "localization:lost; driver:unavailable"
    assert result.health == pb.HEALTH_STATUS_UNKNOWN
    assert not result.HasField("hardware_emergency_stop")
    assert not result.HasField("time")


@pytest.mark.parametrize("native", [None, {}, {"stamp_s": 1, "control_authority": {"estop_latched": True}}])
def test_missing_or_stale_stop_stays_unknown(monkeypatch, native):
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "1")
    monkeypatch.setattr(system_state, "build_readiness_snapshot", lambda _: ({}, 503))
    monkeypatch.setattr(system_state, "read_navigation_status", lambda: native)
    result = system_state.robot_status(SimpleNamespace())
    assert not result.HasField("ready")
    assert not result.HasField("software_emergency_stop")
