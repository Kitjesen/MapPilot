"""Typed fault snapshots preserve current evidence and unavailable-source boundaries."""

import json
import socket
import threading
import time
from contextlib import contextmanager
from types import SimpleNamespace

import grpc
import pytest

from gateway.rpc import faults, services
from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import system_pb2 as pb
from gateway.rpc.generated import system_pb2_grpc as rpc
from gateway.rpc.server import start

SOURCES = ["driver.control", "driver.health", "navigation.control_loop"]


@pytest.fixture
def snapshots(monkeypatch, tmp_path):
    driver_path = tmp_path / "driver.json"
    nav_path = tmp_path / "nav.json"
    monkeypatch.setenv("LINGTU_DRIVER_STATUS_FILE", str(driver_path))
    monkeypatch.setenv("LINGTU_NAV_STATUS_FILE", str(nav_path))
    monkeypatch.delenv("LINGTU_DRIVER_STATUS_MAX_AGE_S", raising=False)
    monkeypatch.delenv("LINGTU_NAV_STATUS_MAX_AGE_S", raising=False)
    driver = {
        "stamp_s": time.time(), "connected": True,
        "control": {"critical_fault": False},
        "health": {"fresh": True, "healthy": True, "reason": "healthy", "fault_code": 0},
    }
    navigation = {
        "stamp_s": time.time(),
        "control_loop_health": {"ready": True, "healthy": True, "reason": "healthy"},
    }

    def publish():
        driver_path.write_text(json.dumps(driver), encoding="utf-8")
        nav_path.write_text(json.dumps(navigation), encoding="utf-8")

    publish()
    return SimpleNamespace(driver=driver, navigation=navigation, driver_path=driver_path, nav_path=nav_path, publish=publish)


def test_clear_observed_sources_keep_partial_health_unknown(snapshots):
    result = faults.read()
    assert result.health == pb.HEALTH_STATUS_UNKNOWN
    assert list(result.observed_sources) == SOURCES
    assert not result.unavailable_sources and not result.faults


def test_critical_fault_is_error_and_is_not_duplicated_from_driver_health(snapshots):
    snapshots.driver["control"]["critical_fault"] = True
    snapshots.driver["health"].update(healthy=False, reason="critical_motor_fault")
    snapshots.publish()
    result = faults.read()
    assert result.health == pb.HEALTH_STATUS_ERROR
    assert len(result.faults) == 1
    fault = result.faults[0]
    assert (fault.source, fault.code, fault.severity) == (
        "driver.control", "critical_motor_fault", pb.FAULT_SEVERITY_ERROR,
    )
    assert fault.HasField("active") and fault.active
    assert not fault.HasField("time")


def test_motor_output_disabled_is_not_a_fault(snapshots):
    snapshots.driver["health"].update(healthy=False, reason="motors_disabled")
    snapshots.publish()
    result = faults.read()
    assert result.health == pb.HEALTH_STATUS_UNKNOWN
    assert not result.faults and list(result.observed_sources) == SOURCES


@pytest.mark.parametrize("reason", ["tilt_limit_exceeded", "imu_attitude_invalid", ""])
def test_explicit_driver_health_failure_is_preserved(snapshots, reason):
    snapshots.driver["health"].update(healthy=False, reason=reason)
    snapshots.publish()
    result = faults.read()
    assert result.health == pb.HEALTH_STATUS_ERROR
    assert result.faults[0].source == "driver.health"
    assert result.faults[0].code == (reason or "unhealthy")


def test_ready_control_loop_fault_is_warning(snapshots):
    snapshots.navigation["control_loop_health"].update(healthy=False, reason="deadline_miss_ratio_high")
    snapshots.publish()
    result = faults.read()
    assert result.health == pb.HEALTH_STATUS_WARNING
    assert result.faults[0].source == "navigation.control_loop"
    assert result.faults[0].code == "deadline_miss_ratio_high"
    assert result.faults[0].severity == pb.FAULT_SEVERITY_WARNING


@pytest.mark.parametrize("condition", ["missing", "stale", "future", "invalid_stamp", "disconnected", "invalid_max_age"])
def test_unavailable_driver_does_not_clear_or_infer_faults(snapshots, monkeypatch, condition):
    snapshots.driver["control"]["critical_fault"] = True
    if condition == "stale":
        snapshots.driver["stamp_s"] = time.time() - 10
    elif condition == "future":
        snapshots.driver["stamp_s"] = time.time() + 10
    elif condition == "invalid_stamp":
        snapshots.driver["stamp_s"] = "unknown"
    elif condition == "disconnected":
        snapshots.driver["connected"] = False
    elif condition == "invalid_max_age":
        monkeypatch.setenv("LINGTU_DRIVER_STATUS_MAX_AGE_S", "nan")
    snapshots.publish()
    if condition == "missing":
        snapshots.driver_path.unlink()
    result = faults.read()
    assert list(result.observed_sources) == ["navigation.control_loop"]
    assert list(result.unavailable_sources) == ["driver.control", "driver.health"]
    assert result.health == pb.HEALTH_STATUS_UNKNOWN and not result.faults


@pytest.mark.parametrize("change", [{"ready": False, "healthy": False}, {"healthy": "false"}, {}])
def test_unready_or_malformed_loop_is_unavailable(snapshots, change):
    snapshots.navigation["control_loop_health"] = change
    snapshots.publish()
    result = faults.read()
    assert list(result.observed_sources) == ["driver.control", "driver.health"]
    assert list(result.unavailable_sources) == ["navigation.control_loop"]
    assert result.health == pb.HEALTH_STATUS_UNKNOWN and not result.faults


def test_missing_all_sources_returns_unknown_report(snapshots):
    snapshots.driver_path.unlink()
    snapshots.nav_path.unlink()
    result = faults.read()
    assert result.health == pb.HEALTH_STATUS_UNKNOWN
    assert not result.observed_sources and not result.faults
    assert list(result.unavailable_sources) == SOURCES


@contextmanager
def serve():
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        address = f"127.0.0.1:{listener.getsockname()[1]}"
    server = start(SimpleNamespace(), address)
    with grpc.insecure_channel(address) as channel:
        try:
            grpc.channel_ready_future(channel).result(timeout=3)
            yield rpc.SystemStub(channel)
        finally:
            server.stop(0).wait(3)


def test_real_rpc_and_stream_follow_current_faults_and_source_loss(snapshots):
    snapshots.driver["control"]["critical_fault"] = True
    snapshots.publish()
    with serve() as client:
        assert client.Faults(common_pb.Empty(), timeout=3).health == pb.HEALTH_STATUS_ERROR
        stream = client.WatchFaults(common_pb.Empty(), timeout=3)
        assert next(stream).health == pb.HEALTH_STATUS_ERROR
        assert next(stream).health == pb.HEALTH_STATUS_ERROR
        snapshots.driver["control"]["critical_fault"] = False
        snapshots.publish()
        for _ in range(20):
            report = next(stream)
            if not report.faults:
                break
        assert report.health == pb.HEALTH_STATUS_UNKNOWN
        assert "driver.control" in report.observed_sources
        snapshots.driver["stamp_s"] = time.time() - 10
        snapshots.publish()
        for _ in range(20):
            report = next(stream)
            if "driver.control" in report.unavailable_sources:
                break
        assert "driver.control" in report.unavailable_sources
        assert "driver.control" not in report.observed_sources
        stream.cancel()


def test_fault_watch_releases_shared_slot_on_close(snapshots):
    slots = threading.BoundedSemaphore(1)
    service = services.System(SimpleNamespace(), slots)
    context = SimpleNamespace(add_callback=lambda _: True, is_active=lambda: True)
    stream = service.WatchFaults(common_pb.Empty(), context)
    assert next(stream).health == pb.HEALTH_STATUS_UNKNOWN
    assert not slots.acquire(blocking=False)
    stream.close()
    assert slots.acquire(blocking=False)
