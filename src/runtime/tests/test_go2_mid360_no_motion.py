from __future__ import annotations

from pathlib import Path

import diagnostics.field.go2_mid360_no_motion as gate
from diagnostics.field.go2_mid360_no_motion import _resolved_environment, evaluate_topics


def _row(samples: int, rate_hz: float, last_ts: float = 1.0) -> dict[str, object]:
    return {"samples": samples, "rate_hz": rate_hz, "last_ts": last_ts}


def _healthy_rows() -> dict[str, dict[str, object]]:
    return {
        "rt/lidar/raw_frame": _row(100, 10.0),
        "rt/imu/raw": _row(1000, 100.0),
        "rt/slam/odometry": _row(100, 10.0),
        "rt/slam/registered_cloud": _row(100, 10.0),
        "rt/nav/cmd_vel": _row(0, 0.0, 0.0),
    }


def test_no_motion_plan_uses_go2_sensor_config_without_launching_a_fake_product() -> None:
    repo = Path(__file__).resolve().parents[3]

    environment = _resolved_environment(repo)

    assert environment["LINGTU_DRIVER_BACKEND"] == "go2"
    assert environment["LINGTU_LIVOX_NET_IFACE"] == "eth0"
    assert environment["LINGTU_LIVOX_LIDAR_IP"] == "192.168.123.20"


def test_no_motion_topics_require_sensor_and_slam_freshness() -> None:
    ok, blockers, rows = evaluate_topics(_healthy_rows())

    assert ok is True
    assert blockers == []
    assert rows["rt/nav/cmd_vel"]["samples"] == 0


def test_no_motion_topics_reject_any_final_velocity_sample() -> None:
    payload = _healthy_rows()
    payload["rt/nav/cmd_vel"] = _row(1, 1.0)

    ok, blockers, _ = evaluate_topics(payload)

    assert ok is False
    assert blockers == ["rt/nav/cmd_vel: unexpected samples=1"]


def test_no_motion_topics_reject_stale_slam_output() -> None:
    payload = _healthy_rows()
    payload["rt/slam/odometry"] = _row(1, 0.2)

    ok, blockers, _ = evaluate_topics(payload)

    assert ok is False
    assert blockers == ["rt/slam/odometry: samples=1, hz=0.20, last_ts=1.000"]


def test_no_motion_topics_require_real_timestamps() -> None:
    payload = _healthy_rows()
    payload["rt/lidar/raw_frame"] = _row(100, 10.0, 0.0)

    ok, blockers, _ = evaluate_topics(payload)

    assert ok is False
    assert blockers == ["rt/lidar/raw_frame: samples=100, hz=10.00, last_ts=0.000"]


def test_no_motion_topics_require_cmd_vel_probe_result() -> None:
    payload = _healthy_rows()
    del payload["rt/nav/cmd_vel"]

    ok, blockers, _ = evaluate_topics(payload)

    assert ok is False
    assert blockers == ["rt/nav/cmd_vel: probe result missing"]


def test_stop_sends_sigint_and_waits(monkeypatch) -> None:
    signals = []

    class Process:
        pid = 42

        @staticmethod
        def poll():
            return None

        @staticmethod
        def wait(*, timeout):
            assert timeout == 5.0

    monkeypatch.setattr(gate.os, "killpg", lambda pid, sig: signals.append((pid, sig)), raising=False)

    gate._stop(Process())  # type: ignore[arg-type]

    assert signals == [(42, gate.signal.SIGINT)]
