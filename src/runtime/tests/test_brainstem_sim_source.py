from __future__ import annotations

import asyncio
import socket
import time
from collections import Counter
from concurrent.futures import TimeoutError as FutureTimeoutError
from typing import Any

import pytest

from runtime.adapters.lcm.endpoint_service import LCMEndpointEvent
from runtime.adapters.lcm.sources.brainstem import ThunderBrainstemSource
from runtime.adapters.lcm.sources.brainstem_sim import BrainstemSimSource, create
from runtime.msgs.geometry import Twist, Vector3
from runtime.runtime_interface import TOPICS


class _FakeEndpointService:
    def __init__(self) -> None:
        self.publish_counts: Counter[str] = Counter()
        self.odometry: list[Any] = []
        self.registered_cloud: list[Any] = []
        self.map_cloud: list[Any] = []
        self.localization_health: list[dict[str, Any]] = []

    def publish_sensor_snapshot(
        self,
        *,
        lidar_scan: Any | None = None,
        imu: Any | None = None,
    ) -> int:
        count = 0
        if lidar_scan is not None:
            self.publish_counts[TOPICS.lidar_scan] += 1
            count += 1
        if imu is not None:
            self.publish_counts[TOPICS.imu] += 1
            count += 1
        return count

    def publish_localization_snapshot(
        self,
        *,
        odometry: Any | None = None,
        registered_cloud: Any | None = None,
        map_cloud: Any | None = None,
        localization_health: dict[str, Any] | None = None,
        localization_quality: float | None = None,
    ) -> int:
        count = 0
        if odometry is not None:
            self.odometry.append(odometry)
            self.publish_counts[TOPICS.odometry] += 1
            count += 1
        if registered_cloud is not None:
            self.registered_cloud.append(registered_cloud)
            self.publish_counts[TOPICS.registered_cloud] += 1
            count += 1
        if map_cloud is not None:
            self.map_cloud.append(map_cloud)
            self.publish_counts[TOPICS.map_cloud] += 1
            count += 1
        if localization_health is not None:
            self.localization_health.append(localization_health)
            self.publish_counts[TOPICS.localization_health] += 1
            count += 1
        if localization_quality is not None:
            self.publish_counts[TOPICS.localization_quality] += 1
            count += 1
        return count


def test_brainstem_sim_direct_cmd_vel_moves_kinematic_feedback() -> None:
    service = _FakeEndpointService()
    source = BrainstemSimSource(
        start_grpc=False,
        direct_cmd_vel=True,
        publish_rate=100.0,
        cmd_timeout_sec=1.0,
        cloud_points=8,
        run_background=False,
    )
    source.start(service)
    try:
        source.on_lingtu_message(
            LCMEndpointEvent(
                topic=TOPICS.cmd_vel,
                channel="LINGTU_NAV_CMD_VEL",
                schema="lingtu.geometry.twist.v1",
                message=Twist(
                    linear=Vector3(0.4, 0.0, 0.0),
                    angular=Vector3(0.0, 0.0, 0.1),
                ),
                ts=time.time(),
            )
        )
        source.step_once(0.1)
    finally:
        source.stop()

    assert service.publish_counts[TOPICS.odometry] >= 1
    assert service.publish_counts[TOPICS.map_cloud] >= 1
    assert service.publish_counts[TOPICS.registered_cloud] >= 1
    assert service.odometry[-1].x > 0.0
    assert service.localization_health[-1]["state"] == "TRACKING"
    assert service.localization_health[-1]["source"] == "brainstem_sim"
    assert source.health()["distance_m"] > 0.0


def test_brainstem_sim_factory_can_disable_grpc_for_local_smoke(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_BRAINSTEM_SIM_START_GRPC", "0")
    monkeypatch.setenv("LINGTU_BRAINSTEM_SIM_DIRECT_CMD_VEL", "1")
    monkeypatch.setenv("LINGTU_BRAINSTEM_SIM_RATE", "30")

    source = create()
    health = source.health()

    assert health["name"] == "brainstem_sim"
    assert health["hardware"] is False
    assert health["grpc_enabled"] is False
    assert health["direct_cmd_vel"] is True


def test_endpoint_runners_load_brainstem_sim_builtin(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_BRAINSTEM_SIM_START_GRPC", "0")

    from runtime.adapters.dds.endpoint_runner import _load_source as load_dds_source
    from runtime.adapters.lcm.endpoint_runner import _load_source as load_lcm_source

    assert load_dds_source("brainstem_sim").health()["name"] == "brainstem_sim"
    assert load_lcm_source("builtin:brainstem_sim").health()["name"] == "brainstem_sim"


def test_brainstem_sink_stop_tolerates_safe_stop_future_timeout(monkeypatch) -> None:
    source = ThunderBrainstemSource(require_sdk=False)
    source._loop = object()
    source._connected = True

    class _TimeoutFuture:
        def result(self, timeout: float | None = None) -> None:
            raise FutureTimeoutError()

    def _fake_run_coroutine_threadsafe(coro: Any, loop: Any) -> _TimeoutFuture:
        coro.close()
        return _TimeoutFuture()

    monkeypatch.setattr(asyncio, "run_coroutine_threadsafe", _fake_run_coroutine_threadsafe)

    source.stop()

    assert source.health()["started"] is False
    assert source.health()["connected"] is False


def test_brainstem_sim_accepts_real_brainstem_walk_grpc() -> None:
    pytest.importorskip("brainstem_api")
    pytest.importorskip("grpc")
    service = _FakeEndpointService()
    port = _free_tcp_port()
    sim = BrainstemSimSource(
        host="127.0.0.1",
        port=port,
        start_grpc=True,
        direct_cmd_vel=False,
        run_background=False,
        cmd_timeout_sec=1.0,
    )
    sink = ThunderBrainstemSource(
        dog_host="127.0.0.1",
        dog_port=port,
        max_linear_speed=1.0,
        max_angular_speed=1.0,
        connect_timeout_sec=3.0,
        control_rate=20.0,
        require_sdk=True,
    )
    sim.start(service)
    sink.start(service)
    try:
        deadline = time.time() + 4.0
        while time.time() < deadline and not sink.health()["connected"]:
            time.sleep(0.05)
        assert sink.health()["connected"] is True

        sink.on_lingtu_message(
            LCMEndpointEvent(
                topic=TOPICS.cmd_vel,
                channel="LINGTU_NAV_CMD_VEL",
                schema="lingtu.geometry.twist.v1",
                message=Twist(linear=Vector3(0.2, 0.0, 0.0)),
                ts=time.time(),
            )
        )
        deadline = time.time() + 2.0
        while time.time() < deadline and sim.health()["grpc_calls"].get("Walk", 0) < 1:
            time.sleep(0.05)
        sim.step_once(0.1)
    finally:
        sink.stop()
        sim.stop()

    assert sim.health()["grpc_calls"]["Walk"] >= 1
    assert service.odometry[-1].x > 0.0


def _free_tcp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])
