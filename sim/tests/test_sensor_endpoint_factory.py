# ruff: noqa: S101

from __future__ import annotations

from fractions import Fraction
from typing import Any

import pytest

from sim.runtime.sensors import (
    SensorEndpoint,
    SensorEndpointRouter,
    SensorEndpointRoutingError,
    SensorRoute,
    SensorStreamPlan,
)


class _Sink:
    def start(self) -> dict[str, bool]:
        return {"ready": True}

    def publish(self, sample: Any) -> None:
        del sample

    def close(self) -> None:
        return None


def _stream() -> SensorStreamPlan:
    return SensorStreamPlan(
        stream_kind="imu",
        instance_id="thunder_01",
        sensor_id="thunder_01.imu",
        frame_id="thunder_01/imu",
        message_type="lingtu.dds.Imu",
        rate_hz=Fraction(200, 1),
        route=SensorRoute(
            owner="physics",
            source="mujoco_sensor",
            transport="typed_dds",
        ),
    )


def _endpoint(source_id: str) -> SensorEndpoint:
    return SensorEndpoint(
        source_id=source_id,
        sink=_Sink(),
        extractor=lambda scheduled, snapshot: (scheduled, snapshot),
    )


def test_endpoint_router_returns_the_only_matching_endpoint() -> None:
    calls: list[str] = []
    expected = _endpoint("imu-dds")

    def no_match(stream: SensorStreamPlan, allocation: Any) -> None:
        del allocation
        calls.append(f"none:{stream.sensor_id}")
        return None

    def match(stream: SensorStreamPlan, allocation: Any) -> SensorEndpoint:
        del allocation
        calls.append(f"match:{stream.sensor_id}")
        return expected

    router = SensorEndpointRouter((no_match, match))

    assert router(_stream(), object()) is expected
    assert calls == ["none:thunder_01.imu", "match:thunder_01.imu"]


def test_endpoint_router_returns_none_when_no_factory_matches() -> None:
    router = SensorEndpointRouter((lambda stream, allocation: None,))

    assert router(_stream(), object()) is None


def test_endpoint_router_rejects_ambiguous_route_ownership() -> None:
    router = SensorEndpointRouter(
        (
            lambda stream, allocation: _endpoint("first"),
            lambda stream, allocation: _endpoint("second"),
        )
    )

    with pytest.raises(
        SensorEndpointRoutingError,
        match=r"thunder_01\.imu.*more than one endpoint factory",
    ):
        router(_stream(), object())


@pytest.mark.parametrize("factories", [(), (None,)])
def test_endpoint_router_requires_callable_factories(factories: tuple[Any, ...]) -> None:
    with pytest.raises(ValueError, match="at least one callable"):
        SensorEndpointRouter(factories)
