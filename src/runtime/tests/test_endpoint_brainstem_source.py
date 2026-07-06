from __future__ import annotations

from runtime.adapters.endpoint_sources.brainstem import ThunderBrainstemSource, create
from runtime.adapters.endpoint_sources.types import EndpointEvent
from runtime.msgs.geometry import Twist, Vector3
from runtime.runtime_interface import TOPICS


def test_brainstem_source_factory_reads_endpoint_environment(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_BRAINSTEM_HOST", "192.168.66.190")
    monkeypatch.setenv("LINGTU_BRAINSTEM_PORT", "13145")
    monkeypatch.setenv("LINGTU_BRAINSTEM_REQUIRE_SDK", "0")

    source = create()
    health = source.health()

    assert health["name"] == "thunder_brainstem"
    assert health["hardware"] is True
    assert health["role"] == "brainstem_command_sink"
    assert health["host"] == "192.168.66.190:13145"
    assert health["sdk_required"] is False


def test_brainstem_source_normalizes_cmd_vel_for_walk() -> None:
    source = ThunderBrainstemSource(
        max_linear_speed=0.5,
        max_angular_speed=2.0,
        require_sdk=False,
    )
    event = EndpointEvent(
        topic=TOPICS.cmd_vel,
        channel="LINGTU_NAV_CMD_VEL",
        schema="lingtu.geometry.twist.v1",
        message=Twist(linear=Vector3(0.25, -1.0, 0.0), angular=Vector3(0.0, 0.0, 4.0)),
        ts=123.0,
    )

    source.on_lingtu_message(event)

    health = source.health()
    assert health["received"][TOPICS.cmd_vel] == 1
    assert health["last_walk"] == [0.5, -1.0, 1.0]
    assert health["errors"]["not_started"] == 1
