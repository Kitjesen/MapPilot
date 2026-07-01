from __future__ import annotations

from typing import Any

from compat.lcm.endpoint_service import LCMEndpointEvent
from compat.lcm.sources.thunder_brainstem import ThunderBrainstemEndpointSource, create
from core.msgs.geometry import Twist, Vector3
from core.msgs.nav import Odometry
from core.runtime_interface import TOPICS


class _FakeIn:
    def __init__(self) -> None:
        self.delivered: list[Any] = []

    def _deliver(self, message: Any) -> None:
        self.delivered.append(message)


class _FakeOut:
    def __init__(self) -> None:
        self._callbacks = []

    def subscribe(self, callback):
        self._callbacks.append(callback)

        def unsubscribe() -> None:
            self._callbacks.remove(callback)

        return unsubscribe

    def publish(self, message: Any) -> None:
        for callback in list(self._callbacks):
            callback(message)


class _FakeDriver:
    def __init__(self, **config: Any) -> None:
        self.config = config
        self.cmd_vel = _FakeIn()
        self.odometry = _FakeOut()
        self.alive = _FakeOut()
        self.robot_state = _FakeOut()
        self.setup_count = 0
        self.start_count = 0
        self.stop_count = 0

    def setup(self) -> None:
        self.setup_count += 1

    def start(self) -> None:
        self.start_count += 1
        self.alive.publish(True)
        self.robot_state.publish({"connected": True, "standing": False})

    def stop(self) -> None:
        self.stop_count += 1

    def health(self) -> dict[str, Any]:
        return {"driver": "fake", "config": dict(self.config)}


class _FakeEndpointService:
    def __init__(self) -> None:
        self.localization_snapshots: list[dict[str, Any]] = []

    def publish_localization_snapshot(self, **snapshot: Any) -> int:
        self.localization_snapshots.append(snapshot)
        return sum(value is not None for value in snapshot.values())


def _event(topic: str, message: Any) -> LCMEndpointEvent:
    return LCMEndpointEvent(
        topic=topic,
        channel=f"CHAN_{topic}",
        schema="test",
        message=message,
        ts=123.0,
    )


def test_thunder_brainstem_source_routes_cmd_vel_to_driver() -> None:
    driver = _FakeDriver(dog_host="10.0.0.2")
    source = ThunderBrainstemEndpointSource(
        driver_factory=lambda **_: driver,
        driver_config={"dog_host": "10.0.0.2"},
    )

    source.start(_FakeEndpointService())  # type: ignore[arg-type]
    cmd = Twist(linear=Vector3(0.3, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.2))
    source.on_lingtu_message(_event(TOPICS.cmd_vel, cmd))

    assert driver.setup_count == 1
    assert driver.start_count == 1
    assert driver.cmd_vel.delivered[-1] is cmd
    assert source.health()["received"] == {TOPICS.cmd_vel: 1}

    source.stop()
    assert driver.stop_count == 1
    assert isinstance(driver.cmd_vel.delivered[-1], Twist)
    assert driver.cmd_vel.delivered[-1].linear.x == 0.0


def test_thunder_brainstem_source_can_publish_optional_dead_reckoning() -> None:
    driver = _FakeDriver()
    service = _FakeEndpointService()
    source = ThunderBrainstemEndpointSource(
        driver_factory=lambda **_: driver,
        publish_odometry=True,
    )

    source.start(service)  # type: ignore[arg-type]
    driver.odometry.publish(Odometry(frame_id="odom"))

    assert len(service.localization_snapshots) == 1
    snapshot = service.localization_snapshots[0]
    assert isinstance(snapshot["odometry"], Odometry)
    assert snapshot["localization_health"]["state"] == "BRAINSTEM_DEAD_RECKONING"
    assert snapshot["localization_quality"] == 0.0
    assert source.health()["published"][TOPICS.odometry] == 1


def test_thunder_brainstem_source_factory_reads_deployment_env(monkeypatch) -> None:
    monkeypatch.setenv("LINGTU_THUNDER_DOG_HOST", "192.168.66.190")
    monkeypatch.setenv("LINGTU_THUNDER_DOG_PORT", "13146")
    monkeypatch.setenv("LINGTU_THUNDER_AUTO_ENABLE", "1")
    monkeypatch.setenv("LINGTU_THUNDER_PUBLISH_ODOMETRY", "1")

    source = create(driver_factory=lambda **config: _FakeDriver(**config))

    health = source.health()
    assert health["driver_config"]["dog_host"] == "192.168.66.190"
    assert health["driver_config"]["dog_port"] == 13146
    assert health["driver_config"]["auto_enable"] is True
    assert health["publish_odometry"] is True
