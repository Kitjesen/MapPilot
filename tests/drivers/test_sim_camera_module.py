from __future__ import annotations

from importlib import reload
from types import SimpleNamespace

import pytest
from sim.engine.core.sensor import CameraConfig

from drivers.sim.camera.impl.mujoco.camera import sample_from_camera
from drivers.sim.camera.module import MujocoCameraModule
from runtime.contracts import CAMERA_BACKEND_SIM, CAMERA_ROLE
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import ImageFormat
from runtime.registry import clear, get, restore, snapshot


def _camera_data():
    return SimpleNamespace(
        rgb=np.zeros((4, 6, 3), dtype=np.uint8),
        depth=np.ones((4, 6), dtype=np.float32),
        intrinsics=(10.0, 11.0, 3.0, 2.0),
        timestamp=123.0,
    )


class FakeEngine:
    def get_camera_data(self, name: str):
        assert name == "front_camera"
        return _camera_data()


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeStopEvent:
    def __init__(self, clock: FakeClock) -> None:
        self.clock = clock
        self.waits: list[float] = []
        self._set = False

    def is_set(self) -> bool:
        return self._set

    def set(self) -> None:
        self._set = True

    def wait(self, timeout: float) -> bool:
        if self._set:
            return True
        self.waits.append(timeout)
        self.clock.advance(timeout)
        return False


def test_mujoco_camera_uses_canonical_30_hz_default() -> None:
    camera_module = MujocoCameraModule()
    health = camera_module.health()

    assert CameraConfig().fps == 30.0
    assert health["nominal_rate_hz"] == 30.0
    assert health["camera_info_nominal_rate_hz"] == 1.0
    assert health["fps"] == 0.0


@pytest.mark.parametrize("fps", [0.0, -1.0])
def test_mujoco_camera_rejects_non_positive_frequency(fps: float) -> None:
    with pytest.raises(ValueError, match="fps must be greater than zero"):
        MujocoCameraModule(fps=fps)


def test_camera_loop_includes_render_time_in_absolute_period(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = FakeClock()
    module = MujocoCameraModule()
    stop = FakeStopEvent(clock)
    starts: list[float] = []
    module._stop_event = stop

    def publish() -> None:
        starts.append(clock.now)
        clock.advance(0.01)
        if len(starts) == 3:
            stop.set()

    monkeypatch.setattr("drivers.sim.camera.module.time.monotonic", clock.monotonic)
    monkeypatch.setattr(module, "_publish_sample", publish)

    module._loop()

    period = 1.0 / 30.0
    assert starts == pytest.approx([0.0, period, 2.0 * period])
    assert stop.waits == pytest.approx([period - 0.01, period - 0.01])


def test_camera_loop_skips_missed_deadlines_without_burst(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = FakeClock()
    module = MujocoCameraModule()
    stop = FakeStopEvent(clock)
    starts: list[float] = []
    module._stop_event = stop

    def publish() -> None:
        starts.append(clock.now)
        if len(starts) == 1:
            clock.advance(0.08)
        else:
            stop.set()

    monkeypatch.setattr("drivers.sim.camera.module.time.monotonic", clock.monotonic)
    monkeypatch.setattr(module, "_publish_sample", publish)

    module._loop()

    assert starts == pytest.approx([0.0, 0.1])
    assert stop.waits == pytest.approx([0.02])
    assert all(delay > 0.0 for delay in stop.waits)


def test_camera_health_separates_nominal_and_observed_rates() -> None:
    module = MujocoCameraModule()

    module._record_publication(1.0)
    assert module.health()["fps"] == 0.0
    module._record_publication(1.05)

    health = module.health()
    assert health["nominal_rate_hz"] == 30.0
    assert health["fps"] == pytest.approx(20.0)


def test_mujoco_camera_source_returns_canonical_sample() -> None:
    sample = sample_from_camera(_camera_data())

    assert sample.color is not None
    assert sample.color.format is ImageFormat.RGB
    assert sample.depth is not None
    assert sample.depth.format is ImageFormat.DEPTH_F32
    assert sample.intrinsics is not None
    assert sample.intrinsics.width == 6
    assert sample.intrinsics.height == 4


def test_mujoco_camera_module_publishes_camera_ports() -> None:
    module = MujocoCameraModule(engine=FakeEngine())
    colors = []
    depths = []
    infos = []
    module.color_image.subscribe(colors.append)
    module.depth_image.subscribe(depths.append)
    module.camera_info.subscribe(infos.append)

    module.poll()

    assert colors[-1].data.shape == (4, 6, 3)
    assert depths[-1].data.shape == (4, 6)
    assert infos[-1].fx == 10.0


def test_camera_info_is_immediate_then_limited_to_one_hz(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = FakeClock()
    module = MujocoCameraModule(engine=FakeEngine())
    colors = []
    infos = []
    module.color_image.subscribe(colors.append)
    module.camera_info.subscribe(infos.append)
    monkeypatch.setattr("drivers.sim.camera.module.time.monotonic", clock.monotonic)

    for timestamp in (0.0, 0.02, 0.99, 1.0):
        clock.now = timestamp
        module.poll()

    assert len(colors) == 4
    assert len(infos) == 2


def test_mujoco_camera_module_binds_driver_owned_engine() -> None:
    module = MujocoCameraModule()
    colors = []
    module.color_image.subscribe(colors.append)

    module.on_system_modules({"MujocoDriverModule": SimpleNamespace(engine=FakeEngine())})
    module.poll()

    assert colors[-1].data.shape == (4, 6, 3)
    assert module.health()["error"] is None


def test_sim_camera_registers_as_camera_backend() -> None:
    state = snapshot()
    try:
        clear()
        import drivers.sim.camera.module as module

        reload(module)

        cls = get(CAMERA_ROLE, CAMERA_BACKEND_SIM)
        assert cls.__name__ == "MujocoCameraModule"
        assert cls.__module__ == "drivers.sim.camera.module"
    finally:
        restore(state)
