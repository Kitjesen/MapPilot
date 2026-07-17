from __future__ import annotations

from importlib import reload
from types import SimpleNamespace

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


def test_camera_gateway_resolves_sim_backend() -> None:
    from lingtu.plugin_seed import seed_builtin_plugins
    from runtime.adapters.perception_gateway import camera_module

    state = snapshot()
    try:
        clear()
        seed_builtin_plugins(groups=("camera_sim",), reload_loaded=True, strict=True)
        cls = camera_module(backend=CAMERA_BACKEND_SIM)
        assert cls is not None
        assert cls.__name__ == "MujocoCameraModule"
        assert cls.__module__ == "drivers.sim.camera.module"
    finally:
        restore(state)
