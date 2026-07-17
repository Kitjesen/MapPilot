from __future__ import annotations

from types import SimpleNamespace

from drivers.sim.lidar.module import MujocoLidarModule
from runtime.msgs.numpy_compat import np


class FakeEngine:
    def get_lidar_points(self):
        return np.asarray(
            [
                [1.0, 2.0, 3.0, 4.0],
                [5.0, 6.0, 7.0, 8.0],
            ],
            dtype=np.float32,
        )


def test_mujoco_lidar_module_binds_driver_owned_engine() -> None:
    module = MujocoLidarModule()
    scans = []
    alive = []
    module.scan.subscribe(scans.append)
    module.alive.subscribe(alive.append)

    module.on_system_modules({"MujocoDriverModule": SimpleNamespace(engine=FakeEngine())})
    module.poll()

    assert alive[-1] is True
    assert scans[-1].points.shape == (2, 4)
    assert module.health()["error"] is None
