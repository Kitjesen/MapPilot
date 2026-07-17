from __future__ import annotations

from types import SimpleNamespace

from drivers.sim.imu.module import MujocoImuModule
from runtime.msgs.numpy_compat import np


class FakeEngine:
    def get_robot_state(self):
        return SimpleNamespace(
            orientation=np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
            imu_gyro=np.asarray([0.1, 0.2, 0.3], dtype=np.float64),
            imu_linear_acceleration=np.asarray([0.0, 0.0, 9.8], dtype=np.float64),
        )


def test_mujoco_imu_module_binds_driver_owned_engine() -> None:
    module = MujocoImuModule()
    imus = []
    alive = []
    module.imu.subscribe(imus.append)
    module.alive.subscribe(alive.append)

    module.on_system_modules({"MujocoDriverModule": SimpleNamespace(engine=FakeEngine())})
    module.poll()

    assert alive[-1] is True
    assert imus[-1].angular_velocity.y == 0.2
    assert imus[-1].linear_acceleration.z == 9.8
    assert module.health()["error"] is None
