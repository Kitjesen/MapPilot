"""Simulation IMU runtime module facade."""

from __future__ import annotations

from typing import Any

from runtime.module import Module
from runtime.msgs.sensor import Imu
from runtime.registry import register
from runtime.stream import Out

from .impl.mujoco import Imu as MujocoImu


@register("imu", "mujoco", description="MuJoCo IMU stream")
class MujocoImuModule(Module, layer=1):
    """Facade for MuJoCo IMU data.

    The MuJoCo engine remains owned by `MujocoDriverModule`; this module binds
    to that engine and publishes the canonical IMU role.
    """

    imu: Out[Imu]
    alive: Out[bool]

    def __init__(
        self,
        engine: Any | None = None,
        driver_module: str = "MujocoDriverModule",
        **kw,
    ) -> None:
        super().__init__(**kw)
        self._source = MujocoImu(engine)
        self._driver_module = driver_module
        self._modules: dict[str, Module] = {}
        self._error: str | None = None

    def bind(self, engine: Any) -> None:
        self._source.bind(engine)
        self._source.connect()
        self._error = None

    def on_system_modules(self, modules: dict[str, Module]) -> None:
        self._modules = dict(modules)
        self._try_bind_driver_engine()

    def setup(self) -> None:
        self._try_bind_driver_engine()
        self._source.connect()
        connected = self._source.is_connected()
        if not connected:
            self._error = "MuJoCo IMU engine is not bound"
        self.alive.publish(connected)

    def stop(self) -> None:
        self._source.disconnect()
        self.alive.publish(False)
        super().stop()

    def poll(self) -> None:
        if not self._source.is_connected() and not self._try_bind_driver_engine():
            self._error = "MuJoCo IMU engine is not bound"
            self.alive.publish(False)
            return
        sample = self._source.read()
        if sample.imu is not None:
            self.imu.publish(sample.imu)
            self._error = None
            self.alive.publish(True)

    def health(self) -> dict[str, object]:
        info = super().port_summary()
        info["role"] = "imu"
        info["imu_backend"] = "mujoco"
        info["error"] = self._error
        info["connected"] = self._source.is_connected()
        return info

    def _try_bind_driver_engine(self) -> bool:
        if self._source.is_connected():
            return True
        driver = self._modules.get(self._driver_module)
        if driver is None:
            for name, module in self._modules.items():
                if name == self._driver_module or type(module).__name__ == self._driver_module:
                    driver = module
                    break
        engine = getattr(driver, "engine", None) if driver is not None else None
        if engine is None:
            return False
        self.bind(engine)
        return True


ImuModule = MujocoImuModule
