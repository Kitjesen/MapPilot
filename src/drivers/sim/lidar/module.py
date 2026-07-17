"""Simulation LiDAR runtime module."""

from __future__ import annotations

from typing import Any

from runtime.module import Module
from runtime.msgs.sensor import Imu, PointCloud2
from runtime.registry import register
from runtime.stream import Out

from .impl.mujoco import Lidar
from .native import Config


@register("lidar", "mujoco", description="MuJoCo LiDAR stream")
class MujocoLidarModule(Module, layer=1):
    """Publish MuJoCo LiDAR frames through canonical LiDAR ports."""

    scan: Out[PointCloud2]
    raw_scan: Out[Any]
    imu: Out[Imu]
    alive: Out[bool]

    def __init__(
        self,
        engine: Any | None = None,
        driver_module: str = "MujocoDriverModule",
        **kw,
    ) -> None:
        super().__init__(**kw)
        self._source = Lidar(engine)
        self._config = Config()
        self._driver_module = driver_module
        self._modules: dict[str, Module] = {}
        self._error: str | None = None

    def bind(self, engine: Any) -> None:
        self._source.bind(engine)
        self._source.connect(self._config)
        self._error = None

    def on_system_modules(self, modules: dict[str, Module]) -> None:
        self._modules = dict(modules)
        self._try_bind_driver_engine()

    def setup(self) -> None:
        self._try_bind_driver_engine()
        self._source.connect(self._config)
        connected = self._source.is_connected()
        if not connected:
            self._error = "MuJoCo LiDAR engine is not bound"
        self.alive.publish(connected)

    def stop(self) -> None:
        self._source.disconnect()
        self.alive.publish(False)
        super().stop()

    def poll(self) -> None:
        if not self._source.is_connected() and not self._try_bind_driver_engine():
            self._error = "MuJoCo LiDAR engine is not bound"
            self.alive.publish(False)
            return
        sample = self._source.read()
        if sample.scan is not None:
            self.scan.publish(sample.scan)
        if sample.raw_scan is not None:
            self.raw_scan.publish(sample.raw_scan)
        if sample.imu is not None:
            self.imu.publish(sample.imu)
        if sample.status.get("connected"):
            self._error = None
            self.alive.publish(True)

    def health(self) -> dict[str, object]:
        info = super().port_summary()
        info["role"] = "lidar"
        info["lidar_backend"] = "mujoco"
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


LidarModule = MujocoLidarModule
