"""MuJoCo live gate sensor-frame helpers."""

from __future__ import annotations

from typing import Any

_MUJOCO_SENSORS: Any | None = None

def _mujoco_sensors() -> Any:
    global _MUJOCO_SENSORS
    if _MUJOCO_SENSORS is None:
        from drivers.sim.mujoco import sensors

        _MUJOCO_SENSORS = sensors
    return _MUJOCO_SENSORS

def _angle_delta_rad(*args: Any, **kwargs: Any) -> Any:
    return _mujoco_sensors().angle_delta_rad(*args, **kwargs)

def _specific_force_body(*args: Any, **kwargs: Any) -> Any:
    return _mujoco_sensors().specific_force_body(*args, **kwargs)

def _world_xyzi_to_sensor_xyzi(*args: Any, **kwargs: Any) -> Any:
    return _mujoco_sensors().world_xyzi_to_sensor_xyzi(*args, **kwargs)

def _yaw_from_quat_xyzw(*args: Any, **kwargs: Any) -> Any:
    return _mujoco_sensors().yaw_from_quat_xyzw(*args, **kwargs)
