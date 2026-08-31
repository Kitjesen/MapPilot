"""Read a native MuJoCo ray-caster LiDAR sensor."""

from __future__ import annotations

from typing import Optional

import numpy as np


def read_plugin_lidar(model, data, sensor_name: str = "lidar_mid360") -> Optional[np.ndarray]:
    """Return valid world-frame XYZ points, or ``None`` when unavailable."""

    import mujoco

    try:
        sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
        if sensor_id < 0:
            return None
        plugin_id = model.sensor_plugin[sensor_id]
        if plugin_id < 0:
            return None
        state_idx = model.plugin_stateadr[plugin_id]
        count = int(data.plugin_state[state_idx]) * int(data.plugin_state[state_idx + 1])
        if count <= 0:
            return None
        address = model.sensor_adr[sensor_id]
        points = data.sensordata[address : address + count * 3].reshape(-1, 3).astype(np.float32)
        valid = np.any(points != 0, axis=1) & np.isfinite(points).all(axis=1)
        return points[valid]
    except Exception as exc:
        raise RuntimeError(f"invalid MuJoCo LiDAR state for {sensor_name!r}") from exc
