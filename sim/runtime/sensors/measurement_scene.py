"""Same-snapshot MuJoCo nearest-geometry raster for measurement cameras."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from .camera_geometry import CameraRaySample
from .samples import SensorSampleStamp


@dataclass(frozen=True, slots=True)
class PhysicalRaster:
    stamp: SensorSampleStamp
    geom_ids: np.ndarray
    distances_m: np.ndarray
    valid_mask: np.ndarray


class MujocoRasterizer:
    """Read the owner's current physics data, without stepping or moving bodies.

    The owner must prevent concurrent stepping while this view is used. A new
    view/stamp is required after an advance/reset. All model geoms participate
    unless the caller supplies the declared geom-group/body exclusion.
    """

    def __init__(self, model: Any, data: Any, source_stamp: SensorSampleStamp):
        self.model, self.data, self.source_stamp = model, data, source_stamp

    def cast(
        self,
        rays: CameraRaySample,
        *,
        max_distance_m: float,
        min_distance_m: float = 0.001,
        body_exclude: int = -1,
        geom_groups: np.ndarray | None = None,
    ) -> PhysicalRaster:
        import mujoco

        if rays.stamp != self.source_stamp or round(float(self.data.time) * 1e9) != rays.stamp.sim_time_ns:
            raise ValueError("physical raster requires the exact source stamp and MuJoCo time")
        if not (
            math.isfinite(min_distance_m) and math.isfinite(max_distance_m) and 0 <= min_distance_m < max_distance_m
        ):
            raise ValueError("physical raster requires finite ordered range limits")
        shape = rays.valid_mask.shape
        if len(shape) != 2 or rays.origins_m.shape != (*shape, 3) or rays.directions_world.shape != (*shape, 3):
            raise ValueError("raster rays must be calibrated HxWx3 arrays")
        origins = np.asarray(rays.origins_m, dtype=np.float64).reshape(-1, 3)
        directions = np.asarray(rays.directions_world, dtype=np.float64).reshape(-1, 3).copy()
        valid = np.asarray(rays.valid_mask, dtype=bool).ravel()
        if not np.isfinite(origins).all() or not np.all(origins == origins[0]):
            raise ValueError("measurement camera rays must share one finite optical origin")
        lengths = np.linalg.norm(directions, axis=1)
        if not np.isfinite(directions[valid]).all() or not np.allclose(lengths[valid], 1.0):
            raise ValueError("valid camera ray directions must be finite unit vectors")
        directions[~valid] = [0, 0, 1]
        ids = np.full(len(valid), -1, dtype=np.int32)
        distances = np.full(len(valid), -1.0, dtype=np.float64)
        # Refresh post-integration derived geometry; never integrate or solve forces.
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_multiRay(
            self.model,
            self.data,
            origins[0],
            directions.ravel(),
            geom_groups,
            True,
            body_exclude,
            ids,
            distances,
            None,
            len(valid),
            max_distance_m,
        )
        mask = valid & (ids >= 0) & (distances >= min_distance_m) & (distances <= max_distance_m)
        ids[~mask] = -1
        distances[~mask] = np.nan
        ids, distances, mask = ids.reshape(shape), distances.reshape(shape), mask.reshape(shape)
        for array in (ids, distances, mask):
            array.setflags(write=False)
        return PhysicalRaster(rays.stamp, ids, distances, mask)


def thermal_calibration(rays: CameraRaySample, calibration_id: str = "simulation-declared") -> dict[str, Any]:
    """Translate the shared projection declaration, retaining world pose metadata."""
    projection = dict(rays.calibration_metadata["projection"])
    result = {
        "model": projection["model"],
        "width": projection["width"],
        "height": projection["height"],
        "calibration_id": calibration_id,
        "ray_calibration": rays.calibration_metadata,
    }
    if projection["model"] == "pinhole":
        result.update({name: projection[f"{name}_px"] for name in ("fx", "fy", "cx", "cy")})
    else:
        result["parameters"] = projection
    return result
