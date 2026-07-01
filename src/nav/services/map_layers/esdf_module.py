"""ESDFModule — 2-D Euclidean Signed Distance Field from occupancy grid.

Subscribes to OccupancyGridModule.occupancy_grid (OccupancyGrid).
Computes per-cell signed distance to the nearest obstacle:

  d > 0  — distance (metres) from a free cell to nearest obstacle
  d = 0  — obstacle boundary
  d < 0  — penetration depth inside an obstacle

Also publishes the gradient (∇d) — used by trajectory optimisers
(MPPI / TEB / CHOMP) for collision-cost gradient descent.

Ports:
  In:  occupancy_grid (OccupancyGrid)
  Out: esdf (dict: distance_field, grad_x, grad_y, resolution, origin)
"""

from __future__ import annotations

import logging
import time
from typing import Any

from nav.services.map_layers.cpp_backend import (
    create_map_kernel_backend,
    esdf_result_to_payload,
    make_grid2d,
)
from runtime.module import Module
from runtime.msgs.nav import OccupancyGrid
from runtime.msgs.numpy_compat import np
from runtime.registry import register
from runtime.stream import In, Out

logger = logging.getLogger(__name__)


@register("map", "esdf", description="2D ESDF + gradient from occupancy grid")
class ESDFModule(Module, layer=2):
    """2-D Signed Distance Field via scipy distance_transform_edt.

    The EDT runs on both the free mask and obstacle mask, then their
    difference gives the signed field.  Gradients are computed with
    numpy central differences over the resulting float field.

    Throttled to 1 Hz by default — EDT is O(N) but still ~30-80 ms
    on a 300×300 grid on the S100P CPU.

    Consumers
    ---------
    Future local trajectory optimisers (MPPI, TEB) subscribe to
    ``esdf`` and use ``distance_field`` + ``grad_x``/``grad_y`` to
    compute repulsive costs without repeated distance queries.
    """

    occupancy_grid: In[OccupancyGrid]
    esdf: Out[dict]

    def __init__(
        self,
        obstacle_threshold: int = 50,  # cells with cost ≥ this are obstacles (0-100)
        publish_hz: float = 1.0,
        **kw,
    ):
        super().__init__(**kw)
        self._obs_thr = obstacle_threshold
        self._interval = 1.0 / publish_hz
        self._map_kernel = None

    def setup(self) -> None:
        self._map_kernel = create_map_kernel_backend()
        self.occupancy_grid.subscribe(self._on_grid)
        self.occupancy_grid.set_policy("throttle", interval=self._interval)

    def _on_grid(self, og: OccupancyGrid) -> None:
        if self._map_kernel is not None:
            grid = make_grid2d(
                self._map_kernel.runtime,
                og.grid,
                resolution=og.resolution,
                origin=[og.origin.x, og.origin.y],
            )
            result = self._map_kernel.runtime.compute_esdf(grid, float(self._obs_thr))
            self.esdf.publish(
                esdf_result_to_payload(result, ts=time.time(), frame_id=og.frame_id)
            )
            return

        from scipy.ndimage import distance_transform_edt  # verified at setup()

        obstacle_mask = og.grid >= self._obs_thr  # True = obstacle
        free_mask = ~obstacle_mask

        # EDT returns distance in *cells*; multiply by resolution for metres
        dist_free = distance_transform_edt(free_mask).astype(np.float32) * og.resolution
        dist_occ  = distance_transform_edt(obstacle_mask).astype(np.float32) * og.resolution

        # signed: positive in free space, negative inside obstacles
        sdf = np.where(free_mask, dist_free, -dist_occ)

        # central-difference gradients (∂d/∂x, ∂d/∂y) in metres/metre = unitless
        grad_x = np.gradient(sdf, og.resolution, axis=1).astype(np.float32)
        grad_y = np.gradient(sdf, og.resolution, axis=0).astype(np.float32)

        self.esdf.publish(self._pack(og, sdf.astype(np.float32), grad_x, grad_y))

    @staticmethod
    def _pack(
        og: OccupancyGrid,
        sdf: np.ndarray,
        grad_x: np.ndarray,
        grad_y: np.ndarray,
    ) -> dict:
        return {
            "distance_field": sdf,
            "grad_x":         grad_x,
            "grad_y":         grad_y,
            "resolution":     og.resolution,
            "origin":         [og.origin.x, og.origin.y],
            "ts":             time.time(),
            "frame_id":       og.frame_id,
            "backend":         "numpy",
        }

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["esdf"] = {
            "obstacle_threshold": self._obs_thr,
            "backend": "nav_kernel" if self._map_kernel is not None else "scipy",
        }
        return info
