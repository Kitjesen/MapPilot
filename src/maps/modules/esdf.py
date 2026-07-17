"""ESDFModule - 2-D Euclidean signed distance field from occupancy grid.

Subscribes to OccupancyGridModule.occupancy_grid (OccupancyGrid).
Computes per-cell signed distance to the nearest obstacle:

  d > 0  - distance in metres from a free cell to the nearest obstacle
  d = 0  - obstacle boundary
  d < 0  - penetration depth inside an obstacle

Also publishes the gradient vector used by trajectory optimisers
(MPPI / TEB / CHOMP) for collision-cost gradient descent.

Ports:
  In:  occupancy_grid (OccupancyGrid)
  Out: esdf (dict: distance_field, grad_x, grad_y, resolution, origin)
"""

from __future__ import annotations

import logging
import time
from typing import Any

from maps.adapters.python.kernels import (
    create_map_kernel_backend,
    esdf_result_to_payload,
    make_grid2d,
)
from runtime.module import Module
from runtime.msgs.nav import OccupancyGrid
from runtime.registry import register
from runtime.stream import In, Out

logger = logging.getLogger(__name__)


@register("map", "esdf", description="2D ESDF + gradient from occupancy grid")
class ESDFModule(Module, layer=2):
    """2-D signed distance field from the native C++ map kernel.

    Throttled to 1 Hz by default. The Module owns only port delivery and
    payload packing; the distance transform and gradients run in C++.

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
        obstacle_threshold: int = 50,  # cells with cost >= this are obstacles (0-100)
        publish_hz: float = 1.0,
        **kw,
    ):
        super().__init__(**kw)
        self._obs_thr = obstacle_threshold
        self._interval = 1.0 / publish_hz
        self._map_kernel = None

    def setup(self) -> None:
        self._map_kernel = create_map_kernel_backend()
        if self._map_kernel is None:
            raise RuntimeError("ESDFModule requires the native map kernel")
        self.occupancy_grid.subscribe(self._on_grid)
        self.occupancy_grid.set_policy("throttle", interval=self._interval)

    def _on_grid(self, og: OccupancyGrid) -> None:
        grid = make_grid2d(
            self._map_kernel.runtime,
            og.grid,
            resolution=og.resolution,
            origin=[og.origin.x, og.origin.y],
        )
        result = self._map_kernel.runtime.compute_esdf(grid, float(self._obs_thr))
        self.esdf.publish(esdf_result_to_payload(result, ts=time.time(), frame_id=og.frame_id))

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["esdf"] = {
            "obstacle_threshold": self._obs_thr,
            "backend": "cpp",
        }
        return info
