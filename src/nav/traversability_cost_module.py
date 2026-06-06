"""TraversabilityCostModule - ESDF relay, slope view, and fused cost map.

NAV COMPUTE CONTRACT (docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md):
  This module produces the L2 SAFETY-GATING / observability signal `fused_cost`
  (a RISK grid = occupancy + slope + ESDF proximity). Its consumers are the
  GLOBAL safety gate (NavigationModule.costmap), exploration, and the Gateway
  visualization — NOT the local planner's primary scoring. On S100P with PCT
  active, fused_cost is only a safe-goal/path-safety helper, not a planner.

The `esdf_field` relay to LocalPlanner is a RESERVED extension point:
nav_core's LocalPlannerCore has no set_esdf binding yet, so ESDF does NOT
currently influence local scoring.

Two core functions:
  1. ESDF -> LocalPlanner - relay ESDF distance field + gradients (RESERVED:
     not yet consumed by nav_core scoring; see contract section 5).
  2. Slope -> Web - compute slope grid from ElevationMap and push via Gateway
     SSE for operator situational awareness (green/yellow/red overlay).

The module merges map-layer evidence without changing the ownership model:

1. ESDF is relayed to LocalPlannerModule for obstacle distance scoring.
2. Slope is computed from ElevationMap for operator visualization.
3. fused_cost gives the Python A* planner terrain awareness on dev/sim
   machines. On S100P with PCT active, it is only a safe-goal helper.

Merge rule: layered max() with hard constraint protection.
  L0  LETHAL(100) / INSCRIBED(99) from OccupancyGrid.
  L1  slope >= max_slope -> LETHAL.
  L2  slope soft cost 1..97, only on free cells.
  L3  ESDF proximity 0..proximity_cap, only on free cells.

Ports:
  In:  costmap (dict), elevation_map (dict), esdf (dict), traversability (dict)
  Out: fused_cost (dict), esdf_field (dict), slope_grid (dict)
"""

from __future__ import annotations

import logging
import time
from typing import Any

from core.module import Module
from core.msgs.numpy_compat import np
from core.registry import register
from core.runtime_interface import TOPICS, normalize_frame_id, topic_default_frame_id
from core.stream import In, Out

logger = logging.getLogger(__name__)


def _resample_to_grid(
    src: np.ndarray,
    src_origin: np.ndarray,
    src_res: float,
    dst_shape: tuple[int, int],
    dst_origin: np.ndarray,
    dst_res: float,
    fill: float = 0.0,
) -> np.ndarray:
    """Bilinear resample src grid onto dst grid coordinates via scipy."""

    from scipy.ndimage import map_coordinates

    dst_h, dst_w = dst_shape
    ys = dst_origin[1] + (np.arange(dst_h) + 0.5) * dst_res
    xs = dst_origin[0] + (np.arange(dst_w) + 0.5) * dst_res

    src_row = (ys - src_origin[1]) / src_res - 0.5
    src_col = (xs - src_origin[0]) / src_res - 0.5

    row_grid = np.broadcast_to(src_row[:, None], dst_shape)
    col_grid = np.broadcast_to(src_col[None, :], dst_shape)
    coords = np.array([row_grid.ravel(), col_grid.ravel()])

    out = map_coordinates(
        src.astype(np.float64),
        coords,
        order=1,
        mode="constant",
        cval=float(fill),
    ).reshape(dst_shape).astype(np.float32)
    return out


@register(
    "map",
    "traversability_cost",
    description="Fuse obstacle + slope + ESDF into unified traversability cost",
)
class TraversabilityCostModule(Module, layer=2):
    """Map layer hub for ESDF relay, slope visualization, and A* cost fusion."""

    costmap: In[dict]
    elevation_map: In[dict]
    esdf: In[dict]
    traversability: In[dict]

    fused_cost: Out[dict]
    esdf_field: Out[dict]
    slope_grid: Out[dict]

    LETHAL = 100
    INSCRIBED = 99

    def __init__(
        self,
        safe_distance: float = 1.5,
        max_slope_deg: float | None = None,
        proximity_cap: float = 50.0,
        publish_hz: float = 2.0,
        frame_id: str | None = None,
        **kw,
    ):
        super().__init__(**kw)
        self._safe_dist = safe_distance
        self._prox_cap = proximity_cap
        self._interval = 1.0 / publish_hz
        self._default_frame_id = (
            normalize_frame_id(frame_id)
            or topic_default_frame_id(TOPICS.exploration_grid)
        )

        if max_slope_deg is not None:
            self._max_slope = max_slope_deg
        else:
            try:
                import math

                from core.config import get_config

                cfg = get_config()
                slope_tan = cfg.raw.get("terrain_analysis", {}).get("slope_max", 1.0)
                self._max_slope = math.degrees(math.atan(slope_tan))
            except (ImportError, AttributeError):
                self._max_slope = 45.0

        self._costmap_data: dict | None = None
        self._elev_data: dict | None = None
        self._esdf_data: dict | None = None
        self._trav_data: dict | None = None
        self._last_publish: float = 0.0

    def setup(self) -> None:
        self.costmap.subscribe(self._on_costmap)
        self.elevation_map.subscribe(self._on_elevation)
        self.elevation_map.set_policy("latest")
        self.esdf.subscribe(self._on_esdf)
        self.esdf.set_policy("latest")
        self.traversability.subscribe(self._on_traversability)
        self.traversability.set_policy("latest")

    def _on_costmap(self, data: dict) -> None:
        self._costmap_data = data
        self._try_fuse()

    def _on_elevation(self, data: dict) -> None:
        self._elev_data = data

    def _on_esdf(self, data: dict) -> None:
        self._esdf_data = data
        self.esdf_field.publish(data)

    def _on_traversability(self, data: dict) -> None:
        self._trav_data = data

    def _try_fuse(self) -> None:
        """Layered priority merge triggered by the costmap input."""

        now = time.time()
        if now - self._last_publish < self._interval:
            return
        self._last_publish = now

        cm = self._costmap_data
        if cm is None:
            return

        obs_grid = np.asarray(cm["grid"], dtype=np.float32)
        res = cm["resolution"]
        origin = np.array(cm["origin"][:2], dtype=np.float64)
        frame_id = (
            normalize_frame_id(cm.get("frame_id"))
            or self._default_frame_id
        )
        shape = obs_grid.shape

        fused = obs_grid.copy()
        is_hard = obs_grid >= self.INSCRIBED

        slope_deg = self._compute_slope(shape, origin, res)
        if slope_deg is not None:
            slope_lethal = slope_deg >= self._max_slope
            fused[slope_lethal & ~is_hard] = self.LETHAL

            soft_mask = ~is_hard & ~slope_lethal & (slope_deg > 3.0)
            slope_cost = np.clip(slope_deg / self._max_slope, 0, 0.97) * 100.0
            fused[soft_mask] = np.maximum(fused[soft_mask], slope_cost[soft_mask])

            self.slope_grid.publish(
                {
                    "grid": slope_deg,
                    "resolution": res,
                    "origin": origin.tolist(),
                    "ts": now,
                    "frame_id": frame_id,
                }
            )

        prox_cost = self._compute_proximity(shape, origin, res)
        if prox_cost is not None:
            free_mask = fused < self.INSCRIBED
            soft_prox = np.clip(prox_cost * (self._prox_cap / 100.0), 0, self._prox_cap)
            fused[free_mask] = np.maximum(fused[free_mask], soft_prox[free_mask])

        fused = np.clip(fused, 0, 100).astype(np.float32)

        self.fused_cost.publish(
            {
                "grid": fused,
                "resolution": res,
                "origin": origin.tolist(),
                "ts": now,
                "frame_id": frame_id,
            }
        )

    def _compute_slope(
        self,
        dst_shape: tuple[int, int],
        dst_origin: np.ndarray,
        dst_res: float,
    ) -> np.ndarray | None:
        """Compute slope in degrees from elevation grid, resampled to dst grid."""

        elev = self._elev_data
        if elev is None:
            return None

        max_z = elev.get("max_z")
        valid = elev.get("valid")
        if max_z is None or valid is None:
            return None

        max_z = np.asarray(max_z, dtype=np.float32)
        valid = np.asarray(valid, dtype=bool)
        elev_res = elev.get("resolution", 0.2)
        elev_origin = np.array(elev["origin"][:2], dtype=np.float64)

        z = np.where(valid, max_z, 0.0)
        dzdx = np.gradient(z, elev_res, axis=1)
        dzdy = np.gradient(z, elev_res, axis=0)
        slope_rad = np.arctan(np.sqrt(dzdx**2 + dzdy**2))
        slope_deg = np.degrees(slope_rad).astype(np.float32)
        slope_deg[~valid] = 0.0

        if slope_deg.shape != dst_shape or not np.allclose(elev_origin, dst_origin):
            slope_deg = _resample_to_grid(
                slope_deg,
                elev_origin,
                elev_res,
                dst_shape,
                dst_origin,
                dst_res,
                fill=0.0,
            )
        return slope_deg

    def _compute_proximity(
        self,
        dst_shape: tuple[int, int],
        dst_origin: np.ndarray,
        dst_res: float,
    ) -> np.ndarray | None:
        """Compute ESDF proximity cost."""

        esdf = self._esdf_data
        if esdf is None:
            return None

        dist = esdf.get("distance_field")
        if dist is None:
            return None

        dist = np.asarray(dist, dtype=np.float32)
        esdf_res = esdf.get("resolution", 0.2)
        esdf_origin = np.array(esdf["origin"][:2], dtype=np.float64)

        cost = np.clip(1.0 - dist / self._safe_dist, 0, 1) * 100.0

        if cost.shape != dst_shape or not np.allclose(esdf_origin, dst_origin):
            cost = _resample_to_grid(
                cost,
                esdf_origin,
                esdf_res,
                dst_shape,
                dst_origin,
                dst_res,
                fill=0.0,
            )
        return cost

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["traversability_cost"] = {
            "merge": "layered_max (LETHAL > INSCRIBED > slope > proximity)",
            "safe_distance": self._safe_dist,
            "max_slope_deg": self._max_slope,
            "proximity_cap": self._prox_cap,
            "default_frame_id": self._default_frame_id,
            "layers": {
                "costmap": self._costmap_data is not None,
                "elevation": self._elev_data is not None,
                "esdf": self._esdf_data is not None,
                "traversability": self._trav_data is not None,
            },
        }
        return info
