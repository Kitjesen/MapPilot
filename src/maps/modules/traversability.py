"""TraversabilityCostModule - ESDF relay, slope view, and fused cost map.

NAV COMPUTE CONTRACT (docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md):
  This module produces the L2 SAFETY-GATING / observability signal `fused_cost`
  (a RISK grid = occupancy + slope + ESDF proximity). Its consumers are the
  GLOBAL safety gate (nav.mission.costmap), exploration, and the Gateway
  visualization -NOT the local planner's primary scoring. On S100P with PCT
  active, fused_cost is only a safe-goal/path-safety helper, not a planner.

The `esdf_field` relay to LocalPlanner is a RESERVED extension point:
nav_kernel LocalPlanner has no set_esdf binding yet, so ESDF does NOT
currently influence local scoring.

Two core functions:
  1. ESDF -> LocalPlanner - relay ESDF distance field + gradients (RESERVED:
     not yet consumed by nav_kernel scoring; see contract section 5).
  2. Slope -> Web - compute slope grid from ElevationMap and push via Gateway
     SSE for operator situational awareness (green/yellow/red overlay).

The module merges map-layer evidence without changing the ownership model:

1. ESDF is relayed to LocalPlanner for obstacle distance scoring.
2. Slope is computed from ElevationMap for operator visualization.
3. fused_cost gives the Python A* planner terrain awareness on dev/sim
   machines. On S100P with PCT active, it is only a safe-goal helper.

Merge rule: layered max() with hard constraint protection.
  L0  LETHAL(100) / INSCRIBED(99) from OccupancyGrid.
  L1  slope >= max_slope -> LETHAL.
  L2  slope soft cost 1..97, only on free cells.
  L3  ESDF proximity 0..proximity_cap, only on free cells.
  L4  terrain traversability risk grid, if provided, only on free cells.

Ports:
  In:  costmap (dict), elevation_map (dict), esdf (dict), traversability (dict)
  Out: fused_cost (dict), esdf_field (dict), slope_grid (dict)
"""

from __future__ import annotations

import logging
import time
from typing import Any

from maps.adapters.python.kernels import (
    create_map_kernel_backend,
    elevation_payload_to_result,
    empty_grid2d,
    grid_to_array,
    make_grid2d,
    terrain_risk_result_to_payload,
)
from runtime.module import Module
from runtime.msgs.numpy_compat import np
from runtime.registry import register
from runtime.runtime_interface import TOPICS, normalize_frame_id, topic_default_frame_id
from runtime.stream import In, Out

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
    """Bilinear resample src grid onto dst grid coordinates via lingtu_maps."""

    backend = create_map_kernel_backend()
    if backend is None:
        raise RuntimeError("native map kernel is required for grid resampling")
    return backend.runtime.resample_grid_bilinear(
        src,
        src_origin=src_origin,
        src_resolution=src_res,
        dst_shape=dst_shape,
        dst_origin=dst_origin,
        dst_resolution=dst_res,
        fill=fill,
    )


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
        self._default_frame_id = normalize_frame_id(frame_id) or topic_default_frame_id(TOPICS.exploration_grid)

        if max_slope_deg is not None:
            self._max_slope = max_slope_deg
        else:
            try:
                import math

                from runtime.config import get_config

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
        self._map_kernel = None

    def setup(self) -> None:
        self._map_kernel = create_map_kernel_backend()
        if self._map_kernel is None:
            raise RuntimeError("TraversabilityCostModule requires the native map kernel")

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
        self._try_fuse()

    def _on_esdf(self, data: dict) -> None:
        self._esdf_data = data
        self.esdf_field.publish(data)
        self._try_fuse()

    def _on_traversability(self, data: dict) -> None:
        self._trav_data = data
        self._try_fuse()

    def _try_fuse(self) -> None:
        """Layered priority merge triggered by the costmap input."""

        now = time.time()
        cm = self._costmap_data
        if cm is None:
            return
        if now - self._last_publish < self._interval:
            return
        self._last_publish = now

        obs_grid = np.asarray(cm["grid"], dtype=np.float32)
        res = cm["resolution"]
        origin = np.array(cm["origin"][:2], dtype=np.float64)
        frame_id = normalize_frame_id(cm.get("frame_id")) or self._default_frame_id
        shape = obs_grid.shape

        cpp_result = self._try_fuse_cpp(obs_grid, shape, origin, res, now, frame_id)
        if cpp_result is None:
            logger.warning("native traversability fusion failed; skipping fused_cost publish")
            return
        fused, slope_deg = cpp_result
        if slope_deg is not None:
            self.slope_grid.publish(
                {
                    "grid": slope_deg,
                    "resolution": res,
                    "origin": origin.tolist(),
                    "ts": now,
                    "frame_id": frame_id,
                    "backend": "cpp",
                }
            )
        self.fused_cost.publish(
            {
                "grid": fused,
                "resolution": res,
                "origin": origin.tolist(),
                "ts": now,
                "frame_id": frame_id,
                "backend": "cpp",
            }
        )
        return

    def _compute_traversability_cost(
        self,
        dst_shape: tuple[int, int],
        dst_origin: np.ndarray,
        dst_res: float,
    ) -> np.ndarray | None:
        """Read an optional terrain risk grid from nav.terrain.

        Accepted payload keys are deliberately narrow: values are expected to be
        a 0..100 risk/cost grid where larger means less traversable.
        """

        trav = self._trav_data
        if trav is None:
            return None
        raw = None
        for key in ("grid", "traversability", "cost", "costmap"):
            value = trav.get(key)
            if value is not None:
                raw = value
                break
        if raw is None:
            return None

        try:
            cost = np.asarray(raw, dtype=np.float32)
        except (TypeError, ValueError):
            return None
        if cost.ndim != 2:
            return None
        cost = np.nan_to_num(cost, nan=0.0, posinf=100.0, neginf=0.0)
        cost = np.clip(cost, 0, 100)

        trav_res = float(trav.get("resolution", dst_res))
        trav_origin = np.array(trav.get("origin", dst_origin)[:2], dtype=np.float64)
        if cost.shape != dst_shape or not np.allclose(trav_origin, dst_origin):
            cost = _resample_to_grid(
                cost,
                trav_origin,
                trav_res,
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
            "backend": "cpp",
        }
        return info

    def _try_fuse_cpp(
        self,
        obs_grid: np.ndarray,
        shape: tuple[int, int],
        origin: np.ndarray,
        res: float,
        now: float,
        frame_id: str,
    ) -> tuple[np.ndarray, np.ndarray | None] | None:
        """Run the production C++ map fusion path.

        Python remains responsible for optional layer packing because runtime
        layers can have different radii. The map math itself is in lingtu_maps.
        """

        if self._map_kernel is None:
            self._map_kernel = create_map_kernel_backend()
        runtime = self._map_kernel.runtime if self._map_kernel is not None else None
        if runtime is None:
            return None
        try:
            cost_grid = make_grid2d(runtime, obs_grid, resolution=res, origin=origin)
            slope_arr, terrain_arr = self._compute_cpp_terrain_layers(
                runtime,
                shape,
                origin,
                res,
                now,
                frame_id,
            )
            esdf_dist = self._compute_esdf_distance(shape, origin, res)
            trav_cost = self._compute_traversability_cost(shape, origin, res)
            if trav_cost is not None:
                terrain_arr = trav_cost if terrain_arr is None else np.maximum(terrain_arr, trav_cost)

            slope_grid = (
                make_grid2d(runtime, slope_arr, resolution=res, origin=origin)
                if slope_arr is not None
                else empty_grid2d(runtime)
            )
            esdf_grid = (
                make_grid2d(runtime, esdf_dist, resolution=res, origin=origin)
                if esdf_dist is not None
                else empty_grid2d(runtime)
            )
            terrain_grid = (
                make_grid2d(runtime, terrain_arr, resolution=res, origin=origin)
                if terrain_arr is not None
                else empty_grid2d(runtime)
            )

            params = runtime.TraversabilityParams()
            params.lethal = float(self.LETHAL)
            params.inscribed = float(self.INSCRIBED)
            params.max_slope_deg = float(self._max_slope)
            params.safe_distance = float(self._safe_dist)
            params.proximity_cap = float(self._prox_cap)
            fused = runtime.fuse_traversability_cost(
                cost_grid,
                slope_grid,
                esdf_grid,
                terrain_grid,
                params,
            )
            return grid_to_array(fused), slope_arr
        except Exception as exc:
            logger.debug("TraversabilityCost lingtu_maps fusion unavailable: %s", exc)
            return None

    def _compute_cpp_terrain_layers(
        self,
        runtime: Any,
        dst_shape: tuple[int, int],
        dst_origin: np.ndarray,
        dst_res: float,
        now: float,
        frame_id: str,
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        elev = self._elev_data
        if elev is None or elev.get("max_z") is None or elev.get("valid") is None:
            return None, None

        risk_params = runtime.TerrainRiskParams()
        risk_params.max_slope_deg = float(self._max_slope)
        risk = runtime.compute_terrain_risk(elevation_payload_to_result(runtime, elev), risk_params)
        payload = terrain_risk_result_to_payload(risk, ts=now, frame_id=frame_id)

        slope = np.asarray(payload["slope_deg"], dtype=np.float32)
        terrain = np.asarray(payload["grid"], dtype=np.float32)
        elev_origin = np.array(payload["origin"][:2], dtype=np.float64)
        elev_res = float(payload["resolution"])

        if slope.shape != dst_shape or not np.allclose(elev_origin, dst_origin):
            slope = _resample_to_grid(
                slope,
                elev_origin,
                elev_res,
                dst_shape,
                dst_origin,
                dst_res,
                fill=0.0,
            )
            terrain = _resample_to_grid(
                terrain,
                elev_origin,
                elev_res,
                dst_shape,
                dst_origin,
                dst_res,
                fill=0.0,
            )
        return slope, terrain

    def _compute_esdf_distance(
        self,
        dst_shape: tuple[int, int],
        dst_origin: np.ndarray,
        dst_res: float,
    ) -> np.ndarray | None:
        esdf = self._esdf_data
        if esdf is None:
            return None
        dist = esdf.get("distance_field")
        if dist is None:
            return None
        dist = np.asarray(dist, dtype=np.float32)
        esdf_res = float(esdf.get("resolution", dst_res))
        esdf_origin = np.array(esdf.get("origin", dst_origin)[:2], dtype=np.float64)
        if dist.shape != dst_shape or not np.allclose(esdf_origin, dst_origin):
            dist = _resample_to_grid(
                dist,
                esdf_origin,
                esdf_res,
                dst_shape,
                dst_origin,
                dst_res,
                fill=float(self._safe_dist),
            )
        return dist
