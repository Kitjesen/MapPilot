"""Traversable frontier candidate generation for inspection/exploration preview.

This module is the LingTu-native first slice of the TravExplorer direction:
frontiers remain ModulePort data, and command ownership stays with
NavigationModule/CmdVelMux/SafetyRing. The module never publishes cmd_vel and
is not wired to NavigationModule by default.
"""

from __future__ import annotations

import math
import time
from typing import Any

import numpy as np

from core.msgs.semantic import SceneGraph
from core.module import skill
from core.registry import register
from core.stream import In, Out
from nav.frontier_explorer_module import WavefrontFrontierExplorer


@register(
    "navigation",
    "traversable_frontier",
    description="TravExplorer-inspired traversable frontier candidate preview",
)
class TraversableFrontierModule(WavefrontFrontierExplorer):
    """Score wavefront frontier candidates with traversability evidence.

    Inputs:
        exploration_grid/costmap/odometry from the wavefront base module
        fused_cost, slope_grid, esdf_field, elevation_map from LingTu map layers

    Outputs:
        traversable_frontiers: ranked candidate dictionaries
        frontier_candidate: best candidate dictionary
    """

    fused_cost: In[dict]
    slope_grid: In[dict]
    esdf_field: In[dict]
    elevation_map: In[dict]
    scene_graph: In[SceneGraph]

    traversable_frontiers: Out[list]
    frontier_candidate: Out[dict]

    def __init__(
        self,
        *,
        max_slope_deg: float = 35.0,
        max_frontier_cost: float = 80.0,
        semantic_prior_weight: float = 0.0,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._max_slope_deg = max(1.0, float(max_slope_deg))
        self._max_frontier_cost = max(0.0, min(100.0, float(max_frontier_cost)))
        self._semantic_prior_weight = max(0.0, min(1.0, float(semantic_prior_weight)))
        self._fused_cost_data: dict | None = None
        self._slope_grid_data: dict | None = None
        self._esdf_field_data: dict | None = None
        self._elevation_map_data: dict | None = None
        self._scene_graph_data: SceneGraph | None = None
        self._last_candidates: list[dict[str, Any]] = []

    def setup(self) -> None:
        super().setup()
        self.fused_cost.subscribe(self._on_fused_cost)
        self.fused_cost.set_policy("latest")
        self.slope_grid.subscribe(self._on_slope_grid)
        self.slope_grid.set_policy("latest")
        self.esdf_field.subscribe(self._on_esdf_field)
        self.esdf_field.set_policy("latest")
        self.elevation_map.subscribe(self._on_elevation_map)
        self.elevation_map.set_policy("latest")
        self.scene_graph.subscribe(self._on_scene_graph)
        self.scene_graph.set_policy("latest")

    def _on_fused_cost(self, data: dict) -> None:
        with self._state_lock:
            self._fused_cost_data = data

    def _on_slope_grid(self, data: dict) -> None:
        with self._state_lock:
            self._slope_grid_data = data

    def _on_esdf_field(self, data: dict) -> None:
        with self._state_lock:
            self._esdf_field_data = data

    def _on_elevation_map(self, data: dict) -> None:
        with self._state_lock:
            self._elevation_map_data = data

    def _on_scene_graph(self, data: SceneGraph) -> None:
        with self._state_lock:
            self._scene_graph_data = data

    def _compute_candidates(self) -> list[dict[str, Any]]:
        """Internal: compute and return frontier candidates as structured data."""
        base_frontiers = super()._compute_frontier_clusters()
        with self._state_lock:
            fused_cost = self._fused_cost_data
            slope_grid = self._slope_grid_data
            esdf_field = self._esdf_field_data
            elevation_map = self._elevation_map_data
            scene_graph = self._scene_graph_data

        candidates = [
            self._enrich_frontier(
                frontier,
                index=index,
                fused_cost=fused_cost,
                slope_grid=slope_grid,
                esdf_field=esdf_field,
                elevation_map=elevation_map,
                scene_graph=scene_graph,
            )
            for index, frontier in enumerate(base_frontiers)
        ]
        candidates.sort(key=lambda item: float(item["reachable_score"]), reverse=True)
        self._last_candidates = candidates
        return candidates

    @skill
    def get_traversable_frontiers(self) -> str:
        """Return ranked frontier candidates enriched with traversability evidence."""
        import json
        return json.dumps(self._compute_candidates(), default=str)

    @skill
    def refresh_candidates(self) -> str:
        """Compute and publish candidates without publishing any motion command."""

        candidates = self._compute_candidates()
        self.traversable_frontiers.publish(candidates)
        if candidates:
            self.frontier_candidate.publish(candidates[0])
        import json
        return json.dumps({
            "candidate_count": len(candidates),
            "best_candidate_id": candidates[0]["id"] if candidates else None,
            "command_published": False,
        })

    def health(self) -> dict[str, Any]:
        info = super().health()
        with self._state_lock:
            layers = {
                "fused_cost": self._fused_cost_data is not None,
                "slope_grid": self._slope_grid_data is not None,
                "esdf_field": self._esdf_field_data is not None,
                "elevation_map": self._elevation_map_data is not None,
                "scene_graph": self._scene_graph_data is not None,
            }
            last_count = len(self._last_candidates)
        info["traversable_frontier"] = {
            "candidate_count": last_count,
            "evidence_layers": layers,
            "max_slope_deg": self._max_slope_deg,
            "max_frontier_cost": self._max_frontier_cost,
            "command_owner": "NavigationModule/CmdVelMux/SafetyRing",
            "publishes_motion_command": False,
        }
        return info

    def _enrich_frontier(
        self,
        frontier: dict[str, Any],
        *,
        index: int,
        fused_cost: dict | None,
        slope_grid: dict | None,
        esdf_field: dict | None,
        elevation_map: dict | None,
        scene_graph: SceneGraph | None,
    ) -> dict[str, Any]:
        x = float(frontier.get("cx", 0.0))
        y = float(frontier.get("cy", 0.0))
        base_score = _finite01(frontier.get("score"), default=0.0)

        terrain_cost = _sample_payload_grid(fused_cost, x, y, key="grid")
        slope_deg = _sample_payload_grid(slope_grid, x, y, key="grid")
        clearance_m = _sample_payload_grid(esdf_field, x, y, key="distance_field")
        support_height = _sample_payload_grid(elevation_map, x, y, key="max_z")
        if support_height is None:
            support_height = _sample_payload_grid(elevation_map, x, y, key="grid")
        z = 0.0 if support_height is None else float(support_height)

        cost_score = 1.0
        if terrain_cost is not None:
            cost_score = max(0.0, 1.0 - max(0.0, float(terrain_cost)) / 100.0)
        slope_score = 1.0
        if slope_deg is not None:
            slope_score = max(0.0, 1.0 - max(0.0, float(slope_deg)) / self._max_slope_deg)
        clearance_score = 0.5
        if clearance_m is not None:
            clearance_score = max(0.0, min(1.0, float(clearance_m) / max(self._safe_dist, 1e-6)))

        semantic_evidence = _semantic_evidence(scene_graph, x, y, z=z)
        semantic_value = semantic_evidence["semantic_value"]
        reachable_score = (
            0.50 * base_score
            + 0.25 * cost_score
            + 0.15 * slope_score
            + 0.10 * clearance_score
            + self._semantic_prior_weight * semantic_value
        )
        reachable_score = max(0.0, min(1.0, reachable_score))

        reasons: list[str] = []
        state = "active"
        if terrain_cost is None:
            reasons.append("missing_fused_cost")
        elif float(terrain_cost) >= self._max_frontier_cost:
            state = "blocked"
            reasons.append("high_traversability_cost")
        if slope_deg is None:
            reasons.append("missing_slope_grid")
        elif float(slope_deg) >= self._max_slope_deg:
            state = "blocked"
            reasons.append("slope_over_limit")
        if clearance_m is None:
            reasons.append("missing_esdf_clearance")
        if support_height is None:
            reasons.append("missing_support_height")

        evidence_layers = {
            "fused_cost": terrain_cost is not None,
            "slope_grid": slope_deg is not None,
            "esdf_field": clearance_m is not None,
            "elevation_map": support_height is not None,
            "scene_graph": bool(semantic_evidence["object_count"]),
        }
        support_type = _support_type(
            slope_deg,
            terrain_cost,
            self._max_slope_deg,
            self._max_frontier_cost,
        )
        centroid = [x, y, z]
        return {
            "id": f"traversable_frontier_{index}",
            "source": "traversable_frontier",
            "state": state,
            "centroid_3d": centroid,
            "cx": x,
            "cy": y,
            "z": z,
            "support_height": z,
            "support_z": z,
            "support_type": support_type,
            "size": int(frontier.get("size") or 0),
            "score": reachable_score,
            "base_frontier_score": base_score,
            "reachable_score": reachable_score,
            "terrain_cost": None if terrain_cost is None else float(terrain_cost),
            "slope_deg": None if slope_deg is None else float(slope_deg),
            "esdf_clearance_m": None if clearance_m is None else float(clearance_m),
            "semantic_value": semantic_value,
            "nearby_labels": semantic_evidence["nearby_labels"],
            "semantic_evidence": semantic_evidence,
            "evidence_layers": evidence_layers,
            "distance": float(frontier.get("distance") or 0.0),
            "reasons": reasons,
            "preview": True,
            "command_published": False,
            "ts": time.time(),
        }


def _finite01(value: Any, *, default: float) -> float:
    try:
        val = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(val):
        return default
    return max(0.0, min(1.0, val))


def _support_type(
    slope_deg: float | None,
    terrain_cost: float | None,
    max_slope_deg: float,
    max_frontier_cost: float,
) -> str:
    if terrain_cost is not None and float(terrain_cost) >= max_frontier_cost:
        return "blocked"
    if slope_deg is None:
        return "unknown"
    slope = float(slope_deg)
    if slope >= max_slope_deg:
        return "steep"
    if slope >= 12.0:
        return "slope"
    return "flat"


def _sample_payload_grid(
    payload: dict | None,
    x: float,
    y: float,
    *,
    key: str,
) -> float | None:
    if not isinstance(payload, dict):
        return None
    grid = payload.get(key)
    if grid is None and key != "grid":
        grid = payload.get("grid")
    if grid is None:
        return None
    arr = np.asarray(grid, dtype=np.float32)
    if arr.ndim == 1:
        height = int(payload.get("height", 0) or 0)
        width = int(payload.get("width", 0) or 0)
        if height <= 0 or width <= 0:
            return None
        arr = arr.reshape(height, width)
    if arr.ndim != 2 or arr.size == 0:
        return None

    resolution = float(payload.get("resolution", 0.0) or 0.0)
    if resolution <= 0.0:
        return None
    origin_x, origin_y = _payload_origin_xy(payload)
    col = int(round((float(x) - origin_x) / resolution))
    row = int(round((float(y) - origin_y) / resolution))
    if not (0 <= row < arr.shape[0] and 0 <= col < arr.shape[1]):
        return None
    val = float(arr[row, col])
    if not math.isfinite(val):
        return None
    return val


def _payload_origin_xy(payload: dict) -> tuple[float, float]:
    origin = payload.get("origin")
    if isinstance(origin, np.ndarray):
        origin = origin.tolist()
    if isinstance(origin, (list, tuple)) and len(origin) >= 2:
        return float(origin[0]), float(origin[1])
    if hasattr(origin, "position"):
        pos = origin.position
        return float(getattr(pos, "x", 0.0)), float(getattr(pos, "y", 0.0))
    return float(payload.get("origin_x", 0.0) or 0.0), float(payload.get("origin_y", 0.0) or 0.0)


def _semantic_evidence(
    scene_graph: SceneGraph | None,
    x: float,
    y: float,
    *,
    z: float,
    radius_m: float = 3.0,
) -> dict[str, Any]:
    if scene_graph is None:
        return {
            "source": "none",
            "object_count": 0,
            "nearby_labels": [],
            "semantic_value": 0.0,
        }

    matches: list[tuple[float, str, float]] = []
    radius = max(0.1, float(radius_m))
    for obj in getattr(scene_graph, "objects", []) or []:
        label = str(getattr(obj, "label", "") or "").strip()
        if not label:
            continue
        pos = getattr(obj, "position", None)
        if pos is None:
            continue
        ox = float(getattr(pos, "x", 0.0))
        oy = float(getattr(pos, "y", 0.0))
        oz = float(getattr(pos, "z", 0.0))
        dist = math.sqrt((ox - float(x)) ** 2 + (oy - float(y)) ** 2 + (oz - float(z)) ** 2)
        if dist > radius:
            continue
        confidence = _finite01(getattr(obj, "confidence", 0.0), default=0.0)
        matches.append((dist, label, confidence))

    matches.sort(key=lambda item: item[0])
    labels: list[str] = []
    best_value = 0.0
    for dist, label, confidence in matches:
        if label not in labels:
            labels.append(label)
        proximity = max(0.0, 1.0 - dist / radius)
        best_value = max(best_value, confidence * proximity)

    return {
        "source": "scene_graph",
        "object_count": len(matches),
        "nearby_labels": labels[:5],
        "semantic_value": max(0.0, min(1.0, best_value)),
    }
