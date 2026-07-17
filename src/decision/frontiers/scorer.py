"""Frontier extraction and semantic frontier scoring."""

from __future__ import annotations

import logging
import math
import time as _time_mod
from collections import deque
from pathlib import Path
from typing import Any

import yaml

from runtime.msgs.numpy_compat import np
from runtime.utils.sanitize import safe_json_dumps

logger = logging.getLogger(__name__)


_DEFAULT_SEMANTIC_PRIOR_WEIGHT: float = 0.15
_frontier_config_weight: float = _DEFAULT_SEMANTIC_PRIOR_WEIGHT
_frontier_weights_loaded: bool = False


def _load_semantic_scoring_yaml() -> dict:
    """Load config/semantic_scoring.yaml from the repo root."""
    repo_root = Path(__file__).resolve().parents[3]
    yaml_path = repo_root / "config" / "semantic_scoring.yaml"
    try:
        with open(yaml_path, encoding="utf-8") as fh:
            return yaml.safe_load(fh) or {}
    except (FileNotFoundError, OSError):
        return {}


def _load_frontier_weights() -> None:
    """Load frontier_scorer.semantic_prior_weight from config (once)."""
    global _frontier_config_weight, _frontier_weights_loaded
    if _frontier_weights_loaded:
        return
    _frontier_weights_loaded = True
    try:
        section = _load_semantic_scoring_yaml().get("frontier_scorer")
        if section is None:
            logger.info(
                "frontier_scorer section absent - using default weights. See config/semantic_scoring.yaml to tune."
            )
            return
        _frontier_config_weight = float(section.get("semantic_prior_weight", _DEFAULT_SEMANTIC_PRIOR_WEIGHT))
        logger.debug(
            "frontier_scorer semantic_prior_weight=%.3f loaded from config",
            _frontier_config_weight,
        )
    except Exception as exc:
        logger.warning("Failed to load frontier_scorer weights from config: %s", exc)


from .types import (
    FREE_CELL,
    UNKNOWN_CELL,
    Frontier,
    angle_diff,
    angle_to_label,
    compute_kg_room_score,
    compute_semantic_prior_score,
    compute_uncertainty_score,
    cooccurrence_score,
    extract_bilingual_keywords,
    generate_frontier_description,
    tsp_sort_frontiers,
)

logger = logging.getLogger(__name__)


class FrontierScorer:
    """Frontier Scorer."""

    def __init__(
        self,
        min_frontier_size: int = 5,
        max_frontiers: int = 10,
        cluster_radius_cells: int = 3,
        distance_weight: float = 0.2,
        novelty_weight: float = 0.3,
        language_weight: float = 0.2,
        grounding_weight: float = 0.3,
        vision_weight: float = 0.0,
        semantic_prior_weight: float = 0.15,
        semantic_uncertainty_weight: float = 0.15,
        tsp_reorder: bool = True,
        tsp_frontier_limit: int = 20,
        tsp_ig_radius_cells: int = 10,
        novelty_distance: float = 5.0,
        nearby_object_radius: float = 3.0,
        grounding_angle_threshold: float = 0.7854,
        cooccurrence_bonus: float = 0.25,
        grounding_spatial_bonus: float = 0.1,
        grounding_keyword_bonus: float = 0.4,
        grounding_relation_bonus: float = 0.15,
    ):
        self.min_frontier_size = min_frontier_size
        self.max_frontiers = max_frontiers
        self.cluster_radius_cells = cluster_radius_cells
        self.distance_weight = distance_weight
        self.novelty_weight = novelty_weight
        self.language_weight = language_weight
        self.grounding_weight = grounding_weight
        self.novelty_distance = max(novelty_distance, 0.1)
        self.nearby_object_radius = max(nearby_object_radius, 0.1)
        self.grounding_angle_threshold = max(grounding_angle_threshold, 0.01)
        self.cooccurrence_bonus = cooccurrence_bonus
        self.grounding_spatial_bonus = grounding_spatial_bonus
        self.grounding_keyword_bonus = grounding_keyword_bonus
        self.grounding_relation_bonus = grounding_relation_bonus

        self.vision_weight = max(vision_weight, 0.0)
        self.semantic_prior_weight = max(semantic_prior_weight, 0.0)
        self.semantic_uncertainty_weight = max(semantic_uncertainty_weight, 0.0)
        self._room_type_posteriors: dict[int, Any] = {}

        self.tsp_reorder = tsp_reorder
        self.tsp_frontier_limit = max(tsp_frontier_limit, 2)
        self.tsp_ig_radius_cells = max(tsp_ig_radius_cells, 1)

        self._grid: np.ndarray | None = None
        self._resolution: float = 0.05
        self._origin_x: float = 0.0
        self._origin_y: float = 0.0
        self._frontiers: list[Frontier] = []

        self._failed_positions: list[np.ndarray] = []
        self._failure_penalty_radius: float = 3.0
        self._failure_penalty_decay: float = 0.7

        # Sticky frontier: anti-oscillation (ASCENT-inspired)
        self._selected_history: list[tuple[np.ndarray, float]] = []
        self._sticky_distance: float = 1.5  # meters
        self._sticky_cooldown: float = 60.0  # seconds
        self._sticky_max_penalty: float = 0.5  # max score multiplier reduction

        self._directional_features: dict[int, np.ndarray] = {}
        self._clip_encoder = None

        self._semantic_prior_engine = None
        self._room_priors_cache: dict[int, float] = {}

        self._room_object_kg = None

        self._frontier_desc_cache: dict[str, str] = {}

        self._bilingual_kw_cache: dict[str, set[str]] = {}

    def update_costmap(
        self,
        grid_data: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
    ) -> None:
        """Update costmap."""
        self._grid = grid_data
        self._resolution = resolution
        self._origin_x = origin_x
        self._origin_y = origin_y

    def extract_frontiers(self, robot_position: np.ndarray) -> list[Frontier]:
        """Extract frontiers."""
        if self._grid is None:
            return []

        rows, cols = self._grid.shape
        frontier_mask = np.zeros((rows, cols), dtype=bool)

        free_mask = self._grid[1:-1, 1:-1] == FREE_CELL
        has_unknown_neighbor = (
            (self._grid[:-2, 1:-1] == UNKNOWN_CELL)
            | (self._grid[2:, 1:-1] == UNKNOWN_CELL)
            | (self._grid[1:-1, :-2] == UNKNOWN_CELL)
            | (self._grid[1:-1, 2:] == UNKNOWN_CELL)
        )
        frontier_mask[1:-1, 1:-1] = free_mask & has_unknown_neighbor

        visited = np.zeros_like(frontier_mask, dtype=bool)
        clusters: list[list[tuple[int, int]]] = []
        _bfs_start = _time_mod.time()
        _bfs_timeout = False

        for r in range(rows):
            if _bfs_timeout:
                break
            for c in range(cols):
                if _bfs_timeout:
                    break
                if frontier_mask[r, c] and not visited[r, c]:
                    cluster: list[tuple[int, int]] = []
                    queue = deque([(r, c)])
                    visited[r, c] = True

                    while queue:
                        if len(cluster) % 200 == 0 and _time_mod.time() - _bfs_start > 0.5:
                            _bfs_timeout = True
                            break
                        cr, cc = queue.popleft()
                        cluster.append((cr, cc))
                        for dr in range(-self.cluster_radius_cells, self.cluster_radius_cells + 1):
                            for dc in range(-self.cluster_radius_cells, self.cluster_radius_cells + 1):
                                nr, nc = cr + dr, cc + dc
                                if 0 <= nr < rows and 0 <= nc < cols and frontier_mask[nr, nc] and not visited[nr, nc]:
                                    visited[nr, nc] = True
                                    queue.append((nr, nc))

                    if not _bfs_timeout and len(cluster) >= self.min_frontier_size:
                        clusters.append(cluster)

                    if _bfs_timeout or _time_mod.time() - _bfs_start > 0.5:
                        if not _bfs_timeout:
                            _bfs_timeout = True
                        logger.warning(
                            "Frontier BFS exceeded 500ms budget, returning partial "
                            "results (%d clusters, last cluster discarded)",
                            len(clusters),
                        )
                        break

        self._frontiers = []
        for i, cluster in enumerate(clusters):
            cells = np.array(cluster, dtype=np.float64)
            center_cell = cells.mean(axis=0)
            center_world = np.array(
                [
                    self._origin_x + center_cell[1] * self._resolution,
                    self._origin_y + center_cell[0] * self._resolution,
                ]
            )
            dist = float(np.linalg.norm(center_world - robot_position[:2]))
            dx = center_world[0] - robot_position[0]
            dy = center_world[1] - robot_position[1]
            direction = angle_to_label(math.atan2(dy, dx))
            self._frontiers.append(
                Frontier(
                    frontier_id=i,
                    cells=cluster,
                    center=center_cell,
                    center_world=center_world,
                    size=len(cluster),
                    distance=dist,
                    direction_label=direction,
                )
            )

        self._frontiers.sort(key=lambda f: f.size, reverse=True)
        self._frontiers = self._frontiers[: self.max_frontiers]
        return self._frontiers

    def set_clip_encoder(self, clip_encoder) -> None:
        """Set clip encoder."""
        self._clip_encoder = clip_encoder
        if self.vision_weight <= 0.0:
            self.vision_weight = 0.15
            logger.info("Frontier vision scoring enabled (weight=0.15)")

    def set_semantic_prior_engine(self, engine) -> None:
        """Set semantic prior engine."""
        self._semantic_prior_engine = engine
        if self.semantic_prior_weight <= 0.0:
            self.semantic_prior_weight = 0.2
            logger.info("Frontier semantic prior scoring enabled (weight=0.2)")

    def set_room_type_posteriors(self, posteriors: dict[int, Any]) -> None:
        """Set room type posteriors."""
        self._room_type_posteriors = dict(posteriors)

    def set_room_type_posteriors_from_json(self, posteriors_json: dict[str, dict]) -> None:
        """Set room type posteriors from json."""

        class _PosteriorProxy:
            __slots__ = ("entropy", "hypotheses")

            def __init__(self, entropy: float, top3: list):
                self.entropy = entropy
                self.hypotheses = {}
                for item in top3:
                    if isinstance(item, dict):
                        for k, v in item.items():
                            self.hypotheses[k] = v

        parsed: dict[int, Any] = {}
        for rid_str, data in posteriors_json.items():
            try:
                rid = int(rid_str)
                parsed[rid] = _PosteriorProxy(
                    float(data.get("entropy", 0.0)),
                    data.get("top3", []),
                )
            except (ValueError, TypeError, AttributeError):
                continue
        self._room_type_posteriors = parsed

    def set_room_object_kg(self, kg) -> None:
        """Set room object kg."""
        self._room_object_kg = kg

    def record_frontier_failure(self, position: np.ndarray) -> None:
        """Record frontier failure."""
        pos = np.asarray(position[:2], dtype=np.float64)
        self._failed_positions.append(pos)
        logger.info(
            "Frontier failure recorded at (%.2f, %.2f), total failures: %d",
            pos[0],
            pos[1],
            len(self._failed_positions),
        )

    def clear_failure_memory(self) -> None:
        """Clear failure memory."""
        self._failed_positions.clear()
        self._selected_history.clear()

    def record_frontier_selection(self, position: np.ndarray) -> None:
        """Record a selected frontier position to prevent re-selection oscillation."""
        pos = np.asarray(position[:2], dtype=np.float64)
        now = _time_mod.time()
        self._selected_history.append((pos, now))
        # Prune expired entries
        cutoff = now - self._sticky_cooldown
        self._selected_history = [(p, t) for p, t in self._selected_history if t > cutoff]

    def clear_sticky_history(self) -> None:
        """Clear sticky frontier history (on new task / goal change)."""
        self._selected_history.clear()

    def _compute_sticky_penalty(self, frontier_center: np.ndarray) -> float:
        """Penalize frontiers too close to recently selected ones.

        Prevents oscillation between adjacent frontiers. Penalty decays
        with both time elapsed and spatial distance.
        """
        if not self._selected_history:
            return 0.0
        now = _time_mod.time()
        max_penalty = 0.0
        for pos, ts in self._selected_history:
            age = now - ts
            if age > self._sticky_cooldown:
                continue
            dist = float(np.linalg.norm(frontier_center - pos))
            if dist < self._sticky_distance:
                time_factor = 1.0 - (age / self._sticky_cooldown)
                dist_factor = 1.0 - (dist / self._sticky_distance)
                penalty = self._sticky_max_penalty * time_factor * dist_factor
                max_penalty = max(max_penalty, penalty)
        return min(1.0, max_penalty)

    def _compute_failure_penalty(self, frontier_center: np.ndarray) -> float:
        """Compute failure penalty."""
        if not self._failed_positions:
            return 0.0
        max_penalty = 0.0
        for fp in self._failed_positions:
            dist = float(np.linalg.norm(frontier_center - fp))
            if dist < self._failure_penalty_radius:
                penalty = self._failure_penalty_decay * (1.0 - dist / self._failure_penalty_radius)
                max_penalty = max(max_penalty, penalty)
        return min(1.0, max_penalty)

    def update_room_priors(
        self,
        instruction: str,
        rooms: list[dict],
        visited_room_ids: set | None = None,
    ) -> None:
        """Update room priors."""
        if not self._semantic_prior_engine:
            return
        priors = self._semantic_prior_engine.score_rooms_for_target(
            instruction,
            rooms,
            visited_room_ids,
        )
        self._room_priors_cache.clear()
        for rp in priors:
            self._room_priors_cache[rp.room_id] = rp.prior_score

    def update_directional_observation(
        self,
        robot_position: np.ndarray,
        camera_yaw: float,
        image_features: np.ndarray,
    ) -> None:
        """Update directional observation."""
        if image_features.size == 0:
            return
        angle_bin = round(camera_yaw / (2 * math.pi) * 8) % 8
        feat = np.asarray(image_features, dtype=np.float64)
        feat_norm = np.linalg.norm(feat)
        if feat_norm > 0:
            feat = feat / feat_norm
        self._directional_features[angle_bin] = feat

    def _compute_vision_score(self, frontier_angle: float, instruction: str) -> float:
        """Compute vision score."""
        if not self._clip_encoder or not self._directional_features:
            return 0.0
        target_bin = round(frontier_angle / (2 * math.pi) * 8) % 8
        for offset in [0, 1, -1]:
            candidate_bin = (target_bin + offset) % 8
            if candidate_bin in self._directional_features:
                img_feat = self._directional_features[candidate_bin]
                try:
                    sim = self._clip_encoder.text_image_similarity(instruction, [img_feat])
                    if sim and len(sim) > 0:
                        return max(0.0, min(1.0, float(sim[0])))
                except (ValueError, TypeError, AttributeError) as e:
                    logger.debug("CLIP frontier vision scoring failed: %s", e)
        return 0.0

    def score_frontiers(
        self,
        instruction: str,
        robot_position: np.ndarray,
        visited_positions: list[np.ndarray] | None = None,
        scene_objects: list[dict] | None = None,
        scene_relations: list[dict] | None = None,
        scene_rooms: list[dict] | None = None,
    ) -> list[Frontier]:
        """Score frontiers."""
        if not self._frontiers:
            return []

        self._frontiers = [f for f in self._frontiers if np.isfinite(f.center_world).all() and np.isfinite(f.distance)]
        if not self._frontiers:
            return []

        max_dist = max(max(f.distance for f in self._frontiers), 1.0)
        inst_lower = instruction.lower()

        if inst_lower not in self._bilingual_kw_cache:
            if len(self._bilingual_kw_cache) >= 64:
                self._bilingual_kw_cache.clear()
            self._bilingual_kw_cache[inst_lower] = extract_bilingual_keywords(inst_lower)
        inst_keywords = self._bilingual_kw_cache[inst_lower]

        _obj_positions = None
        _obj_labels_lower: list[str] = []
        obj_directions: list[tuple[float, str]] = []
        if scene_objects:
            _obj_positions = np.array([[obj["position"]["x"], obj["position"]["y"]] for obj in scene_objects])
            _obj_labels_lower = [obj["label"].lower() for obj in scene_objects]
            deltas = _obj_positions - robot_position[:2]
            dists = np.linalg.norm(deltas, axis=1)
            for i, (d, dist) in enumerate(zip(deltas, dists)):
                if dist > 0.5:
                    obj_directions.append((math.atan2(d[1], d[0]), _obj_labels_lower[i]))

        _visited_arr = np.array([vp[:2] for vp in visited_positions]) if visited_positions else None

        for frontier in self._frontiers:
            frontier.nearby_labels = []

            dist_score = 1.0 - (frontier.distance / max_dist)

            novelty_score = 1.0
            if _visited_arr is not None:
                min_v = float(np.min(np.linalg.norm(_visited_arr - frontier.center_world[:2], axis=1)))
                novelty_score = min(1.0, min_v / self.novelty_distance)

            language_score = 0.0
            nearby_labels: list[str] = []
            if _obj_positions is not None:
                dists_to_f = np.linalg.norm(_obj_positions - frontier.center_world[:2], axis=1)
                for idx in np.where(dists_to_f < self.nearby_object_radius)[0]:
                    lbl = scene_objects[idx]["label"]
                    nearby_labels.append(lbl)
                    lbl_lower = _obj_labels_lower[idx]
                    if any(kw in lbl_lower or lbl_lower in kw for kw in inst_keywords):
                        language_score += 0.5
                    language_score += cooccurrence_score(inst_keywords, lbl_lower)
                frontier.nearby_labels = nearby_labels
                language_score = min(1.0, language_score)

            frontier_angle = math.atan2(
                frontier.center_world[1] - robot_position[1],
                frontier.center_world[0] - robot_position[0],
            )

            grounding_score = 0.0
            for obj_angle, obj_label in obj_directions:
                if abs(angle_diff(frontier_angle, obj_angle)) < self.grounding_angle_threshold:
                    grounding_score += self.grounding_spatial_bonus
                    if any(kw in obj_label or obj_label in kw for kw in inst_keywords):
                        grounding_score += self.grounding_keyword_bonus
            if scene_relations and scene_objects:
                for rel in scene_relations:
                    for nearby_lbl in nearby_labels:
                        related_obj = next(
                            (
                                o
                                for o in scene_objects
                                if o.get("id") == rel.get("object_id")
                                and o.get("label", "").lower() == nearby_lbl.lower()
                            ),
                            None,
                        )
                        if related_obj:
                            grounding_score += self.grounding_relation_bonus
            grounding_score = min(1.0, grounding_score)

            vision_score = (
                self._compute_vision_score(frontier_angle, instruction)
                if self.vision_weight > 0.0 and self._clip_encoder
                else 0.0
            )
            semantic_score = (
                compute_semantic_prior_score(
                    frontier,
                    robot_position,
                    scene_rooms,
                    self._room_priors_cache,
                    self._semantic_prior_engine,
                )
                if self.semantic_prior_weight > 0.0 and scene_rooms
                else 0.0
            )
            uncertainty_score = (
                compute_uncertainty_score(
                    frontier,
                    robot_position,
                    scene_rooms,
                    self._room_type_posteriors,
                )
                if self.semantic_uncertainty_weight > 0.0 and scene_rooms
                else 0.0
            )
            kg_room_score = (
                compute_kg_room_score(inst_keywords, nearby_labels, self._room_object_kg)
                if self._room_object_kg and nearby_labels
                else 0.0
            )

            total_w = (
                self.distance_weight
                + self.novelty_weight
                + self.language_weight
                + self.grounding_weight
                + self.vision_weight
                + self.semantic_prior_weight
                + self.semantic_uncertainty_weight
            )
            if total_w > 0:
                frontier.score = (
                    self.distance_weight / total_w * dist_score
                    + self.novelty_weight / total_w * novelty_score
                    + self.language_weight / total_w * language_score
                    + self.grounding_weight / total_w * grounding_score
                    + self.vision_weight / total_w * vision_score
                    + self.semantic_prior_weight / total_w * semantic_score
                    + self.semantic_uncertainty_weight / total_w * uncertainty_score
                )
            else:
                frontier.score = 0.0

            frontier.score += 0.1 * kg_room_score

            failure_penalty = self._compute_failure_penalty(frontier.center_world)
            if failure_penalty > 0:
                frontier.score *= 1.0 - failure_penalty

            # Sticky frontier: penalize re-selection of recently visited areas
            sticky_penalty = self._compute_sticky_penalty(frontier.center_world)
            if sticky_penalty > 0:
                frontier.score *= 1.0 - sticky_penalty

        self._frontiers.sort(key=lambda f: f.score, reverse=True)

        if self.tsp_reorder and len(self._frontiers) >= 2:
            self._frontiers = tsp_sort_frontiers(
                self._frontiers,
                robot_position,
                self._grid,
                self.tsp_frontier_limit,
                self.tsp_ig_radius_cells,
            )

        for frontier in self._frontiers:
            frontier.description = generate_frontier_description(
                frontier,
                self._semantic_prior_engine,
                self._frontier_desc_cache,
            )

        return self._frontiers

    def _compute_uncertainty_score(
        self,
        frontier: Frontier,
        robot_position: np.ndarray,
        scene_rooms: list[dict],
    ) -> float:
        """Compute uncertainty score."""
        return compute_uncertainty_score(
            frontier,
            robot_position,
            scene_rooms,
            self._room_type_posteriors,
        )

    def _compute_kg_room_score(
        self,
        inst_keywords: set[str],
        nearby_labels: list[str],
    ) -> float:
        """Compute kg room score."""
        return compute_kg_room_score(inst_keywords, nearby_labels, self._room_object_kg)

    def _compute_semantic_prior_score(
        self,
        frontier: Frontier,
        robot_position: np.ndarray,
        scene_rooms: list[dict],
    ) -> float:
        """Compute semantic prior score."""
        return compute_semantic_prior_score(
            frontier,
            robot_position,
            scene_rooms,
            self._room_priors_cache,
            self._semantic_prior_engine,
        )

    def get_best_frontier(self) -> Frontier | None:
        """Get best frontier."""
        return self._frontiers[0] if self._frontiers else None

    def get_frontiers_summary(self) -> str:
        """Get frontiers summary."""
        return safe_json_dumps(
            {
                "frontier_count": len(self._frontiers),
                "frontiers": [f.to_dict() for f in self._frontiers[:5]],
            }
        )
