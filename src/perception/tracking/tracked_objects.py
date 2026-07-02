"""
tracked_objects.py — TrackedObject + re-exports from runtime.msgs.scene

Data types (Region, RoomNode, FloorNode, BeliefMessage, etc.) and constants
live in runtime.msgs.scene for cross-module sharing. This file keeps TrackedObject
(which has algorithm logic) and re-exports everything for backward compatibility.
"""

import math
import time
from dataclasses import dataclass, field

import numpy as np

# Re-export all shared data types and constants from runtime.msgs.scene
from runtime.msgs.scene import (  # noqa: F401
    BELIEF_FRESHNESS_TAU,
    BELIEF_NEG_EVIDENCE_WEIGHT,
    BELIEF_SIGMA_BASE,
    BELIEF_SIGMA_DEPTH_COEFF,
    REGION_CLUSTER_RADIUS,
    RELATION_NEAR_THRESHOLD,
    SAFETY_PRIOR_ALPHA_SCALE,
    SAFETY_THRESHOLDS_INTERACTION,
    SAFETY_THRESHOLDS_NAVIGATION,
    BeliefMessage,
    FloorNode,
    PhantomNode,
    Region,
    RoomNode,
    RoomTypePosterior,
    ViewNode,
    infer_room_type,
)


@dataclass
class TrackedObject:
    """Tracked object instance (BA-HSG + USS-Nav point cloud fusion)."""
    object_id: int
    label: str
    position: np.ndarray
    best_score: float
    detection_count: int = 1
    last_seen: float = 0.0
    features: np.ndarray = field(default_factory=lambda: np.array([]))
    region_id: int = -1
    extent: np.ndarray = field(default_factory=lambda: np.array([0.2, 0.2, 0.2]))
    points: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))

    # BA-HSG belief state
    belief_alpha: float = 1.5
    belief_beta: float = 1.0
    position_variance: float = 1.0
    miss_streak: int = 0
    credibility: float = 0.5

    # KG knowledge enhancement
    kg_concept_id: str = ""
    safety_level: str = "safe"
    affordances: list[str] = field(default_factory=list)
    functional_properties: dict = field(default_factory=dict)
    floor_level: int = 0

    # KG-Augmented Prior
    kg_prior_alpha: float = 0.0
    kg_prior_source: str = ""
    is_kg_expected: bool = False
    safety_nav_threshold: float = 0.25
    safety_interact_threshold: float = 0.40

    # Source tags
    source: str = "observed"
    last_observed_time: float = 0.0
    is_simulated: bool = False

    def update(self, det, alpha: float = 0.3) -> None:
        """Update with new detection (USS-Nav + BA-HSG)."""
        if hasattr(det, 'points') and det.points is not None and len(det.points) > 0:
            self._fuse_pointcloud(det.points)
            self.position = np.mean(self.points, axis=0) if len(self.points) > 0 else det.position.copy()
            self.position_variance = max(0.01, self.position_variance * 0.8)
        else:
            depth = float(np.linalg.norm(det.position[:2]))
            obs_var = (BELIEF_SIGMA_BASE + BELIEF_SIGMA_DEPTH_COEFF * depth) ** 2
            new_var = 1.0 / (1.0 / max(self.position_variance, 1e-6) + 1.0 / max(obs_var, 1e-6))
            self.position = new_var * (
                self.position / max(self.position_variance, 1e-6)
                + det.position / max(obs_var, 1e-6)
            )
            self.position_variance = new_var

        self.best_score = max(self.best_score, det.score)
        self.detection_count += 1
        self.last_seen = time.time()
        self.miss_streak = 0

        safety_scale = SAFETY_PRIOR_ALPHA_SCALE.get(self.safety_level, 1.0)
        self.belief_alpha += 1.0 * safety_scale

        if det.features.size > 0:
            self.features = self._fuse_feature(det.features)
        self._update_extent(det)
        self._update_credibility()

    def _fuse_pointcloud(self, new_points: np.ndarray, max_total: int = 1024) -> None:
        """USS-Nav: incremental point cloud fusion."""
        if new_points is None or len(new_points) == 0:
            return
        if self.points is None or len(self.points) == 0:
            self.points = new_points.copy()
        else:
            self.points = np.vstack([self.points, new_points])

        from .projection import POINTCLOUD_VOXEL_SIZE, _voxel_downsample
        self.points = _voxel_downsample(self.points, POINTCLOUD_VOXEL_SIZE, max_total)

    def _fuse_feature(self, new_feature: np.ndarray) -> np.ndarray:
        """Multi-view CLIP feature fusion (ConceptGraphs EMA + L2 normalize)."""
        nf = np.asarray(new_feature, dtype=np.float64)
        if nf.size == 0:
            return self.features

        nf_norm = np.linalg.norm(nf)
        if nf_norm > 0:
            nf = nf / nf_norm

        if self.features.size == 0:
            return nf

        of = np.asarray(self.features, dtype=np.float64)
        of_norm = np.linalg.norm(of)
        if of_norm > 0:
            of = of / of_norm

        base_alpha = 0.3
        quality_factor = max(0.5, min(1.5, self.best_score / 0.8))
        alpha = min(0.5, base_alpha * quality_factor)

        fused = (1.0 - alpha) * of + alpha * nf
        fused_norm = np.linalg.norm(fused)
        if fused_norm > 0:
            fused = fused / fused_norm
        return fused

    def _update_extent(self, det) -> None:
        """Estimate 3D bounding box from 2D bbox + depth."""
        if det.depth <= 0 or det.bbox_2d.size < 4:
            return
        x1, y1, x2, y2 = det.bbox_2d
        bbox_w = max(float(x2 - x1), 1.0)
        bbox_h = max(float(y2 - y1), 1.0)
        fx_approx = getattr(det, '_intrinsics_fx', 0.0)
        if fx_approx <= 0:
            fx_approx = 600.0
        extent_x = (bbox_w / fx_approx) * det.depth * 0.5
        extent_y = extent_x
        extent_z = (bbox_h / fx_approx) * det.depth * 0.5
        new_ext = np.array([max(extent_x, 0.05), max(extent_y, 0.05), max(extent_z, 0.05)])
        self.extent = 0.3 * new_ext + 0.7 * self.extent

    # -- BA-HSG belief methods --

    @property
    def existence_prob(self) -> float:
        return self.belief_alpha / (self.belief_alpha + self.belief_beta)

    @property
    def existence_uncertainty(self) -> float:
        a, b = self.belief_alpha, self.belief_beta
        return (a * b) / ((a + b) ** 2 * (a + b + 1))

    def record_miss(self) -> None:
        self.miss_streak += 1
        self.belief_beta += BELIEF_NEG_EVIDENCE_WEIGHT
        self._update_credibility()

    def _update_credibility(self) -> None:
        dt = time.time() - self.last_seen if self.last_seen > 0 else 999.0
        view_diversity = 1.0 - 1.0 / max(self.detection_count, 1)
        freshness = math.exp(-dt / BELIEF_FRESHNESS_TAU)
        kg_factor = min(self.kg_prior_alpha / 3.0, 0.3) if self.kg_prior_alpha > 0 else 0.0

        self.credibility = (
            0.35 * self.existence_prob
            + 0.15 * view_diversity
            + 0.15 * freshness
            + 0.15 * min(self.best_score, 1.0)
            + 0.20 * kg_factor
        )
        self.safety_nav_threshold = SAFETY_THRESHOLDS_NAVIGATION.get(self.safety_level, 0.25)
        self.safety_interact_threshold = SAFETY_THRESHOLDS_INTERACTION.get(self.safety_level, 0.40)

    @property
    def is_confirmed_for_navigation(self) -> bool:
        return self.credibility >= self.safety_nav_threshold

    @property
    def is_confirmed_for_interaction(self) -> bool:
        return self.credibility >= self.safety_interact_threshold

    def to_belief_dict(self) -> dict:
        d = {
            "P_exist": round(self.existence_prob, 3),
            "sigma_pos": round(math.sqrt(self.position_variance), 3),
            "credibility": round(self.credibility, 3),
            "detections": self.detection_count,
            "miss_streak": self.miss_streak,
            "confirmed_nav": self.is_confirmed_for_navigation,
            "confirmed_interact": self.is_confirmed_for_interaction,
        }
        if self.kg_concept_id:
            d["kg_concept"] = self.kg_concept_id
            d["safety"] = self.safety_level
            d["affordances"] = self.affordances
        if self.kg_prior_alpha > 0:
            d["kg_prior_alpha"] = round(self.kg_prior_alpha, 3)
            d["kg_prior_source"] = self.kg_prior_source
            d["is_kg_expected"] = self.is_kg_expected
        if self.floor_level != 0:
            d["floor"] = self.floor_level
        return d
