"""VL-Nav v7, equations 3--8 and section III-C, as a selection kernel.

Candidates are proposals, never motion authorization. The caller supplies
prompt-conditioned detector evidence and map-derived unknown ratios. Store each
candidate's VL score at observation time; update distance and unknown ratio when
selecting. Native 3D navigation still owns path feasibility and goal admission.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Literal

from .types import FREE_CELL, UNKNOWN_CELL, Frontier, angle_diff


@dataclass(frozen=True)
class Cue:
    """Prompt-conditioned image evidence, in camera-relative radians."""

    bearing: float
    confidence: float


def vl_score(offset: float, cues: list[Cue], hfov: float, sigma: float = 0.1) -> float:
    """Gaussian mixture and cosine-squared FoV confidence (equations 3--4)."""
    if not (math.isfinite(hfov) and 0 < hfov <= 2 * math.pi):
        raise ValueError("hfov must be in (0, 2*pi]")
    if not (math.isfinite(sigma) and sigma > 0):
        raise ValueError("sigma must be positive")
    if not math.isfinite(offset):
        return 0.0
    offset = angle_diff(offset, 0.0)
    if abs(offset) >= hfov / 2:
        return 0.0
    mixture = sum(
        cue.confidence * math.exp(-0.5 * (angle_diff(offset, cue.bearing) / sigma) ** 2)
        for cue in cues
        if math.isfinite(cue.bearing) and 0 <= cue.confidence <= 1
        and abs(angle_diff(cue.bearing, 0.0)) <= hfov / 2
    )
    return min(1.0, mixture * math.cos(offset * math.pi / hfov) ** 2)


def unknown_ratio(grid, center: tuple[int, int], radius_cells: int) -> float:
    """Count unknown / visited cells by bounded four-neighbor BFS (equation 6).

    Only free and unknown cells participate; obstacles stop expansion. The
    radius is a caller-selected Manhattan radius, not a published paper value.
    This projected-grid statistic may rank proposals but cannot authorize motion.
    """
    if radius_cells < 0:
        raise ValueError("radius_cells must be nonnegative")
    rows, cols = grid.shape
    row, col = center
    if not (0 <= row < rows and 0 <= col < cols) or grid[row, col] not in (FREE_CELL, UNKNOWN_CELL):
        return 0.0
    queue = deque([(row, col, 0)])
    seen = {center}
    unknown = 0
    while queue:
        row, col, depth = queue.popleft()
        unknown += int(grid[row, col] == UNKNOWN_CELL)
        if depth == radius_cells:
            continue
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            neighbor = row + dr, col + dc
            nr, nc = neighbor
            if (neighbor not in seen and 0 <= nr < rows and 0 <= nc < cols
                    and grid[nr, nc] in (FREE_CELL, UNKNOWN_CELL)):
                seen.add(neighbor)
                queue.append((nr, nc, depth + 1))
    return unknown / len(seen)


@dataclass(frozen=True)
class Candidate:
    """One geometry-bound proposal with its observation-time semantic score.

    Instance confidence is detector evidence for the current prompt. An instance
    position identifies the object, not a collision-free robot approach pose.
    """

    id: str
    kind: Literal["instance", "frontier"]
    position: tuple[float, float, float]
    vl: float
    confidence: float = 0.0
    unknown: float = 0.0


@dataclass(frozen=True)
class RankedCandidate:
    candidate: Candidate
    score: float
    distance: float
    distance_score: float
    unknown_score: float


def observe_frontiers(
    frontiers: list[Frontier],
    grid,
    robot_position: tuple[float, float, float],
    camera_yaw: float,
    cues: list[Cue],
    *,
    hfov: float,
    sensor_range: float,
    radius_cells: int,
    reference_z: float,
) -> list[Candidate]:
    """Adapt existing frontier clusters into current-FoV proposals (equation 1).

    reference_z must come from geometry for the active height layer. The helper
    stores initial VL evidence; it never reinterprets old evidence at a new yaw.
    Frontiers remain hypotheses, including centroids that need native validation.
    """
    candidates = []
    for frontier in frontiers:
        x, y = map(float, frontier.center_world[:2])
        dx, dy = x - robot_position[0], y - robot_position[1]
        offset = angle_diff(math.atan2(dy, dx), camera_yaw)
        if math.hypot(dx, dy) > sensor_range or abs(offset) > hfov / 2:
            continue
        center = tuple(int(round(v)) for v in frontier.center[:2])
        candidates.append(Candidate(
            id=f"frontier:{frontier.frontier_id}", kind="frontier", position=(x, y, reference_z),
            vl=vl_score(offset, cues, hfov), unknown=unknown_ratio(grid, center, radius_cells),
        ))
    return candidates


def rank_candidates(
    candidates: list[Candidate],
    robot_position: tuple[float, float, float],
    *,
    detection_threshold: float,
    reached_distance: float,
    distance_weight: float,
    vl_weight: float,
    unknown_scale: float,
    excluded_ids: frozenset[str] = frozenset(),
) -> list[RankedCandidate]:
    """Rank instances first, then NeSy frontiers; empty means hold (III-C).

    Parameters without published numerical values are mandatory. Horizontal
    distance follows the paper; the caller must restrict proposals to the active
    floor/height layer. Preserve z for subsequent native 3D path checks.
    Exclusions belong to the current search (e.g. rejected visual matches).
    """
    parameters = (reached_distance, distance_weight, vl_weight, unknown_scale)
    if not all(math.isfinite(v) and v >= 0 for v in parameters):
        raise ValueError("distance, weights and unknown scale must be finite and nonnegative")
    if not 0 <= detection_threshold <= 1:
        raise ValueError("detection_threshold must be in [0, 1]")
    if not all(math.isfinite(v) for v in robot_position):
        raise ValueError("robot_position must be finite")
    ranked = []
    for candidate in candidates:
        if (candidate.id in excluded_ids or candidate.kind not in {"instance", "frontier"}
                or not all(math.isfinite(v) for v in candidate.position) or not 0 <= candidate.vl <= 1):
            continue
        distance = math.hypot(candidate.position[0] - robot_position[0],
                              candidate.position[1] - robot_position[1])
        if distance <= reached_distance:
            continue
        if candidate.kind == "instance":
            if not detection_threshold < candidate.confidence <= 1:
                continue
            score, dist_score, unk_score = candidate.vl, 0.0, 0.0
        else:
            if not 0 <= candidate.unknown <= 1:
                continue
            dist_score = 1 / (1 + distance)
            unk_score = -math.expm1(-unknown_scale * candidate.unknown)
            score = distance_weight * dist_score + vl_weight * candidate.vl * unk_score
        ranked.append(RankedCandidate(candidate, score, distance, dist_score, unk_score))
    # Retain frontiers after instances so a native-rejected instance can fall back.
    return sorted(ranked, key=lambda r: (r.candidate.kind != "instance", -r.score, r.candidate.id))
